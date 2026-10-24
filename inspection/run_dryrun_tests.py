"""dry-run 场景测试：纯离线，不依赖 ROS / 底盘 / 相机 / 语音。

运行（在 src 目录下）：
    python3 -m inspection.run_dryrun_tests
"""

import importlib.util
import io
import json
import os
import sys
import tempfile
import time
import types as _pytypes

from inspection.controller import InspectionConfig, InspectionController, State
from inspection.recorder import InspectionRecorder
from inspection.inspector import SimInspector, YoloInspector
from inspection.navigator import ChassisNavigator, SimNavigator
from inspection.speaker import SimSpeaker
from inspection.types import (
    InspectionStatus,
    NavOutcome,
    NavResult,
    SpeechState,
)

PASS, FAIL = "PASS", "FAIL"
_results = []


def _run(name, nav, spk, vision, cfg=None, cancel_after=None):
    cfg = cfg or InspectionConfig(dwell_seconds=0.02)
    record_path = os.path.join(tempfile.mkdtemp(prefix="insp_test_"),
                               f"{name}.jsonl")
    rec = InspectionRecorder(record_path, meta={
        "mode": "test", "scenario": name, "simulated": True})
    ctl = InspectionController(nav, spk, vision, rec, cfg)

    thread = None
    if cancel_after is not None:
        import threading, time as _t
        thread = threading.Thread(
            target=lambda: (_t.sleep(cancel_after), ctl.request_cancel()),
            daemon=True)
        thread.start()
    summary = ctl.run()
    if thread:
        thread.join(timeout=2.0)

    with open(record_path, encoding="utf-8") as fp:
        records = [json.loads(line) for line in fp if line.strip()]
    return summary, records, nav, spk, vision


def _check(name, cond, detail=""):
    _results.append((name, PASS if cond else FAIL, detail))
    print(f"  [{PASS if cond else FAIL}] {name}  {detail}")


def test_success():
    nav, spk, vision = SimNavigator(), SimSpeaker(), SimInspector()
    summary, records, nav, spk, vision = _run("success", nav, spk, vision)
    _check("success.final", summary.final_state is State.DONE)
    _check("success.order",
           nav.calls == ["start_point",
                         "POI_01", "POI_01.align", "POI_02", "POI_02.align",
                         "POI_03", "POI_03.align", "restore_speed", "stop"],
           str(nav.calls))
    _check("success.poi_records",
           sum(1 for r in records if r.get("record_type") == "poi") == 3)
    _check("success.all_simulated",
           all(r.get("inspection", {}).get("simulated", True)
               for r in records if r.get("record_type") == "poi"))


def test_nav_fail():
    nav = SimNavigator(fail_at="POI_02", outcome="failed")
    summary, records, nav, *_ = _run("nav_fail", nav, SimSpeaker(), SimInspector())
    _check("nav_fail.state", summary.final_state is State.ABORTED)
    _check("nav_fail.stop_after_poi2",
           "POI_03" not in nav.calls, str(nav.calls))
    _check("nav_fail.poi1_done",
           summary.poi_finals.get("POI_01") == "done")
    _check("nav_fail.poi2_failed",
           summary.poi_finals.get("POI_02") == "nav_failed")


def test_nav_timeout():
    nav = SimNavigator(fail_at="POI_01", outcome="timeout")
    summary, records, nav, *_ = _run("nav_timeout", nav, SimSpeaker(),
                                     SimInspector())
    _check("nav_timeout.state", summary.final_state is State.ABORTED)
    poi_rec = [r for r in records if r.get("record_type") == "poi"][0]
    _check("nav_timeout.outcome",
           poi_rec["nav"]["outcome"] == NavOutcome.TIMEOUT.value)


def test_vision_modes():
    for mode, expect in (("abnormal", InspectionStatus.ABNORMAL),
                         ("uncertain", InspectionStatus.UNCERTAIN),
                         ("timeout", InspectionStatus.TIMEOUT),
                         ("no_frame", InspectionStatus.NO_FRAME)):
        summary, records, *_ = _run(f"vision_{mode}", SimNavigator(),
                                    SimSpeaker(), SimInspector(mode=mode))
        poi_rec = [r for r in records if r.get("record_type") == "poi"][0]
        insp = poi_rec["inspection"]
        _check(f"vision_{mode}.status", insp["status"] == expect.value,
               insp["status"])
        _check(f"vision_{mode}.simulated", insp["simulated"] is True)
        if mode != "abnormal":
            _check(f"vision_{mode}.not_normal",
                   insp["status"] != InspectionStatus.SUCCESS.value)


def test_speech_fail_not_blocking():
    summary, records, _, spk, _ = _run(
        "speech_fail", SimNavigator(),
        SimSpeaker(mode="submit_fail"), SimInspector())
    _check("speech_fail.still_done", summary.final_state is State.DONE)
    states = set()
    for r in records:
        if r.get("record_type") == "poi":
            states.update(v.get("state") for v in r.get("speech", {}).values())
    _check("speech_fail.marked",
           SpeechState.SUBMIT_FAILED.value in states, str(states))


def test_user_cancel():
    nav, spk, vision = SimNavigator(delay_s=0.05), SimSpeaker(), SimInspector()
    cfg = InspectionConfig(dwell_seconds=5.0)  # 长停留，便于取消命中
    summary, records, *_ = _run("user_cancel", nav, spk, vision, cfg=cfg,
                                cancel_after=0.2)
    _check("user_cancel.state", summary.final_state is State.ABORTED)
    _check("user_cancel.detail", "取消" in summary.detail, summary.detail)
    _check("user_cancel.cleanup",
           nav.calls[-2:] == ["restore_speed", "stop"], str(nav.calls))


def test_return_start():
    cfg = InspectionConfig(dwell_seconds=0.02, return_to_start=True)
    nav = SimNavigator()
    summary, records, nav, *_ = _run("return_start", nav, SimSpeaker(),
                                     SimInspector(), cfg=cfg)
    _check("return_start.state", summary.final_state is State.DONE)
    _check("return_start.calls",
           nav.calls[0] == "start_point" and nav.calls.count("start_point") == 2,
           str(nav.calls))


# ── 实机门控与限速回滚：纯 Mock 测试（不连底盘 / 不发声 / 不需 ROS） ──

def _load_run_inspection():
    """按文件路径加载 run_inspection 模块（避免包导入触发其他底盘模块）。"""
    src = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    path = os.path.join(src, "chassis", "run_inspection.py")
    spec = importlib.util.spec_from_file_location("run_inspection_mod", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class _FakeArgs:
    max_speed = 0.4
    model = ""
    expect = []
    abnormal = []
    record = "/tmp/insp_mock_never_written.jsonl"
    return_start = False


def _eof_input(_prompt):
    raise EOFError("simulated non-interactive stdin")


def _save_env(name):
    return os.environ.pop(name, None)


def _restore_env(name, old):
    if old is None:
        os.environ.pop(name, None)
    else:
        os.environ[name] = old


def _patch_robot_module(factory):
    """把 robot_controller 包替换为假模块，返回 (constructed_list, 还原函数)。"""
    fake = _pytypes.ModuleType("robot_controller")
    constructed = []

    def _mk():
        constructed.append(1)
        return factory()

    fake.robot_controller = _mk
    old_mod = sys.modules.get("robot_controller")
    sys.modules["robot_controller"] = fake

    def _restore():
        if old_mod is None:
            sys.modules.pop("robot_controller", None)
        else:
            sys.modules["robot_controller"] = old_mod

    return constructed, _restore


def test_real_gate_confirm_logic():
    ri = _load_run_inspection()
    env = ri.REAL_CONFIRM_ENV
    cfg = InspectionConfig()
    args = _FakeArgs()
    old = _save_env(env)
    try:
        # 未设置环境变量 + 输入 EOF（非交互）→ 拒绝
        _check("gate.unset_eof_reject",
               ri._confirm_real(cfg, args, input_fn=_eof_input) is False)
        # 环境变量值不严格等于 "1" → 拒绝
        os.environ[env] = "yes"
        _check("gate.wrong_env_reject",
               ri._confirm_real(cfg, args, input_fn=_eof_input) is False)
        os.environ.pop(env, None)
        # 交互输入错误 → 拒绝
        _check("gate.wrong_input_reject",
               ri._confirm_real(cfg, args, input_fn=lambda _p: "no") is False)
        # 交互严格输入 YES → 通过
        _check("gate.input_yes_accept",
               ri._confirm_real(cfg, args, input_fn=lambda _p: "YES") is True)
        # 环境变量严格 =1 → 通过（无需输入，EOF 也不影响）
        os.environ[env] = "1"
        _check("gate.env_accept",
               ri._confirm_real(cfg, args, input_fn=_eof_input) is True)
    finally:
        _restore_env(env, old)


def test_real_unconfirmed_exits_before_hardware():
    ri = _load_run_inspection()
    env = ri.REAL_CONFIRM_ENV
    old_env = _save_env(env)
    old_stdin = sys.stdin
    constructed, restore_mod = _patch_robot_module(
        lambda: (_ for _ in ()).throw(
            RuntimeError("不应到达：门控未通过却尝试构建机器人")))
    try:
        sys.stdin = io.StringIO("")  # 非交互终端（isatty=False）
        rc = ri.run_real(_FakeArgs())
    finally:
        sys.stdin = old_stdin
        restore_mod()
        _restore_env(env, old_env)
    _check("gate.real_unconfirmed_rc2", rc == 2, f"rc={rc}")
    _check("gate.real_unconfirmed_no_robot", not constructed,
           "未确认时不得构建 robot_controller")


def test_real_confirmed_enters_flow():
    ri = _load_run_inspection()
    env = ri.REAL_CONFIRM_ENV
    old_env = os.environ.get(env)
    os.environ[env] = "1"
    constructed, restore_mod = _patch_robot_module(lambda: object())
    try:
        try:
            ri.run_real(_FakeArgs())
        except Exception:
            pass  # 预期：后续真实 TTS/ROS 初始化在离线环境失败
    finally:
        restore_mod()
        _restore_env(env, old_env)
    _check("gate.confirmed_constructs_robot", bool(constructed),
           "确认通过后应进入真实流程（构建机器人）")


def test_sim_mode_bypasses_gate():
    ri = _load_run_inspection()
    env = ri.REAL_CONFIRM_ENV
    tmp = os.path.join(tempfile.mkdtemp(prefix="insp_test_"), "sim.jsonl")
    old_argv = sys.argv
    old_env = _save_env(env)
    old_stdin = sys.stdin
    try:
        sys.stdin = io.StringIO("")  # 即使非交互，模拟模式也不应被门控拦截
        sys.argv = ["run_inspection.py", "--record", tmp,
                    "--scenario", "success"]
        rc = ri.main()
    finally:
        sys.argv = old_argv
        sys.stdin = old_stdin
        _restore_env(env, old_env)
    _check("gate.sim_bypass_rc0", rc == 0, f"rc={rc}")


class _MockSpeedRobot:
    """仅模拟 chassis_get/set_motion_param 的离线假底盘（无 ROS/REST）。"""

    def __init__(self, move_spd=0.8, rot_spd=1.0):
        self.params = {"move_spd": move_spd, "rot_spd": rot_spd}
        self.set_calls = []               # [(move_spd, rot_spd)]
        self.get_error = None             # GET 恒抛（模拟原始参数不可读）
        self.set_error = None             # SET 恒抛
        self.set_fail_times = 0           # 前 N 次 SET 抛异常，之后正常
        self.get_error_after_write = None  # 写入后 GET 抛异常（验证失败）
        self.apply_after_write = True     # False=写入后 GET 仍返回旧值
        self._written = False

    def chassis_get_motion_param(self):
        if self._written and self.get_error_after_write is not None:
            raise self.get_error_after_write
        if self.get_error is not None:
            raise self.get_error
        return dict(self.params)

    def chassis_set_motion_param(self, move_spd=None, rot_spd=None):
        self.set_calls.append((move_spd, rot_spd))
        if self.set_error is not None:
            raise self.set_error
        if self.set_fail_times > 0:
            self.set_fail_times -= 1
            raise RuntimeError("simulated transient set failure")
        self._written = True
        if self.apply_after_write:
            if move_spd is not None:
                self.params["move_spd"] = move_spd
            if rot_spd is not None:
                self.params["rot_spd"] = rot_spd


def test_speed_limit_unreadable_prev():
    # 原始参数读不出 → FAILED，且绝不写入、无备份、恢复为空操作
    robot = _MockSpeedRobot()
    robot.get_error = RuntimeError("rest no response")
    nav = ChassisNavigator(robot, max_speed=0.4, speed_apply_timeout=0.4)
    r = nav.apply_speed_limit()
    _check("limit.unreadable_failed", r.outcome is NavOutcome.FAILED)
    _check("limit.unreadable_no_write", robot.set_calls == [],
           str(robot.set_calls))
    _check("limit.unreadable_no_backup", nav._prev_motion_param is None)
    nav.restore_speed_limit()
    _check("limit.unreadable_restore_noop", robot.set_calls == [])


def test_speed_limit_write_raises_restore_fails():
    # 写入抛异常 → 尝试回滚；回滚也失败 → 如实记录，保留备份供 cleanup 重试
    robot = _MockSpeedRobot(move_spd=0.8, rot_spd=1.0)
    robot.set_error = RuntimeError("rest 500")
    nav = ChassisNavigator(robot, max_speed=0.4, speed_apply_timeout=0.4)
    r = nav.apply_speed_limit()
    _check("limit.write_exc_failed", r.outcome is NavOutcome.FAILED)
    _check("limit.write_exc_restore_attempted",
           len(robot.set_calls) == 2 and robot.set_calls[1] == (0.8, 1.0),
           str(robot.set_calls))
    _check("limit.write_exc_restore_logged",
           "回滚" in r.detail and "失败" in r.detail, r.detail)
    _check("limit.write_exc_backup_kept",
           nav._prev_motion_param is not None,
           "回滚失败须保留备份供 cleanup 重试")


def test_speed_limit_write_raises_restore_ok():
    # 首次 SET（写限速）抛异常、回滚 SET 成功 → 参数恢复、备份清除、幂等
    robot = _MockSpeedRobot(move_spd=0.8, rot_spd=1.0)
    robot.set_fail_times = 1
    nav = ChassisNavigator(robot, max_speed=0.4, speed_apply_timeout=0.4)
    r = nav.apply_speed_limit()
    _check("limit.restore_ok_failed", r.outcome is NavOutcome.FAILED)
    _check("limit.restore_ok_logged", "已回滚" in r.detail, r.detail)
    _check("limit.restore_ok_values",
           robot.params["move_spd"] == 0.8 and robot.params["rot_spd"] == 1.0,
           str(robot.params))
    _check("limit.restore_ok_backup_cleared", nav._prev_motion_param is None)
    before = len(robot.set_calls)
    nav.restore_speed_limit()
    _check("limit.restore_idempotent", len(robot.set_calls) == before)


def test_speed_limit_verify_timeout_rollback():
    # 写入成功但验证永不生效（GET 恒返回旧值）→ 超时 FAILED + 回滚原值
    robot = _MockSpeedRobot(move_spd=0.8, rot_spd=1.0)
    robot.apply_after_write = False
    nav = ChassisNavigator(robot, max_speed=0.4, speed_apply_timeout=0.45)
    r = nav.apply_speed_limit()
    _check("limit.verify_timeout_failed", r.outcome is NavOutcome.FAILED)
    _check("limit.verify_timeout_rolled_back", "已回滚" in r.detail, r.detail)
    _check("limit.verify_timeout_restore_call",
           robot.set_calls[-1] == (0.8, 1.0), str(robot.set_calls))
    _check("limit.verify_timeout_backup_cleared",
           nav._prev_motion_param is None)


def test_speed_limit_verify_raises_rollback():
    # 写入成功后验证读取抛异常 → FAILED + 回滚原值
    robot = _MockSpeedRobot(move_spd=0.8, rot_spd=1.0)
    robot.get_error_after_write = RuntimeError("rest read timeout")
    nav = ChassisNavigator(robot, max_speed=0.4, speed_apply_timeout=2.0)
    r = nav.apply_speed_limit()
    _check("limit.verify_exc_failed", r.outcome is NavOutcome.FAILED)
    _check("limit.verify_exc_rolled_back", "已回滚" in r.detail, r.detail)
    _check("limit.verify_exc_restore_call",
           robot.set_calls[-1] == (0.8, 1.0), str(robot.set_calls))


def test_speed_limit_success_and_cleanup_restore():
    # 正常路径：限速生效 → cleanup 恢复原值 → 重复恢复为幂等
    robot = _MockSpeedRobot(move_spd=0.8, rot_spd=1.0)
    nav = ChassisNavigator(robot, max_speed=0.4, speed_apply_timeout=1.0)
    r = nav.apply_speed_limit()
    _check("limit.success", r.outcome is NavOutcome.SUCCESS, r.detail)
    _check("limit.success_applied",
           abs(robot.params["move_spd"] - 0.4) < 1e-9, str(robot.params))
    nav.restore_speed_limit()
    _check("limit.cleanup_restored",
           robot.params["move_spd"] == 0.8 and robot.params["rot_spd"] == 1.0,
           str(robot.params))
    before = len(robot.set_calls)
    nav.restore_speed_limit()
    _check("limit.cleanup_idempotent", len(robot.set_calls) == before)


def test_speed_limit_failure_blocks_navigation():
    # 限速失败 → 控制器 ABORTED，且从未下发任何导航段
    class _FailLimitNav(SimNavigator):
        def apply_speed_limit(self):
            return NavResult(outcome=NavOutcome.FAILED, label="speed_limit",
                             detail="simulated limit failure")

    nav = _FailLimitNav()
    summary, records, nav, *_ = _run("limit_fail_blocks_nav", nav,
                                     SimSpeaker(), SimInspector())
    _check("limit.block_state", summary.final_state is State.ABORTED)
    moved = [c for c in nav.calls if c not in ("restore_speed", "stop")]
    _check("limit.block_no_motion", moved == [], str(nav.calls))


# ── 中断(Ctrl+C)与终止一致性：纯 Mock 测试 ─────────────────────────────

class _BoomSpeaker:
    """命中关键词的播报抛出指定异常（模拟 TTS 故障 / Ctrl+C）。"""

    def __init__(self, boom_on: str, exc):
        self._boom_on = boom_on
        self._exc = exc
        self.calls = []

    def say(self, text, interrupt=True):
        self.calls.append(text)
        if self._boom_on in text:
            raise self._exc
        return SimSpeaker().say(text, interrupt=interrupt)


class _CancelNav(SimNavigator):
    """首个轨道导航段直接返回 CANCELLED（模拟运动段被取消）。"""

    def nav_track_to_poi(self, poi_name, timeout):
        self.calls.append(poi_name)
        return NavResult(outcome=NavOutcome.CANCELLED, label=poi_name,
                         detail="simulated cancel", simulated=True)


class _KIVision(SimInspector):
    """inspect 时抛 KeyboardInterrupt（模拟视觉阶段的 Ctrl+C）。"""

    def inspect(self, poi_name):
        self.calls.append(poi_name)
        raise KeyboardInterrupt()


class _BoomSummaryRecorder(InspectionRecorder):
    """write_summary 恒失败（模拟磁盘写错误）。"""

    def write_summary(self, final_state, detail):
        raise RuntimeError("simulated disk full")


def _poi_records(records):
    return [r for r in records if r.get("record_type") == "poi"]


def _summary_records(records):
    return [r for r in records if r.get("record_type") == "summary"]


def test_ki_during_dwell():
    # dwell 阶段 Ctrl+C → ABORTED，不继续下一 POI，cleanup 仍执行
    orig = InspectionController._interruptible_sleep

    def _boom(self, seconds):
        raise KeyboardInterrupt()

    InspectionController._interruptible_sleep = _boom
    nav = SimNavigator()
    try:
        summary, records, nav, *_ = _run("ki_dwell", nav, SimSpeaker(),
                                         SimInspector())
    finally:
        InspectionController._interruptible_sleep = orig
    _check("ki.dwell_aborted", summary.final_state is State.ABORTED)
    _check("ki.dwell_detail", "中断" in summary.detail, summary.detail)
    _check("ki.dwell_no_next_poi", "POI_02" not in nav.calls, str(nav.calls))
    _check("ki.dwell_cleanup",
           nav.calls[-2:] == ["restore_speed", "stop"], str(nav.calls))
    summaries = _summary_records(records)
    _check("ki.dwell_summary_written",
           len(summaries) == 1 and summaries[0].get("final_state") == "ABORTED",
           str(summaries))


def test_ki_during_vision():
    # 视觉阶段 Ctrl+C → ABORTED，该点记录为 aborted，cleanup 仍执行
    nav = SimNavigator()
    summary, records, nav, *_ = _run("ki_vision", nav, SimSpeaker(),
                                     _KIVision())
    _check("ki.vision_aborted", summary.final_state is State.ABORTED)
    _check("ki.vision_detail", "中断" in summary.detail, summary.detail)
    _check("ki.vision_no_next_poi", "POI_02" not in nav.calls, str(nav.calls))
    _check("ki.vision_cleanup",
           nav.calls[-2:] == ["restore_speed", "stop"], str(nav.calls))
    pois = _poi_records(records)
    _check("ki.vision_poi_aborted",
           len(pois) == 1 and pois[0].get("final") == "aborted", str(pois))
    summaries = _summary_records(records)
    _check("ki.vision_summary_written",
           len(summaries) == 1 and summaries[0].get("final_state") == "ABORTED",
           str(summaries))


def test_ki_during_tts():
    # TTS 播报阶段 Ctrl+C（arrive 播报时）→ ABORTED，不误判 DONE
    nav = SimNavigator()
    spk = _BoomSpeaker("已到达", KeyboardInterrupt())
    summary, records, nav, *_ = _run("ki_tts", nav, spk, SimInspector())
    _check("ki.tts_aborted", summary.final_state is State.ABORTED)
    _check("ki.tts_detail", "中断" in summary.detail, summary.detail)
    _check("ki.tts_no_next_poi", "POI_02" not in nav.calls, str(nav.calls))
    _check("ki.tts_cleanup",
           nav.calls[-2:] == ["restore_speed", "stop"], str(nav.calls))
    summaries = _summary_records(records)
    _check("ki.tts_summary_written",
           len(summaries) == 1 and summaries[0].get("final_state") == "ABORTED",
           str(summaries))


def test_abort_speech_failure_still_aborts():
    # _abort 内终止播报抛异常 → 状态/落盘不受影响，不向上抛
    nav = _CancelNav()
    spk = _BoomSpeaker("取消", RuntimeError("simulated speaker boom"))
    summary, records, nav, *_ = _run("abort_speech_boom", nav, spk,
                                     SimInspector())
    _check("abort.speech_boom_state", summary.final_state is State.ABORTED)
    _check("abort.speech_boom_detail", summary.detail == "用户取消",
           summary.detail)
    summaries = _summary_records(records)
    _check("abort.speech_boom_summary",
           len(summaries) == 1 and summaries[0].get("final_state") == "ABORTED",
           str(summaries))
    pois = _poi_records(records)
    _check("abort.speech_boom_poi",
           len(pois) == 1 and pois[0].get("final") == "aborted", str(pois))


def test_abort_summary_write_failure():
    # _abort 内 write_summary 抛异常 → final_state/detail 不被覆盖伪造
    nav = _CancelNav()
    tmp = os.path.join(tempfile.mkdtemp(prefix="insp_test_"), "boom.jsonl")
    rec = _BoomSummaryRecorder(tmp, meta={"mode": "test", "simulated": True})
    ctl = InspectionController(nav, SimSpeaker(), SimInspector(), rec,
                               InspectionConfig(dwell_seconds=0.02))
    summary = ctl.run()  # 不应抛异常
    _check("abort.summary_boom_state", summary.final_state is State.ABORTED)
    _check("abort.summary_boom_detail", summary.detail == "用户取消",
           summary.detail)
    _check("abort.summary_boom_cleanup",
           nav.calls[-2:] == ["restore_speed", "stop"], str(nav.calls))


def test_done_path_speech_failure_stays_done():
    # DONE 路径的完成播报抛异常 → 不得退化为 ABORTED
    nav = SimNavigator()
    spk = _BoomSpeaker("全部点位", RuntimeError("simulated speaker boom"))
    summary, records, nav, *_ = _run("done_speech_boom", nav, spk,
                                     SimInspector())
    _check("done.boom_state", summary.final_state is State.DONE)
    summaries = _summary_records(records)
    _check("done.boom_summary",
           len(summaries) == 1 and summaries[0].get("final_state") == "DONE",
           str(summaries))
    _check("done.boom_pois_all_done",
           all(r.get("final") == "done" for r in _poi_records(records)),
           str(_poi_records(records)))


# ── YOLO 推理超时：纯 Mock 测试（不开相机 / 不加载模型 / 不跑 GPU） ─────

class _FakeCameraRobot:
    """模拟 camera_get_rgb_depth 的离线假机器人。"""

    def __init__(self, frame=None, exc=None):
        self._frame = frame
        self._exc = exc

    def camera_get_rgb_depth(self, timeout=None, require_new=False,
                             camera_name=None):
        if self._exc is not None:
            raise self._exc
        return self._frame


class _FakeYolo:
    """模拟 YoloLoad.detect 的离线假模型。"""

    def __init__(self, fn):
        self._fn = fn
        self.calls = 0

    def detect(self, rgb):
        self.calls += 1
        return self._fn(rgb)


class _MockYoloInspector(YoloInspector):
    """绕过真实 __init__（不加载模型/不连相机），复用真实 inspect 逻辑。"""

    def __init__(self, frame=None, frame_exc=None, detect_fn=None,
                 inference_timeout=0.2, frame_timeout=0.2,
                 expect=("fire_extinguisher",), abnormal=("smoking",)):
        self._robot = _FakeCameraRobot(frame=frame, exc=frame_exc)
        self._yolo = _FakeYolo(detect_fn or (lambda rgb: {
            "detections": [], "inference_ms": 1.0}))
        self._expect = tuple(expect)
        self._abnormal = tuple(abnormal)
        self._camera_name = "head"
        self._frame_timeout = frame_timeout
        self._inference_timeout = inference_timeout
        self._min_conf = 0.5
        self._poisoned = False


_FAKE_FRAME = {"rgb": object()}  # 仅作为非 None 占位，不会被真实使用


def test_vis_camera_timeout():
    ins = _MockYoloInspector(frame_exc=TimeoutError(
        "timeout waiting for ROS RGB-D frame"))
    r = ins.inspect("POI_01")
    _check("vis.cam_timeout_status",
           r.status is InspectionStatus.NO_FRAME, r.status.value)
    _check("vis.camera_timeout_detail", "取图超时" in r.detail, r.detail)


def test_vis_camera_error():
    ins = _MockYoloInspector(frame_exc=RuntimeError("camera crashed"))
    r = ins.inspect("POI_01")
    _check("vis.camera_error_status",
           r.status is InspectionStatus.NO_FRAME, r.status.value)
    _check("vis.camera_error_detail",
           "取图异常" in r.detail and "超时" not in r.detail, r.detail)


def test_vis_no_frame():
    ins = _MockYoloInspector(frame={"rgb": None})
    r = ins.inspect("POI_01")
    _check("vis.no_frame_status",
           r.status is InspectionStatus.NO_FRAME, r.status.value)


def test_vis_inference_timeout_and_poison():
    # 推理 0.6s > 期限 0.15s → TIMEOUT + poisoned；后续调用不再触碰模型
    ins = _MockYoloInspector(
        frame=_FAKE_FRAME,
        detect_fn=lambda rgb: (time.sleep(0.6), {
            "detections": [], "inference_ms": 600.0})[1],
        inference_timeout=0.15)
    r1 = ins.inspect("POI_01")
    _check("vis.infer_timeout_status",
           r1.status is InspectionStatus.TIMEOUT, r1.status.value)
    _check("vis.infer_timeout_not_success",
           r1.status not in (InspectionStatus.SUCCESS,), r1.status.value)
    _check("vis.infer_timeout_detail",
           "推理超过" in r1.detail and "软超时" in r1.detail, r1.detail)
    _check("vis.infer_timeout_poisoned", ins._poisoned is True)
    r2 = ins.inspect("POI_02")
    _check("vis.poisoned_error",
           r2.status is InspectionStatus.ERROR, r2.status.value)
    _check("vis.poisoned_no_reuse", ins._yolo.calls == 1,
           f"detect 调用次数={ins._yolo.calls}")


def test_vis_inference_error():
    ins = _MockYoloInspector(
        frame=_FAKE_FRAME,
        detect_fn=lambda rgb: (_ for _ in ()).throw(
            RuntimeError("cuda error")))
    r = ins.inspect("POI_01")
    _check("vis.infer_error_status",
           r.status is InspectionStatus.ERROR, r.status.value)
    _check("vis.infer_error_detail", "推理异常" in r.detail, r.detail)
    _check("vis.infer_error_not_poisoned", ins._poisoned is False)


def test_vis_judgement_logic():
    # 真实判定路径：abnormal / success / uncertain / 低置信度过滤
    def det(name, conf):
        return {"class_name": name, "confidence": conf,
                "xyxy": [0, 0, 5, 5]}

    ins_abn = _MockYoloInspector(frame=_FAKE_FRAME, detect_fn=lambda rgb: {
        "detections": [det("smoking", 0.9)], "inference_ms": 3.0})
    _check("vis.judge_abnormal",
           ins_abn.inspect("P").status is InspectionStatus.ABNORMAL)

    ins_ok = _MockYoloInspector(frame=_FAKE_FRAME, detect_fn=lambda rgb: {
        "detections": [det("fire_extinguisher", 0.9)], "inference_ms": 3.0})
    _check("vis.judge_success",
           ins_ok.inspect("P").status is InspectionStatus.SUCCESS)

    ins_unc = _MockYoloInspector(frame=_FAKE_FRAME, detect_fn=lambda rgb: {
        "detections": [det("fire_extinguisher", 0.3)], "inference_ms": 3.0})
    r_unc = ins_unc.inspect("P")
    _check("vis.judge_uncertain",
           r_unc.status is InspectionStatus.UNCERTAIN, r_unc.status.value)

    ins_low_abn = _MockYoloInspector(frame=_FAKE_FRAME, detect_fn=lambda rgb: {
        "detections": [det("smoking", 0.3)], "inference_ms": 3.0})
    r_low = ins_low_abn.inspect("P")
    _check("vis.judge_lowconf_not_abnormal",
           r_low.status is not InspectionStatus.ABNORMAL, r_low.status.value)


def test_vis_timeout_controller_policy():
    # 推理超时后控制器按既定策略：状态如实记录为非 SUCCESS，流程可完成
    nav = SimNavigator()
    vision = _MockYoloInspector(
        frame=_FAKE_FRAME,
        detect_fn=lambda rgb: (time.sleep(0.6), {
            "detections": [], "inference_ms": 600.0})[1],
        inference_timeout=0.15)
    summary, records, nav, *_ = _run("vis_timeout_policy", nav,
                                     SimSpeaker(), vision)
    _check("vis.policy_run_done", summary.final_state is State.DONE,
           summary.final_state.value)
    pois = _poi_records(records)
    statuses = [r.get("inspection", {}).get("status") for r in pois]
    _check("vis.policy_none_success",
           len(pois) == 3 and all(s != "success" for s in statuses),
           str(statuses))
    _check("vis.policy_first_timeout", statuses[0] == "timeout", str(statuses))
    _check("vis.policy_rest_error",
           all(s == "error" for s in statuses[1:]), str(statuses))


def main():
    print("=== dry-run 场景测试 ===")
    test_success()
    test_nav_fail()
    test_nav_timeout()
    test_vision_modes()
    test_speech_fail_not_blocking()
    test_user_cancel()
    test_return_start()
    print("=== 实机门控与限速回滚 Mock 测试 ===")
    test_real_gate_confirm_logic()
    test_real_unconfirmed_exits_before_hardware()
    test_real_confirmed_enters_flow()
    test_sim_mode_bypasses_gate()
    test_speed_limit_unreadable_prev()
    test_speed_limit_write_raises_restore_fails()
    test_speed_limit_write_raises_restore_ok()
    test_speed_limit_verify_timeout_rollback()
    test_speed_limit_verify_raises_rollback()
    test_speed_limit_success_and_cleanup_restore()
    test_speed_limit_failure_blocks_navigation()
    print("=== 中断(Ctrl+C)与终止一致性 Mock 测试 ===")
    test_ki_during_dwell()
    test_ki_during_vision()
    test_ki_during_tts()
    test_abort_speech_failure_still_aborts()
    test_abort_summary_write_failure()
    test_done_path_speech_failure_stays_done()
    print("=== YOLO 推理超时 Mock 测试 ===")
    test_vis_camera_timeout()
    test_vis_camera_error()
    test_vis_no_frame()
    test_vis_inference_timeout_and_poison()
    test_vis_inference_error()
    test_vis_judgement_logic()
    test_vis_timeout_controller_policy()
    failed = [r for r in _results if r[1] == FAIL]
    print(f"\n=== 结果: {len(_results) - len(failed)}/{len(_results)} 通过 ===")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
