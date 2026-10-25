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
from inspection.speaker import SimSpeaker, TtsSpeaker, SETTLE_S
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
                 expect=("fire_extinguisher",), abnormal=("smoking",),
                 conf_by_class=None, to_bgr=False, min_conf=0.5,
                 absence_abnormal=("fire_extinguisher",),
                 poi_focus_classes=None):
        self._robot = _FakeCameraRobot(frame=frame, exc=frame_exc)
        self._yolo = _FakeYolo(detect_fn or (lambda rgb: {
            "detections": [], "inference_ms": 1.0}))
        self._expect = tuple(expect)
        self._abnormal = tuple(abnormal)
        self._absence_abnormal = tuple(absence_abnormal)
        self._poi_focus = {str(k): tuple(v)
                           for k, v in (poi_focus_classes or {}).items()}
        self._camera_name = "head"
        self._frame_timeout = frame_timeout
        self._inference_timeout = inference_timeout
        self._min_conf = min_conf
        self._class_conf = dict(conf_by_class) if conf_by_class else None
        self._to_bgr = to_bgr
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

    # 低置信度灭火器框：不算有效检出 → 按新规则判为 ABNORMAL（不是 SUCCESS）
    ins_unc = _MockYoloInspector(frame=_FAKE_FRAME, detect_fn=lambda rgb: {
        "detections": [det("fire_extinguisher", 0.3)], "inference_ms": 3.0})
    r_unc = ins_unc.inspect("P")
    _check("vis.judge_lowconf_extinguisher_abnormal",
           r_unc.status is InspectionStatus.ABNORMAL, r_unc.status.value)

    # 其余期望目标缺失仍是 UNCERTAIN（安全指示牌规则不变）
    ins_sign = _MockYoloInspector(
        frame=_FAKE_FRAME, expect=("safety_sign", "fire_extinguisher"),
        detect_fn=lambda rgb: {
            "detections": [det("fire_extinguisher", 0.9)],
            "inference_ms": 3.0})
    r_sign = ins_sign.inspect("P")
    _check("vis.judge_uncertain_safety_sign",
           r_sign.status is InspectionStatus.UNCERTAIN, r_sign.status.value)

    # 低置信度 smoking 不构成异常（expect 置空，隔离灭火器规则以便单独验证）
    ins_low_abn = _MockYoloInspector(frame=_FAKE_FRAME, expect=(),
                                     detect_fn=lambda rgb: {
        "detections": [det("smoking", 0.3)], "inference_ms": 3.0})
    r_low = ins_low_abn.inspect("P")
    _check("vis.judge_lowconf_not_abnormal",
           r_low.status is InspectionStatus.SUCCESS
           and "smoking" not in r_low.detail,
           f"{r_low.status.value} / {r_low.detail}")


def test_vis_extinguisher_presence_rule():
    """灭火器规则：≥1 个有效检出即正常（无数量上限），0 个才异常。"""
    def det(name, conf, xyxy=None):
        return {"class_name": name, "confidence": conf,
                "xyxy": xyxy or [0, 0, 5, 5]}

    def inspect_with(dets, **kw):
        ins = _MockYoloInspector(
            frame=_FAKE_FRAME, **kw,
            detect_fn=lambda rgb: {"detections": dets, "inference_ms": 1.0})
        return ins.inspect("P")

    # 0 个 → 异常（不是 SUCCESS，也不是"不确定"）
    r0 = inspect_with([])
    _check("ext.zero_abnormal",
           r0.status is InspectionStatus.ABNORMAL, r0.status.value)
    _check("ext.zero_detail_no_detection", "无检出" in r0.detail, r0.detail)

    # 1 / 2 / 3 / 10 个 → 均正常（无数量上限）
    for n in (1, 2, 3, 10):
        dets = [det("fire_extinguisher", 0.9, [i * 10, 0, i * 10 + 5, 5])
                for i in range(n)]
        r = inspect_with(dets)
        _check(f"ext.count_{n}_success",
               r.status is InspectionStatus.SUCCESS, f"{n}个: {r.status.value}")

    # 重复框（同一目标被重复检出）：按类名集合去重，仍是 SUCCESS
    dup = [det("fire_extinguisher", 0.9, [0, 0, 5, 5])] * 3
    rd = inspect_with(dup)
    _check("ext.duplicate_boxes_success",
           rd.status is InspectionStatus.SUCCESS, rd.status.value)

    # 仅低于阈值的框 → 不算检出成功 → 异常，且 detail 标明阈值原因
    rlow = inspect_with([det("fire_extinguisher", 0.3)], min_conf=0.65)
    _check("ext.lowconf_abnormal",
           rlow.status is InspectionStatus.ABNORMAL, rlow.status.value)
    _check("ext.lowconf_detail_mentions_threshold",
           "低于阈值" in rlow.detail and "0.30" in rlow.detail
           and "0.65" in rlow.detail, rlow.detail)

    # 混合：1 个有效 + 若干低置信度 → 有效者已满足，判正常
    rmix = inspect_with([det("fire_extinguisher", 0.9),
                         det("fire_extinguisher", 0.3)], min_conf=0.65)
    _check("ext.one_valid_plus_lowconf_success",
           rmix.status is InspectionStatus.SUCCESS, rmix.status.value)

    # 吸烟规则不变：达标 smoking + 灭火器缺失 → 仍按 smoking 判异常
    rsmoke = inspect_with([det("smoking", 0.9)])
    _check("ext.smoking_rule_unchanged",
           rsmoke.status is InspectionStatus.ABNORMAL
           and "smoking" in rsmoke.detail, rsmoke.detail)

    # 安全指示牌缺失、灭火器在位 → 仍 UNCERTAIN（其他巡检规则不变）
    rsign = inspect_with([det("fire_extinguisher", 0.9)],
                         expect=("safety_sign", "fire_extinguisher"))
    _check("ext.safety_sign_rule_unchanged",
           rsign.status is InspectionStatus.UNCERTAIN, rsign.status.value)


def test_report_extinguisher_presence_rule():
    """播报文案：0 个不说"正常"，1/2/3 个均播报在位。"""
    from inspection.report import SPEECH_TEXTS, build_speech_text
    from inspection.types import Detection, PoiInspection

    def det(conf, xyxy=None):
        return Detection(class_name="fire_extinguisher", confidence=conf,
                         xyxy=xyxy or [0, 0, 5, 5])

    expect = ("fire_extinguisher",)

    # 0 个：必须告警并点名灭火器，绝不能出现"正常/未发现异常"
    t0 = build_speech_text(
        "配电间",
        PoiInspection(status=InspectionStatus.ABNORMAL, poi="配电间"),
        expect_classes=expect, abnormal_classes=("smoking",), min_conf=0.65)
    _check("report.ext.zero_warns",
           t0 == SPEECH_TEXTS["extinguisher_missing"], t0)
    _check("report.ext.zero_not_normal",
           "未发现异常" not in t0 and "正常" not in t0
           and "火灾" not in t0, t0)

    # 1 / 2 / 3 个：均播报固定"灭火器检查正常。"
    for n in (1, 2, 3):
        dets = [det(0.9, [i * 10, 0, i * 10 + 5, 5]) for i in range(n)]
        t = build_speech_text(
            "配电间",
            PoiInspection(status=InspectionStatus.SUCCESS, poi="配电间",
                          detections=dets),
            expect_classes=expect, abnormal_classes=("smoking",),
            min_conf=0.65)
        _check(f"report.ext.count_{n}_normal",
               t == SPEECH_TEXTS["extinguisher_ok"] and "火灾" not in t,
               f"{n}个: {t}")


def test_fixed_speech_texts():
    """固定播报文本：内容准确、不含"火灾"、吸烟未检出时不播警告。"""
    from inspection.report import SPEECH_TEXTS, build_speech_text
    from inspection.types import Detection, PoiInspection

    expected = {
        "start": "消防巡检开始，请注意现场安全。",
        "extinguisher_ok": "灭火器检查正常。",
        "extinguisher_missing": "警告，未检测到灭火器，请工作人员检查。",
        "safety_sign_ok": "安全出口指示灯检查正常。",
        "safety_sign_missing": "警告，未检测到安全出口指示灯，请工作人员检查。",
        "smoking": "警告，发现吸烟行为，请立即停止吸烟。",
        "done": "本次消防巡检已完成。",
    }
    _check("texts.all_fixed", SPEECH_TEXTS == expected, str(SPEECH_TEXTS))
    _check("texts.no_fire_word",
           all("火灾" not in v for v in SPEECH_TEXTS.values()), "")

    def det(name, conf):
        return Detection(class_name=name, confidence=conf, xyxy=[0, 0, 1, 1])

    # 吸烟未检出 → 正常文本不含吸烟警告
    r = PoiInspection(status=InspectionStatus.SUCCESS, poi="P",
                      detections=[det("fire_extinguisher", 0.9)])
    t = build_speech_text("P", r, expect_classes=("fire_extinguisher",),
                          abnormal_classes=("smoking",), min_conf=0.65)
    _check("texts.no_smoking_when_absent",
           SPEECH_TEXTS["smoking"] not in t and "吸烟" not in t, t)

    # 吸烟 + 灭火器缺失：两条固定警告都播报，绝不含"火灾"
    r = PoiInspection(status=InspectionStatus.ABNORMAL, poi="P",
                      detections=[det("smoking", 0.9)])
    t = build_speech_text("P", r, expect_classes=("fire_extinguisher",),
                          abnormal_classes=("smoking",), min_conf=0.65,
                          conf_by_class={"smoking": 0.25})
    _check("texts.smoking_and_missing_both",
           SPEECH_TEXTS["smoking"] in t
           and SPEECH_TEXTS["extinguisher_missing"] in t
           and "火灾" not in t, t)


def test_manual_loop_speech_wiring():
    """单点入口打通：视觉结论 → 固定文本 → speaker 实际调用。"""
    from inspection.report import SPEECH_TEXTS

    rc, records, spk = _run_manual(["配电间", "q"])
    texts = [t for t, _ in spk.calls]
    _check("manual.wire_rc0", rc == 0, f"rc={rc}")
    _check("manual.wire_start_first",
           texts and texts[0] == SPEECH_TEXTS["start"],
           str(texts[:1]))
    _check("manual.wire_report_fixed",
           SPEECH_TEXTS["extinguisher_ok"] in texts, str(texts))
    _check("manual.wire_done_last",
           texts and texts[-1] == SPEECH_TEXTS["done"], str(texts[-1:]))
    _check("manual.wire_no_fire_word",
           all("火灾" not in t for t in texts), "")

    stages = [r for r in records if r.get("record_type") == "speech_stage"]
    _check("manual.wire_stage_records",
           [s["stage"] for s in stages] == ["start", "done"]
           and all(s["speech"]["state"] == "done_observed" for s in stages),
           str([(s["stage"], s["speech"]["state"]) for s in stages]))


def test_controller_fixed_speech_routing():
    """自动巡检控制器：按识别结论选固定文本，绝不误报火灾。"""
    from inspection.report import SPEECH_TEXTS

    def det(name, conf):
        return {"class_name": name, "confidence": conf, "xyxy": [0, 0, 5, 5]}

    # 1) 0 个灭火器 → 缺失警告；开始/完成使用固定文本
    vision0 = _MockYoloInspector(
        frame=_FAKE_FRAME, expect=("fire_extinguisher",),
        detect_fn=lambda rgb: {"detections": [], "inference_ms": 1.0})
    _, records, _, spk0, _ = _run("ctl_no_ext", SimNavigator(),
                                  SimSpeaker(), vision0)
    texts0 = [p["speech"]["report"]["text"] for p in _poi_records(records)]
    _check("ctl.no_ext_missing_text",
           texts0 and all(t == SPEECH_TEXTS["extinguisher_missing"]
                          for t in texts0), str(texts0))
    calls0 = [t for t, _ in spk0.calls]
    _check("ctl.start_fixed", calls0 and calls0[0] == SPEECH_TEXTS["start"],
           str(calls0[:1]))
    _check("ctl.done_fixed", calls0 and calls0[-1] == SPEECH_TEXTS["done"],
           str(calls0[-1:]))

    # 2) 吸烟（灭火器在位）→ 只播吸烟警告，不播"火灾"
    vision1 = _MockYoloInspector(
        frame=_FAKE_FRAME, expect=("fire_extinguisher",),
        detect_fn=lambda rgb: {
            "detections": [det("smoking", 0.9),
                           det("fire_extinguisher", 0.9)],
            "inference_ms": 1.0})
    _, records, _, spk1, _ = _run("ctl_smoke", SimNavigator(),
                                  SimSpeaker(), vision1)
    texts1 = [p["speech"]["report"]["text"] for p in _poi_records(records)]
    _check("ctl.smoking_text",
           texts1 and all(t == SPEECH_TEXTS["smoking"] for t in texts1),
           str(texts1))
    _check("ctl.smoking_no_fire",
           all("火灾" not in t for t in texts1), "")
    calls1 = [t for t, _ in spk1.calls]
    _check("ctl.smoking_actually_spoken",
           SPEECH_TEXTS["smoking"] in calls1
           and all("火灾" not in t for t in calls1), str(calls1))


def test_report_safety_sign_texts():
    """安全出口指示灯专用文案：正常/异常/不确定/取图失败/超时，且不扩散。"""
    from inspection.report import SPEECH_TEXTS, build_speech_text
    from inspection.types import Detection, PoiInspection

    def det(conf, xyxy=None):
        return Detection(class_name="safety_sign", confidence=conf,
                         xyxy=xyxy or [0, 0, 5, 5])

    def smoke(conf=0.9):
        return Detection(class_name="smoking", confidence=conf,
                         xyxy=[0, 0, 5, 5])

    focus = ("safety_sign",)
    kw = dict(expect_classes=("safety_sign", "fire_extinguisher"),
              abnormal_classes=("smoking",), min_conf=0.65,
              conf_by_class={"smoking": 0.25}, focus_classes=focus)

    # 正常：1/2/3 个有效框都播同一固定文本
    for n in (1, 2, 3):
        dets = [det(0.9, [i * 10, 0, i * 10 + 5, 5]) for i in range(n)]
        t = build_speech_text(
            "安全出口", PoiInspection(status=InspectionStatus.SUCCESS,
                                     detections=dets), **kw)
        _check(f"sign.count_{n}_ok",
               t == SPEECH_TEXTS["safety_sign_ok"], t)

    # 异常：0 个有效检出 / 只有低置信度框 → 缺失固定警告，绝不"正常"
    for name, dets in (("zero", []), ("lowconf", [det(0.3)])):
        t = build_speech_text(
            "安全出口", PoiInspection(status=InspectionStatus.ABNORMAL,
                                     detections=dets), **kw)
        _check(f"sign.{name}_missing",
               t == SPEECH_TEXTS["safety_sign_missing"], t)
        _check(f"sign.{name}_not_normal",
               "正常" not in t and "未发现异常" not in t, t)

    # 不确定 / 取图失败 / 推理超时 / 异常：不得使用确定性结论
    for st in (InspectionStatus.UNCERTAIN, InspectionStatus.NO_FRAME,
               InspectionStatus.TIMEOUT, InspectionStatus.ERROR):
        t = build_speech_text(
            "安全出口", PoiInspection(status=st, poi="安全出口"), **kw)
        _check(f"sign.{st.value}_needs_manual",
               "正常" not in t
               and SPEECH_TEXTS["safety_sign_missing"] not in t
               and "人工" in t, t)

    # 专用点位吸烟：只播吸烟警告，目标在位时不得误报缺失、也不提灭火器
    t = build_speech_text(
        "安全出口", PoiInspection(status=InspectionStatus.ABNORMAL,
                                 detections=[det(0.9), smoke()]), **kw)
    _check("sign.smoking_only_when_present",
           t == SPEECH_TEXTS["smoking"]
           and "灭火器" not in t and "安全出口指示灯" not in t, t)

    # 吸烟 + 指示灯缺失：两条警告合并，仍不得提灭火器（专用点只看本点目标）
    t = build_speech_text(
        "安全出口", PoiInspection(status=InspectionStatus.ABNORMAL,
                                 detections=[smoke()]), **kw)
    _check("sign.smoking_and_missing",
           SPEECH_TEXTS["smoking"] in t
           and SPEECH_TEXTS["safety_sign_missing"] in t
           and "灭火器" not in t and "火灾" not in t, t)

    # 非专用点位：正常结果含 safety_sign 也绝不能播专用正常结论
    t = build_speech_text(
        "货架", PoiInspection(status=InspectionStatus.SUCCESS,
                             detections=[det(0.9)]),
        expect_classes=("safety_sign",), abnormal_classes=("smoking",),
        min_conf=0.65)
    _check("sign.no_focus_not_special",
           t != SPEECH_TEXTS["safety_sign_ok"]
           and "安全出口指示灯检查正常" not in t, t)


def test_poi_focus_parse():
    """专用点位映射只做显式解析，格式错误直接拒绝。"""
    from inspection.report import parse_poi_focus

    m = parse_poi_focus(["安全出口=safety_sign", "P2=safety_sign,fire_extinguisher",
                         "  ", "P3 = safety_sign"])
    _check("focus.parse_values",
           m == {"安全出口": ("safety_sign",),
                 "P2": ("safety_sign", "fire_extinguisher"),
                 "P3": ("safety_sign",)}, str(m))
    _check("focus.parse_empty", parse_poi_focus([]) == {}, "")
    for bad in ("安全出口", "=safety_sign", "P4="):
        try:
            parse_poi_focus([bad])
            rejected = False
        except ValueError:
            rejected = True
        _check(f"focus.parse_reject[{bad}]", rejected)


def test_vis_safety_sign_focus_rule():
    """专用点位判定：只看本点目标；未配置的点位行为不变。"""

    def det(name, conf, xyxy=None):
        return {"class_name": name, "confidence": conf,
                "xyxy": xyxy or [0, 0, 5, 5]}

    def make(dets, poi_focus=None, **kw):
        return _MockYoloInspector(
            frame=_FAKE_FRAME, expect=("fire_extinguisher",),
            min_conf=0.65, poi_focus_classes=poi_focus or {
                "SIGN": ("safety_sign",)},
            detect_fn=lambda rgb: {"detections": dets, "inference_ms": 1.0},
            **kw)

    r = make([det("safety_sign", 0.9)]).inspect("SIGN")
    _check("vis.sign_focus_present_success",
           r.status is InspectionStatus.SUCCESS, r.status.value)
    r = make([det("safety_sign", 0.9), det("safety_sign", 0.9)]).inspect("SIGN")
    _check("vis.sign_focus_duplicate_success",
           r.status is InspectionStatus.SUCCESS, r.status.value)
    r = make([]).inspect("SIGN")
    _check("vis.sign_focus_absent_abnormal",
           r.status is InspectionStatus.ABNORMAL
           and "未检出有效目标" in r.detail and "无检出" in r.detail,
           f"{r.status.value} {r.detail}")
    r = make([det("safety_sign", 0.3)]).inspect("SIGN")
    _check("vis.sign_focus_lowconf_abnormal",
           r.status is InspectionStatus.ABNORMAL and "低于阈值" in r.detail,
           f"{r.status.value} {r.detail}")
    r = make([det("safety_sign", 0.9), det("safety_sign", 0.3)]).inspect("SIGN")
    _check("vis.sign_focus_one_valid_success",
           r.status is InspectionStatus.SUCCESS, r.status.value)
    r = make([det("safety_sign", 0.9), det("smoking", 0.9)]).inspect("SIGN")
    _check("vis.sign_focus_smoking_abnormal",
           r.status is InspectionStatus.ABNORMAL
           and "smoking" in r.detail, f"{r.status.value} {r.detail}")

    # 全局 expect 只在非专用点位生效：缺失 safety_sign → UNCERTAIN（原规则）
    ins_other = _MockYoloInspector(
        frame=_FAKE_FRAME, expect=("safety_sign",),
        min_conf=0.65, poi_focus_classes={"SIGN": ("safety_sign",)},
        detect_fn=lambda rgb: {"detections": [], "inference_ms": 1.0})
    r = ins_other.inspect("OTHER")
    _check("vis.sign_non_focus_uncertain_unchanged",
           r.status is InspectionStatus.UNCERTAIN, r.status.value)
    # 非专用点位仍保留灭火器缺失 → ABNORMAL 的原规则
    ins_ext = _MockYoloInspector(
        frame=_FAKE_FRAME, expect=("safety_sign", "fire_extinguisher"),
        min_conf=0.65,
        detect_fn=lambda rgb: {"detections": [det("safety_sign", 0.9)],
                               "inference_ms": 1.0})
    r = ins_ext.inspect("OTHER")
    _check("vis.ext_rule_unchanged_at_other_poi",
           r.status is InspectionStatus.ABNORMAL
           and "fire_extinguisher" in r.detail, r.detail)

    # 取图失败 / 推理超时：专用点位也不能被当成正常或缺失结论
    r = make([], frame_exc=TimeoutError("timeout")).inspect("SIGN")
    _check("vis.sign_focus_camera_timeout",
           r.status is InspectionStatus.NO_FRAME, r.status.value)
    r = make(
        [], inference_timeout=0.1,
        detect_fn=lambda rgb: (time.sleep(0.4), {
            "detections": [det("safety_sign", 0.9)],
            "inference_ms": 400.0})[1]).inspect("SIGN")
    _check("vis.sign_focus_inference_timeout",
           r.status is InspectionStatus.TIMEOUT, r.status.value)


def test_controller_safety_sign_focus_speech():
    """控制器：专用点位播专用文本，其他点位不播专用文本。"""
    from inspection.report import SPEECH_TEXTS

    def det(name, conf):
        return {"class_name": name, "confidence": conf, "xyxy": [0, 0, 5, 5]}

    cfg = InspectionConfig(pois=("SIGN_PT", "OTHER_PT"), dwell_seconds=0.02,
                           poi_focus_classes={"SIGN_PT": ("safety_sign",)})
    vision = _MockYoloInspector(
        frame=_FAKE_FRAME, expect=("fire_extinguisher",), min_conf=0.65,
        poi_focus_classes={"SIGN_PT": ("safety_sign",)},
        detect_fn=lambda rgb: {
            "detections": [det("safety_sign", 0.9),
                           det("fire_extinguisher", 0.9)],
            "inference_ms": 1.0})
    _, records, _, spk, _ = _run("ctl_sign", SimNavigator(), SimSpeaker(),
                                 vision, cfg)
    by_poi = {p["poi"]: p["speech"]["report"]["text"]
              for p in _poi_records(records)}
    _check("ctl.sign_poi_ok_text",
           by_poi.get("SIGN_PT") == SPEECH_TEXTS["safety_sign_ok"],
           str(by_poi))
    _check("ctl.other_poi_no_sign_text",
           by_poi.get("OTHER_PT") != SPEECH_TEXTS["safety_sign_ok"]
           and by_poi.get("OTHER_PT") == SPEECH_TEXTS["extinguisher_ok"],
           str(by_poi))
    calls = [t for t, _ in spk.calls]
    _check("ctl.sign_focus_spoken",
           SPEECH_TEXTS["safety_sign_ok"] in calls, str(calls))

    # 专用点位缺失 → 专用异常文本；其他点位不受影响
    cfg2 = InspectionConfig(pois=("SIGN_PT", "OTHER_PT"), dwell_seconds=0.02,
                            poi_focus_classes={"SIGN_PT": ("safety_sign",)})
    vision2 = _MockYoloInspector(
        frame=_FAKE_FRAME, expect=("fire_extinguisher",), min_conf=0.65,
        poi_focus_classes={"SIGN_PT": ("safety_sign",)},
        detect_fn=lambda rgb: {
            "detections": [det("fire_extinguisher", 0.9)],
            "inference_ms": 1.0})
    _, records2, _, _, _ = _run("ctl_sign_missing", SimNavigator(),
                                SimSpeaker(), vision2, cfg2)
    by_poi2 = {p["poi"]: p["speech"]["report"]["text"]
               for p in _poi_records(records2)}
    _check("ctl.sign_poi_missing_text",
           by_poi2.get("SIGN_PT") == SPEECH_TEXTS["safety_sign_missing"]
           and "灭火器" not in by_poi2.get("SIGN_PT", ""), str(by_poi2))
    _check("ctl.other_poi_unchanged",
           by_poi2.get("OTHER_PT") == SPEECH_TEXTS["extinguisher_ok"],
           str(by_poi2))


def test_manual_safety_sign_focus_speech():
    """单点入口：映射内的点位播专用文本，未映射点位保持通用文案。"""
    from inspection.report import SPEECH_TEXTS
    rc, records, spk = _run_manual(
        ["安全出口", "货架", "q"], poi_focus={"安全出口": ("safety_sign",)})
    by_poi = {p["poi"]: p["speech"]["report"]["text"]
              for p in _poi_records(records)}
    _check("manual.sign_focus_rc0", rc == 0, f"rc={rc}")
    _check("manual.sign_focus_text",
           by_poi.get("安全出口") == SPEECH_TEXTS["safety_sign_ok"],
           str(by_poi))
    _check("manual.other_poi_not_sign",
           by_poi.get("货架") != SPEECH_TEXTS["safety_sign_ok"], str(by_poi))


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


# ── TTS 语义与完成判定：纯 Mock 测试（不连 ROS / 不发声 / 不真实等待） ─────

import inspection.speaker as _spk_mod


class _FakeResp:
    """模拟 interaction_msgs/srv/TtsService.Response。"""

    def __init__(self, success=True, status=""):
        self.success = success
        self.status = status


class _ScriptedCall:
    """按脚本顺序返回 (ok, resp)，超出脚本走默认值。记录调用 (cmd, text)。"""

    def __init__(self, script):
        # script 元素：(ok, resp) 或 callable(cmd,text)->(ok,resp)
        self.script = list(script)
        self.idx = 0
        self.log = []  # (cmd, text)

    def __call__(self, text, cmd):
        self.log.append((cmd, text))
        if self.idx < len(self.script):
            entry = self.script[self.idx]
            self.idx += 1
            if callable(entry):
                return entry(cmd, text)
            return entry
        # 默认：append 成功；query 返回 idle
        if cmd == "query":
            return True, _FakeResp(True, "idle")
        return True, _FakeResp(True, "success")


class _Clock:
    """可控时钟：monotonic 每次调用前进 step，sleep 空转。"""

    def __init__(self, step=0.5):
        self.t = 0.0
        self.step = step

    def monotonic(self):
        t = self.t
        self.t += self.step
        return t

    def sleep(self, _s):
        pass


def _make_tts_for_test(script, done_timeout_s=30.0, clock_step=0.5):
    """绕过真实 __init__（不连服务），注入脚本化 _call 与可控时钟。"""
    spk = TtsSpeaker.__new__(TtsSpeaker)
    spk._audio_id = "audio_guide"
    spk._done_timeout_s = done_timeout_s
    caller = _ScriptedCall(script)
    spk._call = caller
    clock = _Clock(step=clock_step)
    fake_time = _pytypes.SimpleNamespace(
        monotonic=clock.monotonic, sleep=clock.sleep)
    orig_time = _spk_mod.time
    _spk_mod.time = fake_time
    return spk, caller, lambda: setattr(_spk_mod, "time", orig_time)


def _ok(status="success"):
    return True, _FakeResp(True, status)


def _query(status):
    return True, _FakeResp(True, status)


def test_tts_interrupt_calls_stop_then_append():
    # interrupt=True：必须 stop(空text) → append(目标文本)，两步
    spk, caller, restore = _make_tts_for_test([
        _ok(),            # stop
        _ok(),            # append
        _query("playing"),
        _query("idle"),
        _query("idle"),
    ])
    try:
        r = spk.say("测试文本", interrupt=True)
    finally:
        restore()
    _check("tts.interrupt.stop_first",
           caller.log[0] == ("stop", ""), str(caller.log))
    _check("tts.interrupt.append_second",
           caller.log[1] == ("append", "测试文本"), str(caller.log))
    _check("tts.interrupt.done", r.state is SpeechState.DONE_OBSERVED,
           r.state.value)
    _check("tts.interrupt.no_stop_with_text",
           all(not (c == "stop" and t) for c, t in caller.log), str(caller.log))


def test_tts_no_interrupt_only_append():
    # interrupt=False：只调用 append，不得出现 stop
    spk, caller, restore = _make_tts_for_test([
        _ok(),            # append
        _query("playing"),
        _query("idle"),
        _query("idle"),
    ])
    try:
        r = spk.say("测试文本", interrupt=False)
    finally:
        restore()
    cmds = [c for c, _ in caller.log]
    _check("tts.noint.first_append", cmds[0] == "append", str(cmds))
    _check("tts.noint.no_stop", "stop" not in cmds, str(cmds))
    _check("tts.noint.done", r.state is SpeechState.DONE_OBSERVED,
           r.state.value)


def test_tts_bounce_no_premature_done():
    # idle → playing → idle → playing：中间空窗未稳定，不得判定 DONE
    spk, caller, restore = _make_tts_for_test([
        _ok(), _ok(),                    # stop, append
        _query("idle"), _query("playing"),
        _query("idle"), _query("playing"),
        _query("playing"), _query("playing"),
        _query("playing"), _query("playing"),
    ], done_timeout_s=2.5, clock_step=0.5)
    try:
        r = spk.say("短", interrupt=True)
    finally:
        restore()
    _check("tts.bounce.not_done",
           r.state is not SpeechState.DONE_OBSERVED, r.state.value)
    _check("tts.bounce.timeout",
           r.state is SpeechState.QUERY_TIMEOUT, r.state.value)


def test_tts_settle_before_done():
    # playing 后短暂非播放，再连续稳定非播放：达到 SETTLE_S 才 DONE
    spk, caller, restore = _make_tts_for_test([
        _ok(),                           # append
        _query("playing"),
        _query("idle"),                  # 首次 idle，未达 SETTLE
        _query("idle"),                  # 连续 idle，now-idle>=SETTLE → DONE
    ], clock_step=0.5)  # 0.5 > SETTLE_S(0.4)
    try:
        r = spk.say("短", interrupt=False)
    finally:
        restore()
    _check("tts.settle.done", r.state is SpeechState.DONE_OBSERVED,
           r.state.value)
    _check("tts.settle.observed", r.observed_transition is True)


def test_tts_settle_not_reached_no_done():
    # playing 后只有一次 idle（未达 SETTLE_S），不得 DONE
    spk, caller, restore = _make_tts_for_test([
        _ok(),                           # append
        _query("playing"),
        _query("idle"),                  # 仅一帧 idle，不满足连续稳定
        _query("idle"),
    ], done_timeout_s=1.0, clock_step=0.2)  # 0.2 < SETTLE_S(0.4)
    try:
        r = spk.say("短", interrupt=False)
    finally:
        restore()
    _check("tts.nosettle.not_done",
           r.state is not SpeechState.DONE_OBSERVED, r.state.value)


def test_tts_never_playing_not_faked_done():
    # 始终未观测到 playing：只能 SUBMIT_UNCONFIRMED，不得伪造 DONE_OBSERVED
    spk, caller, restore = _make_tts_for_test([
        _ok(),                           # append
        _query("idle"), _query("idle"), _query("idle"),
        _query("idle"), _query("idle"), _query("idle"),
    ], clock_step=0.5)
    try:
        r = spk.say("测试", interrupt=False)  # est=max(1.5, 2*0.22)=1.5, +1.0=2.5
    finally:
        restore()
    _check("tts.never.state",
           r.state is SpeechState.SUBMIT_UNCONFIRMED, r.state.value)
    _check("tts.never.not_done",
           r.state is not SpeechState.DONE_OBSERVED, r.state.value)
    _check("tts.never.not_observed", r.observed_transition is False)


def test_tts_append_call_exception():
    spk, caller, restore = _make_tts_for_test([
        (False, RuntimeError("network down")),  # append 调用异常
    ])
    try:
        r = spk.say("测试", interrupt=False)
    finally:
        restore()
    _check("tts.append_exc.state",
           r.state is SpeechState.SUBMIT_FAILED, r.state.value)
    _check("tts.append_exc.not_success", r.submit_success is False)


def test_tts_append_success_false():
    spk, caller, restore = _make_tts_for_test([
        (True, _FakeResp(False, "rejected")),  # success=false
    ])
    try:
        r = spk.say("测试", interrupt=False)
    finally:
        restore()
    _check("tts.reject.state",
           r.state is SpeechState.SUBMIT_FAILED, r.state.value)
    _check("tts.reject.status", r.submit_status == "rejected",
           r.submit_status)


def test_tts_query_error():
    # append 成功，但 query 调用失败 → QUERY_ERROR
    spk, caller, restore = _make_tts_for_test([
        _ok(),
        (False, RuntimeError("query rpc failed")),
    ])
    try:
        r = spk.say("测试", interrupt=False)
    finally:
        restore()
    _check("tts.query_err.state",
           r.state is SpeechState.QUERY_ERROR, r.state.value)


def test_tts_always_playing_hard_timeout():
    # 一直 playing，直到硬超时 → QUERY_TIMEOUT
    spk, caller, restore = _make_tts_for_test([
        _ok(),
        _query("playing"), _query("playing"), _query("playing"),
        _query("playing"), _query("playing"),
    ], done_timeout_s=1.5, clock_step=0.5)
    try:
        r = spk.say("测试", interrupt=False)
    finally:
        restore()
    _check("tts.always_playing.state",
           r.state is SpeechState.QUERY_TIMEOUT, r.state.value)
    _check("tts.always_playing.observed", r.observed_transition is True)


def test_tts_stop_failure_still_appends():
    # stop 失败（best-effort）不阻塞 append，仍能正常播报完成
    spk, caller, restore = _make_tts_for_test([
        (False, RuntimeError("stop rpc failed")),  # stop 失败
        _ok(),                                      # append 成功
        _query("playing"), _query("idle"), _query("idle"),
    ])
    try:
        r = spk.say("测试", interrupt=True)
    finally:
        restore()
    _check("tts.stop_fail.done",
           r.state is SpeechState.DONE_OBSERVED, r.state.value)
    _check("tts.stop_fail.append_called",
           ("append", "测试") in caller.log, str(caller.log))


def test_tts_exact_playing_match():
    # 精确匹配：status="PLAYING" 或 "not_playing" 不算在播
    spk, caller, restore = _make_tts_for_test([
        _ok(),
        _query("PLAYING"),      # 大小写不同，不算 playing
        _query("not_playing"),  # 含子串但不精确，不算 playing
        _query("playing"),
        _query("idle"), _query("idle"),
    ], clock_step=0.5)
    try:
        r = spk.say("测试", interrupt=False)
    finally:
        restore()
    _check("tts.exact.done", r.state is SpeechState.DONE_OBSERVED,
           r.state.value)


# ── TTS 结构化日志：纯 Mock，不连真实服务 ─────────────────────────

class _LogCapture:
    """捕获 inspection.speaker 日志记录（不依赖真实 handler 输出）。"""

    def __init__(self):
        import logging as _lg
        self._lg = _lg
        self.records = []
        handler = _lg.Handler()
        handler.emit = self.records.append
        self._handler = handler
        logger = _lg.getLogger("inspection.speaker")
        self._logger = logger
        self._old_level = logger.level
        logger.setLevel(_lg.DEBUG)
        logger.addHandler(handler)

    def messages(self):
        return [r.getMessage() for r in self.records]

    def close(self):
        self._logger.removeHandler(self._handler)
        self._logger.setLevel(self._old_level)


def _say_with_logs(script, text="测试文本", interrupt=True, **kw):
    cap = _LogCapture()
    spk, caller, restore = _make_tts_for_test(script, **kw)
    try:
        r = spk.say(text, interrupt=interrupt)
    finally:
        restore()
    msgs = cap.messages()
    cap.close()
    return r, caller, msgs


def test_tts_log_done_path():
    r, caller, msgs = _say_with_logs([
        _ok(), _ok(),                       # stop, append
        _query("playing"), _query("idle"), _query("idle"),
    ])
    _check("ttslog.done.state", r.state is SpeechState.DONE_OBSERVED,
           r.state.value)
    joined = "\n".join(msgs)
    _check("ttslog.done.start", "start" in joined and "interrupt=True"
           in joined and "audio_id=audio_guide" in joined, joined)
    _check("ttslog.done.submit", "submit ok" in joined, joined)
    _check("ttslog.done.final", "final=done_observed" in joined
           and "elapsed=" in joined, joined)
    _check("ttslog.done.stop", "stop ok=" in joined, joined)


def test_tts_log_no_full_text():
    secret = "绝密播报内容XYZ"
    r, caller, msgs = _say_with_logs([
        _ok(), _query("playing"), _query("idle"), _query("idle"),
    ], text=secret, interrupt=False)
    joined = "\n".join(msgs)
    _check("ttslog.privacy.no_full_text", secret not in joined, joined)
    _check("ttslog.privacy.has_ref", f"len={len(secret)}" in joined
           and "sha1=" in joined, joined)


def test_tts_log_distinct_failures():
    # 四种失败终态：日志 final 标签必须两两可区分
    # 1) 提交调用异常
    _, _, m1 = _say_with_logs([(False, RuntimeError("boom"))],
                              interrupt=False)
    # 2) 查询异常
    _, _, m2 = _say_with_logs([
        _ok(), (False, RuntimeError("query rpc failed"))], interrupt=False)
    # 3) 查询超时（一直在播，永不稳定）——done_timeout_s 小值加速
    _, _, m3 = _say_with_logs([
        _ok(), _query("playing")], done_timeout_s=1.0, clock_step=0.5,
        interrupt=False)
    # 4) 提交未确认（从未观测到 playing，估算期结束）——估算期=1.5+1.0s
    _, _, m4 = _say_with_logs([
        _ok(), _query("idle")], clock_step=0.5, interrupt=False)
    j = ["\n".join(m) for m in (m1, m2, m3, m4)]
    _check("ttslog.fail.submit_failed",
           "final=submit_failed" in j[0], j[0])
    _check("ttslog.fail.query_error",
           "final=query_error" in j[1], j[1])
    _check("ttslog.fail.query_timeout",
           "final=query_timeout" in j[2], j[2])
    _check("ttslog.fail.unconfirmed",
           "final=submit_unconfirmed" in j[3], j[3])
    tags = ["submit_failed", "query_error", "query_timeout",
            "submit_unconfirmed"]
    _check("ttslog.fail.all_distinct", len(set(tags)) == 4
           and all(f"final={t}" in jj for t, jj in zip(tags, j)), str(j))


def test_tts_log_req_ids_unique():
    script = [_ok(), _query("playing"), _query("idle"), _query("idle")]
    _, _, m1 = _say_with_logs(list(script), interrupt=False)
    _, _, m2 = _say_with_logs(list(script), interrupt=False)

    def _rid(msgs):
        for m in msgs:
            if "start" in m:
                return m.split("]")[0]
        return ""
    r1, r2 = _rid(m1), _rid(m2)
    _check("ttslog.reqid.format", r1.startswith("tts[tts-")
           or r1.startswith("tts-") or "tts-" in r1, r1)
    _check("ttslog.reqid.unique", bool(r1) and bool(r2) and r1 != r2,
           f"{r1} vs {r2}")


def test_tts_log_query_raw_recorded():
    _, _, msgs = _say_with_logs([
        _ok(), _query("playing"), _query("idle"), _query("idle"),
    ], interrupt=False)
    joined = "\n".join(msgs)
    _check("ttslog.query.raw_playing", "raw='playing'" in joined, joined)
    _check("ttslog.query.raw_idle", "raw='idle'" in joined, joined)


def test_tts_log_interrupt_order_unchanged():
    # 开启日志捕获后，stop(空text) → append(文本) 调用顺序不变
    r, caller, msgs = _say_with_logs([
        _ok(), _ok(), _query("playing"), _query("idle"), _query("idle"),
    ], interrupt=True)
    cmds = [c for c, _ in caller.log]
    _check("ttslog.order.stop_then_append",
           cmds[0] == "stop" and cmds[1] == "append", str(cmds))
    _check("ttslog.order.stop_empty_text",
           caller.log[0][1] == "" and caller.log[1][1] != "",
           str(caller.log))


# ── 文案映射层 report.py：纯离线 ──────────────────────────────────

def test_report_texts():
    from inspection.report import build_speech_text
    from inspection.types import Detection, PoiInspection

    def det(name, conf):
        return Detection(class_name=name, confidence=conf,
                         xyxy=[0, 0, 5, 5])

    from inspection.report import SPEECH_TEXTS
    expect = ("safety_sign", "fire_extinguisher")
    abnormal = ("smoking",)

    # SUCCESS：灭火器期望 → 固定文本"灭火器检查正常。"
    r = PoiInspection(status=InspectionStatus.SUCCESS, poi="P1",
                      detections=[det("safety_sign", 0.9),
                                  det("fire_extinguisher", 0.9)])
    t = build_speech_text("P1", r, expect_classes=expect,
                          abnormal_classes=abnormal, min_conf=0.65)
    _check("report.success_fixed_text",
           t == SPEECH_TEXTS["extinguisher_ok"], t)
    _check("report.success_no_fire_word", "火灾" not in t, t)

    # ABNORMAL(smoking)：固定吸烟警告，绝不播"火灾"
    # （本例未检出灭火器，按规则会追加缺失警告，故只断言吸烟文本在位）
    r = PoiInspection(status=InspectionStatus.ABNORMAL, poi="P2",
                      detections=[det("smoking", 0.9)])
    t = build_speech_text("P2", r, expect_classes=expect,
                          abnormal_classes=abnormal, min_conf=0.65,
                          conf_by_class={"smoking": 0.25})
    _check("report.abnormal_smoking",
           SPEECH_TEXTS["smoking"] in t and "火灾" not in t, t)

    # ABNORMAL 且低置信度吸烟被阈值过滤 → 只剩灭火器缺失警告，不编造吸烟
    r = PoiInspection(status=InspectionStatus.ABNORMAL, poi="P3",
                      detections=[det("smoking", 0.1)])
    t = build_speech_text("P3", r, expect_classes=expect,
                          abnormal_classes=abnormal, min_conf=0.65,
                          conf_by_class={"smoking": 0.25})
    _check("report.abnormal_no_fabrication",
           t == SPEECH_TEXTS["extinguisher_missing"]
           and "吸烟" not in t and "火灾" not in t, t)

    # UNCERTAIN：点名未能确认的期望目标
    r = PoiInspection(status=InspectionStatus.UNCERTAIN, poi="P4",
                      detections=[det("fire_extinguisher", 0.3)])
    t = build_speech_text("P4", r, expect_classes=expect,
                          abnormal_classes=abnormal, min_conf=0.65)
    _check("report.uncertain_missing",
           "不确定" in t and "安全指示牌" in t and "人工复核" in t, t)

    # 一切非正常态：不出现"正常/未发现异常"，且引导人工
    for st in (InspectionStatus.UNCERTAIN, InspectionStatus.TIMEOUT,
               InspectionStatus.NO_FRAME, InspectionStatus.NO_MODEL,
               InspectionStatus.ERROR):
        t = build_speech_text("P5", PoiInspection(status=st, poi="P5"),
                              expect_classes=expect,
                              abnormal_classes=abnormal)
        _check(f"report.{st.value}_not_normal",
               "未发现异常" not in t and "检查完成，" not in t
               and ("人工" in t), t)


def test_vis_conf_by_class():
    # 按类阈值：smoking 0.3 在双阈值(smoking=0.25)下应判异常
    def det(name, conf):
        return {"class_name": name, "confidence": conf,
                "xyxy": [0, 0, 5, 5]}

    ins = _MockYoloInspector(
        frame=_FAKE_FRAME, conf_by_class={"smoking": 0.25}, min_conf=0.65,
        detect_fn=lambda rgb: {"detections": [det("smoking", 0.3)],
                               "inference_ms": 1.0})
    r = ins.inspect("P")
    _check("vis.per_class_smoking_abnormal",
           r.status is InspectionStatus.ABNORMAL, r.status.value)

    # 其他类 0.55 在 min_conf=0.65 下不达标 → 无效检出 → ABNORMAL（不是 SUCCESS）
    ins2 = _MockYoloInspector(
        frame=_FAKE_FRAME, conf_by_class={"smoking": 0.25}, min_conf=0.65,
        detect_fn=lambda rgb: {"detections": [det("fire_extinguisher", 0.55)],
                               "inference_ms": 1.0})
    r2 = ins2.inspect("P")
    _check("vis.per_class_other_stricter_abnormal",
           r2.status is InspectionStatus.ABNORMAL, r2.status.value)

    # 缺省（无 conf_by_class）：0.55 ≥ 0.5 维持原行为 → SUCCESS
    ins3 = _MockYoloInspector(
        frame=_FAKE_FRAME,
        detect_fn=lambda rgb: {"detections": [det("fire_extinguisher", 0.55)],
                               "inference_ms": 1.0})
    r3 = ins3.inspect("P")
    _check("vis.default_threshold_unchanged",
           r3.status is InspectionStatus.SUCCESS, r3.status.value)


def test_vis_to_bgr():
    # to_bgr=True：喂给模型的帧应是 RGB→BGR 翻转后的连续数组
    import numpy as np

    rgb = np.zeros((2, 3, 3), dtype=np.uint8)
    rgb[..., 0] = 200  # R 通道标记
    rgb[..., 2] = 7
    captured = {}

    def detect_fn(frame):
        captured["frame"] = frame
        return {"detections": [], "inference_ms": 1.0}

    ins = _MockYoloInspector(frame={"rgb": rgb}, detect_fn=detect_fn,
                             to_bgr=True)
    ins.inspect("P")
    fed = captured["frame"]
    # 原帧 R=200/G=0/B=7 → 翻转后 fed[...,0]=7（原B）, fed[...,2]=200（原R）
    _check("vis.to_bgr_flip_exact",
           int(fed[0, 0, 0]) == 7 and int(fed[0, 0, 2]) == 200
           and int(fed[0, 0, 1]) == 0 and fed.flags["C_CONTIGUOUS"],
           f"fed[0,0]={fed[0, 0].tolist()}")

    # to_bgr=False（默认）：帧对象原样传递
    captured2 = {}
    ins2 = _MockYoloInspector(frame={"rgb": rgb}, detect_fn=lambda f: (
        captured2.setdefault("frame", f),
        {"detections": [], "inference_ms": 1.0})[1])
    ins2.inspect("P")
    _check("vis.to_bgr_off_passthrough", captured2["frame"] is rgb)


# ── 手动单点闭环 run_manual_inspection：纯离线 ─────────────────────

def _load_manual_mod():
    src = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    path = os.path.join(src, "chassis", "run_manual_inspection.py")
    spec = importlib.util.spec_from_file_location("run_manual_mod", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class _ManualArgs:
    expect = ["safety_sign", "fire_extinguisher"]
    abnormal = ["smoking"]
    conf_smoking = 0.25
    conf_other = 0.65


def _run_manual(inputs, scenario="normal", poi_focus=None):
    mod = _load_manual_mod()
    vision, spk = mod._build_sim(_FakeManualScenario(scenario))
    record_path = os.path.join(tempfile.mkdtemp(prefix="manual_test_"),
                               f"{scenario}.jsonl")
    rec = InspectionRecorder(record_path, meta={"mode": "test",
                                                "simulated": True})
    it = iter(inputs)

    def input_fn(_prompt):
        v = next(it)
        if isinstance(v, BaseException):
            raise v
        return v

    rc = mod.run_loop(vision, spk, rec, _ManualArgs(),
                      conf_by_class={"smoking": 0.25},
                      input_fn=input_fn, output_fn=lambda *a, **k: None,
                      poi_focus_classes=poi_focus)
    with open(record_path, encoding="utf-8") as fp:
        records = [json.loads(line) for line in fp if line.strip()]
    return rc, records, spk


class _FakeManualScenario:
    def __init__(self, scenario):
        self.scenario = scenario
        self.expect = ["safety_sign", "fire_extinguisher"]


def test_manual_loop_basic():
    rc, records, spk = _run_manual(["货架A", "q"])
    pois = _poi_records(records)
    _check("manual.basic_rc0", rc == 0, f"rc={rc}")
    _check("manual.basic_one_poi", len(pois) == 1, str(len(pois)))
    p = pois[0]
    _check("manual.basic_fields",
           p["poi"] == "货架A" and p["final"] == "done"
           and p["inspection"]["status"] == "success"
           and p["speech"]["report"]["state"] == "done_observed",
           json.dumps(p, ensure_ascii=False)[:200])
    _check("manual.basic_text_accurate",
           "灭火器" in p["speech"]["report"]["text"]
           and "火灾" not in p["speech"]["report"]["text"],
           p["speech"]["report"]["text"])


def test_manual_loop_speech_fail_not_blocking():
    rc, records, spk = _run_manual(["P1", "P2", "q"], scenario="speech_fail")
    pois = _poi_records(records)
    _check("manual.spk_fail_rc0", rc == 0, f"rc={rc}")
    _check("manual.spk_fail_two_pois", len(pois) == 2, str(len(pois)))
    _check("manual.spk_fail_vision_kept",
           all(p["inspection"]["status"] == "success" for p in pois)
           and all(p["speech"]["report"]["state"] == "submit_failed"
                   for p in pois)
           and all(p["final"] == "done" for p in pois),
           json.dumps([p["speech"]["report"]["state"] for p in pois]))


def test_manual_loop_eof_and_interrupt():
    rc_eof, records_eof, _ = _run_manual([EOFError()])
    summaries = [r for r in records_eof if r.get("record_type") == "summary"]
    _check("manual.eof_clean", rc_eof == 0
           and summaries and summaries[0]["final_state"] == "done",
           f"rc={rc_eof}")

    rc_ki, records_ki, _ = _run_manual([KeyboardInterrupt()])
    summaries_ki = [r for r in records_ki
                    if r.get("record_type") == "summary"]
    _check("manual.ki_aborted", rc_ki == 1
           and summaries_ki and summaries_ki[0]["final_state"] == "aborted",
           f"rc={rc_ki}")


def test_manual_gate():
    mod = _load_manual_mod()
    env = mod.REAL_CONFIRM_ENV
    old = _save_env(env)
    try:
        _check("manual.gate_eof_reject",
               mod._confirm_real(_ManualRealArgs(), input_fn=_eof_input)
               is False)
        os.environ[env] = "yes"
        _check("manual.gate_wrong_env_reject",
               mod._confirm_real(_ManualRealArgs(), input_fn=_eof_input)
               is False)
        os.environ.pop(env, None)
        _check("manual.gate_yes_accept",
               mod._confirm_real(_ManualRealArgs(),
                                 input_fn=lambda _p: "YES") is True)
        os.environ[env] = "1"
        _check("manual.gate_env_accept",
               mod._confirm_real(_ManualRealArgs(), input_fn=_eof_input)
               is True)
    finally:
        _restore_env(env, old)


class _ManualRealArgs:
    model = "/nonexistent/best.pt"
    expect = ["safety_sign"]
    abnormal = ["smoking"]
    conf_smoking = 0.25
    conf_other = 0.65
    record = "/tmp/manual_mock_never_written.jsonl"


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
    test_vis_extinguisher_presence_rule()
    test_vis_timeout_controller_policy()
    print("=== TTS 语义与完成判定 Mock 测试 ===")
    test_tts_interrupt_calls_stop_then_append()
    test_tts_no_interrupt_only_append()
    test_tts_bounce_no_premature_done()
    test_tts_settle_before_done()
    test_tts_settle_not_reached_no_done()
    test_tts_never_playing_not_faked_done()
    test_tts_append_call_exception()
    test_tts_append_success_false()
    test_tts_query_error()
    test_tts_always_playing_hard_timeout()
    test_tts_stop_failure_still_appends()
    test_tts_exact_playing_match()
    print("=== TTS 结构化日志 Mock 测试 ===")
    test_tts_log_done_path()
    test_tts_log_no_full_text()
    test_tts_log_distinct_failures()
    test_tts_log_req_ids_unique()
    test_tts_log_query_raw_recorded()
    test_tts_log_interrupt_order_unchanged()
    print("=== 文案映射与单点闭环 Mock 测试 ===")
    test_report_texts()
    test_vis_conf_by_class()
    test_report_extinguisher_presence_rule()
    test_fixed_speech_texts()
    test_report_safety_sign_texts()
    test_poi_focus_parse()
    test_controller_fixed_speech_routing()
    test_controller_safety_sign_focus_speech()
    test_vis_to_bgr()
    test_manual_loop_basic()
    test_manual_loop_speech_fail_not_blocking()
    test_manual_loop_speech_wiring()
    test_manual_safety_sign_focus_speech()
    test_manual_loop_eof_and_interrupt()
    test_manual_gate()
    failed = [r for r in _results if r[1] == FAIL]
    print(f"\n=== 结果: {len(_results) - len(failed)}/{len(_results)} 通过 ===")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
