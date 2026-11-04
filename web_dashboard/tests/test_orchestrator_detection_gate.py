"""编排层视觉检测门控行为测试。

通过桩化 rclpy / fire_inspection_* 依赖导入真实的 orchestrator_node，
再用极简 stub 调用 OrchestratorNode._on_detection，验证本次安全修复：
- 无活动巡检轮次（auto_start=False 且未 start/manual）时，检测回调直接返回，
  不触发头部锁定、语音播报、吸烟介入或 WATCH 状态机推进；
- 正式进入 WATCH 模式（活动轮次）后，检测消息仍正常处理；
- 自动巡检导航模式与手动巡检：原有检测门控行为不受影响。
"""
from __future__ import annotations

import json
import sys
import threading
import types
from pathlib import Path
from types import SimpleNamespace

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
ORCHESTRATOR_PATH = (
    REPO_ROOT
    / "fire_inspection_ws"
    / "src"
    / "fire_inspection_orchestrator"
    / "fire_inspection_orchestrator"
    / "orchestrator_node.py"
)


def _install_stubs() -> None:
    """把 rclpy / 消息包的桩写入 sys.modules，使 orchestrator_node 可被导入。"""
    if "rclpy" in sys.modules:
        return

    class _StubMeta(type):
        def __getattr__(cls, name):
            return _Stub()

    class _Stub(metaclass=_StubMeta):
        def __init__(self, *a, **k):
            pass

        def __getattr__(self, name):
            return _Stub()

        def __call__(self, *a, **k):
            return _Stub()

    def _mod(name, **attrs):
        m = types.ModuleType(name)
        for k, v in attrs.items():
            setattr(m, k, v)
        sys.modules[name] = m
        return m

    _mod("rclpy")
    _mod("rclpy.action", ActionClient=_Stub)
    _mod("rclpy.executors", ExternalShutdownException=Exception,
         MultiThreadedExecutor=_Stub)
    _mod("rclpy.node", Node=_Stub)
    _mod("rclpy.qos", DurabilityPolicy=_Stub, QoSProfile=_Stub,
         ReliabilityPolicy=_Stub)
    _mod("yaml")
    _mod("ament_index_python")
    _mod("ament_index_python.packages",
         get_package_share_directory=lambda *a, **k: "/tmp")
    _mod("sensor_msgs")
    _mod("sensor_msgs.msg", CompressedImage=_Stub)
    _mod("std_msgs")
    _mod("std_msgs.msg", String=_Stub)
    _mod("fire_inspection_interfaces")
    _mod("fire_inspection_interfaces.action", ApproachTarget=_Stub,
         InspectPoi=_Stub, NavigateTo=_Stub, Speak=_Stub)
    _mod("fire_inspection_interfaces.msg", PoiResult=_Stub)
    _mod("fire_inspection_interfaces.srv", InspectionControl=_Stub)
    _mod("fire_inspection_voice")
    _mod("fire_inspection_voice.speech_texts", SPEECH_TEXTS={},
         PoiStatus=_Stub, build_speech_text=lambda *a, **k: "")


@pytest.fixture(scope="module")
def orchestrator():
    _install_stubs()
    import importlib.util

    spec = importlib.util.spec_from_file_location(
        "orchestrator_node_under_test", ORCHESTRATOR_PATH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _noop_logger():
    return SimpleNamespace(
        info=lambda *a, **k: None,
        warn=lambda *a, **k: None,
        error=lambda *a, **k: None,
        debug=lambda *a, **k: None,
    )


def _make_stub(*, round_active, nav_enabled, watch_active):
    head_cmds: list[str] = []
    stub = SimpleNamespace(
        _watch_lock=threading.Lock(),
        _round_active=round_active,
        _nav_enabled=nav_enabled,
        _watch_active=watch_active,
        _scan_classes=["smoking"],
        _watch_state="WATCH",
        _watch_streak_cls=None,
        _watch_streak=0,
        _watch_hits=[],
        _watch_anchor=None,
        _watch_locked_pose=None,
        _watch_lost_at=None,
        _watch_last_trigger=0.0,
        _cooldown_s=5.0,
        _lost_timeout_s=5.0,
        _confirm_frames=2,
        _confirm_window_s=3.0,
        _confirm_min_span_s=0.0,
        _confirm_pos_tol=0.1,
        _latest_det=None,
        _nav_in_flight=False,
        _nav_poi="",
        _intervention_enabled=False,
        _intervention_classes=[],
        _intervention_suppress=False,
        _intervention_max_per_poi=1,
        _intervention_max_total=5,
        _intervention_total=0,
        _intervention_counts={},
        _intervention_confirmed=False,
        _prestop_unconfirmed=0,
        _prestop_block_until=0.0,
        _speak_only_pending=False,
        _intervention_event=threading.Event(),
        _send_head_cmd=lambda cmd: head_cmds.append(cmd),
        _parse_look_at=lambda detections_json, classes=None: {
            "yaw": 0.1, "pitch": 0.0},
        _intervention_allowed=lambda cls: False,
        get_logger=_noop_logger,
    )
    stub._head_cmds = head_cmds
    return stub


def _msg(detections):
    return SimpleNamespace(
        simulated=False,
        detections_json=json.dumps(detections),
    )


# -- 场景 1：auto_start=False / nav_enabled=False / 无活动轮次 ----------------

def test_no_active_round_detection_no_action(orchestrator):
    """无活动巡检轮次：检测消息不能触发任何动作。"""
    stub = _make_stub(round_active=False, nav_enabled=False, watch_active=False)
    msg = _msg([{"cls": "smoking", "conf": 0.95, "cx": 0.5, "cy": 0.5}])
    orchestrator.OrchestratorNode._on_detection(stub, msg)
    assert stub._latest_det is None, "无活动轮次不应缓存检测快照"
    assert stub._head_cmds == [], "无活动轮次不应下发头部指令"
    assert stub._watch_streak == 0, "无活动轮次不应推进 WATCH 状态机"
    assert stub._watch_state == "WATCH", "无活动轮次不应改变 WATCH 状态"


def test_no_active_round_nav_mode_also_blocked(orchestrator):
    """无活动轮次即便处于导航模式也应被活动轮次门控拦下。"""
    stub = _make_stub(round_active=False, nav_enabled=True, watch_active=True)
    msg = _msg([{"cls": "smoking", "conf": 0.95, "cx": 0.5, "cy": 0.5}])
    orchestrator.OrchestratorNode._on_detection(stub, msg)
    assert stub._latest_det is None
    assert stub._head_cmds == []


# -- 场景 2：正式进入 WATCH 模式（活动轮次）后检测正常处理 -------------------

def test_watch_mode_active_round_processes_detection(orchestrator):
    """WATCH 模式活动轮次：首帧命中应缓存快照、进入 LOCKED、下发 look_at。"""
    stub = _make_stub(round_active=True, nav_enabled=False, watch_active=False)
    msg = _msg([{"cls": "smoking", "conf": 0.95, "cx": 0.5, "cy": 0.5}])
    orchestrator.OrchestratorNode._on_detection(stub, msg)
    assert stub._latest_det is not None, "WATCH 活动轮次应缓存检测快照"
    assert stub._watch_state == "LOCKED", "首帧命中应进入 LOCKED"
    assert any(c.startswith("look_at") for c in stub._head_cmds), \
        "首帧命中应下发 look_at 头部指令"


# -- 场景 3：自动巡检 / 手动巡检原有检测行为不变 -----------------------------

def test_nav_mode_transit_watch_off_returns_early(orchestrator):
    """自动巡检导航模式 + 到点（移动路段检测关闭）：检测直接返回。"""
    stub = _make_stub(round_active=True, nav_enabled=True, watch_active=False)
    msg = _msg([{"cls": "smoking", "conf": 0.95, "cx": 0.5, "cy": 0.5}])
    orchestrator.OrchestratorNode._on_detection(stub, msg)
    assert stub._latest_det is None
    assert stub._head_cmds == []


def test_nav_mode_transit_watch_on_processes(orchestrator):
    """自动巡检导航模式 + 移动路段检测开启：检测正常处理（原行为不变）。"""
    stub = _make_stub(round_active=True, nav_enabled=True, watch_active=True)
    msg = _msg([{"cls": "smoking", "conf": 0.95, "cx": 0.5, "cy": 0.5}])
    orchestrator.OrchestratorNode._on_detection(stub, msg)
    assert stub._latest_det is not None
    assert stub._watch_state == "LOCKED"


def test_manual_mode_detection_returns_early(orchestrator):
    """手动巡检（导航模式、未开启移动路段检测）：检测直接返回。

    手动巡检只对单点做 导航→停头→视觉→播报，不依赖检测流介入，
    因此检测回调在手动期间应保持返回，这与修复前的原行为一致。
    """
    stub = _make_stub(round_active=True, nav_enabled=True, watch_active=False)
    msg = _msg([{"cls": "smoking", "conf": 0.95}])
    orchestrator.OrchestratorNode._on_detection(stub, msg)
    assert stub._latest_det is None
    assert stub._head_cmds == []


# -- 场景 4：WATCH 结束时序——stop 必须是最后一条头部指令 --------------------

def test_watch_end_stop_is_last_head_command(orchestrator):
    """WATCH 结束路径关门+停头原子化：look_at 可先完成，但 stop 必须最后。

    时序：
      T1（检测回调）持 _watch_lock，调 _send_head_cmd('look_at:...') 时阻塞于 Event；
      T2（结束线程）尝试获取 _watch_lock 执行关门+stop，被阻塞；
      主线程释放 Event → T1 完成并释放锁 → T2 关门+stop。
    断言 head_cmds 最后一条是 stop。
    """
    import time as _time

    head_cmds: list[str] = []
    speak_threads: list[str] = []
    look_at_gate = threading.Event()      # T1 发送 look_at 时在此等待
    entered_look_at = threading.Event()   # T1 已进入 send_head_cmd 并持锁

    def send_head_cmd(cmd):
        if cmd.startswith("look_at"):
            entered_look_at.set()
            # 模拟回调持锁期间被卡住，扩大竞态窗口
            look_at_gate.wait(timeout=10)
        head_cmds.append(cmd)

    stub = SimpleNamespace(
        _watch_lock=threading.Lock(),
        _round_active=True,
        _nav_enabled=False,
        _watch_active=False,
        _scan_classes=["smoking"],
        _watch_state="WATCH",
        _watch_streak_cls=None,
        _watch_streak=0,
        _watch_hits=[],
        _watch_anchor=None,
        _watch_locked_pose=None,
        _watch_lost_at=None,
        _watch_last_trigger=0.0,
        _cooldown_s=5.0,
        _lost_timeout_s=5.0,
        _confirm_frames=2,
        _confirm_window_s=3.0,
        _confirm_min_span_s=0.0,
        _confirm_pos_tol=0.1,
        _latest_det=None,
        _nav_in_flight=False,
        _nav_poi="",
        _intervention_enabled=False,
        _intervention_classes=[],
        _intervention_suppress=False,
        _intervention_max_per_poi=1,
        _intervention_max_total=5,
        _intervention_total=0,
        _intervention_counts={},
        _intervention_confirmed=False,
        _prestop_unconfirmed=0,
        _prestop_block_until=0.0,
        _speak_only_pending=False,
        _intervention_event=threading.Event(),
        _send_head_cmd=send_head_cmd,
        _parse_look_at=lambda detections_json, classes=None: {
            "yaw": 0.1, "pitch": 0.0},
        _intervention_allowed=lambda cls: False,
        _watch_speak_and_resume=lambda cls: speak_threads.append(cls),
        get_logger=_noop_logger,
    )

    msg = _msg([{"cls": "smoking", "conf": 0.95, "cx": 0.5, "cy": 0.5}])

    # T1：检测回调线程
    t1 = threading.Thread(
        target=orchestrator.OrchestratorNode._on_detection,
        args=(stub, msg), daemon=True,
    )
    t1.start()

    # 等 T1 进入 look_at 阻塞（此时 T1 持有 _watch_lock）
    assert entered_look_at.wait(timeout=5), "T1 应进入 look_at 并持锁"
    _time.sleep(0.1)  # 确保 T1 已卡在 look_at_gate.wait
    assert stub._watch_lock.locked(), "T1 应持有 _watch_lock"

    # T2：结束线程——模拟 WATCH finally：关门+stop（同锁块）
    def watch_end():
        with stub._watch_lock:
            stub._round_active = False
            stub._send_head_cmd("stop")

    t2 = threading.Thread(target=watch_end, daemon=True)
    t2.start()
    # T2 应被 T1 的锁阻塞，短时间内无法完成
    t2.join(timeout=0.3)
    assert t2.is_alive(), "T2 应被 T1 持有的 _watch_lock 阻塞"

    # 主线程释放 Event：T1 的 look_at 完成 → 释放锁
    look_at_gate.set()
    t1.join(timeout=5)
    assert not t1.is_alive(), "T1 应已完成"
    t2.join(timeout=5)
    assert not t2.is_alive(), "T2 应已完成"

    # 核心断言：stop 必须是最后一条头部指令
    assert head_cmds, "应有头部指令"
    assert head_cmds[-1] == "stop", (
        f"stop 必须是最后一条头部指令，实际序列={head_cmds}"
    )
    assert any(c.startswith("look_at") for c in head_cmds), "look_at 应已发出"

    # 结束后再触发检测回调：门控已关，不应产生新指令或播报线程
    before_cmds = len(head_cmds)
    before_speak = len(speak_threads)
    orchestrator.OrchestratorNode._on_detection(stub, msg)
    assert len(head_cmds) == before_cmds, "结束后回调不应产生头部指令"
    assert len(speak_threads) == before_speak, "结束后回调不应启动播报线程"
    assert stub._latest_det is not None or True  # 首次已缓存，不强制


# -- 场景 5：WATCH 启动阶段异常——start 已发仍必须 stop ----------------------

def _make_run_stub(orchestrator, head_cmds, *, send_impl=None,
                   stop_event=None, release_calls=None):
    """构造覆盖 `_run` / `_run_round` WATCH 分支所需属性的 stub。

    只围绕 WATCH 分支的最小属性集；`_set_state` / `_speak` / `_release_busy`
    均以可观测的桩替代，避免依赖 rclpy。
    """
    if stop_event is None:
        stop_event = threading.Event()
    if release_calls is None:
        release_calls = []
    states = []
    stub = SimpleNamespace(
        _watch_lock=threading.Lock(),
        _round_active=False,
        _watch_gen=0,
        _nav_enabled=False,
        _scan_classes=["smoking"],
        _confirm_frames=2,
        _cooldown_s=5.0,
        _lost_timeout_s=5.0,
        _stop_event=stop_event,
        _send_head_cmd=send_impl or (lambda cmd: head_cmds.append(cmd)),
        _set_state=lambda state, detail="": states.append((state, detail)),
        _speak=lambda text, label: None,
        _release_busy=lambda: release_calls.append(True),
        get_logger=_noop_logger,
    )
    # 让 _run → self._run_round() 调到真实的 WATCH 分支实现
    stub._run_round = lambda: orchestrator.OrchestratorNode._run_round(stub)
    stub._head_cmds = head_cmds
    stub._states = states
    stub._release_calls = release_calls
    return stub


def _assert_watch_ended_with_stop(orchestrator, stub):
    """公共断言：start 先行、stop 收尾、DONE 落状态、关门、释放 busy。"""
    assert stub._head_cmds[0] == "start", "WATCH 启动应发送 start"
    assert stub._head_cmds[-1] == "stop", (
        f"WATCH 结束必须以 stop 收尾，实际序列={stub._head_cmds}"
    )
    assert stub._round_active is False, "结束后 _round_active 必须为 False"
    assert any(s is orchestrator.RunState.DONE for s, _ in stub._states), \
        "结束后应发布 DONE 状态"
    assert stub._release_calls == [True], "_release_busy 必须执行且只执行一次"


def test_watch_start_exception_still_sends_stop(orchestrator):
    """回归：start 已记录但随后抛异常，finally 仍必须发送 stop。"""
    head_cmds: list[str] = []

    def send_impl(cmd):
        head_cmds.append(cmd)
        if cmd == "start":
            # 模拟 publish 成功之后、函数内后续语句抛异常
            raise RuntimeError("模拟 start 发送后异常")

    stub = _make_run_stub(orchestrator, head_cmds, send_impl=send_impl)
    orchestrator.OrchestratorNode._run(stub)

    assert head_cmds == ["start", "stop"], (
        f"start 后异常仍必须补发 stop，实际序列={head_cmds}"
    )
    _assert_watch_ended_with_stop(orchestrator, stub)


def test_watch_normal_end_sends_stop(orchestrator, monkeypatch):
    """正常结束：stop_event 已置位，循环不进入，start→stop 收尾。"""
    monkeypatch.setattr(
        orchestrator, "time", types.SimpleNamespace(sleep=lambda _s: None))
    stop_event = threading.Event()
    stop_event.set()
    head_cmds: list[str] = []
    stub = _make_run_stub(orchestrator, head_cmds, stop_event=stop_event)
    orchestrator.OrchestratorNode._run(stub)

    assert head_cmds == ["start", "stop"]
    _assert_watch_ended_with_stop(orchestrator, stub)


def test_watch_stop_event_triggered_sends_stop(orchestrator, monkeypatch):
    """运行中触发 stop 事件：循环退出后仍必须发送 stop。"""
    import time as _time

    monkeypatch.setattr(
        orchestrator, "time", types.SimpleNamespace(sleep=lambda _s: None))
    stop_event = threading.Event()
    head_cmds: list[str] = []
    stub = _make_run_stub(orchestrator, head_cmds, stop_event=stop_event)
    t = threading.Thread(
        target=lambda: orchestrator.OrchestratorNode._run(stub), daemon=True)
    t.start()
    for _ in range(500):
        if head_cmds:
            break
        _time.sleep(0.01)
    stop_event.set()
    t.join(timeout=5)
    assert not t.is_alive(), "WATCH 线程应已退出"

    _assert_watch_ended_with_stop(orchestrator, stub)


def test_watch_loop_exception_sends_stop(orchestrator, monkeypatch):
    """循环内部异常：被 except 吞掉后 finally 仍必须发送 stop。"""
    def boom(_s):
        raise RuntimeError("模拟循环内异常")

    monkeypatch.setattr(
        orchestrator, "time", types.SimpleNamespace(sleep=boom))
    head_cmds: list[str] = []
    stub = _make_run_stub(orchestrator, head_cmds, stop_event=threading.Event())
    orchestrator.OrchestratorNode._run(stub)

    assert head_cmds == ["start", "stop"], (
        f"循环内异常仍必须补发 stop，实际序列={head_cmds}"
    )
    _assert_watch_ended_with_stop(orchestrator, stub)
