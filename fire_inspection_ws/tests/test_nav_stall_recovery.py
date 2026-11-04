#!/usr/bin/env python3
"""受阻自恢复（stall recovery）离线测试。

覆盖：
- load_stall_config 配置校验（未知键 / 非法值 / 后退速度不得为 0）
- _run_chassis_action：停滞 → 取消 → 后退 → 重发 → 仍停滞返回 blocked
- 正常前进不会被误判为受阻；原地转向（detect_stall=False）不参与停滞检测
- _backoff 后退距离按底盘 move_by 的 [50, 5000] mm 限幅，速度经 v 参数限制
- _blocked 推进点位顺序，否则编排层跳过本点后后续 goal 会被顺序校验拒绝
- 编排层 _navigate_inspect：blocked 跳过并播报，其他失败重试一次，stop 原样抛出

纯 stdlib + object.__new__ + 假时钟：不连底盘、不 rclpy.init、不触 TTS。
运行前需已 source ROS 环境与工作区 install：
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    python3 tests/test_nav_stall_recovery.py
"""

from __future__ import annotations

import math
import os
import sys
import threading
from contextlib import contextmanager
from types import SimpleNamespace

_HERE = os.path.dirname(os.path.abspath(__file__))
_WS = os.path.normpath(os.path.join(_HERE, ".."))
_SRC = os.path.join(_WS, "src")
for sub in ("fire_inspection_nav", "fire_inspection_orchestrator",
            "fire_inspection_voice"):
    p = os.path.join(_SRC, sub)
    if p not in sys.path:
        sys.path.insert(0, p)

import fire_inspection_nav.navigate_to_server as nav  # noqa: E402
import fire_inspection_orchestrator.orchestrator_node as orch_mod  # noqa: E402

TESTS = []


def case(fn):
    TESTS.append(fn)
    return fn


class _Logger:
    def __init__(self):
        self.messages = []

    def _add(self, level, msg):
        self.messages.append((level, str(msg)))

    def info(self, msg):
        self._add("info", msg)

    def warn(self, msg):
        self._add("warn", msg)

    def error(self, msg):
        self._add("error", msg)


class _Clock:
    """虚拟时钟：sleep 直接推进时间，测试不用真等。"""

    def __init__(self):
        self.t = 0.0

    def monotonic(self):
        return self.t

    def sleep(self, seconds):
        self.t += max(0.0, seconds)


@contextmanager
def _fake_time():
    clock = _Clock()
    original = nav.time
    nav.time = SimpleNamespace(monotonic=clock.monotonic, sleep=clock.sleep)
    try:
        yield clock
    finally:
        nav.time = original


class FakeChassis:
    """脚本化底盘：动作状态 / 位姿 / 后退 / 侧向绕行动作全部可控，不连网络。"""

    def __init__(self, pose_steps=None, status="running", success_after=None,
                 backoff_status="succeeded", detour_status="succeeded",
                 rotate_status="succeeded"):
        self.pose_steps = list(pose_steps or [])
        self.pose_calls = 0
        self.status = status
        self.status_calls = 0
        self.success_after = success_after
        self.backoff_status = backoff_status
        self.detour_status = detour_status
        self.rotate_status = rotate_status
        self.cancelled = []
        self.move_by_calls = []
        self.move_to_calls = []

    def get_pose(self):
        if self.pose_steps:
            idx = min(self.pose_calls, len(self.pose_steps) - 1)
            self.pose_calls += 1
            return dict(self.pose_steps[idx])
        return {"x": 0.0, "y": 0.0, "yaw": 0.0}

    def get_action_status(self, action_id):
        if action_id == "back-1":
            return {"status": self.backoff_status}
        if action_id == "rotate-1":
            return {"status": self.rotate_status}
        if action_id == "detour-1":
            return {"status": self.detour_status}
        self.status_calls += 1
        if self.success_after is not None and self.status_calls >= self.success_after:
            return {"status": "succeeded"}
        return {"status": self.status}

    def cancel_action(self, action_list=None):
        self.cancelled.append(action_list)
        return {}

    def move_by(self, direction=0.0, distance=0, move_mode=0, v=0.0, w=0.0):
        self.move_by_calls.append({
            "direction": direction, "distance": distance,
            "move_mode": move_mode, "v": v, "w": w,
        })
        # 平移=后退、旋转=脱困，用不同 action_id 区分两段独立动作
        return {"action_id": "rotate-1" if move_mode == 1 else "back-1"}

    def move_to(self, x, y, yaw=0.0, flags=0):
        self.move_to_calls.append({"x": x, "y": y, "yaw": yaw, "flags": flags})
        return {"action_id": "detour-1"}

    def get_robot_health(self):
        return SimpleNamespace(hasWarning=False, hasError=False,
                               hasFatal=False, baseError=[])


def _make_node(chassis, **overrides):
    node = object.__new__(nav.NavigateToServer)
    node.__dict__.update({
        "_chassis": chassis,
        "_pois": {"POI_01": {"x": 5.0, "y": 0.0, "yaw": 0.0}},
        "_stall_enabled": True,
        "_stall_window_s": 1.0,
        "_stall_min_progress_m": 0.05,
        "_stall_backoff_m": 0.5,
        "_stall_backoff_speed_mps": 0.2,
        "_stall_backoff_timeout_s": 2.0,
        "_stall_max_recoveries": 1,
        "_stall_detour_offsets_m": [1.0, 1.5],
        "_stall_detour_timeout_s": 5.0,
        "_stall_rotate_deg": 45.0,
        "_stall_rotate_speed": 0.30,
        "_stall_rotate_timeout_s": 8.0,
        "_stall_rotate_turns": 0,
        "_goal_detoured": False,
        "_action_poll_interval_s": 0.5,
        "_arrival_tolerance_m": 0.5,
        "_range_limit": {},
        "_corridor": {},
        "_shutting_down": False,
        "_seq_lock": threading.Lock(),
        "_start_poi": "start_point",
        "_poi_order": ["POI_01", "POI_02"],
        "_started": True,
        "_next_index": 0,
        "_returned": False,
        "_transit_pois": set(),
        "get_logger": lambda: _Logger(),
        "_publish_feedback": lambda *a, **k: None,
    })
    node.__dict__.update(overrides)
    return node


def _goal(is_cancel_requested=False, is_return=False):
    return SimpleNamespace(
        is_cancel_requested=is_cancel_requested,
        request=SimpleNamespace(is_return=is_return),
        abort=lambda: None,
    )


# ── 导航方式选择 ──────────────────────────────────────────────────

@case
def test_select_motion_mode_use_point_overrides_track():
    """介入后重发要求点对点：巡检点也必须走 point（不再上轨）。"""
    assert nav.select_motion_mode(
        "POI_02", False, True, "start_point", {"POI_04"}) == "point"
    # 未要求时巡检点仍沿轨道
    assert nav.select_motion_mode(
        "POI_02", False, False, "start_point", {"POI_04"}) == "track"
    # 起点/过点本来就是点对点
    assert nav.select_motion_mode(
        "start_point", False, False, "start_point", set()) == "point"
    assert nav.select_motion_mode(
        "POI_04", False, False, "start_point", {"POI_04"}) == "point"
    # 返程必须走返程轨道，即使误传 use_point 也不得点对点
    assert nav.select_motion_mode(
        "start_point", True, True, "start_point", set()) == "track"


# ── 配置校验 ──────────────────────────────────────────────────────

@case
def test_load_stall_config_defaults():
    cfg = nav.load_stall_config({"stall": {
        "enabled": True, "detect_window_s": 8.0, "min_progress_m": 0.10,
        "backoff_m": 0.5, "backoff_speed_mps": 0.2,
        "backoff_timeout_s": 5.0, "max_recoveries": 1,
    }})
    assert cfg["enabled"] is True
    assert cfg["backoff_m"] == 0.5
    # 绕行参数的默认值（现场未显式配置时也要能真正侧移）
    assert cfg["detour_offsets_m"] == [1.0, 1.5], cfg["detour_offsets_m"]
    assert cfg["detour_timeout_s"] == 30.0, cfg["detour_timeout_s"]
    # 后退失败时的脱困转向默认值
    assert cfg["retreat_rotate_deg"] == 45.0, cfg["retreat_rotate_deg"]
    assert cfg["retreat_rotate_speed_rad_s"] == 0.30, cfg
    assert cfg["retreat_rotate_timeout_s"] == 8.0, cfg
    # 配置缺失时默认开启（工具链升级后不静默关掉安全特性）
    assert nav.load_stall_config({})["enabled"] is True
    assert nav.load_stall_config({})["max_recoveries"] == 2


@case
def test_load_stall_config_rejects_unknown_key():
    try:
        nav.load_stall_config({"stall": {"detect_seconds": 3}})
    except RuntimeError as exc:
        assert "未知键" in str(exc), exc
    else:
        raise AssertionError("未知键必须拒绝启动")


@case
def test_load_stall_config_rejects_illegal_values():
    bad = [
        {"detect_window_s": 0.5},      # 小于下限
        {"backoff_m": 9.0},            # 超过底盘最大位移
        {"backoff_speed_mps": 0.0},    # 0 在底盘接口里表示不限速
        {"backoff_speed_mps": 0.9},
        {"max_recoveries": -1},
        {"max_recoveries": 4},
        {"detour_offsets_m": []},          # 空列表无法绕行
        {"detour_offsets_m": [0.1]},       # 小于下限
        {"detour_offsets_m": [4.0]},       # 超过上限
        {"detour_offsets_m": [1.0, 1.5, 2.0, 2.5]},  # 最多 3 个
        {"detour_timeout_s": 1.0},         # 小于下限
        {"retreat_rotate_deg": 5.0},       # 小于下限
        {"retreat_rotate_speed_rad_s": 0.0},  # 0 表示不限速
        {"retreat_rotate_timeout_s": 1.0},    # 小于下限
    ]
    for item in bad:
        try:
            nav.load_stall_config({"stall": item})
        except RuntimeError:
            continue
        else:
            raise AssertionError(f"非法配置未拒绝: {item}")


# ── 停滞检测 ──────────────────────────────────────────────────────

@case
def test_stall_triggers_backoff_detour_then_blocked():
    """一直不动：后退 → 左侧绕行 → 点对点重发；仍不动则返回 blocked。"""
    chassis = FakeChassis()
    node = _make_node(chassis)
    redispatched = []

    def redispatch(point=False):
        redispatched.append(point)
        return "act-2"

    with _fake_time():
        outcome = nav.NavigateToServer._run_chassis_action(
            node, _goal(), "act-1", "POI_01", "track", math.inf, 60.0,
            redispatch=redispatch)

    assert outcome == "blocked", outcome
    # 绕行后已经离开轨道，重发必须走点对点
    assert redispatched == [True], redispatched
    assert node._goal_detoured is True
    assert len(chassis.move_by_calls) == 1, chassis.move_by_calls
    call = chassis.move_by_calls[0]
    assert call["distance"] == 500, call           # 0.5m -> 500mm
    assert call["move_mode"] == 0, call
    assert call["v"] == 0.2, call                  # 后退限速
    assert abs(abs(call["direction"]) - math.pi) < 1e-9, call
    # 位姿 (0,0) 朝 +x，目标 (5,0)：左侧绕行 = y + offset
    assert len(chassis.move_to_calls) == 1, chassis.move_to_calls
    detour = chassis.move_to_calls[0]
    assert abs(detour["x"] - 0.0) < 1e-9 and abs(detour["y"] - 1.0) < 1e-9, detour
    assert detour["flags"] == 0, detour


@case
def test_detour_both_sides_fail_gives_blocked_without_redispatch():
    """左右两侧都挪不出去：直接 blocked，不得重发原目标。"""
    chassis = FakeChassis(detour_status="failed")
    node = _make_node(chassis)
    redispatched = []

    def redispatch(point=False):
        redispatched.append(point)
        return "act-2"

    with _fake_time():
        outcome = nav.NavigateToServer._run_chassis_action(
            node, _goal(), "act-1", "POI_01", "track", math.inf, 60.0,
            redispatch=redispatch)

    assert outcome == "blocked", outcome
    assert redispatched == [], redispatched
    assert len(chassis.move_to_calls) == 2, chassis.move_to_calls
    assert chassis.move_to_calls[0]["y"] > 0.0, chassis.move_to_calls
    assert chassis.move_to_calls[1]["y"] < 0.0, chassis.move_to_calls


@case
def test_backoff_failure_rotates_then_redispatch_instead_of_blocked():
    """后退失败（后方也堵死）→ 先原地转向脱困并重发，不立刻判失败。"""
    chassis = FakeChassis(backoff_status="failed", rotate_status="succeeded")
    node = _make_node(chassis)
    redispatched = []

    def redispatch(point=False):
        redispatched.append(point)
        return "act-2"

    with _fake_time():
        outcome = nav.NavigateToServer._run_chassis_action(
            node, _goal(), "act-1", "POI_01", "track", math.inf, 60.0,
            redispatch=redispatch)

    # max_recoveries=1：第 1 次后退失败→转向→重发；再次停滞才 blocked
    assert outcome == "blocked", outcome
    assert len(chassis.move_by_calls) == 2, chassis.move_by_calls
    backoff, rotate = chassis.move_by_calls
    assert backoff["move_mode"] == 0 and backoff["distance"] == 500, backoff
    assert rotate["move_mode"] == 1, rotate
    assert rotate["distance"] == 0, rotate
    assert rotate["w"] == node._stall_rotate_speed, rotate
    assert abs(rotate["direction"] - math.radians(node._stall_rotate_deg)) < 1e-9, rotate
    # 原地转向没有离开轨道：重发仍按原方式，不走点对点
    assert redispatched == [False], redispatched
    # 没有做侧向绕行
    assert chassis.move_to_calls == [], chassis.move_to_calls


@case
def test_backoff_and_rotate_both_fail_gives_blocked():
    """后退和原地转向都失败 → 本段 blocked，不重发。"""
    chassis = FakeChassis(backoff_status="failed", rotate_status="failed")
    node = _make_node(chassis)
    redispatched = []

    def redispatch(point=False):
        redispatched.append(point)
        return "act-2"

    with _fake_time():
        outcome = nav.NavigateToServer._run_chassis_action(
            node, _goal(), "act-1", "POI_01", "track", math.inf, 60.0,
            redispatch=redispatch)

    assert outcome == "blocked", outcome
    assert redispatched == [], redispatched
    assert len(chassis.move_by_calls) == 2, chassis.move_by_calls


@case
def test_detour_respects_soft_range_limit():
    """绕行点越过应用层软限位时该侧直接放弃，不下发底盘动作。"""
    chassis = FakeChassis()
    node = _make_node(chassis, _range_limit={"y": [-0.5, 0.5]})
    with _fake_time():
        outcome = nav.NavigateToServer._detour(node, _goal(), "POI_01", 1.0, 1)
    assert outcome == "failed", outcome
    assert chassis.move_to_calls == [], chassis.move_to_calls


@case
def test_detour_right_side_geometry():
    """side=-1 向右侧（y 减小）平移，保持当前朝向。"""
    chassis = FakeChassis()
    node = _make_node(chassis)
    chassis.pose_steps = [{"x": 1.0, "y": 0.0, "yaw": 0.7}]
    with _fake_time():
        outcome = nav.NavigateToServer._detour(node, _goal(), "POI_01", 1.5, -1)
    assert outcome == "ok", outcome
    call = chassis.move_to_calls[0]
    assert abs(call["y"] - (-1.5)) < 1e-9, call
    assert abs(call["yaw"] - 0.7) < 1e-9, call


@case
def test_return_leg_detour_redispatches_track_not_point():
    """返程段绕过后仍走返程轨道（point=False）：跨全图点对点会规划失败。"""
    chassis = FakeChassis()
    node = _make_node(chassis)
    redispatched = []

    def redispatch(point=False):
        redispatched.append(point)
        return "act-2"

    with _fake_time():
        outcome = nav.NavigateToServer._run_chassis_action(
            node, _goal(is_return=True), "act-1", "POI_01", "track",
            math.inf, 60.0, redispatch=redispatch, is_return=True)

    assert outcome == "blocked", outcome
    assert redispatched == [False], redispatched
    assert node._goal_detoured is True
    # 绕行动作本身仍然生效（后退 + 侧移各一次）
    assert len(chassis.move_by_calls) == 1, chassis.move_by_calls
    assert len(chassis.move_to_calls) == 1, chassis.move_to_calls


@case
def test_moving_robot_is_not_blocked():
    """持续前进 + 动作成功：不得触发回退。"""
    poses = [{"x": 0.2 * i, "y": 0.0, "yaw": 0.0} for i in range(10)]
    chassis = FakeChassis(pose_steps=poses, success_after=3)
    node = _make_node(chassis)
    with _fake_time():
        outcome = nav.NavigateToServer._run_chassis_action(
            node, _goal(), "act-1", "POI_01", "track", math.inf, 60.0,
            redispatch=lambda: "act-2")
    assert outcome == "success", outcome
    assert not chassis.move_by_calls


@case
def test_arrived_within_tolerance_is_not_blocked():
    """已停在目标容差内（任务未结束）不算受阻，避免到点后误判。"""
    chassis = FakeChassis(pose_steps=[{"x": 4.8, "y": 0.0, "yaw": 0.0}])
    node = _make_node(chassis)
    with _fake_time():
        try:
            nav.NavigateToServer._run_chassis_action(
                node, _goal(), "act-1", "POI_01", "track", math.inf, 5.0,
                redispatch=lambda: "act-2")
        except RuntimeError as exc:
            assert "超时" in str(exc), exc
        else:
            raise AssertionError("容差内不动应等动作超时，而不是判受阻")
    # 距目标 0.2m <= 0.5m 容差：不判受阻，不回退
    assert not chassis.move_by_calls


@case
def test_rotation_does_not_detect_stall():
    """原地转向（detect_stall=False）：位置不变也不能判受阻。"""
    chassis = FakeChassis()
    node = _make_node(chassis)
    with _fake_time():
        try:
            nav.NavigateToServer._run_chassis_action(
                node, _goal(), "act-1", "POI_01", "track", math.inf, 3.0,
                detect_stall=False)
        except RuntimeError as exc:
            assert "超时" in str(exc), exc
    assert not chassis.move_by_calls


@case
def test_backoff_distance_is_clamped_to_chassis_range():
    chassis = FakeChassis()
    node = _make_node(chassis, _stall_backoff_m=5.0)
    with _fake_time():
        assert nav.NavigateToServer._backoff(node, _goal(), "POI_01") is True
    assert chassis.move_by_calls[0]["distance"] == 5000

    node._stall_backoff_m = 0.001
    with _fake_time():
        assert nav.NavigateToServer._backoff(node, _goal(), "POI_01") is True
    assert chassis.move_by_calls[1]["distance"] == 50


@case
def test_backoff_failure_returns_false():
    chassis = FakeChassis(backoff_status="failed")
    node = _make_node(chassis)
    with _fake_time():
        assert nav.NavigateToServer._backoff(node, _goal(), "POI_01") is False


# ── blocked 与点位顺序 ────────────────────────────────────────────

@case
def test_blocked_advances_sequence_so_next_poi_is_allowed():
    """跳过本点后，下一个点位必须能通过顺序校验，否则会连环拒绝。"""
    node = _make_node(FakeChassis())
    result = SimpleNamespace()
    nav.NavigateToServer._blocked(node, _goal(), result, "POI_01")
    assert result.phase == "blocked"
    assert result.success is False
    assert node._next_index == 1, node._next_index
    assert nav.NavigateToServer._order_error(node, "POI_02", False) is None


@case
def test_blocked_on_return_does_not_mark_returned():
    node = _make_node(FakeChassis())
    node._next_index = len(node._poi_order)
    nav.NavigateToServer._blocked(node, _goal(is_return=True),
                                  SimpleNamespace(), "start_point")
    assert node._returned is False


# ── 编排层跳过 ────────────────────────────────────────────────────

def _make_orchestrator(navigate):
    spoken = []

    def _speak(text, label):
        spoken.append((text, label))

    orch = object.__new__(orch_mod.OrchestratorNode)
    orch.__dict__.update({
        "_results": {},
        "_speak": _speak,
        "get_logger": lambda: _Logger(),
        "_navigate": navigate,
    })
    return orch, spoken


@case
def test_orchestrator_skips_blocked_poi():
    calls = []

    def _nav(poi, is_return):
        calls.append(poi)
        raise orch_mod.NavBlocked("blocked")

    orch, spoken = _make_orchestrator(_nav)
    ok = orch_mod.OrchestratorNode._navigate_inspect(orch, "POI_02")
    assert ok is False
    # 导航层已自恢复过一次，编排层不再重试
    assert calls == ["POI_02"], calls
    assert orch._results["POI_02"] == orch_mod.SKIPPED_TEXT
    assert [text for text, _ in spoken] == [orch_mod.SPEECH_TEXTS["nav_blocked"]]


@case
def test_orchestrator_retries_once_for_other_failures():
    calls = []

    def _nav(poi, is_return):
        calls.append(poi)
        raise RuntimeError("timeout")

    orch, _spoken = _make_orchestrator(_nav)
    try:
        orch_mod.OrchestratorNode._navigate_inspect(orch, "POI_01")
    except RuntimeError as exc:
        assert "timeout" in str(exc), exc
    else:
        raise AssertionError("非 blocked 失败重试后仍应抛出")
    assert calls == ["POI_01", "POI_01"], calls


@case
def test_orchestrator_propagates_run_aborted():
    calls = []

    def _nav(poi, is_return):
        calls.append(poi)
        raise orch_mod.RunAborted("stop")

    orch, _spoken = _make_orchestrator(_nav)
    try:
        orch_mod.OrchestratorNode._navigate_inspect(orch, "POI_01")
    except orch_mod.RunAborted:
        pass
    else:
        raise AssertionError("RunAborted 必须原样抛出")
    assert calls == ["POI_01"], calls


@case
def test_advance_return_in_stages_walks_to_start():
    """返程轨道中途续行被拒时，沿返程轨道按 ≤4m 分段点对点推进到起点。"""
    waypoints = [
        {"position": {"x": 20.0, "y": 2.3, "yaw": -3.11}},
        {"position": {"x": 0.0, "y": 1.2, "yaw": -3.11}},
    ]

    class _WalkChassis(FakeChassis):
        """位姿随已下发的分段动作前进：每段 4m，最后一段到起点。"""

        def get_pose(self):
            xs = [20.0, 16.0, 12.0, 8.0, 4.0, 0.1]
            x = xs[min(len(self.move_to_calls), len(xs) - 1)]
            return {"x": x, "y": 1.2 + 0.055 * x, "yaw": -3.11}

    chassis = _WalkChassis()
    node = _make_node(
        chassis,
        _pois={"start_point": {"x": 0.0, "y": 1.2, "yaw": 0.0}},
        _return_waypoints=waypoints,
    )
    with _fake_time():
        result = node._advance_return_in_stages(_goal(), "start_point", 1e6)

    assert result == "ok", result
    assert len(chassis.move_to_calls) == 5, chassis.move_to_calls
    # 每段约 4m，最后一段落在起点附近
    assert [round(c["x"]) for c in chassis.move_to_calls] == \
        [16, 12, 8, 4, 0], chassis.move_to_calls
    assert abs(chassis.move_to_calls[-1]["x"]) < 0.05
    assert abs(chassis.move_to_calls[-1]["y"] - 1.2) < 0.05


@case
def test_advance_return_keeps_intervention_offset():
    """介入侧移后分段推进保持横向偏移，不立刻回轨道中心线。"""
    waypoints = [
        {"position": {"x": 20.0, "y": 2.3, "yaw": -3.11}},
        {"position": {"x": 0.0, "y": 1.2, "yaw": -3.11}},
    ]

    class _OffsetChassis(FakeChassis):
        """车停在轨道中心线下方 0.8m（侧移避让后的位置）。"""

        def get_pose(self):
            xs = [20.0, 16.0, 12.0, 8.0, 4.0, 0.1, 0.0]
            x = xs[min(len(self.move_to_calls), len(xs) - 1)]
            # 最后一段落到起点后，位姿收敛回中线
            y = 1.2 if x == 0.0 else 1.2 + 0.055 * x - 0.8
            return {"x": x, "y": y, "yaw": -3.11}

    chassis = _OffsetChassis()
    node = _make_node(
        chassis,
        _pois={"start_point": {"x": 0.0, "y": 1.2, "yaw": 0.0}},
        _return_waypoints=waypoints,
    )
    with _fake_time():
        result = node._advance_return_in_stages(_goal(), "start_point", 1e6)

    assert result == "ok", result
    calls = chassis.move_to_calls
    assert len(calls) == 6, calls
    # 侧移保持阶段都在轨道中心线下方 0.8m，最后一段落到起点（收敛回中线）
    for call in calls[:-1]:
        expected = 1.2 + 0.055 * call["x"] - 0.8
        assert abs(call["y"] - expected) < 0.05, (call, expected)
    assert abs(calls[-1]["y"] - 1.2) < 0.05, calls[-1]


def main():
    failed = []
    for fn in TESTS:
        try:
            fn()
        except AssertionError as exc:
            failed.append((fn.__name__, exc))
            print(f"[FAIL] {fn.__name__}: {exc}")
        except Exception as exc:  # noqa: BLE001
            failed.append((fn.__name__, exc))
            print(f"[ERROR] {fn.__name__}: {type(exc).__name__}: {exc}")
        else:
            print(f"[ok]   {fn.__name__}")
    print(f"\n{len(TESTS) - len(failed)}/{len(TESTS)} 通过")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
