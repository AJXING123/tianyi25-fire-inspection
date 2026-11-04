#!/usr/bin/env python3
"""吸烟人员介入离线测试（停车 → 接近 3 米 → 播报 → 回原轨道目标）。

不初始化 rclpy、不连底盘、不连相机：
  - compute_approach_geometry：方位角/距离/standoff 的几何换算与非法值；
  - parse_detection_snapshot：视觉扩展字段的可用性判定（range_valid）；
  - 三份配置（inspection_script / pois / vision）的介入相关键与一致性。

运行前需已 source ROS 环境与工作区 install：
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    python3 tests/test_intervention.py
"""

from __future__ import annotations

import math
import os
import sys

import yaml

_HERE = os.path.dirname(os.path.abspath(__file__))
_WS = os.path.normpath(os.path.join(_HERE, ".."))
_SRC = os.path.join(_WS, "src")
for sub in ("fire_inspection_nav", "fire_inspection_orchestrator",
            "fire_inspection_vision"):
    p = os.path.join(_SRC, sub)
    if p not in sys.path:
        sys.path.insert(0, p)

SCRIPT_YAML = os.path.join(
    _SRC, "fire_inspection_orchestrator", "config", "inspection_script.yaml")
POIS_YAML = os.path.join(_SRC, "fire_inspection_nav", "config", "pois.yaml")
VISION_YAML = os.path.join(
    _SRC, "fire_inspection_vision", "config", "vision.yaml")

TESTS = []


def case(fn):
    TESTS.append(fn)
    return fn


def _yaml(path):
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def _close(a, b, tol=1e-6):
    return abs(float(a) - float(b)) <= tol


# ── 接近几何 ────────────────────────────────────────────────────────

@case
def test_geometry_straight_ahead():
    """正前方 6m、期望停车 3m → 前进 3m，朝向不变。"""
    from fire_inspection_nav.navigate_to_server import compute_approach_geometry
    g = compute_approach_geometry(
        {"x": 0.0, "y": 0.0, "yaw": 0.0}, 0.0, 6.0, 3.0)
    assert _close(g["person_x"], 6.0) and _close(g["person_y"], 0.0), g
    assert _close(g["approach_x"], 3.0) and _close(g["approach_y"], 0.0), g
    assert _close(g["travel_m"], 3.0), g
    assert _close(g["yaw"], 0.0), g


@case
def test_geometry_left_bearing_is_positive_y():
    """bearing 约定 +左：地图系 +y 为左，+pi/2 时目标在车体左侧。"""
    from fire_inspection_nav.navigate_to_server import compute_approach_geometry
    g = compute_approach_geometry(
        {"x": 1.0, "y": 2.0, "yaw": 0.0}, math.pi / 2, 5.0, 2.0)
    assert _close(g["person_x"], 1.0) and _close(g["person_y"], 7.0), g
    assert _close(g["approach_x"], 1.0) and _close(g["approach_y"], 5.0), g
    assert _close(g["travel_m"], 3.0), g
    assert _close(g["yaw"], math.pi / 2), g


@case
def test_geometry_right_bearing_is_negative_y():
    """bearing 负（目标在右）→ 地图系 -y 方向。"""
    from fire_inspection_nav.navigate_to_server import compute_approach_geometry
    g = compute_approach_geometry(
        {"x": 0.0, "y": 0.0, "yaw": 0.0}, -math.pi / 2, 4.0, 1.0)
    assert _close(g["person_y"], -4.0), g
    assert _close(g["approach_y"], -3.0), g
    assert _close(g["travel_m"], 3.0), g


@case
def test_geometry_uses_pose_yaw():
    """目标方向 = 车头朝向 + 方位角，不是裸 bearing。"""
    from fire_inspection_nav.navigate_to_server import compute_approach_geometry
    g = compute_approach_geometry(
        {"x": 0.0, "y": 0.0, "yaw": math.pi / 2}, 0.0, 6.0, 3.0)
    assert _close(g["person_x"], 0.0) and _close(g["person_y"], 6.0), g
    assert _close(g["approach_y"], 3.0), g
    assert _close(g["yaw"], math.pi / 2), g


@case
def test_geometry_never_moves_backwards():
    """已比 standoff 更近（或正好等于）→ travel=0，接近点=当前位置。"""
    from fire_inspection_nav.navigate_to_server import compute_approach_geometry
    for rng in (2.9, 3.0, 0.5):
        g = compute_approach_geometry(
            {"x": 4.0, "y": 5.0, "yaw": 0.2}, 0.1, rng, 3.0)
        assert _close(g["travel_m"], 0.0), (rng, g)
        assert _close(g["approach_x"], 4.0) and _close(g["approach_y"], 5.0), g


@case
def test_geometry_yaw_wrapped_to_pi():
    """朝向归一化到 [-pi, pi]，避免 move_to 收到 3pi 这类角度。"""
    from fire_inspection_nav.navigate_to_server import compute_approach_geometry
    g = compute_approach_geometry(
        {"x": 0.0, "y": 0.0, "yaw": 3.0}, 0.5, 6.0, 3.0)
    expected = 3.5 - 2 * math.pi
    assert _close(g["yaw"], expected), (g["yaw"], expected)
    assert -math.pi <= g["yaw"] <= math.pi


@case
def test_geometry_side_offset_stops_beside_target():
    """侧偏 >0 停在视线左侧、<0 右侧；站位点朝向仍指向目标。"""
    from fire_inspection_nav.navigate_to_server import compute_approach_geometry
    pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
    g = compute_approach_geometry(pose, 0.0, 6.0, 3.0, 1.2)
    assert _close(g["approach_x"], 3.0), g
    assert _close(g["approach_y"], 1.2), g
    # 人在右侧，站在人左侧后要向右看 → yaw 为负
    assert g["yaw"] < 0, g
    g2 = compute_approach_geometry(pose, 0.0, 6.0, 3.0, -1.2)
    assert _close(g2["approach_x"], 3.0), g2
    assert _close(g2["approach_y"], -1.2), g
    assert g2["yaw"] > 0, g2
    # 站位到目标的距离仍约为 standoff（侧偏会略微增大）
    for geom in (g, g2):
        rng = math.hypot(geom["person_x"] - geom["approach_x"],
                         geom["person_y"] - geom["approach_y"])
        assert _close(rng, math.hypot(3.0, 1.2), 1e-6), (geom, rng)


@case
def test_geometry_rejects_non_finite():
    """非法测距/方位直接抛 ValueError，由服务端转成 failed。"""
    from fire_inspection_nav.navigate_to_server import compute_approach_geometry
    pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
    for args in (
        (pose, 0.0, float("nan"), 3.0),
        (pose, float("inf"), 6.0, 3.0),
        (pose, 0.0, 6.0, float("nan")),
        ({"x": float("nan"), "y": 0.0, "yaw": 0.0}, 0.0, 6.0, 3.0),
        (pose, 0.0, 6.0, 3.0, float("nan")),
    ):
        try:
            compute_approach_geometry(*args)
        except ValueError:
            continue
        raise AssertionError(f"非有限值应抛 ValueError: {args}")


# ── 视觉快照解析 ────────────────────────────────────────────────────

def _snap(**kw):
    from fire_inspection_orchestrator.orchestrator_node import (
        parse_detection_snapshot,
    )
    return parse_detection_snapshot(kw)


@case
def test_snapshot_valid_requires_flag_and_positive_range():
    """只有视觉标了 range_valid 且距离>0 才允许靠近。"""
    s = _snap(cls="smoking", conf=0.8, range_m=6.2,
              bearing_rad=-0.2, range_valid=True)
    assert s["range_valid"] is True, s
    assert _close(s["range_m"], 6.2) and _close(s["bearing_rad"], -0.2), s
    assert s["cls"] == "smoking" and _close(s["conf"], 0.8)


@case
def test_snapshot_old_schema_is_not_valid():
    """旧版视觉没有 range 字段 → 不可用（只停车播报，不乱走）。"""
    s = _snap(cls="smoking", conf=0.8, cx=0.5, cy=0.5, w=0.2, h=0.3)
    assert s["range_valid"] is False, s
    assert _close(s["range_m"], 0.0)


@case
def test_snapshot_zero_range_is_not_valid():
    """range_m<=0 即使标了可用也不靠近（深度无效的约定值）。"""
    for rng in (0.0, -1.0):
        s = _snap(cls="smoking", conf=0.8, range_m=rng,
                  bearing_rad=0.1, range_valid=True)
        assert s["range_valid"] is False, (rng, s)


@case
def test_snapshot_tolerates_bad_types():
    """字段缺失/类型异常不得抛异常，退化为不可用。"""
    s = _snap(cls="smoking", conf="bad", range_m=None,
              bearing_rad="x", range_valid=True)
    assert s["range_valid"] is False, s
    assert _close(s["conf"], 0.0) and _close(s["range_m"], 0.0)
    assert _close(s["bearing_rad"], 0.0)


@case
def test_snapshot_rejects_non_dict():
    from fire_inspection_orchestrator.orchestrator_node import (
        parse_detection_snapshot,
    )
    assert parse_detection_snapshot(None) is None
    assert parse_detection_snapshot([1, 2]) is None


# ── 配置 ───────────────────────────────────────────────────────────

@case
def test_inspection_script_intervention_section():
    """编排层必须显式开启吸烟介入，且键集合与代码认识的一致。"""
    cfg = _yaml(SCRIPT_YAML)
    itv = cfg.get("intervention")
    assert isinstance(itv, dict), "inspection_script.yaml 缺少 intervention 段"
    allowed = {
        "enabled", "classes", "speak_only_classes",
        "avoid_classes", "avoid_offset_m", "person_on_path",
        "max_per_poi", "max_total",
        "settle_s", "snapshot_wait_s", "timeout_s",
    }
    assert set(itv) <= allowed, f"存在未知键: {sorted(set(itv) - allowed)}"
    assert itv.get("enabled") is True, "intervention.enabled 必须显式 true"
    assert itv.get("classes") == ["smoking", "person"], itv.get("classes")
    # 检测到吸烟/行人：停车 → 先播报 → 侧向绕开人员 → 继续巡检
    assert itv.get("avoid_classes") == ["smoking", "person"], itv.get(
        "avoid_classes")
    pp = itv.get("person_on_path") or {}
    assert set(pp) <= {"max_range_m", "lane_half_width_m", "max_bearing_rad"}
    assert 0.5 <= float(pp["max_range_m"]) <= 20.0, pp
    assert 0.2 <= float(pp["lane_half_width_m"]) <= 3.0, pp
    assert 0.1 <= float(pp["max_bearing_rad"]) <= math.pi / 2, pp
    assert itv.get("speak_only_classes") == [], itv.get("speak_only_classes")
    assert 0.3 <= float(itv.get("avoid_offset_m")) <= 3.0, itv.get(
        "avoid_offset_m")
    assert 0 <= int(itv.get("max_per_poi")) <= 3, "max_per_poi 过大，会原地反复停车"
    assert 0 <= int(itv.get("max_total"))
    assert float(itv.get("timeout_s")) > 0
    assert float(itv.get("settle_s")) >= 0
    assert float(itv.get("snapshot_wait_s")) >= 0


@case
def test_pois_approach_section_matches_request():
    """pois.yaml：停车距离 2m，安全边界自洽（min<=max、限速<=1.5）。"""
    cfg = _yaml(POIS_YAML)
    ap = cfg.get("approach")
    assert isinstance(ap, dict), "pois.yaml 缺少 approach 段"
    allowed = {
        "standoff_m", "side_offset_m", "min_range_m", "max_range_m",
        "max_travel_m", "arrival_tolerance_m", "timeout_s", "speed_mps",
        "retreat_offset_m",
    }
    assert set(ap) <= allowed, f"存在未知键: {sorted(set(ap) - allowed)}"
    assert _close(ap["standoff_m"], 2.0), ap["standoff_m"]
    # 站位侧偏：吸烟介入必须从人员侧向走开，不能为 0
    assert 0.3 <= abs(float(ap["side_offset_m"])) <= 3.0, ap["side_offset_m"]
    # 近距离侧向退让：人已贴到 2m 以内时必须横移让开，不能原地顶着
    assert 0.3 <= float(ap["retreat_offset_m"]) <= 3.0, ap["retreat_offset_m"]
    assert 0 < float(ap["min_range_m"]) <= float(ap["max_range_m"])
    assert 0 < float(ap["max_travel_m"]) <= 30.0
    assert 0 < float(ap["arrival_tolerance_m"]) <= 5.0
    assert 0 < float(ap["timeout_s"]) <= 600.0
    assert 0 < float(ap["speed_mps"]) <= 1.5
    # 接近段限速不得快于巡检段
    assert float(ap["speed_mps"]) <= float(
        (cfg.get("speed") or {}).get("outbound_mps", 0.4))


@case
def test_pois_corridor_rejects_door_frame_side():
    """点2→点3 门框侧横移点 y≈1.00 必须越界，否则会再撞门框。

    2026-11-03 现场：绕行落点 (23.08, 1.00) 撞门框；corridor 车中心下限
    收到 1.25 后，该点会被拒绝并镜像到另一侧，两侧都不可行则原地播报。
    """
    cfg = _yaml(POIS_YAML)
    corridor = cfg.get("corridor")
    assert isinstance(corridor, dict), "pois.yaml 缺少 corridor 段"
    assert set(corridor) <= {"x", "y", "margin_m"}, \
        f"存在未知键: {sorted(set(corridor) - {'x', 'y', 'margin_m'})}"
    lo, hi = (float(v) for v in corridor["y"])
    margin = float(corridor["margin_m"])
    assert 0.0 <= margin <= 2.0, margin
    center_lo, center_hi = lo + margin, hi - margin
    # 门框侧落点必须被排除，且留出安全余量
    assert center_lo > 1.0, (center_lo, "未排除 y≈1.00 的门框落点")
    # 仍要覆盖巡检点附近（POI_01/02 y≈1.33~1.37），否则正常接近会被误拒
    assert center_lo < 1.33, center_lo
    # 上侧不能收到 POI_04 附近（y≈2.48）无法接近
    assert center_hi >= 2.48, center_hi
    # 车中心可行区间必须比车身宽（否则任何横移都会失败）
    assert center_hi - center_lo >= 0.9, (center_lo, center_hi)


@case
def test_vision_depth_and_fov_single_source():
    """vision.yaml 深度与视场角齐备；FOV 与编排层默认参数同源。"""
    cfg = _yaml(VISION_YAML)
    depth = cfg.get("depth") or {}
    assert depth.get("topic"), "缺少 depth.topic"
    assert depth.get("camera_info_topic"), "缺少 depth.camera_info_topic"
    assert 0 < float(depth["min_m"]) < float(depth["max_m"])
    assert 0 < float(depth["sample_ratio"]) <= 1.0
    hfov = float(cfg["hfov_deg"])
    vfov = float(cfg["vfov_deg"])
    assert 0 < hfov < 180 and 0 < vfov < 180
    # 编排层 look_at 与视觉方位角换算必须共用同一份视场角：
    # 参数化读取（hfov_deg/vfov_deg），换算里不得再写死角度
    from fire_inspection_orchestrator.orchestrator_node import OrchestratorNode
    import inspect
    init_src = inspect.getsource(OrchestratorNode.__init__)
    look_src = inspect.getsource(OrchestratorNode._parse_look_at)
    assert "hfov_deg" in init_src and "vfov_deg" in init_src
    assert "self._hfov" in look_src and "self._vfov" in look_src
    assert _close(hfov, 85.5) and _close(vfov, 55.0)


@case
def test_head_pose_timeout_configured():
    """无头部姿态时只给距离不给方位角，因此必须有超时判据。"""
    cfg = _yaml(VISION_YAML)
    assert cfg.get("head_state_topic"), "缺少 head_state_topic"
    assert float(cfg["head_pose_timeout_s"]) > 0


# ── 编排层介入流程 ──────────────────────────────────────────────────

class _NoopLogger:
    """替身 logger：吞掉所有级别，测试只关心控制流。"""

    def __getattr__(self, _name):
        return lambda *a, **k: None


def _bare_orchestrator():
    """构造不连 ROS 的编排节点，只启用手势所需的字段。"""
    import threading

    import fire_inspection_orchestrator.orchestrator_node as onc

    inst = object.__new__(onc.OrchestratorNode)
    inst.get_logger = lambda: _NoopLogger()
    inst._set_state = lambda *a, **k: None
    inst._nav_enabled = True
    inst._nav_client = object()
    inst._nav_goal_timeout_s = 1.0
    inst._watch_lock = threading.Lock()
    inst._nav_in_flight = False
    inst._nav_poi = ""
    inst._intervention_event = threading.Event()
    inst._intervention_enabled = True
    inst._intervention_classes = ["smoking"]
    # 与生产配置一致：smoking 走「停车 → 侧向绕开人员 → 继续巡检」
    inst._intervention_speak_only_classes = set()
    inst._intervention_avoid_classes = {"smoking"}
    inst._avoid_offset_m = 1.5
    # 行人路径判定：默认值与生产配置一致
    inst._person_block_range_m = 3.0
    inst._person_lane_half_width_m = 0.5
    inst._person_max_bearing_rad = 0.35
    inst._person_smoking_grace_s = 0.0
    inst._intervention_cls = None
    inst._smoking_during_person = None
    inst._approach_available = False
    inst._capture_event = lambda *a, **k: None
    inst._speak_async = lambda *a, **k: None
    inst._intervention_max_per_poi = 1
    inst._intervention_max_total = 5
    inst._intervention_total = 0
    inst._intervention_counts = {}
    inst._intervention_suppress = False
    inst._point_resume_poi = None
    inst._cfg = {"inspect_pois": ["POI_02"]}
    inst._send_head_cmd = lambda *a, **k: None
    # 首次命中先停车相关字段（确认帧数与生产默认一致，窗口调小免等）
    inst._confirm_frames = 2
    inst._confirm_window_s = 0.1
    # 最小时间跨度/位置一致性：测试里时钟不推进，跨度门限置 0 以保留原语义，
    # 跨度与跳变行为由专门的用例覆盖
    inst._confirm_min_span_s = 0.0
    inst._confirm_pos_tol = 0.12
    inst._quick_confirm_conf = 0.50
    inst._smoking_quick_confirm_conf = 0.60
    inst._lost_timeout_s = 3.0
    inst._shutdown = False
    inst._stop_event = threading.Event()
    inst._watch_state = "WATCH"
    inst._watch_streak_cls = None
    inst._watch_streak = 0
    inst._watch_hits = []
    inst._watch_anchor = None
    inst._watch_lost_at = None
    inst._intervention_confirmed = False
    inst._speak_only_pending = False
    inst._prestop_unconfirmed = 0
    inst._prestop_block_until = 0.0
    inst._prestop_stopped = False
    return inst


class _NavResult:
    def __init__(self, success, phase):
        self.success = success
        self.phase = phase
        self.message = phase


@case
def test_navigate_resends_same_poi_after_intervention():
    """停车介入后重发同一点位，且必须点对点（不允许退回上轨点再走轨道）。"""
    inst = _bare_orchestrator()
    calls = []
    handle = []

    def fake_handle(poi, allow_approach=True, count=True):
        handle.append((poi, allow_approach))

    def fake_call(client, goal, timeout_s, label, abort_event=None, **kwargs):
        calls.append((goal.poi, bool(goal.use_point)))
        if len(calls) == 1:
            # 模拟首次命中先停车 + 等待期间完成 3 帧确认
            abort_event.set()
            inst._intervention_confirmed = True
            return _NavResult(False, "cancelled")
        return _NavResult(True, "arrived")

    inst._call = fake_call
    inst._handle_intervention = fake_handle
    inst._navigate("POI_02", is_return=False)

    # 首次沿轨道；介入后重发改点对点，避免底盘退回上轨点后
    # 又沿轨道从吸烟人员身上重走一遍
    assert calls == [("POI_02", False), ("POI_02", True)], calls
    assert handle == [("POI_02", True)], handle
    assert inst._nav_in_flight is False
    assert inst._intervention_event.is_set() is False
    # 到达后清空点对点续行标记，后续路段恢复沿轨道
    assert inst._point_resume_poi is None, inst._point_resume_poi


@case
def test_navigate_resumes_when_prestop_unconfirmed():
    """首次命中先停车但未确认（误报/人已走开）：不介入、恢复导航。"""
    inst = _bare_orchestrator()
    calls = []
    handle = []

    def fake_call(client, goal, timeout_s, label, abort_event=None, **kwargs):
        calls.append(bool(goal.use_point))
        if len(calls) == 1:
            abort_event.set()   # 首次命中先停车，但之后没有确认帧
            return _NavResult(False, "cancelled")
        return _NavResult(True, "arrived")

    inst._call = fake_call
    inst._handle_intervention = (
        lambda poi, allow_approach=True, count=True: handle.append(poi))
    inst._navigate("POI_02", is_return=False)

    # 未确认时不得介入播报，但停车位置可能已偏离轨道：
    # 恢复时改点对点直达，避免底盘退回上轨点后重复走一遍
    assert calls == [False, True], calls
    assert handle == [], handle             # 不得误触发接近播报流程
    assert inst._prestop_unconfirmed == 1
    assert inst._prestop_block_until > 0
    assert inst._point_resume_poi is None, inst._point_resume_poi


@case
def test_first_hit_stops_before_confirmation():
    """需要停车的类别：首次命中 1 帧就置位取消事件停车，不必等 3 帧确认。"""
    import json
    from types import SimpleNamespace

    inst = _bare_orchestrator()
    # 模拟「该类别需要停车劝阻」的配置（不在 speak_only_classes 里）
    inst._intervention_speak_only_classes = set()
    # 与生产默认一致：吸烟单帧确认门槛 0.60，0.50 只能走「先停车等确认」
    inst._quick_confirm_conf = 0.90
    inst._smoking_quick_confirm_conf = 0.60
    inst._watch_active = True
    inst._nav_in_flight = True
    inst._nav_poi = "POI_02"
    inst._scan_classes = {"smoking"}
    inst._hfov = 1.0
    inst._vfov = 0.5
    inst._cooldown_s = 15.0
    inst._watch_last_trigger = 0.0
    inst._latest_det = None
    msg = SimpleNamespace(
        detections_json=json.dumps(
            [{"cls": "smoking", "conf": 0.50, "cx": 0.5, "cy": 0.5}]),
        simulated=False,
    )
    inst._on_detection(msg)

    assert inst._intervention_event.is_set(), "首次命中应立刻停车"
    assert inst._watch_state == "LOCKED", inst._watch_state
    assert inst._intervention_confirmed is False


@case
def test_speak_only_first_hit_does_not_stop():
    """只播报不停车的类别：首次命中绝不预停车。"""
    inst = _bare_orchestrator()
    # 该类别配置成只播报（speak_only_classes），不参与绕行
    inst._intervention_speak_only_classes = {"smoking"}
    inst._intervention_avoid_classes = set()
    inst._nav_in_flight = True
    inst._nav_poi = "POI_02"
    _scan_ready(inst)

    inst._on_detection(_det_msg(conf=0.45))

    assert inst._intervention_event.is_set() is False, "只播报类别不得停车"
    assert inst._watch_state == "LOCKED", inst._watch_state
    assert inst._intervention_confirmed is False


@case
def test_navigate_intervenes_in_place_when_already_arrived():
    """取消晚到（已到点）：就地介入且不再接近，避免离开点位导致视觉检查失效。"""
    inst = _bare_orchestrator()
    calls = []
    handle = []

    def fake_call(client, goal, timeout_s, label, abort_event=None, **kwargs):
        calls.append(goal.poi)
        abort_event.set()
        inst._intervention_confirmed = True   # 已确认吸烟，但取消没赶上到点
        return _NavResult(True, "arrived")

    inst._call = fake_call
    inst._handle_intervention = lambda poi, allow_approach=True, count=True: (
        handle.append((poi, allow_approach)))
    inst._navigate("POI_02", is_return=False)

    assert calls == ["POI_02"], calls
    assert handle == [("POI_02", False)], handle


@case
def test_navigate_fails_without_intervention():
    """没有介入事件时，导航失败必须照旧抛异常（不重试、不吞错）。"""
    inst = _bare_orchestrator()
    inst._call = lambda *a, **k: _NavResult(False, "failed")

    def _should_not_run(*a, **k):
        raise AssertionError("不应触达介入")

    inst._handle_intervention = _should_not_run
    try:
        inst._navigate("POI_02", is_return=False)
    except RuntimeError as exc:
        assert "POI_02" in str(exc), exc
    else:
        raise AssertionError("导航失败应抛 RuntimeError")


@case
def test_intervention_gating():
    """介入开关 / 类别 / 每点次数 / 本轮上限 / 是否在途 都必须同时满足。"""
    inst = _bare_orchestrator()
    inst._nav_in_flight = True
    inst._nav_poi = "POI_02"
    assert inst._intervention_allowed("smoking") is True

    assert inst._intervention_allowed("safety_sign") is False, "非介入类别"
    inst._nav_in_flight = False
    assert inst._intervention_allowed("smoking") is False, "未在导航途中"
    inst._nav_in_flight = True
    inst._intervention_enabled = False
    assert inst._intervention_allowed("smoking") is False, "总开关关闭"
    inst._intervention_enabled = True
    inst._intervention_counts["POI_02"] = 1
    assert inst._intervention_allowed("smoking") is False, "每点次数用尽"
    assert inst._intervention_allowed("smoking") is False
    inst._intervention_counts.clear()
    inst._intervention_total = 5
    assert inst._intervention_allowed("smoking") is False, "本轮总次数用尽"
    inst._intervention_max_total = 0  # 0 表示不限
    assert inst._intervention_allowed("smoking") is True
    inst._intervention_suppress = True
    assert inst._intervention_allowed("smoking") is False, "路段硬上限已置位"


@case
def test_navigate_caps_interventions_per_leg():
    """max_per_poi/max_total 配成 0（不限）时，同一路段仍有硬上限，不会无限循环。"""
    inst = _bare_orchestrator()
    inst._intervention_max_per_poi = 0
    inst._intervention_max_total = 0
    calls = []
    handled = []

    def fake_call(client, goal, timeout_s, label, abort_event=None, **kwargs):
        calls.append(goal.poi)
        # 抑制置位后检测流不再触发介入 → 本段正常走完
        if not inst._intervention_suppress:
            abort_event.set()
            inst._intervention_confirmed = True   # 模拟停车后确认成立
            return _NavResult(False, "cancelled")
        return _NavResult(True, "arrived")

    inst._call = fake_call
    inst._handle_intervention = (
        lambda poi, allow_approach=True, count=True: handled.append(poi))
    inst._navigate("POI_02", is_return=False)

    assert len(handled) == 3, handled
    assert len(calls) == 4 and calls[-1] == "POI_02", calls
    assert inst._intervention_suppress is True
    assert inst._nav_in_flight is False


@case
def test_navigate_stops_head_on_arrived_feedback():
    """巡检点一到就通过反馈停扫视，不等到点转向/停留走完；重复反馈不重复发。"""
    inst = _bare_orchestrator()
    stopped = []
    inst._send_head_cmd = lambda cmd: stopped.append(cmd)

    class _Feedback:
        def __init__(self, phase):
            self.phase = phase

    def fake_call(client, goal, timeout_s, label, abort_event=None,
                  feedback_cb=None, **kwargs):
        assert feedback_cb is not None, "巡检点必须带反馈回调"
        feedback_cb(_Feedback("navigating"))
        assert stopped == [], "行进中不应停扫视"
        feedback_cb(_Feedback("arrived"))
        assert stopped == ["stop"], stopped
        feedback_cb(_Feedback("arrived"))
        assert stopped == ["stop"], "重复 arrived 不得重复发指令"
        return _NavResult(True, "arrived")

    inst._call = fake_call
    inst._navigate("POI_02", is_return=False)
    assert stopped == ["stop"], stopped
    assert inst._nav_in_flight is False


@case
def test_navigate_skips_head_stop_for_non_inspect_poi():
    """过点点位/返程不停头：只巡检点才需要提前停扫视取图。

    这两类路段仍要监听 detour 相位（受阻绕行播报），因此回调必须存在，
    但收到 arrived 时绝不能下发停扫视指令。
    """
    inst = _bare_orchestrator()
    stopped = []
    inst._send_head_cmd = lambda cmd: stopped.append(cmd)

    class _Feedback:
        def __init__(self, phase):
            self.phase = phase

    def fake_call(client, goal, timeout_s, label, abort_event=None,
                  feedback_cb=None, **kwargs):
        assert feedback_cb is not None, "所有路段都要监听 detour 相位"
        feedback_cb(_Feedback("arrived"))
        return _NavResult(True, "arrived")

    inst._call = fake_call
    inst._navigate("POI_04", is_return=False)
    inst._navigate("POI_02", is_return=True)
    assert stopped == [], stopped


@case
def test_navigate_speaks_on_detour_feedback_and_completion():
    """受阻绕行：detour 反馈只播一次「正在绕行」，到点后播「已绕过障碍」。"""
    inst = _bare_orchestrator()
    async_spoken, sync_spoken = [], []
    inst._speak_async = lambda text, label: async_spoken.append(text)
    inst._speak = lambda text, label: sync_spoken.append(text)

    class _Feedback:
        def __init__(self, phase):
            self.phase = phase

    def fake_call(client, goal, timeout_s, label, abort_event=None,
                  feedback_cb=None, **kwargs):
        feedback_cb(_Feedback("detour"))
        feedback_cb(_Feedback("detour"))   # 同一段路重复反馈不得重复播报
        return _NavResult(True, "arrived")

    inst._call = fake_call
    inst._navigate("POI_02", is_return=False)

    import fire_inspection_orchestrator.orchestrator_node as onc
    assert async_spoken == [onc.SPEECH_TEXTS["nav_obstacle"]], async_spoken
    assert sync_spoken == [onc.SPEECH_TEXTS["nav_detour_ok"]], sync_spoken


@case
def test_navigate_no_detour_speech_when_clear_path():
    """未发生绕行：不得播报绕行相关文本，避免误导现场。"""
    inst = _bare_orchestrator()
    async_spoken, sync_spoken = [], []
    inst._speak_async = lambda text, label: async_spoken.append(text)
    inst._speak = lambda text, label: sync_spoken.append(text)

    class _Feedback:
        def __init__(self, phase):
            self.phase = phase

    def fake_call(client, goal, timeout_s, label, abort_event=None,
                  feedback_cb=None, **kwargs):
        feedback_cb(_Feedback("navigating"))
        return _NavResult(True, "arrived")

    inst._call = fake_call
    inst._navigate("POI_02", is_return=False)

    assert async_spoken == [] and sync_spoken == [], (async_spoken, sync_spoken)


# ── 近距离侧向退让 ──────────────────────────────────────────────────

class _ApproachGoal:
    is_cancel_requested = False

    def __init__(self):
        self.succeeded = False
        self.aborted = False

    def succeed(self):
        self.succeeded = True

    def abort(self):
        self.aborted = True


def _bare_approach_node(retreat=1.2, min_range=2.0, pose=None,
                        range_error=None):
    import fire_inspection_nav.navigate_to_server as navm

    inst = object.__new__(navm.NavigateToServer)
    inst.get_logger = lambda: _NoopLogger()
    inst._dry_run = False
    inst._approach_retreat_m = retreat
    inst._approach_min_range_m = min_range
    inst._read_pose = lambda: dict(pose or {"x": 0.0, "y": 0.0, "yaw": 0.0})
    inst._point_range_error = lambda x, y: range_error
    inst._corridor = {}
    return inst, navm


def _run_retreat(inst, navm, bearing, range_m):
    captured = {}

    def fake_exec(gh, result, geometry, standoff, timeout_s,
                  phase="arrived", message=None):
        captured.update(geometry=geometry, phase=phase, message=message)
        result.success = True
        result.phase = phase
        result.final_range_m = 1.2
        result.message = "x"
        gh.succeed()
        return result

    inst._execute_approach_real = fake_exec
    goal = _ApproachGoal()
    result = navm.ApproachTarget.Result()
    ret = inst._retreat_beside_target(goal, result, bearing, range_m, 10.0)
    return ret, captured, goal


@case
def test_retreat_beside_target_strafes_away_from_person():
    """人偏左→向右退、偏右→向左退，只横移不前进，phase=retreated。"""
    inst, navm = _bare_approach_node()
    ret, cap, goal = _run_retreat(inst, navm, 0.0, 0.8)
    geom = cap["geometry"]
    # 正前方默认向右退：地图系 y 负方向，朝向沿退让方向
    assert _close(geom["approach_x"], 0.0), geom
    assert _close(geom["approach_y"], -1.2), geom
    assert _close(geom["yaw"], -math.pi / 2), geom
    assert _close(geom["travel_m"], 1.2), geom
    assert _close(geom["person_x"], 0.8) and _close(geom["person_y"], 0.0), geom
    assert cap["phase"] == "retreated", cap
    assert "右" in cap["message"], cap["message"]
    assert ret.phase == "retreated" and goal.succeeded

    inst, navm = _bare_approach_node()
    ret, cap, goal = _run_retreat(inst, navm, -0.5, 1.0)
    geom = cap["geometry"]
    assert _close(geom["approach_y"], 1.2), geom
    assert _close(geom["yaw"], math.pi / 2), geom
    assert "左" in cap["message"], cap["message"]


@case
def test_retreat_falls_back_to_skip_when_unsafe():
    """软限位越界 / 退让距离配 0：不得横移，退回「原地不接近」。"""
    inst, navm = _bare_approach_node(range_error="超出软限位")
    ret, _, goal = _run_retreat(inst, navm, 0.0, 0.8)
    assert ret.phase == "skipped" and goal.succeeded, ret.phase

    inst, navm = _bare_approach_node(retreat=0.0)
    ret, _, goal = _run_retreat(inst, navm, 0.0, 0.8)
    assert ret.phase == "skipped" and goal.succeeded, ret.phase


@case
def test_retreat_rejected_when_outside_corridor():
    """退让点跑出可行走廊（现场实测 y=0.62 落在走廊外）→ 原地不接近。"""
    inst, navm = _bare_approach_node()
    inst._corridor = {"y": [1.0, 2.8]}
    ret, _, goal = _run_retreat(inst, navm, 0.0, 0.8)
    assert ret.phase == "skipped" and goal.succeeded, ret.phase
    assert "走廊" in ret.message, ret.message


@case
def test_retreat_switches_side_when_preferred_hugs_wall():
    """首选侧贴着走廊边界时改走另一侧（2026-11-02 右侧贴墙现场）。"""
    inst, navm = _bare_approach_node()
    inst._corridor = {"y": [-0.60, 4.40], "margin_m": 0.90}
    inst._read_pose = lambda: {"x": 16.93, "y": 2.19, "yaw": -3.112}
    # 人略偏左 → 首选右侧退让（y≈3.39，距上限 3.50 仅 0.11m）
    ret, cap, goal = _run_retreat(inst, navm, 0.147, 0.88)
    geom = cap["geometry"]
    assert ret.phase == "retreated" and goal.succeeded, ret.phase
    # 应改向左退（y≈0.99），而不是贴近上侧墙
    assert _close(geom["approach_y"], 0.9905, 1e-3), geom
    assert _close(geom["yaw"], -1.541, 1e-3), geom
    assert "左" in cap["message"], cap["message"]


@case
def test_corridor_margin_rejects_wall_adjacent_point():
    """5m 走廊（墙 y=-0.6/4.4）+ 0.9m 车体余量：贴墙拒绝，正常侧移放行。"""
    inst, navm = _bare_approach_node()
    inst._corridor = {"y": [-0.60, 4.40], "margin_m": 0.90}

    # 内缩后车中心有效区间 [0.30, 3.50]
    assert inst._point_corridor_error(23.12, 0.10) is not None
    assert inst._point_corridor_error(23.12, 4.20) is not None
    assert inst._point_corridor_error(23.12, 1.90) is None
    # 1.2m 侧向退让/绕行后的落点仍在安全区（0.7 / 3.1）
    assert inst._point_corridor_error(23.12, 0.70) is None
    assert inst._point_corridor_error(23.12, 3.10) is None


class _ApproachFakeChassis:
    """按顺序返回动作状态的假底盘，记录 move_to 下发次数。"""

    def __init__(self, statuses):
        self._statuses = list(statuses)
        self.moves = 0

    def move_to(self, x, y, yaw, flags=0):
        self.moves += 1
        return {"action_id": f"act-{self.moves}"}

    def get_action_status(self, action_id):
        if not self._statuses:
            return {"status": "failed"}
        return self._statuses.pop(0)


def _bare_approach_exec(statuses):
    import fire_inspection_nav.navigate_to_server as navm

    inst = object.__new__(navm.NavigateToServer)
    inst.get_logger = lambda: _NoopLogger()
    inst._dry_run = False
    inst._shutting_down = False
    inst._check_localization = lambda strict, goal_handle: "ok"
    inst._check_health = lambda: None
    inst._apply_speed = lambda *a, **k: None
    inst._set_aeb = lambda on: None
    inst._cancel_action = lambda *a, **k: None
    inst._cancel_and_settle = lambda *a, **k: None
    inst._publish_approach_feedback = lambda *a, **k: None
    inst._range_to_point = lambda x, y: 2.0
    inst._warn_approach_arrival = lambda *a, **k: None
    inst._approach_tolerance_m = 0.8
    inst._approach_speed_mps = 0.35
    inst._action_poll_interval_s = 0.0
    inst._chassis = _ApproachFakeChassis(statuses)
    return inst, navm


_APPROACH_FAIL_ST = {
    "status": "failed", "error_code": 0,
    "distance_remaining": 23244, "distance_time": 11200,
}


@case
def test_approach_retries_once_after_action_failure():
    """接近动作首次失败（现场那条 0.76s failed）→ 撤干净后重发一次成功。"""
    inst, navm = _bare_approach_exec([_APPROACH_FAIL_ST, {"status": "success"}])
    geometry = {"approach_x": 1.0, "approach_y": 2.0, "yaw": 0.0,
                "person_x": 3.0, "person_y": 2.0}
    result = navm.ApproachTarget.Result()
    goal = _ApproachGoal()

    ret = inst._execute_approach_real(
        goal, result, geometry, 2.0, 60.0)

    assert inst._chassis.moves == 2, inst._chassis.moves
    assert ret.success is True and ret.phase == "arrived", ret
    assert goal.succeeded is True


@case
def test_approach_gives_up_after_second_failure():
    """连续两次失败必须抛出，交给编排层原地播报，不无限重发。"""
    inst, navm = _bare_approach_exec(
        [_APPROACH_FAIL_ST, _APPROACH_FAIL_ST])
    geometry = {"approach_x": 1.0, "approach_y": 2.0, "yaw": 0.0,
                "person_x": 3.0, "person_y": 2.0}
    result = navm.ApproachTarget.Result()
    goal = _ApproachGoal()

    try:
        inst._execute_approach_real(goal, result, geometry, 2.0, 60.0)
    except RuntimeError:
        pass
    else:
        raise AssertionError("连续失败应抛出异常")
    assert inst._chassis.moves == 2, inst._chassis.moves
    assert goal.succeeded is False


# ── 预停车降噪 / 介入次数用尽后停车播报 ──────────────────────────────

def _det_msg(conf=0.6, cx=0.5, cy=0.5, cls="smoking"):
    import json
    from types import SimpleNamespace

    return SimpleNamespace(
        detections_json=json.dumps(
            [{"cls": cls, "conf": conf, "cx": cx, "cy": cy}]),
        simulated=False,
    )


def _scan_ready(inst, conf_window=10.0):
    inst._watch_active = True
    inst._scan_classes = {"smoking"}
    inst._hfov = 1.0
    inst._vfov = 0.5
    inst._cooldown_s = 15.0
    inst._watch_last_trigger = 0.0
    inst._latest_det = None
    inst._confirm_window_s = conf_window


@case
def test_prestop_ignores_low_confidence():
    """低置信度（<0.40）不得首帧就停：现场日志里这类误报占比最高。"""
    inst = _bare_orchestrator()
    inst._nav_in_flight = True
    inst._nav_poi = "POI_02"
    _scan_ready(inst)

    inst._on_detection(_det_msg(conf=0.35))

    assert inst._intervention_event.is_set() is False, "低分误报不得停车"
    assert inst._watch_state == "LOCKED", inst._watch_state


@case
def test_high_conf_single_frame_speaks_without_stopping():
    """只播报类别：高置信度单帧确认后播报+抓拍，但绝不停车。

    现场日志：首次命中 conf=0.63 → 0.2s 后目标丢失，2 帧永远凑不齐，
    所以仍保留单帧确认；区别只在于确认后不取消导航。
    """
    inst = _bare_orchestrator()
    # 该类别配置成只播报（speak_only_classes）
    inst._intervention_speak_only_classes = {"smoking"}
    inst._intervention_avoid_classes = set()
    inst._nav_in_flight = True
    inst._nav_poi = "POI_02"
    _scan_ready(inst)
    spoken = []
    inst._speak_async = lambda text, label: spoken.append(text)

    inst._on_detection(_det_msg(conf=0.63))

    import fire_inspection_orchestrator.orchestrator_node as onc
    assert inst._intervention_event.is_set() is False, "只播报类别不得停车"
    assert inst._watch_state == "COOLDOWN", inst._watch_state
    assert inst._intervention_confirmed is False
    assert spoken == [onc.SPEECH_TEXTS["smoking"]], spoken


@case
def test_low_conf_does_not_single_frame_confirm():
    """低于快速门槛时不得单帧确认，必须继续走多帧 + 位置一致性。"""
    inst = _bare_orchestrator()
    inst._nav_in_flight = True
    inst._nav_poi = "POI_02"
    _scan_ready(inst)

    inst._on_detection(_det_msg(conf=0.45))

    assert inst._watch_state == "LOCKED", inst._watch_state
    assert inst._intervention_confirmed is False


@case
def test_confirmed_smoking_with_exhausted_quota_still_stops():
    """需要停车的类别：每点位介入次数用尽后确认仍要停车播报（不接近）。"""
    inst = _bare_orchestrator()
    # 模拟「该类别需要停车劝阻」的配置（不在 speak_only_classes 里）
    inst._intervention_speak_only_classes = set()
    inst._nav_in_flight = True
    inst._nav_poi = "POI_02"
    inst._intervention_counts["POI_02"] = 1   # max_per_poi=1 已用尽
    _scan_ready(inst)

    for _ in range(2):
        inst._on_detection(_det_msg())

    assert inst._intervention_allowed("smoking") is False
    assert inst._watch_state == "INTERVENE", inst._watch_state
    assert inst._speak_only_pending is True
    assert inst._intervention_confirmed is True
    assert inst._intervention_event.is_set(), "次数用尽也必须停车"
    assert inst._intervention_snap["cls"] == "smoking"


@case
def test_speak_only_class_never_stops_even_with_exhausted_quota():
    """只播报类别：介入次数用尽也不停车，继续执行巡检任务。"""
    inst = _bare_orchestrator()
    # 该类别配置成只播报（speak_only_classes）
    inst._intervention_speak_only_classes = {"smoking"}
    inst._intervention_avoid_classes = set()
    inst._nav_in_flight = True
    inst._nav_poi = "POI_02"
    inst._intervention_counts["POI_02"] = 1   # max_per_poi=1 已用尽
    _scan_ready(inst)
    spoken = []
    inst._speak_async = lambda text, label: spoken.append(text)

    for _ in range(2):
        inst._on_detection(_det_msg())

    assert inst._intervention_allowed("smoking") is False
    assert inst._intervention_event.is_set() is False
    assert inst._watch_state == "COOLDOWN", inst._watch_state
    assert spoken, "确认后仍要播报"


# ── 侧向绕开人员（avoid_classes） ────────────────────────────────

def _avoid_orch():
    inst = _bare_orchestrator()
    inst._approach_available = True
    inst._intervention_timeout_s = 60.0
    inst._approach_client = object()
    return inst


@case
def test_snapshot_exposes_bearing_valid_flag():
    """区分「没有方位角」与「方位角为 0」：绕行方向判断依赖它。"""
    s = _snap(cls="smoking", conf=0.8, range_m=4.0, bearing_rad=-0.2,
              range_valid=True)
    assert s["bearing_valid"] is True, s
    assert _snap(cls="smoking", conf=0.8)["bearing_valid"] is False
    assert _snap(cls="smoking", bearing_rad=None)["bearing_valid"] is False
    assert _snap(cls="smoking", bearing_rad="x")["bearing_valid"] is False


@case
def test_avoid_target_moves_away_from_person():
    """standoff == range（不前进），按方位角向远离人员的一侧横移。"""
    inst = _avoid_orch()
    goals = []

    def fake_call(client, goal, timeout_s, label, **kw):
        goals.append(goal)
        return _NavResult(True, "arrived")

    inst._call = fake_call
    snap = {"cls": "smoking", "bearing_rad": 0.4, "bearing_valid": True,
            "range_m": 4.0, "range_valid": True, "conf": 0.8}
    assert inst._avoid_target(snap) is True
    g = goals[0]
    assert _close(g.range_m, 4.0) and _close(g.standoff_m, 4.0), g
    # 人在左侧（bearing>0）→ 从右侧绕（side<0）
    assert _close(g.side_offset_m, -1.5), g.side_offset_m

    inst2 = _avoid_orch()
    goals2 = []
    inst2._call = lambda c, goal, t, label, **k: (
        goals2.append(goal) or _NavResult(True, "retreated"))
    snap2 = dict(snap, bearing_rad=-0.4)
    assert inst2._avoid_target(snap2) is True
    # 人在右侧 → 从左侧绕（side>0）
    assert _close(goals2[0].side_offset_m, 1.5), goals2[0].side_offset_m


@case
def test_avoid_target_skips_without_bearing_or_range():
    """没有有效方位/测距时不盲动（只播报），绝不下发侧移动作。"""
    inst = _avoid_orch()
    calls = []
    inst._call = lambda *a, **k: calls.append(a) or _NavResult(True, "arrived")
    base = {"cls": "smoking", "bearing_rad": 0.2, "bearing_valid": True,
            "range_m": 4.0, "range_valid": True, "conf": 0.8}
    assert inst._avoid_target(dict(base, bearing_valid=False)) is False
    assert inst._avoid_target(dict(base, range_valid=False)) is False
    assert inst._avoid_target(dict(base, range_m=0.0, range_valid=True)) is False
    assert inst._avoid_target(None) is False
    assert calls == [], calls


@case
def test_handle_intervention_speaks_before_avoiding():
    """绕行类别：先播「发现吸烟」→ 再侧移 → 最后播「已避开」。"""
    import threading as _threading

    import fire_inspection_orchestrator.orchestrator_node as onc

    inst = _bare_orchestrator()
    inst._intervention_snap = {"cls": "smoking", "audio": False}
    inst._intervention_settle_s = 0.0
    inst._intervention_snapshot_wait_s = 0.0
    inst._cooldown_s = 15.0
    inst._transit_watch_on = lambda: None
    inst._stop_event = _threading.Event()
    events = []
    inst._speak = lambda text, label: events.append(("speak", text))
    inst._avoid_target = lambda snap: (events.append(("avoid", None)), True)[1]
    approached = []
    inst._approach_target = lambda snap: approached.append(snap)

    inst._handle_intervention("POI_02", allow_approach=True, count=True)

    assert [kind for kind, _ in events] == ["speak", "avoid", "speak"], events
    assert events[0][1] == onc.SPEECH_TEXTS["smoking"], events
    assert events[2][1] == onc.SPEECH_TEXTS["smoking_resume"], events
    assert approached == [], approached


@case
def test_handle_intervention_person_speaks_before_avoiding():
    """行人绕行：先播让行提示 → 再侧移 → 最后播「已避让行人」。"""
    import threading as _threading

    import fire_inspection_orchestrator.orchestrator_node as onc

    inst = _bare_orchestrator()
    inst._intervention_avoid_classes = {"smoking", "person"}
    inst._intervention_snap = {"cls": "person", "audio": False}
    inst._intervention_settle_s = 0.0
    inst._intervention_snapshot_wait_s = 0.0
    inst._cooldown_s = 15.0
    inst._transit_watch_on = lambda: None
    inst._stop_event = _threading.Event()
    events = []
    inst._speak = lambda text, label: events.append(("speak", text))
    inst._avoid_target = lambda snap: (events.append(("avoid", None)), True)[1]

    inst._handle_intervention("POI_02", allow_approach=True, count=True)

    assert [kind for kind, _ in events] == ["speak", "avoid", "speak"], events
    assert events[0][1] == onc.SPEECH_TEXTS["person"], events
    assert events[2][1] == onc.SPEECH_TEXTS["person_resume"], events


@case
def test_target_on_path_gating():
    """只有很近且基本正前方才停车；偏到路外/较远只播报。"""
    inst = _bare_orchestrator()
    ahead = {"bearing_valid": True, "bearing_rad": 0.05,
             "range_valid": True, "range_m": 2.0}
    assert inst._target_on_path(ahead) is True
    # 2m、方位 0.6rad → 横向偏移约 1.1m，超出 0.5m 路宽
    assert inst._target_on_path(dict(ahead, bearing_rad=0.6)) is False
    # 超过 3m：只播报
    assert inst._target_on_path(dict(ahead, range_m=3.1)) is False
    # 缺方位/缺测距：保守按在路径上处理
    assert inst._target_on_path({"bearing_valid": False}) is True
    assert inst._target_on_path({
        "bearing_valid": True, "bearing_rad": 0.1,
        "range_valid": False}) is True
    assert inst._target_on_path(None) is True


@case
def test_first_hit_side_target_without_bearing_does_not_prestop():
    """首帧没有 bearing_rad 时用 cx 估算方位：侧前方的人不得预停车。

    2026-11-03 返程实测：首帧 bearing_valid=False → 一律按“在路径上”，
    侧前方约 1m 的人连续 3 次触发预停车；确认时拿到 bearing≈-0.8rad
    才发现根本不在路线上。这里验证缺方位时用 cx 估算能挡住。
    """
    inst = _bare_orchestrator()
    inst._nav_in_flight = True
    inst._nav_poi = "POI_02"
    inst._intervention_classes = ["person", "smoking"]
    _scan_ready(inst)
    inst._scan_classes = {"person"}
    # hfov=1.0，cx=0.1 → 估算方位 +0.40rad，超过 max_bearing 0.35
    inst._on_detection(_det_msg(conf=0.85, cx=0.1, cls="person"))

    assert inst._intervention_event.is_set() is False, "侧前方的人不得预停车"
    assert inst._watch_state == "WATCH", inst._watch_state


@case
def test_first_hit_side_smoking_without_bearing_does_not_prestop():
    """侧前方吸烟（首帧缺 bearing）只记录，不在返程途中逼停。"""
    inst = _bare_orchestrator()
    inst._nav_in_flight = True
    inst._nav_poi = "POI_02"
    _scan_ready(inst)
    inst._on_detection(_det_msg(conf=0.85, cx=0.1))

    assert inst._intervention_event.is_set() is False, "侧前方吸烟不得预停车"
    assert inst._intervention_confirmed is False


@case
def test_first_hit_centered_target_without_bearing_still_prestops():
    """正前方的目标即使缺 bearing_rad，也必须照常预停车。"""
    inst = _bare_orchestrator()
    inst._nav_in_flight = True
    inst._nav_poi = "POI_02"
    inst._intervention_classes = ["person", "smoking"]
    _scan_ready(inst)
    inst._scan_classes = {"person"}
    # cx=0.45 → 估算方位 +0.05rad，在路线内
    inst._on_detection(_det_msg(conf=0.85, cx=0.45, cls="person"))

    assert inst._intervention_event.is_set() is True
    assert inst._watch_state == "LOCKED", inst._watch_state


@case
def test_smoking_priority_over_person_when_both_detected():
    """同一帧同时检出人和吸烟：吸烟优先，不能因 person 分高丢掉吸烟警告。"""
    import json
    from types import SimpleNamespace

    inst = _bare_orchestrator()
    inst._nav_in_flight = True
    inst._nav_poi = "POI_02"
    _scan_ready(inst)
    inst._scan_classes = {"smoking", "person"}
    msg = SimpleNamespace(
        detections_json=json.dumps([
            {"cls": "person", "conf": 0.92, "cx": 0.5, "cy": 0.5},
            {"cls": "smoking", "conf": 0.74, "cx": 0.5, "cy": 0.5},
        ]),
        simulated=False,
    )

    inst._on_detection(msg)

    assert inst._watch_streak_cls == "smoking", inst._watch_streak_cls
    assert inst._latest_det["cls"] == "smoking", inst._latest_det


@case
def test_person_lock_upgrades_to_smoking():
    """锁定 person 期间消防模型才看到烟：必须升级为 smoking，不能按人放走。"""
    import json
    from types import SimpleNamespace

    inst = _bare_orchestrator()
    inst._nav_in_flight = True
    inst._nav_poi = "POI_02"
    _scan_ready(inst)
    inst._scan_classes = {"smoking", "person"}

    def msg(dets):
        return SimpleNamespace(
            detections_json=json.dumps(dets), simulated=False)

    # 第一帧：只有人 → 锁定 person（普通行人不预停车、不单帧确认）
    inst._on_detection(msg([
        {"cls": "person", "conf": 0.90, "cx": 0.5, "cy": 0.5}]))
    assert inst._watch_streak_cls == "person", inst._watch_streak_cls
    assert inst._watch_state == "LOCKED", inst._watch_state

    # 第二帧：抬头后消防模型看到烟（低于单帧门槛）→ 升级为 smoking 并重新累计
    inst._on_detection(msg([
        {"cls": "person", "conf": 0.92, "cx": 0.5, "cy": 0.5},
        {"cls": "smoking", "conf": 0.55, "cx": 0.5, "cy": 0.5}]))
    assert inst._watch_streak_cls == "smoking", inst._watch_streak_cls
    assert inst._watch_streak == 1, inst._watch_streak


@case
def test_person_lock_single_frame_smoking_confirms():
    """锁定 person 时出现高置信吸烟帧（≥0.60）：直接按吸烟确认，不等多帧。"""
    import json
    from types import SimpleNamespace

    inst = _bare_orchestrator()
    inst._nav_in_flight = True
    inst._nav_poi = "POI_02"
    _scan_ready(inst)
    inst._scan_classes = {"smoking", "person"}

    def msg(dets):
        return SimpleNamespace(
            detections_json=json.dumps(dets), simulated=False)

    inst._on_detection(msg([
        {"cls": "person", "conf": 0.90, "cx": 0.5, "cy": 0.5}]))
    assert inst._watch_state == "LOCKED", inst._watch_state

    inst._on_detection(msg([
        {"cls": "person", "conf": 0.92, "cx": 0.5, "cy": 0.5},
        {"cls": "smoking", "conf": 0.73, "cx": 0.5, "cy": 0.5}]))

    assert inst._watch_state == "INTERVENE", inst._watch_state
    assert inst._intervention_confirmed is True
    assert inst._intervention_event.is_set()
    assert inst._intervention_snap["cls"] == "smoking", inst._intervention_snap


@case
def test_trigger_confirmed_off_path_person_is_ignored():
    """未预停车时，确认行人不在路径上：不播报、不占冷却。"""
    inst = _bare_orchestrator()
    inst._intervention_classes = ["smoking", "person"]
    inst._intervention_avoid_classes = {"smoking", "person"}
    inst._watch_active = True
    inst._nav_in_flight = True
    inst._nav_poi = "POI_02"
    inst._watch_streak = 3
    spoken = []
    inst._speak_async = lambda text, label: spoken.append(text)
    inst._intervention_event.clear()
    top = {"cls": "person", "conf": 0.9, "cx": 0.5, "cy": 0.5,
           "bearing_rad": 0.6, "bearing_valid": True,
           "range_m": 3.0, "range_valid": True}

    inst._trigger_confirmed("person", top)

    assert inst._watch_state == "WATCH", inst._watch_state
    assert inst._intervention_event.is_set() is False
    assert inst._intervention_confirmed is False
    assert spoken == [], spoken


@case
def test_trigger_confirmed_off_path_person_announces_if_prestopped():
    """已经预停车刹停过：侧前方路人也补播一句让行提示，不占冷却。"""
    inst = _bare_orchestrator()
    inst._intervention_classes = ["smoking", "person"]
    inst._intervention_avoid_classes = {"smoking", "person"}
    inst._watch_active = True
    inst._nav_in_flight = True
    inst._nav_poi = "POI_02"
    inst._watch_streak = 3
    inst._prestop_stopped = True
    spoken = []
    inst._speak_async = lambda text, label: spoken.append(text)
    inst._intervention_event.clear()
    top = {"cls": "person", "conf": 0.9, "cx": 0.5, "cy": 0.5,
           "bearing_rad": 0.6, "bearing_valid": True,
           "range_m": 3.0, "range_valid": True}

    inst._trigger_confirmed("person", top)

    from fire_inspection_voice.speech_texts import SPEECH_TEXTS
    assert spoken == [SPEECH_TEXTS["person"]], spoken
    # 一次性消费：下一次侧前方路人不再重复播报
    assert inst._prestop_stopped is False
    assert inst._watch_state == "WATCH", inst._watch_state
    assert inst._intervention_confirmed is False


@case
def test_trigger_confirmed_off_path_smoking_still_speaks():
    """吸烟是安全异常事件：即使不在路线上也要播报（只播报不停车）。"""
    import fire_inspection_orchestrator.orchestrator_node as onc

    inst = _bare_orchestrator()
    inst._intervention_classes = ["smoking", "person"]
    inst._intervention_avoid_classes = {"smoking", "person"}
    inst._watch_active = True
    inst._nav_in_flight = True
    inst._nav_poi = "POI_02"
    inst._watch_streak = 3
    spoken = []
    inst._speak_async = lambda text, label: spoken.append(text)
    inst._intervention_event.clear()
    top = {"cls": "smoking", "conf": 0.9, "cx": 0.5, "cy": 0.5,
           "bearing_rad": 0.6, "bearing_valid": True,
           "range_m": 3.0, "range_valid": True}

    inst._trigger_confirmed("smoking", top)

    assert inst._watch_state == "COOLDOWN", inst._watch_state
    assert inst._intervention_event.is_set() is False
    assert spoken == [onc.SPEECH_TEXTS["smoking"]], spoken


@case
def test_person_off_path_not_locked_at_first_hit():
    """侧前方的行人首帧不锁定头部；正前方同样距离才正常锁定。"""
    import json
    from types import SimpleNamespace

    inst = _bare_orchestrator()
    inst._intervention_classes = ["smoking", "person"]
    inst._intervention_avoid_classes = {"smoking", "person"}
    inst._nav_in_flight = True
    inst._nav_poi = "POI_02"
    _scan_ready(inst)
    inst._scan_classes = {"smoking", "person"}
    head_cmds = []
    inst._send_head_cmd = lambda cmd: head_cmds.append(cmd)

    def det_msg(bearing):
        return SimpleNamespace(
            detections_json=json.dumps([{
                "cls": "person", "conf": 0.90, "cx": 0.5, "cy": 0.5,
                "bearing_rad": bearing, "range_m": 2.0,
                "range_valid": True, "look_pitch_rad": 0.0,
            }]),
            simulated=False,
        )

    inst._on_detection(det_msg(0.6))
    assert inst._watch_state == "WATCH", inst._watch_state
    assert head_cmds == [], head_cmds

    inst._on_detection(det_msg(0.0))
    assert inst._watch_state == "LOCKED", inst._watch_state
    assert head_cmds and head_cmds[0].startswith("look_at:"), head_cmds


@case
def test_person_intervention_switches_to_smoking_when_seen():
    """人形介入停车后补看到吸烟：改播吸烟警告，而不是「请让一下」。"""
    import threading as _threading
    import time as _time

    import fire_inspection_orchestrator.orchestrator_node as onc

    inst = _bare_orchestrator()
    inst._intervention_snap = {"cls": "person", "audio": False, "conf": 0.8}
    inst._intervention_settle_s = 0.0
    inst._intervention_snapshot_wait_s = 0.0
    inst._person_smoking_grace_s = 0.5
    inst._cooldown_s = 15.0
    inst._transit_watch_on = lambda: None
    inst._stop_event = _threading.Event()
    spoken = []
    inst._speak = lambda text, label: spoken.append(text)
    inst._avoid_target = lambda snap: True
    inst._approach_target = lambda snap: None

    def feed_smoking():
        _time.sleep(0.1)
        inst._smoking_during_person = {
            "cls": "smoking", "conf": 0.82, "bearing_rad": 0.0,
            "bearing_valid": True, "range_m": 2.0, "range_valid": True}

    worker = _threading.Thread(target=feed_smoking, daemon=True)
    worker.start()
    inst._handle_intervention("POI_02", allow_approach=True, count=True)
    worker.join(timeout=1.0)

    assert spoken[0] == onc.SPEECH_TEXTS["smoking"], spoken
    assert onc.SPEECH_TEXTS["smoking_resume"] in spoken, spoken
    assert onc.SPEECH_TEXTS["person"] not in spoken, spoken


@case
def test_handle_intervention_avoid_skip_still_warns():
    """没有有效方位/测距导致绕行跳过：仍要播警告，但不得播「已避开」。"""
    import threading as _threading

    import fire_inspection_orchestrator.orchestrator_node as onc

    inst = _bare_orchestrator()
    inst._intervention_snap = {"cls": "smoking", "audio": False}
    inst._intervention_settle_s = 0.0
    inst._intervention_snapshot_wait_s = 0.0
    inst._cooldown_s = 15.0
    inst._transit_watch_on = lambda: None
    inst._stop_event = _threading.Event()
    spoken = []
    inst._speak = lambda text, label: spoken.append(text)
    inst._avoid_target = lambda snap: False

    inst._handle_intervention("POI_02", allow_approach=True, count=True)

    assert spoken == [onc.SPEECH_TEXTS["smoking"]], spoken


@case
def test_confirm_requires_min_time_span():
    """跨度不足时即使凑满帧数也不确认（防同一瞬间的连续误检）。"""
    inst = _bare_orchestrator()
    # 用只播报类别隔离「预停车」干扰，只验跨度逻辑
    inst._intervention_speak_only_classes = {"smoking"}
    inst._intervention_avoid_classes = set()
    inst._nav_in_flight = True
    inst._nav_poi = "POI_02"
    _scan_ready(inst)
    inst._confirm_min_span_s = 5.0

    for _ in range(2):
        inst._on_detection(_det_msg(conf=0.45))

    assert inst._watch_state == "LOCKED", inst._watch_state
    # 只播报类别不会预停车；跨度不足时同样不得进入确认
    assert inst._intervention_confirmed is False
    assert inst._intervention_event.is_set() is False


@case
def test_confirm_resets_when_box_jumps():
    """框中心跳到画面别处 → 视为换目标，重新从 1 帧开始计。"""
    inst = _bare_orchestrator()
    inst._nav_in_flight = True
    inst._nav_poi = "POI_02"
    _scan_ready(inst)
    inst._confirm_min_span_s = 0.0

    inst._on_detection(_det_msg(conf=0.45, cx=0.5))
    inst._on_detection(_det_msg(conf=0.45, cx=0.9))

    assert inst._watch_streak == 1, inst._watch_streak
    assert inst._watch_anchor == (0.9, 0.5), inst._watch_anchor


@case
def test_navigate_speak_only_never_approaches_or_counts():
    """只停车播报的确认：接近与计数都要关掉，但仍点对点续行。"""
    inst = _bare_orchestrator()
    calls = []
    handled = []

    def fake_call(client, goal, timeout_s, label, abort_event=None, **kwargs):
        calls.append((goal.poi, bool(goal.use_point)))
        if len(calls) == 1:
            abort_event.set()
            inst._speak_only_pending = True
            inst._intervention_confirmed = True
            return _NavResult(False, "cancelled")
        return _NavResult(True, "arrived")

    def fake_handle(poi, allow_approach=True, count=True):
        handled.append((poi, allow_approach, count))

    inst._call = fake_call
    inst._handle_intervention = fake_handle
    inst._navigate("POI_02", is_return=False)

    assert calls == [("POI_02", False), ("POI_02", True)], calls
    assert handled == [("POI_02", False, False)], handled
    assert inst._speak_only_pending is False


# ── 视觉测距 ────────────────────────────────────────────────────────

def _bare_scan_node():
    """构造不连相机/ROS 的持续检测节点，只供深度解码与测距使用。"""
    from fire_inspection_vision.vision_scan_node import VisionScanNode

    inst = object.__new__(VisionScanNode)
    inst._depth_unit = "auto"
    inst._depth_scale = None
    inst._depth_sample_ratio = 0.5
    inst._depth_min_m = 0.3
    inst._depth_max_m = 12.0
    inst._depth_min_valid_ratio = 0.5
    inst._depth_max_spread_m = 1.0
    # 内参未到达的状态：像素偏移走视场角兜底
    inst._cam_fx = None
    inst._cam_fy = None
    inst._cam_w = 0
    inst._cam_h = 0
    return inst


@case
def test_depth_decode_16uc1_auto_millimeters():
    """16UC1 + unit=auto：按中位数量级判定毫米编码，统一换算成米。"""
    import numpy as np
    from types import SimpleNamespace

    inst = _bare_scan_node()
    raw = np.array([[1000, 2000, 0], [3000, 0, 0]], dtype=np.uint16)
    msg = SimpleNamespace(encoding="16UC1", height=2, width=3,
                          step=3 * 2, data=raw.tobytes())
    d = inst._decode_depth(msg)
    assert d.shape == (2, 3), d.shape
    assert _close(d[0, 0], 1.0) and _close(d[0, 1], 2.0) and _close(d[1, 0], 3.0)
    assert _close(inst._depth_scale, 0.001)


@case
def test_depth_decode_float32_keeps_source_intact():
    """32FC1：NaN/inf 清零必须是副本，不能改写 ROS 消息内存。"""
    import numpy as np
    from types import SimpleNamespace

    inst = _bare_scan_node()
    src = np.array([[1.5, np.nan], [np.inf, 2.0]], dtype=np.float32)
    msg = SimpleNamespace(encoding="32FC1", height=2, width=2,
                          step=2 * 4, data=src.tobytes())
    d = inst._decode_depth(msg)
    assert np.isnan(src[0, 1]) and np.isinf(src[1, 0]), "源数组被改写"
    assert _close(d[0, 0], 1.5) and _close(d[1, 1], 2.0)
    assert _close(d[0, 1], 0.0) and _close(d[1, 0], 0.0)


@case
def test_depth_decode_unsupported_encoding():
    """未支持的编码/尺寸非法返回 None，调用方据此标记 range_valid=false。"""
    from types import SimpleNamespace

    inst = _bare_scan_node()
    assert inst._decode_depth(
        SimpleNamespace(encoding="rgb8", height=1, width=1, step=3, data=b"\0" * 3)
    ) is None
    assert inst._decode_depth(
        SimpleNamespace(encoding="32FC1", height=0, width=0, step=0, data=b"")
    ) is None


@case
def test_measure_range_center_patch_and_pitch():
    """只取框中心区域，按头部 pitch 做水平投影；全零/超量程判无效。"""
    import numpy as np

    inst = _bare_scan_node()
    depth = np.full((100, 100), 5.0, dtype=np.float32)
    depth[:50, :] = 100.0        # 上半幅是远处背景，不能被取到
    assert _close(inst._measure_range(depth, 0.5, 0.75, 0.4, 0.4, 0.0), 5.0)
    # 低头 60° → 水平距离 5*cos(60°)=2.5m
    assert _close(inst._measure_range(depth, 0.5, 0.75, 0.4, 0.4,
                                      math.pi / 3), 2.5, 1e-5)
    # 框内全是无效值（0 / 超上限）→ None（不写成 0 距离）
    assert inst._measure_range(np.zeros((20, 20), dtype=np.float32),
                               0.5, 0.5, 0.5, 0.5, 0.0) is None
    far = np.full((20, 20), 50.0, dtype=np.float32)
    assert inst._measure_range(far, 0.5, 0.5, 0.5, 0.5, 0.0) is None


@case
def test_measure_range_rejects_large_depth_spread():
    """框内同时覆盖近处地面和远处目标（离散过大）→ 判无效。"""
    import numpy as np

    inst = _bare_scan_node()
    depth = np.full((100, 100), 1.0, dtype=np.float32)
    depth[:, 50:] = 8.0
    assert inst._measure_range(depth, 0.5, 0.5, 1.0, 1.0, 0.0) is None


@case
def test_audio_sweep_ratio_separates_siren_from_speech():
    """扫频判据：真警报主峰扫频(>1.3)，人声/稳态音主峰固定(≈1.0)。

    数据取自 2026-10-31 现场日志：三种情况的 MFCC 相似度区间重叠
    （真警报 0.866 / 人声 0.884），只能靠主峰变化区分。
    """
    from fire_inspection_audio.audio_scan_node import AudioScanNode

    # 现场外放的真警报（peak 一路升高）
    assert AudioScanNode._sweep_ratio([469, 430, 523, 523, 617]) > 1.30
    # 更典型的一次：859→1703Hz
    assert AudioScanNode._sweep_ratio([859, 977, 1125, 1133, 1641, 1703]) > 1.30
    # 人声：主峰死死停在 617Hz
    assert AudioScanNode._sweep_ratio([617, 617, 617, 617, 617]) < 1.30
    # 样本不足无法判断
    assert AudioScanNode._sweep_ratio([617]) is None
    assert AudioScanNode._sweep_ratio([]) is None


@case
def test_smoking_threshold_only_no_geometry_filter():
    """吸烟恢复为纯阈值判定（2026-10-31 的框几何校验已回退）。

    0.45 及以上的检出全部保留，不再按框尺寸做二次过滤；
    低于 0.45 的候选仍被阈值挡掉。
    """
    from fire_inspection_vision.detect_core import Detector

    det = Detector(model_path="", conf_smoking=0.45, conf_other=0.65)
    real_small = {"cls": "smoking", "conf": 0.60, "w": 0.060, "h": 0.059}
    real_tall = {"cls": "smoking", "conf": 0.76, "w": 0.058, "h": 0.113}
    huge = {"cls": "smoking", "conf": 0.90, "w": 0.80, "h": 0.80}
    assert det.filter([real_small, real_tall, huge]) == [real_small, real_tall, huge]

    # 阈值仍然独立生效
    low = {"cls": "smoking", "conf": 0.40, "w": 0.06, "h": 0.10}
    assert det.filter([low]) == []


@case
def test_camera_info_tolerates_numpy_k():
    """部分驱动的 msg.k 是 numpy 数组，内参日志不得因此崩溃（曾致节点退出）。"""
    import numpy as np
    from types import SimpleNamespace

    inst = _bare_scan_node()
    inst.get_logger = lambda: _NoopLogger()
    inst._camera_info_logged = False
    inst._hfov = math.radians(60.0)
    inst._vfov = math.radians(34.0)
    inst._on_camera_info(SimpleNamespace(
        k=np.array([300.0, 0.0, 320.0, 0.0, 300.0, 240.0, 0.0, 0.0, 1.0]),
        width=640, height=480))
    assert inst._camera_info_logged is True
    # 缺字段 / 全零内参：静默跳过，不抛异常，也不重复尝试
    inst2 = _bare_scan_node()
    inst2.get_logger = lambda: _NoopLogger()
    inst2._camera_info_logged = False
    inst2._on_camera_info(SimpleNamespace(k=[], width=0, height=0))
    assert inst2._camera_info_logged is False
    inst2._on_camera_info(SimpleNamespace(width=640, height=480))
    assert inst2._camera_info_logged is False


@case
def test_pixel_offsets_use_camera_info_then_fov_fallback():
    """内参到达后用 atan 精确换算，缺失时才退回视场角线性近似。"""
    import numpy as np
    from types import SimpleNamespace

    inst = _bare_scan_node()
    inst.get_logger = lambda: _NoopLogger()
    inst._camera_info_logged = False
    inst._hfov = math.radians(85.5)
    inst._vfov = math.radians(55.0)
    # 未到内参：走视场角近似
    assert _close(inst._yaw_offset(0.9), (0.5 - 0.9) * math.radians(85.5))
    assert _close(inst._pitch_offset(0.2), (0.2 - 0.5) * math.radians(55.0))
    inst._on_camera_info(SimpleNamespace(
        k=np.array([692.0, 0.0, 640.0, 0.0, 691.0, 360.0, 0.0, 0.0, 1.0]),
        width=1280, height=720))
    # 内参到达：atan(dx/fx)，比线性近似更贴合真实透视
    expected = math.atan2((0.5 - 0.9) * 1280, 692.0)
    assert _close(inst._yaw_offset(0.9), expected, 1e-9)
    assert inst._yaw_offset(0.5) == 0.0
    assert _close(inst._pitch_offset(0.2),
                  math.atan2((0.2 - 0.5) * 720, 691.0), 1e-9)


@case
def test_look_at_prefers_vision_exact_fields():
    """头部锁定优先用 bearing_rad / look_pitch_rad，逐字段缺失时回退近似。"""
    import json

    import fire_inspection_orchestrator.orchestrator_node as onc

    inst = object.__new__(onc.OrchestratorNode)
    inst._hfov = math.radians(85.5)
    inst._vfov = math.radians(55.0)

    # look_at 输出按 1e-4 rad 取整（够头部关节用，避免下发无意义精度），
    # 因此断言容差取 1e-4，不能用默认的 1e-6
    def _near(a, b):
        return _close(a, b, tol=1e-4)

    got = inst._parse_look_at(json.dumps([{
        "cls": "smoking", "conf": 0.9, "cx": 0.9, "cy": 0.2,
        "bearing_rad": -0.66, "look_pitch_rad": 0.3}]))
    assert _near(got["yaw"], -0.66) and _near(got["pitch"], 0.3), got

    # 两个扩展字段都没有 → 视场角近似（符号：右转/低头为正 pitch）
    got = inst._parse_look_at(json.dumps([{
        "cls": "smoking", "conf": 0.9, "cx": 0.9, "cy": 0.2}]))
    assert _near(got["yaw"], (0.5 - 0.9) * math.radians(85.5)), got
    assert _near(got["pitch"], (0.2 - 0.5) * math.radians(55.0)), got

    # 只有 bearing_rad → yaw 用精确值，pitch 仍走近似
    got = inst._parse_look_at(json.dumps([{
        "conf": 0.9, "cx": 0.9, "cy": 0.2, "bearing_rad": -0.4}]))
    assert _near(got["yaw"], -0.4) and _near(
        got["pitch"], (0.2 - 0.5) * math.radians(55.0)), got

    # 超限位要被夹住；非有限值退回近似
    got = inst._parse_look_at(json.dumps([{
        "conf": 0.9, "cx": 0.5, "cy": 0.5,
        "bearing_rad": 3.0, "look_pitch_rad": 1.2}]))
    assert _near(got["yaw"], math.radians(90.0)) and _near(
        got["pitch"], math.radians(25.0)), got
    got = inst._parse_look_at(json.dumps([{
        "conf": 0.9, "cx": 0.5, "cy": 0.5,
        "bearing_rad": float("nan"), "look_pitch_rad": float("inf")}]))
    assert _near(got["yaw"], 0.0) and _near(got["pitch"], 0.0), got

    # 人形框锁上半身：现场相机 0 位仍看地面，person 要适当抬头，
    # 否则只看到腿；2026-11-02 按现场反馈从 -0.38 下调到 -0.22。
    # smoking 框本身就在上半身，不套这个下限。
    got_person = inst._parse_look_at(json.dumps([{
        "cls": "person", "conf": 0.9, "cx": 0.5, "cy": 0.5, "h": 0.8,
        "bearing_rad": 0.0, "look_pitch_rad": 0.2}]))
    assert got_person["pitch"] <= -0.22, got_person
    got_smoking = inst._parse_look_at(json.dumps([{
        "cls": "smoking", "conf": 0.9, "cx": 0.5, "cy": 0.5, "h": 0.8,
        "bearing_rad": 0.0, "look_pitch_rad": 0.2}]))
    assert abs(got_smoking["pitch"] - 0.2) < 1e-4, got_smoking


@case
def test_to_json_passes_look_pitch_through():
    """detections_json 透传 look_pitch_rad，旧字段顺序不变。"""
    import json

    from fire_inspection_vision.detect_core import Detector

    raw = json.loads(Detector.to_json([{
        "cls": "smoking", "conf": 0.9, "cx": 0.4, "cy": 0.6, "w": 0.2, "h": 0.3,
        "range_m": 4.2, "bearing_rad": 0.12, "look_pitch_rad": 0.21,
        "range_valid": True}], limit=5))
    assert raw[0]["look_pitch_rad"] == 0.21, raw
    assert raw[0]["bearing_rad"] == 0.12 and raw[0]["range_m"] == 4.2
    assert raw[0]["range_valid"] is True
    # 旧 schema（无扩展字段）不得凭空写入扩展键
    old = json.loads(Detector.to_json([{
        "cls": "safety_sign", "conf": 0.7, "cx": 0.5, "cy": 0.5,
        "w": 0.1, "h": 0.1}], limit=5))
    assert set(old[0]) == {"cls", "conf", "cx", "cy", "w", "h"}, old


# ── 介入停车后的续跑（现场：重发导航 167ms 即 failed） ────────────

@case
def test_never_started_detects_instant_reject():
    """现场返回的 failed（success=True/error_code=0/distance_time=0）按瞬态识别。"""
    from fire_inspection_nav.navigate_to_server import _never_started

    field = {
        "request_id": "req-1793192728281-2984", "success": True,
        "error_code": 0, "action_id": "2dfc4f38-a4bc-1e60-6d8c-778c68aa66c1",
        "distance_remaining": 0, "distance_time": 0,
        "status": "failed", "type": "move_to_locations",
    }
    assert _never_started(field) is True
    # 跑了一半才失败：不能当瞬态，必须如实报错
    assert _never_started({**field, "distance_time": 120}) is False
    assert _never_started({**field, "distance_remaining": 3.2}) is False
    # 结构不符时保守处理
    assert _never_started({}) is False
    assert _never_started(None) is False
    assert _never_started("failed") is False


class _FakeGoal:
    is_cancel_requested = False


class _FakeChassis:
    def __init__(self, statuses):
        self._statuses = list(statuses)

    def get_action_status(self, _action_id):
        if self._statuses:
            return self._statuses.pop(0)
        return {"status": "running"}


class _FakeInstance:
    def __init__(self):
        from fire_inspection_nav.navigate_to_server import NavigateToServer

        self._shutting_down = False
        self._action_poll_interval_s = 0.0
        self._chassis = None
        self.cancelled = []
        # _run_chassis_action 里用到的位姿辅助函数（真实实现直接绑定）
        self._pose_xy = NavigateToServer._pose_xy
        self._observed_no_movement = (
            NavigateToServer._observed_no_movement.__get__(self))

    def _cancel_and_settle(self, action_id):
        self.cancelled.append(action_id)

    def _read_pose(self):
        return {}

    def _distance_to(self, _poi, _pose):
        return 0.0

    def _publish_feedback(self, *_args, **_kwargs):
        pass

    def get_logger(self):
        return _NoopLogger()


def _nav_bare(statuses):
    from fire_inspection_nav.navigate_to_server import NavigateToServer

    inst = _FakeInstance()
    inst._chassis = _FakeChassis(statuses)
    inst._run = NavigateToServer._run_chassis_action.__get__(inst)
    return inst


@case
def test_instant_failure_returns_transient():
    """刚下发即 failed 且从未起步 → 返回 transient（交给上层重发），不抛异常。"""
    import time as _time

    inst = _nav_bare([{
        "success": True, "error_code": 0, "distance_remaining": 0,
        "distance_time": 0, "status": "failed",
    }])
    outcome = inst._run(  # type: ignore[attr-defined]
        _FakeGoal(), "a1", "POI_03", "track",
        _time.monotonic() + 30.0, 30.0)
    assert outcome == "transient", outcome
    assert inst.cancelled == ["a1"], inst.cancelled


@case
def test_midway_failure_still_raises():
    """已跑过一段才 failed → 照旧抛异常，不得静默重试。"""
    import time as _time

    inst = _nav_bare([{
        "success": True, "error_code": 0, "distance_remaining": 2.0,
        "distance_time": 42, "status": "failed",
    }])
    try:
        inst._run(  # type: ignore[attr-defined]
            _FakeGoal(), "a2", "POI_03", "track",
            _time.monotonic() + 30.0, 30.0)
    except RuntimeError as exc:
        assert "POI_03" in str(exc)
        return
    raise AssertionError("中途失败必须抛异常")


@case
def test_instant_failure_with_plan_distance_still_transient():
    """现场 2026-10-31：拒绝响应里带整段规划距离（10246mm）/distance_time=4640，
    但我们自己的位姿没有变化 → 仍按瞬态重发，不把整轮巡检判死。"""
    import time as _time

    inst = _nav_bare([{
        "success": True, "error_code": 0, "distance_remaining": 10246,
        "distance_time": 4640, "status": "failed",
    }])
    inst._read_pose = lambda: {"x": 1.0, "y": 2.0, "yaw": 0.0}
    outcome = inst._run(  # type: ignore[attr-defined]
        _FakeGoal(), "a3", "POI_03", "track",
        _time.monotonic() + 30.0, 30.0)
    assert outcome == "transient", outcome


@case
def test_failure_after_observed_movement_is_not_transient():
    """观测到位移后失败 → 不是瞬态，必须如实抛异常。"""
    import time as _time

    inst = _nav_bare([{
        "success": True, "error_code": 0, "distance_remaining": 2.0,
        "distance_time": 42, "status": "failed",
    }])
    poses = iter([
        {"x": 0.0, "y": 0.0, "yaw": 0.0},   # 下发时的起步位置
        {"x": 0.6, "y": 0.0, "yaw": 0.0},   # 失败时已离开 0.6m
    ])
    inst._read_pose = lambda: next(poses, {"x": 0.6, "y": 0.0, "yaw": 0.0})
    try:
        inst._run(  # type: ignore[attr-defined]
            _FakeGoal(), "a4", "POI_03", "track",
            _time.monotonic() + 30.0, 30.0)
    except RuntimeError as exc:
        assert "POI_03" in str(exc)
        return
    raise AssertionError("起步后失败必须抛异常")


def _track_instance(waypoints):
    from fire_inspection_nav.navigate_to_server import NavigateToServer

    inst = _FakeInstance()
    inst._outbound_waypoints = waypoints
    inst._return_waypoints = []
    inst._yaw = NavigateToServer._track_departure_yaw.__get__(inst)
    inst._track_resume_pose = NavigateToServer._track_resume_pose.__get__(inst)
    inst._dispatch_return = NavigateToServer._dispatch_return.__get__(inst)
    inst._track_point_ahead = NavigateToServer._track_point_ahead.__get__(inst)
    inst._track_points = NavigateToServer._track_points
    inst._project_on_track = NavigateToServer._project_on_track
    return inst


@case
def test_track_departure_yaw_between_waypoints():
    """车停在两个航点之间（离最近航点 >1m）也要能给出行进方向。"""
    inst = _track_instance([
        {"position": {"x": 0.0, "y": 0.0}},
        {"position": {"x": 4.0, "y": 0.0}},
        {"position": {"x": 8.0, "y": 0.0}},
    ])
    # 距最近航点 2m，旧实现的 1.0m 上限会返回 None
    assert _close(inst._yaw(False, {"x": 2.0, "y": 0.0, "yaw": 0.0}), 0.0)
    # 车在第二个航点之后，方向仍沿轨道（不是回头）
    assert _close(inst._yaw(False, {"x": 6.5, "y": 0.3, "yaw": 0.0}), 0.0)


@case
def test_track_departure_yaw_follows_segment():
    """拐角处按所在线段取方向，避免拿背后航点算出反向。"""
    inst = _track_instance([
        {"position": {"x": 0.0, "y": 0.0}},
        {"position": {"x": 4.0, "y": 0.0}},
        {"position": {"x": 4.0, "y": 4.0}},
    ])
    # 刚拐过弯，在竖直线段上 → 应朝 +y（pi/2），不是回头朝 -x
    assert _close(inst._yaw(False, {"x": 4.1, "y": 1.0, "yaw": 0.0}),
                  math.pi / 2)


# ── 返程被打断后从当前位置续行（不退回上轨点 POI_04） ──────────────

@case
def test_track_resume_pose_projects_current_position():
    """返程续行点 = 当前位姿在轨道上的最近投影，朝向取该段行进方向。"""
    inst = _track_instance([
        {"position": {"x": 0.0, "y": 0.0}},
        {"position": {"x": 4.0, "y": 0.0}},
        {"position": {"x": 8.0, "y": 0.0}},
    ])
    inst._read_pose = lambda: {"x": 6.0, "y": 0.8, "yaw": 1.0}
    resume = inst._track_resume_pose(inst._outbound_waypoints)
    assert resume is not None, resume
    assert _close(resume["x"], 6.0) and _close(resume["y"], 0.0), resume
    assert _close(resume["yaw"], 0.0), resume

    inst._read_pose = lambda: {"x": 6.0, "y": 0.8, "yaw": 1.0}
    reverse = list(reversed(inst._outbound_waypoints))
    resume2 = inst._track_resume_pose(reverse)
    assert _close(abs(resume2["yaw"]), math.pi), resume2


class _ReturnChassis:
    """记录返程下发方式：move_to_locations / move_to_locations_poi。"""

    def __init__(self):
        self.calls = []

    def move_to_locations(self, track_id, start_position=None,
                          target_position=None, uptrack_enabled=None):
        self.calls.append(("locations", track_id, start_position,
                           target_position, uptrack_enabled))
        return {"action_id": "a1"}

    def move_to_locations_poi(self, track_id, poi_name, uptrack_enabled=None):
        self.calls.append(("locations_poi", track_id, poi_name, uptrack_enabled))
        return {"action_id": "a2"}


@case
def test_return_resume_uses_track_projection():
    """被打断后重发返程：带当前轨道投影作起点续行，底盘不会回 POI_04。"""
    inst = _track_instance([
        {"position": {"x": 0.0, "y": 0.0}},
        {"position": {"x": 8.0, "y": 0.0}},
    ])
    inst._return_track_id = 1001
    inst._pois = {"start_point": {"x": 0.0, "y": 0.0, "yaw": 0.0}}
    inst._return_waypoints = [
        {"position": {"x": 8.0, "y": 0.0}},
        {"position": {"x": 0.0, "y": 0.0}},
    ]
    inst._read_pose = lambda: {"x": 6.0, "y": 0.8, "yaw": 0.0}
    inst._return_interrupted = True
    chassis = _ReturnChassis()
    inst._chassis = chassis

    rv = inst._dispatch_return("start_point")

    assert rv["action_id"] == "a1", rv
    # 必须显式带起点，否则底盘按上轨点推算进度会拒绝/把车带回 POI_04
    assert len(chassis.calls) == 1, chassis.calls
    kind, track_id, start_position, target_position, uptrack = chassis.calls[0]
    assert kind == "locations" and track_id == 1001, chassis.calls
    assert uptrack is False, chassis.calls
    assert _close(start_position["x"], 6.0) and _close(start_position["y"], 0.0), (
        chassis.calls)
    assert _close(abs(start_position["yaw"]), math.pi), chassis.calls
    assert _close(target_position["x"], 0.0) and _close(target_position["y"], 0.0), (
        chassis.calls)


@case
def test_return_first_dispatch_keeps_uptrack_behavior():
    """首次返程（未被中断）仍走原 move_to_locations_poi(uptrack=True)。"""
    inst = _track_instance([])
    inst._return_track_id = 1001
    inst._return_waypoints = []
    inst._return_interrupted = False
    chassis = _ReturnChassis()
    inst._chassis = chassis

    rv = inst._dispatch_return("start_point")

    assert rv["action_id"] == "a2", rv
    assert chassis.calls == [("locations_poi", 1001, "start_point", True)], (
        chassis.calls)


class _NullLog:
    def info(self, _msg):
        pass

    def warn(self, _msg):
        pass

    def error(self, _msg):
        pass


class _Goal:
    is_cancel_requested = False


@case
def test_return_resume_point_is_ahead_on_track():
    """重接点沿返程轨道从投影点向前推，不回到刚避让的位置。"""
    inst = _track_instance([])
    inst._return_waypoints = [
        {"position": {"x": 8.0, "y": 0.0}},
        {"position": {"x": 0.0, "y": 0.0}},
    ]
    inst._read_pose = lambda: {"x": 6.0, "y": 0.8, "yaw": 0.0}
    ahead = inst._track_point_ahead(inst._return_waypoints, 1.5)
    assert ahead is not None, ahead
    # 投影点 (6, 0) → 沿轨道（指向 x=0）前进 1.5m → (4.5, 0)
    assert _close(ahead["x"], 4.5) and _close(ahead["y"], 0.0), ahead
    assert _close(abs(ahead["yaw"]), math.pi), ahead


@case
def test_pois_yaw_override_for_poi03():
    """POI_03 到点朝向覆盖为 -90°（面向走廊一侧）。"""
    cfg = _yaml(POIS_YAML)
    overrides = cfg.get("yaw_overrides") or {}
    assert "POI_03" in overrides, overrides
    assert _close(float(overrides["POI_03"]), -math.pi / 2, 1e-3), overrides


# ── 运行器 ──────────────────────────────────────────────────────────

def main() -> int:
    passed, failed = [], []
    for fn in TESTS:
        try:
            fn()
            passed.append(fn.__name__)
            print(f"[PASS] {fn.__name__}")
        except Exception as exc:
            failed.append(fn.__name__)
            print(f"[FAIL] {fn.__name__}: {exc}")
            import traceback
            traceback.print_exc()
    print(f"\n=== 结果: {len(passed)}/{len(TESTS)} 通过 ===")
    if failed:
        print("失败:", ", ".join(failed))
    return 0 if not failed else 1


if __name__ == "__main__":
    sys.exit(main())
