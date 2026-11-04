"""消防巡检导航服务端。

实现范围：
- 启动时通过底盘 REST 读取 POI 表、轨道信息、定位状态与底盘健康，做前置校验；
- `dry_run:=true`：只读校验 + 模拟到点，不下发任何运动指令，也不写限速；
- `dry_run:=false`：实机运动（首点点对点、后续沿轨道、到点按 POI yaw 转向），
  含 AEB、动作状态轮询、取消/超时、限速备份与回滚、定位与健康严格校验。

只使用底盘 SDK（KnewbotChassisNav），不初始化 robot_controller 门面，
避免 dry_run 阶段连带拉起手臂 / 手部 / 相机等与导航无关的硬件。
导航语义与 /home/nvidia/robot_controller/src/chassis/run_poi_sequence.py 一致：
第一个点走点对点，后续点走唯一轨道，到点后转向该点 yaw。
"""

from __future__ import annotations

import math
import os
import sys
import threading
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
import yaml

from fire_inspection_interfaces.action import ApproachTarget, NavigateTo

# robot_controller 是普通 Python 包，未被 colcon 安装，需手动注入路径
_ROBOT_CONTROLLER_SRC = os.environ.get(
    "ROBOT_CONTROLLER_SRC", "/home/nvidia/robot_controller/src"
)
if _ROBOT_CONTROLLER_SRC not in sys.path:
    sys.path.insert(0, _ROBOT_CONTROLLER_SRC)

from robot_controller.chassis import KnewbotChassisNav  # noqa: E402


def _log_at(logger, level: str, msg: str) -> None:
    """ROS 2 Jazzy 的 rcutils_logger 用 (filename, lineno, func) 标识每个日志调用点，
    同一行号两次用不同 severity 会抛 ValueError('Logger severity cannot be changed
    between calls')。这里用显式 if-elif 分发，每个 severity 落在不同行号上，避免冲突。
    替代原来的 getattr(logger, level)(msg) 动态选择。
    """
    if level == "debug":
        logger.debug(msg)
    elif level == "info":
        logger.info(msg)
    elif level == "warn":
        logger.warn(msg)
    elif level == "error":
        logger.error(msg)
    elif level == "fatal":
        logger.fatal(msg)
    else:
        logger.info(msg)


def _unwrap_list(rv, *keys):
    """容错：REST 返回可能是 {"<key>": [...]} / dict 包 list / 直接 list。"""
    if rv is None:
        return []
    if isinstance(rv, list):
        return rv
    if isinstance(rv, dict):
        for k in keys:
            v = rv.get(k)
            if isinstance(v, list):
                return v
    return []


def is_blocking_dtc(entry) -> bool:
    """DTC 是否必须阻止运动。

    底盘健康接口的 hasFatal/hasError 是聚合标志：只要有 C 类告警
    （例如「WiFi 未连接」）也会把 hasFatal 置 True。真正决定能不能动的
    是每条故障的 severity：
        A类故障(FATAL) / B类故障(ERROR) -> 阻断
        C类故障(WARNING)                -> 只告警，不阻断（如 WiFi 未连接）
    没有 severity 字段的老固件保持保守策略，视为阻断。
    """
    sev = str(getattr(entry, "severity", "") or "").strip().upper()
    if not sev:
        return True
    return any(tag in sev for tag in ("FATAL", "ERROR", "A类", "B类"))


def split_health_errors(entries):
    """按严重程度把 DTC 分成 (阻断, 仅告警) 两组，便于日志区分。"""
    blocking, warnings = [], []
    for entry in entries or []:
        (blocking if is_blocking_dtc(entry) else warnings).append(entry)
    return blocking, warnings


def describe_dtc(entry) -> str:
    """单条 DTC 的日志/异常文案：ID + 严重程度 + 描述 + 处理建议。"""
    try:
        dtc_id = f"0x{int(getattr(entry, 'id', 0)):08X}"
    except (TypeError, ValueError):
        dtc_id = "0x????????"
    severity = str(getattr(entry, "severity", "") or "").strip()
    desc = str(getattr(entry, "description", "") or "").strip()
    solution = str(getattr(entry, "solution", "") or "").strip()
    head = " ".join(x for x in (dtc_id, severity, desc) if x)
    return f"{head}（处理：{solution}）" if solution else head


# 底盘动作终态判定（兼容不同固件返回的字段与措辞，与 run_POI.py 保持一致）
_TERMINAL_SUCCESS = {
    "successed", "succeeded", "success", "finished", "done", "completed",
}
_TERMINAL_FAIL = {
    "failed", "failure", "error",
    "cancelled", "canceled", "stopped", "aborted", "rejected",
}

# 只有这些定位状态的异常才主动下发重定位：
# 1=丢失 / 2=精度下降 / 3=异常 / 5=等待重定位；
# 6=重定位中、7=加载地图、8=等待传感器属于底盘正在恢复，重复下发只会互相打断
_RELOCALIZE_TRIGGER_STATUS = frozenset({1, 2, 3, 5})

# 轨道段出发前朝向对齐的容差：小于该角度不再原地补转
_TRAVEL_ALIGN_TOLERANCE_RAD = 0.15

# 点对点段出发前朝向对齐：距目标太近（无有效方向）时跳过
_POINT_ALIGN_MIN_DIST_M = 0.4

# 接近位移小于该值时不再前后移动（已在期望距离附近），只保证朝向目标
_APPROACH_MIN_TRAVEL_M = 0.3
# 近距离侧向退让时，退让点距可行走廊边界至少要留出的余量（米）。
# 现场 2026-11-02：首选右侧退让点 y=3.39，而走廊内缩后上限 3.50，
# 只余 0.11m，车看起来在往墙上贴；低于该余量就改走另一侧。
_RETREAT_MIN_CLEARANCE_M = 0.20

# 动作刚下发就被底盘判 failed 且从未起步（distance_time=0）的观察窗口：
# 这通常是「取消后重发」撞上固件停稳过渡期的瞬态拒绝，值得重发一次，
# 不能因为一次拒绝就把整轮巡检判失败。
_INSTANT_FAILURE_S = 1.5


class _ActionFailed(RuntimeError):
    """底盘动作返回失败终态（区别于超时/取消，允许重发一次）。"""


# 轨道重发仍被拒时，若距目标已在这个范围内，允许退回点对点兜底，
# 避免最后几米因轨道状态问题永久卡死（远处不改用点对点，防止跨区乱窜）。
_POINT_FALLBACK_MAX_M = 5.0

# 返程轨道接口只接受从「上轨点」(POI_04) 续行，传入轨道中间的投影起点会被
# 底盘 0.11s 即拒绝（2026-11-03 实测）。返程中途续行被拒时，沿返程轨道按
# 不超过该距离分段做点对点推进，绕开这个固件限制。
_RETURN_STAGE_M = 4.0
# 返程分段推进时保持介入后的横向偏移上限（米）。偏移再大就贴着走廊
# 边界了，宁可回到轨道中心线。
_RETURN_OFFSET_MAX_M = 1.0
# 每段点对点动作的等待上限：4m 按 0.4m/s + 起步/停稳余量取 30s。
_RETURN_STAGE_TIMEOUT_S = 30.0


def _never_started(status) -> bool:
    """底盘返回 failed 时，判断这条动作是否「从未起步」。

    固件在取消过渡期拒绝新任务时会立刻返回 failed，且距离/耗时都是 0；
    真正跑了一半才失败的动作这些字段非 0，不能按瞬态重试。
    """
    if not isinstance(status, dict):
        return False
    seen = False
    for key in ("distance_time", "distance_remaining", "progress"):
        if key not in status:
            continue
        seen = True
        try:
            if float(status[key]) != 0.0:
                return False
        except (TypeError, ValueError):
            return False
    return seen


def load_stall_config(config) -> dict:
    """解析并校验 pois.yaml 的 `stall`（受阻自恢复）配置。

    返回统一字段的 dict；配置非法或出现未知键一律抛 RuntimeError，
    由节点启动时拒绝启动，不做静默降级。
    """
    stall_cfg = (config or {}).get("stall") or {}
    if not isinstance(stall_cfg, dict):
        raise RuntimeError("stall 必须是映射")
    unknown = set(stall_cfg) - {
        "enabled", "detect_window_s", "min_progress_m", "backoff_m",
        "backoff_speed_mps", "backoff_timeout_s", "max_recoveries",
        "detour_offsets_m", "detour_timeout_s",
        "retreat_rotate_deg", "retreat_rotate_speed_rad_s",
        "retreat_rotate_timeout_s",
    }
    if unknown:
        raise RuntimeError(f"stall 存在未知键: {sorted(unknown)}")
    raw_offsets = stall_cfg.get("detour_offsets_m", [1.0, 1.5])
    if not isinstance(raw_offsets, (list, tuple)) or not raw_offsets:
        raise RuntimeError(f"stall.detour_offsets_m 必须是非空列表: {raw_offsets!r}")
    try:
        offsets = [float(v) for v in raw_offsets]
    except (TypeError, ValueError):
        raise RuntimeError(f"stall.detour_offsets_m 含非数值: {raw_offsets!r}")
    for value in offsets:
        if not math.isfinite(value) or not 0.3 <= value <= 3.0:
            raise RuntimeError(
                f"stall.detour_offsets_m 元素非法: {value}（应在 [0.3, 3.0]）")
    if len(offsets) > 3:
        raise RuntimeError(
            f"stall.detour_offsets_m 最多 3 个: {offsets}")
    values = {
        "enabled": bool(stall_cfg.get("enabled", True)),
        "detect_window_s": float(stall_cfg.get("detect_window_s", 8.0)),
        "min_progress_m": float(stall_cfg.get("min_progress_m", 0.10)),
        "backoff_m": float(stall_cfg.get("backoff_m", 0.5)),
        "backoff_speed_mps": float(stall_cfg.get("backoff_speed_mps", 0.2)),
        "backoff_timeout_s": float(stall_cfg.get("backoff_timeout_s", 5.0)),
        "max_recoveries": int(stall_cfg.get("max_recoveries", 2)),
        "detour_offsets_m": offsets,
        "detour_timeout_s": float(stall_cfg.get("detour_timeout_s", 30.0)),
        "retreat_rotate_deg": float(
            stall_cfg.get("retreat_rotate_deg", 45.0)),
        "retreat_rotate_speed_rad_s": float(
            stall_cfg.get("retreat_rotate_speed_rad_s", 0.30)),
        "retreat_rotate_timeout_s": float(
            stall_cfg.get("retreat_rotate_timeout_s", 8.0)),
    }
    for label, lower, upper in (
        ("detect_window_s", 2.0, 60.0),
        ("min_progress_m", 0.01, 1.0),
        ("backoff_m", 0.05, 5.0),
        ("backoff_speed_mps", 0.0, 0.5),
        ("backoff_timeout_s", 1.0, 120.0),
        ("detour_timeout_s", 3.0, 120.0),
        ("retreat_rotate_deg", 10.0, 180.0),
        ("retreat_rotate_speed_rad_s", 0.0, 1.0),
        ("retreat_rotate_timeout_s", 2.0, 60.0),
    ):
        value = values[label]
        if not math.isfinite(value) or not lower <= value <= upper:
            raise RuntimeError(
                f"stall.{label} 非法: {value}（应在 [{lower}, {upper}]）")
    # 0 在底盘 move_by 里表示「不限速」，后退绝不能不限速
    if values["backoff_speed_mps"] <= 0.0:
        raise RuntimeError(
            f"stall.backoff_speed_mps 非法: {values['backoff_speed_mps']}"
            f"（应在 (0, 0.5]）")
    # 角速度 0 在底盘接口里同样表示「不限速」，转向也必须限速
    if values["retreat_rotate_speed_rad_s"] <= 0.0:
        raise RuntimeError(
            "stall.retreat_rotate_speed_rad_s 非法: "
            f"{values['retreat_rotate_speed_rad_s']}（应在 (0, 1.0]）")
    if not 0 <= values["max_recoveries"] <= 3:
        raise RuntimeError(
            f"stall.max_recoveries 非法: {values['max_recoveries']}")
    return values


def compute_approach_geometry(pose, bearing_rad, range_m, standoff_m,
                              side_offset_m=0.0):
    """把「目标相对车头的方位角 + 距离」换算成地图上的接近点与朝向。

    约定（与视觉层一致）：bearing_rad 为车头系角度，+左 / -右；
    底盘 yaw 为地图系，+逆时针。因此目标方向 = pose.yaw + bearing_rad。

    side_offset_m 让站位点相对视线侧移：>0 停在目标左侧、<0 右侧（沿视线
    方向的垂直方向），用于从吸烟者侧向走开而不是正面靠近；0 表示原行为。

    返回 dict：
      person_x/person_y    估计的目标位置（地图系）
      approach_x/approach_y 接近点（沿视线、距目标 standoff_m，可侧向偏移）
      yaw                   到点后朝向目标的绝对朝向（rad，归一化到 [-pi, pi]）
      travel_m              本次需要移动的直线距离（m，>=0）

    pose 必须含有限的 x/y/yaw；非法时抛 ValueError，由调用方转成失败原因。
    """
    px, py, pyaw = (float(pose[k]) for k in ("x", "y", "yaw"))
    bearing = float(bearing_rad)
    distance = float(range_m)
    standoff = float(standoff_m)
    side = float(side_offset_m)
    for label, value in (("x", px), ("y", py), ("yaw", pyaw),
                         ("bearing_rad", bearing), ("range_m", distance),
                         ("standoff_m", standoff), ("side_offset_m", side)):
        if not math.isfinite(value):
            raise ValueError(f"{label} 非有限值: {value}")
    # 目标方向不归一化，直接用于 cos/sin，避免 ±pi 处的跳变
    direction = pyaw + bearing
    dx, dy = math.cos(direction), math.sin(direction)
    travel = max(0.0, distance - standoff)
    person_x = px + distance * dx
    person_y = py + distance * dy
    # 侧向偏移沿视线的左侧 (-dy, dx)：>0 左、<0 右
    approach_x = px + travel * dx - side * dy
    approach_y = py + travel * dy + side * dx
    if side == 0.0:
        yaw = math.atan2(math.sin(direction), math.cos(direction))
    else:
        yaw = math.atan2(person_y - approach_y, person_x - approach_x)
    return {
        "person_x": person_x,
        "person_y": person_y,
        "approach_x": approach_x,
        "approach_y": approach_y,
        "yaw": yaw,
        "travel_m": math.hypot(approach_x - px, approach_y - py),
        "side_offset_m": side,
    }


def select_motion_mode(poi, is_return, use_point, start_poi, transit_pois):
    """选择本段的导航方式：point（点对点）或 track（沿轨道）。

    - 返程必须走返程轨道（跨全图点对点规划会失败）；
    - use_point=True 为调用方显式要求点对点（吸烟介入后从人员侧方直接
      前往点位，避免重新上轨后又从人员身上碾过去）；
    - 起点与过点点位默认点对点，巡检点默认沿轨道。
    """
    if is_return:
        return "track"
    if use_point:
        return "point"
    if poi == start_poi or poi in transit_pois:
        return "point"
    return "track"


def _classify_action_status(st) -> str:
    """把底盘动作状态分类为 success / failure / running。"""
    if not isinstance(st, dict):
        return "running"
    if st.get("done") is True or st.get("is_finished") is True:
        return "success"
    for key in ("status", "state", "action_state"):
        val = st.get(key)
        if isinstance(val, str):
            v = val.lower()
            if v in _TERMINAL_SUCCESS:
                return "success"
            if v in _TERMINAL_FAIL:
                return "failure"
    return "running"


def _describe(rv) -> str:
    """给 REST 原始返回一个简短描述，避免结构不符被误读成"没有数据"。"""
    if isinstance(rv, dict):
        return f"dict keys={sorted(rv.keys())}"
    return f"{type(rv).__name__}"


class NavigateToServer(Node):
    """NavigateTo Action 服务端（dry_run 只读）。"""

    def __init__(self) -> None:
        super().__init__("fire_navigate_to")

        self.declare_parameter("host", "192.168.11.10")
        self.declare_parameter("rest_port", 9090)
        self.declare_parameter("request_timeout", 10)
        self.declare_parameter("config_file", "")
        self.declare_parameter("dry_run", True)
        self.declare_parameter("dry_run_delay_s", 0.5)
        self.declare_parameter("preflight_retry_interval_s", 5.0)
        self.declare_parameter("max_speed_mps", 0.4)
        self.declare_parameter("speed_apply_timeout_s", 5.0)
        self.declare_parameter("action_poll_interval_s", 0.5)

        self._dry_run = bool(self.get_parameter("dry_run").value)
        self._dry_run_delay_s = float(self.get_parameter("dry_run_delay_s").value)
        self._preflight_retry_interval_s = float(
            self.get_parameter("preflight_retry_interval_s").value
        )
        self._max_speed_mps = float(self.get_parameter("max_speed_mps").value)
        self._speed_apply_timeout_s = float(
            self.get_parameter("speed_apply_timeout_s").value
        )
        self._action_poll_interval_s = float(
            self.get_parameter("action_poll_interval_s").value
        )
        # 限速原值备份：写入前读取，退出/失败必须回滚，禁止猜默认值
        self._speed_backup = None
        self._speed_limit_applied = False

        config = self._load_config()
        self._start_poi = str(config.get("start_poi", "start_point"))
        self._poi_order = [str(name) for name in config.get("poi_order", [])]
        timeouts = config.get("timeouts") or {}
        self._default_timeout_s = float(timeouts.get("per_poi_s", 180.0))
        self._start_timeout_s = float(timeouts.get("to_start_s", 180.0))
        self._poi_source = str(config.get("poi_source", "chassis"))
        self._range_limit = config.get("range_limit") or {}
        # 人员接近/绕行落点的可行走廊（现场实测可行驶区域）：
        # 只约束临时落点，不约束 POI；落点越界视为不可达，宁可不接近，
        # 也不下发必然被底盘规划拒绝的目标（曾出现退让点 y 落在走廊外，
        # 底盘直接返回 failed，现场表现为「看到人却没有任何避让」）。
        self._corridor = config.get("corridor") or {}
        # 启动即校验格式，避免配错了要到介入时才暴露
        self._corridor_bounds()
        self._dwell_s = float(config.get("dwell_s", 5.0))
        self._yaw_overrides = {
            str(name): float(yaw)
            for name, yaw in (config.get("yaw_overrides") or {}).items()
        }
        if not math.isfinite(self._dwell_s) or self._dwell_s < 0:
            raise RuntimeError(f"dwell_s 非法: {self._dwell_s}")
        self._dwell_overrides = {
            str(name): float(sec)
            for name, sec in (config.get("dwell_overrides") or {}).items()
        }
        for name, sec in self._dwell_overrides.items():
            if not math.isfinite(sec) or sec < 0:
                raise RuntimeError(f"dwell_overrides[{name}] 非法: {sec}")
        for name, yaw in self._yaw_overrides.items():
            if not math.isfinite(yaw) or not -math.pi <= yaw <= math.pi:
                raise RuntimeError(f"yaw_overrides[{name}] 非法: {yaw}")
        self._arrival_tolerance_m = float(config.get("arrival_tolerance_m", 0.5))
        if (
            not math.isfinite(self._arrival_tolerance_m)
            or self._arrival_tolerance_m <= 0
        ):
            raise RuntimeError(f"arrival_tolerance_m 非法: {self._arrival_tolerance_m}")

        # 受阻自恢复：底盘 AEB 只会刹停、没有绕障/重规划接口，长时间无位移
        # 只能由导航层取消动作、后退一小段再重发原目标；仍受阻则本段失败，
        # 由编排层跳过该点，不在这里做绕障或禁行区
        stall = load_stall_config(config)
        self._stall_enabled = stall["enabled"]
        self._stall_window_s = stall["detect_window_s"]
        self._stall_min_progress_m = stall["min_progress_m"]
        self._stall_backoff_m = stall["backoff_m"]
        self._stall_backoff_speed_mps = stall["backoff_speed_mps"]
        self._stall_backoff_timeout_s = stall["backoff_timeout_s"]
        self._stall_max_recoveries = stall["max_recoveries"]
        self._stall_detour_offsets_m = stall["detour_offsets_m"]
        self._stall_detour_timeout_s = stall["detour_timeout_s"]
        # 后退失败（后方也堵死）时的脱困转向参数
        self._stall_rotate_deg = stall["retreat_rotate_deg"]
        self._stall_rotate_speed = stall["retreat_rotate_speed_rad_s"]
        self._stall_rotate_timeout_s = stall["retreat_rotate_timeout_s"]
        # 左右交替转向：避免每次往同一侧拧
        self._stall_rotate_turns = 0
        # 本轮 goal 是否发生过侧向绕行（用于到点转向与结果说明）
        self._goal_detoured = False

        # 人员接近（ApproachTarget）：安全边界全部留在导航配置里，
        # 编排层只说明「要接近谁」，不重复写一套距离阈值
        approach_cfg = config.get("approach") or {}
        if not isinstance(approach_cfg, dict):
            raise RuntimeError("approach 必须是映射")
        unknown_approach_keys = set(approach_cfg) - {
            "standoff_m", "min_range_m", "max_range_m", "max_travel_m",
            "arrival_tolerance_m", "timeout_s", "speed_mps",
            "side_offset_m", "retreat_offset_m",
        }
        if unknown_approach_keys:
            raise RuntimeError(
                f"approach 存在未知键: {sorted(unknown_approach_keys)}")
        self._approach_standoff_m = float(approach_cfg.get("standoff_m", 2.0))
        self._approach_side_offset_m = float(
            approach_cfg.get("side_offset_m", 1.2))
        self._approach_retreat_m = float(
            approach_cfg.get("retreat_offset_m", 1.2))
        self._approach_min_range_m = float(
            approach_cfg.get("min_range_m", self._approach_standoff_m))
        self._approach_max_range_m = float(
            approach_cfg.get("max_range_m", 10.0))
        self._approach_max_travel_m = float(
            approach_cfg.get("max_travel_m", 8.0))
        self._approach_tolerance_m = float(
            approach_cfg.get("arrival_tolerance_m", 0.8))
        self._approach_timeout_s = float(
            approach_cfg.get("timeout_s", 60.0))
        self._approach_speed_mps = float(
            approach_cfg.get("speed_mps", self._max_speed_mps))
        for label, value, lower, upper in (
            ("standoff_m", self._approach_standoff_m, 0.0, 10.0),
            ("min_range_m", self._approach_min_range_m, 0.0, 30.0),
            ("max_range_m", self._approach_max_range_m, 0.0, 30.0),
            ("max_travel_m", self._approach_max_travel_m, 0.0, 30.0),
            ("arrival_tolerance_m", self._approach_tolerance_m, 0.0, 5.0),
            ("timeout_s", self._approach_timeout_s, 0.0, 600.0),
        ):
            if not math.isfinite(value) or not lower < value <= upper:
                raise RuntimeError(
                    f"approach.{label} 非法: {value}（应在 ({lower}, {upper}]）")
        if self._approach_min_range_m > self._approach_max_range_m:
            raise RuntimeError(
                f"approach.min_range_m({self._approach_min_range_m}) 不能大于 "
                f"max_range_m({self._approach_max_range_m})")
        if (
            not math.isfinite(self._approach_speed_mps)
            or not 0 < self._approach_speed_mps <= 1.5
        ):
            raise RuntimeError(
                f"approach.speed_mps 非法: {self._approach_speed_mps}"
                f"（应在 (0, 1.5]）")
        # 站位侧向偏移：±3m 以内，过大会把车贴近墙体/离开可行区域
        if (
            not math.isfinite(self._approach_side_offset_m)
            or abs(self._approach_side_offset_m) > 3.0
        ):
            raise RuntimeError(
                f"approach.side_offset_m 非法: {self._approach_side_offset_m}"
                f"（应在 [-3.0, 3.0]）")
        # 近距离侧向退让距离：0 表示不做退让（退回原来的原地不接近）
        if (
            not math.isfinite(self._approach_retreat_m)
            or not 0.0 <= self._approach_retreat_m <= 3.0
        ):
            raise RuntimeError(
                f"approach.retreat_offset_m 非法: {self._approach_retreat_m}"
                f"（应在 [0.0, 3.0]）")
        speed_cfg = config.get("speed") or {}
        if not isinstance(speed_cfg, dict):
            raise RuntimeError("speed 必须是 {outbound_mps: 值, return_mps: 值}")
        unknown_speed_keys = set(speed_cfg) - {"outbound_mps", "return_mps"}
        if unknown_speed_keys:
            raise RuntimeError(f"speed 存在未知键: {sorted(unknown_speed_keys)}")
        self._outbound_speed = float(
            speed_cfg.get("outbound_mps", self._max_speed_mps)
        )
        self._return_speed = float(speed_cfg.get("return_mps", self._max_speed_mps))
        for label, value in (
            ("outbound_mps", self._outbound_speed),
            ("return_mps", self._return_speed),
        ):
            if not math.isfinite(value) or not 0 < value <= 1.5:
                raise RuntimeError(f"speed.{label} 非法: {value}（应在 (0, 1.5]）")

        tracks_cfg = config.get("tracks") or {}
        if not isinstance(tracks_cfg, dict):
            raise RuntimeError("tracks 必须是 {outbound: 名称, return: 名称}")
        unknown_track_keys = set(tracks_cfg) - {"outbound", "return"}
        if unknown_track_keys:
            raise RuntimeError(f"tracks 存在未知键: {sorted(unknown_track_keys)}")
        self._outbound_track_name = str(tracks_cfg.get("outbound") or "")
        self._return_track_name = str(tracks_cfg.get("return") or "")
        self._transit_pois = {
            str(name) for name in (config.get("transit_pois") or [])
        }
        if not self._poi_order:
            raise RuntimeError(f"配置缺少 poi_order: {self._config_path}")

        # 自动重定位：仅实机模式生效，dry_run 永不触发
        reloc_cfg = config.get("relocalize") or {}
        if not isinstance(reloc_cfg, dict):
            raise RuntimeError("relocalize 必须是映射")
        unknown_reloc_keys = set(reloc_cfg) - {
            "enabled", "timeout_s", "poll_interval_s", "max_time_ms", "pose"
        }
        if unknown_reloc_keys:
            raise RuntimeError(f"relocalize 存在未知键: {sorted(unknown_reloc_keys)}")
        self._relocalize_enabled = bool(reloc_cfg.get("enabled", False))
        self._relocalize_timeout_s = float(reloc_cfg.get("timeout_s", 60.0))
        if (
            not math.isfinite(self._relocalize_timeout_s)
            or not 0 < self._relocalize_timeout_s <= 600
        ):
            raise RuntimeError(
                f"relocalize.timeout_s 非法: {self._relocalize_timeout_s}"
                f"（应在 (0, 600]）"
            )
        self._relocalize_poll_s = float(reloc_cfg.get("poll_interval_s", 0.5))
        if (
            not math.isfinite(self._relocalize_poll_s)
            or not 0.05 <= self._relocalize_poll_s <= 5.0
        ):
            raise RuntimeError(
                f"relocalize.poll_interval_s 非法: {self._relocalize_poll_s}"
                f"（应在 [0.05, 5.0]）"
            )
        self._relocalize_max_time_ms = reloc_cfg.get("max_time_ms")
        if self._relocalize_max_time_ms is not None:
            self._relocalize_max_time_ms = int(self._relocalize_max_time_ms)
            if self._relocalize_max_time_ms <= 0:
                raise RuntimeError(
                    f"relocalize.max_time_ms 非法: {self._relocalize_max_time_ms}"
                )
        self._relocalize_pose_override = None
        pose_cfg = reloc_cfg.get("pose")
        if pose_cfg is not None:
            if not isinstance(pose_cfg, dict):
                raise RuntimeError("relocalize.pose 必须是 {x, y, yaw}")
            missing_pose = [k for k in ("x", "y", "yaw") if pose_cfg.get(k) is None]
            if missing_pose:
                raise RuntimeError(f"relocalize.pose 缺少字段: {missing_pose}")
            self._relocalize_pose_override = {
                k: float(pose_cfg[k]) for k in ("x", "y", "yaw")
            }
            for key, value in self._relocalize_pose_override.items():
                if not math.isfinite(value):
                    raise RuntimeError(f"relocalize.pose.{key} 非法: {value}")
            yaw = self._relocalize_pose_override["yaw"]
            if not -math.pi <= yaw <= math.pi:
                raise RuntimeError(f"relocalize.pose.yaw 非法: {yaw}")

        self._chassis = KnewbotChassisNav(
            host=str(self.get_parameter("host").value),
            rest_port=int(self.get_parameter("rest_port").value),
            request_timeout=int(self.get_parameter("request_timeout").value),
        )
        self._map_name = None

        # 顺序状态：未到起点前不能直接要 POI
        self._started = False
        self._returned = False
        self._next_index = 0
        self._seq_lock = threading.Lock()
        # 返程被打断（介入停车/取消）：重发时从当前轨道投影继续，
        # 不再让 uptrack 把车拉回上轨点 POI_04
        self._return_interrupted = False

        # 同一时刻只允许一个 goal：goal_callback 拿不到锁即拒绝
        self._goal_lock = threading.Lock()
        self._shutting_down = False

        # 前置校验失败不阻塞节点启动，但会拒绝所有 goal；
        # 失败后按 preflight_retry_interval_s 允许重试，避免底盘晚启动就要重启节点
        self._preflight_error = None
        self._last_preflight_at = 0.0
        self._preflight_lock = threading.Lock()
        self._preflight_running = False
        self._pois = {}
        self._pois_raw_desc = ""
        self._track_id = None
        self._track_name = None
        self._return_track_id = None
        # 轨道航点缓存：用于轨道段出发前把车头对齐到行进方向
        self._outbound_waypoints = []
        self._return_waypoints = []
        self._preflight()
        self._last_preflight_at = time.monotonic()

        # ReentrantCallbackGroup + 多线程 executor：保证 execute 阻塞时
        # cancel 请求仍能被处理（默认互斥组 + 单线程会让取消完全失效）
        self._action_server = ActionServer(
            self,
            NavigateTo,
            "/fire_inspection/navigate_to",
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=ReentrantCallbackGroup(),
        )
        # 人员接近与导航共用同一个节点 / 同一把 _goal_lock：
        # 只有导航模块能碰底盘，两个 Action 不会同时驱动底盘
        self._approach_server = ActionServer(
            self,
            ApproachTarget,
            "/fire_inspection/approach_target",
            execute_callback=self._execute_approach,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=ReentrantCallbackGroup(),
        )
        # 前置校验失败后不必等 goal 才重试：周期后台重试，底盘晚于导航
        # 启动时也能自愈。否则错误会一直挂到第一轮巡检，第一个 goal 被旧
        # 错误直接拒绝（11-04 首次按 B 播报后 abort 即由此引起）。
        self._preflight_timer = self.create_timer(
            max(1.0, self._preflight_retry_interval_s / 2.0),
            self._ensure_preflight)
        self.get_logger().info(
            f"NavigateTo 服务端就绪：dry_run={self._dry_run}，"
            f"点位顺序={[self._start_poi] + self._poi_order}"
        )
        self.get_logger().info(
            f"ApproachTarget 服务端就绪：standoff={self._approach_standoff_m:.2f}m，"
            f"适用距离 {self._approach_min_range_m:.2f}~"
            f"{self._approach_max_range_m:.2f}m，"
            f"单次最大位移 {self._approach_max_travel_m:.2f}m，"
            f"限速 {self._approach_speed_mps:.2f}m/s，"
            f"近距离侧向退让 {self._approach_retreat_m:.2f}m"
        )

    # ------------------------------------------------------------------
    # 配置
    # ------------------------------------------------------------------
    def _load_config(self) -> dict:
        configured = str(self.get_parameter("config_file").value or "")
        self._config_path = configured or os.path.join(
            get_package_share_directory("fire_inspection_nav"), "config", "pois.yaml"
        )
        with open(self._config_path, "r", encoding="utf-8") as f:
            config = yaml.safe_load(f) or {}
        if not isinstance(config, dict):
            raise RuntimeError(f"配置文件必须是映射: {self._config_path}")
        return config

    # ------------------------------------------------------------------
    # 前置校验（只读）
    # ------------------------------------------------------------------
    def _preflight(self) -> None:
        try:
            if self._poi_source != "chassis":
                raise RuntimeError(
                    f"poi_source={self._poi_source} 未实现，只支持 chassis"
                )
            self._map_name = self._read_map_name()

            self._pois = self._read_pois()
            if not self._pois:
                raise RuntimeError(
                    f"未读取到任何 POI（接口返回 {self._pois_raw_desc}）"
                )
            if self._start_poi in self._poi_order:
                raise RuntimeError(
                    f"start_poi={self._start_poi} 不应出现在 poi_order 中"
                )
            if len(set(self._poi_order)) != len(self._poi_order):
                raise RuntimeError(f"poi_order 存在重复项: {self._poi_order}")
            required = [self._start_poi] + self._poi_order
            missing = [name for name in required if name not in self._pois]
            if missing:
                raise RuntimeError(f"当前地图缺少 POI: {missing}")
            unknown_yaw = sorted(
                name for name in self._yaw_overrides if name not in self._pois
            )
            if unknown_yaw:
                raise RuntimeError(f"yaw_overrides 引用了不存在的 POI: {unknown_yaw}")
            for name in required:
                if any(self._pois[name].get(k) is None for k in ("x", "y", "yaw")):
                    raise RuntimeError(f"POI {name} 缺少 x/y/yaw")
            self._check_range_limit(required)

            unknown_transit = sorted(self._transit_pois - set(self._poi_order))
            if unknown_transit:
                raise RuntimeError(
                    f"transit_pois 必须属于 poi_order: {unknown_transit}"
                )
            unknown_dwell = sorted(set(self._dwell_overrides) - set(self._poi_order))
            if unknown_dwell:
                raise RuntimeError(
                    f"dwell_overrides 引用了不在 poi_order 的点位: {unknown_dwell}"
                )
            tracks_raw = self._read_tracks()
            tracks = [
                t
                for t in _unwrap_list(tracks_raw, "tracks", "track", "data")
                if isinstance(t, dict)
            ]
            by_name = {
                str(t.get("track_name") or t.get("track_id")): t for t in tracks
            }
            if self._outbound_track_name:
                outbound = by_name.get(self._outbound_track_name)
                if outbound is None:
                    raise RuntimeError(
                        f"未找到 outbound 轨道 {self._outbound_track_name}，"
                        f"当前: {sorted(by_name)}"
                    )
            elif len(tracks) == 1:
                outbound = tracks[0]
            else:
                raise RuntimeError(
                    f"存在多条轨道，必须在 tracks.outbound 指定，当前: {sorted(by_name)}"
                )
            if outbound.get("track_id") is None:
                raise RuntimeError("outbound 轨道缺少 track_id")
            self._track_id = outbound["track_id"]
            self._track_name = outbound.get("track_name") or str(self._track_id)
            self._outbound_waypoints = list(outbound.get("waypoints") or [])

            self._return_track_id = None
            self._return_waypoints = []
            if self._return_track_name:
                return_track = by_name.get(self._return_track_name)
                if return_track is None:
                    raise RuntimeError(
                        f"未找到 return 轨道 {self._return_track_name}，"
                        f"当前: {sorted(by_name)}"
                    )
                if return_track.get("track_id") is None:
                    raise RuntimeError("return 轨道缺少 track_id")
                self._return_track_id = return_track["track_id"]
                self._return_waypoints = list(return_track.get("waypoints") or [])
                self._check_return_track(return_track)

            self._check_poi_on_track(
                [n for n in required if n not in self._transit_pois], outbound
            )
            self._warn_track_yaw_mismatch(
                [n for n in required if n not in self._transit_pois],
                outbound,
                self._track_name,
            )

            # dry_run 不下发运动，定位异常只告警；实机模式定位异常会先尝试
            # 自动重定位（relocalize.enabled），仍不可用才拒绝
            self._check_localization(strict=not self._dry_run)
            if not self._dry_run:
                self._check_health()

            self.get_logger().info(
                f"前置校验通过：地图={self._map_name}，{len(self._pois)} 个 POI，"
                f"巡检轨道 {self._track_name} (track_id={self._track_id})，"
                f"返程轨道 {self._return_track_name or '未配置'} "
                f"(track_id={self._return_track_id})"
            )
            self._preflight_error = None
        except Exception as exc:  # noqa: BLE001 - 校验失败需完整保留原因
            self._preflight_error = str(exc)
            self.get_logger().error(f"前置校验失败，将拒绝所有 goal：{exc}")

    def _range_bounds(self) -> dict:
        """解析应用层软限位；配置存在但格式非法时直接报错。

        避免"看起来有保护、实际没生效"。
        """
        if not self._range_limit:
            return {}
        if not isinstance(self._range_limit, dict):
            raise RuntimeError("range_limit 必须是 {x: [下限, 上限], y: [下限, 上限]}")
        unknown = set(self._range_limit) - {"x", "y"}
        if unknown:
            raise RuntimeError(f"range_limit 存在未知键: {sorted(unknown)}")
        bounds = {}
        for axis in ("x", "y"):
            values = self._range_limit.get(axis)
            if values is None:
                continue
            if not isinstance(values, (list, tuple)) or len(values) != 2:
                raise RuntimeError(f"range_limit.{axis} 必须是 [下限, 上限]")
            lo, hi = float(values[0]), float(values[1])
            if lo >= hi:
                raise RuntimeError(f"range_limit.{axis} 下限必须小于上限: {values}")
            bounds[axis] = (lo, hi)
        return bounds

    def _point_range_error(self, x: float, y: float):
        """单点软限位校验：返回原因字符串，合规返回 None。"""
        for axis, (lo, hi) in self._range_bounds().items():
            value = x if axis == "x" else y
            if value < lo or value > hi:
                return f"{axis}={value:.3f} 超出软限位 [{lo}, {hi}]"
        return None

    def _corridor_bounds(self) -> dict:
        """解析临时落点可行走廊，并按车体安全余量内缩。

        corridor 填的是走廊物理边界（墙面 / 可行驶区边缘），这里统一内缩
        margin_m（车半宽 + 安全余量）后再校验车中心点，避免出现「中心点在
        走廊内、车体已经贴墙」的情况。未配置时返回空（不额外校验）。
        """
        if not self._corridor:
            return {}
        if not isinstance(self._corridor, dict):
            raise RuntimeError(
                "corridor 必须是 {x: [下限, 上限], y: [下限, 上限], margin_m: 米}")
        unknown = set(self._corridor) - {"x", "y", "margin_m"}
        if unknown:
            raise RuntimeError(f"corridor 存在未知键: {sorted(unknown)}")
        try:
            margin = float(self._corridor.get("margin_m", 0.0))
        except (TypeError, ValueError):
            raise RuntimeError(
                f"corridor.margin_m 非法: {self._corridor.get('margin_m')!r}")
        if not (math.isfinite(margin) and 0.0 <= margin <= 2.0):
            raise RuntimeError(f"corridor.margin_m 非法: {margin}")
        bounds = {}
        for axis in ("x", "y"):
            values = self._corridor.get(axis)
            if values is None:
                continue
            if not isinstance(values, (list, tuple)) or len(values) != 2:
                raise RuntimeError(f"corridor.{axis} 必须是 [下限, 上限]")
            lo, hi = float(values[0]), float(values[1])
            if not (math.isfinite(lo) and math.isfinite(hi)) or lo >= hi:
                raise RuntimeError(f"corridor.{axis} 非法: {values}")
            # 内缩后若区间为空，说明走廊比车窄：任何临时落点都会被拒绝，
            # 车只会原地停车播报（宁可不避让，也不贴墙）
            bounds[axis] = (lo + margin, hi - margin)
        return bounds

    def _point_corridor_error(self, x: float, y: float):
        """临时落点（接近/退让/绕行）的可行走廊校验。"""
        for axis, (lo, hi) in self._corridor_bounds().items():
            value = x if axis == "x" else y
            if value < lo or value > hi:
                return f"{axis}={value:.3f} 超出可行走廊 [{lo}, {hi}]"
        return None

    def _corridor_clearance(self, x: float, y: float):
        """临时落点距可行走廊边界的最小余量（米）；无走廊配置返回 None。"""
        bounds = self._corridor_bounds()
        if not bounds:
            return None
        clearance = math.inf
        for axis, (lo, hi) in bounds.items():
            value = x if axis == "x" else y
            clearance = min(clearance, value - lo, hi - value)
        return clearance

    def _check_range_limit(self, names) -> None:
        """按配置的应用层软限位校验 POI 坐标。"""
        bounds = self._range_bounds()
        for name in names:
            poi = self._pois[name]
            for axis, (lo, hi) in bounds.items():
                value = float(poi[axis])
                if value < lo or value > hi:
                    raise RuntimeError(
                        f"POI {name} 的 {axis}={value:.3f} 超出软限位 [{lo}, {hi}]"
                    )

    def _check_return_track(self, track) -> None:
        """返程轨道必须经过起点；上轨点与配置的过点点位不一致时告警。"""
        waypoints = track.get("waypoints") if isinstance(track, dict) else None
        if not isinstance(waypoints, list) or not waypoints:
            raise RuntimeError("return 轨道没有航点数据")
        points = []
        for waypoint in waypoints:
            position = waypoint.get("position") if isinstance(waypoint, dict) else None
            if (
                isinstance(position, dict)
                and position.get("x") is not None
                and position.get("y") is not None
            ):
                points.append((float(position["x"]), float(position["y"])))
        if not points:
            raise RuntimeError("return 轨道航点缺少坐标")
        start = self._pois[self._start_poi]
        sx, sy = float(start["x"]), float(start["y"])
        nearest = min(math.hypot(sx - x, sy - y) for x, y in points)
        if nearest > 0.2:
            raise RuntimeError(
                f"return 轨道不经过起点 {self._start_poi}（最近 {nearest:.2f}m）"
            )
        boarding = [name for name in self._poi_order if name in self._transit_pois]
        if boarding:
            poi = self._pois[boarding[-1]]
            bx, by = float(poi["x"]), float(poi["y"])
            first = min(math.hypot(bx - x, by - y) for x, y in points)
            if first > 0.5:
                self.get_logger().warn(
                    f"return 轨道起点距 {boarding[-1]} {first:.2f}m（>0.5m），"
                    f"请确认上轨点"
                )
        self._warn_track_yaw_mismatch(
            [self._start_poi], track, self._return_track_name or "return"
        )

    def _check_poi_on_track(self, names, track) -> None:
        """告警级校验：POI 是否与轨道航点位置吻合。

        只提示、不拒绝，避免现场微调点位后无法启动；轨道没有航点数据时跳过。
        """
        waypoints = track.get("waypoints") if isinstance(track, dict) else None
        if not isinstance(waypoints, list) or not waypoints:
            return
        points = []
        for waypoint in waypoints:
            position = waypoint.get("position") if isinstance(waypoint, dict) else None
            if (
                isinstance(position, dict)
                and position.get("x") is not None
                and position.get("y") is not None
            ):
                points.append((float(position["x"]), float(position["y"])))
        if not points:
            return
        for name in names:
            poi = self._pois[name]
            px, py = float(poi["x"]), float(poi["y"])
            nearest = min(math.hypot(px - x, py - y) for x, y in points)
            if nearest > 0.5:
                self.get_logger().warn(
                    f"POI {name} 距最近轨道航点 {nearest:.2f}m（>0.5m），"
                    f"请确认该点位在轨道可达范围内"
                )

    def _warn_track_yaw_mismatch(self, names, track, track_label) -> None:
        """轨道航点朝向与点位朝向不一致时告警。

        固件会忽略 move_to_locations 的目标 yaw，轨道到点后只能原地转向；
        若把轨道末端航点朝向改成点位朝向，机器人到点即为规定角度。
        """
        waypoints = track.get("waypoints") if isinstance(track, dict) else None
        if not isinstance(waypoints, list) or not waypoints:
            return
        for name in names:
            poi = self._pois[name]
            px, py = float(poi["x"]), float(poi["y"])
            nearest, best = None, float("inf")
            for waypoint in waypoints:
                position = waypoint.get("position") if isinstance(waypoint, dict) else None
                if not isinstance(position, dict):
                    continue
                if position.get("x") is None or position.get("y") is None:
                    continue
                d = math.hypot(px - float(position["x"]), py - float(position["y"]))
                # 同一坐标可能同时有"到达航点"和"转向航点"，取靠后的那个（实际到达朝向）
                if d <= best:
                    nearest, best = position, d
            if nearest is None or best > 0.5 or nearest.get("yaw") is None:
                continue
            waypoint_yaw = float(nearest["yaw"])
            poi_yaw = self._yaw_overrides.get(name, float(poi["yaw"]))
            error = abs(math.atan2(
                math.sin(poi_yaw - waypoint_yaw), math.cos(poi_yaw - waypoint_yaw)
            ))
            if error > 0.2:
                self.get_logger().warn(
                    f"{track_label} 在 {name} 处航点朝向={waypoint_yaw:.3f}rad，"
                    f"点位朝向={poi_yaw:.3f}rad，到点后会原地转向；"
                    f"如需到点即朝向规定角度，请把该航点朝向改为 {poi_yaw:.3f}"
                )

    def _ensure_preflight(self) -> None:
        """预检失败后在后台按间隔重试。

        重试不占用 goal 执行线程：网络黑洞时 REST 调用最长要等 request_timeout，
        同步重试会让客户端一直等、取消也无法及时生效。
        """
        if self._preflight_error is None:
            return
        now = time.monotonic()
        with self._preflight_lock:
            if self._preflight_running:
                return
            if now - self._last_preflight_at < self._preflight_retry_interval_s:
                return
            self._last_preflight_at = now
            self._preflight_running = True
        self.get_logger().info("前置校验失败过，后台重试中…")
        threading.Thread(target=self._preflight_worker, daemon=True).start()

    def _preflight_worker(self) -> None:
        try:
            self._preflight()
        finally:
            with self._preflight_lock:
                self._preflight_running = False

    def _read_map_name(self):
        rv = self._chassis.get_current_map()
        if isinstance(rv, dict) and rv.get("map_name"):
            return rv["map_name"]
        return None

    def _read_pois(self) -> dict:
        result = {}
        rv = self._chassis.get_pois(map_name=self._map_name, point_name=None)
        self._pois_raw_desc = _describe(rv)
        for poi in _unwrap_list(rv, "pois", "poi", "data"):
            if not isinstance(poi, dict):
                continue
            name = poi.get("name") or poi.get("point_name")
            if name:
                result[str(name)] = poi
        return result

    def _read_tracks(self):
        return self._chassis.get_tracks(map_name=self._map_name)

    def _read_slam_status(self):
        rv = self._chassis.get_localization_status() or {}
        return rv.get("slam_status") if isinstance(rv, dict) else None

    def _check_localization(self, strict: bool, goal_handle=None) -> str:
        """定位校验；实机模式下定位异常时尝试一次自动重定位。

        - dry_run / 未启用 relocalize：保持原语义，只告警或直接拒绝；
        - 实机 + 已启用：仅 slam_status ∈ {1,2,3,5} 才下发重定位，
          以 ``start_poi`` 位姿（或配置覆盖值）恢复，轮询到 0；
          底盘自身正在恢复（6/7/8）时不插手，直接拒绝本次导航；
        - goal 执行期间允许取消，返回 ``cancelled`` 由调用方收尾。
        """
        slam_status = self._read_slam_status()
        if slam_status == 0:
            return "ok"
        message = f"定位状态异常 slam_status={slam_status}（需为 0）"
        if not (strict and self._relocalize_enabled and not self._dry_run):
            if strict:
                raise RuntimeError(message)
            self.get_logger().warn(f"{message}；dry_run 不下发运动，仅记录为风险")
            return "ok"
        if slam_status not in _RELOCALIZE_TRIGGER_STATUS:
            raise RuntimeError(
                f"{message}；该状态不在自动重定位范围"
                f"{sorted(_RELOCALIZE_TRIGGER_STATUS)}（底盘可能正在恢复/加载地图），"
                f"本次导航拒绝"
            )

        self.get_logger().warn(f"{message}，尝试自动重定位")
        outcome = self._relocalize(goal_handle)
        if outcome == "cancelled":
            return "cancelled"
        self.get_logger().info("重定位成功，定位状态已恢复")
        return "ok"

    def _relocalize_pose(self) -> dict:
        """重定位初始位姿：优先配置覆盖值，否则用起点的 x/y/yaw。"""
        if self._relocalize_pose_override is not None:
            return dict(self._relocalize_pose_override)
        poi = self._pois.get(self._start_poi)
        if not isinstance(poi, dict) or any(poi.get(k) is None for k in ("x", "y", "yaw")):
            raise RuntimeError(
                f"重定位缺少初始位姿：请配置 relocalize.pose，"
                f"或确保已读到起点 {self._start_poi} 的 x/y/yaw"
            )
        return {"x": float(poi["x"]), "y": float(poi["y"]), "yaw": float(poi["yaw"])}

    def _relocalize(self, goal_handle=None) -> str:
        """下发重定位并轮询到恢复；失败抛异常，取消返回 ``cancelled``。"""
        pose = self._relocalize_pose()
        self.get_logger().info(
            f"下发重定位：x={pose['x']:.3f}，y={pose['y']:.3f}，"
            f"yaw={pose['yaw']:.3f}，超时={self._relocalize_timeout_s:.1f}s"
        )
        try:
            rv = self._chassis.recover_localization(
                pose["x"],
                pose["y"],
                pose["yaw"],
                max_time_ms=self._relocalize_max_time_ms,
            )
        except Exception as exc:  # noqa: BLE001 - 统一转成可读拒绝原因
            raise RuntimeError(f"重定位下发失败：{exc}") from exc
        action_id = rv.get("action_id") if isinstance(rv, dict) else None
        if action_id is None:
            self.get_logger().warn(f"重定位未返回 action_id（{rv!r}），仅轮询定位状态")
        else:
            self.get_logger().info(f"重定位动作已下发：action_id={action_id}")

        deadline = time.monotonic() + self._relocalize_timeout_s
        while time.monotonic() < deadline:
            if self._shutting_down:
                raise RuntimeError("节点关闭中，重定位中止")
            if goal_handle is not None and goal_handle.is_cancel_requested:
                self._cancel_action()
                return "cancelled"
            if self._read_slam_status() == 0:
                return "ok"
            time.sleep(
                min(self._relocalize_poll_s, max(0.0, deadline - time.monotonic()))
            )
        raise RuntimeError(
            f"重定位超时（{self._relocalize_timeout_s:.1f}s），"
            f"slam_status={self._read_slam_status()}"
        )

    def _check_health(self) -> None:
        """底盘健康校验：只有 A/B 类故障阻止运动，C 类告警仅记录。

        注意不能用 hasFatal/hasError 聚合标志直接判死：底盘只要有一条 C 类
        告警（如「WiFi 网络未连接」）就会把 hasFatal 置 True，而 WiFi 断连
        并不影响本地导航——之前实地巡检就是在无 WiFi 情况下跑完的。
        """
        health = self._chassis.get_robot_health()
        entries = list(getattr(health, "baseError", []) or [])
        blocking, warnings = split_health_errors(entries)
        if warnings:
            self.get_logger().warn(
                "底盘告警（不影响运动，继续）："
                + "；".join(describe_dtc(e) for e in warnings))
        if blocking:
            raise RuntimeError(
                "底盘健康检查未通过："
                + "；".join(describe_dtc(e) for e in blocking))
        if not entries and (
            getattr(health, "hasFatal", False)
            or getattr(health, "hasError", False)
        ):
            # 标志为真却没有任何 DTC 明细：无法判断严重程度，保守拒绝
            raise RuntimeError(
                "底盘健康检查未通过：未返回 DTC 详情"
                f"（hasFatal={getattr(health, 'hasFatal', None)}, "
                f"hasError={getattr(health, 'hasError', None)}）")

    # ------------------------------------------------------------------
    # 顺序管理
    # ------------------------------------------------------------------
    def _expected_poi(self):
        if not self._started:
            return self._start_poi
        if self._next_index < len(self._poi_order):
            return self._poi_order[self._next_index]
        # 所有 POI 走完后，允许再请求一次起点用于返回
        return None if self._returned else self._start_poi

    def _advance(self, poi: str) -> None:
        if poi == self._start_poi:
            if not self._started:
                self._started = True
            elif self._round_complete():
                # 一轮完整结束（走完所有点位并已返回）后再次到起点：复位为新一轮
                self._next_index = 0
                self._returned = False
            else:
                self._returned = True
        elif poi in self._poi_order:
            if self._round_complete():
                # 一轮完整结束后直接请求首个点位：同样视为新一轮
                self._next_index = 0
            self._next_index = self._poi_order.index(poi) + 1
            # 恢复巡检后允许再次返回
            self._returned = False

    def _round_complete(self) -> bool:
        return (
            self._started
            and self._next_index >= len(self._poi_order)
            and self._returned
        )

    def _progress(self) -> str:
        """当前进度快照，写入取消/失败结果与日志，便于判断从哪继续。"""
        if self._next_index < len(self._poi_order):
            next_poi = self._poi_order[self._next_index]
        else:
            next_poi = "无"
        return (
            f"进度[已开始={self._started}, 已返回起点={self._returned}, "
            f"已完成={self._next_index}/{len(self._poi_order)}, 下一站={next_poi}]"
        )

    def _order_error(self, poi: str, is_return: bool):
        """返回 None 表示放行，否则返回拒绝原因（poi 已去除首尾空白）。

        这里只做判断，不调用 abort；调用方负责在锁外把 goal 置为失败，
        避免持锁执行 rclpy 回调。
        """
        if is_return:
            if poi != self._start_poi:
                return f"is_return=true 时 poi 必须是 {self._start_poi}，收到 {poi}"
            if not self._started:
                return "尚未开始巡检，不能返回起点"
            if self._returned:
                return "已经在起点，无需重复返回"
            if self._next_index == 0:
                return "尚未离开起点，无需返回"
            return None

        if self._round_complete() and poi in (self._start_poi, self._poi_order[0]):
            # 一轮完整结束后请求起点或首个点位，都视为开始新一轮
            return None

        expected = self._expected_poi()
        if expected is None:
            return "点位序列已全部完成，无下一个目标"
        if poi != expected:
            return f"点位顺序不匹配：期望 {expected}，收到 {poi}"
        return None

    def _fail(self, goal_handle, result, message: str, level: str = "warn",
              phase: str = "failed"):
        result.success = False
        result.phase = phase
        result.is_return = bool(goal_handle.request.is_return)
        result.message = f"{message}；{self._progress()}"
        _log_at(self.get_logger(), level, result.message)
        goal_handle.abort()
        return result

    def _blocked(self, goal_handle, result, poi: str):
        """运动受阻且受阻自恢复已用尽：phase=blocked 供编排层跳过该点。

        非返程段在这里推进点位顺序：编排层跳过本点后会直接前往下一个点位，
        不推进的话后续 goal 会被顺序校验全部拒绝。返程段不推进，避免把
        「没回到起点」误记成已返回。
        """
        if not bool(goal_handle.request.is_return):
            with self._seq_lock:
                self._advance(poi)
        return self._fail(
            goal_handle, result,
            f"{poi} 运动受阻（长时间无位移，自动回退重试无效），跳过该点",
            "error", phase="blocked",
        )

    # ------------------------------------------------------------------
    # Goal 执行
    # ------------------------------------------------------------------
    def _on_goal(self, _goal_request):
        """同一时刻只接受一个 goal，避免并发执行打乱顺序与硬件状态。"""
        if not self._goal_lock.acquire(blocking=False):
            self.get_logger().warn("已有 goal 在执行，拒绝新的 goal")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_cancel(self, _goal_handle):
        """rclpy 默认取消回调是 REJECT，必须显式接受，否则取消永远不生效。"""
        self.get_logger().warn("收到取消请求，将在当前步骤内生效")
        return CancelResponse.ACCEPT

    def _release_goal(self) -> None:
        if self._goal_lock.locked():
            try:
                self._goal_lock.release()
            except RuntimeError:
                pass

    def _timeout_for(self, goal_handle, poi: str) -> float:
        """goal 显式指定超时优先，否则用配置里的默认值。

        只有"首次前往起点"用 to_start_s；返程属于常规段，用 per_poi_s。
        """
        requested = float(goal_handle.request.timeout_s or 0.0)
        if requested > 0.0:
            return requested
        if goal_handle.request.is_return:
            return self._default_timeout_s
        if poi == self._start_poi and not self._started:
            return self._start_timeout_s
        return self._default_timeout_s

    def _interruptible_sleep(self, goal_handle, seconds: float) -> bool:
        """分片睡眠，期间持续检查取消/关闭请求。返回 True 表示应中止。"""
        deadline = time.monotonic() + max(0.0, seconds)
        while time.monotonic() < deadline:
            if self._shutting_down or goal_handle.is_cancel_requested:
                return True
            time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))
        return self._shutting_down or goal_handle.is_cancel_requested

    def _execute(self, goal_handle):
        """任何异常都必须落到 abort，禁止异常逸出导致客户端永久等待。"""
        try:
            return self._execute_impl(goal_handle)
        except Exception as exc:  # noqa: BLE001
            result = NavigateTo.Result()
            result.success = False
            result.phase = "failed"
            result.is_return = bool(goal_handle.request.is_return)
            result.message = f"导航执行异常：{exc}"
            self.get_logger().error(result.message)
            try:
                goal_handle.abort()
            except Exception:  # noqa: BLE001
                pass
            return result
        finally:
            self._release_goal()

    def _execute_impl(self, goal_handle):
        poi = str(goal_handle.request.poi or "").strip()
        is_return = bool(goal_handle.request.is_return)
        result = NavigateTo.Result()

        if not poi:
            return self._fail(goal_handle, result, "poi 不能为空")

        if self._preflight_error is not None:
            # 预检失败过：后台按间隔重试，本次 goal 立即拒绝，不阻塞客户端
            self._ensure_preflight()
        if self._preflight_error is not None:
            return self._fail(
                goal_handle, result, f"前置校验未通过：{self._preflight_error}", "error"
            )

        with self._seq_lock:
            order_error = self._order_error(poi, is_return)
        if order_error is not None:
            return self._fail(goal_handle, result, order_error)

        use_point = bool(goal_handle.request.use_point)
        if use_point and is_return:
            return self._fail(
                goal_handle, result,
                "use_point 不能用于返程：返程必须走返程轨道")

        timeout_s = self._timeout_for(goal_handle, poi)
        # 起点与过点点位走点对点，巡检点沿巡检轨道，返程沿返程轨道；
        # use_point 让介入后的重发绕开轨道（否则底盘会先退回上轨点再重走一遍）
        mode = select_motion_mode(
            poi, is_return, use_point, self._start_poi, self._transit_pois)
        self.get_logger().info(
            f"收到 goal：poi={poi}，返回={is_return}，方式={mode}，"
            f"超时={timeout_s:.1f}s（dry_run={self._dry_run}）"
        )

        if not self._dry_run:
            return self._execute_real(goal_handle, poi, is_return, mode, timeout_s)

        if goal_handle.is_cancel_requested:
            return self._cancelled(goal_handle, result, poi)

        try:
            self._check_localization(strict=False)
        except Exception as exc:  # noqa: BLE001
            return self._fail(goal_handle, result, f"读取定位状态失败：{exc}", "error")

        pose = self._read_pose()
        if not isinstance(pose, dict):
            self.get_logger().warn(f"读取当前位姿失败：{pose}")
        distance_m = self._distance_to(poi, pose)
        self._publish_feedback(goal_handle, "navigating", mode, distance_m)
        if self._interruptible_sleep(
            goal_handle, min(self._dry_run_delay_s, timeout_s)
        ):
            return self._cancelled(goal_handle, result, poi)

        with self._seq_lock:
            self._advance(poi)
        if poi == self._start_poi:
            # 到达起点（首次集合或返程结束）：清掉返程打断标记
            self._return_interrupted = False

        result.success = True
        result.phase = "arrived"
        result.is_return = is_return
        action = "返回起点" if is_return else "前往"
        track_desc = "点对点"
        if mode == "track":
            track_desc = self._return_track_name if is_return else self._track_name
        result.message = (
            f"[dry_run] {action} {poi} 校验通过，未下发运动指令；"
            f"方式={mode}，轨道={track_desc}，当前位姿={pose}，"
            f"距目标={distance_m:.2f}m；{self._progress()}"
        )
        self._publish_feedback(goal_handle, "arrived", mode, 0.0)
        self.get_logger().info(result.message)
        goal_handle.succeed()
        return result

    # ------------------------------------------------------------------
    # 实机运动
    # ------------------------------------------------------------------
    def _execute_real(self, goal_handle, poi, is_return, mode, timeout_s):
        """实机运动：出发朝向对齐 → 下发底盘动作 → 轮询到终态 → 到点转向。"""
        result = NavigateTo.Result()
        result.is_return = is_return
        # 每次 goal 单独记录是否发生过侧向绕行（影响到点转向与结果说明）
        self._goal_detoured = False
        if self._check_localization(strict=True, goal_handle=goal_handle) == "cancelled":
            return self._cancelled(goal_handle, result, poi)
        self._check_health()
        # 巡检段与返程段分别下发限速（配置相同则不会重复写底盘参数）
        if is_return:
            self._apply_speed(self._return_speed, "返程")
        else:
            self._apply_speed(self._outbound_speed, "巡检")
        self._set_aeb(True)
        deadline = time.monotonic() + timeout_s
        try:
            if goal_handle.is_cancel_requested or self._shutting_down:
                return self._cancelled(goal_handle, result, poi)
            target = self._pois[poi]
            if is_return:
                if self._return_track_id is None:
                    raise RuntimeError("未配置返程轨道 tracks.return")
                motion_mode = "track"
                active_track = self._return_track_name
            elif mode == "point":
                motion_mode = "point"
                active_track = self._track_name
            else:
                # 固件会忽略 target_position.yaw，轨道段只能先沿轨道到点再原地转向
                motion_mode = "track"
                active_track = self._track_name

            def redispatch(point=False):
                """受阻自恢复后重发同一目标。

                point=True 表示车辆已被侧向绕行挪离轨道，优先点对点直奔目标
                （避免重新上轨时又被同一障碍挡住）。返程段例外：跨全图点对点
                规划会失败，仍走返程轨道，靠 uptrack 自主回到上轨点。
                """
                if point or motion_mode == "point":
                    return self._dispatch_motion(
                        poi, is_return, mode, use_point=True
                    )
                return self._dispatch_motion(poi, is_return, mode)

            # 轨道段不会自动把车头转到行进方向，先原地对齐，
            # 否则车会保持点位朝向沿轨道横着平移（如 POI_02→POI_03 差 90°）
            if motion_mode == "track":
                if self._align_travel_direction(
                    goal_handle, poi, is_return, deadline, timeout_s
                ) == "cancelled":
                    return self._cancelled(goal_handle, result, poi)
            else:
                # 点对点默认「边转边移」，车头背离目标时会画弧横移，
                # 现场看到的就是「横着跑」；先原地转正再走
                if self._align_point_departure(
                    goal_handle, poi, target, deadline, timeout_s
                ) == "cancelled":
                    return self._cancelled(goal_handle, result, poi)

            action_id = self._dispatch_motion(poi, is_return, mode)
            self.get_logger().info(
                f"实机运动已下发：poi={poi}，方式={motion_mode}，action_id={action_id}"
            )
            outcome = self._run_chassis_action(
                goal_handle, action_id, poi, motion_mode, deadline, timeout_s,
                redispatch=redispatch, is_return=is_return,
            )
            if outcome == "cancelled":
                return self._cancelled(goal_handle, result, poi)
            if outcome == "blocked":
                return self._blocked(goal_handle, result, poi)
            if outcome == "transient":
                # 介入停车后立刻重发，固件常会在停稳过渡期直接拒绝；
                # 等待一拍后原样重发一次，避免一次拒绝就终止整轮巡检
                self.get_logger().warn(
                    f"{poi} 首次下发被底盘瞬时拒绝，1.0s 后重发原任务")
                if self._interruptible_sleep(goal_handle, 1.0):
                    return self._cancelled(goal_handle, result, poi)
                if is_return and self._return_interrupted:
                    # 返程续行首次被拒多发生在停车后的固件过渡期（实测 0.11s
                    # 即 failed）。原样重试不上轨续行即可；不要立刻回退上轨，
                    # 上轨会先把车带回上轨点 POI_04（现场「一播报就回点4」）。
                    self.get_logger().warn(
                        "返程续行首次被拒，重试不上轨续行（不回退上轨点）")
                action_id = self._dispatch_motion(poi, is_return, mode)
                self.get_logger().info(
                    f"实机运动已重发：poi={poi}，方式={motion_mode}，action_id={action_id}"
                )
                outcome = self._run_chassis_action(
                    goal_handle, action_id, poi, motion_mode, deadline, timeout_s,
                    redispatch=redispatch,
                )
                if outcome == "cancelled":
                    return self._cancelled(goal_handle, result, poi)
                if outcome == "blocked":
                    return self._blocked(goal_handle, result, poi)
            if outcome == "transient":
                # 返程轨道接口只接受从上轨点 (POI_04) 续行，介入后车停在
                # 轨道中间，任何轨道续行都会被 0.1s 瞬拒；先点对点重接轨道
                # 也只是把车拉回原来的轨道中心线（现场就是「绕一下又回到
                # 那个点」，然后和同一个人再次相遇）。这里直接沿返程轨道
                # 分段点对点推进，并保持介入后的横向偏移。
                if is_return and self._return_interrupted:
                    self.get_logger().warn(
                        f"{poi} 返程轨道中途续行被拒（只认上轨点），"
                        "改为保持横向偏移分段点对点推进")
                    staged = self._advance_return_in_stages(
                        goal_handle, poi, deadline)
                    if staged == "cancelled":
                        return self._cancelled(goal_handle, result, poi)
                    if staged == "failed":
                        return self._blocked(goal_handle, result, poi)
                    outcome = "ok"
                # 重发仍被拒：只有「已经离目标很近」时才允许点对点兜底，
                # 远处继续走轨道，宁可失败也不让车跨区直线乱窜
                remaining = self._distance_to(poi, self._read_pose())
                if (
                    motion_mode == "track"
                    and not is_return
                    and remaining <= _POINT_FALLBACK_MAX_M
                ):
                    self.get_logger().warn(
                        f"{poi} 轨道重发仍被拒绝，距目标 {remaining:.2f}m，"
                        "改用点对点兜底")
                    motion_mode = "point"
                    action_id = self._dispatch_motion(
                        poi, is_return, mode, use_point=True
                    )
                    self.get_logger().info(
                        f"实机运动已下发（点对点兜底）：poi={poi}，action_id={action_id}"
                    )
                    outcome = self._run_chassis_action(
                        goal_handle, action_id, poi, motion_mode, deadline, timeout_s,
                        redispatch=redispatch,
                    )
                    if outcome == "cancelled":
                        return self._cancelled(goal_handle, result, poi)
                    if outcome == "blocked":
                        return self._blocked(goal_handle, result, poi)
            if outcome == "transient":
                # 连续立即被拒：底盘当前状态下执行不了这个目标（前方被挡 /
                # 规划拒绝 / 仍在停稳过渡期）。按受阻返回，让编排层跳过该点继续，
                # 而不是抛异常把整轮巡检判死。
                self.get_logger().error(
                    f"{poi} 实机运动被底盘连续立即拒绝，按受阻跳过本段")
                return self._blocked(goal_handle, result, poi)

            # 车已停稳在点位：先发 arrived 反馈，编排层据此提前停掉头部扫视。
            # 后续还有原地转向 + 到点停留（合计可达数秒），这段时间若镜头仍在
            # 左右扫视，现场会看到「车到了、头还在扫」。
            self._publish_feedback(goal_handle, "arrived", motion_mode, 0.0)

            # 轨道到达朝向与设定角度不同，到点后原地转向并校验实际朝向；
            # 绕行过的段即使原本是点对点也要补转向，保证镜头朝向点位
            if motion_mode == "track" or self._goal_detoured:
                yaw = self._yaw_overrides.get(poi, float(target["yaw"]))
                rotate = self._rotate_to(
                    goal_handle, poi, target, yaw, deadline, timeout_s
                )
                if rotate == "cancelled":
                    return self._cancelled(goal_handle, result, poi)

            # 固件报成功不等于车到了巡检点：位置偏差过大直接判失败
            self._check_arrival_position(poi)

            # 到点停留（起点与返程不停留），供后续拍照/语音使用
            dwell_s = self._dwell_for(poi)
            if dwell_s > 0:
                if not self._dwell(goal_handle, poi, deadline, dwell_s):
                    return self._cancelled(goal_handle, result, poi)
        finally:
            # 无论成功 / 失败 / 取消，都先取消动作再关 AEB
            self._cancel_action()
            self._set_aeb(False)

        with self._seq_lock:
            self._advance(poi)
        if poi == self._start_poi:
            # 到达起点（首次集合或返程结束）：清掉返程打断标记
            self._return_interrupted = False

        result.success = True
        result.phase = "arrived"
        action = "返回起点" if is_return else "前往"
        track_desc = active_track if motion_mode == "track" else "点对点"
        result.message = (
            f"{action} {poi} 完成；方式={motion_mode}，轨道={track_desc}；"
            f"{'（已绕行）' if self._goal_detoured else ''}{self._progress()}"
        )
        self._publish_feedback(goal_handle, "arrived", motion_mode, 0.0)
        self.get_logger().info(result.message)
        goal_handle.succeed()
        return result

    # ------------------------------------------------------------------
    # ApproachTarget：走到人员前方 standoff 米（吸烟介入用）
    # ------------------------------------------------------------------
    def _execute_approach(self, goal_handle):
        """任何异常都必须落到 abort，禁止逸出导致客户端永久等待。"""
        try:
            return self._execute_approach_impl(goal_handle)
        except Exception as exc:  # noqa: BLE001
            result = ApproachTarget.Result()
            result.success = False
            result.phase = "failed"
            result.final_range_m = 0.0
            result.message = f"接近执行异常：{exc}"
            self.get_logger().error(result.message)
            try:
                goal_handle.abort()
            except Exception:  # noqa: BLE001
                pass
            return result
        finally:
            self._release_goal()

    def _execute_approach_impl(self, goal_handle):
        request = goal_handle.request
        result = ApproachTarget.Result()
        result.final_range_m = 0.0

        bearing = float(request.bearing_rad)
        range_m = float(request.range_m)
        standoff = float(request.standoff_m or 0.0)
        if standoff <= 0.0:
            standoff = self._approach_standoff_m
        timeout_s = float(request.timeout_s or 0.0)
        if timeout_s <= 0.0:
            timeout_s = self._approach_timeout_s
        side_offset = float(request.side_offset_m or 0.0)
        if side_offset == 0.0:
            side_offset = self._approach_side_offset_m

        if not math.isfinite(bearing) or abs(bearing) > math.pi + 1e-6:
            return self._fail_approach(
                goal_handle, result, f"bearing_rad 非法：{bearing}")
        if not math.isfinite(range_m):
            return self._fail_approach(
                goal_handle, result, f"range_m 非法：{range_m}")
        if not math.isfinite(standoff) or standoff < 0.0:
            return self._fail_approach(
                goal_handle, result, f"standoff_m 非法：{standoff}")
        if not math.isfinite(timeout_s) or timeout_s <= 0.0:
            return self._fail_approach(
                goal_handle, result, f"timeout_s 非法：{timeout_s}")
        if not math.isfinite(side_offset) or abs(side_offset) > 3.0:
            return self._fail_approach(
                goal_handle, result, f"side_offset_m 非法：{side_offset}")

        if self._preflight_error is not None:
            self._ensure_preflight()
        if self._preflight_error is not None:
            return self._fail_approach(
                goal_handle, result,
                f"前置校验未通过：{self._preflight_error}", "error")

        # 距离不合格一律保持原位：宁可只播报，也不盲目前后乱走
        if range_m <= 0.0:
            return self._skip_approach(
                goal_handle, result, "无有效测距（range_m<=0），保持原地不接近")
        # 纯侧向绕行（standoff == range，前进量为 0）不能走「太近→后退」：
        # 否则会在请求的横移之外再叠加一次退让（现场 2026-11-03 返程避让
        # 时叠加后退 1.2m，把车推到门框边，随后返程一直受阻）。
        # 只有仍打算向目标靠近（standoff > range）时才需要后退拉开距离。
        if (range_m < self._approach_min_range_m
                and standoff > range_m + 1e-6):
            return self._retreat_beside_target(
                goal_handle, result, bearing, range_m, timeout_s)
        if range_m > self._approach_max_range_m:
            return self._skip_approach(
                goal_handle, result,
                f"目标距离 {range_m:.2f}m 超过接近上限 "
                f"{self._approach_max_range_m:.2f}m，保持原地不接近")

        pose = self._read_pose()
        if (
            not isinstance(pose, dict)
            or any(pose.get(k) is None for k in ("x", "y", "yaw"))
        ):
            return self._fail_approach(
                goal_handle, result, f"无法读取当前位姿：{pose!r}", "error")
        try:
            geometry = compute_approach_geometry(
                {k: float(pose[k]) for k in ("x", "y", "yaw")},
                bearing, range_m, standoff, side_offset,
            )
        except (KeyError, TypeError, ValueError) as exc:
            return self._fail_approach(
                goal_handle, result, f"接近几何换算失败：{exc}", "error")

        if geometry["travel_m"] > self._approach_max_travel_m:
            return self._skip_approach(
                goal_handle, result,
                f"需前进 {geometry['travel_m']:.2f}m 超过单次接近上限 "
                f"{self._approach_max_travel_m:.2f}m，保持原地不接近")
        if geometry["travel_m"] < _APPROACH_MIN_TRAVEL_M:
            # 已在期望距离附近：接近点就是当前位置，move_to 退化为原地转向，
            # 不再前后挪动（避免低速下的来回调整）
            self.get_logger().info(
                f"位移仅 {geometry['travel_m']:.2f}m "
                f"(< {_APPROACH_MIN_TRAVEL_M:.2f}m)，只原地转向目标，不前后移动")
        range_error = self._point_range_error(
            geometry["approach_x"], geometry["approach_y"])
        corridor_error = self._point_corridor_error(
            geometry["approach_x"], geometry["approach_y"])
        if ((range_error is not None or corridor_error is not None)
                and side_offset != 0.0):
            # 现场走廊很窄：按「远离人员」选出的那一侧常常正好贴墙，
            # 直接跳过就会表现为「看到人却从不绕开」。这里先镜像到另一侧
            # 再校验一次，只有两侧都不可用才放弃。
            alt_side = -side_offset
            try:
                alt_geometry = compute_approach_geometry(
                    {k: float(pose[k]) for k in ("x", "y", "yaw")},
                    bearing, range_m, standoff, alt_side,
                )
            except (KeyError, TypeError, ValueError) as exc:
                self.get_logger().warn(f"镜像绕行侧几何换算失败：{exc}")
            else:
                alt_range_error = self._point_range_error(
                    alt_geometry["approach_x"], alt_geometry["approach_y"])
                alt_corridor_error = self._point_corridor_error(
                    alt_geometry["approach_x"], alt_geometry["approach_y"])
                if alt_range_error is None and alt_corridor_error is None:
                    self.get_logger().warn(
                        f"原绕行侧落点不可用（{range_error or corridor_error}），"
                        f"改向另一侧横移 side_offset={alt_side:+.2f}m")
                    geometry = alt_geometry
                    range_error = None
                    corridor_error = None
        if range_error is not None:
            return self._skip_approach(
                goal_handle, result, f"接近点超出软限位（{range_error}），保持原地")
        if corridor_error is not None:
            return self._skip_approach(
                goal_handle, result,
                f"接近点超出可行走廊（{corridor_error}），保持原地不接近")

        self.get_logger().info(
            f"接近请求：bearing={bearing:+.3f}rad range={range_m:.2f}m "
            f"standoff={standoff:.2f}m → 接近点=({geometry['approach_x']:.2f}, "
            f"{geometry['approach_y']:.2f}) 朝向={geometry['yaw']:+.3f}rad "
            f"位移={geometry['travel_m']:.2f}m dry_run={self._dry_run}"
        )

        if self._dry_run:
            return self._simulate_approach(
                goal_handle, result, geometry, standoff, timeout_s)
        return self._execute_approach_real(
            goal_handle, result, geometry, standoff, timeout_s)

    def _retreat_beside_target(self, goal_handle, result, bearing, range_m,
                               timeout_s):
        """目标已贴到接近下限以内：不往前冲，横移让开通道后继续。

        目标在左侧（bearing>0）就向右退，反之向左退；正前方默认向右。
        只做一次固定距离横移，超出软限位或位姿不可读时退回原地不接近。
        """
        offset = self._approach_retreat_m
        if offset <= 0.0:
            return self._skip_approach(
                goal_handle, result,
                f"目标距离 {range_m:.2f}m 已小于接近下限 "
                f"{self._approach_min_range_m:.2f}m，保持原地不接近")

        pose = self._read_pose()
        if (
            not isinstance(pose, dict)
            or any(pose.get(k) is None for k in ("x", "y", "yaw"))
        ):
            return self._skip_approach(
                goal_handle, result,
                f"目标距离 {range_m:.2f}m 已小于接近下限，但读不到位姿，保持原地")
        try:
            geometry = compute_approach_geometry(
                {k: float(pose[k]) for k in ("x", "y", "yaw")},
                bearing, range_m, 0.0, 0.0,
            )
        except (KeyError, TypeError, ValueError) as exc:
            return self._skip_approach(
                goal_handle, result, f"退让几何换算失败：{exc}，保持原地")

        # 人偏左 → 优先向右退；人偏右 → 优先向左退；正前方按向右。
        # 若首选侧退让点离走廊边界太近（现场曾出现右侧贴墙），改走另一侧；
        # 两侧都不可用或都贴墙时保持原地，宁可不避让也不往墙上贴。
        preferred_right = bearing >= 0.0
        preferred_delta = -math.pi / 2.0 if preferred_right else math.pi / 2.0
        preferred_label = "右" if preferred_right else "左"
        alt_delta = math.pi / 2.0 if preferred_right else -math.pi / 2.0
        alt_label = "左" if preferred_right else "右"

        def _candidate(delta):
            lateral = float(pose["yaw"]) + delta
            tx = float(pose["x"]) + offset * math.cos(lateral)
            ty = float(pose["y"]) + offset * math.sin(lateral)
            range_error = self._point_range_error(tx, ty)
            if range_error is not None:
                return None, lateral, tx, ty, (
                    f"侧向退让点超出软限位（{range_error}），保持原地")
            corridor_error = self._point_corridor_error(tx, ty)
            if corridor_error is not None:
                return None, lateral, tx, ty, (
                    f"侧向退让点超出可行走廊（{corridor_error}），"
                    "保持原地不接近")
            return True, lateral, tx, ty, None

        ok, lateral, target_x, target_y, error = _candidate(preferred_delta)
        if not ok:
            return self._skip_approach(goal_handle, result, error)
        side = preferred_label
        clearance = self._corridor_clearance(target_x, target_y)
        if clearance is not None and clearance < _RETREAT_MIN_CLEARANCE_M:
            alt_ok, alt_lateral, alt_x, alt_y, alt_error = _candidate(alt_delta)
            alt_clearance = (
                self._corridor_clearance(alt_x, alt_y) if alt_ok else None)
            if alt_ok and (
                alt_clearance is None
                or alt_clearance >= _RETREAT_MIN_CLEARANCE_M
                or alt_clearance > clearance
            ):
                self.get_logger().warn(
                    f"首选{preferred_label}侧退让点离走廊边界仅 "
                    f"{clearance:.2f}m（<{_RETREAT_MIN_CLEARANCE_M:.2f}m），"
                    f"改向{alt_label}侧退让")
                lateral = alt_lateral
                target_x, target_y = alt_x, alt_y
                side = alt_label
            else:
                reason = alt_error or "另一侧也贴近走廊边界"
                return self._skip_approach(
                    goal_handle, result,
                    f"首选{preferred_label}侧退让点贴近走廊边界"
                    f"（余量 {clearance:.2f}m），{reason}")
        geometry["approach_x"] = target_x
        geometry["approach_y"] = target_y
        geometry["yaw"] = math.atan2(
            math.sin(lateral), math.cos(lateral))
        geometry["travel_m"] = offset

        self.get_logger().warn(
            f"目标距离 {range_m:.2f}m 已小于接近下限 "
            f"{self._approach_min_range_m:.2f}m → 向{side}侧退让 "
            f"{offset:.2f}m 让开通道：退让点=({target_x:.2f}, {target_y:.2f}) "
            f"朝向={geometry['yaw']:+.3f}rad dry_run={self._dry_run}"
        )

        message = (
            f"目标距离 {range_m:.2f}m 已小于接近下限"
            f"{self._approach_min_range_m:.2f}m，已向{side}侧退让 "
            f"{offset:.2f}m（退让后距目标 {{final_range:.2f}}m）"
        )
        if self._dry_run:
            return self._simulate_approach(
                goal_handle, result, geometry, 0.0, timeout_s,
                phase="retreated", message=message)
        return self._execute_approach_real(
            goal_handle, result, geometry, 0.0, timeout_s,
            phase="retreated", message=message)

    def _simulate_approach(self, goal_handle, result, geometry, standoff,
                           timeout_s, phase="arrived", message=None):
        """dry_run：只做几何与控制流校验，不下发任何运动指令。"""
        self._publish_approach_feedback(
            goal_handle, "approaching", geometry["travel_m"])
        if self._interruptible_sleep(
            goal_handle, min(self._dry_run_delay_s, timeout_s)
        ):
            result.success = False
            result.phase = "cancelled"
            result.message = "[dry_run] 接近被取消"
            self.get_logger().warn(result.message)
            self._mark_cancelled(goal_handle)
            return result

        self._publish_approach_feedback(goal_handle, phase, 0.0)
        result.success = True
        result.phase = phase
        result.final_range_m = float(standoff)
        if message is not None:
            result.message = "[dry_run] " + message.format(
                final_range=result.final_range_m)
        else:
            result.message = (
                f"[dry_run] 接近校验通过，未下发运动指令；接近点=("
                f"{geometry['approach_x']:.2f}, {geometry['approach_y']:.2f})，"
                f"朝向={geometry['yaw']:+.3f}rad"
            )
        self.get_logger().info(result.message)
        goal_handle.succeed()
        return result

    def _execute_approach_real(self, goal_handle, result, geometry, standoff,
                               timeout_s, phase="arrived", message=None):
        """实机接近：定位/健康检查 → 限速 → AEB → move_to → 轮询 → 校验。"""
        if self._check_localization(strict=True, goal_handle=goal_handle) == "cancelled":
            return self._approach_cancelled(goal_handle, result, "接近前定位检查被取消")
        self._check_health()
        self._apply_speed(self._approach_speed_mps, "接近")
        self._set_aeb(True)
        deadline = time.monotonic() + timeout_s
        final_range = 0.0
        try:
            if goal_handle.is_cancel_requested or self._shutting_down:
                return self._approach_cancelled(goal_handle, result, "接近被取消")

            def dispatch():
                rv = self._chassis.move_to(
                    float(geometry["approach_x"]),
                    float(geometry["approach_y"]),
                    float(geometry["yaw"]),
                    flags=0,
                )
                aid = rv.get("action_id") if isinstance(rv, dict) else None
                if aid is None:
                    raise RuntimeError(f"接近动作未返回 action_id：{rv!r}")
                return aid

            # 目标点已过走廊/软限位校验；首次被底盘拒绝时撤干净重发一次。
            # 现场 2026-10-31：刚取消轨道任务就下发接近，0.76s 即 failed，
            # 直接放弃导致「看见人却没有避让」；重发一次可覆盖这种过渡期拒绝。
            for attempt in (1, 2):
                action_id = dispatch()
                self.get_logger().info(
                    f"接近动作已{'重发' if attempt == 2 else '下发'}："
                    f"action_id={action_id}")
                try:
                    outcome = self._run_approach_action(
                        goal_handle, action_id, deadline, timeout_s,
                        geometry["person_x"], geometry["person_y"], standoff,
                    )
                except _ActionFailed as exc:
                    self._cancel_and_settle(action_id)
                    if (
                        attempt == 2
                        or goal_handle.is_cancel_requested
                        or self._shutting_down
                        or time.monotonic() + 1.0 >= deadline
                    ):
                        raise
                    self.get_logger().warn(
                        f"接近动作失败（{exc}），1.0s 后重发一次")
                    time.sleep(1.0)
                    continue
                if outcome == "transient":
                    if (
                        attempt == 2
                        or goal_handle.is_cancel_requested
                        or self._shutting_down
                        or time.monotonic() + 1.0 >= deadline
                    ):
                        raise RuntimeError("接近动作被底盘连续立即拒绝，放弃本次接近")
                    self.get_logger().warn(
                        "接近动作被底盘瞬时拒绝，1.0s 后重发一次")
                    time.sleep(1.0)
                    continue
                if outcome == "cancelled":
                    return self._approach_cancelled(
                        goal_handle, result, "接近被取消")
                break
            final_range = self._range_to_point(
                geometry["person_x"], geometry["person_y"])
            if phase == "arrived":
                self._warn_approach_arrival(geometry, standoff, final_range)
        finally:
            # 无论成功 / 失败 / 取消，都先取消动作再关 AEB
            self._cancel_action()
            self._set_aeb(False)

        result.success = True
        result.phase = phase
        result.final_range_m = float(final_range)
        if message is not None:
            result.message = message.format(final_range=final_range)
        else:
            result.message = (
                f"已接近到目标前方 {final_range:.2f}m（期望 {standoff:.2f}m），"
                f"到位朝向 {float(geometry['yaw']):+.3f}rad"
            )
        self._publish_approach_feedback(goal_handle, phase, 0.0)
        self.get_logger().info(result.message)
        goal_handle.succeed()
        return result

    def _run_approach_action(self, goal_handle, action_id, deadline, timeout_s,
                             person_x, person_y, standoff) -> str:
        """轮询接近动作到终态。

        返回 success / cancelled / transient；其余失败抛异常。
        transient 表示「刚下发就 failed 且从未起步」——通常是与上一段
        轨道任务取消撞车（固件停稳过渡期拒新任务），由调用方撤干净后重发。
        """
        deadline = min(deadline, time.monotonic() + timeout_s)
        started_at = time.monotonic()
        last_log = 0.0
        while True:
            if self._shutting_down or goal_handle.is_cancel_requested:
                self._cancel_and_settle(action_id)
                return "cancelled"

            try:
                st = self._chassis.get_action_status(action_id)
            except Exception as exc:  # noqa: BLE001 - 轮询抖动不立即终止
                self.get_logger().warn(f"轮询动作状态异常（继续）：{exc}")
                st = None

            verdict = _classify_action_status(st)
            if verdict == "success":
                return "success"
            if verdict == "failure":
                elapsed = time.monotonic() - started_at
                if (
                    not self._shutting_down
                    and not goal_handle.is_cancel_requested
                    and elapsed <= _INSTANT_FAILURE_S
                    and _never_started(st)
                ):
                    self.get_logger().warn(
                        f"接近动作下发 {elapsed:.2f}s 即被底盘拒绝（从未起步）："
                        f"{st!r}；按瞬态处理")
                    self._cancel_and_settle(action_id)
                    return "transient"
                raise _ActionFailed(f"接近运动执行失败：status={st!r}")

            # 用例：距离估计是快照值，人员移动或视角变化时只能作为参考
            range_now = self._range_to_point(person_x, person_y)
            self._publish_approach_feedback(
                goal_handle, "approaching", max(0.0, range_now - standoff))
            now = time.monotonic()
            if now - last_log >= 2.0:
                last_log = now
                self.get_logger().info(
                    f"接近进行中：距目标约 {range_now:.2f}m，"
                    f"剩余超时 {max(0.0, deadline - now):.0f}s"
                )
            if now >= deadline:
                self._cancel_and_settle(action_id)
                raise RuntimeError(
                    f"接近超时（{timeout_s:.0f}s），已下发取消；最后状态={st!r}"
                )
            time.sleep(self._action_poll_interval_s)

    def _range_to_point(self, x: float, y: float) -> float:
        """当前位姿到指定点的水平距离；位姿不可读时返回 0.0。"""
        pose = self._read_pose()
        if not isinstance(pose, dict):
            return 0.0
        try:
            return math.hypot(
                float(x) - float(pose["x"]), float(y) - float(pose["y"]))
        except (KeyError, TypeError, ValueError):
            return 0.0

    def _warn_approach_arrival(self, geometry, standoff, final_range) -> None:
        """接近到位校验：接近点由视觉估算，精度低于点位坐标，只告警不判失败。"""
        pose = self._read_pose()
        if isinstance(pose, dict) and pose.get("yaw") is not None:
            error = abs(math.atan2(
                math.sin(float(geometry["yaw"]) - float(pose["yaw"])),
                math.cos(float(geometry["yaw"]) - float(pose["yaw"])),
            ))
            if error > 0.2:
                self.get_logger().warn(
                    f"接近到位后朝向偏差 {error:.3f}rad（>0.20），请现场确认")
        if final_range <= 0.0:
            self.get_logger().warn("接近到位后读不到位姿，final_range_m 记为 0")
            return
        tolerance = max(self._approach_tolerance_m, 1.0)
        if abs(final_range - standoff) > tolerance:
            self.get_logger().warn(
                f"接近距离 {final_range:.2f}m 与期望 {standoff:.2f}m 偏差超过 "
                f"{tolerance:.2f}m（视觉估算精度有限，仅告警）"
            )

    def _fail_approach(self, goal_handle, result, message: str,
                       level: str = "warn"):
        result.success = False
        result.phase = "failed"
        result.message = message
        _log_at(self.get_logger(), level, message)
        goal_handle.abort()
        return result

    def _skip_approach(self, goal_handle, result, message: str):
        """不移动直接结束：距离不合格 / 超出安全范围时保持原位。

        用 succeed() 而不是 abort()：这不是执行故障，而是有意的安全取舍，
        编排层据此决定「就地播报后继续巡检」。
        """
        result.success = False
        result.phase = "skipped"
        result.message = message
        self.get_logger().warn(message)
        goal_handle.succeed()
        return result

    def _approach_cancelled(self, goal_handle, result, message: str):
        result.success = False
        result.phase = "cancelled"
        result.message = message
        self.get_logger().warn(message)
        self._mark_cancelled(goal_handle)
        return result

    def _mark_cancelled(self, goal_handle) -> None:
        try:
            goal_handle.canceled()
        except Exception as exc:  # noqa: BLE001 - 关闭过程中 goal 可能已失效
            self.get_logger().warn(f"标记取消状态失败：{exc}")

    def _publish_approach_feedback(self, goal_handle, phase: str,
                                   distance_remaining_m: float) -> None:
        feedback = ApproachTarget.Feedback()
        feedback.phase = phase
        feedback.distance_remaining_m = float(distance_remaining_m)
        goal_handle.publish_feedback(feedback)

    def _run_chassis_action(self, goal_handle, action_id, poi, mode, deadline,
                            timeout_s, detect_stall=True, redispatch=None,
                            is_return=False):
        """轮询底盘动作到终态。

        返回 success / cancelled / transient / blocked；超时抛异常，由 _execute
        统一 abort。transient 表示「刚下发就 failed 且从未起步」，由调用方决定
        是否重发。blocked 表示运动受阻且自恢复已用尽，由调用方判本段失败。

        detect_stall 只对「平移」段开启：原地转向时位置本就不变，不能在
        那里把正常转向误判成受阻。停滞后的恢复流程是「取消 → 后退 →
        向左/右绕行 → 点对点重发」；redispatch 用于重发原目标。
        """
        deadline = min(deadline, time.monotonic() + timeout_s)
        started_at = time.monotonic()
        # 起步位置：判断「刚下发就 failed」时能不能按瞬态重发。
        # 固件返回的 distance_remaining/distance_time 语义不可靠（拒绝时常带
        # 整段规划距离），只有我们自己观测到位移才算真的起步过。
        start_xy = self._pose_xy(self._read_pose())
        last_log = 0.0
        # 受阻检测：锚点是从「最近一次确认有位移」的位置算起
        stall_anchor = None
        stall_anchor_at = time.monotonic()
        recoveries = 0
        while True:
            if self._shutting_down or goal_handle.is_cancel_requested:
                self._cancel_and_settle(action_id)
                return "cancelled"

            try:
                st = self._chassis.get_action_status(action_id)
            except Exception as exc:  # noqa: BLE001 - 轮询抖动不立即终止
                self.get_logger().warn(f"轮询动作状态异常（继续）：{exc}")
                st = None

            verdict = _classify_action_status(st)
            if verdict == "success":
                return "success"
            if verdict == "failure":
                elapsed = time.monotonic() - started_at
                if (
                    not self._shutting_down
                    and not goal_handle.is_cancel_requested
                    and elapsed <= _INSTANT_FAILURE_S
                    and (
                        _never_started(st)
                        or self._observed_no_movement(start_xy)
                    )
                ):
                    # 取消/停稳过渡期把新任务直接拒掉：先撤干净再交给调用方重发。
                    # 现场 2026-10-31：拒绝响应里带整段规划距离（10246mm）和
                    # 非零 distance_time，单看字段会把「从未起步」误判成「跑了一半」。
                    self.get_logger().warn(
                        f"{poi} 运动下发 {elapsed:.2f}s 即被底盘拒绝"
                        f"（未观测到位移）：{st!r}；按瞬态处理")
                    self._cancel_and_settle(action_id)
                    return "transient"
                raise RuntimeError(f"{poi} 实机运动执行失败：status={st!r}")

            pose = self._read_pose()
            distance_m = self._distance_to(poi, pose)
            self._publish_feedback(goal_handle, "navigating", mode, distance_m)
            now = time.monotonic()

            if detect_stall and self._stall_enabled:
                xy = self._pose_xy(pose)
                if xy is not None:
                    if stall_anchor is None:
                        stall_anchor, stall_anchor_at = xy, now
                    elif math.hypot(
                        xy[0] - stall_anchor[0], xy[1] - stall_anchor[1]
                    ) >= self._stall_min_progress_m:
                        stall_anchor, stall_anchor_at = xy, now
                stalled = (
                    stall_anchor is not None
                    and now - stall_anchor_at >= self._stall_window_s
                    and distance_m > self._arrival_tolerance_m
                )
                if stalled:
                    if redispatch is None or recoveries >= self._stall_max_recoveries:
                        self._cancel_and_settle(action_id)
                        self.get_logger().error(
                            f"{poi} 运动受阻：{now - stall_anchor_at:.1f}s 内位移"
                            f"不足 {self._stall_min_progress_m:.2f}m，"
                            f"距目标 {distance_m:.2f}m，自恢复已用尽，本段失败")
                        return "blocked"
                    recoveries += 1
                    if is_return:
                        # 返程中途回退/绕行后同样需要“从当前位置续行”，
                        # 否则重发时 uptrack 会把车拉回上轨点 POI_04
                        self._return_interrupted = True
                    self.get_logger().warn(
                        f"{poi} 运动受阻：{now - stall_anchor_at:.1f}s 内位移不足"
                        f" {self._stall_min_progress_m:.2f}m，距目标"
                        f" {distance_m:.2f}m；取消后回退并侧向绕行"
                        f"（{recoveries}/{self._stall_max_recoveries}）")
                    self._cancel_and_settle(action_id)
                    # 刚判定受阻就上报 detour 相位：编排层据此在车停住时
                    # 播报「前方有障碍，正在尝试绕行」，而不是等绕行做完才说
                    self._publish_feedback(goal_handle, "detour", mode, distance_m)
                    # 回退前重新校验健康，避免带故障倒车
                    self._check_health()
                    if not self._backoff(goal_handle, poi):
                        if self._shutting_down or goal_handle.is_cancel_requested:
                            return "cancelled"
                        # 剩余导航时间不够做脱困转向时直接判受阻：主循环随后
                        # 抛「导航超时」会让返程段走 aborted，而 blocked 会走
                        # 编排层的「未返回起点」降级收尾
                        if (
                            time.monotonic() + self._stall_rotate_timeout_s
                            >= deadline
                        ):
                            self.get_logger().error(
                                f"{poi} 后退失败且剩余时间不足以脱困转向，"
                                f"本段放弃")
                            return "blocked"
                        # 后退失败（后方也堵死）：不立刻放弃，先原地转向换
                        # 个朝向让 AEB/规划重新判断，再重发原目标；转向也
                        # 失败才判本段受阻（由调用方跳过该点/降级收尾）
                        if not self._rotate_in_place(goal_handle, poi):
                            self.get_logger().error(
                                f"{poi} 受阻回退与原地转向均失败，本段放弃")
                            return "blocked"
                        # 未侧移离开轨道，按原方式（轨道）重发，不用点对点
                        action_id = redispatch()
                        self.get_logger().info(
                            f"{poi} 受阻自恢复：已原地转向并重发原目标，"
                            f"action_id={action_id}")
                        started_at = time.monotonic()
                        stall_anchor = self._pose_xy(self._read_pose())
                        stall_anchor_at = time.monotonic()
                        continue
                    if self._shutting_down or goal_handle.is_cancel_requested:
                        return "cancelled"
                    # 侧向绕行：后退之后绕到障碍侧面再重发原目标。第 1 次向左、
                    # 第 2 次向右，偏移量随恢复次数递增；同侧挪不出就立刻换另一侧，
                    # 两侧都不行才判本段受阻（由调用方跳过该点并播报）。
                    offset_m = self._stall_detour_offsets_m[
                        min(recoveries - 1,
                            len(self._stall_detour_offsets_m) - 1)
                    ]
                    side = 1 if recoveries % 2 == 1 else -1
                    detoured = False
                    for _ in range(2):
                        detour = self._detour(goal_handle, poi, offset_m, side)
                        if detour == "cancelled":
                            return "cancelled"
                        if detour == "ok":
                            detoured = True
                            break
                        side = -side
                    if not detoured:
                        self.get_logger().error(
                            f"{poi} 左右两侧绕行均失败，本段放弃")
                        return "blocked"
                    self._goal_detoured = True
                    # 已侧偏离开轨道：前往点位时改点对点直奔点位，避免重
                    # 新上轨时又被同一障碍挡住；返程段仍走返程轨道（跨全图
                    # 点对点规划会失败，uptrack 允许自主回到上轨点）
                    action_id = redispatch(point=not is_return)
                    self.get_logger().info(
                        f"{poi} 受阻自恢复：已绕行并重发原目标，"
                        f"action_id={action_id}")
                    # 重发后的新动作重新计时，有自己的瞬态判定窗口
                    started_at = time.monotonic()
                    stall_anchor = self._pose_xy(self._read_pose())
                    stall_anchor_at = time.monotonic()
                    continue

            if now - last_log >= 2.0:
                last_log = now
                self.get_logger().info(
                    f"{poi} 运动进行中：距目标 {distance_m:.2f}m，"
                    f"剩余超时 {max(0.0, deadline - now):.0f}s"
                )
            if now >= deadline:
                self._cancel_and_settle(action_id)
                raise RuntimeError(
                    f"{poi} 导航超时（{timeout_s:.0f}s），已下发取消；"
                    f"最后状态={st!r}"
                )
            time.sleep(self._action_poll_interval_s)

    def _dispatch_motion(self, poi, is_return, mode, use_point=False):
        """下发一段运动并返回 action_id；use_point=True 时强制走点对点。"""
        if use_point:
            rv = self._chassis.move_to_poi(poi)
        elif is_return:
            rv = self._dispatch_return(poi)
        elif mode == "point":
            rv = self._chassis.move_to_poi(poi)
        else:
            rv = self._chassis.move_to_locations_poi(
                self._track_id, poi, uptrack_enabled=True
            )
        action_id = rv.get("action_id") if isinstance(rv, dict) else None
        if action_id is None:
            raise RuntimeError(f"底盘未返回 action_id：{rv!r}")
        return action_id

    def _dispatch_return(self, poi):
        """返程段下发：被打断后从当前位置沿轨道继续，不退回上轨点。

        旧实现只发 ``move_to_locations_poi(uptrack_enabled=False)``，该接口
        不带起点信息，底盘仍按「上轨点」推算进度：车停在上轨点 POI_04 时
        能走，停在轨道中间就被瞬时拒绝，只能回退上轨 = 掉头回 POI_04，
        现场表现为「绕半圈又回原地」。这里改发 ``move_to_locations``，
        显式带上当前轨道投影作为起点、终点仍为起点 POI，底盘就从车所在
        位置继续向前，不会回头。
        """
        if self._return_interrupted:
            resume = self._track_resume_pose(self._return_waypoints)
            target = self._pois.get(poi)
            if resume is not None and isinstance(target, dict):
                try:
                    target_position = {
                        "x": float(target["x"]),
                        "y": float(target["y"]),
                        "yaw": float(target.get("yaw", resume["yaw"])),
                    }
                except (KeyError, TypeError, ValueError):
                    target_position = None
                if target_position is not None:
                    try:
                        rv = self._chassis.move_to_locations(
                            self._return_track_id,
                            start_position={
                                "x": resume["x"], "y": resume["y"],
                                "yaw": resume["yaw"],
                            },
                            target_position=target_position,
                            uptrack_enabled=False,
                        )
                    except Exception as exc:  # noqa: BLE001 - 续行失败回退原方式
                        self.get_logger().warn(
                            f"返程续行（不上轨）下发失败：{exc}")
                    else:
                        if isinstance(rv, dict) and rv.get("action_id") is not None:
                            self.get_logger().info(
                                f"返程续行（不上轨）：从轨道投影 "
                                f"({resume['x']:.2f}, {resume['y']:.2f}) 继续，"
                                "不回退上轨点")
                            return rv
                        self.get_logger().warn(
                            f"返程续行（不上轨）未返回 action_id（{rv!r}），"
                            "改为点对点前进")
            # 拿不到轨道投影或轨道接口不可用：直接点对点朝目标继续，
            # 绝不回退上轨点（否则就是现场看到的掉头回 POI_04）。
            self.get_logger().warn(
                f"{poi} 无法用轨道接口从当前位置续行，改为点对点前进"
                "（不回退上轨点）")
            return self._chassis.move_to_poi(poi)
        return self._chassis.move_to_locations_poi(
            self._return_track_id, poi, uptrack_enabled=True
        )

    @staticmethod
    def _track_points(waypoints):
        """轨道航点 → [(航点, x, y)]，按轨道顺序，跳过非法坐标。"""
        points = []
        for waypoint in waypoints or []:
            position = waypoint.get("position") if isinstance(waypoint, dict) else None
            if (
                not isinstance(position, dict)
                or position.get("x") is None
                or position.get("y") is None
            ):
                continue
            try:
                points.append(
                    (waypoint, float(position["x"]), float(position["y"]))
                )
            except (TypeError, ValueError):
                continue
        return points

    @staticmethod
    def _project_on_track(points, px, py):
        """把 (px, py) 投影到轨道折线，返回 (距离, x, y, dx, dy, 起点, 终点)。"""
        if len(points) < 2:
            return None
        best = None
        for start, end in zip(points, points[1:]):
            dx, dy = end[1] - start[1], end[2] - start[2]
            length = math.hypot(dx, dy)
            if length < 0.05:
                continue     # 同坐标的到达/出发航点，跳过零长段
            t = ((px - start[1]) * dx + (py - start[2]) * dy) / (length * length)
            t = max(0.0, min(1.0, t))
            cx, cy = start[1] + t * dx, start[2] + t * dy
            distance = math.hypot(px - cx, py - cy)
            if best is None or distance < best[0]:
                best = (distance, cx, cy, dx, dy, start, end)
        return best

    def _track_resume_pose(self, waypoints):
        """当前位姿在轨道上的最近投影点 + 该段行进朝向；无法计算返回 None。"""
        xy = self._pose_xy(self._read_pose())
        if xy is None:
            return None
        best = self._project_on_track(self._track_points(waypoints), xy[0], xy[1])
        if best is None:
            return None
        _, cx, cy, dx, dy = best[:5]
        if math.hypot(dx, dy) < 0.05:
            return None
        return {"x": cx, "y": cy, "yaw": math.atan2(dy, dx)}

    def _track_point_ahead(self, waypoints, ahead_m):
        """返程轨道上「当前位姿投影点沿行进方向前进 ahead_m」的落点。

        侧向退让后车已离开轨道，直接回投影点会贴住刚避让的人；沿轨道折线
        往前推一段再重接，既回到轨道又留出安全距离。航点按行进顺序排列，
        因此沿列表方向前进即为返程方向。无法计算（无位姿/航点）返回 None。
        """
        points = self._track_points(waypoints)
        if len(points) < 2:
            return None
        xy = self._pose_xy(self._read_pose())
        if xy is None:
            return None
        best = None
        for index in range(len(points) - 1):
            start, end = points[index], points[index + 1]
            dx, dy = end[1] - start[1], end[2] - start[2]
            length = math.hypot(dx, dy)
            if length < 0.05:
                continue     # 同坐标的到达/出发航点，跳过零长段
            t = ((xy[0] - start[1]) * dx + (xy[1] - start[2]) * dy) / (length * length)
            t = max(0.0, min(1.0, t))
            cx, cy = start[1] + t * dx, start[2] + t * dy
            distance = math.hypot(xy[0] - cx, xy[1] - cy)
            if best is None or distance < best[0]:
                best = (distance, index, cx, cy)
        if best is None:
            return None
        _, index, px, py = best
        need = max(0.0, float(ahead_m))
        yaw = None
        while True:
            end = points[index + 1]
            dx, dy = end[1] - px, end[2] - py
            length = math.hypot(dx, dy)
            if length > 0.05:
                yaw = math.atan2(dy, dx)
                if need <= length:
                    ratio = need / length
                    return {"x": px + ratio * dx, "y": py + ratio * dy, "yaw": yaw}
                need -= length
            px, py = end[1], end[2]
            index += 1
            if index + 1 >= len(points):
                break
        if yaw is None:
            return None
        # 轨道剩余长度不足 ahead_m：就取轨道终点（朝向仍按该段行进方向）
        return {"x": px, "y": py, "yaw": yaw}

    def _return_lateral_offset(self, xy):
        """当前位姿相对返程轨道的带符号横向偏移（左正右负，米）。

        用于分段推进时保持介入后的侧移量：车被侧移到走廊一侧后，若马上
        回轨道中心线，会和刚避开的人再次相遇。小于 0.20m 视为正常循迹
        误差返回 0；无法投影或航点不足返回 0。
        """
        points = self._track_points(self._return_waypoints)
        if len(points) < 2 or xy is None:
            return 0.0
        best = None
        for index in range(len(points) - 1):
            start, end = points[index], points[index + 1]
            dx, dy = end[1] - start[1], end[2] - start[2]
            length = math.hypot(dx, dy)
            if length < 0.05:
                continue     # 同坐标的到达/出发航点，跳过零长段
            t = ((xy[0] - start[1]) * dx + (xy[1] - start[2]) * dy) / (length * length)
            t = max(0.0, min(1.0, t))
            cx, cy = start[1] + t * dx, start[2] + t * dy
            distance = math.hypot(xy[0] - cx, xy[1] - cy)
            if best is None or distance < best[0]:
                best = (distance, dx, dy, length, cx, cy)
        if best is None:
            return 0.0
        _, dx, dy, length, cx, cy = best
        # 行进方向的左侧单位向量为 (-dy, dx)
        side = ((xy[0] - cx) * (-dy) + (xy[1] - cy) * dx) / length
        if abs(side) < 0.20:
            return 0.0
        return max(-_RETURN_OFFSET_MAX_M, min(_RETURN_OFFSET_MAX_M, side))

    def _advance_return_in_stages(self, goal_handle, poi, deadline):
        """返程轨道接口拒绝中途续行时，沿返程轨道分段点对点推进到目标。

        固件只允许 ``move_to_locations`` 从轨道上轨点 (POI_04) 续行，传入
        轨道中间投影作为 start_position 会 0.11s 即 failed（2026-11-03 返程
        在 (19.6~16.6, 2.2) 连续瞬拒）；而一次性跨图点对点（16m）底盘接受
        后又不产生位移。这里沿返程轨道折线每 ``_RETURN_STAGE_M`` 取一个
        落点，用 ``move_to`` 逐段推进，每段都做软限位/走廊校验。
        返回 ok / failed / cancelled。
        """
        target = self._pois.get(poi)
        if not isinstance(target, dict):
            self.get_logger().warn(f"{poi} 分段推进失败：缺少目标点位配置")
            return "failed"
        try:
            tx, ty = float(target["x"]), float(target["y"])
            tyaw = float(target.get("yaw", 0.0))
        except (KeyError, TypeError, ValueError):
            self.get_logger().warn(f"{poi} 分段推进失败：目标坐标非法")
            return "failed"

        # 16m 返程按 4m 一段最多 5 段；留余量到 10 段，防止绕路/重试时死循环
        for _ in range(10):
            if goal_handle.is_cancel_requested or self._shutting_down:
                return "cancelled"
            xy = self._pose_xy(self._read_pose())
            if xy is None:
                self.get_logger().warn(f"{poi} 分段推进失败：无法读取位姿")
                return "failed"
            remaining = math.hypot(tx - xy[0], ty - xy[1])
            if remaining <= 0.30:
                return "ok"
            side = 0.0
            if remaining <= _RETURN_STAGE_M:
                nx, ny, nyaw = tx, ty, tyaw
            else:
                ahead = self._track_point_ahead(
                    self._return_waypoints, _RETURN_STAGE_M)
                if ahead is None:
                    ratio = _RETURN_STAGE_M / remaining
                    nx = xy[0] + (tx - xy[0]) * ratio
                    ny = xy[1] + (ty - xy[1]) * ratio
                    nyaw = math.atan2(ty - ny, tx - nx)
                else:
                    nx, ny, nyaw = ahead["x"], ahead["y"], ahead["yaw"]
                    # 介入后车被侧移到走廊一侧：沿轨道前进时保持该横向
                    # 偏移，别立刻回轨道中心线，否则又开回刚避开的人面前。
                    side = self._return_lateral_offset(xy)
                    if side != 0.0:
                        nx += -math.sin(nyaw) * side
                        ny += math.cos(nyaw) * side
            reason = self._point_range_error(nx, ny)
            corridor_error = self._point_corridor_error(nx, ny)
            if (reason is not None or corridor_error is not None) and side != 0.0:
                # 保持偏移的落点出界：退回轨道中心线再走
                nx -= -math.sin(nyaw) * side
                ny -= math.cos(nyaw) * side
                side = 0.0
                reason = self._point_range_error(nx, ny)
                corridor_error = self._point_corridor_error(nx, ny)
            if reason is not None:
                self.get_logger().warn(
                    f"{poi} 分段落点 ({nx:.2f}, {ny:.2f}) 超出软限位"
                    f"（{reason}），放弃")
                return "failed"
            corridor_error = self._point_corridor_error(nx, ny)
            if corridor_error is not None:
                self.get_logger().warn(
                    f"{poi} 分段落点 ({nx:.2f}, {ny:.2f}) 超出可行走廊"
                    f"（{corridor_error}），放弃")
                return "failed"

            self.get_logger().warn(
                f"{poi} 分段点对点推进：目标 ({nx:.2f}, {ny:.2f})，"
                f"剩余 {remaining:.2f}m")
            verdict = None
            stage_deadline = min(
                time.monotonic() + _RETURN_STAGE_TIMEOUT_S, deadline)
            # 上一段刚停稳时固件可能瞬拒（实测 0.11s failed），等一拍重发一次
            for attempt in range(2):
                try:
                    rv = self._chassis.move_to(nx, ny, nyaw)
                except Exception as exc:  # noqa: BLE001 - 分段推进失败即判受阻
                    self.get_logger().error(
                        f"{poi} 分段点对点指令下发失败：{exc}")
                    return "failed"
                action_id = rv.get("action_id") if isinstance(rv, dict) else None
                if action_id is None:
                    self.get_logger().error(
                        f"{poi} 分段点对点未返回 action_id：{rv!r}")
                    return "failed"
                while True:
                    if goal_handle.is_cancel_requested or self._shutting_down:
                        self._cancel_and_settle(action_id)
                        return "cancelled"
                    if time.monotonic() > stage_deadline:
                        self._cancel_and_settle(action_id)
                        self.get_logger().warn(
                            f"{poi} 分段点对点超时"
                            f"（{_RETURN_STAGE_TIMEOUT_S:.0f}s）")
                        return "failed"
                    try:
                        st = self._chassis.get_action_status(action_id)
                    except Exception:  # noqa: BLE001 - 轮询抖动继续等
                        st = None
                    verdict = _classify_action_status(st)
                    if verdict in ("success", "failure"):
                        break
                    time.sleep(self._action_poll_interval_s)
                if verdict == "success":
                    break
                if not _never_started(st) or attempt == 1:
                    self.get_logger().warn(
                        f"{poi} 分段点对点被底盘拒绝：status={st!r}")
                    return "failed"
                if self._interruptible_sleep(goal_handle, 1.0):
                    return "cancelled"
                stage_deadline = min(
                    time.monotonic() + _RETURN_STAGE_TIMEOUT_S, deadline)
            if verdict != "success":
                return "failed"
            # 停稳过渡期留一拍，避免下一段刚下发就被瞬拒
            if self._interruptible_sleep(goal_handle, 1.0):
                return "cancelled"
        self.get_logger().warn(f"{poi} 分段推进段数用尽，仍未到达")
        return "failed"

    def _track_departure_yaw(self, is_return, pose):
        """计算当前轨道段的行进朝向；无法确定时返回 None。

        用「车位置到轨道线段的投影」找所在线段，所以车停在两个航点之间
        也能给出行进方向（旧实现要求距最近航点 <=1.0m，中途介入停车后
        会直接放弃对齐）。方向优先取该段起点的第一个控制点（曲线段），
        没有控制点时取段终点（直线段）。
        """
        waypoints = self._return_waypoints if is_return else self._outbound_waypoints
        if not waypoints or not isinstance(pose, dict):
            return None
        try:
            px, py = float(pose["x"]), float(pose["y"])
        except (KeyError, TypeError, ValueError):
            return None
        best = self._project_on_track(self._track_points(waypoints), px, py)
        if best is None:
            return None
        start, end = best[5], best[6]

        ahead = None
        for point in start[0].get("control_points") or []:
            if (
                isinstance(point, dict)
                and point.get("x") is not None
                and point.get("y") is not None
            ):
                ahead = (float(point["x"]), float(point["y"]))
                break
        if ahead is None:
            ahead = (end[1], end[2])

        dx = ahead[0] - start[1]
        dy = ahead[1] - start[2]
        if math.hypot(dx, dy) < 0.05:
            return None
        return math.atan2(dy, dx)

    def _align_travel_direction(
        self, goal_handle, poi, is_return, deadline, timeout_s
    ) -> str:
        """轨道段出发前原地转向到行进方向，避免车头未朝前导致横着平移。"""
        pose = self._read_pose()
        target_yaw = self._track_departure_yaw(is_return, pose)
        if target_yaw is None:
            self.get_logger().info(
                f"{poi} 未能确定轨道出发方向，跳过出发朝向对齐"
            )
            return "ok"
        return self._align_departure_yaw(
            goal_handle, poi, target_yaw, deadline, timeout_s, "轨道行进朝向"
        )

    def _align_point_departure(
        self, goal_handle, poi, target, deadline, timeout_s
    ) -> str:
        """点对点段出发前把车头转向目标点。

        move_to_poi 默认 flags=1「边转边移」：车头背离目标时会一边画弧一边
        横向平移，现场看到的就是「横着跑」。先原地转正再走，路径才顺。
        """
        pose = self._read_pose()
        try:
            px, py = float(pose["x"]), float(pose["y"])
            tx, ty = float(target["x"]), float(target["y"])
        except (KeyError, TypeError, ValueError):
            self.get_logger().warn(
                f"{poi} 位姿或点位缺少 x/y，跳过点对点出发朝向对齐："
                f"pose={pose!r} target={target!r}"
            )
            return "ok"
        if math.hypot(tx - px, ty - py) < _POINT_ALIGN_MIN_DIST_M:
            self.get_logger().info(
                f"{poi} 距目标不足 {_POINT_ALIGN_MIN_DIST_M:.1f}m，"
                "跳过点对点出发朝向对齐"
            )
            return "ok"
        return self._align_departure_yaw(
            goal_handle, poi, math.atan2(ty - py, tx - px),
            deadline, timeout_s, "点对点行进朝向",
        )

    def _align_departure_yaw(
        self, goal_handle, poi, target_yaw, deadline, timeout_s, reason
    ) -> str:
        """原地转向到指定朝向。

        对齐失败只告警不阻断导航；取消请求返回 cancelled。
        """
        pose = self._read_pose()
        try:
            x, y = float(pose["x"]), float(pose["y"])
            actual = float(pose["yaw"])
        except (KeyError, TypeError, ValueError):
            self.get_logger().warn(
                f"{poi} 位姿缺少 x/y/yaw，跳过{reason}对齐：{pose!r}"
            )
            return "ok"

        error = abs(math.atan2(
            math.sin(target_yaw - actual), math.cos(target_yaw - actual)
        ))
        if error <= _TRAVEL_ALIGN_TOLERANCE_RAD:
            self.get_logger().info(
                f"{poi} {reason}已对齐：目标={target_yaw:.3f}rad，"
                f"实际={actual:.3f}rad，误差={error:.3f}rad"
            )
            return "ok"

        self.get_logger().info(
            f"{poi} 出发前{reason}对齐：目标={target_yaw:.3f}rad，"
            f"实际={actual:.3f}rad，误差={error:.3f}rad，原地转向"
        )
        rv = self._chassis.move_to(x, y, target_yaw, flags=0)
        action_id = rv.get("action_id") if isinstance(rv, dict) else None
        if action_id is None:
            self.get_logger().warn(
                f"{poi} {reason}对齐未返回 action_id：{rv!r}，继续下发运动"
            )
            return "ok"
        outcome = self._run_chassis_action(
            goal_handle, action_id, poi, "track", deadline, timeout_s,
            detect_stall=False,
        )
        if outcome == "cancelled":
            return "cancelled"
        if outcome == "transient":
            # 原地转向只是让行进更顺，被底盘瞬时拒绝就跳过，不阻断导航
            self.get_logger().warn(
                f"{poi} {reason}对齐被底盘瞬时拒绝，跳过对齐继续下发运动")
            return "ok"

        pose = self._read_pose()
        try:
            actual = float(pose["yaw"])
        except (KeyError, TypeError, ValueError):
            self.get_logger().warn(f"{poi} 对齐后无法读取朝向：{pose!r}")
            return "ok"
        error = abs(math.atan2(
            math.sin(target_yaw - actual), math.cos(target_yaw - actual)
        ))
        if error > _TRAVEL_ALIGN_TOLERANCE_RAD:
            self.get_logger().warn(
                f"{poi} {reason}对齐后误差 {error:.3f}rad 超过 "
                f"{_TRAVEL_ALIGN_TOLERANCE_RAD:.2f}，继续下发运动"
            )
        else:
            self.get_logger().info(
                f"{poi} {reason}对齐完成：目标={target_yaw:.3f}rad，"
                f"实际={actual:.3f}rad，误差={error:.3f}rad"
            )
        return "ok"

    def _rotate_to(self, goal_handle, poi, target, target_yaw,
                   deadline, timeout_s) -> str:
        """到点原地转向到目标 yaw，并校验实际朝向；偏差过大重试一次。"""
        pose = self._chassis.get_pose()
        if isinstance(pose, dict) and pose.get("yaw") is not None:
            actual = float(pose["yaw"])
            error = abs(math.atan2(
                math.sin(target_yaw - actual), math.cos(target_yaw - actual)
            ))
            if error <= 0.10:
                # 轨道末航点朝向已与点位朝向一致，不再原地补转
                self.get_logger().info(
                    f"{poi} 到点朝向已符合：目标={target_yaw:.3f}rad，"
                    f"实际={actual:.3f}rad，误差={error:.3f}rad，无需转向"
                )
                return "ok"
        actual = None
        for attempt in range(2):
            rv = self._chassis.move_to(
                float(target["x"]), float(target["y"]), target_yaw, flags=0
            )
            action_id = rv.get("action_id") if isinstance(rv, dict) else None
            if action_id is None:
                raise RuntimeError(f"转向动作未返回 action_id：{rv!r}")
            outcome = self._run_chassis_action(
                goal_handle, action_id, poi, "track", deadline, timeout_s,
                detect_stall=False,
            )
            if outcome == "cancelled":
                return "cancelled"
            if outcome == "transient":
                if attempt == 0:
                    self.get_logger().warn(
                        f"{poi} 到点转向被底盘瞬时拒绝，1.0s 后重试")
                    time.sleep(1.0)
                    continue
                raise RuntimeError(f"{poi} 到点转向被底盘连续立即拒绝")
            pose = self._chassis.get_pose()
            if not isinstance(pose, dict) or pose.get("yaw") is None:
                raise RuntimeError(f"无法读取朝向用于校验：{pose!r}")
            actual = float(pose["yaw"])
            error = abs(math.atan2(
                math.sin(target_yaw - actual), math.cos(target_yaw - actual)
            ))
            self.get_logger().info(
                f"{poi} 到点朝向校验：目标={target_yaw:.3f}rad，"
                f"实际={actual:.3f}rad，误差={error:.3f}rad"
            )
            if error <= 0.10:
                return "ok"
            if attempt == 0:
                self.get_logger().warn(
                    f"{poi} 朝向误差 {error:.3f}rad 超过 0.10，重试一次"
                )
        raise RuntimeError(
            f"{poi} 朝向未到位：目标={target_yaw:.3f}rad，实际={actual:.3f}rad"
        )

    def _check_arrival_position(self, poi: str) -> None:
        """到点位置校验：实际位置与 POI 偏差超过容差则判失败。

        固件返回动作成功只代表动作走完，定位漂移或轨道提前结束时车可能停在别处，
        仅校验朝向不足以证明巡检点到位。
        """
        target = self._pois[poi]
        pose = self._chassis.get_pose()
        if (
            not isinstance(pose, dict)
            or pose.get("x") is None
            or pose.get("y") is None
        ):
            raise RuntimeError(f"{poi} 到点位置校验失败：无法读取位姿 {pose!r}")
        error = math.hypot(
            float(target["x"]) - float(pose["x"]),
            float(target["y"]) - float(pose["y"]),
        )
        self.get_logger().info(
            f"{poi} 到点位置校验：目标=({float(target['x']):.3f}, "
            f"{float(target['y']):.3f})，实际=({float(pose['x']):.3f}, "
            f"{float(pose['y']):.3f})，误差={error:.3f}m"
        )
        if error > self._arrival_tolerance_m:
            raise RuntimeError(
                f"{poi} 到点位置偏差 {error:.2f}m 超过容差 "
                f"{self._arrival_tolerance_m:.2f}m"
            )

    def _dwell_for(self, poi: str) -> float:
        """该点位到点后的停留时长。

        起点不停留；`dwell_overrides` 优先；未单独配置的过点点位（transit_pois）不停留；
        其余巡检点位用全局 `dwell_s`。
        """
        if poi == self._start_poi:
            return 0.0
        seconds = self._dwell_overrides.get(poi)
        if seconds is not None:
            return seconds
        if poi in self._transit_pois:
            return 0.0
        return self._dwell_s

    def _dwell(self, goal_handle, poi, deadline, seconds) -> bool:
        """到点停留；返回 False 表示收到取消，由调用方返回 cancelled。"""
        end = min(time.monotonic() + seconds, deadline)
        self.get_logger().info(f"{poi} 到点停留 {seconds:.1f}s")
        while time.monotonic() < end:
            if self._shutting_down or goal_handle.is_cancel_requested:
                return False
            time.sleep(0.1)
        if time.monotonic() >= deadline:
            raise RuntimeError(f"{poi} 停留期间达到 goal 超时")
        return True

    def _cancel_and_settle(self, action_id=None):
        """下发取消并短暂等待动作退出 running，避免带着运动关 AEB。"""
        self._cancel_action(action_id)
        settle_deadline = time.monotonic() + 2.0
        while time.monotonic() < settle_deadline:
            try:
                st = self._chassis.get_action_status(action_id)
            except Exception:  # noqa: BLE001 - 查询失败不再等待
                break
            if _classify_action_status(st) != "running":
                break
            time.sleep(0.2)
        self._cancel_action()

    def _cancel_action(self, action_id=None):
        try:
            self._chassis.cancel_action([action_id] if action_id else None)
        except Exception as exc:  # noqa: BLE001 - 清理失败只告警
            self.get_logger().warn(f"取消底盘动作失败：{exc}")

    def _set_aeb(self, on: bool):
        try:
            self._chassis.set_aeb_control("On" if on else "Off")
        except Exception as exc:  # noqa: BLE001 - AEB 失败不阻塞主流程
            state = "开启" if on else "关闭"
            self.get_logger().warn(f"AEB {state}失败（继续）：{exc}")

    def _apply_speed(self, limit, label):
        """按段写入限速；首次写入前备份原值，失败立即回滚。"""
        if not self._speed_backup:
            previous = self._chassis.get_speed_params() or {}
            old_move = previous.get("max_moving_speed")
            if old_move is None:
                raise RuntimeError("读取底盘限速失败，无法备份原值，终止")
            self._speed_backup = {
                "max_moving_speed": old_move,
                "max_angular_speed": previous.get("max_angular_speed"),
            }
            old_desc = f"{old_move} m/s"
        else:
            old_desc = f"{self._speed_backup['max_moving_speed']} m/s"
        try:
            current = (self._chassis.get_speed_params() or {}).get(
                "max_moving_speed"
            )
            if current is None or abs(float(current) - limit) >= 1e-3:
                self._chassis.set_speed_params(max_moving_speed=limit)
            if not self._wait_speed(limit):
                raise RuntimeError(
                    f"限速 {limit} m/s 未在 {self._speed_apply_timeout_s}s 内生效"
                )
        except Exception:
            self._restore_speed_limit()
            raise
        self._speed_limit_applied = True
        self.get_logger().info(
            f"{label}限速 {limit} m/s（底盘原值 {old_desc}）"
        )

    def _wait_speed(self, target) -> bool:
        deadline = time.monotonic() + self._speed_apply_timeout_s
        while time.monotonic() < deadline:
            try:
                current = (self._chassis.get_speed_params() or {}).get(
                    "max_moving_speed"
                )
            except Exception:  # noqa: BLE001 - 单次读取失败继续重试
                current = None
            if current is not None and abs(float(current) - float(target)) < 1e-3:
                return True
            time.sleep(0.2)
        return False

    def _restore_speed_limit(self):
        """恢复限速原值；成功后清空备份，失败保留以便关闭时再试。"""
        backup = self._speed_backup
        if not backup:
            return
        target = backup.get("max_moving_speed")
        try:
            self._chassis.set_speed_params(
                max_moving_speed=target,
                max_angular_speed=backup.get("max_angular_speed"),
            )
            if self._wait_speed(target):
                self.get_logger().info(f"已恢复底盘限速 {target} m/s")
                self._speed_limit_applied = False
                self._speed_backup = None
            else:
                self.get_logger().error(
                    f"底盘限速恢复未确认，目标 {target} m/s，请人工检查"
                )
        except Exception as exc:  # noqa: BLE001 - 回滚失败必须显式暴露
            self.get_logger().error(f"恢复底盘限速失败：{exc}")

    def _cancelled(self, goal_handle, result, poi: str):
        result.success = False
        result.phase = "cancelled"
        result.is_return = bool(goal_handle.request.is_return)
        result.message = f"{poi} 导航被取消；{self._progress()}"
        if result.is_return:
            # 返程被介入/取消打断：下次重发要从当前位置续行，不回上轨点
            self._return_interrupted = True
        self.get_logger().warn(result.message)
        try:
            goal_handle.canceled()
        except Exception as exc:  # noqa: BLE001 - 关闭过程中 goal 可能已失效
            self.get_logger().warn(f"标记取消状态失败：{exc}")
        return result

    def _distance_to(self, poi: str, pose) -> float:
        """当前位姿到目标 POI 的直线距离；位姿或坐标不可用时返回 0.0。"""
        target = self._pois.get(poi)
        if not isinstance(pose, dict) or not isinstance(target, dict):
            return 0.0
        try:
            return math.hypot(
                float(target["x"]) - float(pose["x"]),
                float(target["y"]) - float(pose["y"]),
            )
        except (KeyError, TypeError, ValueError):
            return 0.0

    def _read_pose(self):
        try:
            pose = self._chassis.get_pose()
        except Exception as exc:  # noqa: BLE001
            return f"读取失败({exc})"
        if isinstance(pose, dict):
            return {k: pose.get(k) for k in ("x", "y", "yaw") if pose.get(k) is not None}
        return pose

    @staticmethod
    def _pose_xy(pose):
        """从位姿字典取 (x, y)；缺失/非法返回 None（受阻检测用）。"""
        if not isinstance(pose, dict):
            return None
        try:
            x, y = float(pose["x"]), float(pose["y"])
        except (KeyError, TypeError, ValueError):
            return None
        if not (math.isfinite(x) and math.isfinite(y)):
            return None
        return x, y

    def _observed_no_movement(self, start_xy, threshold_m: float = 0.10) -> bool:
        """从动作下发到现在，是否确认「完全没有位移」。

        只有两个时刻都读到位姿、且位移小于 threshold 才返回 True；
        任一次位姿不可用返回 False（无法判断时按「已起步」保守处理）。
        """
        if start_xy is None:
            return False
        current = self._pose_xy(self._read_pose())
        if current is None:
            return False
        return math.hypot(
            current[0] - start_xy[0], current[1] - start_xy[1]
        ) < threshold_m

    def _backoff(self, goal_handle, poi: str) -> bool:
        """受阻后沿车头反向退一小段，给底盘留出重新起步的空间。

        距离由配置限定在底盘 move_by 允许的 [50, 5000] mm 内，速度用
        `v` 参数限制（不改底盘限速配置）。返回 False 表示后退失败/被取消。
        """
        distance_mm = int(round(self._stall_backoff_m * 1000.0))
        distance_mm = max(50, min(5000, distance_mm))
        self.get_logger().warn(
            f"{poi} 受阻，后退 {distance_mm / 1000.0:.2f}m"
            f"（限速 {self._stall_backoff_speed_mps:.2f} m/s）后重发原目标")
        try:
            rv = self._chassis.move_by(
                direction=math.pi, distance=distance_mm, move_mode=0,
                v=self._stall_backoff_speed_mps,
            )
        except Exception as exc:  # noqa: BLE001 - 后退失败只影响本段
            self.get_logger().error(f"{poi} 后退指令下发失败：{exc}")
            return False
        action_id = rv.get("action_id") if isinstance(rv, dict) else None
        if action_id is None:
            self.get_logger().error(f"{poi} 后退指令未返回 action_id：{rv!r}")
            return False
        deadline = time.monotonic() + self._stall_backoff_timeout_s
        while time.monotonic() < deadline:
            if self._shutting_down or goal_handle.is_cancel_requested:
                self._cancel_and_settle(action_id)
                return False
            try:
                st = self._chassis.get_action_status(action_id)
            except Exception:  # noqa: BLE001 - 轮询抖动继续等
                st = None
            verdict = _classify_action_status(st)
            if verdict == "success":
                self.get_logger().info(
                    f"{poi} 已后退 {distance_mm / 1000.0:.2f}m")
                return True
            if verdict == "failure":
                self.get_logger().warn(f"{poi} 后退动作失败：status={st!r}")
                return False
            time.sleep(self._action_poll_interval_s)
        self._cancel_and_settle(action_id)
        self.get_logger().warn(
            f"{poi} 后退超时（{self._stall_backoff_timeout_s:.1f}s）")
        return False

    def _rotate_in_place(self, goal_handle, poi: str) -> bool:
        """后退失败时的脱困动作：原地转向换个朝向，让 AEB/规划重新判断。

        左右交替（第 1 次左转、第 2 次右转），角度由 stall.retreat_rotate_deg
        控制。返回 False 表示转向失败/超时/被取消，由调用方判本段受阻。
        """
        angle = math.radians(self._stall_rotate_deg)
        direction = angle if self._stall_rotate_turns % 2 == 0 else -angle
        self._stall_rotate_turns += 1
        direction = max(-math.pi, min(math.pi, direction))
        self.get_logger().warn(
            f"{poi} 后退失败，原地转向 {math.degrees(direction):+.0f}° 脱困"
            f"（限速 {self._stall_rotate_speed:.2f} rad/s）")
        try:
            rv = self._chassis.move_by(
                direction=direction, distance=0, move_mode=1,
                w=self._stall_rotate_speed,
            )
        except Exception as exc:  # noqa: BLE001 - 转向失败只影响本段
            self.get_logger().error(f"{poi} 原地转向指令下发失败：{exc}")
            return False
        action_id = rv.get("action_id") if isinstance(rv, dict) else None
        if action_id is None:
            self.get_logger().error(f"{poi} 原地转向未返回 action_id：{rv!r}")
            return False
        deadline = time.monotonic() + self._stall_rotate_timeout_s
        while time.monotonic() < deadline:
            if self._shutting_down or goal_handle.is_cancel_requested:
                self._cancel_and_settle(action_id)
                return False
            try:
                st = self._chassis.get_action_status(action_id)
            except Exception:  # noqa: BLE001 - 轮询抖动继续等
                st = None
            verdict = _classify_action_status(st)
            if verdict == "success":
                self.get_logger().info(f"{poi} 原地转向脱困完成")
                return True
            if verdict == "failure":
                self.get_logger().warn(f"{poi} 原地转向失败：status={st!r}")
                return False
            time.sleep(self._action_poll_interval_s)
        self._cancel_and_settle(action_id)
        self.get_logger().warn(
            f"{poi} 原地转向超时（{self._stall_rotate_timeout_s:.1f}s）")
        return False

    def _detour(self, goal_handle, poi: str, offset_m: float, side: int) -> str:
        """侧向绕行：从当前位置向「目标连线」的左侧/右侧平移 offset_m。

        side=+1 为左侧（沿朝向逆时针 90°），side=-1 为右侧。坐标按当前位姿
        计算，保持当前朝向只做平移（flags=0），避开重规划与跨区直线。
        绕行点先过应用层软限位校验。返回 ok / failed / cancelled。
        """
        pose = self._read_pose()
        xy = self._pose_xy(pose)
        if xy is None:
            self.get_logger().warn(f"{poi} 绕行失败：无法读取当前位姿")
            return "failed"
        target = self._pois[poi]
        dx = float(target["x"]) - xy[0]
        dy = float(target["y"]) - xy[1]
        norm = math.hypot(dx, dy)
        if norm < 1e-6:
            self.get_logger().warn(f"{poi} 绕行失败：已在目标点上，无绕行方向")
            return "failed"
        # (dx, dy) 前进方向的左侧为 (-dy, dx)（ROS 右手系 x 前 y 左）
        ux, uy = -dy / norm, dx / norm
        nx = xy[0] + side * offset_m * ux
        ny = xy[1] + side * offset_m * uy
        reason = self._point_range_error(nx, ny)
        if reason is not None:
            self.get_logger().warn(
                f"{poi} 绕行点 ({nx:.2f}, {ny:.2f}) 超出软限位（{reason}），"
                f"放弃{'左' if side > 0 else '右'}侧")
            return "failed"
        corridor_error = self._point_corridor_error(nx, ny)
        if corridor_error is not None:
            self.get_logger().warn(
                f"{poi} 绕行点 ({nx:.2f}, {ny:.2f}) 超出可行走廊"
                f"（{corridor_error}），放弃{'左' if side > 0 else '右'}侧")
            return "failed"
        yaw = 0.0
        if isinstance(pose, dict) and pose.get("yaw") is not None:
            yaw = float(pose["yaw"])
        self.get_logger().warn(
            f"{poi} 受阻绕行：向{'左' if side > 0 else '右'}侧平移 "
            f"{offset_m:.2f}m 到 ({nx:.2f}, {ny:.2f})")
        try:
            rv = self._chassis.move_to(nx, ny, yaw, flags=0)
        except Exception as exc:  # noqa: BLE001 - 绕行失败交给上层换侧/放弃
            self.get_logger().error(f"{poi} 绕行指令下发失败：{exc}")
            return "failed"
        action_id = rv.get("action_id") if isinstance(rv, dict) else None
        if action_id is None:
            self.get_logger().error(f"{poi} 绕行指令未返回 action_id：{rv!r}")
            return "failed"
        deadline = time.monotonic() + self._stall_detour_timeout_s
        while time.monotonic() < deadline:
            if self._shutting_down or goal_handle.is_cancel_requested:
                self._cancel_and_settle(action_id)
                return "cancelled"
            try:
                st = self._chassis.get_action_status(action_id)
            except Exception:  # noqa: BLE001 - 轮询抖动继续等
                st = None
            verdict = _classify_action_status(st)
            if verdict == "success":
                self.get_logger().info(f"{poi} 侧向绕行到位")
                return "ok"
            if verdict == "failure":
                self.get_logger().warn(f"{poi} 侧向绕行失败：status={st!r}")
                return "failed"
            time.sleep(self._action_poll_interval_s)
        self._cancel_and_settle(action_id)
        self.get_logger().warn(
            f"{poi} 侧向绕行超时（{self._stall_detour_timeout_s:.1f}s）")
        return "failed"

    def _publish_feedback(
        self, goal_handle, phase: str, mode: str, distance_m: float
    ) -> None:
        feedback = NavigateTo.Feedback()
        feedback.phase = phase
        feedback.mode = mode
        feedback.distance_remaining_m = float(distance_m)
        goal_handle.publish_feedback(feedback)

    def begin_shutdown(self) -> None:
        """在 executor 停机前调用，让在途 goal 尽快看到关闭标志。"""
        self._shutting_down = True

    def destroy_node(self):
        # 先在途 goal 尽快退出，再等它让出锁，最后关底盘会话
        self._shutting_down = True
        if self._goal_lock.acquire(timeout=2.0):
            self._goal_lock.release()
        else:
            self.get_logger().warn("仍有 goal 未退出，直接关闭底盘会话")
        # 退出前必须停车并回滚限速原值
        self._cancel_action()
        self._restore_speed_limit()
        try:
            self._chassis.close()
        except Exception:  # noqa: BLE001
            pass
        return super().destroy_node()


def main(argv=None) -> None:
    # 用 rclpy 默认的信号处理：它能在 SIGINT/SIGTERM 时唤醒 rcl_wait 并关闭 context，
    # 自己用 Python handler 反而唤不醒阻塞中的 rcl_wait（信号会一直挂起）。
    rclpy.init(args=argv)
    node = NavigateToServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.begin_shutdown()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
