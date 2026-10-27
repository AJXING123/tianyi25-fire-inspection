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

from fire_inspection_interfaces.action import NavigateTo

# robot_controller 是普通 Python 包，未被 colcon 安装，需手动注入路径
_ROBOT_CONTROLLER_SRC = os.environ.get(
    "ROBOT_CONTROLLER_SRC", "/home/nvidia/robot_controller/src"
)
if _ROBOT_CONTROLLER_SRC not in sys.path:
    sys.path.insert(0, _ROBOT_CONTROLLER_SRC)

from robot_controller.chassis import KnewbotChassisNav  # noqa: E402


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


# 底盘动作终态判定（兼容不同固件返回的字段与措辞，与 run_POI.py 保持一致）
_TERMINAL_SUCCESS = {
    "successed", "succeeded", "success", "finished", "done", "completed",
}
_TERMINAL_FAIL = {
    "failed", "failure", "error",
    "cancelled", "canceled", "stopped", "aborted", "rejected",
}


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
        self.get_logger().info(
            f"NavigateTo 服务端就绪：dry_run={self._dry_run}，"
            f"点位顺序={[self._start_poi] + self._poi_order}"
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

            self._return_track_id = None
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
                self._check_return_track(return_track)

            self._check_poi_on_track(
                [n for n in required if n not in self._transit_pois], outbound
            )
            self._warn_track_yaw_mismatch(
                [n for n in required if n not in self._transit_pois],
                outbound,
                self._track_name,
            )

            # dry_run 不下发运动，定位异常只告警；实机模式必须严格通过
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

    def _check_range_limit(self, names) -> None:
        """按配置的应用层软限位校验 POI 坐标。

        配置存在但格式非法时直接报错，避免"看起来有保护、实际没生效"。
        """
        if not self._range_limit:
            return
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

    def _check_localization(self, strict: bool) -> None:
        """定位校验。dry_run 只警告，因为不下发运动；实机模式必须严格通过。"""
        slam_status = self._read_slam_status()
        if slam_status == 0:
            return
        message = f"定位状态异常 slam_status={slam_status}（需为 0）"
        if strict:
            raise RuntimeError(message)
        self.get_logger().warn(f"{message}；dry_run 不下发运动，仅记录为风险")

    def _check_health(self) -> None:
        """底盘健康校验：存在 A 类故障时拒绝运动，并给出 DTC 描述与处理建议。"""
        health = self._chassis.get_robot_health()
        if not (
            getattr(health, "hasFatal", False) or getattr(health, "hasError", False)
        ):
            return
        details = "；".join(
            f"0x{entry.id:08X} {entry.description}"
            f"（处理：{entry.solution}）"
            for entry in getattr(health, "baseError", [])
        ) or "未返回 DTC 详情"
        raise RuntimeError(f"底盘健康检查未通过：{details}")

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

    def _fail(self, goal_handle, result, message: str, level: str = "warn"):
        result.success = False
        result.phase = "failed"
        result.is_return = bool(goal_handle.request.is_return)
        result.message = f"{message}；{self._progress()}"
        getattr(self.get_logger(), level)(result.message)
        goal_handle.abort()
        return result

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

        timeout_s = self._timeout_for(goal_handle, poi)
        self.get_logger().info(
            f"收到 goal：poi={poi}，返回={is_return}，超时={timeout_s:.1f}s"
            f"（dry_run={self._dry_run}）"
        )

        # 起点与过点点位走点对点，巡检点沿巡检轨道，返程沿返程轨道
        if is_return:
            mode = "track"
        elif poi == self._start_poi or poi in self._transit_pois:
            mode = "point"
        else:
            mode = "track"

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
        """实机运动：下发底盘动作 → 轮询到终态 → 到点转向 → 返回结果。"""
        result = NavigateTo.Result()
        result.is_return = is_return
        self._check_localization(strict=True)
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
                rv = self._chassis.move_to_locations_poi(
                    self._return_track_id, poi, uptrack_enabled=True
                )
            elif mode == "point":
                motion_mode = "point"
                active_track = self._track_name
                rv = self._chassis.move_to_poi(poi)
            else:
                # 固件会忽略 target_position.yaw，轨道段只能先沿轨道到点再原地转向
                motion_mode = "track"
                active_track = self._track_name
                rv = self._chassis.move_to_locations_poi(
                    self._track_id, poi, uptrack_enabled=True
                )
            action_id = rv.get("action_id") if isinstance(rv, dict) else None
            if action_id is None:
                raise RuntimeError(f"底盘未返回 action_id：{rv!r}")
            self.get_logger().info(
                f"实机运动已下发：poi={poi}，方式={motion_mode}，action_id={action_id}"
            )
            if self._run_chassis_action(
                goal_handle, action_id, poi, motion_mode, deadline, timeout_s
            ) == "cancelled":
                return self._cancelled(goal_handle, result, poi)

            # 轨道到达朝向与设定角度不同，到点后原地转向并校验实际朝向
            if motion_mode == "track":
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

        result.success = True
        result.phase = "arrived"
        action = "返回起点" if is_return else "前往"
        track_desc = active_track if motion_mode == "track" else "点对点"
        result.message = (
            f"{action} {poi} 完成；方式={motion_mode}，轨道={track_desc}；"
            f"{self._progress()}"
        )
        self._publish_feedback(goal_handle, "arrived", motion_mode, 0.0)
        self.get_logger().info(result.message)
        goal_handle.succeed()
        return result

    def _run_chassis_action(self, goal_handle, action_id, poi, mode, deadline, timeout_s):
        """轮询底盘动作到终态。

        返回 success / cancelled；失败或超时抛异常，由 _execute 统一 abort。
        """
        deadline = min(deadline, time.monotonic() + timeout_s)
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
                raise RuntimeError(f"{poi} 实机运动执行失败：status={st!r}")

            pose = self._read_pose()
            distance_m = self._distance_to(poi, pose)
            self._publish_feedback(goal_handle, "navigating", mode, distance_m)
            now = time.monotonic()
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
            if self._run_chassis_action(
                goal_handle, action_id, poi, "track", deadline, timeout_s
            ) == "cancelled":
                return "cancelled"
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
