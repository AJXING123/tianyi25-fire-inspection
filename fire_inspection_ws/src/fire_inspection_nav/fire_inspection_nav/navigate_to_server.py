"""消防巡检导航服务端。

当前实现范围：**dry_run 只读链路**。
- 启动时通过底盘 REST 读取 POI 表、轨道信息、定位状态，做前置校验；
- 收到 NavigateTo goal 时重新读取定位状态，然后模拟到点并返回结果；
- **dry_run 不下发任何运动指令，也不写限速参数**；
- 实机运动（轨道导航 + 转向 yaw）尚未实现，goal 会返回失败并说明原因。

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

        self._dry_run = bool(self.get_parameter("dry_run").value)
        self._dry_run_delay_s = float(self.get_parameter("dry_run_delay_s").value)
        self._preflight_retry_interval_s = float(
            self.get_parameter("preflight_retry_interval_s").value
        )

        config = self._load_config()
        self._start_poi = str(config.get("start_poi", "start_point"))
        self._poi_order = [str(name) for name in config.get("poi_order", [])]
        timeouts = config.get("timeouts") or {}
        self._default_timeout_s = float(timeouts.get("per_poi_s", 180.0))
        self._start_timeout_s = float(timeouts.get("to_start_s", 180.0))
        self._poi_source = str(config.get("poi_source", "chassis"))
        self._range_limit = config.get("range_limit") or {}
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
        self._preflight()
        self._last_preflight_at = time.monotonic()

        # ReentrantCallbackGroup + 多线程 executor：保证 execute 阻塞时
        # cancel 请求仍能被处理（默认互斥组 + 单线程会让取消完全失效）
        self._action_server = ActionServer(
            self,
            NavigateTo,
            "/inspection/navigate_to",
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
            for name in required:
                if any(self._pois[name].get(k) is None for k in ("x", "y", "yaw")):
                    raise RuntimeError(f"POI {name} 缺少 x/y/yaw")
            self._check_range_limit(required)

            tracks_raw = self._read_tracks()
            tracks = [
                t
                for t in _unwrap_list(tracks_raw, "tracks", "track", "data")
                if isinstance(t, dict)
            ]
            if len(tracks) != 1:
                names = [t.get("track_name") or t.get("track_id") for t in tracks]
                raise RuntimeError(
                    f"当前地图应只有一条轨道，实际 {len(tracks)} 条: {names}"
                    f"（接口返回 {_describe(tracks_raw)}）"
                )
            if tracks[0].get("track_id") is None:
                raise RuntimeError("轨道缺少 track_id")
            self._track_id = tracks[0]["track_id"]
            self._track_name = tracks[0].get("track_name") or str(self._track_id)
            self._check_poi_on_track(required, tracks[0])

            self._check_localization(strict=False)

            self.get_logger().info(
                f"前置校验通过：地图={self._map_name}，{len(self._pois)} 个 POI，"
                f"轨道 {self._track_name} (track_id={self._track_id})"
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

        if not self._dry_run:
            return self._fail(
                goal_handle,
                result,
                "实机运动尚未实现：当前仅支持 dry_run 只读链路，请设置 dry_run:=true",
                "error",
            )

        if goal_handle.is_cancel_requested:
            return self._cancelled(goal_handle, result, poi)

        try:
            self._check_localization(strict=False)
        except Exception as exc:  # noqa: BLE001
            return self._fail(goal_handle, result, f"读取定位状态失败：{exc}", "error")

        # 首次去起点用点对点，其余段（含返回起点）都沿轨道
        mode = "track" if is_return else (
            "point" if (poi == self._start_poi and not self._started) else "track"
        )
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
        result.message = (
            f"[dry_run] {action} {poi} 校验通过，未下发运动指令；"
            f"方式={mode}，轨道={self._track_name}，当前位姿={pose}，"
            f"距目标={distance_m:.2f}m；{self._progress()}"
        )
        self._publish_feedback(goal_handle, "arrived", mode, 0.0)
        self.get_logger().info(result.message)
        goal_handle.succeed()
        return result

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
