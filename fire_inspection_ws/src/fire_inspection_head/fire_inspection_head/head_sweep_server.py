"""消防巡检头部扫视服务端（机器人端）。

临时控制接口（未改动 fire_inspection_interfaces，后续可升级为 Action）：
  /fire_inspection/head_sweep_cmd   std_msgs/String   start | stop
  /fire_inspection/head_status      std_msgs/String   idle / sweeping / step:N:名称 / stopping / stopped

扫视序列、节奏、发布频率、手柄停止键都来自 config/head_sweep.yaml，改参数不必重编译。
手柄急停：/sbus_data 轴5 > +0.5 且 轴4 < -0.5（G 拨右 + E 拨上），上升沿触发；
开机时若拨杆已停在触发位，首帧不算，需要松手再拨一次。
"""

from __future__ import annotations

import os
import threading

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String

from fire_inspection_head.head_adapter import HeadAdapter


def load_config():
    share = get_package_share_directory("fire_inspection_head")
    path = os.path.join(share, "config", "head_sweep.yaml")
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


class HeadSweepServer(Node):
    def __init__(self):
        super().__init__("fire_head_sweep")
        cfg = load_config()

        self.dry_run = bool(cfg.get("dry_run", False))
        self.declare_parameter("dry_run", self.dry_run)
        self.declare_parameter("auto_start", False)
        self.dry_run = bool(self.get_parameter("dry_run").value)

        self.speed = float(cfg.get("speed", 0.15))
        self.current = float(cfg.get("current", 2.0))
        self.duration = float(cfg.get("step_duration_s", 2.0))
        self.dwell = float(cfg.get("step_dwell_s", 3.0))
        self.steps = list(cfg.get("steps") or [])
        self.stop_axes = [(int(i), 1.0 if float(s) > 0 else -1.0)
                          for i, s in cfg.get("stop_axes", [[5, 1], [4, -1]])]
        self.axis_threshold = float(cfg.get("axis_threshold", 0.5))

        cmd_topic = cfg.get("cmd_topic", "/fire_inspection/head_sweep_cmd")
        status_topic = cfg.get("status_topic", "/fire_inspection/head_status")
        joy_topic = cfg.get("joy_topic", "/sbus_data")

        self.adapter = HeadAdapter(
            self,
            cmd_topic=cfg.get("head_cmd_topic", "/head/cmd"),
            state_topic=cfg.get("robot_state_topic", "/robot_state"),
            rate=float(cfg.get("publish_rate_hz", 100.0)),
            dry_run=self.dry_run,
        )

        self._status_pub = self.create_publisher(String, status_topic, 10)
        self.create_subscription(String, cmd_topic, self._on_cmd, 10)
        self.create_subscription(Joy, joy_topic, self._on_joy, 10)

        self._stop_evt = threading.Event()
        self._worker = None
        self._lock = threading.Lock()
        self._prev_joy = None

        self.get_logger().info(
            f"头部扫视服务端就绪 cmd={cmd_topic} status={status_topic} "
            f"steps={len(self.steps)} dry_run={self.dry_run}")

        if bool(self.get_parameter("auto_start").value):
            delay = float(cfg.get("auto_start_delay_s", 30.0))
            self._auto_timer = self.create_timer(delay, self._auto_start_once)
            self.get_logger().info(f"{delay:.0f}s 后自动开始扫视（等待控制栈就绪）")

    # ---------- 外部指令 ----------
    def _on_cmd(self, msg):
        cmd = msg.data.strip().lower()
        if cmd in ("start", "begin", "sweep"):
            self.start_sweep()
        elif cmd in ("stop", "cancel"):
            self._request_stop("收到停止指令")
        elif cmd in ("status",):
            self._publish("sweeping" if self.is_running else "idle")
        else:
            self.get_logger().warn(f"未知指令: {msg.data!r}（支持 start / stop / status）")

    def _on_joy(self, msg):
        satisfied = True
        for idx, sign in self.stop_axes:
            if idx >= len(msg.axes) or msg.axes[idx] * sign < self.axis_threshold:
                satisfied = False
                break
        prev = self._prev_joy
        self._prev_joy = satisfied
        # 首帧已在触发位视为长按，不触发；需要松手再拨
        if prev is None or prev:
            return
        if satisfied:
            self._request_stop("手柄组合键 G+E")

    # ---------- 扫视生命周期 ----------
    @property
    def is_running(self):
        return self._worker is not None and self._worker.is_alive()

    def _auto_start_once(self):
        # 只自动启动一次：手柄按下停止后不会被定时器重新拉起
        self._auto_timer.cancel()
        if not self.is_running:
            self.start_sweep()

    def start_sweep(self):
        with self._lock:
            if self.is_running:
                self.get_logger().info("扫视已在进行中，忽略 start")
                return
            if not self.steps:
                self.get_logger().error("head_sweep.yaml 里没有 steps，拒绝启动")
                return
            if not self.adapter.wait_pose(3.0):
                self.get_logger().warn("未收到 /robot_state，按 0 姿态起步（确认 robot_control 已启动）")
            self._stop_evt.clear()
            self.adapter.stop_requested = False
            self._worker = threading.Thread(target=self._sweep_loop,
                                            name="head_sweep", daemon=True)
            self._worker.start()

    def _request_stop(self, reason):
        if self._stop_evt.is_set():
            return
        self._stop_evt.set()
        self.adapter.stop_requested = True
        self.get_logger().warn(f"停止扫视：{reason}")
        self._publish("stopping")

    def _sweep_loop(self):
        self._publish("sweeping")
        while not self._stop_evt.is_set():
            for idx, step in enumerate(self.steps, 1):
                if self._stop_evt.is_set():
                    break
                pose = [float(v) for v in step["pose"]]
                name = step.get("name", "")
                self._publish(f"step:{idx}:{name}")
                self.adapter.move_to(pose, self.duration, 0.0,
                                     self.speed, self.current)
                if self._stop_evt.wait(self.dwell):
                    break
        # 回中后再收尾，避免头歪着停在半路
        self.adapter.stop_requested = False
        self.adapter.move_to([0.0, 0.0, 0.0], 1.5, 0.2,
                             self.speed, self.current)
        self._publish("stopped" if self._stop_evt.is_set() else "idle")

    def _publish(self, state):
        if not rclpy.ok():
            return
        msg = String()
        msg.data = state
        self._status_pub.publish(msg)
        self.get_logger().info(f"head_status = {state}")

    def shutdown_sweep(self):
        self._request_stop("服务退出")
        if self._worker is not None:
            self._worker.join(timeout=5.0)


def main(argv=None):
    rclpy.init(args=argv)
    node = HeadSweepServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_sweep()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
