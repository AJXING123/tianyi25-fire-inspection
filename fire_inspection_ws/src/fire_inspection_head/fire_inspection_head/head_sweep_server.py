"""消防巡检头部扫视服务端（机器人端）。

控制接口（std_msgs/String）：
  /fire_inspection/head_sweep_cmd
    start                     开始循环扫视（从第 1 步）
    pause                     暂停扫视，头部保持当前姿态（不回中）
    resume                    从暂停的那一步继续扫视
    look_at:<yaw>,<pitch>     头部转向指定绝对姿态（弧度），到位后持续保持
    center                    头部回中位并保持
    stop                      终止扫视并回中
    status                    上报当前状态
  /fire_inspection/head_status
    idle / sweeping / step:N:名称 / paused / looking / holding / stopping / stopped

任意时刻只有本节点的 motion 线程向 /head/cmd 发指令，避免多线程互抢。
扫视序列、节奏、发布频率、手柄停止键都来自 config/head_sweep.yaml，改参数不必重编译。
手柄急停：/sbus_data 轴5 > +0.5 且 轴4 < -0.5（G 拨右 + E 拨上），上升沿触发；
开机时若拨杆已停在触发位，首帧不算，需要松手再拨一次。
"""

from __future__ import annotations

import os
import threading
import time

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
        self.look_duration = float(cfg.get("look_duration_s", 1.2))
        self.center_duration = float(cfg.get("center_duration_s", 1.5))
        self.rate = float(cfg.get("publish_rate_hz", 100.0))
        self.period = 1.0 / max(1.0, self.rate)
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
            rate=self.rate,
            dry_run=self.dry_run,
        )

        self._status_pub = self.create_publisher(String, status_topic, 10)
        self.create_subscription(String, cmd_topic, self._on_cmd, 10)
        self.create_subscription(Joy, joy_topic, self._on_joy, 10)

        # sweep = 扫视 / look = 保持某个指定姿态
        self._mode = "sweep"
        self._look_target = [0.0, 0.0, 0.0]
        self._step_idx = 0
        self._stop_evt = threading.Event()
        self._pause_evt = threading.Event()
        self._worker = None
        self._lock = threading.Lock()
        self._prev_joy = None
        self._state = "idle"

        self.get_logger().info(
            f"头部扫视服务端就绪 cmd={cmd_topic} status={status_topic} "
            f"steps={len(self.steps)} dry_run={self.dry_run}")

        if bool(self.get_parameter("auto_start").value):
            delay = float(cfg.get("auto_start_delay_s", 30.0))
            self._auto_timer = self.create_timer(delay, self._auto_start_once)
            self.get_logger().info(f"{delay:.0f}s 后自动开始扫视（等待控制栈就绪）")

    # ---------- 外部指令 ----------
    def _on_cmd(self, msg):
        raw = msg.data.strip()
        cmd = raw.lower()
        if cmd in ("start", "begin", "sweep"):
            self.start_sweep()
        elif cmd in ("pause", "hold"):
            self.pause_sweep()
        elif cmd in ("resume", "continue", "go"):
            self.resume_sweep()
        elif cmd == "center":
            self.center_look()
        elif cmd.startswith("look_at"):
            self._parse_look_at(raw)
        elif cmd in ("stop", "cancel"):
            self._request_stop("收到停止指令")
        elif cmd == "status":
            self._publish_state()
        else:
            self.get_logger().warn(
                f"未知指令: {raw!r}（支持 start/pause/resume/"
                f"look_at:yaw,pitch/center/stop/status）")

    def _parse_look_at(self, raw):
        try:
            _, args = raw.split(":", 1)
            yaw_s, pitch_s = args.split(",")
            yaw = float(yaw_s)
            pitch = float(pitch_s)
        except ValueError:
            self.get_logger().warn(
                f"look_at 参数非法: {raw!r}（格式 look_at:yaw,pitch，单位弧度）")
            return
        self.look_at(yaw, pitch)

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
        # 控制栈没就绪就不动头，等下一个周期再来（/robot_state 无数据时起扫会空跑）
        if not self.adapter.pose_ready:
            self.get_logger().warn("等待 /robot_state 就绪，暂不自动开始扫视")
            return
        # 只自动启动一次：手柄按下停止后不会被定时器重新拉起
        self._auto_timer.cancel()
        if not self.is_running:
            self.start_sweep()

    def start_sweep(self):
        if not self.steps:
            self.get_logger().error("head_sweep.yaml 里没有 steps，拒绝启动")
            return
        if self.is_running and self._mode == "sweep" and not self._pause_evt.is_set():
            self.get_logger().info("扫视已在进行中，忽略 start")
            return
        self._step_idx = 0
        self._pause_evt.clear()
        self._ensure_worker("sweep")
        self._publish("sweeping")

    def pause_sweep(self):
        if not self.is_running:
            self.get_logger().info("当前未在扫视，pause 忽略")
            return
        self._pause_evt.set()
        self.adapter.stop_requested = True
        self.get_logger().info(f"暂停扫视，保持当前姿态（第 {self._step_idx + 1} 步）")

    def resume_sweep(self):
        if not self.steps:
            self.get_logger().error("head_sweep.yaml 里没有 steps，拒绝启动")
            return
        self._pause_evt.clear()
        if self.is_running:
            # 线程还活着：切回 sweep，look 的保持循环会在下一拍退出
            self._mode = "sweep"
        else:
            self._ensure_worker("sweep")
        self.get_logger().info(f"恢复扫视，从第 {self._step_idx + 1} 步继续")

    def look_at(self, yaw, pitch):
        self._look_target = [0.0, float(pitch), float(yaw)]
        self._ensure_worker("look")
        # 打断正在进行的扫视步骤：move_to 的过渡/停留循环会立即返回，
        # 下一拍就切到 look。否则要等整步（2s 过渡 + 2s 停留）走完才转向，
        # 编排层 3s 确认窗口内锁头会失效，目标已经丢了。
        self.adapter.stop_requested = True
        self.get_logger().info(f"看向 yaw={yaw:.3f} pitch={pitch:.3f} rad（关节限位内自动裁剪）")

    def center_look(self):
        self.look_at(0.0, 0.0)
        self.get_logger().info("头部回中位并保持")

    def _ensure_worker(self, mode):
        with self._lock:
            self._mode = mode
            if self.is_running:
                return
            if not self.steps:
                self.get_logger().error("head_sweep.yaml 里没有 steps，拒绝启动")
                return
            if not self.adapter.wait_pose(1.0):
                self.get_logger().warn("未收到 /robot_state，按 0 姿态起步（确认 robot_control 已启动）")
            self._stop_evt.clear()
            self._pause_evt.clear()
            self.adapter.stop_requested = False
            self._worker = threading.Thread(target=self._motion_loop,
                                            name="head_motion", daemon=True)
            self._worker.start()

    def _request_stop(self, reason):
        if self._stop_evt.is_set():
            return
        self._stop_evt.set()
        self.adapter.stop_requested = True
        self.get_logger().warn(f"停止扫视：{reason}")
        self._publish("stopping")

    # ---------- 运动线程（唯一向 /head/cmd 发指令的线程） ----------
    def _motion_loop(self):
        try:
            while not self._stop_evt.is_set():
                mode = self._mode
                if mode == "look":
                    self._run_look()
                elif mode == "sweep":
                    if self._pause_evt.is_set():
                        self._hold_current("paused")
                    else:
                        self._run_step()
                else:
                    time.sleep(self.period)
        finally:
            self._finish()

    def _run_step(self):
        step = self.steps[self._step_idx]
        pose = [float(v) for v in step["pose"]]
        name = step.get("name", "")
        want = f"step:{self._step_idx + 1}:{name}"
        if self._state != want:
            self._publish(want)
        self.adapter.stop_requested = False
        # 停留期间 move_to 持续下发，防止第三方发布者抢走 /head/cmd
        self.adapter.move_to(pose, self.duration, self.dwell,
                             self.speed, self.current)
        # 被 look_at 打断时不推进步骤，恢复扫视后从这一步继续
        if (self._stop_evt.is_set() or self._pause_evt.is_set()
                or self._mode != "sweep"):
            return
        self._step_idx = (self._step_idx + 1) % len(self.steps)

    def _run_look(self):
        if self._state != "looking":
            self._publish("looking")
        # 目标可能被连续的 look_at 更新，且新的 look_at 会把 stop_requested
        # 置 True 打断上一轮 move_to；每轮循环重读目标，才能跟上最新指向。
        while not self._stop_evt.is_set() and self._mode == "look":
            target = list(self._look_target)
            self.adapter.stop_requested = False
            self.adapter.move_to(target, self.look_duration, 0.0,
                                 self.speed, self.current)
            if self._stop_evt.is_set() or self._mode != "look":
                return
            if self.adapter.stop_requested:
                continue     # 被新的 look_at 打断，按新目标重来
            if self._state != "holding":
                self._publish("holding")
            # 保持到 resume / center / stop：既看得住目标，也压住第三方发布者
            while (not self._stop_evt.is_set() and self._mode == "look"
                   and not self.adapter.stop_requested):
                self.adapter.send(target, self.speed, self.current)
                time.sleep(self.period)

    def _hold_current(self, state):
        if self._state != state:
            self._publish(state)
        while (not self._stop_evt.is_set() and self._mode == "sweep"
               and self._pause_evt.is_set()):
            self.adapter.send(self.adapter.last_cmd, self.speed, self.current)
            time.sleep(self.period)

    def _finish(self):
        self._pause_evt.clear()
        self.adapter.stop_requested = False
        # 回中后再收尾，避免头歪着停在半路
        self.adapter.move_to([0.0, 0.0, 0.0], self.center_duration, 0.2,
                             self.speed, self.current)
        self._publish("stopped")

    # ---------- 状态上报 ----------
    def _publish_state(self):
        self._publish(self._state)

    def _publish(self, state):
        self._state = state
        if not rclpy.ok():
            return
        msg = String()
        msg.data = state
        self._status_pub.publish(msg)
        self.get_logger().info(f"head_status = {state}")

    def shutdown_sweep(self):
        self._request_stop("服务退出")
        if self._worker is not None:
            self._worker.join(timeout=8.0)


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
