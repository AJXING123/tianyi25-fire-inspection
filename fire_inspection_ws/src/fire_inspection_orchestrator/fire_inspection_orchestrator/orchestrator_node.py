"""消防巡检编排层：检测命中 -> 头部锁定 -> 语音播报 -> 恢复扫视。

这是「看到目标就停下来看一眼、播报完继续走」这条链路的唯一指挥者，
模块之间禁止互调：本节点只发 /fire_inspection/head_sweep_cmd、调 Speak Action。

状态机：
    WATCH  扫视中，等待视觉命中
      --连续 confirm_frames 帧同类命中--> LOOK
    LOOK   已发 pause + look_at，头部转向目标
      --Speak 可用--> SPEAK
      --Speak 不可用/被拒绝--> 直接回 WATCH
    SPEAK  等待播报结果
      --完成/失败/超时 speak_timeout_s--> 发 resume -> WATCH

防误报规则：
1. 连续 N 帧（默认 3）同类才触发，中途换类或丢失即清零；
2. 同一类触发后冷却 cooldown_s（默认 15s），冷却期内不再触发；
3. simulated=true 的检测结果一律忽略；
4. 播报超时强制 resume，绝不让头部卡死在锁定状态；
5. 头部报 stopped/stopping（手柄急停）时停止触发。
"""

from __future__ import annotations

import json
import math
import os
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from ros2_bridge_msgs.msg import RobotState
from std_msgs.msg import String

from fire_inspection_interfaces.action import Speak
from fire_inspection_interfaces.msg import PoiResult

STATE_WATCH = "watch"
STATE_LOOK = "look"
STATE_SPEAK = "speak"

ALIASES = {
    "fire": "fire_extinguisher",
    "extinguisher": "fire_extinguisher",
    "smoke": "smoking",
    "sign": "safety_sign",
}


def canonical(name):
    key = str(name).strip().lower()
    return ALIASES.get(key, key)


def load_config():
    share = get_package_share_directory("fire_inspection_orchestrator")
    path = os.path.join(share, "config", "inspection_script.yaml")
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def detection_qos():
    """T3 的话题 QoS：best_effort / depth 1，与 vision_scan_node 发布端一致。"""
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        durability=DurabilityPolicy.VOLATILE,
    )


class OrchestratorNode(Node):
    def __init__(self) -> None:
        super().__init__("fire_inspection_orchestrator")
        cfg = load_config()
        scan = cfg.get("scan") or {}

        self.declare_parameter("dry_run", True)
        self.dry_run = bool(self.get_parameter("dry_run").value)

        self.classes = [canonical(c) for c in scan.get("classes", ["smoking"])]
        self.abnormal = {canonical(c) for c in
                         scan.get("abnormal_classes", ["smoking"])}
        self.confirm_frames = max(1, int(scan.get("confirm_frames", 3)))
        self.cooldown = float(scan.get("cooldown_s", 15.0))
        self.gain = float(scan.get("fov_gain", 0.8))
        self.fov_h = math.radians(float(scan.get("fov_h_deg", 80.0)))
        self.fov_v = math.radians(float(scan.get("fov_v_deg", 60.0)))
        self.speak_timeout = float(scan.get("speak_timeout_s", 15.0))
        self.speak_enabled = bool(scan.get("speak_enabled", True))
        self.speech_tpl = scan.get("speech_abnormal", "警告，检测到{cn}")
        self.speech_normal_tpl = scan.get("speech_normal", "发现{cn}")
        self.cls_cn = scan.get("cls_cn") or {}

        head_cmd_topic = scan.get("head_cmd_topic", "/fire_inspection/head_sweep_cmd")
        head_status_topic = scan.get("head_status_topic", "/fire_inspection/head_status")
        robot_state_topic = scan.get("robot_state_topic", "/robot_state")
        det_topic = scan.get("topic", "/fire_inspection/detection")
        speak_action = scan.get("speak_action", "/fire_inspection/speak")

        self._cmd_pub = self.create_publisher(String, head_cmd_topic, 10)
        # T3 按契约 §8 是 best_effort / depth 1，订阅端必须一致，否则收不到
        self.create_subscription(PoiResult, det_topic, self._on_detection,
                                 detection_qos())
        self.create_subscription(String, head_status_topic, self._on_head_status, 10)
        self.create_subscription(RobotState, robot_state_topic, self._on_state, 10)
        self._speak_client = ActionClient(self, Speak, speak_action)

        self._state = STATE_WATCH
        self._armed = True
        self._head_pose = [0.0, 0.0, 0.0]
        self._pose_ready = False
        self._streak = 0
        self._streak_cls = ""
        self._cooldown = {}
        self._goal_handle = None
        self._speak_timer = None

        self.get_logger().info(
            f"编排层就绪 检测={det_topic} 头部={head_cmd_topic} 播报={speak_action} "
            f"classes={self.classes} confirm={self.confirm_frames} "
            f"cooldown={self.cooldown:.0f}s dry_run={self.dry_run}")

    # ---------- 输入 ----------
    def _on_state(self, msg):
        try:
            st = {int(s.name): float(s.pos) for s in msg.head.status}
        except AttributeError:
            return
        if all(j in st for j in (1, 2, 3)):
            self._head_pose = [st[1], st[2], st[3]]
            self._pose_ready = True

    def _on_head_status(self, msg):
        status = msg.data.strip().lower()
        if status in ("stopped", "stopping"):
            self._armed = False
        elif status.startswith("sweeping") or status.startswith("step:"):
            self._armed = True

    def _on_detection(self, msg):
        if self._state != STATE_WATCH or not self._armed:
            return
        if msg.simulated:
            return
        try:
            dets = json.loads(msg.detections_json) if msg.detections_json else []
        except json.JSONDecodeError:
            return

        now = time.monotonic()
        best = None
        for d in dets:
            cls = canonical(d.get("cls", ""))
            if cls not in self.classes:
                continue
            if now - self._cooldown.get(cls, 0.0) < self.cooldown:
                continue
            if best is None or float(d.get("conf", 0.0)) > float(best.get("conf", 0.0)):
                best = d

        if best is None:
            self._streak = 0
            self._streak_cls = ""
            return

        cls = canonical(best["cls"])
        if cls == self._streak_cls:
            self._streak += 1
        else:
            self._streak_cls = cls
            self._streak = 1

        self.get_logger().info(
            f"检测到 {cls} conf={float(best.get('conf', 0.0)):.2f} "
            f"连续 {self._streak}/{self.confirm_frames}")

        if self._streak >= self.confirm_frames:
            self._trigger(cls, best)

    # ---------- 主流程 ----------
    def _trigger(self, cls, det):
        self._state = STATE_LOOK
        self._streak = 0
        self._streak_cls = ""
        self._cooldown[cls] = time.monotonic()

        yaw, pitch = self._aim(float(det.get("cx", 0.5)),
                               float(det.get("cy", 0.5)))
        self._send_cmd("pause")
        self._send_cmd(f"look_at:{yaw:.3f},{pitch:.3f}")
        self.get_logger().warn(
            f"命中 {cls} -> 头部锁定 yaw={yaw:.3f} pitch={pitch:.3f} rad")

        text = self._render(cls)
        self._speak(text)

    def _aim(self, cx, cy):
        """像素中心 -> 绝对关节角：向右的目标要让 yaw 变小（左为正）。"""
        yaw0 = self._head_pose[2]
        pitch0 = self._head_pose[1]
        yaw = yaw0 + self.gain * (0.5 - cx) * self.fov_h
        pitch = pitch0 + self.gain * (cy - 0.5) * self.fov_v
        return yaw, pitch

    def _render(self, cls):
        cn = self.cls_cn.get(cls, cls)
        tpl = self.speech_tpl if cls in self.abnormal else self.speech_normal_tpl
        return tpl.format(cn=cn, cls=cls)

    def _send_cmd(self, text):
        if self.dry_run:
            self.get_logger().info(f"[dry_run] head_cmd <- {text}")
            return
        msg = String()
        msg.data = text
        self._cmd_pub.publish(msg)
        self.get_logger().info(f"head_cmd -> {text}")

    # ---------- 播报 ----------
    def _speak(self, text):
        if not self.speak_enabled:
            self.get_logger().info(f"播报已关闭，文案: {text}")
            self._finish_cycle()
            return
        if not self._speak_client.server_is_ready():
            self.get_logger().warn("Speak Action 不在线，跳过播报（头部仍会恢复扫视）")
            self._finish_cycle()
            return
        if self.dry_run:
            self.get_logger().info(f"[dry_run] 播报文案: {text}")
            self._finish_cycle()
            return

        self._state = STATE_SPEAK
        goal = Speak.Goal()
        goal.text = text
        goal.interrupt = True
        self._speak_timer = self.create_timer(self.speak_timeout,
                                              self._on_speak_timeout)
        self._speak_client.send_goal_async(goal).add_done_callback(
            self._on_goal_response)

    def _on_goal_response(self, future):
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().warn("播报目标被拒绝")
            self._finish_cycle()
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_speak_result)

    def _on_speak_result(self, future):
        try:
            result = future.result().result
            self.get_logger().info(
                f"播报结束 success={result.success} message={result.message}")
        except Exception as exc:  # 取消/异常都按结束处理
            self.get_logger().warn(f"播报结果异常: {exc}")
        self._finish_cycle()

    def _on_speak_timeout(self):
        self.get_logger().warn(f"播报超时（>{self.speak_timeout:.0f}s），强制恢复扫视")
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None
        self._finish_cycle()

    def _finish_cycle(self):
        if self._speak_timer is not None:
            self._speak_timer.cancel()
            self.destroy_timer(self._speak_timer)
            self._speak_timer = None
        # 先恢复扫视，再切状态，避免异常打断导致头部一直锁着
        self._send_cmd("resume")
        self._goal_handle = None
        self._state = STATE_WATCH
        self.get_logger().info("恢复扫视，继续等待目标")


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = OrchestratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
