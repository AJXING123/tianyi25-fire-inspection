"""头部关节适配层：把 /head/cmd 协议封装成「平滑移动到目标姿态」。

从 tianyi25-fire-inspection/head_control/head_client.py 抽取：
  - 去掉 Node 继承与命令行入口，改为由服务端持有的适配类
  - 不再自行 spin（由服务端的 executor 负责），避免与主线程冲突

协议：/head/cmd  ros2_bridge_msgs/msg/HeadCtrl
  mode=0（位置模式）、label=0、reserved=165
  ctrl=[MotorCtrl(name=关节ID, pos=弧度, spd=速度, cur=电流)]

实测映射（2026-10-24 逐关节验证）：
  1 = roll 侧倾（正 = 向右歪，巡检固定 0）
  2 = pitch 俯仰（正 = 低头，抬头必须发负值）
  3 = yaw 左右转（正 = 向左）
限位：±26° / ±25° / ±90°
"""

from __future__ import annotations

import math
import threading
import time

from ros2_bridge_msgs.msg import HeadCtrl, MotorCtrl, RobotState
from std_msgs.msg import Header

HEAD_JOINTS = [1, 2, 3]
LIMIT_DEG = {1: 26.0, 2: 25.0, 3: 90.0}
LIMIT_RAD = {k: math.radians(v) for k, v in LIMIT_DEG.items()}
RESERVED = 165


def clamp(joint, pos):
    limit = LIMIT_RAD[joint]
    return max(-limit, min(limit, pos))


def smoothstep(t):
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)


class HeadAdapter:
    def __init__(self, node, cmd_topic="/head/cmd", state_topic="/robot_state",
                 rate=100.0, dry_run=False):
        self.node = node
        self.rate = float(rate)
        self.dry_run = bool(dry_run)
        self.pub = node.create_publisher(HeadCtrl, cmd_topic, 10)
        node.create_subscription(RobotState, state_topic, self._on_state, 10)
        self.pose = [0.0, 0.0, 0.0]
        self._pose_ready = threading.Event()
        # 置 True 后 move_to 立即中断，用于手柄急停
        self.stop_requested = False
        self._log = node.get_logger()

    @property
    def pose_ready(self):
        return self._pose_ready.is_set()

    def _on_state(self, msg):
        try:
            st = {int(s.name): float(s.pos) for s in msg.head.status}
        except AttributeError:
            return
        if all(j in st for j in HEAD_JOINTS):
            self.pose = [st[j] for j in HEAD_JOINTS]
            self._pose_ready.set()

    def wait_pose(self, timeout=3.0):
        return self._pose_ready.wait(timeout)

    def build(self, positions, speed, current):
        msg = HeadCtrl()
        msg.header = Header()
        msg.header.frame_id = "head"
        msg.mode = 0
        msg.label = 0
        msg.reserved = RESERVED
        ctrl = []
        for joint, pos in zip(HEAD_JOINTS, positions):
            m = MotorCtrl()
            m.name = joint
            m.pos = float(clamp(joint, pos))
            m.spd = float(speed)
            m.cur = float(current)
            ctrl.append(m)
        msg.ctrl = ctrl
        return msg

    def send(self, positions, speed, current):
        if self.dry_run:
            self._log.info(
                f"[dry_run] /head/cmd -> {[round(clamp(j, p), 3) for j, p in zip(HEAD_JOINTS, positions)]}")
            return
        self.pub.publish(self.build(positions, speed, current))

    def move_to(self, target, duration=1.5, hold=0.5, speed=0.15, current=2.0):
        """从当前姿态平滑过渡到目标，再保持 hold 秒；stop_requested 可随时中断。"""
        target = [clamp(j, v) for j, v in zip(HEAD_JOINTS, target)]
        start = list(self.pose) if self.pose_ready else [0.0] * 3
        period = 1.0 / self.rate

        t0 = time.time()
        while not self.stop_requested:
            t = (time.time() - t0) / duration if duration > 0 else 1.0
            s = smoothstep(t)
            self.send([a + (b - a) * s for a, b in zip(start, target)],
                      speed, current)
            if t >= 1.0:
                break
            time.sleep(period)

        end = time.time() + hold
        while not self.stop_requested and time.time() < end:
            self.send(target, speed, current)
            time.sleep(period)
        return target
