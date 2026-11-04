#!/usr/bin/env python3
"""
天轶 2.5 头部控制客户端（在机器人端运行）

协议（与 joystick_bridge/joint_control.py 一致）：
  /head/cmd  ros2_bridge_msgs/msg/HeadCtrl
  mode=0（位置模式）, label=0, reserved=165
  ctrl=[MotorCtrl(name=关节ID, pos=弧度, spd=速度, cur=电流)]

关节编号与限位（config_ty2.5.yaml，配置单位是度）：
  1: ±26°, 2: ±25°, 3: ±90°

实测映射（2026-10-24，已用 0.25rad 逐关节验证）：
  1 -> roll 侧倾（正角度 = 头向右歪，巡检不用，固定 0）
  2 -> pitch 俯仰（正角度 = 低头，抬头必须发负值）
  3 -> yaw 左右转头（正角度 = 向左转）

用法（机器人终端，先 source /opt/ros/jazzy/setup.bash 和 /home/nvidia/xos/setup.bash）：
  python3 head_client.py --peek                    # 实时打印头部角度
  python3 head_client.py --joint 1 --angle 0.1     # 单关节小角度
  python3 head_client.py --pose 0.2 0.1 0          # 三关节目标
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
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


class HeadClient(Node):
    def __init__(self):
        super().__init__("head_client")
        self.pub = self.create_publisher(HeadCtrl, "/head/cmd", 10)
        self.create_subscription(RobotState, "/robot_state", self._on_state, 10)
        self.pose = [0.0, 0.0, 0.0]
        self.pose_ready = False
        # 置 True 后 move_to 会立刻停止过渡，用于手柄急停
        self.stop_requested = False

    def _on_state(self, msg):
        try:
            st = {int(s.name): float(s.pos) for s in msg.head.status}
        except AttributeError:
            return
        if all(j in st for j in HEAD_JOINTS):
            self.pose = [st[j] for j in HEAD_JOINTS]
            self.pose_ready = True

    def wait_pose(self, timeout=3.0):
        deadline = time.time() + timeout
        while rclpy.ok() and not self.pose_ready and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.pose_ready

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
        self.pub.publish(self.build(positions, speed, current))

    def move_to(self, target, duration=1.5, hold=0.5,
                speed=0.3, current=2.0, rate=100.0):
        """从当前姿态平滑过渡到目标，再保持 hold 秒"""
        target = [clamp(j, v) for j, v in zip(HEAD_JOINTS, target)]
        start = list(self.pose) if self.pose_ready else [0.0] * 3
        period = 1.0 / rate

        t0 = time.time()
        while rclpy.ok() and not self.stop_requested:
            t = (time.time() - t0) / duration if duration > 0 else 1.0
            s = smoothstep(t)
            self.send([a + (b - a) * s for a, b in zip(start, target)],
                      speed, current)
            rclpy.spin_once(self, timeout_sec=0)
            if t >= 1.0:
                break
            time.sleep(period)

        end = time.time() + hold
        while rclpy.ok() and not self.stop_requested and time.time() < end:
            self.send(target, speed, current)
            rclpy.spin_once(self, timeout_sec=0)
            time.sleep(period)
        return target


def main():
    p = argparse.ArgumentParser()
    g = p.add_mutually_exclusive_group()
    g.add_argument("--joint", type=int, choices=HEAD_JOINTS,
                   help="只动一个关节")
    g.add_argument("--pose", type=float, nargs=3, metavar=("J1", "J2", "J3"),
                   help="三个关节的目标弧度")
    g.add_argument("--peek", action="store_true", help="实时打印头部角度")
    p.add_argument("--angle", type=float, default=0.0,
                   help="配合 --joint 使用的目标弧度")
    p.add_argument("--duration", type=float, default=1.5, help="过渡时长（秒）")
    p.add_argument("--hold", type=float, default=0.5, help="到位后保持时长（秒）")
    p.add_argument("--speed", type=float, default=0.3)
    p.add_argument("--current", type=float, default=2.0)
    p.add_argument("--rate", type=float, default=100.0, help="发布频率 Hz")
    p.add_argument("--dry-run", action="store_true", help="只打印消息，不发布")
    args = p.parse_args()

    if args.joint is None and args.pose is None and not args.peek:
        p.error("需要 --joint / --pose / --peek 之一")

    rclpy.init()
    node = HeadClient()
    ok = node.wait_pose(3.0)
    pose = [round(v, 3) for v in node.pose]
    print(f"当前头部角度 {pose}"
          + ("" if ok else "（未收到 /robot_state，按 0 处理）"))

    try:
        if args.peek:
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.2)
                vals = {j: round(v, 3) for j, v in zip(HEAD_JOINTS, node.pose)}
                print(f"\r头部角度 {vals}", end="", flush=True)
        elif args.dry_run:
            if args.joint is not None:
                target = list(node.pose)
                target[HEAD_JOINTS.index(args.joint)] = args.angle
            else:
                target = list(args.pose)
            print(node.build(target, args.speed, args.current))
        else:
            if args.joint is not None:
                target = list(node.pose)
                target[HEAD_JOINTS.index(args.joint)] = args.angle
            else:
                target = list(args.pose)
            print(f"目标 {[round(v, 3) for v in target]}")
            node.move_to(target, args.duration, args.hold,
                         args.speed, args.current, args.rate)
            print("结束，已停止发布 /head/cmd")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
