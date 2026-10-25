#!/usr/bin/env python3
"""
手柄按键探针（机器人端运行）

只打印"发生变化"的通道，一行一个，不会被整串字典淹没：
  轴 5 : 0.00 -> 1.00
  按钮 4 : 0 -> 1

用法（机器人终端）：
  python3 joy_probe.py --topic /sbus_data
"""
import argparse

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

# 小于这个变化量视为噪声，不打印
AXIS_DELTA = 0.3


class JoyProbe(Node):
    def __init__(self, topic):
        super().__init__("joy_probe")
        self.prev_axes = None
        self.prev_buttons = None
        self.create_subscription(Joy, topic, self._on_joy, 10)
        print(f"订阅 {topic}，请拨动手柄...（Ctrl+C 退出）", flush=True)

    def _on_joy(self, msg):
        axes = list(msg.axes)
        buttons = list(msg.buttons)

        if self.prev_axes is None:
            print(f"[初始] 轴共 {len(axes)} 路: "
                  f"{[round(a, 2) for a in axes]}", flush=True)
            print(f"[初始] 按钮共 {len(buttons)} 个", flush=True)
            self.prev_axes = axes
            self.prev_buttons = buttons
            return

        changed = False
        for i, new in enumerate(axes):
            old = self.prev_axes[i] if i < len(self.prev_axes) else 0.0
            if abs(new - old) >= AXIS_DELTA:
                print(f"轴 {i} : {old:+.2f} -> {new:+.2f}", flush=True)
                changed = True
        for i, new in enumerate(buttons):
            old = self.prev_buttons[i] if i < len(self.prev_buttons) else 0
            if new != old:
                print(f"按钮 {i} : {old} -> {new}", flush=True)
                changed = True
        if changed:
            print("-" * 30, flush=True)

        self.prev_axes = axes
        self.prev_buttons = buttons


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--topic", default="/sbus_data", help="Joy 话题名")
    args = p.parse_args()

    rclpy.init()
    node = JoyProbe(args.topic)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
