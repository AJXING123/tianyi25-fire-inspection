#!/usr/bin/env python3
"""
停靠点头部扫视（机器人端运行）

机器人到达巡检点后执行分步扫视，配合视觉检测目标，可无限循环：
  低头左 -> 低头右 -> 平视左 -> 平视右 -> 微抬头左 -> 微抬头右 -> 循环
每步之间平滑过渡，并在每步停留一段时间给视觉脚本推理。

用法（机器人终端）：
  python3 look_around.py --dry-run     # 只打印动作序列
  python3 look_around.py               # 扫视一轮
  python3 look_around.py --loop        # 无限循环（巡检常驻）
  python3 look_around.py --repeat 3    # 连续扫 3 轮

停止：手柄 G + E 拨杆同时拨到触发位（轴5 为正 且 轴4 为负）即立刻停住。
轴号可用 --stop-axes 改，格式 轴号:符号，例如 --stop-axes 5:+ 4:-
"""
import argparse

import rclpy
from sensor_msgs.msg import Joy

from head_client import HeadClient

# 关节实测映射（2026-10-24 逐关节验证）：
#   1 = roll 侧倾（正 = 向右歪，巡检保持 0）
#   2 = pitch 俯仰（正 = 低头，抬头要负值）
#   3 = yaw 左右转（正 = 向左转）
# 每步: (目标角度 [J1 侧倾, J2 俯仰, J3 左右]（弧度）, 说明)
PITCH_UP = -0.15    # 微抬头约 8.6°（限位 ±0.436，留余量）
PITCH_DOWN = 0.22   # 低头约 12.6°，灭火器/地脚指示灯在下方
YAW_SIDE = 0.35     # 左右各约 20°（限位 ±1.571，很宽裕）

STEPS = [
    ([0.0, PITCH_DOWN, YAW_SIDE], "低头 + 向左（灭火器/指示灯）"),
    ([0.0, PITCH_DOWN, -YAW_SIDE], "低头 + 向右（灭火器/指示灯）"),
    ([0.0, 0.0, YAW_SIDE], "平视 + 向左（吸烟）"),
    ([0.0, 0.0, -YAW_SIDE], "平视 + 向右（吸烟）"),
    ([0.0, PITCH_UP, YAW_SIDE], "微抬头 + 向左"),
    ([0.0, PITCH_UP, -YAW_SIDE], "微抬头 + 向右"),
]


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--repeat", type=int, default=1, help="扫视轮数")
    p.add_argument("--loop", action="store_true", help="无限循环扫视（常驻巡检用）")
    p.add_argument("--duration", type=float, default=2.0, help="单步过渡时长（秒）")
    p.add_argument("--dwell", type=float, default=3.0, help="每步停留时长（秒）")
    p.add_argument("--speed", type=float, default=0.15)
    p.add_argument("--current", type=float, default=2.0)
    p.add_argument("--dry-run", action="store_true", help="只打印动作序列")
    p.add_argument("--joy-topic", default="/sbus_data", help="手柄 Joy 话题名")
    p.add_argument("--stop-axes", nargs="+", default=["5:+", "4:-"],
                   help="停止组合键，格式 轴号:符号，需同时满足")
    p.add_argument("--axis-threshold", type=float, default=0.5,
                   help="判定拨杆已拨到的阈值")
    args = p.parse_args()

    stop_axes = []
    for item in args.stop_axes:
        idx_s, sign_s = item.split(":")
        stop_axes.append((int(idx_s), 1.0 if sign_s.startswith("+") else -1.0))

    for i, (target, name) in enumerate(STEPS, 1):
        print(f"{i}. {name} -> {target}")

    if args.dry_run:
        rounds = "无限循环" if args.loop else f"{args.repeat} 轮"
        print(f"共 {len(STEPS)} 步 × {rounds}，"
              f"每步 {args.duration:.1f}s 过渡 + {args.dwell:.1f}s 停留，"
              f"单轮约 {len(STEPS) * (args.duration + args.dwell):.0f}s")
        return

    rclpy.init()
    node = HeadClient()

    joy_state = {"prev": None}

    def _on_joy(msg):
        # 组合键：配置的所有轴必须同时越过阈值且方向正确
        satisfied = True
        for idx, sign in stop_axes:
            if idx >= len(msg.axes):
                satisfied = False
                break
            if msg.axes[idx] * sign < args.axis_threshold:
                satisfied = False
                break

        prev = joy_state["prev"]
        joy_state["prev"] = satisfied
        # 首帧若已在触发位（开机时拨杆就拨着），视为长按，不触发，等松手再按
        if prev is None or prev:
            return
        if satisfied and not node.stop_requested:
            print("\n收到手柄停止组合键（G+E），立即停止扫视", flush=True)
            node.stop_requested = True

    node.create_subscription(Joy, args.joy_topic, _on_joy, 10)
    print(f"停止组合键：{args.joy_topic} " +
          " 且 ".join(f"轴{i}{'+' if s > 0 else '-'}" for i, s in stop_axes),
          flush=True)

    ok = node.wait_pose(3.0)
    print(f"起始姿态 {[round(v, 3) for v in node.pose]}"
          + ("" if ok else "（未收到 /robot_state，按 0 处理）"))

    rounds = 0
    try:
        while not node.stop_requested:
            rounds += 1
            if not args.loop and rounds > args.repeat:
                break
            print(f"--- 第 {rounds} 轮 ---", flush=True)
            for target, name in STEPS:
                if node.stop_requested:
                    break
                print(f"{name} -> {[round(v, 3) for v in target]}", flush=True)
                node.move_to(target, args.duration, args.dwell,
                             args.speed, args.current)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    print("扫视结束")


if __name__ == "__main__":
    main()
