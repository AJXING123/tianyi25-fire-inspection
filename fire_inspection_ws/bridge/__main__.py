"""``python -m bridge``：仅机器人 Jazzy 环境可用（依赖 rclpy）。"""

from __future__ import annotations

from .ros_node import main

if __name__ == "__main__":
    raise SystemExit(main())
