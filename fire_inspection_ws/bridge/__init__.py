"""ROS 2 Jazzy 端独立 Bridge（只在机器人环境运行，不依赖网页后端依赖）。

模块划分：
- round_tracker / outbox / backend_client：纯 Python，可在开发机测试；
- gateway / http_app：控制与状态 HTTP 协议，可用假网关测试；
- ros_node：唯一 import rclpy 的文件，只在机器人 Jazzy 环境运行。
"""

__version__ = "0.2.0"
