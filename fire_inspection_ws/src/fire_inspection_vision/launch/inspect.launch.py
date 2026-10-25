#!/usr/bin/env python3
"""inspect.launch.py —— 启动消防巡检视觉服务端（fire_inspection_vision）。

配置来自 config/vision.yaml（模型路径、阈值、相机话题），launch 只负责覆盖运行模式。

用法：
    ros2 launch fire_inspection_vision inspect.launch.py                 # dry_run=true，仅验证链路
    ros2 launch fire_inspection_vision inspect.launch.py dry_run:=false  # 真推理
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument(
            "dry_run",
            default_value="true",
            description="true=不加载模型、不推理，直接返回模拟结果",
        ),
    ]

    inspect_action_server = Node(
        package="fire_inspection_vision",
        executable="inspect_action_server",
        name="fire_inspect_poi",
        output="screen",
        parameters=[{"dry_run": LaunchConfiguration("dry_run")}],
    )

    return LaunchDescription(args + [inspect_action_server])
