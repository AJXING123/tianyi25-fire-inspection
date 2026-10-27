#!/usr/bin/env python3
"""orchestrator.launch.py —— 启动消防巡检编排层（单独启动，调试用）。

用法：
    ros2 launch fire_inspection_orchestrator orchestrator.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument("script_file", default_value=""),
        DeclareLaunchArgument("dwell_s", default_value="3.0"),
        DeclareLaunchArgument("return_to_start", default_value="false"),
        DeclareLaunchArgument(
            "dry_run",
            default_value="true",
            description="true=不调用真实 Action，仅验证状态机",
        ),
    ]

    orchestrator = Node(
        package="fire_inspection_orchestrator",
        executable="orchestrator_node",
        name="fire_inspection_orchestrator",
        output="screen",
        parameters=[{
            "script_file": LaunchConfiguration("script_file"),
            "dwell_s": LaunchConfiguration("dwell_s"),
            "return_to_start": LaunchConfiguration("return_to_start"),
            "dry_run": LaunchConfiguration("dry_run"),
        }],
    )

    return LaunchDescription(args + [orchestrator])
