#!/usr/bin/env python3
"""head_sweep.launch.py —— 启动消防巡检头部扫视服务端（fire_inspection_head）。

用法：
    ros2 launch fire_inspection_head head_sweep.launch.py                  # 等外部 start 指令
    ros2 launch fire_inspection_head head_sweep.launch.py auto_start:=true # 启动即扫视
    ros2 launch fire_inspection_head head_sweep.launch.py dry_run:=true    # 只打印不下发

开始/停止（另开终端）：
    ros2 topic pub --once /fire_inspection/head_sweep_cmd std_msgs/String "{data: start}"
    ros2 topic pub --once /fire_inspection/head_sweep_cmd std_msgs/String "{data: stop}"
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument("dry_run", default_value="false",
                              description="true=只打印 /head/cmd，不真正下发"),
        DeclareLaunchArgument("auto_start", default_value="false",
                              description="true=启动后自动开始扫视"),
    ]

    head_sweep_server = Node(
        package="fire_inspection_head",
        executable="head_sweep_server",
        name="fire_head_sweep",
        output="screen",
        parameters=[{
            "dry_run": LaunchConfiguration("dry_run"),
            "auto_start": LaunchConfiguration("auto_start"),
        }],
    )

    return LaunchDescription(args + [head_sweep_server])
