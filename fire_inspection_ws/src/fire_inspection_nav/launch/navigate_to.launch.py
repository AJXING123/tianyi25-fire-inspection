#!/usr/bin/env python3
"""navigate_to.launch.py —— 启动消防巡检导航服务端（fire_inspection_nav）。

点位顺序与超时由 config/pois.yaml 提供（唯一配置源），launch 只负责连通参数。

用法：
    ros2 launch fire_inspection_nav navigate_to.launch.py
    ros2 launch fire_inspection_nav navigate_to.launch.py config_file:=/path/to/pois.yaml

dry_run:=true 只做只读校验；dry_run:=false 走实机轨道运动（需已重定位）。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument("config_file", default_value=""),
        DeclareLaunchArgument("host", default_value="192.168.11.10"),
        DeclareLaunchArgument("rest_port", default_value="9090"),
        DeclareLaunchArgument(
            "dry_run",
            default_value="true",
            description="true=只读校验；false=实机运动（需已重定位，有人守护）",
        ),
    ]

    navigate_to_server = Node(
        package="fire_inspection_nav",
        executable="navigate_to_server",
        name="fire_navigate_to",
        output="screen",
        parameters=[{
            "config_file": LaunchConfiguration("config_file"),
            "host": LaunchConfiguration("host"),
            "rest_port": LaunchConfiguration("rest_port"),
            "dry_run": LaunchConfiguration("dry_run"),
        }],
    )

    return LaunchDescription(args + [navigate_to_server])
