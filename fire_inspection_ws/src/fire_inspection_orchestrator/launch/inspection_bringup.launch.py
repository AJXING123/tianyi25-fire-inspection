#!/usr/bin/env python3
"""inspection_bringup.launch.py —— 一键拉起消防巡检全链路。

包含：导航服务端 + 语音服务端 + 视觉服务端 + 编排层。
默认 dry_run=true，不动机器人、不出声、不加载模型；实机需显式 dry_run:=false。

用法：
    ros2 launch fire_inspection_orchestrator inspection_bringup.launch.py
    ros2 launch fire_inspection_orchestrator inspection_bringup.launch.py dry_run:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _include(package: str, launch_file: str):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package), "launch", launch_file)
        )
    )


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument(
            "dry_run",
            default_value="true",
            description="true=全链路模拟，不驱动底盘/不出声/不加载模型",
        ),
    ]

    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("fire_inspection_nav"),
                "launch",
                "navigate_to.launch.py",
            )
        ),
        launch_arguments={"dry_run": LaunchConfiguration("dry_run")}.items(),
    )
    voice = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("fire_inspection_voice"),
                "launch",
                "speak.launch.py",
            )
        ),
        launch_arguments={"dry_run": LaunchConfiguration("dry_run")}.items(),
    )
    vision = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("fire_inspection_vision"),
                "launch",
                "inspect.launch.py",
            )
        ),
        launch_arguments={"dry_run": LaunchConfiguration("dry_run")}.items(),
    )
    orchestrator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("fire_inspection_orchestrator"),
                "launch",
                "orchestrator.launch.py",
            )
        ),
        launch_arguments={"dry_run": LaunchConfiguration("dry_run")}.items(),
    )

    return LaunchDescription(args + [nav, voice, vision, orchestrator])
