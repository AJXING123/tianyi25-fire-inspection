#!/usr/bin/env python3
"""inspection_bringup.launch.py —— 一键拉起消防巡检全链路。

包含：头部扫视服务端 + 导航服务端 + 语音服务端 + 视觉服务端 + 编排层。
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
        DeclareLaunchArgument(
            "start_scan",
            default_value="true",
            description="true=拉起常驻检测节点，移动路段（含返程）持续巡检",
        ),
        DeclareLaunchArgument(
            "transit_watch",
            default_value="true",
            description="true=移动路段头部扫视+检测；false=行走时头部静止（排查斜走用）",
        ),
    ]

    # 头部扫视服务端：不自动扫视（由编排层到点后下发回中/停扫指令），
    # dry_run 跟随全链路开关，模拟模式下只打印 /head/cmd 不真正下发
    head = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("fire_inspection_head"),
                "launch",
                "head_sweep.launch.py",
            )
        ),
        launch_arguments={
            "dry_run": LaunchConfiguration("dry_run"),
            "auto_start": "false",
        }.items(),
    )

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
        launch_arguments={
            "dry_run": LaunchConfiguration("dry_run"),
            "start_scan": LaunchConfiguration("start_scan"),
        }.items(),
    )
    # 编排层没有 dry_run 概念：真运动/真推理由 nav / vision 的 dry_run 决定，
    # 这里只按默认 start_delay_s 等待其他服务端就绪后开始巡检。
    orchestrator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("fire_inspection_orchestrator"),
                "launch",
                "orchestrator.launch.py",
            )
        ),
        launch_arguments={
            "transit_watch": LaunchConfiguration("transit_watch"),
        }.items(),
    )

    return LaunchDescription(args + [head, nav, voice, vision, orchestrator])
