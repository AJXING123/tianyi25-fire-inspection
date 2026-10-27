#!/usr/bin/env python3
"""orchestrator.launch.py —— 启动消防巡检编排层（单独启动，调试用）。

用法：
    ros2 launch fire_inspection_orchestrator orchestrator.launch.py
    ros2 launch fire_inspection_orchestrator orchestrator.launch.py start_delay_s:=0.0
    ros2 launch fire_inspection_orchestrator orchestrator.launch.py head_stop_enabled:=false

注意：编排层本身没有 dry_run 概念，它只负责按顺序调用 Action；
是否真运动/真推理由 nav / vision 各自的 dry_run 参数决定。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument(
            "script_file",
            default_value="",
            description="巡检脚本 YAML，留空使用包内 config/inspection_script.yaml",
        ),
        DeclareLaunchArgument(
            "nav_config_file",
            default_value="",
            description="导航点位 YAML，留空使用 fire_inspection_nav 包内 config/pois.yaml",
        ),
        DeclareLaunchArgument(
            "start_delay_s",
            default_value="5.0",
            description="启动后等待多少秒再开始巡检（给其他服务端留出就绪时间）",
        ),
        DeclareLaunchArgument(
            "head_stop_enabled",
            default_value="true",
            description="true=每个巡检点前让头部停扫并回中，再取图",
        ),
        DeclareLaunchArgument(
            "speak_enabled",
            default_value="true",
            description="true=按点位播报巡检结论；false=只记录日志不出声",
        ),
        DeclareLaunchArgument(
            "speak_timeout_s",
            default_value="40.0",
            description="单条播报的最长等待时间（语音服务端自身硬上限 30s）",
        ),
        DeclareLaunchArgument(
            "transit_watch",
            default_value="true",
            description="true=导航移动路段（去程点位之间+返程）开启持续检测与头部扫视",
        ),
    ]

    orchestrator = Node(
        package="fire_inspection_orchestrator",
        executable="orchestrator_node",
        name="fire_inspection_orchestrator",
        output="screen",
        parameters=[{
            "script_file": LaunchConfiguration("script_file"),
            "nav_config_file": LaunchConfiguration("nav_config_file"),
            "start_delay_s": LaunchConfiguration("start_delay_s"),
            "head_stop_enabled": LaunchConfiguration("head_stop_enabled"),
            "speak_enabled": LaunchConfiguration("speak_enabled"),
            "speak_timeout_s": LaunchConfiguration("speak_timeout_s"),
            "transit_watch": LaunchConfiguration("transit_watch"),
        }],
    )

    return LaunchDescription(args + [orchestrator])
