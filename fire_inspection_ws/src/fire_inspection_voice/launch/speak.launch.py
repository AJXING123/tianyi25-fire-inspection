#!/usr/bin/env python3
"""speak.launch.py —— 启动消防巡检语音服务端（fire_inspection_voice）。

用法：
    ros2 launch fire_inspection_voice speak.launch.py
    ros2 launch fire_inspection_voice speak.launch.py dry_run:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument("tts_service", default_value="/intelligent_interaction/tts/play"),
        DeclareLaunchArgument("audio_id", default_value="audio_guide"),
        DeclareLaunchArgument("query_timeout_s", default_value="30.0"),
        DeclareLaunchArgument(
            "dry_run",
            default_value="true",
            description="true=不真实播报，仅验证链路",
        ),
    ]

    speak_action_server = Node(
        package="fire_inspection_voice",
        executable="speak_action_server",
        name="fire_speak",
        output="screen",
        parameters=[{
            "tts_service": LaunchConfiguration("tts_service"),
            "audio_id": LaunchConfiguration("audio_id"),
            "query_timeout_s": LaunchConfiguration("query_timeout_s"),
            "dry_run": LaunchConfiguration("dry_run"),
        }],
    )

    return LaunchDescription(args + [speak_action_server])
