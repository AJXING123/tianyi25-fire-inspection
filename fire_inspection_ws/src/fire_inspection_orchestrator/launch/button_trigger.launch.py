#!/usr/bin/env python3
"""button_trigger.launch.py —— 遥控器按键触发巡检全流程。

只启动监听节点；真正的巡检链路由按键触发时以子进程拉起
inspection_bringup.launch.py（dry_run 由参数决定）。

用法：
    ros2 launch fire_inspection_orchestrator button_trigger.launch.py
    ros2 launch fire_inspection_orchestrator button_trigger.launch.py dry_run:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument(
            "trigger_key", default_value="4",
            description="触发键位编码（SbusData.key_event_new）：3=B_UP 4=B_DOWN；"
                        "A(1/2) 被平台自检占用，其余键位见 SbusData.KEY_*"),
        DeclareLaunchArgument(
            "dry_run", default_value="false",
            description="true=按键只跑模拟链路（不动底盘不出声）",
        ),
        DeclareLaunchArgument(
            "start_head", default_value="false",
            description="true=由巡检 launch 拉起头部；false=复用开机自启的头部",
        ),
        DeclareLaunchArgument(
            "transit_watch", default_value="true",
            description="true=行走时头部扫视+检测",
        ),
        DeclareLaunchArgument(
            "extra_args", default_value="",
            description="追加给 inspection_bringup 的参数，如 start_scan:=false",
        ),
        DeclareLaunchArgument(
            "stop_on_done", default_value="true",
            description="true=巡检结束自动关闭本次 launch，便于再次按键启动",
        ),
        DeclareLaunchArgument(
            "stop_on_second_press", default_value="true",
            description="true=巡检运行中再按一次即终止本轮（经 /fire_inspection/control 安全收尾）",
        ),
        DeclareLaunchArgument(
            "stop_confirm_s", default_value="10.0",
            description="终止请求的确认超时；超时直接关闭本次 launch",
        ),
    ]
    node = Node(
        package="fire_inspection_orchestrator",
        executable="button_trigger_node",
        name="inspection_button_trigger",
        output="screen",
        parameters=[{
            "trigger_key": LaunchConfiguration("trigger_key"),
            "dry_run": LaunchConfiguration("dry_run"),
            "start_head": LaunchConfiguration("start_head"),
            "transit_watch": LaunchConfiguration("transit_watch"),
            "extra_args": LaunchConfiguration("extra_args"),
            "stop_on_done": LaunchConfiguration("stop_on_done"),
            "stop_on_second_press": LaunchConfiguration("stop_on_second_press"),
            "stop_confirm_s": LaunchConfiguration("stop_confirm_s"),
        }],
    )
    return LaunchDescription(args + [node])
