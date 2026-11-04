#!/usr/bin/env python3
"""inspection_bringup.launch.py —— 一键拉起消防巡检全链路。

包含：头部扫视服务端 + 导航服务端 + 语音服务端 + 视觉服务端 + 听觉检测 + 编排层。
默认 dry_run=true，不动机器人、不出声、不加载模型；实机需显式 dry_run:=false。
只想听声音、不动车时用：dry_run:=true speak_dry_run:=false（语音单独走真实播报）。

用法：
    ros2 launch fire_inspection_orchestrator inspection_bringup.launch.py
    ros2 launch fire_inspection_orchestrator inspection_bringup.launch.py dry_run:=false
    ros2 launch fire_inspection_orchestrator inspection_bringup.launch.py dry_run:=true speak_dry_run:=false

注意：navigate-to-server / head-sweep / fire-inspection-orchestrator 是常驻
systemd 服务。服务在跑时本 launch 会复用，不再重复拉起；要做 dry_run 模拟
请先停掉对应服务，否则会复用真实服务端。
"""

import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _include(package: str, launch_file: str):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package), "launch", launch_file)
        )
    )


def _service_running(pattern: str, unit: str) -> bool:
    """判断常驻 systemd 服务是否已在提供同名服务端。

    本机导航/头部/编排层都有常驻 systemd 服务；launch 再拉一套会出现
    重复 action server，甚至两个编排层并发消费 start。这里在生成
    launch 时先探测，服务已在跑就复用、不重复拉起。
    先看 systemd 单元状态（避免服务重启瞬间漏判），再看进程兜底。
    """
    try:
        if subprocess.run(
            ["systemctl", "is-active", "--quiet", unit],
            capture_output=True, timeout=2,
        ).returncode == 0:
            return True
    except (OSError, subprocess.SubprocessError):
        pass
    try:
        result = subprocess.run(
            ["pgrep", "-f", pattern],
            capture_output=True, text=True, timeout=2,
        )
    except (OSError, subprocess.SubprocessError):
        return False
    for pid in result.stdout.split():
        try:
            with open(f"/proc/{pid}/cgroup", "r", encoding="utf-8") as cgroup:
                if unit in cgroup.read():
                    return True
        except OSError:
            continue
    return False


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
        # 语音单独开关：默认跟随 dry_run。dry_run:=true 且 speak_dry_run:=false 时
        # 底盘/模型仍走模拟，但会真实出声，便于「按 B 听声音」验证而不动车。
        DeclareLaunchArgument(
            "speak_dry_run",
            default_value=LaunchConfiguration("dry_run"),
            description="true=语音只打印不播报；默认跟随 dry_run",
        ),
        DeclareLaunchArgument(
            "transit_watch",
            default_value="true",
            description="true=移动路段头部扫视+检测；false=行走时头部静止（排查斜走用）",
        ),
        DeclareLaunchArgument(
            "start_head",
            default_value="true",
            description="true=由本 launch 拉起头部节点；false=复用开机自启的头部节点（head-sweep.service）",
        ),
        DeclareLaunchArgument(
            "start_audio",
            default_value="true",
            description="true=拉起听觉检测节点（fire_inspection_audio），监听防空警报；false=跳过",
        ),
        DeclareLaunchArgument(
            "auto_start",
            default_value="false",
            description="true=编排层启动后自动开始巡检；按键触发会传 true",
        ),
        # 钉钉自定义机器人推送 —— 异常抓拍时发 Markdown 卡片到群里
        DeclareLaunchArgument(
            "dingtalk_enabled",
            default_value="false",
            description="true=启用钉钉异常推送；false=关闭（默认）",
        ),
        DeclareLaunchArgument(
            "dingtalk_access_token",
            default_value="",
            description="钉钉机器人 webhook URL 里 ?access_token= 后面那串",
        ),
        DeclareLaunchArgument(
            "dingtalk_secret",
            default_value="",
            description="钉钉机器人安全设置里的加签密钥（加签模式必填）",
        ),
    ]

    # 常驻 systemd 服务已在跑时复用，避免同一 action/service 出现两套实例
    nav_in_service = _service_running(
        "navigate_to_server", "navigate-to-server.service")
    head_in_service = _service_running(
        "head_sweep_server", "head-sweep.service")
    # 编排层是单实例（flock）。常驻服务已在跑时必须复用，否则本 launch 里
    # 再起一个会因拿不到锁立刻退出；按键触发随后补发的 start 依然有效，
    # 但日志里会出现「本 launch 的编排层启动即退出」的误导信息。
    orch_in_service = _service_running(
        "orchestrator_node", "fire-inspection-orchestrator.service")
    if nav_in_service:
        print("[bringup] navigate-to-server.service 已在运行，复用导航服务端")
    if head_in_service:
        print("[bringup] head-sweep.service 已在运行，start_head:=true 时复用")
    if orch_in_service:
        print("[bringup] fire-inspection-orchestrator.service 已在运行，复用编排层")

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
        # start_head:=true 且 head-sweep.service 未运行时才拉起，避免重复头部服务端
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration("start_head"), "' == 'true'",
            " and ", "True" if not head_in_service else "False",
        ])),
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
        # 只传 speak_dry_run：多个子链路共用 dry_run 名字会互相覆盖（实测
        # 语音的 dry_run 会渗到后面视觉 include），这里保持名字唯一
        launch_arguments={"speak_dry_run": LaunchConfiguration("speak_dry_run")}.items(),
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
    # 听觉检测：防空警报监听，dry_run 跟随全链路（模拟模式只打印不发 alert）
    audio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("fire_inspection_audio"),
                "launch",
                "audio_scan.launch.py",
            )
        ),
        launch_arguments={
            "dry_run": LaunchConfiguration("dry_run"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("start_audio")),
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
            "auto_start": LaunchConfiguration("auto_start"),
            "dingtalk_enabled": LaunchConfiguration("dingtalk_enabled"),
            "dingtalk_access_token": LaunchConfiguration("dingtalk_access_token"),
            "dingtalk_secret": LaunchConfiguration("dingtalk_secret"),
        }.items(),
        # 常驻编排层已在跑时复用，避免重复实例因单实例锁启动即退出
        condition=IfCondition("false" if orch_in_service else "true"),
    )

    # 前置清理：清理「上一轮 launch 残留」且不属于常驻 systemd 服务的进程。
    # 注意三点：
    # 1) 常驻服务实例（导航/头部/编排层）必须保留，否则 systemd 会重启，
    #    反而重新制造两套实例；
    # 2) 不要删除 /tmp/fire_inspection_orchestrator.lock —— flock 绑定在
    #    inode 上，删掉锁文件会让新编排层拿到另一把锁，与常驻实例并发运行。
    #    （11-03 上午 11:56/11:57 两轮巡检启动即失败即由此引起。）
    # 3) 光杀节点不够：上一轮的 `ros2 launch` 父进程如果卡住不退，会一直占着
    #    进程组，后面按 B 也永远不再起新 launch。这里按进程组补杀旧 launch，
    #    并用祖先链排除当前 launch 自己，避免自杀。
    # ExecuteProcess 默认阻塞直到命令完成，所以会先清理完再继续拉起后续节点。
    pre_cleanup = ExecuteProcess(
        cmd=[
            "bash", "-c",
            """
set +e
patterns='[o]rchestrator_node|[n]avigate_to_server|[h]ead_sweep_server|[i]nspect_action_server|[v]ision_scan_node|[s]peak_action_server|[a]udio_scan_node'
launch_pattern='[r]os2 launch fire_inspection_orchestrator inspection_bringup.launch.py'
self_pid=$$
self_pgid=$(ps -o pgid= -p "$self_pid" 2>/dev/null | tr -d ' ')

# pre_cleanup 自己是当前 launch 的子进程；当前 launch 及其所有祖先都不能杀。
ancestors=" "
pid=$self_pid
while [ -n "$pid" ] && [ "$pid" != "0" ] && [ "$pid" != "1" ]; do
    ancestors="$ancestors$pid "
    pid=$(awk '/^PPid:/{print $2}' "/proc/$pid/status" 2>/dev/null)
done
ancestors="${ancestors}1 "
is_ancestor() {
    case " $ancestors " in
        *" $1 "*) return 0 ;;
    esac
    return 1
}

is_service_node() {
    cg=$(cat "/proc/$1/cgroup" 2>/dev/null)
    case "$cg" in
        *navigate-to-server.service*|*head-sweep.service*|*fire-inspection-orchestrator.service*)
            return 0 ;;
    esac
    return 1
}

# 优先按进程组杀；旧 launch 由 Popen(start_new_session=True) 拉起，pgid 独立。
# 若 pgid 缺失/非法/与当前 launch 同组，则退化为只杀单个 PID。
kill_pid() {
    sig=$1
    pid=$2
    pgid=$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ')
    case "$pgid" in
        ''|*[!0-9]*)
            kill -s "$sig" "$pid" 2>/dev/null || true
            return
            ;;
    esac
    if [ "$pgid" -le 1 ] || [ "$pgid" = "$self_pgid" ]; then
        kill -s "$sig" "$pid" 2>/dev/null || true
    else
        kill -s "$sig" -- "-$pgid" 2>/dev/null \\
            || kill -s "$sig" "$pid" 2>/dev/null || true
    fi
}

# 第一遍：先杀上一轮 launch 父进程组，再杀残留节点
for pid in $(pgrep -f "$launch_pattern" || true); do
    if is_ancestor "$pid"; then
        echo "[bringup] keep current launch pid=$pid"
        continue
    fi
    echo "[bringup] kill stale launch pid=$pid"
    kill_pid TERM "$pid"
done
for pid in $(pgrep -f "$patterns" || true); do
    if is_service_node "$pid" || is_ancestor "$pid"; then
        echo "[bringup] keep systemd-managed/current pid=$pid"
        continue
    fi
    echo "[bringup] kill stale node pid=$pid"
    kill_pid TERM "$pid"
done
sleep 1
# 第二遍：TERM 没退的强制 KILL
for pid in $(pgrep -f "$launch_pattern" || true); do
    if is_ancestor "$pid"; then
        continue
    fi
    echo "[bringup] force kill stale launch pid=$pid"
    kill_pid KILL "$pid"
done
for pid in $(pgrep -f "$patterns" || true); do
    if is_service_node "$pid" || is_ancestor "$pid"; then
        continue
    fi
    echo "[bringup] force kill stale node pid=$pid"
    kill_pid KILL "$pid"
done
echo "[bringup] cleanup done"
""",
        ],
        name="bringup_pre_cleanup",
    )

    actions = [head, voice, vision, audio, orchestrator]
    if not nav_in_service:
        actions.insert(0, nav)
    # 必须等 pre_cleanup 退出后再拉起业务节点：ExecuteProcess 不会阻塞
    # 后续动作，若直接并列会把刚启动的新实例当成残留节点杀掉。
    return LaunchDescription(args + [
        pre_cleanup,
        RegisterEventHandler(
            OnProcessExit(target_action=pre_cleanup, on_exit=actions)
        ),
    ])
