#!/usr/bin/env python3
"""遥控器按键触发巡检的离线测试（不初始化 rclpy、不启动子进程）。

覆盖：
  - build_launch_command：先 source 环境再 exec ros2 launch（服务环境过时
    时唯一可靠的修复手段），路径带空格要能用，空脚本项要跳过；
  - default_setup_scripts：至少包含 ROS / xos / 本工作区三份环境脚本，
    且推导出的文件真实存在；
  - 按键事件过滤：B 键按下才触发、键位保持不动不重复、连按去抖、
    运行中再按走终止分支。

运行前需已 source ROS 环境与工作区 install：
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    python3 tests/test_button_trigger.py
"""

from __future__ import annotations

import os
import signal
import sys
import time
from types import SimpleNamespace

_HERE = os.path.dirname(os.path.abspath(__file__))
_WS = os.path.normpath(os.path.join(_HERE, ".."))
_SRC = os.path.join(_WS, "src", "fire_inspection_orchestrator")
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from bodyctrl_msgs.msg import SbusData  # noqa: E402

from fire_inspection_orchestrator import button_trigger_node as button_mod  # noqa: E402
from fire_inspection_orchestrator.button_trigger_node import (  # noqa: E402
    ButtonTrigger,
    build_launch_args,
    build_launch_command,
    default_setup_scripts,
)

TESTS = []


def case(fn):
    TESTS.append(fn)
    return fn


# ── 拉起命令拼装 ───────────────────────────────────────────────────

@case
def test_command_sources_then_exec():
    """顺序必须是「逐个 source && exec ros2 launch」，不能只剩 exec。"""
    cmd = build_launch_command(
        ["/opt/ros/jazzy/setup.bash", "/ws/install/setup.bash"],
        "fire_inspection_orchestrator", "inspection_bringup.launch.py",
        ["dry_run:=false"])
    assert cmd[:2] == ["/bin/bash", "-c"], cmd
    body = cmd[2]
    assert body.startswith(
        "source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && "), body
    assert body.endswith(
        "exec ros2 launch fire_inspection_orchestrator "
        "inspection_bringup.launch.py dry_run:=false"), body


@case
def test_command_quotes_paths_with_space():
    """安装路径可能含空格，source 的参数必须被正确引用。"""
    cmd = build_launch_command(
        ["/home/nvidia/my ws/install/setup.bash"], "pkg", "a.launch.py", [])
    assert "source '/home/nvidia/my ws/install/setup.bash'" in cmd[2], cmd


@case
def test_command_skips_blank_scripts():
    """参数里多打的冒号不应产生 `source ` 这种空语句。"""
    cmd = build_launch_command(
        ["", "/opt/ros/jazzy/setup.bash", ""], "pkg", "a.launch.py", [])
    assert cmd[2].count("source ") == 1, cmd[2]


@case
def test_command_quotes_launch_args():
    """launch 参数含空格/特殊字符时也要作为一个整体传下去。"""
    cmd = build_launch_command(
        [], "pkg", "a.launch.py", ["log_level:=warn", "node_name:=my node"])
    assert "exec ros2 launch pkg a.launch.py log_level:=warn 'node_name:=my node'" \
        in cmd[2], cmd[2]


@case
def test_command_without_scripts_is_plain_exec():
    """留空 setup_scripts = 旧行为（直接继承环境），命令仍要合法。"""
    cmd = build_launch_command([], "pkg", "a.launch.py", [])
    assert cmd == ["/bin/bash", "-c", "exec ros2 launch pkg a.launch.py"], cmd


# ── launch 参数 ────────────────────────────────────────────────────

@case
def test_args_default_follow_dry_run():
    """speak_real 默认 false：dry_run=true 时语音也模拟（旧行为）。"""
    args = build_launch_args(True, False, True)
    assert args == ["dry_run:=true", "start_head:=false", "transit_watch:=true"], args


@case
def test_args_speak_real_overrides_only_in_dry_run():
    """dry_run=true + speak_real=true：不动车但真实播报（按 B 听声音）。"""
    args = build_launch_args(True, False, True, True)
    assert "dry_run:=true" in args, args
    assert "speak_dry_run:=false" in args, args


@case
def test_args_speak_real_ignored_when_not_dry_run():
    """dry_run=false 时语音本来就真实，不必也不应再传 speak_dry_run。"""
    args = build_launch_args(False, False, True, True)
    assert "dry_run:=false" in args, args
    assert not [a for a in args if a.startswith("speak_dry_run")], args


@case
def test_args_extra_args_appended():
    """extra_args 里可以追加/覆盖任意 launch 参数。"""
    args = build_launch_args(
        True, False, True, False, "start_scan:=false dry_run:=false")
    assert args[-2:] == ["start_scan:=false", "dry_run:=false"], args


@case
def test_args_auto_start_appended():
    """auto_start=true 时按 B 拉起后自动开始巡检；默认不传该参数。"""
    args = build_launch_args(False, False, True, auto_start=True)
    assert "auto_start:=true" in args, args
    default_args = build_launch_args(False, False, True)
    assert not [a for a in default_args if a.startswith("auto_start")], (
        default_args)


# ── 默认环境脚本 ───────────────────────────────────────────────────

@case
def test_default_setup_scripts_cover_ros_xos_and_ws():
    """必须重新 source 系统 ROS、xos 和工作区 install 三处。"""
    scripts = default_setup_scripts()
    assert "/opt/ros/jazzy/setup.bash" in scripts, scripts
    assert "/home/nvidia/xos/setup.bash" in scripts, scripts
    ws_setup = os.path.join(_WS, "install", "setup.bash")
    assert ws_setup in scripts, scripts


@case
def test_default_setup_scripts_exist():
    """推导出的脚本必须真实存在，否则 source 失败会让启动直接中断。"""
    for path in default_setup_scripts():
        assert os.path.isfile(path), f"环境脚本不存在: {path}"


# ── 按键事件过滤 ───────────────────────────────────────────────────

class _NoopLogger:
    def __getattr__(self, _name):
        return lambda *a, **k: None


def _fake_node(running=False):
    """构造不连 ROS 的节点实例，只保留按键路径需要的字段。"""
    inst = object.__new__(ButtonTrigger)
    inst.get_logger = lambda: _NoopLogger()
    inst._names = {}
    inst.launch_stack = False
    inst.trigger_key = int(SbusData.KEY_B_DOWN)
    inst.debounce = 2.0
    inst.stop_on_second = True
    inst._last_trigger = 0.0
    inst._state_active = running
    inst._stop_at = None
    inst._force_stop_at = None
    inst._restart_pending = False
    inst._proc = SimpleNamespace(poll=lambda: None) if running else None
    inst._kill_at = None
    inst._kill_hard_at = None
    inst.terminate_grace = 5.0
    inst.terminate_kill_grace = 3.0
    inst._start_retry = False
    inst._start_retry_stop_sent = False
    inst._start_retry_stop_future = None
    inst.start_retry_timeout = 60.0
    inst._start_future = None
    inst._start_at = None
    inst._start_deadline = None
    inst._last_start_error = ""
    inst._log_path = None
    inst.calls = []
    inst._start = lambda: inst.calls.append("start")
    inst._request_stop = lambda: inst.calls.append("stop")
    return inst


def _evt(new, old):
    return SimpleNamespace(key_event_new=new, key_event_old=old)


@case
def test_b_down_triggers_once():
    """按下 B（KEY_B_DOWN，旧键态为松开）→ 启动一次巡检。"""
    node = _fake_node()
    node._on_key_event(_evt(SbusData.KEY_B_DOWN, SbusData.KEY_NONE))
    assert node.calls == ["start"], node.calls


@case
def test_b_up_does_not_trigger():
    """松开 B 产生的 KEY_B_UP 不得触发巡检（否则一按一松跑两轮）。"""
    node = _fake_node()
    node._on_key_event(_evt(SbusData.KEY_B_UP, SbusData.KEY_B_DOWN))
    assert node.calls == [], node.calls


@case
def test_other_keys_ignored():
    """A 键被平台自检占用，其余键与本节点无关。"""
    node = _fake_node()
    for key in (SbusData.KEY_A_DOWN, SbusData.KEY_C_DOWN, SbusData.KEY_NONE):
        node._on_key_event(_evt(key, SbusData.KEY_NONE))
    assert node.calls == [], node.calls


@case
def test_held_key_not_repeated():
    """键位保持不动（old == new）不重复触发。"""
    node = _fake_node()
    node._on_key_event(_evt(SbusData.KEY_B_DOWN, SbusData.KEY_B_DOWN))
    assert node.calls == [], node.calls


@case
def test_debounce_suppresses_double_press():
    """2s 内连按只认第一次，避免抖动把同一轮巡检拉起两次。"""
    node = _fake_node()
    node._on_key_event(_evt(SbusData.KEY_B_DOWN, SbusData.KEY_NONE))
    node._on_key_event(_evt(SbusData.KEY_B_DOWN, SbusData.KEY_NONE))
    assert node.calls == ["start"], node.calls


@case
def test_second_press_while_running_requests_stop():
    """运行中再按一次 → 走终止分支，不重复启动。"""
    node = _fake_node(running=True)
    node._last_trigger = -100.0
    node._on_key_event(_evt(SbusData.KEY_B_DOWN, SbusData.KEY_NONE))
    assert node.calls == ["stop"], node.calls


@case
def test_press_during_terminal_grace_queues_restart():
    """终态宽限期内按 B 视为重新开始，不再多按一次去「停止」。"""
    node = _state_node()
    node._stop_at = 1.0  # 自动关闭 launch 的宽限期还没到
    node._last_trigger = -100.0
    node._on_key_event(_evt(SbusData.KEY_B_DOWN, SbusData.KEY_NONE))
    assert node.calls == [], node.calls
    assert node._restart_pending is True


@case
def test_trigger_key_default_is_b_down():
    """默认触发键必须是 KEY_B_DOWN(=4)，与部署参数 trigger_key:=4 一致。"""
    assert int(SbusData.KEY_B_DOWN) == 4, SbusData.KEY_B_DOWN


# ── 终态判定 ───────────────────────────────────────────────────────

def _state_node(running=True):
    """构造 launch 模式下只走状态回调的实例。"""
    inst = _fake_node(running=running)
    inst.launch_stack = True
    inst.stop_on_done = True
    inst.grace = 3.0
    inst._state_text = ""
    inst._state_active = False
    inst._stop_at = None
    inst._force_stop_at = None
    inst._stop_deadline = None
    inst._stop_future = None
    return inst


@case
def test_aborted_with_detail_is_terminal():
    """aborted:run 带阶段后缀也必须算终态，否则 launch 永不自动关闭。"""
    node = _state_node()
    node._on_state(SimpleNamespace(data="aborted:run"))
    assert node._stop_at is not None, "aborted:run 应安排关闭 launch"
    assert node._state_active is False, node._state_active


@case
def test_stopped_with_detail_is_terminal():
    """stopped:xxx 同样只按冒号前的枚举判定。"""
    node = _state_node()
    node._on_state(SimpleNamespace(data="stopped:run"))
    assert node._stop_at is not None


@case
def test_running_with_detail_is_not_terminal():
    """running:inspect:POI_02 期间不得安排关闭 launch。"""
    node = _state_node()
    node._on_state(SimpleNamespace(data="running:inspect:POI_02"))
    assert node._stop_at is None
    assert node._state_active is True


@case
def test_non_terminal_state_clears_pending_close():
    """已经安排关闭后又收到运行状态 → 撤销关闭（避免误杀正在跑的一轮）。"""
    node = _state_node()
    node._on_state(SimpleNamespace(data="aborted:run"))
    assert node._stop_at is not None
    node._on_state(SimpleNamespace(data="running:nav:POI_01"))
    assert node._stop_at is None


# ── SIGINT 卡住后的 SIGKILL 兜底 ─────────────────────────────────

@case
def test_terminate_escalates_to_sigkill():
    """SIGINT 宽限到期必须发 SIGKILL，避免 launch 卡死后再也起不来。"""
    node = _state_node()
    node.launch_stack = True
    node._proc = SimpleNamespace(pid=12345, poll=lambda: None, returncode=None)
    sent = []
    orig_getpgid, orig_killpg = button_mod.os.getpgid, button_mod.os.killpg
    button_mod.os.getpgid = lambda pid: pid
    button_mod.os.killpg = lambda pgid, sig: sent.append(sig)
    try:
        node._terminate()
        assert sent == [signal.SIGINT], sent
        assert node._kill_at is not None
        node._kill_at = time.monotonic() - 1.0  # 模拟宽限已过
        node._force_kill()
        assert sent[-1] == signal.SIGKILL, sent
        assert node._kill_hard_at is not None
    finally:
        button_mod.os.getpgid = orig_getpgid
        button_mod.os.killpg = orig_killpg


# ── start 被拒后的 stop + 重试 ────────────────────────────────────

def _done_future(success=True, message="ok"):
    return SimpleNamespace(
        done=lambda: True,
        result=lambda: SimpleNamespace(success=success, message=message),
    )


@case
def test_busy_start_requests_stop_then_retries():
    """编排层返回「正在进行中」时要先补 stop，再在窗口内重试 start。"""
    node = _state_node()
    node._state_active = True
    node._ctl_cli = SimpleNamespace(
        service_is_ready=lambda: True,
        call_async=lambda req: node.calls.append(req.cmd) or _done_future(),
    )
    now = time.monotonic()
    node._schedule_start_retry(now, "本轮巡检正在进行中，请先 stop 或等待结束")
    assert node.calls == ["stop"], node.calls
    assert node._start_retry is True
    assert node._start_retry_stop_sent is True
    assert node._start_at is not None
    assert node._start_deadline == now + node.start_retry_timeout
    # 第二次失败不得重复发 stop
    node._schedule_start_retry(now, "本轮巡检正在进行中，请先 stop 或等待结束")
    assert node.calls == ["stop"], node.calls


@case
def test_start_retry_gives_up_after_window():
    """超过重试窗口就放弃，避免状态一直不退出时无限重试。"""
    node = _state_node()
    now = time.monotonic()
    node._start_retry = True
    node._start_deadline = now - 1.0
    node._schedule_start_retry(now, "本轮巡检正在进行中")
    assert node._start_retry is False
    assert node._start_at is None
    assert node._start_deadline is None


@case
def test_start_config_error_does_not_retry():
    """配置不可用不是「等一会就好」，不进入重试。"""
    node = _state_node()
    node._schedule_start_retry(time.monotonic(), "配置不可用：yml 解析失败")
    assert node._start_retry is False
    assert node._start_at is None
    assert node.calls == []


def main() -> int:
    failed = 0
    for fn in TESTS:
        try:
            fn()
        except Exception as exc:  # noqa: BLE001 - 测试脚本要打印全部失败
            failed += 1
            print(f"FAIL {fn.__name__}: {exc}")
        else:
            print(f"PASS {fn.__name__}")
    print(f"\n{len(TESTS) - failed}/{len(TESTS)} 通过")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
