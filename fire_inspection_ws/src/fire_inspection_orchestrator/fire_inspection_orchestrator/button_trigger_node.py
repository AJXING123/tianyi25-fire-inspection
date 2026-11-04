"""遥控器按键触发巡检：拨动指定键位 → 通过唯一的控制服务启停常驻巡检。

链路（推荐，launch_stack=false）:
    /sbus_data/event (bodyctrl_msgs/SbusData)
      -> 本节点识别指定 key_event_new（默认 KEY_B_DOWN，A 键由平台自检占用）
      -> 请求 /fire_inspection/control (cmd=start|stop) 启停常驻编排层
      -> 订阅 /fire_inspection/state 判断当前是否正在巡检

链路（兼容旧行为，launch_stack=true）:
      -> 子进程 ros2 launch fire_inspection_orchestrator inspection_bringup.launch.py
      -> 订阅 /fire_inspection/state，收到 done/stopped/aborted 后关闭本次 launch

语义约定：
- 巡检未运行时按键 = 启动本轮巡检（常驻模式只发 start，不重复拉起节点）；
- 巡检运行中按键 = 请求终止本轮（调 /fire_inspection/control cmd=stop，
  由编排层取消当前动作并安全收尾；launch 模式下失败/超时才会关闭自己拉起的 launch）；
- 同一键位保持不动不会重复触发（要求 key_event_old != key_event_new）；
- dry_run / speak_real / start_head 仅对 launch_stack=true 的旧链路有意义；
- 遥控器为三档开关：拨到配置档位算一次触发，重新巡检需先拨回再拨下。
"""

from __future__ import annotations

import os
import shlex
import signal
import subprocess
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from bodyctrl_msgs.msg import SbusData
from fire_inspection_interfaces.srv import InspectionControl
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

# 终端态：编排层结束后自动关闭本次 launch
TERMINAL_STATES = {"done", "stopped", "aborted"}


def default_setup_scripts() -> list:
    """按当前安装位置推导需要 source 的环境脚本。

    服务只在启动时读一次环境：若之后又 build 过（symlink-install 会把
    build/<pkg> 加进 PYTHONPATH），长期运行的服务里子进程会拿到过时环境，
    表现为「按 B 后 launch 里所有节点立刻退出」。所以每次拉起 launch 都
    重新 source 一遍，跟当前 install 树保持一致。
    """
    scripts = ["/opt/ros/jazzy/setup.bash", "/home/nvidia/xos/setup.bash"]
    try:
        share = get_package_share_directory("fire_inspection_orchestrator")
        install_prefix = os.path.dirname(
            os.path.dirname(os.path.dirname(share)))
        setup = os.path.join(install_prefix, "setup.bash")
        if os.path.isfile(setup):
            scripts.append(setup)
    except Exception:  # noqa: BLE001 - 推导失败就只用系统环境
        pass
    return scripts


def build_launch_command(setup_scripts, pkg, launch_file, args):
    """拼出「先 source 环境、再 exec ros2 launch」的命令（纯函数，便于测试）。"""
    launch = shlex.join(["ros2", "launch", pkg, launch_file] + list(args))
    parts = [f"source {shlex.quote(p)}" for p in setup_scripts if p]
    parts.append(f"exec {launch}")
    return ["/bin/bash", "-c", " && ".join(parts)]


def build_launch_args(dry_run, start_head, transit_watch,
                      speak_real=False, extra_args="", auto_start=False):
    """拼 launch 参数（纯函数，便于测试）。

    speak_real 只在 dry_run=true 时生效：追加 speak_dry_run:=false，
    让语音单独走真实播报（车不动、模型不加载），便于「按 B 听声音」验证。
    dry_run=false 时不追加，语音跟随 dry_run=false 本身就是真实播报。

    auto_start=true：编排层等服务端就绪后自动开始巡检（按 B 直接开跑）。
    """
    args = [
        f"dry_run:={'true' if dry_run else 'false'}",
        f"start_head:={'true' if start_head else 'false'}",
        f"transit_watch:={'true' if transit_watch else 'false'}",
    ]
    if auto_start:
        args.append("auto_start:=true")
    if dry_run and speak_real:
        args.append("speak_dry_run:=false")
    return args + shlex.split(str(extra_args or ""))


def _key_names():
    """key_event 编码 -> 名称，仅用于日志可读。"""
    return {
        int(getattr(SbusData, name)): name
        for name in dir(SbusData)
        if name.startswith("KEY_") and isinstance(getattr(SbusData, name), int)
    }


class ButtonTrigger(Node):
    def __init__(self) -> None:
        super().__init__("inspection_button_trigger")

        self.declare_parameter("event_topic", "/sbus_data/event")
        # 默认 B 点动键（A 被平台 proc_manager 用于开机自检，不能占用）
        self.declare_parameter("trigger_key", int(SbusData.KEY_B_DOWN))
        self.declare_parameter("state_topic", "/fire_inspection/state")
        self.declare_parameter("launch_pkg", "fire_inspection_orchestrator")
        self.declare_parameter("launch_file", "inspection_bringup.launch.py")
        self.declare_parameter("dry_run", False)
        # 强制真实播报：dry_run=true（不动车）时也能出声，便于「按 B 听声音」验证；
        # dry_run=false 时本参数无意义（语音本来就跟 dry_run=false）
        self.declare_parameter("speak_real", False)
        self.declare_parameter("start_head", False)  # 头部由开机自启的 head-sweep.service 提供
        self.declare_parameter("transit_watch", True)
        # true=按 B 拉起链路后由编排层自动开始巡检（再按一次终止）；
        # false=只拉起服务端，等待网页/控制台发 start
        self.declare_parameter("auto_start", True)
        self.declare_parameter("extra_args", "")
        self.declare_parameter("debounce_s", 2.0)
        # 每次拉起 launch 前重新 source 的环境脚本（冒号分隔）；
        # 留空 = 直接继承本节点启动时的环境（旧行为，build 后可能过时）
        self.declare_parameter(
            "setup_scripts", ":".join(default_setup_scripts()))
        self.declare_parameter("stop_on_done", True)
        self.declare_parameter("done_grace_s", 3.0)
        self.declare_parameter("control_service", "/fire_inspection/control")
        self.declare_parameter("stop_on_second_press", True)
        self.declare_parameter("stop_confirm_s", 10.0)
        # 启动宽限：链路刚拉起、控制服务还没出现时，第二次按键不直接杀
        # launch（现场连按两次会把正在启动的巡检直接关掉），只提示后忽略；
        # 超过该时间控制服务仍不可用，才按原逻辑强制关闭。
        self.declare_parameter("startup_grace_s", 30.0)
        # true=按 B 拉起 inspection_bringup（旧行为，会尝试启动第二套服务）；
        # false=只通过 /fire_inspection/control 启停常驻编排层（推荐，统一入口）
        self.declare_parameter("launch_stack", False)
        # 常驻模式下等待控制服务就绪的最长时间
        self.declare_parameter("control_ready_timeout_s", 20.0)
        # SIGINT 后给 launch 自己收尾的时间；到点还不退就 SIGKILL 整个进程组。
        # SIGKILL 后再等一小段时间回收；仍不退则放弃等待并允许下一次按 B 重启，
        # 避免 launch 父进程卡死导致后面按 B 永远起不来。
        self.declare_parameter("terminate_grace_s", 5.0)
        self.declare_parameter("terminate_kill_grace_s", 3.0)
        # start 被「本轮巡检正在进行中」拒绝时，先 stop、等状态退出运行后
        # 再重试 start 的总窗口。
        self.declare_parameter("start_retry_timeout_s", 60.0)

        self.trigger_key = int(self.get_parameter("trigger_key").value)
        self.debounce = float(self.get_parameter("debounce_s").value)
        self.stop_on_done = bool(self.get_parameter("stop_on_done").value)
        self.launch_stack = bool(self.get_parameter("launch_stack").value)
        self.control_ready_timeout = float(
            self.get_parameter("control_ready_timeout_s").value)
        self.grace = float(self.get_parameter("done_grace_s").value)
        self.stop_on_second = bool(self.get_parameter("stop_on_second_press").value)
        self.stop_confirm = float(self.get_parameter("stop_confirm_s").value)
        self.startup_grace = float(
            self.get_parameter("startup_grace_s").value)
        self.terminate_grace = float(
            self.get_parameter("terminate_grace_s").value)
        self.terminate_kill_grace = float(
            self.get_parameter("terminate_kill_grace_s").value)
        self.start_retry_timeout = float(
            self.get_parameter("start_retry_timeout_s").value)

        self._names = _key_names()
        self._proc = None
        self._started_at = None
        self._log_fp = None
        self._log_path = None
        self._last_trigger = 0.0
        self._stop_at = None
        self._force_stop_at = None
        self._stop_future = None
        # 终态等待关闭 launch 期间按 B：不再当作「停止」（那会白白多按一次），
        # 而是记下来，等 launch 关掉后立刻重新开始一轮。
        self._restart_pending = False
        # auto_start=true 时，若常驻编排层（systemd 服务）已持有单实例锁，
        # launch 里的编排层会直接退出；此时必须由本节点主动给常驻实例发 start，
        # 否则表现为「按 B 全链路起来了，但底盘不动」。
        self._start_future = None
        self._start_at = None
        self._start_deadline = None
        self._stop_deadline = None
        # SIGINT -> SIGKILL 强杀兜底的两个时间点
        self._kill_at = None
        self._kill_hard_at = None
        # start 被拒绝后的「先 stop 再重试」状态
        self._start_retry = False
        self._start_retry_stop_sent = False
        self._start_retry_stop_future = None
        self._last_start_error = ""
        # 常驻模式用 /fire_inspection/state 判断是否正在巡检；
        # launch 模式仍以自己拉起的子进程为准
        self._state_text = ""
        self._state_active = False
        self._ctl_cli = self.create_client(
            InspectionControl, str(self.get_parameter("control_service").value))

        # 网页 Bridge 与遥控器 B 键共用这个 Launch 管理入口。
        self._stack_srv = self.create_service(
            InspectionControl,
            "/fire_inspection/stack_control",
            self._on_stack_control,
        )

        self._sub = self.create_subscription(
            SbusData, str(self.get_parameter("event_topic").value),
            self._on_key_event, 10)
        state_qos = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._state_sub = self.create_subscription(
            String, str(self.get_parameter("state_topic").value),
            self._on_state, state_qos)
        self._timer = self.create_timer(0.5, self._poll)

        self.get_logger().info(
            f"按键触发就绪：{self._names.get(self.trigger_key, self.trigger_key)}"
            f" -> {'常驻控制服务 start/stop' if not self.launch_stack else 'inspection_bringup'} "
            f"dry_run={bool(self.get_parameter('dry_run').value)} "
            f"speak_real={bool(self.get_parameter('speak_real').value)} "
            f"再按一次={'终止' if self.stop_on_second else '忽略'}")

    # ---------------- 统一 Launch 管理接口 ----------------
    def _on_stack_control(self, request, response):
        cmd = (request.cmd or "").strip().lower()
        operator = request.operator_id or "web"

        if not self.launch_stack:
            response.success = False
            response.message = (
                "Launch 管理模式尚未启用，请将 button-trigger.service "
                "设置为 launch_stack:=true"
            )
            response.current_state = self._state_text or "idle"
            return response

        if cmd == "start":
            if self._running():
                response.success = False
                response.message = "巡检 Launch 已经在运行"
                response.current_state = self._state_text or "running"
                return response

            self.get_logger().info(
                f"收到统一启动请求，操作人：{operator}"
            )
            self._start()
            response.success = self._running()
            response.message = (
                "巡检 Launch 已启动"
                if response.success else "巡检 Launch 启动失败，请查看节点日志"
            )
            response.current_state = self._state_text or "starting"
            return response

        if cmd == "stop":
            if not self._running():
                response.success = True
                response.message = "巡检 Launch 当前未运行"
                response.current_state = self._state_text or "idle"
                return response

            self.get_logger().warn(
                f"收到统一停止请求，操作人：{operator}"
            )
            self._request_stop()
            response.success = True
            response.message = "已受理停止请求，正在安全停止巡检"
            response.current_state = self._state_text or "stopping"
            return response

        response.success = False
        response.message = f"Launch 管理接口不支持命令：{cmd}"
        response.current_state = self._state_text or "unknown"
        return response

    # ---------------- 按键 ----------------
    def _on_key_event(self, msg: SbusData) -> None:
        key = int(msg.key_event_new)
        if key != self.trigger_key or int(msg.key_event_old) == key:
            return
        now = time.monotonic()
        if now - self._last_trigger < self.debounce:
            self.get_logger().info("按键抖动/连按过快，忽略")
            return
        self._last_trigger = now
        if self._running():
            if (
                self.launch_stack
                and self._stop_at is not None
                and self._force_stop_at is None
            ):
                # 本轮已到终态（aborted/done/...），launch 正在宽限期内自动
                # 关闭。此时再按 B 的本意是「重新开始」，不是「停止」：
                # 记下来，等 launch 退出后自动拉起，省掉多按的那一次。
                self._restart_pending = True
                self.get_logger().info(
                    f"本轮已结束（{self._state_text}），launch 关闭后自动重新开始")
                return
            if self.stop_on_second:
                self._request_stop()
            else:
                self.get_logger().info("巡检运行中，忽略本次按键（stop_on_second_press=false）")
            return
        self._start()

    def _running(self) -> bool:
        if self.launch_stack:
            return self._proc is not None and self._proc.poll() is None
        # 常驻模式：以编排层发布的状态为准（running:xxx / paused:xxx）
        return self._state_active

    def _send_stop_request(self) -> bool:
        """向控制服务发送 stop；服务不可用或调用异常时返回 False。"""
        try:
            if not self._ctl_cli.service_is_ready():
                if not self._ctl_cli.wait_for_service(timeout_sec=1.0):
                    return False
            req = InspectionControl.Request()
            req.cmd = "stop"
            req.arg = ""
            req.operator_id = "rc_button"
            self._stop_future = self._ctl_cli.call_async(req)
            return True
        except Exception as exc:  # noqa: BLE001 - 服务异常不能中断按键流程
            self.get_logger().warn(f"发送终止请求异常（{exc}）")
            return False

    def _request_stop(self) -> None:
        """运行中再次按键：请求编排层终止本轮。

        常驻模式（launch_stack=false）只发 stop，失败时在 stop_confirm_s 内
        重试，绝不终止常驻编排层；旧 launch 模式失败/超时才直接关 launch。
        """
        if self._stop_future is not None:
            self.get_logger().info("终止请求已发出，等待编排层响应")
            return
        self.get_logger().warn("巡检运行中，再次按键 → 请求终止本轮巡检")
        # 取消尚未发出的主动 start，避免刚停就又被拉起一轮
        self._start_at = None
        self._start_deadline = None
        self._clear_start_retry()
        if not self.launch_stack:
            # 常驻模式：只发 stop，不碰任何进程。无论首次发送成功与否，
            # 都从按键时刻起算 stop_confirm_s 总超时，由 _poll 统一裁决
            self._force_stop_at = None
            self._stop_deadline = time.monotonic() + self.stop_confirm
            self._send_stop_request()
            return
        self._force_stop_at = time.monotonic() + self.stop_confirm
        if not self._ctl_cli.service_is_ready():
            elapsed = (
                time.monotonic() - self._started_at
                if self._started_at is not None else 0.0
            )
            if elapsed < self.startup_grace:
                self.get_logger().warn(
                    f"巡检链路启动中（{elapsed:.0f}s < "
                    f"{self.startup_grace:.0f}s），控制服务尚未就绪，"
                    "忽略本次终止请求；确需强制终止请稍后再按一次")
                self._force_stop_at = None
                return
            if not self._send_stop_request():
                self.get_logger().warn(
                    f"控制服务 {self._ctl_cli.srv_name} 未就绪，直接关闭 launch")
                self._terminate()
                return
            return
        self._send_stop_request()

    # ---------------- 启动 ----------------
    def _send_start_request(self) -> bool:
        """向控制服务发送 start；服务未就绪时返回 False。"""
        if not self._ctl_cli.service_is_ready():
            if not self._ctl_cli.wait_for_service(timeout_sec=1.0):
                return False
        req = InspectionControl.Request()
        req.cmd = "start"
        req.arg = ""
        req.operator_id = "rc_button"
        self._start_future = self._ctl_cli.call_async(req)
        return True

    def _clear_start_retry(self) -> None:
        self._start_retry = False
        self._start_retry_stop_sent = False

    def _send_restart_stop(self) -> None:
        """为重新开始而终止上一轮；响应失败也不关 launch。"""
        if self._start_retry_stop_future is not None:
            return
        if not self._ctl_cli.service_is_ready():
            return
        try:
            req = InspectionControl.Request()
            req.cmd = "stop"
            req.arg = ""
            req.operator_id = "rc_button_restart"
            self._start_retry_stop_future = self._ctl_cli.call_async(req)
            self._start_retry_stop_sent = True
            self.get_logger().info(
                "上一轮仍在运行，已请求编排层先 stop，退出后再重试 start")
        except Exception as exc:  # noqa: BLE001 - 服务异常不能中断按键流程
            self.get_logger().warn(f"重新开始前的 stop 请求异常（{exc}）")

    def _schedule_start_retry(self, now: float, message: str) -> None:
        """start 未成功后安排重试；被「正在进行中」拒绝时先补一个 stop。"""
        text = message or "服务调用异常"
        if "配置不可用" in text:
            self.get_logger().error(f"编排层配置不可用，放弃自动 start：{text}")
            self._start_at = None
            self._start_deadline = None
            self._clear_start_retry()
            return
        if not self._start_retry:
            self._start_retry = True
            self._start_deadline = now + self.start_retry_timeout
        elif now >= self._start_deadline:
            self.get_logger().warn(
                f"start 在 {self.start_retry_timeout:.0f}s 内仍未成功"
                f"（{text}），放弃重试；再按一次 B 可重新尝试")
            self._start_at = None
            self._start_deadline = None
            self._clear_start_retry()
            return
        busy = self._state_active or ("正在进行" in text)
        if busy and not self._start_retry_stop_sent:
            self._send_restart_stop()
        self._last_start_error = text
        self._start_at = now + 1.0

    def _request_start(self) -> None:
        """常驻模式：只请求编排层开始本轮巡检，不拉起任何 launch。"""
        if self._start_future is not None:
            self.get_logger().info("启动请求已发出，等待编排层响应")
            return
        self.get_logger().info("按键触发巡检：请求常驻编排层开始本轮巡检")
        self._stop_at = None
        self._force_stop_at = None
        self._stop_deadline = None
        if self._send_start_request():
            return
        # 控制服务尚未就绪（编排层仍在启动）：在超时窗口内每 0.5s 重试
        now = time.monotonic()
        self._start_at = now + 0.5
        self._start_deadline = now + self.control_ready_timeout
        self.get_logger().warn(
            f"控制服务 {self._ctl_cli.srv_name} 未就绪，"
            f"将在 {self.control_ready_timeout:.0f}s 内重试")

    def _start(self) -> None:
        self._restart_pending = False
        # 新一轮开始前清掉上一轮的强杀/重试残留，避免旧 Future 或旧 deadline
        # 影响这一轮。
        self._kill_at = None
        self._kill_hard_at = None
        self._start_future = None
        self._last_start_error = ""
        self._clear_start_retry()
        if not self.launch_stack:
            self._request_start()
            return
        pkg = str(self.get_parameter("launch_pkg").value)
        file = str(self.get_parameter("launch_file").value)
        args = build_launch_args(
            bool(self.get_parameter("dry_run").value),
            bool(self.get_parameter("start_head").value),
            bool(self.get_parameter("transit_watch").value),
            bool(self.get_parameter("speak_real").value),
            self.get_parameter("extra_args").value,
            bool(self.get_parameter("auto_start").value),
        )
        setups = [
            p.strip() for p in
            str(self.get_parameter("setup_scripts").value or "").split(":")
            if p.strip()
        ]
        cmd = build_launch_command(setups, pkg, file, args)
        launch_desc = shlex.join(["ros2", "launch", pkg, file] + args)

        log_path = time.strftime("/tmp/fire_inspection_button_%Y%m%d_%H%M%S.log")
        self._log_fp = open(log_path, "ab")  # noqa: SIM115 - 生命周期与子进程同步
        self._log_path = log_path
        self.get_logger().info(
            f"按键触发巡检：{launch_desc}"
            f"（先 source {len(setups)} 个环境脚本，日志 {log_path}）")
        try:
            self._proc = subprocess.Popen(
                cmd, stdout=self._log_fp, stderr=subprocess.STDOUT,
                env=os.environ.copy(), start_new_session=True)
        except OSError as exc:
            self.get_logger().error(f"启动巡检失败：{exc}")
            self._release()
            return
        self._started_at = time.monotonic()
        self._stop_at = None
        # 常驻编排层已运行时，launch 会跳过重复的编排层节点；稍等控制
        # 服务就绪后由这里补发 start（期间每 0.5s 重试）。
        if bool(self.get_parameter("auto_start").value):
            self._start_at = time.monotonic() + 1.0
            self._start_deadline = (
                self._start_at + self.control_ready_timeout)
        else:
            self._start_at = None
            self._start_deadline = None

    # ---------------- 状态 ----------------
    def _on_state(self, msg: String) -> None:
        state = (msg.data or "").strip()
        # 状态文本是「枚举[:阶段]」，如 running:inspect:POI_02 / aborted:run。
        # 终态判定必须只看冒号前的枚举：直接拿整串比较会把 aborted:run /
        # stopped:xxx 当成"仍在运行"，导致一次 abort 后 launch 永远不会自动
        # 关闭，下一次按 B 被当成"停止"（11-03 现场：按 B 三次才真正重启）。
        base = state.split(":", 1)[0].strip()
        self._state_text = state
        self._state_active = base in ("running", "paused")
        if not self.launch_stack:
            # 常驻模式没有自己的子进程，状态回到 idle/done/stopped/aborted
            # 时清掉终止超时，避免残留告警
            if not self._state_active:
                self._force_stop_at = None
                self._stop_deadline = None
                self._stop_future = None
            return
        if base in TERMINAL_STATES and self._running() and self.stop_on_done:
            if self._stop_at is None:
                self.get_logger().info(
                    f"巡检状态 {state}，{self.grace:.0f}s 后关闭本次 launch")
                self._stop_at = time.monotonic() + self.grace
            self._force_stop_at = None
        elif base not in TERMINAL_STATES:
            self._stop_at = None

    def _poll(self) -> None:
        # SIGINT 后 launch 迟迟不退：到点强杀整个进程组；SIGKILL 后仍不退，
        # 放弃等待并允许下一次按 B 重新拉起（否则父进程卡死会永远起不来）。
        if self._proc is not None:
            now = time.monotonic()
            if self._kill_hard_at is not None and now >= self._kill_hard_at:
                self.get_logger().error(
                    f"launch SIGKILL 后 {self.terminate_kill_grace:.0f}s 仍未退出，"
                    f"放弃等待并允许重新启动；请查看日志 {self._log_path}")
                self._release()
                if self._restart_pending:
                    self._restart_pending = False
                    self.get_logger().info("上一轮 launch 已被放弃，按之前的按键重新开始")
                    self._start()
                return
            if self._kill_at is not None and now >= self._kill_at:
                self.get_logger().warn(
                    f"launch SIGINT 后 {self.terminate_grace:.0f}s 未退出，发送 SIGKILL")
                self._force_kill()

        # 重新开始前补发的 stop：失败只记录，不在这里关 launch
        if (self._start_retry_stop_future is not None
                and self._start_retry_stop_future.done()):
            fut, self._start_retry_stop_future = (
                self._start_retry_stop_future, None)
            try:
                resp = fut.result()
            except Exception as exc:  # noqa: BLE001 - 停止失败不能中断重启流程
                self.get_logger().warn(f"重新开始前的 stop 请求失败（{exc}）")
            else:
                self.get_logger().info(
                    f"重新开始前的 stop {'已受理' if resp.success else '未受理'}："
                    f"{resp.message}")

        # 终止请求的响应
        if self._stop_future is not None and self._stop_future.done():
            fut, self._stop_future = self._stop_future, None
            try:
                resp = fut.result()
            except Exception as exc:  # noqa: BLE001 - 服务调用异常必须兜底
                if self.launch_stack:
                    self.get_logger().warn(
                        f"终止请求失败（{exc}），直接关闭 launch")
                    self._terminate()
                else:
                    self.get_logger().warn(
                        f"终止请求失败（{exc}），将在总超时前重试")
            else:
                if resp.success:
                    self.get_logger().info(
                        f"编排层已受理终止（当前 {resp.current_state}）：{resp.message}")
                    self._stop_deadline = None
                    self._force_stop_at = None
                elif self.launch_stack:
                    self.get_logger().warn(
                        f"编排层未受理终止：{resp.message}，直接关闭 launch")
                    self._terminate()
                else:
                    self.get_logger().warn(
                        f"编排层未受理终止：{resp.message}，将在总超时前重试")

        # 常驻模式：停止流程有总 deadline（从按键时刻起算）。必须先检查
        # deadline 再决定是否重试；Future 未完成时不重复发送
        if not self.launch_stack and self._stop_deadline is not None:
            now = time.monotonic()
            if not self._state_active:
                # 编排层已回到非运行状态：停止流程结束
                self._stop_deadline = None
                self._force_stop_at = None
                self._stop_future = None
            elif now >= self._stop_deadline:
                self._stop_deadline = None
                self._force_stop_at = None
                self._stop_future = None
                self.get_logger().warn(
                    f"停止请求在 {self.stop_confirm:.0f}s 内未完成，"
                    "已放弃本次停止流程；常驻编排层保持运行")
            elif self._stop_future is None:
                self._send_stop_request()

        # 主动 start：常驻编排层持有单实例锁时，launch 里的编排层起不来，
        # 只有把 start 发给常驻实例才会真正开始巡检。
        # 注意不能用 _running() 作为重试条件：常驻模式下 start 失败时状态
        # 还不会变成 running，那样就永远不会重试（现场表现为按 B 没反应）。
        if self._start_at is not None:
            now = time.monotonic()
            if self.launch_stack and not self._running():
                self.get_logger().warn("巡检 launch 已退出，放弃本次主动 start")
                self._start_at = None
                self._start_deadline = None
                self._clear_start_retry()
            elif not self.launch_stack and self._state_active:
                # 常驻模式状态已经是 running/paused，说明这轮已经在跑
                self._start_at = None
                self._start_deadline = None
                self._clear_start_retry()
            elif now >= self._start_at:
                if self._start_retry and self._state_active:
                    # start 被「正在进行中」拒绝：等 stop 生效、状态退出运行
                    if not self._start_retry_stop_sent:
                        self._send_restart_stop()
                    if now >= self._start_deadline:
                        self.get_logger().warn(
                            f"上一轮在 {self.start_retry_timeout:.0f}s 内仍未退出，"
                            "放弃本次 start；再按一次 B 可重新尝试")
                        self._start_at = None
                        self._start_deadline = None
                        self._clear_start_retry()
                    else:
                        self._start_at = now + 0.5
                elif self._ctl_cli.service_is_ready():
                    req = InspectionControl.Request()
                    req.cmd = "start"
                    req.arg = ""
                    req.operator_id = "rc_button"
                    self._start_future = self._ctl_cli.call_async(req)
                    self._start_at = None
                elif now >= self._start_deadline:
                    self.get_logger().warn(
                        f"控制服务 {self.control_ready_timeout:.0f}s 未就绪，"
                        "跳过主动 start（若 launch 内编排层已自启则不影响巡检）")
                    self._start_at = None
                    self._start_deadline = None
                    self._clear_start_retry()
                else:
                    self._start_at = now + 0.5
        if self._start_future is not None and self._start_future.done():
            fut, self._start_future = self._start_future, None
            try:
                resp = fut.result()
            except Exception as exc:  # noqa: BLE001 - 服务异常不能中断按键流程
                self.get_logger().warn(f"主动 start 请求失败（{exc}）")
                self._schedule_start_retry(time.monotonic(), "服务调用异常")
            else:
                if resp.success:
                    self.get_logger().info(
                        f"已通知编排层开始巡检：{resp.message}")
                    self._start_at = None
                    self._start_deadline = None
                    self._clear_start_retry()
                else:
                    # 常驻实例正在跑一轮时会返回「正在进行中」：先 stop 再重试
                    self.get_logger().warn(
                        f"主动 start 未执行：{resp.message}")
                    self._schedule_start_retry(time.monotonic(), resp.message)

        if self._running():
            now = time.monotonic()
            if self._force_stop_at is not None and now >= self._force_stop_at:
                if self.launch_stack:
                    self.get_logger().warn(
                        f"终止请求 {self.stop_confirm:.0f}s 内未确认，直接关闭 launch")
                    self._terminate()
                else:
                    self.get_logger().warn(
                        f"终止请求 {self.stop_confirm:.0f}s 内未确认，"
                        f"编排层状态仍为 {self._state_text or '未知'}；"
                        "不再重试，常驻编排层保持运行")
                    self._force_stop_at = None
                return
            if self._stop_at is not None and now >= self._stop_at:
                self.get_logger().info("关闭本次巡检 launch")
                self._terminate()
            return
        if self._proc is not None:
            rc = self._proc.returncode
            if rc not in (0, -2, -15):  # 非正常结束（-2/-15 = SIGINT/SIGTERM，按键终止属正常）
                self.get_logger().error(
                    f"巡检 launch 异常退出（rc={rc}），节点可能启动失败，"
                    f"请查看日志：{self._log_path}")
            else:
                self.get_logger().info(f"巡检 launch 已退出（rc={rc}）")
            self._release()
            if self._restart_pending:
                self._restart_pending = False
                self.get_logger().info("上一轮 launch 已关闭，按之前的按键重新开始")
                self._start()

    def _terminate(self) -> None:
        self._stop_at = None
        self._force_stop_at = None
        # 常驻模式没有自己的子进程：不能在这里终止 systemd 拉起的常驻编排层
        if not self.launch_stack or self._proc is None:
            return
        if not self._running():
            return
        # 已经安排过强杀就不再重置计时，避免反复按 B 无限延长等待
        if self._kill_at is not None or self._kill_hard_at is not None:
            return
        try:
            os.killpg(os.getpgid(self._proc.pid), signal.SIGINT)
        except OSError as exc:
            self.get_logger().warn(f"发送 SIGINT 失败：{exc}")
        self._kill_at = time.monotonic() + self.terminate_grace
        self.get_logger().warn(
            f"已向 launch 进程组发送 SIGINT，{self.terminate_grace:.0f}s "
            "仍未退出将发送 SIGKILL")

    def _force_kill(self) -> None:
        """SIGINT 宽限已过：SIGKILL 整个进程组，并再给一小段回收时间。"""
        if self._proc is None:
            return
        self._kill_at = None
        self._kill_hard_at = time.monotonic() + self.terminate_kill_grace
        try:
            os.killpg(os.getpgid(self._proc.pid), signal.SIGKILL)
        except OSError as exc:
            self.get_logger().warn(f"发送 SIGKILL 失败：{exc}")

    def _release(self) -> None:
        if self._log_fp is not None:
            self._log_fp.close()
            self._log_fp = None
        self._proc = None
        self._started_at = None
        self._stop_future = None
        self._force_stop_at = None
        self._stop_at = None
        self._start_future = None
        self._start_at = None
        self._start_deadline = None
        self._stop_deadline = None
        self._kill_at = None
        self._kill_hard_at = None
        self._start_retry = False
        self._start_retry_stop_sent = False
        self._start_retry_stop_future = None
        self._last_start_error = ""

    def destroy_node(self) -> bool:
        # 常驻模式没有子进程：不终止、不等待任何进程，避免误伤常驻链路
        if self.launch_stack:
            self._terminate()
            deadline = time.monotonic() + 10.0
            while self._running() and time.monotonic() < deadline:
                time.sleep(0.2)
            # spin 已停止，_poll 不会再执行，强杀兜底必须在这里自己走完
            if self._running():
                self.get_logger().warn("关闭节点：launch 10s 未退出，发送 SIGKILL")
                self._force_kill()
                end = time.monotonic() + self.terminate_kill_grace
                while self._running() and time.monotonic() < end:
                    time.sleep(0.2)
            self._release()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = ButtonTrigger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

if __name__ == "__main__":
    main()
