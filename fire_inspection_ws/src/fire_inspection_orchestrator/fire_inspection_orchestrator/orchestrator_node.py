"""消防巡检编排层 —— 导航 + 视觉 + 语音。

状态机：
    INIT -> SPEAK(start) -> TO_START
      -> 每个巡检点: NAV_TO_POI -> (头部停扫视 + 稳定等待) -> INSPECT -> SPEAK(结论)
      -> 每个过点点位: NAV_TO_POI
      -> [RETURN_TO_START] -> SPEAK(done) -> DONE
    任意导航失败 -> ABORTED

约定：
- 只有本节点调用 Action（单一指挥者），NAV / VISION / VOICE 之间互不调用；
- 导航失败默认终止本轮，不自动跳下一点；
- 视觉 UNCERTAIN / TIMEOUT / NO_FRAME / NO_MODEL / ERROR 原样记录，
  绝不落成"正常"；`simulated=true` 只记日志，不作为真实结论，且禁止播报；
- 播报文案统一由 fire_inspection_voice.speech_texts 生成（固定文本唯一来源），
  本文件不重复书写任何一句固定文案；
- 语音是旁路能力：服务端不可用、播报失败或超时都只告警，不阻断巡检；
- 运行状态只有 `_set_state()` 一个出口，状态变化时广播 T4
  `/fire_inspection/state`（`reliable` + `transient_local` / depth 1），
  外部（控制台 / 后续 S1 控制服务）只读这一份状态，不去猜日志；
- 控制面 S1 `/fire_inspection/control` 的六条指令都在本节点落地：
  `start/pause/resume/skip/stop/manual`。stop 与 pause 只在步骤之间生效
  （`_checkpoint`），Action 不会被半路打断，避免车停在半路或播到一半。
"""

from __future__ import annotations

import json
import os
import threading
import time
from enum import Enum
from types import SimpleNamespace

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from fire_inspection_interfaces.action import InspectPoi, NavigateTo, Speak
from fire_inspection_interfaces.srv import InspectionControl
from fire_inspection_voice.speech_texts import (
    SPEECH_TEXTS,
    PoiStatus,
    build_speech_text,
)

# InspectPoi.status 枚举（与 fire_inspection_interfaces 一致）
STATUS_CN = {
    0: "正常",
    1: "异常",
    2: "不确定",
    3: "超时",
    4: "无图像",
    5: "无模型",
    6: "错误",
}

# S1 skip 指令生效时写入结果，与视觉状态区分开（视觉永远不该返回"已跳过"）
SKIPPED_TEXT = "已跳过"


class RunState(Enum):
    """编排层对外状态（T4 /fire_inspection/state 的取值来源）。

    发布格式：`<state>` 或 `<state>:<phase>[:<poi>]`，全 ASCII，便于控制台解析。
        idle                  未开始 / 本轮已结束且无结果
        running:wait          等待各 Action 服务端就绪
        running:speak:start   播报开始语
        running:nav:<poi>     前往某点位
        running:settle:<poi>  头部停扫、回中并稳定等待
        running:inspect:<poi> 视觉检查某点位
        running:speak:<poi>   播报某点位结论
        running:return        沿返程轨道回起点
        running:speak:done    播报结束语
        running:manual:<poi>  手动点名检查某点位（不属于一轮巡检）
        paused:<phase>        已暂停，恢复后回到同一 phase
        done                  本轮正常走完
        aborted               本轮异常终止（配置 / 服务端 / 导航失败）
        stopped               本轮被人为中断（S1 stop 指令 / 关闭节点）
    """

    IDLE = "idle"
    RUNNING = "running"
    PAUSED = "paused"
    DONE = "done"
    ABORTED = "aborted"
    STOPPED = "stopped"


class RunAborted(Exception):
    """本轮巡检被 stop 指令 / 节点关闭提前结束（不是异常，状态置 stopped）。"""


# 契约 §8：T4 必须 reliable + transient_local / depth 1，
# 保证控制台后接入也能立即拿到当前状态
STATE_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class OrchestratorNode(Node):
    def __init__(self) -> None:
        super().__init__("fire_inspection_orchestrator")

        self.declare_parameter("script_file", "")
        self.declare_parameter("nav_config_file", "")
        self.declare_parameter("start_delay_s", 5.0)
        self.declare_parameter("nav_action", "/fire_inspection/navigate_to")
        self.declare_parameter("inspect_action", "/fire_inspection/inspect_poi")
        self.declare_parameter("speak_action", "/fire_inspection/speak")
        self.declare_parameter("speak_enabled", True)
        self.declare_parameter("speak_timeout_s", 40.0)
        self.declare_parameter("state_topic", "/fire_inspection/state")
        self.declare_parameter("control_service", "/fire_inspection/control")
        self.declare_parameter("head_cmd_topic", "/fire_inspection/head_sweep_cmd")
        self.declare_parameter("head_status_topic", "/fire_inspection/head_status")
        self.declare_parameter("head_stop_enabled", True)
        self.declare_parameter("head_stop_timeout_s", 4.0)
        self.declare_parameter("settle_s", 1.0)
        self.declare_parameter("inspect_timeout_s", 5.0)
        self.declare_parameter("wait_servers_s", 60.0)
        self.declare_parameter("nav_goal_timeout_s", 200.0)
        self.declare_parameter("nav_enabled", True)
        self.declare_parameter("confirm_frames", 3)
        self.declare_parameter("cooldown_s", 15.0)
        self.declare_parameter("lost_timeout_s", 2.0)
        self.declare_parameter("scan_classes", ["smoking", "fire_extinguisher"])
        self.declare_parameter("detection_topic", "/fire_inspection/detection")
        self.declare_parameter("transit_watch", True)

        self._shutdown = False
        self._goal_handle = None
        self._goal_lock = threading.Lock()
        self._worker = None
        self._results = {}
        self._state_text = ""
        self._phase = ""
        self._state_lock = threading.Lock()

        self._settle_s = float(self.get_parameter("settle_s").value)
        self._inspect_timeout_s = float(
            self.get_parameter("inspect_timeout_s").value
        )
        self._wait_servers_s = float(self.get_parameter("wait_servers_s").value)
        self._nav_goal_timeout_s = float(
            self.get_parameter("nav_goal_timeout_s").value
        )
        self._head_stop_enabled = bool(
            self.get_parameter("head_stop_enabled").value
        )
        self._head_stop_timeout_s = float(
            self.get_parameter("head_stop_timeout_s").value
        )
        self._head_state = None
        self._head_lock = threading.Lock()

        self._speak_enabled = bool(self.get_parameter("speak_enabled").value)
        self._speak_timeout_s = float(self.get_parameter("speak_timeout_s").value)

        # WATCH 模式参数
        self._nav_enabled = bool(self.get_parameter("nav_enabled").value)
        self._confirm_frames = int(self.get_parameter("confirm_frames").value)
        self._cooldown_s = float(self.get_parameter("cooldown_s").value)
        self._lost_timeout_s = float(self.get_parameter("lost_timeout_s").value)
        self._scan_classes = list(self.get_parameter("scan_classes").value)
        self._transit_watch = bool(self.get_parameter("transit_watch").value)

        # WATCH 模式状态
        self._watch_state = "WATCH"       # WATCH / LOCKED / SPEAK / COOLDOWN
        self._watch_streak_cls = None
        self._watch_streak = 0
        self._watch_locked_pose = None     # {"yaw": float, "pitch": float}
        self._watch_lost_at = None
        self._watch_last_trigger = 0.0
        self._watch_lock = threading.Lock()
        # 导航模式下的移动路段检测门控：True 时 _on_detection 才处理检测流
        self._watch_active = False

        # detection 订阅（WATCH 模式驱动源）
        from fire_inspection_interfaces.msg import PoiResult
        self._detection_sub = self.create_subscription(
            PoiResult,
            str(self.get_parameter("detection_topic").value),
            self._on_detection,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT),
        )

        # 控制面（S1）：stop / pause 通过这两个开关打断 worker，
        # _busy 同时充当"本轮是否在跑"的唯一判据，start 与 manual 互斥
        self._stop_event = threading.Event()
        self._paused = threading.Event()
        self._busy = threading.Lock()
        self._skip_lock = threading.Lock()
        self._skip_pois = set()

        self._config_error = None
        self._cfg = {}
        try:
            self._cfg = self._load_config()
        except Exception as exc:  # noqa: BLE001 - 配置错误必须显式暴露
            self._config_error = str(exc)

        self._nav_client = ActionClient(
            self, NavigateTo, str(self.get_parameter("nav_action").value)
        )
        self._inspect_client = ActionClient(
            self, InspectPoi, str(self.get_parameter("inspect_action").value)
        )
        self._speak_client = ActionClient(
            self, Speak, str(self.get_parameter("speak_action").value)
        )
        self._head_pub = self.create_publisher(
            String, str(self.get_parameter("head_cmd_topic").value), 10
        )
        self.create_subscription(
            String,
            str(self.get_parameter("head_status_topic").value),
            self._on_head_status,
            10,
        )
        self._state_pub = self.create_publisher(
            String, str(self.get_parameter("state_topic").value), STATE_QOS
        )
        self._control_srv = self.create_service(
            InspectionControl,
            str(self.get_parameter("control_service").value),
            self._on_control,
        )
        # 先发一次 idle，后接入的控制台也能立刻拿到当前状态（transient_local）
        self._set_state(RunState.IDLE)

        delay = max(0.0, float(self.get_parameter("start_delay_s").value))
        self._start_timer = self.create_timer(delay, self._start_once)

        if self._config_error is not None:
            self.get_logger().error(f"配置加载失败：{self._config_error}")
        else:
            self.get_logger().info(
                f"编排层就绪：起点={self._cfg['start_poi']}，"
                f"巡检点={self._cfg['inspect_pois']}，"
                f"过点={self._cfg['transit_pois']}，"
                f"返程={self._cfg['return_to_start']}，"
                f"播报={'开' if self._speak_enabled else '关'}，"
                f"start_delay={delay:.1f}s"
            )

    # ------------------------------------------------------------------
    # 配置
    # ------------------------------------------------------------------
    def _load_config(self) -> dict:
        path = str(self.get_parameter("script_file").value or "")
        if not path:
            path = os.path.join(
                get_package_share_directory("fire_inspection_orchestrator"),
                "config",
                "inspection_script.yaml",
            )
        with open(path, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f) or {}
        if not isinstance(cfg, dict):
            raise RuntimeError(f"配置必须是映射: {path}")

        start_poi = str(cfg.get("start_poi") or "").strip()
        if not start_poi:
            raise RuntimeError("start_poi 不能为空")

        inspect_pois = []
        inspect_cfg = {}
        for item in cfg.get("pois") or []:
            if not isinstance(item, dict):
                raise RuntimeError(f"pois 条目必须是映射: {item}")
            name = str(item.get("name") or "").strip()
            if not name:
                raise RuntimeError("pois 存在空点位名")
            if name == start_poi:
                raise RuntimeError("pois 不能包含 start_poi")
            if name in inspect_cfg:
                raise RuntimeError(f"pois 重复点位: {name}")
            expect = item.get("expect_classes") or []
            abnormal = item.get("abnormal_classes") or []
            focus = item.get("focus_classes") or []
            if not isinstance(expect, list) or not isinstance(abnormal, list):
                raise RuntimeError(f"{name} 的 expect/abnormal 必须是列表")
            if not isinstance(focus, list):
                raise RuntimeError(f"{name} 的 focus_classes 必须是列表")
            inspect_pois.append(name)
            inspect_cfg[name] = {
                "expect_classes": [str(c) for c in expect],
                "abnormal_classes": [str(c) for c in abnormal],
                "focus_classes": tuple(str(c) for c in focus),
            }

        # 全局阈值映射（与 vision.yaml 单一来源）
        conf_by_class_raw = cfg.get("conf_by_class") or {}
        if not isinstance(conf_by_class_raw, dict):
            raise RuntimeError("conf_by_class 必须是映射")
        conf_by_class = {}
        for cls, thr in conf_by_class_raw.items():
            try:
                conf_by_class[str(cls)] = float(thr)
            except (TypeError, ValueError):
                raise RuntimeError(f"conf_by_class.{cls} 无法转为 float: {thr!r}")

        transit_pois = [str(p) for p in (cfg.get("transit_pois") or [])]
        if len(set(transit_pois)) != len(transit_pois):
            raise RuntimeError(f"transit_pois 存在重复项: {transit_pois}")
        for name in transit_pois:
            if name == start_poi:
                raise RuntimeError("transit_pois 不能包含 start_poi")
            if name in inspect_cfg:
                raise RuntimeError(f"点位 {name} 不能同时是巡检点和过点点位")

        settle_s = float(cfg.get("settle_s", self._settle_s))
        if settle_s < 0:
            raise RuntimeError(f"settle_s 非法: {settle_s}")
        self._settle_s = settle_s

        inspect_timeout_s = float(
            cfg.get("inspect_timeout_s", self._inspect_timeout_s)
        )
        if inspect_timeout_s <= 0:
            raise RuntimeError(f"inspect_timeout_s 非法: {inspect_timeout_s}")
        self._inspect_timeout_s = inspect_timeout_s

        result = {
            "path": path,
            "start_poi": start_poi,
            "inspect_pois": inspect_pois,
            "transit_pois": transit_pois,
            "inspect_cfg": inspect_cfg,
            "return_to_start": bool(cfg.get("return_to_start", False)),
            "conf_by_class": conf_by_class,
        }
        self._validate_against_nav(result)
        return result

    def _validate_against_nav(self, cfg: dict) -> None:
        """校验编排层点位顺序与导航配置一致，避免跑起来才被拒绝。"""
        configured = str(self.get_parameter("nav_config_file").value or "")
        try:
            path = configured or os.path.join(
                get_package_share_directory("fire_inspection_nav"),
                "config",
                "pois.yaml",
            )
            with open(path, "r", encoding="utf-8") as f:
                nav_cfg = yaml.safe_load(f) or {}
        except OSError as exc:
            self.get_logger().warn(f"无法读取导航配置，跳过一致性校验：{exc}")
            return
        if not isinstance(nav_cfg, dict):
            raise RuntimeError(f"导航配置必须是映射: {path}")

        nav_start = str(nav_cfg.get("start_poi") or "").strip()
        if nav_start != cfg["start_poi"]:
            raise RuntimeError(
                f"起点不一致：编排层={cfg['start_poi']}，导航={nav_start}")

        nav_order = [str(name) for name in (nav_cfg.get("poi_order") or [])]
        expected = cfg["inspect_pois"] + cfg["transit_pois"]
        if nav_order != expected:
            raise RuntimeError(
                f"点位顺序不一致：编排层={expected}，导航={nav_order}")

        nav_transit = {str(name) for name in (nav_cfg.get("transit_pois") or [])}
        if nav_transit != set(cfg["transit_pois"]):
            raise RuntimeError(
                f"过点点位不一致：编排层={sorted(cfg['transit_pois'])}，"
                f"导航={sorted(nav_transit)}")
        self.get_logger().info(
            f"点位配置校验通过：{expected}，过点={sorted(nav_transit)}")

    # ------------------------------------------------------------------
    # 启动
    # ------------------------------------------------------------------
    def _start_once(self) -> None:
        self._start_timer.cancel()
        if self._config_error is not None:
            self.get_logger().error("配置不可用，本轮不执行")
            self._set_state(RunState.ABORTED, "config")
            return
        ok, msg = self._begin_round("start_delay")
        if not ok:
            self.get_logger().warn(f"定时启动失败：{msg}")

    def _begin_round(self, source: str):
        """占用 _busy 并起 worker 跑一轮；返回 (是否成功, 说明)。"""
        if self._config_error is not None:
            return False, f"配置不可用：{self._config_error}"
        if not self._busy.acquire(blocking=False):
            return False, "本轮巡检正在进行中，请先 stop 或等待结束"
        self._stop_event.clear()
        self._paused.clear()
        with self._skip_lock:
            self._skip_pois.clear()
        self._results.clear()
        self._worker = threading.Thread(
            target=self._run, name=f"inspection-{source}", daemon=True
        )
        self._worker.start()
        return True, "已开始本轮巡检"

    # ------------------------------------------------------------------
    # 状态
    # ------------------------------------------------------------------
    def _set_state(self, state: RunState, detail: str = "") -> None:
        """唯一的状态出口：切换状态并广播 T4（同一状态不重复发布）。"""
        text = f"{state.value}:{detail}" if detail else state.value
        with self._state_lock:
            if text == self._state_text:
                return
            self._state_text = text
            if state is RunState.RUNNING:
                self._phase = detail
        try:
            self._state_pub.publish(String(data=text))
        except Exception:  # noqa: BLE001
            # 节点已销毁 / SIGINT 后 context 已关闭：发不出去也不该从工作线程抛出
            return
        self.get_logger().debug(f"状态 -> {text}")

    def _snapshot(self):
        """一次取回 (状态文本, running 阶段的 detail)，避免两次加锁不一致。"""
        with self._state_lock:
            return self._state_text, self._phase

    def _current_poi(self) -> str:
        """从当前 phase 里取出点位名，取出不到返回空串。"""
        _, phase = self._snapshot()
        tail = phase.split(":")[-1]
        if tail in self._cfg.get("inspect_pois", []):
            return tail
        return ""

    def _is_skipped(self, poi: str) -> bool:
        with self._skip_lock:
            return poi in self._skip_pois

    def _checkpoint(self) -> None:
        """步骤之间的可中断 / 可暂停点。

        stop 与 close 立即抛 RunAborted；pause 在此阻塞，直到 resume。
        只在两个步骤之间生效——导航、视觉、播报这类 Action 不会被半路打断，
        避免车停在半路或播到一半。
        """
        if self._shutdown:
            raise RunAborted("节点关闭")
        if self._stop_event.is_set():
            raise RunAborted("收到 stop 指令")
        if not self._paused.is_set():
            return
        _, phase = self._snapshot()
        self._set_state(RunState.PAUSED, phase)
        self.get_logger().info(f"巡检已暂停（停在 {phase or '空闲'}），等待 resume")
        while self._paused.is_set():
            if self._shutdown:
                raise RunAborted("暂停期间节点关闭")
            if self._stop_event.is_set():
                raise RunAborted("暂停期间收到 stop 指令")
            self._paused.wait(timeout=0.1)
        self.get_logger().info("巡检已恢复")
        self._set_state(RunState.RUNNING, phase)

    # ------------------------------------------------------------------
    # 控制面（S1 /fire_inspection/control）
    # ------------------------------------------------------------------
    def _on_control(self, request, response):
        cmd = str(request.cmd).strip().lower()
        arg = str(request.arg).strip()
        operator = str(request.operator_id).strip()
        handlers = {
            "start": self._cmd_start,
            "pause": self._cmd_pause,
            "resume": self._cmd_resume,
            "skip": lambda: self._cmd_skip(arg),
            "stop": self._cmd_stop,
            "manual": lambda: self._cmd_manual(arg),
        }
        handler = handlers.get(cmd)
        if handler is None:
            ok, msg = False, (
                f"未知指令「{cmd or '(空)'}」，"
                "允许 start/pause/resume/skip/stop/manual")
        else:
            ok, msg = handler()
        text = f"S1 指令 {cmd or '(空)'}"
        if arg:
            text += f" arg={arg}"
        if operator:
            text += f" 操作人={operator}"
        text += f"：{msg}"
        if ok:
            self.get_logger().info(text)
        else:
            self.get_logger().warn(text)
        response.success = bool(ok)
        response.message = msg
        response.current_state = self._snapshot()[0]
        return response

    def _cmd_start(self):
        return self._begin_round("control")

    def _cmd_pause(self):
        if not self._busy.locked():
            return False, "当前没有正在进行的巡检"
        if self._paused.is_set():
            return False, "已处于暂停状态"
        self._paused.set()
        _, phase = self._snapshot()
        self._set_state(RunState.PAUSED, phase)
        return True, f"已请求暂停，将在当前步骤（{phase or '空闲'}）结束后停住"

    def _cmd_resume(self):
        if not self._paused.is_set():
            return False, "当前未处于暂停状态"
        self._paused.clear()
        _, phase = self._snapshot()
        self._set_state(RunState.RUNNING, phase)
        return True, "已恢复巡检"

    def _cmd_stop(self):
        if not self._busy.locked():
            return False, "当前没有正在进行的巡检"
        self._stop_event.set()
        # 解除暂停闸门，避免 worker 还阻塞在 _checkpoint 里等 resume
        self._paused.clear()
        self._cancel_current_goal()
        return True, "已请求终止本轮巡检，当前动作已取消"

    def _cmd_skip(self, arg: str):
        if not self._busy.locked():
            return False, "当前没有正在进行的巡检"
        inspect_pois = self._cfg.get("inspect_pois", [])
        if arg and arg not in inspect_pois:
            return False, f"点位「{arg}」不在本轮巡检范围内"
        target = arg or self._current_poi()
        if not target:
            return False, "当前不在巡检点位上，请用 arg 指定要跳过的点位"
        with self._skip_lock:
            self._skip_pois.add(target)
        _, phase = self._snapshot()
        if phase == f"inspect:{target}":
            self._cancel_current_goal()
            return True, f"已取消 {target} 的视觉检查，按{SKIPPED_TEXT}记录"
        return True, f"{target} 将在到达后跳过视觉检查"

    def _cmd_manual(self, arg: str):
        if self._config_error is not None:
            return False, f"配置不可用：{self._config_error}"
        inspect_pois = self._cfg.get("inspect_pois", [])
        if arg not in inspect_pois:
            return False, (
                f"点位「{arg or '(空)'}」不可手动检查，"
                f"合法值：{inspect_pois}")
        if not self._busy.acquire(blocking=False):
            return False, "本轮巡检正在进行中，禁止 manual"
        self._stop_event.clear()
        self._paused.clear()
        self._worker = threading.Thread(
            target=self._run_manual, args=(arg,), name=f"manual-{arg}", daemon=True
        )
        self._worker.start()
        return True, f"已开始手动检查 {arg}"

    # ------------------------------------------------------------------
    # 主流程
    # ------------------------------------------------------------------
    def _run(self) -> None:
        if not self._nav_enabled:
            # WATCH 模式：不导航，直接订阅 detection 等触发
            self.get_logger().info("=== WATCH 模式启动（nav_enabled=false）===")
            self.get_logger().info(
                f"确认帧数={self._confirm_frames} 冷却={self._cooldown_s}s "
                f"丢失超时={self._lost_timeout_s}s 关注={self._scan_classes}"
            )
            self._set_state(RunState.RUNNING, "watch")
            self._speak("消防巡检开始，请注意现场安全。", "开始")
            self._send_head_cmd("start")
            # 阻塞等 stop，期间 _on_detection 回调驱动状态机
            try:
                while not self._stop_event.is_set():
                    time.sleep(0.5)
            except Exception as exc:
                self.get_logger().error(f"WATCH 模式异常：{exc}")
            finally:
                self._send_head_cmd("stop")
                self._set_state(RunState.DONE)
            return

        # 原有导航巡检流程
        try:
            self._set_state(RunState.RUNNING, "wait")
            self._wait_servers()
            self._checkpoint()
            self._set_state(RunState.RUNNING, "speak:start")
            self._speak(SPEECH_TEXTS["start"], "开始")
            self._transit_watch_on()
            self._navigate(self._cfg["start_poi"], is_return=False)

            for poi in self._cfg["inspect_pois"]:
                self._checkpoint()
                self._navigate(poi, is_return=False)
                self._set_state(RunState.RUNNING, f"settle:{poi}")
                self._transit_watch_off()
                self._stop_head()
                self._sleep(self._settle_s)
                self._checkpoint()
                self._inspect(poi)
                self._transit_watch_on()

            for poi in self._cfg["transit_pois"]:
                self._checkpoint()
                self._navigate(poi, is_return=False)

            self._checkpoint()
            if self._cfg["return_to_start"]:
                self._navigate(self._cfg["start_poi"], is_return=True)

            self._set_state(RunState.RUNNING, "speak:done")
            self._speak(SPEECH_TEXTS["done"], "结束")
            self._set_state(RunState.DONE)
            self.get_logger().info(
                "巡检流程结束：" + self._summary())
        except RunAborted as exc:
            self.get_logger().info(f"本轮巡检提前结束：{exc}")
            self._set_state(RunState.STOPPED)
            self._cancel_current_goal()
        except Exception as exc:  # noqa: BLE001 - 必须显式终止
            self.get_logger().error(f"巡检终止：{exc}")
            self._set_state(RunState.ABORTED, "run")
            self._cancel_current_goal()
        finally:
            # 统一收尾：关检测门控 + 头部停扫回中（done/aborted/stopped 都走这里）
            self._transit_watch_off()
            if self._nav_enabled and self._transit_watch:
                self._send_head_cmd("stop")
            self._release_busy()

    def _run_manual(self, poi: str) -> None:
        """manual：只对一个点位做 导航 → 停头 → 视觉 → 播报。"""
        try:
            self._set_state(RunState.RUNNING, f"manual:{poi}")
            self.get_logger().info(f"手动点名检查：{poi}")
            self._wait_servers()
            self._navigate(poi, is_return=False)
            self._set_state(RunState.RUNNING, f"settle:{poi}")
            self._stop_head()
            self._sleep(self._settle_s)
            self._inspect(poi)
            self._set_state(RunState.IDLE)
            self.get_logger().info(f"手动检查结束：{poi}")
        except RunAborted as exc:
            self.get_logger().info(f"手动检查提前结束：{exc}")
            self._set_state(RunState.STOPPED)
            self._cancel_current_goal()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"手动检查失败：{exc}")
            self._set_state(RunState.ABORTED, "manual")
            self._cancel_current_goal()
        finally:
            self._release_busy()

    def _release_busy(self) -> None:
        try:
            self._busy.release()
        except RuntimeError:
            pass

    # ------------------------------------------------------------------
    # WATCH 模式：检测回调驱动的"首次命中即锁定→三次确认→播报→恢复扫视"
    # ------------------------------------------------------------------
    def _send_head_cmd(self, cmd: str) -> None:
        """向 /fire_inspection/head_sweep_cmd 发 String 指令（start/stop/pause/resume/center/look_at:yaw,pitch）。"""
        try:
            pub = getattr(self, "_head_cmd_pub", None)
            if pub is None:
                # 延迟创建，避免 __init__ 里顺序问题
                pub = self.create_publisher(String,
                    str(self.get_parameter("head_cmd_topic").value), 10)
                self._head_cmd_pub = pub
            pub.publish(String(data=cmd))
            self.get_logger().info(f"head_cmd -> {cmd}")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"head_cmd 发送失败 {cmd}: {exc}")

    def _transit_watch_on(self) -> None:
        """移动路段开启持续巡检：打开检测门控 + 头部恢复扫视。"""
        if not self._transit_watch:
            return
        with self._watch_lock:
            self._watch_active = True
        self._send_head_cmd("start")
        self.get_logger().info("移动路段持续巡检开启：头部恢复扫视")

    def _transit_watch_off(self) -> None:
        """到点关闭持续巡检：丢弃未确认触发；进行中的播报等其自然收尾。"""
        if not self._transit_watch:
            return
        with self._watch_lock:
            self._watch_active = False
            if self._watch_state == "LOCKED":
                # 未确认的触发直接丢弃，不发 resume（头部控制权移交定点流程）
                self._watch_state = "WATCH"
                self._watch_streak_cls = None
                self._watch_streak = 0
                self._watch_lost_at = None
            speaking = self._watch_state == "SPEAK"
        if speaking:
            # 等移动路段播报自然结束，避免与定点结论播报争抢 Speak Action
            self.get_logger().info("移动路段播报进行中，等待其结束后再定点检查")
            deadline = time.monotonic() + self._speak_timeout_s
            while time.monotonic() < deadline:
                with self._watch_lock:
                    if self._watch_state != "SPEAK":
                        break
                if self._shutdown or self._stop_event.is_set():
                    break
                time.sleep(0.1)

    def _parse_look_at(self, detections_json: str):
        """从 detections_json 里取最高 conf 的目标中心像素，映射到相机 FOV 估算 yaw/pitch。"""
        try:
            arr = json.loads(detections_json)
        except Exception:
            return None
        if not isinstance(arr, list) or not arr:
            return None
        # 按 conf 降序
        arr.sort(key=lambda d: d.get("conf", 0.0), reverse=True)
        top = arr[0]
        cx = float(top.get("cx", 0.5))   # 归一化 0~1
        cy = float(top.get("cy", 0.5))
        # 相机 1280x720，FOV 水平 60° 垂直 34°，yaw 右正 pitch 上正
        yaw = (cx - 0.5) * (60.0 * 3.14159 / 180.0)   # rad
        pitch = -(cy - 0.5) * (34.0 * 3.14159 / 180.0)
        # 限位
        yaw = max(-1.5708, min(1.5708, yaw))
        pitch = max(-0.4363, min(0.4363, pitch))
        return {"yaw": round(yaw, 4), "pitch": round(pitch, 4), "cls": top.get("cls", "")}

    def _on_detection(self, msg) -> None:
        """detection 话题回调——WATCH 模式核心状态机。"""
        with self._watch_lock:
            if self._nav_enabled and not self._watch_active:
                return  # 导航模式下只在移动路段处理检测流
            if getattr(msg, "simulated", False):
                return  # 模拟结果一律忽略，不作为真实触发

            cls = None
            conf = 0.0
            try:
                arr = json.loads(msg.detections_json)
                if isinstance(arr, list) and arr:
                    arr.sort(key=lambda d: d.get("conf", 0.0), reverse=True)
                    top = arr[0]
                    c = top.get("cls", "")
                    if c in self._scan_classes:
                        cls = c
                        conf = float(top.get("conf", 0.0))
            except Exception:
                pass

            now = time.time()

            # 冷却期：跳过直到超时
            if self._watch_state == "COOLDOWN":
                if now - self._watch_last_trigger < self._cooldown_s:
                    return
                self._watch_state = "WATCH"
                self.get_logger().info("冷却结束，恢复扫视")
                return

            if self._watch_state == "SPEAK":
                return  # 播报期间跳过

            if self._watch_state == "WATCH":
                if cls is not None:
                    # 首次命中 → 锁定头部（直接发 look_at，它自带锁定效果，不用先 pause）
                    pose = self._parse_look_at(msg.detections_json)
                    yaw = pose["yaw"] if pose else 0.0
                    pitch = pose["pitch"] if pose else 0.0
                    self._watch_state = "LOCKED"
                    self._watch_streak_cls = cls
                    self._watch_streak = 1
                    self._watch_locked_pose = {"yaw": yaw, "pitch": pitch}
                    self._watch_lost_at = None
                    self.get_logger().warn(
                        f"首次命中 {cls} conf={conf:.2f} 头部锁定 "
                        f"yaw={yaw:.3f} pitch={pitch:.3f} rad 开始 1/{self._confirm_frames} 确认"
                    )
                    # 只发 look_at（自带锁定，不用先 pause）
                    self._send_head_cmd(f"look_at:{yaw:.4f},{pitch:.4f}")
                return

            if self._watch_state == "LOCKED":
                if cls == self._watch_streak_cls:
                    # 同类 → 累加
                    self._watch_streak += 1
                    self._watch_lost_at = None
                    if self._watch_streak >= self._confirm_frames:
                        # 三次确认 → 播报
                        self._watch_state = "SPEAK"
                        self._watch_last_trigger = now
                        self.get_logger().warn(
                            f"{cls} 三次确认完成 → 播报"
                        )
                        # 异步播报（别阻塞回调）
                        threading.Thread(
                            target=self._watch_speak_and_resume,
                            args=(cls,),
                            daemon=True,
                        ).start()
                    else:
                        self.get_logger().info(
                            f"目标 {cls} 连续 {self._watch_streak}/{self._confirm_frames} 帧 conf={conf:.2f}"
                        )
                else:
                    # 目标丢失或换类
                    if self._watch_lost_at is None:
                        self._watch_lost_at = now
                        self.get_logger().info(
                            f"目标 {self._watch_streak_cls} 丢失，已锁定 "
                            f"{self._watch_streak}/{self._confirm_frames} 帧，等待复现（超时 {self._lost_timeout_s}s）"
                        )
                    elif now - self._watch_lost_at > self._lost_timeout_s:
                        # 超时 → 放弃，恢复扫视
                        self._watch_state = "WATCH"
                        self._watch_streak_cls = None
                        self._watch_streak = 0
                        self._watch_lost_at = None
                        self.get_logger().warn(
                            f"目标丢失超过 {self._lost_timeout_s}s → 放弃本次触发，恢复扫视"
                        )
                        self._send_head_cmd("resume")

    def _watch_speak_and_resume(self, cls: str) -> None:
        """异步播报 + 播报完成后恢复扫视。"""
        spoke_ok = False
        try:
            goal = Speak.Goal()
            if cls == "smoking":
                goal.text = SPEECH_TEXTS["smoking"]
            elif cls == "fire_extinguisher":
                goal.text = SPEECH_TEXTS["extinguisher_ok"]
            else:
                goal.text = f"发现目标：{cls}"
            goal.interrupt = True
            self.get_logger().info(f"播报[{cls}]：{goal.text}")
            if self._speak_enabled:
                try:
                    result = self._call(
                        self._speak_client, goal,
                        self._speak_timeout_s, f"播报 {cls}"
                    )
                    spoke_ok = True
                    self.get_logger().info(f"播报完成 success={result.success}")
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().warn(f"播报失败（头部继续锁定 3s 再恢复）：{exc}")
                    # 播报失败也别马上 resume，让头部多锁一会让用户看清
                    time.sleep(3.0)
            else:
                self.get_logger().info("播报已关闭（speak_enabled=false），头部锁定 2s 后恢复")
                time.sleep(2.0)
        finally:
            # 只有播报成功或等够时间才恢复扫视
            with self._watch_lock:
                self._watch_state = "COOLDOWN"
                self._watch_streak_cls = None
                self._watch_streak = 0
                active = not self._nav_enabled or self._watch_active
            if active:
                self._send_head_cmd("resume")
                self.get_logger().info("恢复扫视，继续等待目标")
            else:
                self.get_logger().info("检测门控已关闭（已到点定检），本次不再恢复扫视")

    def _wait_servers(self) -> None:
        if self._nav_enabled:
            if not self._nav_client.wait_for_server(timeout_sec=self._wait_servers_s):
                raise RuntimeError("导航 Action 服务端不可用")
        else:
            self.get_logger().info("nav_enabled=false，跳过导航 Action 服务端等待")
        if not self._inspect_client.wait_for_server(
            timeout_sec=self._wait_servers_s
        ):
            raise RuntimeError("视觉 Action 服务端不可用")
        if not self._speak_enabled:
            self.get_logger().info("播报已关闭（speak_enabled=false）")
            return
        # 语音是旁路能力：服务端缺失只关掉播报，不让整轮巡检起不来
        if not self._speak_client.wait_for_server(
            timeout_sec=self._wait_servers_s
        ):
            self._speak_enabled = False
            self.get_logger().warn("语音 Action 服务端不可用，本轮只记录日志、不播报")

    def _navigate(self, poi: str, is_return: bool) -> None:
        goal = NavigateTo.Goal()
        goal.poi = poi
        goal.timeout_s = 0.0
        goal.is_return = bool(is_return)
        action = "返回起点" if is_return else "前往"
        self._set_state(
            RunState.RUNNING, "return" if is_return else f"nav:{poi}"
        )
        self.get_logger().info(f"{action}：{poi}")
        result = self._call(
            self._nav_client, goal, self._nav_goal_timeout_s, f"{action} {poi}"
        )
        if not result.success or result.phase != "arrived":
            raise RuntimeError(
                f"{action} {poi} 失败：phase={result.phase}，{result.message}")
        self.get_logger().info(f"已到达：{poi}")

    def _inspect(self, poi: str) -> None:
        if self._is_skipped(poi):
            self._results[poi] = SKIPPED_TEXT
            self.get_logger().info(f"{poi} 已按 skip 指令跳过视觉检查")
            return
        cfg = self._cfg["inspect_cfg"][poi]
        goal = InspectPoi.Goal()
        goal.poi = poi
        goal.expect_classes = list(cfg["expect_classes"])
        goal.abnormal_classes = list(cfg["abnormal_classes"])
        goal.timeout_s = self._inspect_timeout_s
        timeout = self._inspect_timeout_s + 20.0
        self._set_state(RunState.RUNNING, f"inspect:{poi}")
        try:
            result = self._call(
                self._inspect_client, goal, timeout, f"视觉检查 {poi}"
            )
        except RunAborted:
            raise
        except Exception as exc:  # noqa: BLE001 - 视觉失败不得阻断整条流程
            self.get_logger().error(f"{poi} 视觉调用失败：{exc}")
            if not self._shutdown:
                # 调用失败≠判定异常：按"不确定"播报，引导人工复核
                self._set_state(RunState.RUNNING, f"speak:{poi}")
                self._speak(self._build_text(poi, PoiStatus.UNCERTAIN, []), poi)
            return

        # skip 在推理期间到达：以指令为准，不解释这份可能已被取消的结果
        if self._is_skipped(poi):
            self._results[poi] = SKIPPED_TEXT
            self.get_logger().warn(f"{poi} 视觉检查被 skip 指令取消，按跳过记录")
            return

        status_text = STATUS_CN.get(int(result.status), f"未知({result.status})")
        self._results[poi] = status_text
        if result.simulated:
            self.get_logger().info(
                f"{poi} 视觉结果：{status_text} [simulated] {result.detail} "
                f"—— 模拟结果，不作为真实结论，不播报")
            return
        self.get_logger().info(
            f"{poi} 视觉结果：{status_text} | {result.detail} | "
            f"detections={result.detections_json}")
        self._set_state(RunState.RUNNING, f"speak:{poi}")
        self._speak(
            self._build_text(poi, int(result.status), self._parse_detections(result)),
            poi,
        )

    # ------------------------------------------------------------------
    # 语音
    # ------------------------------------------------------------------
    def _build_text(self, poi: str, status: int, detections: list) -> str:
        """把视觉结论转成播报文本；文案规则全部来自 speech_texts（唯一来源）。"""
        cfg = self._cfg["inspect_cfg"][poi]
        payload = SimpleNamespace(status=int(status), detections=detections)
        return build_speech_text(
            poi,
            payload,
            expect_classes=cfg["expect_classes"],
            abnormal_classes=cfg["abnormal_classes"],
            # detections_json 已由视觉按类别阈值过滤，这里不再二次过滤，
            # 保证播报口径与判定结论一致。
            min_conf=0.0,
            # conf_by_class 与视觉层 vision.yaml 单一来源，
            # 传递给文案层作为统一阈值来源，避免未来视觉未过滤时
            # 文案层用默认 0.5 阈值造成口径不一致。
            conf_by_class=self._cfg.get("conf_by_class"),
            # focus_classes：当前点位显式指定的专用检查目标。
            # 空 tuple 时沿用通用文案，绝不按 POI 名称猜测专用目标。
            focus_classes=cfg.get("focus_classes", ()),
        )

    def _parse_detections(self, result) -> list:
        """按契约 §6.1 解析 detections_json，只取 cls / conf 两键。"""
        raw = str(result.detections_json or "").strip()
        if not raw:
            return []
        try:
            items = json.loads(raw)
        except ValueError as exc:
            self.get_logger().warn(f"{result.poi} detections_json 解析失败：{exc}")
            return []
        if not isinstance(items, list):
            self.get_logger().warn(f"{result.poi} detections_json 不是数组，忽略")
            return []
        detections = []
        for item in items:
            if not isinstance(item, dict):
                continue
            cls = item.get("cls")
            conf = item.get("conf")
            if cls is None or conf is None:
                continue
            detections.append(
                {"class_name": str(cls), "confidence": float(conf)}
            )
        return detections

    def _speak(self, text: str, label: str) -> None:
        """串行播报一条文本；失败 / 超时只告警，绝不阻断巡检。"""
        if not self._speak_enabled or not text:
            return
        goal = Speak.Goal()
        goal.text = text
        # 逐条串行播报：interrupt=true 清掉上一条可能残留的音频，避免排队叠加
        goal.interrupt = True
        self.get_logger().info(f"播报[{label}]：{text}")
        try:
            result = self._call(
                self._speak_client, goal, self._speak_timeout_s, f"播报 {label}"
            )
        except RunAborted:
            raise
        except Exception as exc:  # noqa: BLE001 - 语音不得阻断巡检
            self.get_logger().warn(f"播报失败（不阻断巡检）：{exc}")
            return
        if not result.success:
            self.get_logger().warn(
                f"播报未确认完成（不阻断巡检）：state={result.state}，"
                f"{result.message}")

    # ------------------------------------------------------------------
    # Action 调用
    # ------------------------------------------------------------------
    def _call(self, client: ActionClient, goal, timeout_s: float, label: str):
        send_future = client.send_goal_async(goal)
        if not self._wait_future(send_future, timeout_s):
            if self._stop_event.is_set():
                raise RunAborted(f"{label} 收到 stop 指令")
            raise TimeoutError(f"{label} 发送超时")
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f"{label} 被拒绝")

        with self._goal_lock:
            self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        try:
            if not self._wait_future(result_future, timeout_s):
                goal_handle.cancel_goal_async()
                if self._stop_event.is_set():
                    raise RunAborted(f"{label} 收到 stop 指令")
                raise TimeoutError(f"{label} 执行超时")
            return result_future.result().result
        finally:
            with self._goal_lock:
                self._goal_handle = None

    def _wait_future(self, future, timeout_s: float) -> bool:
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        deadline = time.monotonic() + max(0.0, timeout_s)
        while time.monotonic() < deadline:
            if self._shutdown or self._stop_event.is_set():
                return False
            if done.wait(0.05):
                return True
        return future.done()

    def _cancel_current_goal(self) -> None:
        with self._goal_lock:
            goal_handle = self._goal_handle
        if goal_handle is not None:
            try:
                goal_handle.cancel_goal_async()
            except Exception:  # noqa: BLE001
                pass

    # ------------------------------------------------------------------
    # 头部与等待
    # ------------------------------------------------------------------
    def _on_head_status(self, msg: String) -> None:
        with self._head_lock:
            self._head_state = str(msg.data).strip()

    def _stop_head(self) -> None:
        if not self._head_stop_enabled:
            return
        with self._head_lock:
            before = self._head_state
        msg = String()
        msg.data = "stop"
        self._head_pub.publish(msg)

        if int(self._head_pub.get_subscription_count()) <= 0:
            self.get_logger().warn(
                "未发现头部服务，无法确认回中，仅等待固定稳定时间")
            return
        if before == "stopped":
            self.get_logger().info("头部已处于停止状态，无需等待回中")
            return

        self.get_logger().info("已请求头部停止扫视并回中，等待头部停止")
        deadline = time.monotonic() + max(0.0, self._head_stop_timeout_s)
        while time.monotonic() < deadline:
            if self._shutdown or self._stop_event.is_set():
                return
            with self._head_lock:
                state = self._head_state
            if state == "stopped":
                self.get_logger().info("头部已回中")
                return
            time.sleep(0.05)
        self.get_logger().warn(
            f"等待头部回中超时（{self._head_stop_timeout_s:.1f}s），继续取图")

    def _sleep(self, seconds: float) -> None:
        deadline = time.monotonic() + max(0.0, seconds)
        while time.monotonic() < deadline:
            if self._shutdown:
                raise RunAborted("节点关闭")
            if self._stop_event.is_set():
                raise RunAborted("收到 stop 指令")
            time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))

    def _summary(self) -> str:
        if not self._results:
            return "无视觉结果"
        return "，".join(f"{poi}={status}" for poi, status in self._results.items())

    # ------------------------------------------------------------------
    # 收尾
    # ------------------------------------------------------------------
    def destroy_node(self):
        self._shutdown = True
        self._cancel_current_goal()
        if self._worker is not None:
            self._worker.join(timeout=2.0)
        with self._state_lock:
            unfinished = self._state_text.startswith(
                (RunState.RUNNING.value, RunState.PAUSED.value)
            )
        if unfinished:
            # 工作线程没能及时退出时，也要让外部看到本轮是被中断的
            self._set_state(RunState.STOPPED)
        return super().destroy_node()


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = OrchestratorNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        executor.shutdown()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
