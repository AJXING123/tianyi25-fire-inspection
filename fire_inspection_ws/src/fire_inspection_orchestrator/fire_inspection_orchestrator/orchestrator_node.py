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

import base64
import fcntl
import hashlib
import hmac
import json
import math
import os
import sys
import threading
import time
import urllib.parse
import urllib.request
from enum import Enum
from types import SimpleNamespace

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from fire_inspection_interfaces.action import (ApproachTarget, InspectPoi,
                                               NavigateTo, Speak)
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

# 同一导航路段允许的介入次数硬上限：`intervention.max_per_poi=0`（不限）时的兜底，
# 避免同一个人站在路上导致「停车→接近→播报→重发」无限循环、巡检永远走不完
_NAV_MAX_INTERVENTIONS_PER_LEG = 3

# 首次命中先停车（不等 3 帧确认）的兜底：误报/人员已走开时频繁停车会让巡检
# 走不动，同一路段未确认停车超过该次数后回退为「确认后再停」的旧行为
_NAV_MAX_PRESTOP_UNCONFIRMED = 3
# 未确认恢复后的一小段冷却：避免刚起步又立刻被同一个误报停住
_NAV_PRESTOP_COOLDOWN_S = 3.0
# 首次命中先停车（不等确认）：低分命中只停 3s，确认不成立就恢复导航，
# 不会播报、不会绕行。2026-11-02 现场把门槛提到 0.70 后，0.48/0.50 的
# 早期命中不预停车，车一直开到离人 0.88m 才发现，已经接近碰撞；这里
# 恢复 0.40，让「疑似吸烟」先停车，再由多帧确认决定是否真正介入。
_PRESTOP_MIN_CONF = 0.40


# 单实例锁：同一台机器只允许一个编排层消费 S1 控制指令
_INSTANCE_LOCK_PATH = os.environ.get(
    "FIRE_INSPECTION_ORCHESTRATOR_LOCK",
    "/tmp/fire_inspection_orchestrator.lock",
)


def parse_detection_snapshot(item):
    """把单条 detections_json 解析成吸烟介入用的快照。

    range_valid 只有「视觉标记可用」且「距离为正」同时成立才为 true；
    旧版视觉 / 未配深度时没有这些字段，一律视为不可用——调用方据此
    只停车播报、不靠近人员，避免拿着瞎猜的距离乱走。
    """
    if not isinstance(item, dict):
        return None
    try:
        conf = float(item.get("conf", 0.0) or 0.0)
    except (TypeError, ValueError):
        conf = 0.0
    try:
        range_m = float(item.get("range_m", 0.0) or 0.0)
    except (TypeError, ValueError):
        range_m = 0.0
    try:
        bearing = float(item.get("bearing_rad", 0.0) or 0.0)
    except (TypeError, ValueError):
        bearing = 0.0
    # bearing_valid 区分「没有方位角」和「方位角恰好为 0」：
    # 绕行必须知道人在左侧还是右侧，缺方位时宁可不横移。
    bearing_valid = False
    if item.get("bearing_rad") is not None:
        try:
            bearing_valid = math.isfinite(float(item["bearing_rad"]))
        except (TypeError, ValueError):
            bearing_valid = False
    return {
        "cls": str(item.get("cls", "") or ""),
        "conf": conf,
        "range_m": range_m,
        "bearing_rad": bearing,
        "bearing_valid": bearing_valid,
        "range_valid": bool(item.get("range_valid", False)) and range_m > 0.0,
        "t": time.monotonic(),
    }


def parse_detection_center(item):
    """取检测框中心（画面归一化 cx/cy）；缺失或非法返回 None。

    多帧确认时用它判断是否还是同一个目标，防止不同位置的误检被累加。
    """
    if not isinstance(item, dict):
        return None
    try:
        cx = float(item.get("cx"))
        cy = float(item.get("cy"))
    except (TypeError, ValueError):
        return None
    if not (math.isfinite(cx) and math.isfinite(cy)):
        return None
    return cx, cy


class RunState(Enum):
    """编排层对外状态（T4 /fire_inspection/state 的取值来源）。

    发布格式：`<state>` 或 `<state>:<phase>[:<poi>]`，全 ASCII，便于控制台解析。
        idle                  未开始 / 本轮已结束且无结果
        running:wait          等待各 Action 服务端就绪
        running:speak:start   播报开始语
        running:nav:<poi>     前往某点位
        running:intervene:<poi> 途中确认吸烟，停车→接近人员→播报→回轨道
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


class NavBlocked(RuntimeError):
    """导航层判定该点位受阻且自恢复无效（phase=blocked），可跳过继续。"""


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
        self.declare_parameter("auto_start", False)
        self.declare_parameter("nav_action", "/fire_inspection/navigate_to")
        self.declare_parameter("approach_action", "/fire_inspection/approach_target")
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
        # 等 Action 服务端的窗口：nav 启动时可能先做自动重定位
        # （relocalize.timeout_s 最长 60s），这里留 90s，避免刚重定位到一半
        # 就被判「导航不可用」。
        self.declare_parameter("wait_servers_s", 90.0)
        self.declare_parameter("nav_goal_timeout_s", 200.0)
        self.declare_parameter("cancel_grace_s", 10.0)
        self.declare_parameter("nav_enabled", True)
        # 确认帧数：恢复「3 秒窗口内累计 3 帧」。10-31 日志证实空走廊画面
        # 出现单帧 smoking=0.53 幻觉，2 帧 + 0.15s 跨度几乎等于「检出即报」。
        self.declare_parameter("confirm_frames", 3)
        self.declare_parameter("confirm_window_s", 3.0)
        # 「首次命中到确认」的最短时间跨度。5Hz 下相邻两帧间隔 0.2s，
        # 1.2s 可挡掉同一处幻觉在 0.2~1.0s 内连刷几帧的假确认
        # （2026-11-01 现场：空走廊/拿平板的人 1.0s 内刷 6 帧被误确认）。
        self.declare_parameter("confirm_min_span_s", 1.2)
        # 同一目标的框中心在画面内允许的漂移（归一化）。人走动、头部扫视
        # 都会让框移动：0.12 过严（日志里 3/3 帧被反复清零），放宽到 0.40。
        self.declare_parameter("confirm_pos_tol", 0.40)
        # 单帧快速确认门槛：2026-11-01 现场 0.46~0.85 的吸烟命中大多
        # 是「人拿手机/平板」或空走廊的误报，不能再按 0.45 一帧即报。
        # 只有极高置信度（≥0.90）才允许单帧确认，其余必须走多帧窗口。
        self.declare_parameter("quick_confirm_conf", 0.90)
        # 吸烟单独的单帧确认门槛：现场模型对真吸烟常常只响应 1 帧
        # （日志：0.73 闪一帧后消失），0.90 永远等不到第 2 帧；而误检
        # 候选通常 ≤0.40。0.60 只放宽吸烟，不影响 person 的多帧确认。
        self.declare_parameter("smoking_quick_confirm_conf", 0.60)
        # 人形确认后先停车补看吸烟的窗口：现场烟往往在人停稳、头部抬起
        # 后才进入画面（日志：停车后约 2.4s 出现 smoking=0.77）。在这段
        # 时间内看到烟就切换为吸烟播报/绕行，避免只报“有人挡路”。
        self.declare_parameter("person_smoking_grace_s", 2.5)
        self.declare_parameter("cooldown_s", 8.0)
        self.declare_parameter("lost_timeout_s", 3.0)
        # 移动路段（WATCH）只关注吸烟与行人；灭火器/安全出口指示灯属于点位定检内容，
        # 行走中检测会在同一区域反复播报。person 由视觉节点的人体模型给出，
        # 确认后走「先播报 → 侧向绕开 → 继续巡检」。
        self.declare_parameter("scan_classes", ["smoking", "person"])
        self.declare_parameter("detection_topic", "/fire_inspection/detection")
        self.declare_parameter("transit_watch", True)
        self.declare_parameter("hfov_deg", 85.5)
        self.declare_parameter("vfov_deg", 55.0)

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
        self._cancel_grace_s = float(self.get_parameter("cancel_grace_s").value)
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

        # 视场角：与 vision.yaml 同一套取值（参数默认值保持一致），
        # 这里的 look_at 换算与视觉层方位角换算必须用同一份视场角
        self._hfov = math.radians(float(self.get_parameter("hfov_deg").value))
        self._vfov = math.radians(float(self.get_parameter("vfov_deg").value))

        # WATCH 模式参数
        self._nav_enabled = bool(self.get_parameter("nav_enabled").value)
        self._confirm_frames = int(self.get_parameter("confirm_frames").value)
        self._confirm_window_s = float(
            self.get_parameter("confirm_window_s").value)
        self._confirm_min_span_s = max(
            0.0, float(self.get_parameter("confirm_min_span_s").value))
        self._confirm_pos_tol = max(
            0.0, float(self.get_parameter("confirm_pos_tol").value))
        self._quick_confirm_conf = float(
            self.get_parameter("quick_confirm_conf").value)
        self._smoking_quick_confirm_conf = float(
            self.get_parameter("smoking_quick_confirm_conf").value)
        if not (0.0 < self._smoking_quick_confirm_conf <= 1.0):
            raise RuntimeError(
                "smoking_quick_confirm_conf 非法: "
                f"{self._smoking_quick_confirm_conf}（应在 (0, 1]）")
        self._person_smoking_grace_s = max(
            0.0, float(self.get_parameter("person_smoking_grace_s").value))
        self._cooldown_s = float(self.get_parameter("cooldown_s").value)
        self._lost_timeout_s = float(self.get_parameter("lost_timeout_s").value)
        self._scan_classes = list(self.get_parameter("scan_classes").value)
        self._transit_watch = bool(self.get_parameter("transit_watch").value)

        # WATCH 模式状态
        self._watch_state = "WATCH"       # WATCH / LOCKED / INTERVENE / SPEAK / COOLDOWN
        self._watch_streak_cls = None
        self._watch_streak = 0
        # 同一类别的命中时刻（秒）：确认只看「窗口内累计命中数」，
        # 单帧闪烁不会把已攒下的帧数清零（现场实测隔帧丢失导致确认反复失败）
        self._watch_hits = []
        # 首次命中时的框中心（画面归一化 cx/cy）：后续命中必须落在同一目标
        # 附近才算连续确认，防止画面里不同位置的误检被累计成一次确认
        self._watch_anchor = None
        self._watch_locked_pose = None     # {"yaw": float, "pitch": float}
        self._watch_lost_at = None
        self._watch_last_trigger = 0.0
        self._watch_lock = threading.Lock()
        # 导航模式下的移动路段检测门控：True 时 _on_detection 才处理检测流
        self._watch_active = False

        # 吸烟人员介入：导航途中确认 → 停车 → 接近 → 播报 → 回原轨道目标
        # 本节只做状态记录，真正的底盘动作全部交给导航模块的 ApproachTarget
        self._nav_in_flight = False
        self._nav_poi = ""
        self._intervention_event = threading.Event()
        # 路段级「不再介入」开关：同一路段介入次数达到硬上限后置位，
        # 保证即使配置成 max_per_poi: 0（不限）也不会在同一段路无限停车
        self._intervention_suppress = False
        self._intervention_snap = None     # 确认瞬间的检测快照
        # 正在执行的介入类别；人员介入期间用它判断要不要接收吸烟补帧
        self._intervention_cls = None
        # 人员介入期间检到的吸烟快照（停车后补看窗口内写入）
        self._smoking_during_person = None
        self._latest_det = None            # 最近一帧关注类别的检测快照
        self._intervention_counts = {}     # 每个点位已介入次数
        self._intervention_total = 0       # 本轮介入总次数
        # 介入过的点位：该段重发导航改点对点，避免底盘退回上轨点后又从
        # 人员所在的轨道上重走一遍（到达后清空）
        self._point_resume_poi = None
        # 首次命中先停车：确认是否成立、本段未确认次数、冷却截止时刻
        self._intervention_confirmed = False
        # 介入次数已用尽时的「只停车播报」标记：不动手接近，也不计数
        self._speak_only_pending = False
        self._prestop_unconfirmed = 0
        self._prestop_block_until = 0.0
        # 预停车是否真的把车刹停了：确认是「侧前方路人、不在路线上」时
        # 补播一句让行提示，避免现场「车停了让了却没声音」。
        self._prestop_stopped = False
        self._approach_available = False   # 接近 Action 服务端是否可用
        # 由 inspection_script.yaml 的 intervention 段覆盖；先给保守默认值
        self._intervention_enabled = False
        self._intervention_classes = []
        # 只播报不停车的类别（speak_only_classes）：不取消导航，继续执行任务
        self._intervention_speak_only_classes = set()
        # 停车→侧向绕开人员→继续 的类别（avoid_classes）
        self._intervention_avoid_classes = set()
        self._avoid_offset_m = 1.5
        # 行人是否在行进路线上的判定（只影响 person/smoking 的停车与绕行）：
        # 横向偏移 = range*sin(bearing)。只有「很近 + 基本正前方」才停车，
        # 其余只播报后继续巡检；门槛宁紧勿松，避免一路反复停车。
        # 3m 停车判定 + 0.5m 深度测距余量（现场 3.17m 曾被判成「不在路径」）
        self._person_block_range_m = 3.5
        self._person_lane_half_width_m = 0.5
        self._person_max_bearing_rad = 0.35
        self._intervention_max_per_poi = 1
        self._intervention_max_total = 5
        self._intervention_settle_s = 0.5
        self._intervention_snapshot_wait_s = 2.0
        self._intervention_timeout_s = 60.0

        # detection 订阅（WATCH 模式驱动源）
        from fire_inspection_interfaces.msg import PoiResult
        self._detection_sub = self.create_subscription(
            PoiResult,
            str(self.get_parameter("detection_topic").value),
            self._on_detection,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT),
        )

        # 听觉警报订阅（fire_inspection_audio 节点发来）
        self.declare_parameter("audio_alert_topic", "/fire_inspection/audio_alert")
        self._audio_alert_sub = self.create_subscription(
            String,
            str(self.get_parameter("audio_alert_topic").value),
            self._on_audio_alert,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT),
        )
        self._audio_alert_cooldown = {}   # 每个 alert 样本的最近触发时间（monotonic）
        self._audio_alert_min_gap_s = 10.0  # 同一警报至少间隔这么久才重复触发

        # 异常抓拍：订阅相机 compressed 流缓存最新帧，异常判定时 JPEG 直写落盘。
        # compressed 图像的 data 就是完整 JPEG 字节流，不解码、零推理开销；
        # 帧龄通常 <0.5s（相机 ~15Hz），异常确认瞬间画面里目标大概率还在。
        self.declare_parameter(
            "capture_dir",
            "/home/nvidia/fire-inspection/fire_inspection_ws/picture")
        self.declare_parameter(
            "capture_camera_topic",
            "/ob_camera_head/color/image_raw/compressed")
        self._capture_dir = str(self.get_parameter("capture_dir").value)
        self._cam_lock = threading.Lock()
        self._cam_frame = None          # (recv_monotonic, jpeg_bytes)
        self._capture_last = {}         # 各异常类型的落盘冷却（monotonic）
        self._capture_cooldown_s = 10.0  # 同类异常至少间隔这么久才再拍
        # 抓拍图自动保留天数（超过 N 天的 jpg 会被清理）。0 表示不清理。
        self.declare_parameter("capture_retention_days", 7)
        self._capture_retention_days = float(
            self.get_parameter("capture_retention_days").value)
        self._capture_purge_cooldown_s = 6 * 3600.0  # 目录扫描节流：6h 最多清一次
        self._capture_last_purge = 0.0

        # 钉钉自定义机器人 webhook —— 异常抓拍时自动发 Markdown 卡片到群里
        # 安全设置用"加签"模式，需要 access_token（webhook ?access_token= 后面那串）
        # 和 secret（机器人安全设置页面里的加签密钥）。两者都为空时禁用钉钉推送。
        self.declare_parameter("dingtalk_enabled", False)
        self._dingtalk_enabled = bool(
            self.get_parameter("dingtalk_enabled").value)
        self.declare_parameter("dingtalk_access_token", "")
        self._dingtalk_token = str(
            self.get_parameter("dingtalk_access_token").value).strip()
        self.declare_parameter("dingtalk_secret", "")
        self._dingtalk_secret = str(
            self.get_parameter("dingtalk_secret").value).strip()
        self._dingtalk_last_send = {}       # 每类异常钉钉发送节流（防刷屏）
        self._dingtalk_cooldown_s = 30.0    # 同类异常钉钉至少间隔这么久才再发一次
        # 启动时打印钉钉状态，方便 journalctl 直接确认参数是否注入成功
        _dt_ok = (self._dingtalk_enabled and self._dingtalk_token
                  and self._dingtalk_secret)
        if _dt_ok:
            self.get_logger().info(
                f"📣 钉钉推送已启用：token={self._dingtalk_token[:6]}...{self._dingtalk_token[-4:]}，"
                f"secret={self._dingtalk_secret[:6]}...{self._dingtalk_secret[-4:]}，"
                f"白名单=吸烟/设施异常/防空警报（person 不入钉钉），无冷却=检测几次发几次")
        else:
            self.get_logger().warn(
                f"📣 钉钉推送未启用：enabled={self._dingtalk_enabled}，"
                f"token={'有' if self._dingtalk_token else '无'}，"
                f"secret={'有' if self._dingtalk_secret else '无'}")
        self._camera_sub = self.create_subscription(
            CompressedImage,
            str(self.get_parameter("capture_camera_topic").value),
            self._on_camera_frame,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
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
        self._approach_client = ActionClient(
            self, ApproachTarget, str(self.get_parameter("approach_action").value)
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
        auto_start = bool(self.get_parameter("auto_start").value)
        if auto_start:
            self._start_timer = self.create_timer(delay, self._start_once)
            self.get_logger().info(f"自动巡检已启用，将在 {delay:.1f} 秒后启动")
        else:
            self._start_timer = None
            self.get_logger().info("自动巡检已关闭，等待网页控制启动")

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
            if self._intervention_enabled:
                self.get_logger().info(
                    f"吸烟介入：类别={self._intervention_classes}，"
                    f"只播报不停车={sorted(self._intervention_speak_only_classes) or '无'}，"
                    f"侧向绕开={sorted(self._intervention_avoid_classes) or '无'}"
                    f"（横移 {self._avoid_offset_m:.1f}m），"
                    f"每点位上限={self._intervention_max_per_poi or '不限'}，"
                    f"本轮总上限={self._intervention_max_total or '不限'}，"
                    f"路径判定=距离≤{self._person_block_range_m:.1f}m/"
                    f"横向≤{self._person_lane_half_width_m:.2f}m/"
                    f"方位≤{math.degrees(self._person_max_bearing_rad):.0f}°，"
                    f"停车稳定={self._intervention_settle_s:.1f}s，"
                    f"目标距离上限由 pois.yaml 的 approach 段约束"
                )
            else:
                self.get_logger().info(
                    "吸烟介入已关闭（intervention.enabled=false）："
                    "途中确认吸烟只播报、不停车不接近"
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

        # 吸烟人员介入：本段只决定「要不要介入、介入几次、停车等多久」，
        # 距离/位移/限速等安全边界全部在 pois.yaml 的 approach 段（导航侧唯一真源）
        intervention_cfg = cfg.get("intervention") or {}
        if not isinstance(intervention_cfg, dict):
            raise RuntimeError("intervention 必须是映射")
        unknown_intervention = set(intervention_cfg) - {
            "enabled", "classes", "speak_only_classes",
            "avoid_classes", "avoid_offset_m",
            "person_on_path",
            "max_per_poi", "max_total",
            "settle_s", "snapshot_wait_s", "timeout_s",
        }
        if unknown_intervention:
            raise RuntimeError(
                f"intervention 存在未知键: {sorted(unknown_intervention)}")
        classes = intervention_cfg.get("classes", [])
        if not isinstance(classes, list):
            raise RuntimeError("intervention.classes 必须是列表")
        speak_only = intervention_cfg.get("speak_only_classes", [])
        if not isinstance(speak_only, list):
            raise RuntimeError("intervention.speak_only_classes 必须是列表")
        avoid = intervention_cfg.get("avoid_classes", [])
        if not isinstance(avoid, list):
            raise RuntimeError("intervention.avoid_classes 必须是列表")
        self._intervention_enabled = bool(
            intervention_cfg.get("enabled", False))
        self._intervention_classes = [str(c) for c in classes]
        self._intervention_speak_only_classes = {str(c) for c in speak_only}
        self._intervention_avoid_classes = {str(c) for c in avoid}
        self._avoid_offset_m = float(
            intervention_cfg.get("avoid_offset_m", 1.5))
        if not (math.isfinite(self._avoid_offset_m)
                and 0.3 <= self._avoid_offset_m <= 3.0):
            raise RuntimeError(
                f"intervention.avoid_offset_m 非法: {self._avoid_offset_m}"
                f"（应在 [0.3, 3.0]）")
        overlap = (self._intervention_speak_only_classes
                   & self._intervention_avoid_classes)
        if overlap:
            raise RuntimeError(
                f"intervention.speak_only_classes 与 avoid_classes 不能重叠: "
                f"{sorted(overlap)}")
        person_on_path = intervention_cfg.get("person_on_path") or {}
        if not isinstance(person_on_path, dict):
            raise RuntimeError("intervention.person_on_path 必须是映射")
        unknown_person = set(person_on_path) - {
            "max_range_m", "lane_half_width_m", "max_bearing_rad"}
        if unknown_person:
            raise RuntimeError(
                f"intervention.person_on_path 存在未知键: "
                f"{sorted(unknown_person)}")
        self._person_block_range_m = float(
            person_on_path.get("max_range_m", 2.0))
        self._person_lane_half_width_m = float(
            person_on_path.get("lane_half_width_m", 0.5))
        self._person_max_bearing_rad = float(
            person_on_path.get("max_bearing_rad", 0.35))
        if not (math.isfinite(self._person_block_range_m)
                and 0.5 <= self._person_block_range_m <= 20.0):
            raise RuntimeError(
                "intervention.person_on_path.max_range_m 非法: "
                f"{self._person_block_range_m}（应在 [0.5, 20.0]）")
        if not (math.isfinite(self._person_lane_half_width_m)
                and 0.2 <= self._person_lane_half_width_m <= 3.0):
            raise RuntimeError(
                "intervention.person_on_path.lane_half_width_m 非法: "
                f"{self._person_lane_half_width_m}（应在 [0.2, 3.0]）")
        if not (math.isfinite(self._person_max_bearing_rad)
                and 0.1 <= self._person_max_bearing_rad <= math.pi / 2):
            raise RuntimeError(
                "intervention.person_on_path.max_bearing_rad 非法: "
                f"{self._person_max_bearing_rad}（应在 [0.1, pi/2]）")
        self._intervention_max_per_poi = int(
            intervention_cfg.get("max_per_poi", 1))
        self._intervention_max_total = int(
            intervention_cfg.get("max_total", 5))
        self._intervention_settle_s = float(
            intervention_cfg.get("settle_s", 0.5))
        self._intervention_snapshot_wait_s = float(
            intervention_cfg.get("snapshot_wait_s", 2.0))
        self._intervention_timeout_s = float(
            intervention_cfg.get("timeout_s", 60.0))
        if self._intervention_max_per_poi < 0 or self._intervention_max_total < 0:
            raise RuntimeError(
                "intervention.max_per_poi / max_total 不能为负（0 表示不限）")
        if self._intervention_settle_s < 0 or self._intervention_snapshot_wait_s < 0:
            raise RuntimeError(
                "intervention.settle_s / snapshot_wait_s 不能为负")
        if (
            not math.isfinite(self._intervention_timeout_s)
            or self._intervention_timeout_s <= 0
        ):
            raise RuntimeError(
                f"intervention.timeout_s 非法: {self._intervention_timeout_s}")

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
        if self._start_timer is not None:
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
        self._reset_watch_state()
        self._worker = threading.Thread(
            target=self._run, name=f"inspection-{source}", daemon=True
        )
        self._worker.start()
        return True, "已开始本轮巡检"

    def _reset_watch_state(self) -> None:
        """新一轮（含 manual）：清掉上一轮残留的锁定/介入状态与介入计数。"""
        with self._watch_lock:
            self._watch_state = "WATCH"
            self._watch_streak_cls = None
            self._watch_streak = 0
            self._watch_hits = []
            self._watch_lost_at = None
            self._watch_locked_pose = None
            self._intervention_event.clear()
            self._intervention_snap = None
            self._intervention_cls = None
            self._smoking_during_person = None
            self._latest_det = None
            self._intervention_counts.clear()
            self._intervention_total = 0

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
        self._reset_watch_state()
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
            # 巡检一启动就让头部扫起来，不等服务就绪与开始语播报
            # （重复 start 无害：head_sweep 已在扫时会忽略）
            if self._transit_watch:
                self._send_head_cmd("start")
            self._set_state(RunState.RUNNING, "wait")
            self._wait_servers()
            self._checkpoint()
            self._set_state(RunState.RUNNING, "speak:start")
            self._speak(SPEECH_TEXTS["start"], "开始")
            self._transit_watch_on()
            self._navigate_start()

            for poi in self._cfg["inspect_pois"]:
                self._checkpoint()
                # 巡检点受阻（AEB 刹停/被挡）：导航层已做过回退重发，
                # 这里再重试一次，仍失败则跳过该点并播报，不终止整轮巡检
                if not self._navigate_inspect(poi):
                    continue
                self._set_state(RunState.RUNNING, f"settle:{poi}")
                self._transit_watch_off()
                self._stop_head()
                self._sleep(self._settle_s)
                self._checkpoint()
                self._inspect(poi)
                self._transit_watch_on()

            for poi in self._cfg["transit_pois"]:
                self._checkpoint()
                # 过点只是返程轨道的上轨点，失败不应终止整轮巡检：
                # 先重试一次，仍失败则跳过，让返程自动对接轨道。
                try:
                    self._navigate(poi, is_return=False)
                except RunAborted:
                    raise
                except Exception as exc:  # noqa: BLE001
                    self.get_logger().warn(
                        f"过点 {poi} 未到达（{exc}），重试一次")
                    try:
                        self._navigate(poi, is_return=False)
                    except RunAborted:
                        raise
                    except Exception as exc2:  # noqa: BLE001
                        self.get_logger().warn(
                            f"过点 {poi} 重试仍失败（{exc2}），跳过并继续返程")

            self._checkpoint()
            if self._cfg["return_to_start"]:
                self._set_state(RunState.RUNNING, "return")
                try:
                    self._navigate(self._cfg["start_poi"], is_return=True)
                except RunAborted:
                    raise
                except Exception as exc:  # noqa: BLE001
                    # 返程是最后一步，本轮巡检结果已经记录：任何失败
                    # （受阻 blocked、底盘返回 cancelled/failed、超时）
                    # 都不该把整轮判成 aborted，记录后正常收尾由人工接管。
                    self._results[self._cfg["start_poi"]] = "未返回起点"
                    self.get_logger().error(
                        f"返程失败，未能返回起点（{exc}）；"
                        f"本轮巡检结果保留，请人工接管车辆")
                    self._speak(SPEECH_TEXTS["nav_blocked"], "返程受阻")

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
    # WATCH 模式：检测回调驱动的"首次命中即锁定→连续 N 帧确认→播报→恢复扫视"
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
            if self._watch_state in ("LOCKED", "INTERVENE"):
                # 未确认 / 未执行的触发直接丢弃，不发 resume
                # （头部控制权移交定点流程；介入已结束时由 _handle_intervention 收尾）
                self._watch_state = "WATCH"
                self._watch_streak_cls = None
                self._watch_streak = 0
                self._watch_hits = []
                self._watch_lost_at = None
                self._intervention_cls = None
                self._smoking_during_person = None
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

    def _parse_look_at(self, detections_json: str, classes=None):
        """取最高 conf 目标中心，换算成头部 look_at 的 yaw/pitch（弧度）。

        符号与 vision.yaml / 头部实测一致（head_adapter：yaw 正=向左，
        pitch 正=低头）：
          图像右侧（cx>0.5）→ 头向右转 → yaw 负
          图像下方（cy>0.5）→ 低头 → pitch 正

        classes 非空时只在关注类别里挑目标，避免画面里高 conf 的非关注
        目标（如灭火器）把头部锁到错误物体上。

        优先用视觉给出的精确值：
          bearing_rad     = 头部 yaw + 像素偏移（用 camera_info 内参换算）
                            → 就是"转到该 yaw 正好对准目标"
          look_pitch_rad  = 头部 pitch + 俯仰偏移（绝对姿态）
        旧版视觉没有这些字段时退回视场角线性近似（受参数 hfov/vfov 影响）。
        """
        try:
            arr = json.loads(detections_json)
        except Exception:
            return None
        if not isinstance(arr, list) or not arr:
            return None
        if classes:
            arr = [d for d in arr if d.get("cls") in classes]
            if not arr:
                return None
        # 按 conf 降序
        try:
            arr.sort(key=lambda d: d.get("conf", 0.0), reverse=True)
            top = arr[0]
            cx = float(top.get("cx", 0.5))   # 归一化 0~1
            cy = float(top.get("cy", 0.5))
        except (AttributeError, TypeError, ValueError):
            return None

        def _precise(key, fallback):
            """视觉精确值优先；缺失或非有限时退回视场角近似，逐字段独立判断。"""
            try:
                value = float(top[key])
            except (AttributeError, KeyError, TypeError, ValueError):
                return fallback
            return value if math.isfinite(value) else fallback

        yaw = _precise("bearing_rad", (0.5 - cx) * self._hfov)
        pitch = _precise("look_pitch_rad", (cy - 0.5) * self._vfov)
        if not (math.isfinite(yaw) and math.isfinite(pitch)):
            return None
        if str(top.get("cls", "") or "") == "person":
            # 现场相机安装角导致关节 0 位仍偏向地面，锁定人形时要适当抬头，
            # 否则画面只剩腿。原值 -0.38（抬头约 22°，接近 25° 限位）视觉上
            # 像在仰头看天，2026-11-02 按现场反馈下调到 -0.22（约 13°）。
            # 代价：近距离上半身/手更容易出画，可能影响烟/吸烟的可见性。
            pitch = min(pitch, -0.22)
        # 限位：头部关节实测限位 yaw ±90°、pitch ±25°
        yaw = max(-math.radians(90.0), min(math.radians(90.0), yaw))
        pitch = max(-math.radians(25.0), min(math.radians(25.0), pitch))
        return {"yaw": round(yaw, 4), "pitch": round(pitch, 4), "cls": top.get("cls", "")}

    def _intervention_allowed(self, cls) -> bool:
        """是否停车介入。必须在持有 _watch_lock 时调用。"""
        if not self._intervention_enabled:
            return False
        if not self._nav_enabled or not self._nav_in_flight:
            return False
        if cls not in self._intervention_classes:
            return False
        if self._intervention_suppress:
            # 本路段介入次数已达硬上限（见 _NAV_MAX_INTERVENTIONS_PER_LEG）
            return False
        if (
            self._intervention_max_total > 0
            and self._intervention_total >= self._intervention_max_total
        ):
            return False
        poi = self._nav_poi
        if (
            self._intervention_max_per_poi > 0
            and self._intervention_counts.get(poi, 0)
            >= self._intervention_max_per_poi
        ):
            return False
        return True

    def _stop_vehicle_for(self, cls) -> bool:
        """该类命中时是否停车介入；False = 只播报，导航继续执行任务。"""
        return cls not in self._intervention_speak_only_classes

    def _avoid_class(self, cls) -> bool:
        """该类命中时是否「停车→侧向绕开人员→继续」。"""
        return cls in self._intervention_avoid_classes

    def _target_on_path(self, snap, top=None) -> bool:
        """判断目标是否在机器人行进路线上。

        只用视觉给出的「距离 + 方位角」做保守判断，不读地图：
        横向偏移 = range * sin(bearing)，超过任一门限即视为不在路径上，
        此时只播报、不取消导航、不停车。缺方位/缺测距时按“在路径上”
        处理，宁可直接停车确认，也不把真正挡路的人漏掉。

        top 供首帧预筛使用：深度/头部姿态尚未就绪时检测结果没有
        bearing_rad（bearing_valid=False），若仍按“在路径上”处理，
        侧前方很近的人会触发预停车（2026-11-03 返程侧前方约 1m 的人
        连续逼停 3 次，每次都要等确认窗口才恢复）。传入原始检测框后，
        缺方位时用画面横向位置与 hfov 估算方位，仅用于排除明显偏侧
        的目标；距离门限仍只在 range_valid 时生效。
        """
        if snap is None:
            return True
        bearing = None
        if snap.get("bearing_valid"):
            bearing = abs(float(snap.get("bearing_rad", 0.0)))
        elif top is not None:
            try:
                bearing = abs((0.5 - float(top.get("cx"))) * self._hfov)
            except (AttributeError, TypeError, ValueError):
                bearing = None
            if bearing is not None and not math.isfinite(bearing):
                bearing = None
        if bearing is None:
            return True
        if bearing > self._person_max_bearing_rad:
            return False
        if not snap.get("range_valid"):
            return True
        range_m = float(snap.get("range_m", 0.0))
        if range_m > self._person_block_range_m:
            return False
        lateral = abs(range_m * math.sin(bearing))
        return lateral <= self._person_lane_half_width_m

    def _take_speak_only(self) -> bool:
        """取走「只停车播报」标记：一次性使用，读完立即清零。"""
        with self._watch_lock:
            flag = self._speak_only_pending
            self._speak_only_pending = False
        return flag

    # ------------------------------------------------------------------
    # 听觉警报（防空警报）：fire_inspection_audio 节点通过
    # /fire_inspection/audio_alert 发来样本纯名（如 'air_raid'）。
    # 命中即播报 SPEECH_TEXTS 中对应文案。导航途中先停车介入再播报，
    # 非导航途中仅播报不打断。
    # ------------------------------------------------------------------
    # ------------------------------------------------------------------
    # 异常抓拍
    # ------------------------------------------------------------------
    def _on_camera_frame(self, msg: CompressedImage) -> None:
        """缓存最新一帧相机 JPEG（仅保存字节与到达时间，不解码）。"""
        with self._cam_lock:
            self._cam_frame = (time.monotonic(), bytes(msg.data))

    def _capture_event(self, kind: str, detail: str = "") -> None:
        """异常确认瞬间把缓存的最新相机帧落盘（picture 目录，JPEG 直写）。

        抓拍失败只记日志，绝不阻断巡检主流程；同类异常带冷却防连拍。
        detail 可选：POI 视觉结果的 result.detail，用来在钉钉卡片上
        精确区分"灭火器缺失"vs"安全指示牌异常"等具体设施。
        """
        try:
            now = time.monotonic()
            if now - self._capture_last.get(kind, 0.0) < self._capture_cooldown_s:
                return
            with self._cam_lock:
                frame = self._cam_frame
                self._cam_frame = None
            if frame is None:
                self.get_logger().warn(f"[capture] {kind}：无缓存相机帧，跳过抓拍")
                return
            age = now - frame[0]
            os.makedirs(self._capture_dir, exist_ok=True)
            ts = time.strftime("%Y%m%d_%H%M%S")
            path = os.path.join(self._capture_dir, f"{kind}_{ts}.jpg")
            with open(path, "wb") as f:
                f.write(frame[1])
            self._capture_last[kind] = now
            self.get_logger().warn(
                f"[capture] {kind} 异常抓拍已保存：{path}（帧龄 {age:.1f}s）")
            self._purge_old_captures()
            self._dingtalk_notify(kind, path, detail)
        except Exception as exc:  # noqa: BLE001 - 抓拍失败不影响巡检
            self.get_logger().error(f"[capture] {kind} 抓拍失败：{exc}")

    # 钉钉推送白名单：只有这些 kind 会发到钉钉，person 等一律跳过
    _DINGTALK_WHITELIST = ("smoking", "air_raid")

    def _dingtalk_notify(self, kind: str, pic_path: str, detail: str = "") -> None:
        """抓拍成功后异步发钉钉 Markdown 卡片到群里。

        必须放到独立线程：DNS 解析失败时 urllib 的 timeout=3 并不生效，
        实测会阻塞 20s 以上（2026-11-03 18:04/18:06 现场日志）。这条链路
        原本在检测回调里同步执行，会把「确认 → 播报」整体推迟 20s，现场
        表现就是「都避让了却迟迟没播报」。检测几次发几次（无冷却）——
        冷却只在抓拍落盘层，钉钉本身每发一次。person 等无关 kind 一律跳过，
        只推吸烟 / 设施异常 / 防空警报。
        """
        threading.Thread(
            target=self._dingtalk_send, args=(kind, pic_path, detail),
            daemon=True).start()

    def _dingtalk_send(self, kind: str, pic_path: str, detail: str = "") -> None:
        """真正的钉钉推送（在后台线程里跑，失败只记日志）。"""
        try:
            if not self._dingtalk_enabled:
                return
            if not self._dingtalk_token or not self._dingtalk_secret:
                self.get_logger().debug(
                    "[dingtalk] access_token/secret 未配置，跳过发送")
                return
            # 白名单过滤：person 等不入钉钉
            if kind == "person":
                self.get_logger().debug(
                    "[dingtalk] person 不入钉钉，跳过发送")
                return

            # 构建加签 URL
            timestamp_ms = str(int(time.time() * 1000))
            string_to_sign = f"{timestamp_ms}\n{self._dingtalk_secret}"
            hmac_code = hmac.new(
                self._dingtalk_secret.encode("utf-8"),
                string_to_sign.encode("utf-8"),
                digestmod=hashlib.sha256).digest()
            sign = urllib.parse.quote_plus(base64.b64encode(hmac_code))
            url = (f"https://oapi.dingtalk.com/robot/send"
                   f"?access_token={self._dingtalk_token}"
                   f"&timestamp={timestamp_ms}&sign={sign}")

            # 构建 Markdown 消息
            pic_name = os.path.basename(pic_path)
            ts_readable = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
            # POI_XX_abnormal / POI_XX_uncertain → 拆成 点位 + 类型
            facility = ""   # 从 detail 里解析具体设施（灭火器/安全指示牌）
            if "_abnormal" in kind:
                poi = kind.replace("_abnormal", "")
                # 从 result.detail 里解析具体设施类型
                facility = self._parse_facility(detail)
                kind_label = facility or "设施异常"
            elif "_uncertain" in kind:
                poi = kind.replace("_uncertain", "")
                kind_label = "需人工复核"
            elif kind == "smoking":
                poi = "移动路段"
                kind_label = "违规吸烟"
            elif kind == "air_raid":
                poi = "移动路段"
                kind_label = "防空警报"
            else:
                poi = "—"
                kind_label = kind
            title = f"🔥 巡检异常 | {poi} | {kind_label}"
            text = (
                f"## {title}\n\n"
                f"- **时间**：{ts_readable}\n"
                f"- **点位**：{poi}\n"
                f"- **类型**：{kind_label}\n"
                f"- **抓拍图**：`{pic_name}`\n"
            )
            body = {
                "msgtype": "markdown",
                "markdown": {"title": title, "text": text},
                "at": {"atMobiles": [], "isAtAll": False},
            }

            data = json.dumps(body).encode("utf-8")
            req = urllib.request.Request(
                url, data=data, method="POST",
                headers={"Content-Type": "application/json"})
            with urllib.request.urlopen(req, timeout=3) as resp:
                result = json.loads(resp.read().decode("utf-8"))
                if result.get("errcode") == 0:
                    self.get_logger().info(
                        f"[dingtalk] {kind} 推送成功：{pic_name}")
                else:
                    self.get_logger().warn(
                        f"[dingtalk] {kind} 推送失败：{result}")
        except Exception as exc:  # noqa: BLE001 - 钉钉失败不影响巡检
            self.get_logger().warn(
                f"[dingtalk] {kind} 发送异常：{exc}")

    # 设施英文类名 → 钉钉卡片上显示的中文
    _FACILITY_LABELS = {
        "fire_extinguisher": "灭火器缺失",
        "safety_sign": "安全指示牌异常",
        "smoke": "烟雾检测异常",
        "smoking": "违规吸烟",
        "fire": "火灾检测异常",
    }

    def _parse_facility(self, detail: str) -> str:
        """从视觉 result.detail 里提取具体设施中文名。

        detail 形如：
          "期望目标未全部检出，缺: fire_extinguisher"
          "检出异常类别: smoking, fire"
        返回对应的中文标签（如"灭火器缺失"），匹配不到返回空串。
        """
        if not detail:
            return ""
        for en, zh in self._FACILITY_LABELS.items():
            if en in detail:
                return zh
        return ""

    def _purge_old_captures(self) -> None:
        """删除抓拍目录中超过保留天数的 jpg 文件。

        清理被节流：6h 内最多扫一次目录，避免每次抓拍都遍历。
        retention_days=0 时跳过（禁用自动清理）。
        """
        try:
            if self._capture_retention_days <= 0:
                return
            now = time.monotonic()
            if now - self._capture_last_purge < self._capture_purge_cooldown_s:
                return
            self._capture_last_purge = now
            cutoff = time.time() - self._capture_retention_days * 86400.0
            if not os.path.isdir(self._capture_dir):
                return
            removed = 0
            total = 0
            for name in os.listdir(self._capture_dir):
                if not name.lower().endswith(".jpg"):
                    continue
                total += 1
                path = os.path.join(self._capture_dir, name)
                try:
                    if os.path.getmtime(path) < cutoff:
                        os.remove(path)
                        removed += 1
                except OSError as exc:
                    self.get_logger().warn(
                        f"[capture] 删除旧图失败 {name}: {exc}")
            if removed > 0:
                self.get_logger().warn(
                    f"[capture] 自动清理：删除 {removed} 张超过 "
                    f"{self._capture_retention_days:.0f} 天的抓拍（目录共 {total}）")
        except Exception as exc:  # noqa: BLE001 - 清理失败不影响巡检
            self.get_logger().warn(f"[capture] 清理旧图异常（忽略）：{exc}")

    def _on_audio_alert(self, msg) -> None:
        name = (msg.data or "").strip()
        if not name:
            return
        # 音频节点已做 15s 冷却，这里再加一道 10s 保险丝，避免突刺
        now = time.monotonic()
        last = self._audio_alert_cooldown.get(name, 0.0)
        if now - last < self._audio_alert_min_gap_s:
            return
        speech = SPEECH_TEXTS.get(name, "")
        if not speech:
            self.get_logger().warn(f"[audio] 未知警报 {name}（无对应文案），忽略")
            return
        # 防空警报属确定性异常事件：触发即抓拍当前画面留证
        self._capture_event(name)

        with self._watch_lock:
            self._audio_alert_cooldown[name] = now
            if self._watch_state in ("SPEAK", "INTERVENE"):
                # 防空警报属安全事件，不能丢。但语音服务端同时只受理一个
                # Goal，硬发会被 REJECT，所以排队：等当前播报/介入结束后补播。
                self.get_logger().warn(
                    f"[audio] {name} 检测到，当前正在播报/介入 → 排队补播")
                threading.Thread(
                    target=self._audio_alert_speak_when_idle,
                    args=(name, speech),
                    daemon=True,
                ).start()
                return

            # 导航途中 + 介入可用 → 走停车介入（与吸烟一致）
            if (self._nav_enabled and self._nav_in_flight
                    and self._intervention_enabled):
                self._intervention_snap = {"cls": name, "audio": True}
                self._watch_state = "INTERVENE"
                # 音频节点已完成 3 帧确认+冷却，无需视觉确认帧；
                # 不置 True 会被导航线程当作「预停车未确认」直接恢复，
                # 播报永远发不出去（实测只停车 2.3s 就重新起步）。
                self._intervention_confirmed = True
                self._intervention_event.set()
                self.get_logger().warn(f"[audio] {name} → 停车介入播报")
                return

            # 非导航途中：异步播报，不打断当前流程
            self._watch_state = "SPEAK"
        threading.Thread(
            target=self._audio_alert_speak_only, args=(name, speech),
            daemon=True,
        ).start()

    def _audio_alert_speak_when_idle(self, alert: str, speech: str,
                                     wait_s: float = 30.0) -> None:
        """排队补播：等当前播报/介入结束再播警报，不改 _watch_state。"""
        deadline = time.monotonic() + wait_s
        while time.monotonic() < deadline:
            if self._shutdown or self._stop_event.is_set():
                return
            with self._watch_lock:
                busy = self._watch_state in ("SPEAK", "INTERVENE")
            if not busy:
                break
            time.sleep(0.5)
        else:
            self.get_logger().warn(
                f"[audio] {alert} 排队等待超时（{wait_s:.0f}s），放弃补播")
            return
        self._speak_with_retry(speech, f"音频警报 {alert}")

    def _audio_alert_speak_only(self, alert: str, speech: str) -> None:
        """非导航途中的警报播报（不停车、不介入）。"""
        try:
            self._send_head_cmd("pause")
            self._speak_with_retry(speech, f"音频警报 {alert}")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"音频警报播报失败：{exc}")
        finally:
            with self._watch_lock:
                self._watch_state = "WATCH"
            self._send_head_cmd("resume")

    def _speak_with_retry(self, text: str, label: str,
                          attempts: int = 4, gap_s: float = 2.0) -> bool:
        """音频警报专用播报：语音服务端同时只受理一个 Goal，开始时"开始语"、
        点位结论播报还在播时，警报 Goal 会被 REJECT（警报就丢了），故退避重试。
        """
        if not self._speak_enabled or not text:
            return False
        for i in range(1, attempts + 1):
            if self._shutdown or self._stop_event.is_set():
                return False
            goal = Speak.Goal()
            goal.text = text
            goal.interrupt = True
            suffix = f"（第 {i}/{attempts} 次）" if i > 1 else ""
            self.get_logger().info(f"播报[{label}]：{text}{suffix}")
            try:
                result = self._call(
                    self._speak_client, goal, self._speak_timeout_s,
                    f"播报 {label}")
            except RunAborted:
                raise
            except Exception as exc:  # noqa: BLE001 - 语音不得阻断巡检
                self.get_logger().warn(
                    f"播报{label}失败（{i}/{attempts}）：{exc}")
                if i < attempts:
                    self._sleep(gap_s)
                continue
            if result.success:
                return True
            self.get_logger().warn(
                f"播报未确认完成（不阻断巡检）：state={result.state}，{result.message}")
            return False
        return False

    def _on_detection(self, msg) -> None:
        """detection 话题回调——WATCH 模式核心状态机。

        导航模式下确认吸烟且介入可用时不播报，而是置位介入事件：
        由工作线程的 _navigate 取消当前 goal、停车、接近、播报，再重发原目标。
        """
        with self._watch_lock:
            if self._nav_enabled and not self._watch_active:
                return  # 导航模式下只在移动路段处理检测流
            if getattr(msg, "simulated", False):
                return  # 模拟结果一律忽略，不作为真实触发

            cls = None
            conf = 0.0
            top = None
            try:
                arr = json.loads(msg.detections_json)
                if isinstance(arr, list) and arr:
                    # 同时检出「人」和「吸烟」时吸烟优先：同一帧里吸烟是异常
                    # 事件，不能因为 person 置信度更高（现场常 0.9+ vs 0.7）
                    # 就把吸烟警告丢掉。同类内部仍按 conf 取最高分。
                    priority = {"smoking": 0, "person": 1}
                    candidates = [
                        det for det in arr
                        if det.get("cls", "") in self._scan_classes
                    ]
                    if candidates:
                        candidates.sort(key=lambda det: (
                            priority.get(det.get("cls", ""), 99),
                            -float(det.get("conf", 0.0)),
                        ))
                        top = candidates[0]
                        cls = top.get("cls", "")
                        conf = float(top.get("conf", 0.0))
            except Exception:
                top = None
            if cls is not None:
                snap = parse_detection_snapshot(top)
                if snap is not None:
                    self._latest_det = snap

            now = time.time()

            # 冷却期：跳过直到超时
            if self._watch_state == "COOLDOWN":
                if now - self._watch_last_trigger < self._cooldown_s:
                    return
                self._watch_state = "WATCH"
                self.get_logger().info("冷却结束，恢复扫视")
                return

            if self._watch_state in ("SPEAK", "INTERVENE"):
                # 人形介入期间继续收吸烟帧：现场烟往往在停车、头部抬起后才
                # 进入画面；工作线程会在停车后等一小段，看到烟就改播吸烟。
                if (cls == "smoking" and self._intervention_cls == "person"
                        and snap is not None
                        and float(snap.get("conf", 0.0))
                        >= self._smoking_quick_confirm_conf):
                    self._smoking_during_person = snap
                    self.get_logger().warn(
                        f"人员介入期间检到吸烟 conf={snap['conf']:.2f}"
                        " → 待切换为吸烟播报")
                return

            if self._watch_state == "WATCH":
                if cls is not None:
                    # 行人只在「正前方路线上」才锁定：侧前方的人一锁定，
                    # 头部就会转过去（现场 yaw 曾到 ±1.1rad），正前方的人
                    # 反而出画；确认后播报还会占用冷却。两者都会造成
                    # 「有的时候检测不出来人」。距离上限仍在 person_on_path。
                    if cls == "person" and not self._target_on_path(snap, top):
                        return
                    # 目标还在较远处（> person_on_path.max_range_m）：
                    # 不锁定、不停车、不播报，等车走近再处理。否则远处的人
                    # 会先触发一次播报+冷却，等真正挡路时反而被冷却压住。
                    if (
                        cls in self._intervention_avoid_classes
                        and snap is not None
                        and snap.get("range_valid")
                        and float(snap.get("range_m", 0.0))
                        > self._person_block_range_m
                    ):
                        return
                    # 首次命中 → 锁定头部（直接发 look_at，它自带锁定效果，不用先 pause）
                    pose = self._parse_look_at(msg.detections_json,
                                               self._scan_classes)
                    yaw = pose["yaw"] if pose else 0.0
                    pitch = pose["pitch"] if pose else 0.0
                    self._watch_state = "LOCKED"
                    self._watch_streak_cls = cls
                    self._watch_streak = 1
                    self._watch_hits = [now]
                    self._watch_locked_pose = {"yaw": yaw, "pitch": pitch}
                    self._watch_anchor = parse_detection_center(top)
                    self._watch_lost_at = None
                    self.get_logger().warn(
                        f"首次命中 {cls} conf={conf:.2f} 头部锁定 "
                        f"yaw={yaw:.3f} pitch={pitch:.3f} rad 开始 1/{self._confirm_frames} 确认"
                    )
                    # 只发 look_at（自带锁定，不用先 pause）
                    self._send_head_cmd(f"look_at:{yaw:.4f},{pitch:.4f}")
                    self._intervention_confirmed = False
                    # 高置信度单帧直接确认：现场实测模型对同一个吸烟目标
                    # 常常只响应 1 帧（conf 0.6~0.8），下一帧就完全没有输出，
                    # 等第 2 帧永远等不到（日志：首次命中 0.63 → 0.2s 后丢失）。
                    quick_conf = (
                        self._smoking_quick_confirm_conf
                        if cls == "smoking" else self._quick_confirm_conf)
                    if (
                        conf >= quick_conf
                        and self._intervention_allowed(cls)
                        and cls != "person"
                        and self._target_on_path(snap, top)
                    ):
                        self.get_logger().warn(
                            f"首次命中 {cls} conf={conf:.2f} ≥ "
                            f"{quick_conf:.2f}，单帧直接确认")
                        self._trigger_confirmed(cls, top)
                        return
                    # 首次命中就先停车：不等确认完成，否则确认的这 1~2 秒里
                    # 车还在往前走（现场曾贴到 0.9m 才停）。确认不成立再起步。
                    # 只播报不停车的类别（speak_only_classes）绝不预停车。
                    # person 也预停车：2026-11-03 前往 POI_03 时，7 帧确认
                    # 期间车又走了约 0.5m，停稳时已距人 0.72m；误检造成的
                    # 多余停车由 _NAV_MAX_PRESTOP_UNCONFIRMED 兜底。
                    if (
                        self._intervention_allowed(cls)
                        and self._stop_vehicle_for(cls)
                        and self._target_on_path(snap, top)
                        and conf >= _PRESTOP_MIN_CONF
                        and self._prestop_unconfirmed
                        < _NAV_MAX_PRESTOP_UNCONFIRMED
                        and now >= self._prestop_block_until
                    ):
                        self._prestop_stopped = True
                        self._intervention_event.set()
                        self.get_logger().warn(
                            f"首次命中 {cls} → 先停车等待确认"
                            f"（{self._confirm_window_s:.1f}s 内未确认则恢复导航）")
                return

            if self._watch_state == "LOCKED":
                if cls == "smoking" and self._watch_streak_cls == "person":
                    # 锁定人形期间消防模型才看到烟：升级为吸烟，不能按行人
                    # 放走。吸烟常常只闪 1 帧，达到吸烟单帧门槛就直接确认；
                    # 低于门槛则重新累计，等后续帧。
                    self.get_logger().warn(
                        f"锁定目标由 person 升级为 smoking（conf={conf:.2f}）")
                    self._watch_streak = 1
                    if conf >= self._smoking_quick_confirm_conf:
                        self.get_logger().warn(
                            f"smoking conf={conf:.2f} ≥ "
                            f"{self._smoking_quick_confirm_conf:.2f}，单帧确认")
                        self._trigger_confirmed("smoking", top)
                        return
                    self._watch_streak_cls = "smoking"
                    self._watch_anchor = parse_detection_center(top)
                    self._watch_hits = [now]
                    self._watch_lost_at = None
                    pose = self._parse_look_at(msg.detections_json, {"smoking"})
                    if pose is not None:
                        self._send_head_cmd(
                            f"look_at:{pose['yaw']:.4f},{pose['pitch']:.4f}")
                    return
                if cls == self._watch_streak_cls:
                    anchor = parse_detection_center(top)
                    if (
                        anchor is not None
                        and self._watch_anchor is not None
                        and math.hypot(
                            anchor[0] - self._watch_anchor[0],
                            anchor[1] - self._watch_anchor[1],
                        ) > self._confirm_pos_tol
                    ):
                        # 框跳到画面别处：不是同一目标，重新从 1 帧开始计
                        self.get_logger().info(
                            f"目标 {cls} 位置跳变超过 "
                            f"{self._confirm_pos_tol:.2f}（画面归一化），重新计数")
                        self._watch_anchor = anchor
                        self._watch_hits = [now]
                        self._watch_streak = 1
                        self._watch_lost_at = None
                        return
                    # 同类 → 记一次命中；窗口内累计，单帧丢失不清零
                    hits = [t for t in self._watch_hits
                            if now - t <= self._confirm_window_s]
                    hits.append(now)
                    self._watch_hits = hits
                    self._watch_streak = len(hits)
                    self._watch_lost_at = None
                    span = now - self._watch_hits[0]
                    if (self._watch_streak >= self._confirm_frames
                            and span >= self._confirm_min_span_s):
                        self._trigger_confirmed(cls, top)
                    else:
                        self.get_logger().info(
                            f"目标 {cls} 窗口内 {self._watch_streak}/"
                            f"{self._confirm_frames} 帧 跨度 {span:.1f}s/"
                            f"{self._confirm_min_span_s:.1f}s conf={conf:.2f}"
                        )
                else:
                    # 目标丢失或换类（窗口内的命中不清零，复现后继续累计）
                    if self._watch_lost_at is None:
                        self._watch_lost_at = now
                        self.get_logger().info(
                            f"目标 {self._watch_streak_cls} 丢失，窗口内已锁定 "
                            f"{self._watch_streak}/{self._confirm_frames} 帧，"
                            f"等待复现（超时 {self._lost_timeout_s}s）"
                        )
                    elif now - self._watch_lost_at > self._lost_timeout_s:
                        # 超时 → 放弃，恢复扫视
                        self._watch_state = "WATCH"
                        self._watch_streak_cls = None
                        self._watch_streak = 0
                        self._watch_hits = []
                        self._watch_lost_at = None
                        self.get_logger().warn(
                            f"目标丢失超过 {self._lost_timeout_s}s → 放弃本次触发，恢复扫视"
                        )
                        self._send_head_cmd("resume")

    def _trigger_confirmed(self, cls: str, top) -> None:
        """已确认目标：置状态并交给工作线程执行停车 / 接近 / 播报。

        调用方必须持有 _watch_lock。单帧快速确认与多帧确认共用这一处，
        保证两条路径的状态与日志完全一致。
        """
        self._watch_last_trigger = time.time()
        # 确认瞬间先抓拍：此时头部仍锁定目标、画面里还有人，
        # 不要等停车停稳 + 等快照（2~3s）后再拍，否则人可能已离开画面。
        self._capture_event(cls)
        self._intervention_snap = parse_detection_snapshot(top)
        frames = self._watch_streak
        stop_vehicle = self._stop_vehicle_for(cls)
        snap = self._intervention_snap
        if (cls in self._intervention_avoid_classes
                and not self._target_on_path(snap)):
            # 不在行进路线上：不取消导航、不停车。行人不再播报、也不进入
            # 冷却——侧前方路过的人一播报就会吃掉冷却，之后真正站到
            # 路中间的人反而漏报（现场「有的时候检测不出来人」）。吸烟是
            # 安全异常事件，仍按原逻辑只播报后继续巡检。
            self._intervention_confirmed = False
            self._intervention_snap = None
            self._watch_streak_cls = None
            self._watch_streak = 0
            self._watch_hits = []
            self._watch_lost_at = None
            detail = ""
            if snap is not None:
                detail = (
                    f"bearing={snap.get('bearing_rad', 0.0):+.3f}rad "
                    f"range={snap.get('range_m', 0.0):.2f}m "
                    f"range_valid={snap.get('range_valid')}")
            if cls == "person":
                # 侧前方路人本来不播报（避免路人吃掉冷却导致漏报真正挡路
                # 的人）；但如果首帧预停车已经把车刹停过，现场只看到「车停
                # 了一下又走」会以为没反应，这里补播一句让行提示，不占冷却、
                # 不计介入次数。
                stopped = self._prestop_stopped
                self._prestop_stopped = False
                self._watch_state = "WATCH"
                if stopped:
                    self.get_logger().warn(
                        f"person 连续 {frames} 帧确认完成 → 不在行进路线上"
                        f"（{detail}），已预停车，补播让行提示（不占冷却）")
                    self._speak_async(
                        SPEECH_TEXTS["person"], "person 侧前方停车")
                else:
                    self.get_logger().warn(
                        f"person 连续 {frames} 帧确认完成 → 不在行进路线上"
                        f"（{detail}），忽略（不播报、不占冷却）")
                self._send_head_cmd("resume")
                return
            self._prestop_stopped = False
            self._watch_state = "COOLDOWN"
            self.get_logger().warn(
                f"{cls} 连续 {frames} 帧确认完成 → 不在行进路线上"
                f"（{detail}），只播报后继续巡检")
            self._speak_async(
                SPEECH_TEXTS.get(cls, f"发现目标：{cls}"), f"{cls} 播报")
            self._send_head_cmd("resume")
            return
        if stop_vehicle and self._intervention_allowed(cls):
            # 停车介入：这里只置位，真正的停车/接近/播报在工作线程里串行做
            self._prestop_stopped = False
            self._intervention_confirmed = True
            self._intervention_cls = cls
            self._smoking_during_person = None
            self._watch_state = "INTERVENE"
            self._intervention_event.set()
            self.get_logger().warn(
                f"{cls} 连续 {frames} 帧确认完成 → 停车介入"
                f"（先停轨道，再接近播报，最后回原目标）")
        elif stop_vehicle and self._nav_in_flight:
            # 每点位/本轮介入次数已用完：仍要停车播报（不接近），
            # 不能像以前那样边走边播报从人旁边擦过去
            self._prestop_stopped = False
            self._intervention_confirmed = True
            self._intervention_cls = cls
            self._smoking_during_person = None
            self._watch_state = "INTERVENE"
            self._speak_only_pending = True
            self._intervention_event.set()
            self.get_logger().warn(
                f"{cls} 连续 {frames} 帧确认完成 → "
                f"介入次数已用完，停车播报后继续（不接近）")
        elif self._nav_enabled and self._nav_in_flight:
            # 只播报不停车：抓拍 + 后台播报，绝不取消导航，
            # 车继续前往原目标执行巡检任务（不靠边、不接近人员）
            self._prestop_stopped = False
            self._intervention_confirmed = False
            self._watch_state = "COOLDOWN"
            self._watch_streak_cls = None
            self._watch_streak = 0
            self._watch_hits = []
            self._watch_lost_at = None
            self.get_logger().warn(
                f"{cls} 连续 {frames} 帧确认完成 → 只播报不停车，"
                f"继续执行巡检任务")
            # 抓拍已在方法入口统一完成，这里直接播报
            self._speak_async(
                SPEECH_TEXTS.get(cls, f"发现目标：{cls}"), f"{cls} 播报")
            self._send_head_cmd("resume")
        else:
            self._intervention_confirmed = True
            self._watch_state = "SPEAK"
            self.get_logger().warn(f"{cls} 连续 {frames} 帧确认完成 → 播报")
            # 异步播报（别阻塞回调）
            threading.Thread(
                target=self._watch_speak_and_resume,
                args=(cls,),
                daemon=True,
            ).start()

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
                # person 等已注册固定文案的类别用固定文案，避免播英文类名
                goal.text = SPEECH_TEXTS.get(cls, f"发现目标：{cls}")
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
                self._watch_hits = []
                active = not self._nav_enabled or self._watch_active
            if active:
                self._send_head_cmd("resume")
                self.get_logger().info("恢复扫视，继续等待目标")
            else:
                self.get_logger().info("检测门控已关闭（已到点定检），本次不再恢复扫视")

    # ------------------------------------------------------------------
    # 吸烟人员介入：停车 → 接近人员前方 → 播报 → 回原轨道目标
    # ------------------------------------------------------------------
    def _handle_intervention(self, poi: str, allow_approach: bool = True,
                             count: bool = True) -> None:
        """执行一次完整介入。

        调用时导航 goal 已结束（被取消或已到点），底盘正在/已经停住。
        这里只调用 Action（接近 / 播报），不直接碰底盘，符合「单一指挥者」约定。

        count=False：介入次数已用尽时的「只停车播报」，不再累加计数。
        触发源有两种：
          - 视觉吸烟：cls='smoking'，无 audio 键，允许接近
          - 音频警报：cls='air_raid' 等，snap 含 audio=True，跳过视觉快照与接近
        """
        snap = self._intervention_snap
        self._intervention_snap = None
        cls = (snap or {}).get("cls") or "smoking"
        is_audio = bool((snap or {}).get("audio"))
        allow_approach = allow_approach and not is_audio   # 音频警报永远不接近
        self._intervention_cls = cls
        with self._watch_lock:
            self._smoking_during_person = None
        if is_audio:
            label = "音频警报"
        elif self._avoid_class(cls):
            label = "人员避让"
        else:
            label = "吸烟介入"
        self._set_state(RunState.RUNNING, f"intervene:{poi}")
        self.get_logger().warn(
            f"{label}开始：点位={poi}，类别={cls}，"
            f"接近={'允许' if allow_approach else '跳过（非视觉源）'}"
        )
        stopped_at = time.monotonic()
        try:
            # 1) 等底盘彻底停稳：取消到真正停住之间还有减速过程
            self._sleep(self._intervention_settle_s)
            # 1.5) 人形介入先别急着播「请让一下」：现场烟往往在停车、头部
            #      抬起后才进入画面（日志：停车后约 2.4s 出现 smoking=0.77）。
            #      在这段窗口内看到吸烟就切换成吸烟播报/绕行。
            if (cls == "person" and not is_audio
                    and self._person_smoking_grace_s > 0.0):
                self.get_logger().info(
                    f"人员介入：停车后补看吸烟 "
                    f"{self._person_smoking_grace_s:.1f}s")
                deadline = time.monotonic() + self._person_smoking_grace_s
                while time.monotonic() < deadline:
                    with self._watch_lock:
                        smoking = self._smoking_during_person
                    if smoking is not None:
                        cls = "smoking"
                        snap = smoking
                        label = "吸烟介入"
                        self.get_logger().warn(
                            "人员介入期间检到吸烟 → 切换为吸烟播报/绕行")
                        break
                    if self._shutdown or self._stop_event.is_set():
                        break
                    time.sleep(0.1)
            # 2) 视觉介入用停车后的最新一帧覆盖确认快照；音频源无视觉
            if is_audio:
                self.get_logger().info("音频警报，跳过视觉快照等待")
            else:
                snap = self._await_intervention_snapshot(cls, snap, stopped_at)
                # 吸烟属确定性异常：停车停稳后抓拍当前画面留证
                #（音频源已在 _on_audio_alert 里抓过，不重复）
                self._capture_event(cls)
            # 2.5) 停车后的快照可能刷新了测距/方位，重新判断目标是否真的
            # 挡在行进路线上。确认瞬间缺有效测距时按保守的「在路径上」
            # 处理，停车后常发现人其实偏在路外（现场 1.27m/bearing
            # -0.768rad，横向偏移 0.88m），这时再横移会白走一段并把车
            # 推到走廊边（门框），必须改为只播报后继续。
            off_path_after_stop = (
                not is_audio
                and isinstance(snap, dict)
                and snap.get("bearing_valid")
                and not self._target_on_path(snap)
            )
            if off_path_after_stop:
                self.get_logger().warn(
                    f"停车后重新判定：{cls} 不在行进路线上"
                    f"（bearing={float(snap.get('bearing_rad', 0.0)):+.3f}rad "
                    f"range={float(snap.get('range_m', 0.0)):.2f}m），"
                    "不绕行/不接近，只播报后继续")
            # 3) 视觉介入：绕行类别先播警告再横移；其余类别保持原「接近」流程
            avoided = None
            speech = SPEECH_TEXTS.get(cls) or SPEECH_TEXTS.get("smoking", "")
            avoid_flow = (allow_approach and self._avoid_class(cls)
                          and not off_path_after_stop)
            if off_path_after_stop:
                # 人已不在路线上：person 按既有策略不播报（不占冷却）；
                # 吸烟属安全异常，仍播警告，但不横移。
                if cls != "person":
                    self._speak(speech, f"{label} {poi}")
            elif avoid_flow:
                # 车已停稳：先让人听清警告，再侧向横移绕开
                self._speak(speech, f"{label} {poi}")
                avoided = self._avoid_target(snap)
            elif allow_approach:
                self._approach_target(snap)
            else:
                self.get_logger().info("跳过接近，直接原地播报")
            # 4) 播报：绕行类别已在横移前播过警告，这里只补「已避开」
            if is_audio:
                # 警报不能因语音服务端忙而被丢弃，走带重试的播报
                self._speak_with_retry(speech, f"{label} {poi}")
            elif not avoid_flow and not off_path_after_stop:
                self._speak(speech, f"{label} {poi}")
            # 5) 视觉避让：横移完成后再播「已避开并继续」，现场能明确
            #    听到机器人已从人员侧向绕开、即将继续巡检。
            #    绕行被跳过/失败（avoided=False）或只停车播报时并没有
            #    真的绕开，不能播这句，否则现场会以为已经避让过了。
            resume_text = SPEECH_TEXTS.get(f"{cls}_resume", "")
            if avoid_flow and avoided is not False and resume_text:
                self._speak(resume_text, f"{label} {poi} 恢复")
            # 6) 人形介入都走完了才出现吸烟：至少补播吸烟警告，不能悄悄放过
            if cls == "person":
                with self._watch_lock:
                    extra = self._smoking_during_person
                if extra is not None:
                    self.get_logger().warn(
                        "人员介入结束后检到吸烟 → 补播吸烟警告")
                    self._speak(SPEECH_TEXTS["smoking"], f"{label} 吸烟补充")
        except RunAborted:
            raise
        except Exception as exc:  # noqa: BLE001 - 介入失败不得阻断整轮巡检
            self.get_logger().error(f"{label}失败（继续巡检）：{exc}")
        finally:
            stopped = self._shutdown or self._stop_event.is_set()
            with self._watch_lock:
                if count:
                    self._intervention_counts[poi] = (
                        self._intervention_counts.get(poi, 0) + 1)
                    self._intervention_total += 1
                self._watch_state = "COOLDOWN"
                self._watch_last_trigger = time.time()
                self._watch_streak_cls = None
                self._watch_streak = 0
                self._watch_hits = []
                self._watch_lost_at = None
                self._intervention_cls = None
                self._smoking_during_person = None
            if not stopped:
                # 6) 恢复移动路段扫视；随后 _navigate 重发原目标沿轨道回去
                self._transit_watch_on()
            self.get_logger().warn(
                f"{label}结束：点位={poi}，本轮累计 "
                f"{self._intervention_total} 次，冷却 {self._cooldown_s:.0f}s"
            )

    def _await_intervention_snapshot(self, cls, fallback, fresh_after):
        """停车后等一帧更新的有效测距；没有就沿用确认瞬间的快照。"""
        wait_s = max(0.0, self._intervention_snapshot_wait_s)
        deadline = time.monotonic() + wait_s
        while wait_s > 0 and time.monotonic() < deadline:
            if self._shutdown or self._stop_event.is_set():
                break
            with self._watch_lock:
                snap = self._latest_det
            if (
                snap is not None
                and snap.get("cls") == cls
                and snap.get("range_valid")
                and snap.get("t", 0.0) >= fresh_after
            ):
                self.get_logger().info(
                    f"停车后取到更新测距：range={snap['range_m']:.2f}m "
                    f"bearing={snap['bearing_rad']:+.3f}rad")
                return snap
            time.sleep(0.1)
        if fallback is not None:
            self.get_logger().info(
                "停车后未取到更新的有效测距，沿用确认瞬间的快照")
        return fallback

    def _avoid_target(self, snap) -> bool:
        """向远离人员的一侧横移一次，随后由调用方继续原目标。

        复用导航侧 ApproachTarget：standoff_m == range_m 时前进量为 0，
        只剩 side_offset_m 的侧向横移；方向按方位角取「远离人员」的一侧。
        没有有效方位角/测距时不盲动，返回 False（调用方只原地播报）。
        """
        if not self._approach_available:
            self.get_logger().warn("接近服务端不可用，跳过绕行，只原地播报")
            return False
        if (
            not snap
            or not snap.get("range_valid")
            or not snap.get("bearing_valid")
        ):
            self.get_logger().warn(
                "无有效测距/方位（深度未就绪或头部姿态不可用），"
                "跳过绕行，只原地播报")
            return False
        bearing = float(snap["bearing_rad"])
        range_m = float(snap["range_m"])
        if (
            not (math.isfinite(bearing) and math.isfinite(range_m))
            or range_m <= 0.0
        ):
            self.get_logger().warn(
                f"绕行参数非法：bearing={bearing} range={range_m}，只原地播报")
            return False
        # 人偏左（bearing>0）→ 从右侧绕；人偏右 → 从左侧绕；正前方默认右侧
        side = -self._avoid_offset_m if bearing >= 0.0 else self._avoid_offset_m
        goal = ApproachTarget.Goal()
        goal.bearing_rad = bearing
        goal.range_m = range_m
        # 前进量 = range - standoff：两者相等 → 只侧移，不向人靠近
        goal.standoff_m = range_m
        goal.side_offset_m = side
        goal.timeout_s = float(self._intervention_timeout_s)
        self.get_logger().warn(
            f"绕开人员：bearing={bearing:+.3f}rad range={range_m:.2f}m → "
            f"请求向{'右' if side < 0 else '左'}侧横移 {abs(side):.2f}m"
            f"（导航侧会按走廊余量选择实际方向）")
        try:
            result = self._call(
                self._approach_client, goal,
                goal.timeout_s + 20.0, "绕开人员")
        except RunAborted:
            raise
        except Exception as exc:  # noqa: BLE001 - 绕行失败不阻断播报
            self.get_logger().warn(f"绕行未完成（不阻断播报）：{exc}")
            return False
        if result.success and result.phase in ("arrived", "retreated"):
            self.get_logger().info(
                f"已绕开人员：phase={result.phase} | {result.message}")
            return True
        self.get_logger().warn(
            f"未绕开（phase={result.phase}），改为原地播报：{result.message}")
        return False

    def _approach_target(self, snap) -> None:
        """调用导航侧的 ApproachTarget：走到人员前方并转向人员。"""
        if not self._approach_available:
            self.get_logger().warn("接近服务端不可用，跳过接近，只原地播报")
            return
        if not snap or not snap.get("range_valid"):
            self.get_logger().warn(
                "无有效测距/方位（深度未就绪或头部姿态不可用），"
                "跳过接近，只原地播报")
            return
        goal = ApproachTarget.Goal()
        goal.bearing_rad = float(snap["bearing_rad"])
        goal.range_m = float(snap["range_m"])
        # 0 = 用 pois.yaml 的 approach.standoff_m（距离唯一真源在导航侧）
        goal.standoff_m = 0.0
        # 0 = 用 pois.yaml 的 approach.side_offset_m：站位相对视线侧移，
        # 从吸烟者侧向走开而不是正面穿过人
        goal.side_offset_m = 0.0
        goal.timeout_s = float(self._intervention_timeout_s)
        self.get_logger().info(
            f"接近人员：bearing={goal.bearing_rad:+.3f}rad "
            f"range={goal.range_m:.2f}m timeout={goal.timeout_s:.0f}s")
        try:
            result = self._call(
                self._approach_client, goal,
                goal.timeout_s + 20.0, "接近人员")
        except RunAborted:
            raise
        except Exception as exc:  # noqa: BLE001 - 接近失败不阻断播报
            self.get_logger().warn(f"接近未完成（不阻断播报）：{exc}")
            return
        if result.success and result.phase == "arrived":
            self.get_logger().info(
                f"接近完成：距人员 {result.final_range_m:.2f}m | {result.message}")
        elif result.success and result.phase == "retreated":
            self.get_logger().warn(
                f"人员已贴得太近，已侧向退让让开通道："
                f"距人员 {result.final_range_m:.2f}m | {result.message}")
        else:
            self.get_logger().warn(
                f"未接近（phase={result.phase}），改为原地播报：{result.message}")

    def _wait_action(self, client, name: str, timeout_s: float) -> bool:
        """等待 Action 服务端就绪，期间每 5s 打印一次进度。

        旧实现直接阻塞 wait_for_server(60s)，现场只看到 60 秒空白后突然
        报「不可用」，无法判断是哪个服务没起来。这里把等待过程显式化。
        """
        timeout_s = max(0.0, float(timeout_s))
        started = time.monotonic()
        deadline = started + timeout_s
        next_report = started + 5.0
        while time.monotonic() < deadline:
            if client.wait_for_server(timeout_sec=0.5):
                return True
            if self._shutdown or self._stop_event.is_set():
                return False
            now = time.monotonic()
            if now >= next_report:
                self.get_logger().warn(
                    f"等待 {name} 就绪：{now - started:.0f}/{timeout_s:.0f}s")
                next_report = now + 5.0
        return bool(client.wait_for_server(timeout_sec=0.1))

    def _wait_servers(self) -> None:
        if self._nav_enabled:
            if not self._wait_action(
                self._nav_client, "导航 Action", self._wait_servers_s
            ):
                raise RuntimeError(
                    "导航 Action 服务端不可用（请确认 navigate_to_server 已启动，"
                    "见 /home/nvidia/logs/nav.log）")
        else:
            self.get_logger().info("nav_enabled=false，跳过导航 Action 服务端等待")
        # 接近是导航节点的第二个 Action：不可用时介入降级为「只停车播报」
        if self._nav_enabled and self._intervention_enabled:
            ready = self._wait_action(
                self._approach_client, "接近 Action",
                min(5.0, self._wait_servers_s))
            self._approach_available = bool(ready)
            if not ready:
                self.get_logger().warn(
                    "接近 Action 服务端不可用：吸烟介入只停车播报，不靠近人员")
        else:
            self._approach_available = False
        if not self._wait_action(
            self._inspect_client, "视觉 Action", self._wait_servers_s
        ):
            raise RuntimeError(
                "视觉 Action 服务端不可用（请确认 inspect_action_server 已启动，"
                "见 /home/nvidia/logs/vision.log）")
        if not self._speak_enabled:
            self.get_logger().info("播报已关闭（speak_enabled=false）")
            return
        # 语音是旁路能力：服务端缺失只关掉播报，不让整轮巡检起不来
        if not self._wait_action(
            self._speak_client, "语音 Action", self._wait_servers_s
        ):
            self._speak_enabled = False
            self.get_logger().warn("语音 Action 服务端不可用，本轮只记录日志、不播报")

    @staticmethod
    def _is_transient_nav_error(text: str) -> bool:
        """导航端「前置校验未通过 / 连不上底盘」属于可自愈的瞬态故障。

        导航服务端可能先于底盘就绪，校验失败后由后台周期重试恢复；
        编排层这里多等一会儿再重试，避免整轮巡检刚播报就被判失败。
        """
        return any(marker in text for marker in (
            "前置校验未通过", "Cannot connect", "[3001]"))

    def _navigate_start(self) -> None:
        """前往起点：失败/受阻都不能把整轮巡检判死。

        起点只是集合点：导航侧对「连续被底盘立即拒绝」会返回 blocked 并
        推进点位顺序，这里直接开始巡检点位；其他失败先重试一次，
        仍失败才抛出（此时顺序状态未推进，后续点位无法导航）。
        前置校验/底盘连接这类瞬态故障给导航端后台重试留时间，多试几次。
        """
        start = self._cfg["start_poi"]
        attempts = 3
        for attempt in range(1, attempts + 1):
            try:
                self._navigate(start, is_return=False)
                return
            except RunAborted:
                raise
            except NavBlocked as exc:
                self.get_logger().warn(
                    f"前往起点受阻（{exc}），跳过起点直接开始巡检")
                self._results[start] = "未到位"
                return
            except Exception as exc:  # noqa: BLE001
                transient = self._is_transient_nav_error(str(exc))
                if attempt >= attempts or (not transient and attempt >= 2):
                    raise
                if transient:
                    self.get_logger().warn(
                        f"前往起点失败（{exc}），1.5s 后重试"
                        f"（{attempt + 1}/{attempts}）")
                    self._sleep(1.5)
                else:
                    self.get_logger().warn(f"前往起点失败（{exc}），重试一次")

    def _navigate_inspect(self, poi: str) -> bool:
        """前往巡检点：受阻（phase=blocked）跳过该点，其余失败重试一次。

        导航层已对受阻做过「取消→回退→重发」自恢复，并推进了点位顺序，
        因此这里不再重试；其他失败（瞬时拒绝、超时、到位偏差等）仍按原有
        语义重试一次，重试再失败则抛出让本轮巡检终止。
        返回 True 表示已到达；False 表示已跳过，调用方继续下一个点位。
        """
        try:
            self._navigate(poi, is_return=False)
            return True
        except NavBlocked as exc:
            return self._skip_blocked(poi, exc)
        except RunAborted:
            raise
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"巡检点 {poi} 未到达（{exc}），重试一次")
        try:
            self._navigate(poi, is_return=False)
            return True
        except NavBlocked as exc:
            return self._skip_blocked(poi, exc)

    def _skip_blocked(self, poi: str, exc: Exception) -> bool:
        """记录跳过并播报；返回 False 供调用方 continue。"""
        self.get_logger().error(
            f"巡检点 {poi} 受阻（{exc}），跳过该点并继续后续流程")
        self._results[poi] = SKIPPED_TEXT
        self._speak(SPEECH_TEXTS["nav_blocked"], f"{poi} 受阻跳过")
        return False

    def _navigate(self, poi: str, is_return: bool) -> None:
        """前往点位；途中确认吸烟时停车介入，介入后重新发同一点位回到轨道。

        导航服务端的顺序状态只在成功后推进，被取消的目标可以原样重发，
        因此「介入 → 回到原目标」不会打乱点位顺序。
        """
        action = "返回起点" if is_return else "前往"
        interventions = 0
        with self._watch_lock:
            # 每段路重新允许介入（上一段的抑制标记不外溢）
            self._intervention_suppress = False
            # 预停车计数/冷却/确认标记同样按路段重置
            self._prestop_unconfirmed = 0
            self._prestop_block_until = 0.0
            self._prestop_stopped = False
            self._intervention_confirmed = False
            self._speak_only_pending = False
        while True:
            goal = NavigateTo.Goal()
            goal.poi = poi
            goal.timeout_s = 0.0
            goal.is_return = bool(is_return)
            # 本段此前因吸烟介入停过车：重发改点对点直达点位，
            # 不让底盘退回上轨点后再沿轨道从人员身上重走一遍
            goal.use_point = bool(not is_return and self._point_resume_poi == poi)
            self._set_state(
                RunState.RUNNING, "return" if is_return else f"nav:{poi}"
            )
            self.get_logger().info(f"{action}：{poi}")
            with self._watch_lock:
                # 介入事件只在导航在途时由 _on_detection 置位，
                # 两处都在 _watch_lock 内读写，避免上一段的残留误触发
                self._nav_in_flight = True
                self._nav_poi = poi
                self._intervention_event.clear()

            # 巡检点：车一停稳（导航发 arrived 反馈）就先停扫视，不等
            # NavigateTo 走完到点转向与停留；否则现场会看到车到了头还在扫。
            # 回调里只发一条指令，不等待头部停稳（等待在主流程 _stop_head 里）。
            # 所有路段都监听 detour 相位：受阻绕行时后台播报一次「正在绕行」，
            # 回调线程绝不阻塞、也不走 _call（否则会改写正在导航的 goal_handle）。
            head_stopped = threading.Event()
            detour_spoken = threading.Event()
            stop_head_on_arrive = (
                not is_return and poi in self._cfg.get("inspect_pois", []))

            def on_feedback(_feedback):
                phase = getattr(_feedback, "phase", "")
                if phase == "detour":
                    if not detour_spoken.is_set():
                        detour_spoken.set()
                        self._speak_async(
                            SPEECH_TEXTS["nav_obstacle"], f"{poi} 遇障绕行")
                    return
                if phase != "arrived" or head_stopped.is_set():
                    return
                if not stop_head_on_arrive:
                    head_stopped.set()
                    return
                head_stopped.set()
                self._send_head_cmd("stop")

            try:
                result = self._call(
                    self._nav_client, goal, self._nav_goal_timeout_s,
                    f"{action} {poi}", abort_event=self._intervention_event,
                    feedback_cb=on_feedback,
                )
            finally:
                with self._watch_lock:
                    # 先只读不清理：预停车还要靠这个事件判断是否被打断
                    interrupted = self._intervention_event.is_set()

            arrived = bool(result.success) or result.phase == "arrived"

            confirmed = self._intervention_confirmed
            try:
                if interrupted and not confirmed and not arrived:
                    # 首次命中先停车：车已停住，继续把确认窗口等完。等待期间
                    # _nav_in_flight 必须保持 True，检测回调才能继续累计确认帧。
                    wait_s = min(self._confirm_window_s, 2.0) + 0.3
                    self.get_logger().warn(
                        f"{action} {poi} 首次命中已先停车，等待确认"
                        f"（≤{wait_s:.1f}s）")
                    deadline = time.monotonic() + wait_s
                    while (
                        not self._intervention_confirmed
                        and time.monotonic() < deadline
                    ):
                        if self._shutdown or self._stop_event.is_set():
                            # stop/关闭要立刻生效，不能把车停在原地等满窗口
                            raise RunAborted(f"{action} {poi} 等待确认时收到 stop")
                        time.sleep(0.1)
            finally:
                # 无论正常走完还是被 stop 打断，都要复位在途/事件状态，
                # 避免残留 _nav_in_flight=True 或已置位事件影响下一段
                with self._watch_lock:
                    self._nav_in_flight = False
                    interrupted = (
                        interrupted
                        or self._intervention_event.is_set()
                        or self._intervention_confirmed
                    )
                    confirmed = self._intervention_confirmed
                    self._intervention_event.clear()
                    self._intervention_confirmed = False

            if arrived:
                self._point_resume_poi = None
                if confirmed:
                    speak_only = self._take_speak_only()
                    # 取消请求到得太晚：已经到点，就地介入
                    # （不离开点位，保证后续视觉检查有效）
                    self.get_logger().warn(
                        f"{poi} 在取消生效前已到点，改为原地介入、不再接近")
                    self._handle_intervention(
                        poi, allow_approach=False, count=not speak_only)
                    return
                self.get_logger().info(f"已到达：{poi}")
                if detour_spoken.is_set():
                    self._speak(
                        SPEECH_TEXTS["nav_detour_ok"], f"{poi} 绕行完成")
                return

            if not interrupted:
                message = (
                    f"{action} {poi} 失败：phase={result.phase}，"
                    f"{result.message}")
                if result.phase == "blocked":
                    raise NavBlocked(message)
                raise RuntimeError(message)

            if not confirmed:
                # 预停车后未确认（误报 / 人员已走开）：恢复扫视、重新起步
                with self._watch_lock:
                    self._prestop_unconfirmed += 1
                    # 本次预停车已结束，避免标记残留到下一段/下一次命中
                    self._prestop_stopped = False
                    # 与 _on_detection 的 now（time.time）保持同一时钟
                    self._prestop_block_until = (
                        time.time() + _NAV_PRESTOP_COOLDOWN_S)
                    self._watch_state = "WATCH"
                    self._watch_streak_cls = None
                    self._watch_streak = 0
                    self._watch_hits = []
                    self._watch_lost_at = None
                self._send_head_cmd("resume")
                # 停车位置可能已偏离轨道：重发改走点对点直达，
                # 避免底盘先退回上轨点、再从原路重新碾过去
                self._point_resume_poi = poi
                self.get_logger().info(
                    f"{action} {poi} 首次命中未确认，恢复扫视并重新起步"
                    f"（本段未确认 {self._prestop_unconfirmed} 次）")
                continue

            speak_only = self._take_speak_only()
            if speak_only:
                self.get_logger().warn(
                    f"{action} {poi} 已因介入停车（介入次数已用完，"
                    f"phase={result.phase}）：停车播报后继续，不绕行不接近")
            else:
                self.get_logger().warn(
                    f"{action} {poi} 已因介入停车（phase={result.phase}），"
                    f"执行人员避让")
            # 停车位置可能已偏离轨道：重发改为点对点，
            # 沿轨道重走会让底盘先退回上轨点、再从人员所在处碾过去
            self._point_resume_poi = poi
            self._handle_intervention(
                poi, allow_approach=not speak_only, count=not speak_only)
            interventions += 1
            if interventions >= _NAV_MAX_INTERVENTIONS_PER_LEG:
                # 配置可能写成 0（不限）：这里再加一道路段硬上限，
                # 否则同一段路会无限「停车→接近→播报→重发」
                with self._watch_lock:
                    self._intervention_suppress = True
                self.get_logger().warn(
                    f"{poi} 同一路段已介入 {interventions} 次，达到安全上限，"
                    "剩余路程只播报、不再停车接近")
            self.get_logger().info(f"介入结束，重新发起原目标：{poi}")

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
                # 调用失败≠判定异常：按"不确定"播报，引导人工复核；
                # 复核需要现场画面，同样抓拍留证
                self._capture_event(f"{poi}_uncertain")
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
        # 定点检查结论为「异常」（如灭火器缺失、安全出口指示灯不在）：
        # 确定性异常，抓拍当前画面留证（status 枚举见 STATUS_CN，1=异常）
        # result.detail 里有具体设施类型，钉钉卡片上解析显示
        if int(result.status) == 1:
            self._capture_event(f"{poi}_abnormal", detail=result.detail)
        elif int(result.status) != 0:
            # 不确定/超时/无图像/无模型/错误：给不出确凿结论，需人工复核，
            # 一并抓拍留证；正常(0)不拍
            self._capture_event(f"{poi}_uncertain", detail=result.detail)
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

    def _speak_async(self, text: str, label: str) -> None:
        """后台线程播报一条文本，供导航反馈回调使用。

        反馈回调必须立刻返回，不能走 `_call`——那里会改写共享的
        `_goal_handle`，导致停车 / 终止指令取消不到正在导航的目标。
        这里只发一条独立的 Speak goal，不登记 goal_handle；任何失败只告警。
        """
        if not self._speak_enabled or not text:
            return

        def worker() -> None:
            try:
                goal = Speak.Goal()
                goal.text = text
                goal.interrupt = True
                send = self._speak_client.send_goal_async(goal)
                if not self._wait_future(send, self._speak_timeout_s):
                    self.get_logger().warn(f"后台播报[{label}]发送超时")
                    return
                handle = send.result()
                if handle is None or not handle.accepted:
                    self.get_logger().warn(f"后台播报[{label}]被拒绝")
                    return
                result_future = handle.get_result_async()
                if not self._wait_future(result_future, self._speak_timeout_s):
                    self.get_logger().warn(f"后台播报[{label}]未确认完成")
                    return
                if not result_future.result().result.success:
                    self.get_logger().warn(f"后台播报[{label}]未确认完成")
            except Exception as exc:  # noqa: BLE001 - 语音不得阻断巡检
                self.get_logger().warn(f"后台播报[{label}]失败（不阻断）：{exc}")

        threading.Thread(
            target=worker, name=f"speak-async-{label}", daemon=True).start()

    # ------------------------------------------------------------------
    # Action 调用
    # ------------------------------------------------------------------
    def _call(self, client: ActionClient, goal, timeout_s: float, label: str,
              abort_event=None, feedback_cb=None):
        """发 goal 并等结果。

        abort_event 被置位时主动取消 goal 并等服务端返回 cancelled 结果，
        由调用方决定「介入后重发同一目标」还是「正式失败」；
        stop / 关闭仍然抛 RunAborted，语义不变。
        feedback_cb 只在需要按反馈提前动作时传入（如到点前停头部扫视），
        回调里只做发布类的非阻塞动作，不做等待，避免占住执行器线程。
        """
        send_future = client.send_goal_async(
            goal, feedback_callback=feedback_cb)
        if not self._wait_future(send_future, timeout_s):
            if self._stop_event.is_set():
                raise RunAborted(f"{label} 收到 stop 指令")
            if self._shutdown:
                raise RunAborted(f"{label} 节点关闭")
            raise TimeoutError(f"{label} 发送超时")
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError(f"{label} 被拒绝")

        with self._goal_lock:
            self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        try:
            if self._wait_future(result_future, timeout_s, abort_event):
                return result_future.result().result

            interrupted = abort_event is not None and abort_event.is_set()
            goal_handle.cancel_goal_async()
            if self._stop_event.is_set():
                raise RunAborted(f"{label} 收到 stop 指令")
            if self._shutdown:
                raise RunAborted(f"{label} 节点关闭")
            if interrupted:
                # 必须等服务端停车并回 cancelled 结果，别在车还在动时就往下走
                if self._wait_future(result_future, self._cancel_grace_s):
                    return result_future.result().result
                if self._stop_event.is_set():
                    raise RunAborted(f"{label} 收到 stop 指令")
                if self._shutdown:
                    raise RunAborted(f"{label} 节点关闭")
                raise TimeoutError(
                    f"{label} 取消后 {self._cancel_grace_s:.0f}s 内未收到结果")
            raise TimeoutError(f"{label} 执行超时")
        finally:
            with self._goal_lock:
                self._goal_handle = None

    def _wait_future(self, future, timeout_s: float, abort_event=None) -> bool:
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        deadline = time.monotonic() + max(0.0, timeout_s)
        while time.monotonic() < deadline:
            # 先看 future：发送/取消这类短窗口操作可能已经完成，
            # 若先看 abort 就会把已接受的目标误判成"未发出去"而放任车继续走
            if done.wait(0.05):
                return True
            if self._shutdown or self._stop_event.is_set():
                return False
            if abort_event is not None and abort_event.is_set():
                return False
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


def _instance_lock_owner() -> str:
    try:
        with open(_INSTANCE_LOCK_PATH, "r", encoding="utf-8") as f:
            return f.read().strip() or "未知"
    except OSError:
        return "未知"


def _acquire_instance_lock():
    """单实例锁：同一台机器只允许一个 orchestrator 消费控制指令。

    2026-11-01 现场同时跑了两个 orchestrator，同一条 start 被两个节点
    重复消费：一个抢到导航目标，另一个立即报「前往被拒绝」退出，语音
    也开始语互相拒绝。这里用文件锁兜底，启动第二个直接退出。
    """
    fd = os.open(_INSTANCE_LOCK_PATH, os.O_CREAT | os.O_RDWR, 0o644)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        owner = _instance_lock_owner()
        os.close(fd)
        print(
            f"[ERROR] 已有 fire_inspection_orchestrator 实例在运行"
            f"（锁文件 {_INSTANCE_LOCK_PATH}，PID={owner}），本次启动退出。",
            file=sys.stderr,
        )
        return None
    os.ftruncate(fd, 0)
    os.write(fd, f"{os.getpid()}\n".encode())
    return fd


def _release_instance_lock(fd) -> None:
    if fd is None:
        return
    try:
        fcntl.flock(fd, fcntl.LOCK_UN)
    finally:
        os.close(fd)


def main(argv=None) -> int:
    rclpy.init(args=argv)
    lock_fd = _acquire_instance_lock()
    if lock_fd is None:
        rclpy.try_shutdown()
        return 0
    node = None
    executor = None
    try:
        node = OrchestratorNode()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        try:
            executor.spin()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        return 0
    finally:
        if node is not None:
            node.destroy_node()
        if executor is not None:
            executor.shutdown()
        rclpy.try_shutdown()
        _release_instance_lock(lock_fd)


if __name__ == "__main__":
    sys.exit(main())
