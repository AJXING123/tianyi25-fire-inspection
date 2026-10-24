"""巡检总控的数据结构与状态枚举。

只用标准库，模拟模式下不依赖 ROS / 底盘 / 相机。
"""

from __future__ import annotations

import enum
from dataclasses import dataclass, field
from datetime import datetime
from typing import Optional


def now_iso() -> str:
    return datetime.now().isoformat(timespec="milliseconds")


# ── 导航结果 ────────────────────────────────────────────────────────

class NavOutcome(enum.Enum):
    """单段导航结果。

    CANCELLED 单独成类：现底盘 REST 把 cancelled/stopped 也放进终态失败集合，
    但「用户取消」与「真失败」对上层语义不同。当前只能从 WorkflowError 消息
    文本区分，属脆弱约定，REST 是否有正式取消语义未确认。
    """

    SUCCESS = "success"
    FAILED = "failed"            # 底盘返回失败终态 / 下发异常
    TIMEOUT = "timeout"          # 轮询超时
    CANCELLED = "cancelled"      # 用户中断（Ctrl+C 经 _EmergencyStop 转换）
    PRECHECK_REFUSED = "precheck_refused"  # 前置校验拒绝，未下发任何运动
    SKIPPED = "skipped"          # 模拟模式或配置跳过


@dataclass
class NavResult:
    outcome: NavOutcome
    label: str = ""
    detail: str = ""
    elapsed_s: float = 0.0
    simulated: bool = False

    @property
    def ok(self) -> bool:
        return self.outcome is NavOutcome.SUCCESS

    def to_dict(self) -> dict:
        return {
            "outcome": self.outcome.value,
            "label": self.label,
            "detail": self.detail,
            "elapsed_s": round(self.elapsed_s, 3),
            "simulated": self.simulated,
        }


# ── 语音结果 ────────────────────────────────────────────────────────

class SpeechState(enum.Enum):
    """一次播报请求的可观测状态。

    注意：audio_node 的 query 返回的是全局播放状态（playing/idle），
    不能严格关联到「本条请求」。DONE_OBSERVED 仅表示提交后观察到
    playing→非playing 的跳变；SUBMIT_UNCONFIRMED 表示提交被接受但
    从未观测到 playing（可能瞬播完、可能被抢占、也可能实际未播）。
    """

    DONE_OBSERVED = "done_observed"          # 观测到 playing→结束 跳变
    SUBMIT_UNCONFIRMED = "submit_unconfirmed"  # 提交成功但未观测到 playing
    SUBMIT_FAILED = "submit_failed"          # success=false 或调用异常
    QUERY_TIMEOUT = "query_timeout"          # 轮询期间一直是 playing，超时
    QUERY_ERROR = "query_error"              # query 调用本身失败
    SKIPPED = "skipped"                      # 未请求播报（如模拟关闭）


@dataclass
class SpeechResult:
    state: SpeechState
    text: str = ""
    submit_success: bool = False
    submit_status: str = ""
    observed_transition: bool = False  # 是否确实看到 playing→非playing
    elapsed_s: float = 0.0
    simulated: bool = False

    def to_dict(self) -> dict:
        return {
            "state": self.state.value,
            "text": self.text,
            "submit_success": self.submit_success,
            "submit_status": self.submit_status,
            "observed_transition": self.observed_transition,
            "elapsed_s": round(self.elapsed_s, 3),
            "simulated": self.simulated,
        }


# ── 视觉结果 ────────────────────────────────────────────────────────

class InspectionStatus(enum.Enum):
    """单点视觉检查结论。

    任何非正常路径都不得落成 SUCCESS：
    - UNCERTAIN   置信度不足 / 帧数不足，需人工复核
    - TIMEOUT     取图或推理超时
    - NO_FRAME    取不到图像
    - NO_MODEL    模型未配置（真实模式下的明确拒绝）
    """

    SUCCESS = "success"          # 有检测且判定为正常（含"无异常目标"策略结论）
    ABNORMAL = "abnormal"        # 检出异常目标（如吸烟）
    UNCERTAIN = "uncertain"
    TIMEOUT = "timeout"
    NO_FRAME = "no_frame"
    NO_MODEL = "no_model"
    ERROR = "error"
    SKIPPED = "skipped"


@dataclass
class Detection:
    class_name: str
    confidence: float
    xyxy: Optional[list] = None
    simulated: bool = False

    def to_dict(self) -> dict:
        return {
            "class_name": self.class_name,
            "confidence": round(float(self.confidence), 4),
            "xyxy": self.xyxy,
            "simulated": self.simulated,
        }


@dataclass
class PoiInspection:
    status: InspectionStatus
    poi: str = ""
    detections: list = field(default_factory=list)  # list[Detection]
    detail: str = ""
    inference_ms: float = 0.0
    simulated: bool = False

    def to_dict(self) -> dict:
        return {
            "status": self.status.value,
            "poi": self.poi,
            "detections": [d.to_dict() for d in self.detections],
            "detail": self.detail,
            "inference_ms": round(self.inference_ms, 1),
            "simulated": self.simulated,
        }


# ── 巡检点完整记录 ──────────────────────────────────────────────────

@dataclass
class PoiRecord:
    poi: str
    ts_start: str = field(default_factory=now_iso)
    ts_end: str = ""
    nav: Optional[dict] = None
    align: Optional[dict] = None
    inspection: Optional[dict] = None
    speech: dict = field(default_factory=dict)   # 阶段名 -> SpeechResult.to_dict()
    pose: Optional[dict] = None
    final: str = "pending"   # pending / done / nav_failed / cancelled / aborted

    def to_dict(self) -> dict:
        return {
            "record_type": "poi",
            "poi": self.poi,
            "ts_start": self.ts_start,
            "ts_end": self.ts_end,
            "nav": self.nav,
            "align": self.align,
            "inspection": self.inspection,
            "speech": self.speech,
            "pose": self.pose,
            "final": self.final,
        }
