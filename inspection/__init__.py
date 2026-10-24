"""消防巡检总控包。

模块边界：
    types      数据结构与状态枚举（无外部依赖）
    recorder   JSONL 巡检记录
    speaker    TTS 语音适配层（真实 / 模拟）
    navigator  导航适配层（复用 run_POI 的 _run_motion_safe / 模拟）
    inspector  视觉适配层（YoloLoad / 模拟）
    controller 巡检状态机与点位调度

入口脚本：src/chassis/run_inspection.py（默认模拟模式，实机需显式 --real）。
"""

from .types import (
    InspectionStatus,
    NavOutcome,
    NavResult,
    PoiInspection,
    PoiRecord,
    SpeechState,
    SpeechResult,
)

__all__ = [
    "InspectionStatus",
    "NavOutcome",
    "NavResult",
    "PoiInspection",
    "PoiRecord",
    "SpeechState",
    "SpeechResult",
]
