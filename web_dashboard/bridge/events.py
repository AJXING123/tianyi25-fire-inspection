"""Bridge 事件模型（纯 Python，无 ROS 依赖）。"""

from __future__ import annotations

import uuid

ROUND_START = "round_start"
ROUND_STATE = "round_state"
POI_RESULT = "poi_result"
ROUND_FINISH = "round_finish"

# 手动点名检查事件：与自动巡检事件完全独立，键名用 manual_id
MANUAL_START = "manual_start"
MANUAL_STATE = "manual_state"
MANUAL_RESULT = "manual_result"
MANUAL_FINISH = "manual_finish"

MANUAL_KINDS = (MANUAL_START, MANUAL_STATE, MANUAL_RESULT, MANUAL_FINISH)
EVENT_KINDS = (ROUND_START, ROUND_STATE, POI_RESULT, ROUND_FINISH) + MANUAL_KINDS


def new_event(kind: str, ref_id: str, payload: dict, created_at: str) -> dict:
    if kind not in EVENT_KINDS:
        raise ValueError(f"未知事件类型：{kind}")
    # 自动巡检事件沿用 round_id 键；手动检查事件用 manual_id 键
    key = "manual_id" if kind in MANUAL_KINDS else "round_id"
    return {
        "event_id": f"evt-{uuid.uuid4().hex}",
        "kind": kind,
        key: ref_id,
        "created_at": created_at,
        "payload": payload,
    }
