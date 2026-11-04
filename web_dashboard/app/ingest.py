"""ROS Bridge 专用数据接入 API（仅回环 + 共享令牌）。

Bridge 是唯一写入真实巡检数据的来源；网页 API 本身保持只读（控制除外）。
所有接入请求必须携带 ``X-Bridge-Token``；未配置令牌时接口整体禁用（503）。

约定：
- 轮次由 Bridge 生成 round_id，后端按 round_id 幂等建轮；
- 点位结果按 (round_id, event_id) 幂等去重，原始 status_code / simulated /
  detail / detections_json 原样保存，不做任何推断；
- 终态轮次拒绝再写入结果（409），避免事后污染已结束记录；
- 结束轮次时按已落库结果推导 outcome，未看清（UNCERTAIN 等）不会落成正常。
"""

from __future__ import annotations

import json
import secrets
import sqlite3
from typing import Annotated

from fastapi import APIRouter, Depends, Header, HTTPException, Request
from fastapi.responses import JSONResponse

from .db import ROUND_STATUSES, connect, utc_now
from .repository import (
    classify_outcome,
    create_manual_if_absent,
    create_round_if_absent,
    finish_manual,
    finish_round,
    get_manual,
    get_round,
    insert_event,
    insert_poi_result,
    next_sequence,
    record_manual_result,
    update_manual_state,
    update_round_state,
)
from .schemas import (
    IngestEvent,
    IngestFinish,
    IngestManualFinish,
    IngestManualResult,
    IngestManualStart,
    IngestManualState,
    IngestResult,
    IngestRoundStart,
    IngestState,
)

router = APIRouter(prefix="/api/ingest")

TERMINAL_STATUSES = ("completed", "stopped", "aborted")


def require_token(
    request: Request,
    x_bridge_token: Annotated[str, Header(alias="X-Bridge-Token")] = "",
) -> None:
    expected = request.app.state.settings.bridge_token
    if not expected:
        raise HTTPException(
            status_code=503,
            detail="数据接入未启用：后端未配置 FIRE_DASHBOARD_BRIDGE_TOKEN",
        )
    if not secrets.compare_digest(x_bridge_token, expected):
        raise HTTPException(status_code=401, detail="Bridge 令牌无效")


TokenDep = Annotated[None, Depends(require_token)]


def get_db(request: Request):
    conn: sqlite3.Connection = connect(request.app.state.db_path)
    try:
        yield conn
    finally:
        conn.close()


DbDep = Annotated[sqlite3.Connection, Depends(get_db)]


def _terminal(round_data: dict) -> bool:
    return round_data["status"] in TERMINAL_STATUSES


def _validate_detections(simulated: bool, detections_json: str | None) -> None:
    """detections_json 必须是 JSON 数组；simulated=true 时必须为空数组（契约 §6）。"""
    if not detections_json:
        return
    try:
        detections = json.loads(detections_json)
    except ValueError as exc:
        raise HTTPException(
            status_code=422, detail="detections_json 不是合法 JSON"
        ) from exc
    if not isinstance(detections, list):
        raise HTTPException(status_code=422, detail="detections_json 必须是 JSON 数组")
    if simulated and detections:
        raise HTTPException(
            status_code=422,
            detail="simulated=true 时 detections_json 必须为空数组（契约 §6）",
        )


@router.post("/rounds")
def start_round(payload: IngestRoundStart, conn: DbDep, _: TokenDep):
    data, created = create_round_if_absent(
        conn,
        round_id=payload.round_id,
        mode=payload.mode,
        status="running",
        operator_id=payload.operator_id,
        ros_state=payload.ros_state,
        total_pois=payload.total_pois,
        started_at=payload.started_at,
    )
    body = {
        "ok": True,
        "round_id": payload.round_id,
        "created": created,
        "status": data["status"],
    }
    return JSONResponse(status_code=201 if created else 200, content=body)


@router.post("/rounds/{round_id}/state")
def update_state(round_id: str, payload: IngestState, conn: DbDep, _: TokenDep):
    data = get_round(conn, round_id)
    if data is None:
        raise HTTPException(status_code=404, detail=f"巡检记录「{round_id}」不存在")
    if _terminal(data):
        raise HTTPException(status_code=409, detail="轮次已结束，拒绝更新状态")
    assert payload.status in ROUND_STATUSES
    update_round_state(conn, round_id, status=payload.status, ros_state=payload.ros_state)
    return {"ok": True, "round_id": round_id, "status": payload.status}


@router.post("/rounds/{round_id}/results")
def add_result(round_id: str, payload: IngestResult, conn: DbDep, _: TokenDep):
    data = get_round(conn, round_id)
    if data is None:
        raise HTTPException(status_code=404, detail=f"巡检记录「{round_id}」不存在")
    if _terminal(data):
        raise HTTPException(status_code=409, detail="轮次已结束，拒绝写入点位结果")

    _validate_detections(payload.simulated, payload.detections_json)

    stored = insert_poi_result(
        conn,
        round_id=round_id,
        poi=payload.poi,
        sequence=next_sequence(conn, round_id),
        status_code=payload.status_code,
        simulated=payload.simulated,
        detail=payload.detail,
        detections_json=payload.detections_json or None,
        checked_at=payload.checked_at,
        duration_s=payload.duration_s,
        event_id=payload.event_id,
    )
    return {
        "ok": True,
        "round_id": round_id,
        "event_id": payload.event_id,
        "stored": stored,
        "duplicate": not stored,
    }


@router.post("/rounds/{round_id}/finish")
def finish(round_id: str, payload: IngestFinish, conn: DbDep, _: TokenDep):
    data = get_round(conn, round_id)
    if data is None:
        raise HTTPException(status_code=404, detail=f"巡检记录「{round_id}」不存在")
    if _terminal(data):
        return {
            "ok": True,
            "round_id": round_id,
            "status": data["status"],
            "outcome": data["outcome"],
            "idempotent": True,
        }

    outcome = classify_outcome(conn, round_id)
    error_message = payload.error_message
    if payload.status == "aborted" and not error_message:
        error_message = "ROS 编排层报告 aborted（未提供结构化原因）"
    finish_round(
        conn,
        round_id,
        status=payload.status,
        outcome=outcome,
        summary=payload.summary,
        error_message=error_message,
        finished_at=payload.finished_at or utc_now(),
    )
    return {
        "ok": True,
        "round_id": round_id,
        "status": payload.status,
        "outcome": outcome,
        "idempotent": False,
    }


@router.post("/events")
def add_event(payload: IngestEvent, conn: DbDep, _: TokenDep):
    """上报异常事件（防空警报 / 视觉异常 / 语音播报 / 通信异常）。

    - 按 event_id 全局幂等：重复上报返回 duplicate=true，不新增历史记录；
    - round_id 可为空（不在巡检期间的警报），不做外键约束；
    - simulated 原样保存，模拟事件不会被当成真实事件。
    """
    stored = insert_event(
        conn,
        event_id=payload.event_id,
        event_type=payload.event_type,
        timestamp=payload.timestamp,
        source=payload.source,
        severity=payload.severity,
        message=payload.message,
        details=payload.details,
        round_id=payload.round_id,
        poi=payload.poi,
        simulated=payload.simulated,
    )
    return {
        "ok": True,
        "event_id": payload.event_id,
        "stored": stored,
        "duplicate": not stored,
    }


# ---------------------------------------------------------------------------
# 手动点名检查接入（与自动巡检轮次完全独立）
# ---------------------------------------------------------------------------


@router.post("/manual-inspections")
def start_manual(payload: IngestManualStart, conn: DbDep, _: TokenDep):
    data, created = create_manual_if_absent(
        conn,
        manual_id=payload.manual_id,
        poi=payload.poi,
        operator_id=payload.operator_id,
        ros_state=payload.ros_state,
        started_at=payload.started_at,
    )
    body = {
        "ok": True,
        "manual_id": payload.manual_id,
        "created": created,
        "status": data["status"],
    }
    return JSONResponse(status_code=201 if created else 200, content=body)


@router.post("/manual-inspections/{manual_id}/state")
def update_manual(manual_id: str, payload: IngestManualState, conn: DbDep, _: TokenDep):
    data = get_manual(conn, manual_id)
    if data is None:
        raise HTTPException(status_code=404, detail=f"手动检查记录「{manual_id}」不存在")
    if _terminal(data):
        raise HTTPException(status_code=409, detail="手动检查已结束，拒绝更新状态")
    update_manual_state(conn, manual_id, ros_state=payload.ros_state)
    return {"ok": True, "manual_id": manual_id}


@router.post("/manual-inspections/{manual_id}/result")
def add_manual_result(
    manual_id: str, payload: IngestManualResult, conn: DbDep, _: TokenDep
):
    data = get_manual(conn, manual_id)
    if data is None:
        raise HTTPException(status_code=404, detail=f"手动检查记录「{manual_id}」不存在")

    _validate_detections(payload.simulated, payload.detections_json)

    outcome = record_manual_result(
        conn,
        manual_id,
        status_code=payload.status_code,
        simulated=payload.simulated,
        detail=payload.detail,
        detections_json=payload.detections_json or None,
        checked_at=payload.checked_at,
        event_id=payload.event_id,
    )
    if outcome == "conflict":
        # 一条手动检查只对应一次点检；不同 event_id 的第二份结果属异常
        raise HTTPException(status_code=409, detail="该手动检查已有不同的正式结果")
    return {
        "ok": True,
        "manual_id": manual_id,
        "event_id": payload.event_id,
        "stored": outcome == "stored",
        "duplicate": outcome == "duplicate",
    }


@router.post("/manual-inspections/{manual_id}/finish")
def finish_manual_record(
    manual_id: str, payload: IngestManualFinish, conn: DbDep, _: TokenDep
):
    data = get_manual(conn, manual_id)
    if data is None:
        raise HTTPException(status_code=404, detail=f"手动检查记录「{manual_id}」不存在")
    if _terminal(data):
        return {
            "ok": True,
            "manual_id": manual_id,
            "status": data["status"],
            "idempotent": True,
        }

    error_message = payload.error_message
    if payload.status == "aborted" and not error_message:
        error_message = "ROS 编排层报告 aborted（手动检查，未提供结构化原因）"
    finish_manual(
        conn,
        manual_id,
        status=payload.status,
        error_message=error_message,
        finished_at=payload.finished_at or utc_now(),
    )
    return {
        "ok": True,
        "manual_id": manual_id,
        "status": payload.status,
        "idempotent": False,
    }
