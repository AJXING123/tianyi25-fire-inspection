"""HTTP API：状态 / 历史 / 详情 / 控制。

安全边界：控制接口只把请求交给 ROS 适配层；第一阶段适配层未启用时
返回 503 并明确说明"未发送任何指令"，不写数据库、不伪造成功。
"""

from __future__ import annotations

import sqlite3
from typing import Annotated, Literal

from fastapi import APIRouter, Depends, Query, Request
from fastapi.responses import JSONResponse

from .db import ROUND_OUTCOMES, ROUND_SOURCES, ROUND_STATUSES, connect, utc_now
from .repository import (
    ack_event,
    get_active_round,
    get_event,
    get_manual,
    get_round,
    list_events,
    list_manuals,
    list_rounds,
)
from .ros_adapter import ControlResult, RosAdapter, RosUnavailable
from .schemas import (
    ControlRequest,
    ControlResponse,
    EventAckRequest,
    EventListResponse,
    EventOut,
    HealthResponse,
    ManualInspectionOut,
    ManualListResponse,
    RobotStatusOut,
    RoundDetail,
    RoundListResponse,
    RoundSummary,
    StatusResponse,
)

router = APIRouter(prefix="/api")


def get_db(request: Request):
    conn = connect(request.app.state.db_path)
    try:
        yield conn
    finally:
        conn.close()


def get_adapter(request: Request) -> RosAdapter:
    return request.app.state.ros_adapter


DbDep = Annotated[sqlite3.Connection, Depends(get_db)]
AdapterDep = Annotated[RosAdapter, Depends(get_adapter)]


def _robot_status(adapter: RosAdapter) -> RobotStatusOut:
    return RobotStatusOut.model_validate(adapter.get_robot_status().__dict__)


@router.get("/health", response_model=HealthResponse)
def health(conn: DbDep, adapter: AdapterDep) -> HealthResponse:
    try:
        # 读取真实表，才能发现文件损坏（SELECT 1 不触碰数据页，无法验证）
        conn.execute("SELECT 1 FROM inspection_round LIMIT 1").fetchone()
        db_state: Literal["ok", "error"] = "ok"
    except sqlite3.Error:
        db_state = "error"
    return HealthResponse(
        status="ok" if db_state == "ok" else "degraded",
        database=db_state,
        robot=_robot_status(adapter),
        server_time=utc_now(),
    )


@router.get("/status", response_model=StatusResponse)
def status(conn: DbDep, adapter: AdapterDep) -> StatusResponse:
    active = get_active_round(conn)
    return StatusResponse(
        server_time=utc_now(),
        robot=_robot_status(adapter),
        current_round=RoundDetail.model_validate(active) if active else None,
    )


@router.get("/rounds", response_model=RoundListResponse)
def rounds(
    conn: DbDep,
    status: Literal["running", "paused", "completed", "stopped", "aborted"] | None = None,
    outcome: Literal["normal", "abnormal", "uncertain"] | None = None,
    source: Literal["real", "demo", "simulated"] | None = None,
    limit: int = Query(default=20, ge=1, le=100),
    offset: int = Query(default=0, ge=0),
) -> RoundListResponse:
    # 断言枚举与 DB 校验一致，防止两边漂移
    assert status is None or status in ROUND_STATUSES
    assert outcome is None or outcome in ROUND_OUTCOMES
    assert source is None or source in ROUND_SOURCES
    items, total = list_rounds(
        conn,
        status=status,
        outcome=outcome,
        source=source,
        limit=limit,
        offset=offset,
    )
    return RoundListResponse(
        items=[RoundSummary.model_validate(item) for item in items],
        total=total,
        limit=limit,
        offset=offset,
    )


@router.get("/rounds/{round_id}", response_model=RoundDetail)
def round_detail(round_id: str, conn: DbDep) -> RoundDetail | JSONResponse:
    data = get_round(conn, round_id)
    if data is None:
        return JSONResponse(
            status_code=404,
            content={"detail": f"巡检记录「{round_id}」不存在"},
        )
    return RoundDetail.model_validate(data)


# ---------------------------------------------------------------------------
# 手动点名检查（只读查询；与自动巡检历史完全分开）
# ---------------------------------------------------------------------------


@router.get("/manual-inspections", response_model=ManualListResponse)
def manuals(
    conn: DbDep,
    status: Literal["running", "completed", "stopped", "aborted"] | None = None,
    limit: int = Query(default=20, ge=1, le=100),
    offset: int = Query(default=0, ge=0),
) -> ManualListResponse:
    items, total = list_manuals(conn, status=status, limit=limit, offset=offset)
    return ManualListResponse(
        items=[ManualInspectionOut.model_validate(item) for item in items],
        total=total,
        limit=limit,
        offset=offset,
    )


@router.get("/manual-inspections/{manual_id}", response_model=ManualInspectionOut)
def manual_detail(manual_id: str, conn: DbDep) -> ManualInspectionOut | JSONResponse:
    data = get_manual(conn, manual_id)
    if data is None:
        return JSONResponse(
            status_code=404,
            content={"detail": f"手动检查记录「{manual_id}」不存在"},
        )
    return ManualInspectionOut.model_validate(data)


@router.post("/control", response_model=ControlResponse)
def control(payload: ControlRequest, adapter: AdapterDep) -> ControlResponse | JSONResponse:
    cmd = payload.cmd
    arg = payload.arg.strip()
    operator = payload.operator_id.strip()

    if cmd == "manual" and not arg:
        return JSONResponse(
            status_code=400,
            content=ControlResponse(
                success=False, message="manual 指令必须指定点位（arg）", dispatched=False
            ).model_dump(),
        )
    if cmd == "skip" and arg and " " in arg:
        return JSONResponse(
            status_code=400,
            content=ControlResponse(
                success=False, message="arg 不能包含空格", dispatched=False
            ).model_dump(),
        )

    try:
        result: ControlResult = adapter.send_control(cmd, arg, operator)
    except RosUnavailable as exc:
        # 明确失败：不写库、不返回 success=true
        return JSONResponse(
            status_code=503,
            content=ControlResponse(
                success=False,
                message=str(exc),
                current_state=None,
                dispatched=False,
            ).model_dump(),
        )

    return ControlResponse(
        success=result.success,
        message=result.message,
        current_state=result.current_state,
        dispatched=True,
    )


# ---------------------------------------------------------------------------
# 统一异常事件（只读查询 + 标记处理）
# ---------------------------------------------------------------------------


@router.get("/events", response_model=EventListResponse)
def events(
    conn: DbDep,
    event_type: Literal[
        "alarm_detected", "vision_abnormal", "voice_broadcast",
        "comm_failure", "comm_restored",
    ] | None = None,
    severity: Literal["info", "warning", "critical"] | None = None,
    status: Literal["open", "acknowledged", "resolved"] | None = None,
    source: str | None = Query(default=None, max_length=32),
    round_id: str | None = Query(default=None, max_length=64),
    limit: int = Query(default=20, ge=1, le=100),
    offset: int = Query(default=0, ge=0),
) -> EventListResponse:
    items, total = list_events(
        conn,
        event_type=event_type,
        severity=severity,
        status=status,
        source=source,
        round_id=round_id,
        limit=limit,
        offset=offset,
    )
    return EventListResponse(
        items=[EventOut.model_validate(item) for item in items],
        total=total,
        limit=limit,
        offset=offset,
    )


@router.get("/events/{event_id}", response_model=EventOut)
def event_detail(event_id: str, conn: DbDep) -> EventOut | JSONResponse:
    data = get_event(conn, event_id)
    if data is None:
        return JSONResponse(
            status_code=404,
            content={"detail": f"事件「{event_id}」不存在"},
        )
    return EventOut.model_validate(data)


@router.post("/events/{event_id}/ack", response_model=EventOut)
def event_ack(
    event_id: str, payload: EventAckRequest, conn: DbDep
) -> EventOut | JSONResponse:
    """标记事件已处理（acknowledged）或已解决（resolved）。"""
    data = ack_event(
        conn,
        event_id,
        operator_id=payload.operator_id.strip() or None,
        status=payload.status,
    )
    if data is None:
        return JSONResponse(
            status_code=404,
            content={"detail": f"事件「{event_id}」不存在"},
        )
    return EventOut.model_validate(data)
