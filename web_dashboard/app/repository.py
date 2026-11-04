"""巡检数据的写入与查询（纯 sqlite3，无 ORM）。

本模块同时服务于：只读 API、演示数据脚本，以及第二阶段 ROS 适配层落库。
所有写入函数都做枚举校验，避免把未知状态写进数据库。
"""

from __future__ import annotations

import json
import sqlite3
import uuid
from typing import Any

from .db import (
    EVENT_SEVERITIES,
    EVENT_STATUSES,
    EVENT_TYPES,
    MANUAL_STATUSES,
    MANUAL_TERMINAL_STATUSES,
    ROUND_MODES,
    ROUND_OUTCOMES,
    ROUND_SOURCES,
    ROUND_STATUSES,
    SKIPPED_STATUS_NAME,
    STATUS_NAMES,
    utc_now,
)

ACTIVE_STATUSES = ("running", "paused")


def _check(value: str | None, allowed: tuple[str, ...], field: str) -> str | None:
    if value is None:
        return None
    if value not in allowed:
        raise ValueError(f"{field}「{value}」非法，允许值：{', '.join(allowed)}")
    return value


def insert_round(
    conn: sqlite3.Connection,
    *,
    round_id: str | None = None,
    mode: str = "inspection",
    source: str = "real",
    status: str = "running",
    outcome: str | None = None,
    operator_id: str | None = None,
    ros_state: str | None = None,
    summary: str | None = None,
    error_message: str | None = None,
    total_pois: int | None = None,
    started_at: str | None = None,
    finished_at: str | None = None,
) -> str:
    """写入一轮巡检，返回 round_id。is_demo 由 source 推导，不可单独伪造。"""
    _check(mode, ROUND_MODES, "mode")
    _check(source, ROUND_SOURCES, "source")
    _check(status, ROUND_STATUSES, "status")
    _check(outcome, ROUND_OUTCOMES, "outcome")
    if total_pois is not None and total_pois < 0:
        raise ValueError("total_pois 不能为负数")

    rid = round_id or uuid.uuid4().hex
    now = utc_now()
    started = started_at or now
    is_demo = 1 if source == "demo" else 0
    with conn:
        conn.execute(
            """
            INSERT INTO inspection_round
              (round_id, mode, source, is_demo, status, outcome, operator_id,
               ros_state, summary, error_message, total_pois, started_at,
               finished_at, created_at, updated_at)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """,
            (
                rid,
                mode,
                source,
                is_demo,
                status,
                outcome,
                operator_id,
                ros_state,
                summary,
                error_message,
                total_pois,
                started,
                finished_at,
                now,
                now,
            ),
        )
    return rid


def finish_round(
    conn: sqlite3.Connection,
    round_id: str,
    *,
    status: str,
    outcome: str | None = None,
    summary: str | None = None,
    error_message: str | None = None,
    finished_at: str | None = None,
) -> bool:
    """把一轮巡检置为终态（completed / stopped / aborted）。"""
    _check(status, ("completed", "stopped", "aborted"), "status")
    _check(outcome, ROUND_OUTCOMES, "outcome")
    now = utc_now()
    with conn:
        cur = conn.execute(
            """
            UPDATE inspection_round
               SET status = ?, outcome = ?, summary = ?, error_message = ?,
                   finished_at = ?, updated_at = ?
             WHERE round_id = ?
            """,
            (status, outcome, summary, error_message, finished_at or now, now, round_id),
        )
    return cur.rowcount > 0


def update_round_state(
    conn: sqlite3.Connection,
    round_id: str,
    *,
    status: str,
    ros_state: str | None = None,
) -> bool:
    """更新进行中轮次的实时状态（由第二阶段 ROS 适配层调用）。"""
    _check(status, ROUND_STATUSES, "status")
    now = utc_now()
    with conn:
        cur = conn.execute(
            """
            UPDATE inspection_round
               SET status = ?, ros_state = COALESCE(?, ros_state), updated_at = ?
             WHERE round_id = ?
            """,
            (status, ros_state, now, round_id),
        )
    return cur.rowcount > 0


def create_round_if_absent(
    conn: sqlite3.Connection,
    *,
    round_id: str,
    mode: str = "inspection",
    status: str = "running",
    operator_id: str | None = None,
    ros_state: str | None = None,
    total_pois: int | None = None,
    started_at: str | None = None,
) -> tuple[dict[str, Any], bool]:
    """Bridge 建轮：同一个 round_id 重复投递不产生第二条记录。

    返回 (轮次详情, 是否新建)。source 固定 real，演示数据只能走 seed_demo。
    """
    now = utc_now()
    started = started_at or now
    with conn:
        cur = conn.execute(
            """
            INSERT OR IGNORE INTO inspection_round
              (round_id, mode, source, is_demo, status, outcome, operator_id,
               ros_state, summary, error_message, total_pois, started_at,
               finished_at, created_at, updated_at)
            VALUES (?, ?, 'real', 0, ?, NULL, ?, ?, NULL, NULL, ?, ?, NULL, ?, ?)
            """,
            (
                round_id,
                mode,
                status,
                operator_id,
                ros_state,
                total_pois,
                started,
                now,
                now,
            ),
        )
    data = get_round(conn, round_id)
    if data is None:  # 理论不可达：INSERT OR IGNORE 后必然可查
        raise RuntimeError(f"建轮失败：{round_id}")
    return data, cur.rowcount == 1


def next_sequence(conn: sqlite3.Connection, round_id: str) -> int:
    row = conn.execute(
        "SELECT COALESCE(MAX(sequence) + 1, 0) FROM poi_result WHERE round_id = ?",
        (round_id,),
    ).fetchone()
    return int(row[0])


def insert_poi_result(
    conn: sqlite3.Connection,
    *,
    round_id: str,
    poi: str,
    sequence: int = 0,
    status_code: int | None = None,
    status_name: str | None = None,
    skipped: bool = False,
    simulated: bool = False,
    detail: str | None = None,
    detections_json: str | None = None,
    checked_at: str | None = None,
    duration_s: float | None = None,
    event_id: str | None = None,
) -> bool:
    """写入单个巡检点结果，返回是否真正插入（event_id 重复时返回 False）。

    status_code 只允许 0-6（与 PoiResult.msg 一致）；跳过必须显式 skipped=True
    且不写状态码，避免把"跳过"伪装成某种视觉结论。
    """
    if skipped:
        if status_code is not None:
            raise ValueError("跳过点位不得写 status_code，跳过不是视觉结论")
        name = SKIPPED_STATUS_NAME
    else:
        if status_code is None or status_code not in STATUS_NAMES:
            raise ValueError(f"status_code={status_code} 非法，必须是 0-6")
        name = status_name or STATUS_NAMES[status_code]

    with conn:
        cur = conn.execute(
            """
            INSERT OR IGNORE INTO poi_result
              (event_id, round_id, poi, sequence, status_code, status_name, skipped,
               simulated, detail, detections_json, checked_at, duration_s)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """,
            (
                event_id,
                round_id,
                poi,
                sequence,
                status_code,
                name,
                1 if skipped else 0,
                1 if simulated else 0,
                detail,
                detections_json,
                checked_at or utc_now(),
                duration_s,
            ),
        )
    return cur.rowcount > 0


def _round_row(row: sqlite3.Row) -> dict[str, Any]:
    return {
        "round_id": row["round_id"],
        "mode": row["mode"],
        "source": row["source"],
        "is_demo": bool(row["is_demo"]),
        "status": row["status"],
        "outcome": row["outcome"],
        "operator_id": row["operator_id"],
        "ros_state": row["ros_state"],
        "summary": row["summary"],
        "error_message": row["error_message"],
        "total_pois": row["total_pois"],
        "started_at": row["started_at"],
        "finished_at": row["finished_at"],
        "recorded_count": int(row["recorded_count"] or 0),
        "processed_count": int(row["processed_count"] or 0),
        "abnormal_count": int(row["abnormal_count"] or 0),
    }


_ROUND_SELECT = """
SELECT r.*,
       (SELECT COUNT(*) FROM poi_result p
         WHERE p.round_id = r.round_id AND p.skipped = 0) AS recorded_count,
       (SELECT COUNT(*) FROM poi_result p
         WHERE p.round_id = r.round_id) AS processed_count,
       (SELECT COUNT(*) FROM poi_result p
         WHERE p.round_id = r.round_id AND p.status_code = 1) AS abnormal_count
  FROM inspection_round r
"""


def _build_filters(
    status: str | None, outcome: str | None, source: str | None
) -> tuple[str, list[Any]]:
    where: list[str] = []
    params: list[Any] = []
    if status:
        if status in ("running", "paused"):
            where.append("r.status IN ('running', 'paused')")
        elif status in ROUND_STATUSES:
            where.append("r.status = ?")
            params.append(status)
        else:
            raise ValueError(f"status「{status}」非法")
    if outcome:
        _check(outcome, ROUND_OUTCOMES, "outcome")
        where.append("r.outcome = ?")
        params.append(outcome)
    if source:
        _check(source, ROUND_SOURCES, "source")
        where.append("r.source = ?")
        params.append(source)
    clause = f" WHERE {' AND '.join(where)}" if where else ""
    return clause, params


def list_rounds(
    conn: sqlite3.Connection,
    *,
    status: str | None = None,
    outcome: str | None = None,
    source: str | None = None,
    limit: int = 20,
    offset: int = 0,
) -> tuple[list[dict[str, Any]], int]:
    clause, params = _build_filters(status, outcome, source)
    total = conn.execute(
        f"SELECT COUNT(*) FROM inspection_round r{clause}", params
    ).fetchone()[0]
    rows = conn.execute(
        f"{_ROUND_SELECT}{clause} ORDER BY r.started_at DESC, r.id DESC "
        "LIMIT ? OFFSET ?",
        [*params, limit, offset],
    ).fetchall()
    return [_round_row(row) for row in rows], int(total)


def get_round(conn: sqlite3.Connection, round_id: str) -> dict[str, Any] | None:
    row = conn.execute(f"{_ROUND_SELECT} WHERE r.round_id = ?", (round_id,)).fetchone()
    if row is None:
        return None
    data = _round_row(row)
    data["results"] = get_poi_results(conn, round_id)
    return data


def get_active_round(conn: sqlite3.Connection) -> dict[str, Any] | None:
    """当前进行中（running / paused）的最近一轮；没有返回 None。"""
    row = conn.execute(
        f"{_ROUND_SELECT} WHERE r.status IN ('running', 'paused') "
        "ORDER BY r.started_at DESC, r.id DESC LIMIT 1"
    ).fetchone()
    if row is None:
        return None
    data = _round_row(row)
    data["results"] = get_poi_results(conn, data["round_id"])
    return data


def get_poi_results(conn: sqlite3.Connection, round_id: str) -> list[dict[str, Any]]:
    rows = conn.execute(
        "SELECT * FROM poi_result WHERE round_id = ? ORDER BY sequence, id",
        (round_id,),
    ).fetchall()
    results = []
    for row in rows:
        raw = row["detections_json"]
        try:
            detections = json.loads(raw) if raw else None
            if not isinstance(detections, list):
                detections = None
        except (TypeError, ValueError):
            detections = None
        results.append(
            {
                "poi": row["poi"],
                "sequence": int(row["sequence"]),
                "status_code": row["status_code"],
                "status_name": row["status_name"],
                "skipped": bool(row["skipped"]),
                "simulated": bool(row["simulated"]),
                "detail": row["detail"],
                "detections": detections,
                "checked_at": row["checked_at"],
                "duration_s": row["duration_s"],
            }
        )
    return results


def classify_outcome(conn: sqlite3.Connection, round_id: str) -> str | None:
    """按点位结果推导本轮结论：任一 ABNORMAL -> abnormal。

    没有任何可判定结果时返回 None；存在 UNCERTAIN/TIMEOUT 等未见异常时
    返回 uncertain，绝不让"没看清"落成 normal。
    """
    rows = conn.execute(
        "SELECT status_code FROM poi_result WHERE round_id = ? AND skipped = 0",
        (round_id,),
    ).fetchall()
    codes = [row["status_code"] for row in rows if row["status_code"] is not None]
    if not codes:
        return None
    if 1 in codes:
        return "abnormal"
    if all(code == 0 for code in codes):
        return "normal"
    return "uncertain"


# ---------------------------------------------------------------------------
# 统一异常事件（防空警报 / 视觉异常 / 语音播报 / 通信异常）
#
# 字段为网页侧建议契约，**待机器人端确认**：
#   event_id / event_type / timestamp / source / severity / message / details
# 额外自用字段：round_id、poi、simulated、处理状态（open/acknowledged/resolved）。
# ---------------------------------------------------------------------------


def insert_event(
    conn: sqlite3.Connection,
    *,
    event_id: str,
    event_type: str,
    source: str,
    message: str,
    severity: str = "warning",
    timestamp: str | None = None,
    details: dict | None = None,
    round_id: str | None = None,
    poi: str | None = None,
    simulated: bool = False,
) -> bool:
    """写入一条事件，返回是否真正插入（同一 event_id 重复上报返回 False）。"""
    _check(event_type, EVENT_TYPES, "event_type")
    _check(severity, EVENT_SEVERITIES, "severity")
    if not source or not source.strip():
        raise ValueError("source 不能为空")
    if not message or not message.strip():
        raise ValueError("message 不能为空")
    now = utc_now()
    with conn:
        cur = conn.execute(
            """
            INSERT OR IGNORE INTO event_record
              (event_id, event_type, occurred_at, source, severity, message,
               details_json, round_id, poi, simulated, status, handled_by,
               handled_at, created_at)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, 'open', NULL, NULL, ?)
            """,
            (
                event_id,
                event_type,
                timestamp or now,
                source.strip(),
                severity,
                message.strip(),
                json.dumps(details, ensure_ascii=False) if details is not None else None,
                round_id,
                poi,
                1 if simulated else 0,
                now,
            ),
        )
    return cur.rowcount > 0


def _event_row(row: sqlite3.Row) -> dict[str, Any]:
    raw = row["details_json"]
    try:
        details = json.loads(raw) if raw else None
        if not isinstance(details, dict):
            details = None
    except (TypeError, ValueError):
        details = None
    return {
        "event_id": row["event_id"],
        # API 对外字段名与建议契约一致：timestamp
        "timestamp": row["occurred_at"],
        "event_type": row["event_type"],
        "source": row["source"],
        "severity": row["severity"],
        "message": row["message"],
        "details": details,
        "round_id": row["round_id"],
        "poi": row["poi"],
        "simulated": bool(row["simulated"]),
        "status": row["status"],
        "handled_by": row["handled_by"],
        "handled_at": row["handled_at"],
    }


def list_events(
    conn: sqlite3.Connection,
    *,
    event_type: str | None = None,
    severity: str | None = None,
    status: str | None = None,
    source: str | None = None,
    round_id: str | None = None,
    limit: int = 20,
    offset: int = 0,
) -> tuple[list[dict[str, Any]], int]:
    _check(event_type, EVENT_TYPES, "event_type")
    _check(severity, EVENT_SEVERITIES, "severity")
    _check(status, EVENT_STATUSES, "status")

    where: list[str] = []
    params: list[Any] = []
    if event_type:
        where.append("event_type = ?")
        params.append(event_type)
    if severity:
        where.append("severity = ?")
        params.append(severity)
    if status:
        where.append("status = ?")
        params.append(status)
    if source:
        where.append("source = ?")
        params.append(source)
    if round_id:
        where.append("round_id = ?")
        params.append(round_id)
    clause = f" WHERE {' AND '.join(where)}" if where else ""

    total = conn.execute(
        f"SELECT COUNT(*) FROM event_record{clause}", params
    ).fetchone()[0]
    rows = conn.execute(
        f"SELECT * FROM event_record{clause} "
        "ORDER BY occurred_at DESC, id DESC LIMIT ? OFFSET ?",
        [*params, limit, offset],
    ).fetchall()
    return [_event_row(row) for row in rows], int(total)


def get_event(conn: sqlite3.Connection, event_id: str) -> dict[str, Any] | None:
    row = conn.execute(
        "SELECT * FROM event_record WHERE event_id = ?", (event_id,)
    ).fetchone()
    return _event_row(row) if row is not None else None


def ack_event(
    conn: sqlite3.Connection,
    event_id: str,
    *,
    operator_id: str | None = None,
    status: str = "acknowledged",
) -> dict[str, Any] | None:
    """标记事件处理状态；事件不存在返回 None。重复标记是幂等的。"""
    _check(status, ("acknowledged", "resolved"), "status")
    now = utc_now()
    with conn:
        cur = conn.execute(
            """
            UPDATE event_record
               SET status = ?, handled_by = ?, handled_at = ?
             WHERE event_id = ?
            """,
            (status, operator_id or None, now, event_id),
        )
    if cur.rowcount == 0:
        return None
    return get_event(conn, event_id)


# ---------------------------------------------------------------------------
# 手动点名检查（与自动巡检完全独立：不进 inspection_round / poi_result）
#
# status 是执行状态（running/completed/stopped/aborted）；
# status_code / status_name 是正式视觉检查状态（PoiResult 0-6），
# 没有正式结果时保持 NULL——"没结果"绝不能显示成正常。
# ---------------------------------------------------------------------------


def create_manual_if_absent(
    conn: sqlite3.Connection,
    *,
    manual_id: str,
    poi: str,
    operator_id: str | None = None,
    ros_state: str | None = None,
    started_at: str | None = None,
) -> tuple[dict[str, Any], bool]:
    """Bridge 建手动检查记录：同一 manual_id 重复投递不产生第二条记录。"""
    if not poi or not poi.strip():
        raise ValueError("poi 不能为空")
    now = utc_now()
    with conn:
        cur = conn.execute(
            """
            INSERT OR IGNORE INTO manual_inspection
              (manual_id, poi, operator_id, status, ros_state, status_code,
               status_name, simulated, detail, detections_json, result_event_id,
               started_at, checked_at, finished_at, error_message,
               created_at, updated_at)
            VALUES (?, ?, ?, 'running', ?, NULL, NULL, 0, NULL, NULL, NULL,
                    ?, NULL, NULL, NULL, ?, ?)
            """,
            (manual_id, poi.strip(), operator_id, ros_state,
             started_at or now, now, now),
        )
    data = get_manual(conn, manual_id)
    if data is None:  # 理论不可达：INSERT OR IGNORE 后必然可查
        raise RuntimeError(f"建手动检查记录失败：{manual_id}")
    return data, cur.rowcount == 1


def update_manual_state(
    conn: sqlite3.Connection,
    manual_id: str,
    *,
    ros_state: str | None = None,
) -> bool:
    """更新进行中手动检查的实时状态文本；终态记录拒绝更新（返回 False）。"""
    now = utc_now()
    with conn:
        cur = conn.execute(
            """
            UPDATE manual_inspection
               SET ros_state = COALESCE(?, ros_state), updated_at = ?
             WHERE manual_id = ? AND status = 'running'
            """,
            (ros_state, now, manual_id),
        )
    return cur.rowcount > 0


def record_manual_result(
    conn: sqlite3.Connection,
    manual_id: str,
    *,
    status_code: int,
    simulated: bool = False,
    detail: str | None = None,
    detections_json: str | None = None,
    checked_at: str | None = None,
    event_id: str,
) -> str:
    """写入正式视觉结果，返回 'stored' / 'duplicate' / 'conflict'。

    - 只允许 0-6（与 PoiResult.msg 一致），status_name 由枚举推导；
    - 一条手动检查只对应一次点检：已有结果且 event_id 不同 -> 'conflict'；
    - 已结束（终态）但尚无结果的记录允许补录，容忍结果/状态话题乱序。
    """
    if status_code not in STATUS_NAMES:
        raise ValueError(f"status_code={status_code} 非法，必须是 0-6")
    now = utc_now()
    with conn:
        cur = conn.execute(
            """
            UPDATE manual_inspection
               SET status_code = ?, status_name = ?, simulated = ?, detail = ?,
                   detections_json = ?, checked_at = ?, result_event_id = ?,
                   updated_at = ?
             WHERE manual_id = ? AND result_event_id IS NULL
            """,
            (
                status_code,
                STATUS_NAMES[status_code],
                1 if simulated else 0,
                detail,
                detections_json,
                checked_at or now,
                event_id,
                now,
                manual_id,
            ),
        )
    if cur.rowcount == 1:
        return "stored"
    row = conn.execute(
        "SELECT result_event_id FROM manual_inspection WHERE manual_id = ?",
        (manual_id,),
    ).fetchone()
    if row is not None and row["result_event_id"] == event_id:
        return "duplicate"
    return "conflict"


def finish_manual(
    conn: sqlite3.Connection,
    manual_id: str,
    *,
    status: str,
    error_message: str | None = None,
    finished_at: str | None = None,
) -> bool:
    """把手动检查置为终态（completed/stopped/aborted）；已终态返回 False。"""
    _check(status, MANUAL_TERMINAL_STATUSES, "status")
    now = utc_now()
    with conn:
        cur = conn.execute(
            """
            UPDATE manual_inspection
               SET status = ?, error_message = COALESCE(?, error_message),
                   finished_at = ?, updated_at = ?
             WHERE manual_id = ? AND status = 'running'
            """,
            (status, error_message, finished_at or now, now, manual_id),
        )
    return cur.rowcount > 0


def _manual_row(row: sqlite3.Row) -> dict[str, Any]:
    raw = row["detections_json"]
    try:
        detections = json.loads(raw) if raw else None
        if not isinstance(detections, list):
            detections = None
    except (TypeError, ValueError):
        detections = None
    return {
        "manual_id": row["manual_id"],
        "poi": row["poi"],
        "operator_id": row["operator_id"],
        "status": row["status"],
        "ros_state": row["ros_state"],
        "status_code": row["status_code"],
        "status_name": row["status_name"],
        "simulated": bool(row["simulated"]),
        "detail": row["detail"],
        "detections": detections,
        "started_at": row["started_at"],
        "checked_at": row["checked_at"],
        "finished_at": row["finished_at"],
        "error_message": row["error_message"],
    }


def get_manual(conn: sqlite3.Connection, manual_id: str) -> dict[str, Any] | None:
    row = conn.execute(
        "SELECT * FROM manual_inspection WHERE manual_id = ?", (manual_id,)
    ).fetchone()
    return _manual_row(row) if row is not None else None


def list_manuals(
    conn: sqlite3.Connection,
    *,
    status: str | None = None,
    limit: int = 20,
    offset: int = 0,
) -> tuple[list[dict[str, Any]], int]:
    _check(status, MANUAL_STATUSES, "status")
    clause = " WHERE status = ?" if status else ""
    params: list[Any] = [status] if status else []
    total = conn.execute(
        f"SELECT COUNT(*) FROM manual_inspection{clause}", params
    ).fetchone()[0]
    rows = conn.execute(
        "SELECT * FROM manual_inspection"
        f"{clause} ORDER BY started_at DESC, id DESC LIMIT ? OFFSET ?",
        [*params, limit, offset],
    ).fetchall()
    return [_manual_row(row) for row in rows], int(total)
