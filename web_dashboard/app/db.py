"""SQLite 连接与建表。

数据模型与 ROS 巡检系统对齐（见 接口契约冻结表.md）：
- inspection_round 一轮巡检（本轮状态 / 结果分类 / 数据来源 / 计划点位总数）
- poi_result        单个巡检点的正式视觉结果（status 0-6 / simulated / detail /
                    detections_json），跳过时记为 skipped，不用视觉状态码冒充。
- event_record      统一异常事件（防空警报 / 视觉异常 / 语音播报 / 通信异常），
                    按 event_id 全局幂等。字段为网页侧建议契约，待机器人端确认。
"""

from __future__ import annotations

import sqlite3
from datetime import datetime, timezone
from pathlib import Path

SCHEMA = """
CREATE TABLE IF NOT EXISTS inspection_round (
    id            INTEGER PRIMARY KEY AUTOINCREMENT,
    round_id      TEXT    NOT NULL UNIQUE,
    mode          TEXT    NOT NULL DEFAULT 'inspection',
    source        TEXT    NOT NULL DEFAULT 'real',
    is_demo       INTEGER NOT NULL DEFAULT 0,
    status        TEXT    NOT NULL DEFAULT 'running',
    outcome       TEXT,
    operator_id   TEXT,
    ros_state     TEXT,
    summary       TEXT,
    error_message TEXT,
    total_pois    INTEGER,
    started_at    TEXT    NOT NULL,
    finished_at   TEXT,
    created_at    TEXT    NOT NULL,
    updated_at    TEXT    NOT NULL
);

CREATE INDEX IF NOT EXISTS idx_round_status  ON inspection_round(status);
CREATE INDEX IF NOT EXISTS idx_round_started ON inspection_round(started_at DESC);

CREATE TABLE IF NOT EXISTS poi_result (
    id              INTEGER PRIMARY KEY AUTOINCREMENT,
    event_id        TEXT,
    round_id        TEXT    NOT NULL REFERENCES inspection_round(round_id)
                            ON DELETE CASCADE,
    poi             TEXT    NOT NULL,
    sequence        INTEGER NOT NULL DEFAULT 0,
    status_code     INTEGER,
    status_name     TEXT,
    skipped         INTEGER NOT NULL DEFAULT 0,
    simulated       INTEGER NOT NULL DEFAULT 0,
    detail          TEXT,
    detections_json TEXT,
    checked_at      TEXT,
    duration_s      REAL
);

CREATE INDEX IF NOT EXISTS idx_poi_round ON poi_result(round_id, sequence);

CREATE TABLE IF NOT EXISTS event_record (
    id           INTEGER PRIMARY KEY AUTOINCREMENT,
    event_id     TEXT    NOT NULL UNIQUE,
    event_type   TEXT    NOT NULL,
    occurred_at  TEXT    NOT NULL,
    source       TEXT    NOT NULL,
    severity     TEXT    NOT NULL,
    message      TEXT    NOT NULL,
    details_json TEXT,
    round_id     TEXT,
    poi          TEXT,
    simulated    INTEGER NOT NULL DEFAULT 0,
    status       TEXT    NOT NULL DEFAULT 'open',
    handled_by   TEXT,
    handled_at   TEXT,
    created_at   TEXT    NOT NULL
);

CREATE INDEX IF NOT EXISTS idx_event_time ON event_record(occurred_at DESC, id DESC);
CREATE INDEX IF NOT EXISTS idx_event_type ON event_record(event_type, occurred_at DESC);

-- 手动点名检查：与自动巡检完全独立的记录（不进 inspection_round / poi_result）。
-- status 是执行状态（running/completed/stopped/aborted）；
-- status_code/status_name 是正式视觉检查状态（0-6），NULL 表示没有正式结果，
-- 绝不把"没结果"显示成正常。
CREATE TABLE IF NOT EXISTS manual_inspection (
    id              INTEGER PRIMARY KEY AUTOINCREMENT,
    manual_id       TEXT    NOT NULL UNIQUE,
    poi             TEXT    NOT NULL,
    operator_id     TEXT,
    status          TEXT    NOT NULL DEFAULT 'running',
    ros_state       TEXT,
    status_code     INTEGER,
    status_name     TEXT,
    simulated       INTEGER NOT NULL DEFAULT 0,
    detail          TEXT,
    detections_json TEXT,
    result_event_id TEXT,
    started_at      TEXT    NOT NULL,
    checked_at      TEXT,
    finished_at     TEXT,
    error_message   TEXT,
    created_at      TEXT    NOT NULL,
    updated_at      TEXT    NOT NULL
);

CREATE INDEX IF NOT EXISTS idx_manual_started ON manual_inspection(started_at DESC);
CREATE INDEX IF NOT EXISTS idx_manual_status  ON manual_inspection(status);

-- 结果幂等键：同一结果事件重复投递不覆盖已有结果
CREATE UNIQUE INDEX IF NOT EXISTS uq_manual_result_event
    ON manual_inspection(result_event_id) WHERE result_event_id IS NOT NULL;
"""

# event_id 用于 Bridge 重试去重；老库需要补列（见 _migrate）
EVENT_UNIQUE_INDEX = """
CREATE UNIQUE INDEX IF NOT EXISTS uq_poi_event
    ON poi_result(round_id, event_id) WHERE event_id IS NOT NULL
"""

# 与 fire_inspection_interfaces/msg/PoiResult.msg 完全一致的枚举
STATUS_NAMES = {
    0: "NORMAL",
    1: "ABNORMAL",
    2: "UNCERTAIN",
    3: "TIMEOUT",
    4: "NO_FRAME",
    5: "NO_MODEL",
    6: "ERROR",
}

# 视觉永远不该返回"已跳过"，跳过由控制指令产生，单独记 skipped
SKIPPED_STATUS_NAME = "SKIPPED"

ROUND_STATUSES = ("running", "paused", "completed", "stopped", "aborted")
ROUND_OUTCOMES = ("normal", "abnormal", "uncertain")
ROUND_SOURCES = ("real", "demo", "simulated")
ROUND_MODES = ("inspection", "manual")

# 手动点名检查的执行状态（与视觉状态码是两个概念；手动流程不暂停，无 paused）
MANUAL_STATUSES = ("running", "completed", "stopped", "aborted")
MANUAL_TERMINAL_STATUSES = ("completed", "stopped", "aborted")

# 统一异常事件枚举（网页侧建议契约，待机器人端确认）
EVENT_TYPES = (
    "alarm_detected",
    "vision_abnormal",
    "voice_broadcast",
    "comm_failure",
    "comm_restored",
)
EVENT_SEVERITIES = ("info", "warning", "critical")
EVENT_STATUSES = ("open", "acknowledged", "resolved")
EVENT_SOURCE_PATTERN = r"^[A-Za-z0-9_.-]{1,32}$"


def utc_now() -> str:
    """统一使用带时区的 UTC ISO 时间，前端按本地时区展示。"""
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def connect(db_path: Path | str) -> sqlite3.Connection:
    path = Path(db_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    conn = sqlite3.connect(str(path), timeout=10, check_same_thread=False)
    conn.row_factory = sqlite3.Row
    conn.execute("PRAGMA foreign_keys = ON")
    return conn


def init_db(db_path: Path | str) -> None:
    conn = connect(db_path)
    try:
        conn.execute("PRAGMA journal_mode = WAL")
        conn.executescript(SCHEMA)
        _migrate(conn)
        conn.commit()
    finally:
        conn.close()


def _migrate(conn: sqlite3.Connection) -> None:
    """向前兼容已建数据库：补 poi_result.event_id 与 inspection_round.total_pois。"""
    columns = {
        row["name"] for row in conn.execute("PRAGMA table_info(poi_result)").fetchall()
    }
    if "event_id" not in columns:
        conn.execute("ALTER TABLE poi_result ADD COLUMN event_id TEXT")
    conn.execute(EVENT_UNIQUE_INDEX)

    round_columns = {
        row["name"]
        for row in conn.execute("PRAGMA table_info(inspection_round)").fetchall()
    }
    if "total_pois" not in round_columns:
        conn.execute("ALTER TABLE inspection_round ADD COLUMN total_pois INTEGER")
