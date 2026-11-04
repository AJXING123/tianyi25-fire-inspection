"""数据层测试：枚举校验、跳过语义、结果分类、分页、老库迁移、进度计数。"""

from __future__ import annotations

import sqlite3

import pytest

from app.db import connect, init_db
from app.repository import (
    classify_outcome,
    finish_round,
    insert_event,
    insert_poi_result,
    insert_round,
    list_rounds,
)


@pytest.fixture
def conn(tmp_path):
    db_path = tmp_path / "repo.db"
    init_db(db_path)  # 可重复执行（IF NOT EXISTS）
    init_db(db_path)
    connection = connect(db_path)
    yield connection
    connection.close()


def test_insert_and_finish_round(conn):
    rid = insert_round(conn, round_id="r1", source="real", operator_id="alice")
    assert finish_round(
        conn, rid, status="completed", outcome="normal", summary="全部正常"
    )
    rounds, total = list_rounds(conn)
    assert total == 1
    row = rounds[0]
    assert row["round_id"] == "r1"
    assert row["status"] == "completed"
    assert row["outcome"] == "normal"
    assert row["is_demo"] is False
    assert row["finished_at"]


def test_invalid_enums_rejected(conn):
    with pytest.raises(ValueError):
        insert_round(conn, source="fake")
    with pytest.raises(ValueError):
        insert_round(conn, status="flying")
    with pytest.raises(ValueError):
        list_rounds(conn, status="unknown")


def test_demo_flag_is_derived_from_source(conn):
    demo_id = insert_round(conn, source="demo", status="running")
    real_id = insert_round(conn, source="real", status="running")
    rounds, _ = list_rounds(conn)
    flags = {row["round_id"]: row["is_demo"] for row in rounds}
    assert flags[demo_id] is True
    assert flags[real_id] is False


def test_skip_is_not_a_visual_status(conn):
    rid = insert_round(conn, round_id="r2")
    with pytest.raises(ValueError):
        insert_poi_result(conn, round_id=rid, poi="POI_01", skipped=True, status_code=0)
    insert_poi_result(conn, round_id=rid, poi="POI_01", skipped=True)
    with pytest.raises(ValueError):
        insert_poi_result(conn, round_id=rid, poi="POI_02", status_code=9)


def test_classify_outcome(conn):
    rid = insert_round(conn, round_id="r3")
    assert classify_outcome(conn, rid) is None

    insert_poi_result(conn, round_id=rid, poi="POI_01", sequence=0, status_code=0)
    assert classify_outcome(conn, rid) == "normal"

    # NO_MODEL 不得落成正常
    insert_poi_result(conn, round_id=rid, poi="POI_02", sequence=1, status_code=5)
    assert classify_outcome(conn, rid) == "uncertain"

    insert_poi_result(conn, round_id=rid, poi="POI_03", sequence=2, status_code=1)
    assert classify_outcome(conn, rid) == "abnormal"


def test_pagination_and_order(conn):
    for index in range(3):
        insert_round(
            conn,
            round_id=f"r{index}",
            started_at=f"2026-10-2{index}T10:00:00+00:00",
        )
    page, total = list_rounds(conn, limit=2, offset=0)
    assert total == 3
    assert [row["round_id"] for row in page] == ["r2", "r1"]
    page2, _ = list_rounds(conn, limit=2, offset=2)
    assert [row["round_id"] for row in page2] == ["r0"]


def test_progress_counts_skipped_but_not_recorded(conn):
    """进度按已处理点位计算（含跳过），已记录只统计真实视觉结果。"""
    rid = insert_round(conn, round_id="r5", total_pois=3)
    insert_poi_result(conn, round_id=rid, poi="POI_01", sequence=0, status_code=0)
    insert_poi_result(conn, round_id=rid, poi="POI_02", sequence=1, status_code=1)
    insert_poi_result(conn, round_id=rid, poi="POI_03", sequence=2, skipped=True)
    row = list_rounds(conn, limit=1)[0][0]
    assert row["round_id"] == rid
    assert row["total_pois"] == 3
    assert row["processed_count"] == 3
    assert row["recorded_count"] == 2


def test_legacy_database_migration_keeps_old_rows(tmp_path):
    """第四阶段前的老库缺少 total_pois / event_id / event_record，需无损迁移。"""
    db_path = tmp_path / "legacy.db"
    raw = sqlite3.connect(db_path)
    raw.executescript(
        """
        CREATE TABLE inspection_round (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            round_id TEXT NOT NULL UNIQUE,
            mode TEXT NOT NULL DEFAULT 'inspection',
            source TEXT NOT NULL DEFAULT 'real',
            is_demo INTEGER NOT NULL DEFAULT 0,
            status TEXT NOT NULL DEFAULT 'running',
            outcome TEXT, operator_id TEXT, ros_state TEXT, summary TEXT,
            error_message TEXT, started_at TEXT NOT NULL, finished_at TEXT,
            created_at TEXT NOT NULL, updated_at TEXT NOT NULL
        );
        CREATE TABLE poi_result (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            round_id TEXT NOT NULL,
            poi TEXT NOT NULL,
            sequence INTEGER NOT NULL DEFAULT 0,
            status_code INTEGER, status_name TEXT,
            skipped INTEGER NOT NULL DEFAULT 0,
            simulated INTEGER NOT NULL DEFAULT 0,
            detail TEXT, detections_json TEXT,
            checked_at TEXT, duration_s REAL
        );
        INSERT INTO inspection_round
          (round_id, status, started_at, created_at, updated_at)
        VALUES ('legacy-1', 'completed', '2026-10-01T00:00:00+00:00',
                '2026-10-01T00:00:00+00:00', '2026-10-01T00:00:00+00:00');
        INSERT INTO poi_result (round_id, poi, sequence, status_code, status_name)
        VALUES ('legacy-1', 'POI_01', 0, 0, 'NORMAL');
        """
    )
    raw.commit()
    raw.close()

    init_db(db_path)  # 应补列并创建新表，不丢老数据

    check = sqlite3.connect(db_path)
    check.row_factory = sqlite3.Row
    round_cols = {row["name"] for row in check.execute("PRAGMA table_info(inspection_round)")}
    poi_cols = {row["name"] for row in check.execute("PRAGMA table_info(poi_result)")}
    assert "total_pois" in round_cols
    assert "event_id" in poi_cols
    tables = {row[0] for row in check.execute("SELECT name FROM sqlite_master WHERE type='table'")}
    assert "event_record" in tables
    assert check.execute("SELECT COUNT(*) FROM inspection_round").fetchone()[0] == 1
    assert check.execute("SELECT COUNT(*) FROM poi_result").fetchone()[0] == 1
    check.close()

    # 迁移后新功能可用
    conn = connect(db_path)
    try:
        assert insert_event(conn, event_id="evt-legacy-1", event_type="alarm_detected",
                            source="alarm_monitor", message="迁移后写入")
        assert insert_poi_result(
            conn, round_id="legacy-1", poi="POI_02", sequence=1,
            status_code=0, event_id="evt-poi-legacy-1",
        )
    finally:
        conn.close()
