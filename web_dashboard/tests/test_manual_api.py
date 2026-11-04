"""手动点名检查数据层与 API 测试：幂等、结果语义、与自动巡检的隔离。"""

from __future__ import annotations

import pytest

from app.db import connect, init_db
from app.repository import (
    create_manual_if_absent,
    finish_manual,
    get_manual,
    list_manuals,
    list_rounds,
    record_manual_result,
    update_manual_state,
)
from conftest import BRIDGE_TOKEN

HEADERS = {"X-Bridge-Token": BRIDGE_TOKEN}
MANUAL_ID = "man-0123456789abcdef"


@pytest.fixture
def conn(tmp_path):
    db_path = tmp_path / "manual_repo.db"
    init_db(db_path)
    connection = connect(db_path)
    yield connection
    connection.close()


def create(conn, manual_id: str = MANUAL_ID, **kwargs):
    kwargs.setdefault("poi", "POI_01")
    return create_manual_if_absent(conn, manual_id=manual_id, **kwargs)


# -- repository -------------------------------------------------------------


def test_create_is_idempotent(conn):
    data, created = create(conn, operator_id="op-alice",
                           ros_state="running:manual:POI_01")
    assert created is True
    assert data["status"] == "running"
    assert data["operator_id"] == "op-alice"
    assert data["status_code"] is None  # 未完成检查没有视觉状态
    assert data["status_name"] is None

    again, created_again = create(conn, operator_id="op-bob")
    assert created_again is False
    assert again["operator_id"] == "op-alice"  # 重复投递不覆盖

    items, total = list_manuals(conn)
    assert total == 1
    assert items[0]["manual_id"] == MANUAL_ID


def test_create_rejects_empty_poi(conn):
    with pytest.raises(ValueError):
        create(conn, poi="  ")


def test_update_state_only_while_running(conn):
    create(conn)
    assert update_manual_state(conn, MANUAL_ID, ros_state="running:manual:inspect:POI_01")
    assert get_manual(conn, MANUAL_ID)["ros_state"] == "running:manual:inspect:POI_01"

    assert finish_manual(conn, MANUAL_ID, status="completed")
    assert not update_manual_state(conn, MANUAL_ID, ros_state="running:manual:speak:POI_01")
    assert get_manual(conn, MANUAL_ID)["ros_state"] == "running:manual:inspect:POI_01"


def test_record_result_lifecycle(conn):
    create(conn)
    outcome = record_manual_result(
        conn, MANUAL_ID, status_code=1, simulated=True,
        detail="检出异常", detections_json="[]",
        checked_at="2026-10-31T01:00:00+00:00", event_id="evt-result-1",
    )
    assert outcome == "stored"

    data = get_manual(conn, MANUAL_ID)
    assert data["status_code"] == 1
    assert data["status_name"] == "ABNORMAL"  # 名称由枚举推导，不接受外部文本
    assert data["simulated"] is True
    assert data["detections"] == []
    assert data["checked_at"] == "2026-10-31T01:00:00+00:00"

    # 同一结果事件重复投递：幂等
    assert record_manual_result(
        conn, MANUAL_ID, status_code=1, simulated=True, event_id="evt-result-1"
    ) == "duplicate"
    # 不同 event_id 的第二份结果：冲突，不覆盖
    assert record_manual_result(
        conn, MANUAL_ID, status_code=0, event_id="evt-result-2"
    ) == "conflict"
    assert get_manual(conn, MANUAL_ID)["status_code"] == 1


def test_record_result_rejects_invalid_status_code(conn):
    create(conn)
    with pytest.raises(ValueError):
        record_manual_result(conn, MANUAL_ID, status_code=7, event_id="evt-x")
    with pytest.raises(ValueError):
        record_manual_result(conn, MANUAL_ID, status_code=-1, event_id="evt-x")


def test_result_backfill_after_finish(conn):
    """已结束但无结果的记录允许补录（容忍结果/状态话题乱序）。"""
    create(conn)
    assert finish_manual(conn, MANUAL_ID, status="completed")
    outcome = record_manual_result(
        conn, MANUAL_ID, status_code=0, detail="正常", event_id="evt-late"
    )
    assert outcome == "stored"
    data = get_manual(conn, MANUAL_ID)
    assert data["status"] == "completed"
    assert data["status_code"] == 0


def test_finish_manual_transitions(conn):
    create(conn)
    assert finish_manual(conn, MANUAL_ID, status="stopped",
                       error_message="人工停止")
    data = get_manual(conn, MANUAL_ID)
    assert data["status"] == "stopped"
    assert data["error_message"] == "人工停止"
    assert data["finished_at"]

    # 已终态不可再结束
    assert not finish_manual(conn, MANUAL_ID, status="completed")
    with pytest.raises(ValueError):
        finish_manual(conn, "man-other00000000000", status="flying")


def test_list_manuals_filter_and_pagination(conn):
    create(conn, manual_id="man-a111111111111111", poi="POI_01")
    create(conn, manual_id="man-a222222222222222", poi="POI_02")
    finish_manual(conn, "man-a222222222222222", status="completed")
    create(conn, manual_id="man-a333333333333333", poi="POI_03")
    finish_manual(conn, "man-a333333333333333", status="aborted")

    items, total = list_manuals(conn)
    assert total == 3
    running, running_total = list_manuals(conn, status="running")
    assert running_total == 1
    assert running[0]["poi"] == "POI_01"
    _, completed_total = list_manuals(conn, status="completed")
    assert completed_total == 1

    page, page_total = list_manuals(conn, limit=2, offset=0)
    assert page_total == 3
    assert len(page) == 2

    with pytest.raises(ValueError):
        list_manuals(conn, status="paused")  # 手动流程没有 paused


def test_manual_records_isolated_from_rounds(conn):
    """手动检查绝不进入自动巡检的轮次统计。"""
    create(conn)
    rounds, total = list_rounds(conn)
    assert total == 0
    assert rounds == []


# -- ingest API -------------------------------------------------------------


def start_manual(client, manual_id: str = MANUAL_ID, **extra):
    payload = {"manual_id": manual_id, "poi": "POI_01", **extra}
    return client.post("/api/ingest/manual-inspections", json=payload, headers=HEADERS)


def post_result(client, manual_id: str = MANUAL_ID, **extra):
    body = {
        "event_id": f"{manual_id}-result",
        "status_code": 1,
        "simulated": False,
        **extra,
    }
    return client.post(
        f"/api/ingest/manual-inspections/{manual_id}/result", json=body, headers=HEADERS
    )


def test_manual_ingest_requires_token(ingest_client):
    resp = ingest_client.post(
        "/api/ingest/manual-inspections",
        json={"manual_id": MANUAL_ID, "poi": "POI_01"},
    )
    assert resp.status_code == 401


def test_manual_start_idempotent(ingest_client):
    first = start_manual(ingest_client, operator_id="op-alice")
    assert first.status_code == 201
    assert first.json()["created"] is True

    second = start_manual(ingest_client)
    assert second.status_code == 200
    assert second.json()["created"] is False

    detail = ingest_client.get(f"/api/manual-inspections/{MANUAL_ID}").json()
    assert detail["operator_id"] == "op-alice"
    assert detail["status"] == "running"


def test_manual_state_update_and_terminal_rejection(ingest_client):
    resp = ingest_client.post(
        "/api/ingest/manual-inspections/man-ffffffffffffffff/state",
        json={"ros_state": "running:manual:POI_01"}, headers=HEADERS,
    )
    assert resp.status_code == 404

    start_manual(ingest_client)
    ok = ingest_client.post(
        f"/api/ingest/manual-inspections/{MANUAL_ID}/state",
        json={"ros_state": "running:manual:inspect:POI_01"}, headers=HEADERS,
    )
    assert ok.status_code == 200

    ingest_client.post(
        f"/api/ingest/manual-inspections/{MANUAL_ID}/finish",
        json={"status": "completed"}, headers=HEADERS,
    )
    rejected = ingest_client.post(
        f"/api/ingest/manual-inspections/{MANUAL_ID}/state",
        json={"ros_state": "idle"}, headers=HEADERS,
    )
    assert rejected.status_code == 409


def test_manual_result_endpoints(ingest_client):
    assert post_result(ingest_client).status_code == 404  # 记录不存在

    start_manual(ingest_client)
    stored = post_result(
        ingest_client, detail="检出异常", detections_json='[{"cls": "smoke", "conf": 0.8}]'
    )
    assert stored.status_code == 200
    assert stored.json()["stored"] is True

    duplicate = post_result(ingest_client)
    assert duplicate.status_code == 200
    assert duplicate.json()["duplicate"] is True

    conflict = post_result(ingest_client, event_id="evt-different-result")
    assert conflict.status_code == 409

    detail = ingest_client.get(f"/api/manual-inspections/{MANUAL_ID}").json()
    assert detail["status_code"] == 1
    assert detail["status_name"] == "ABNORMAL"
    assert detail["detections"] == [{"cls": "smoke", "conf": 0.8}]


def test_manual_result_validation(ingest_client):
    start_manual(ingest_client)
    bad_json = post_result(ingest_client, detections_json="not-json")
    assert bad_json.status_code == 422

    simulated_with_detections = post_result(
        ingest_client, simulated=True, detections_json='[{"cls": "fire"}]'
    )
    assert simulated_with_detections.status_code == 422

    out_of_range = post_result(ingest_client, status_code=7)
    assert out_of_range.status_code == 422


def test_manual_finish_and_idempotency(ingest_client):
    resp = ingest_client.post(
        "/api/ingest/manual-inspections/man-ffffffffffffffff/finish",
        json={"status": "completed"}, headers=HEADERS,
    )
    assert resp.status_code == 404

    start_manual(ingest_client)
    first = ingest_client.post(
        f"/api/ingest/manual-inspections/{MANUAL_ID}/finish",
        json={"status": "completed"}, headers=HEADERS,
    )
    assert first.status_code == 200
    assert first.json()["idempotent"] is False

    again = ingest_client.post(
        f"/api/ingest/manual-inspections/{MANUAL_ID}/finish",
        json={"status": "stopped"}, headers=HEADERS,
    )
    assert again.status_code == 200
    assert again.json()["idempotent"] is True
    assert again.json()["status"] == "completed"  # 保持首次终态

    detail = ingest_client.get(f"/api/manual-inspections/{MANUAL_ID}").json()
    assert detail["status"] == "completed"
    assert detail["finished_at"]


def test_manual_aborted_default_error_message(ingest_client):
    start_manual(ingest_client)
    ingest_client.post(
        f"/api/ingest/manual-inspections/{MANUAL_ID}/finish",
        json={"status": "aborted"}, headers=HEADERS,
    )
    detail = ingest_client.get(f"/api/manual-inspections/{MANUAL_ID}").json()
    assert detail["status"] == "aborted"
    assert detail["error_message"]  # 异常必须有明确语义


# -- 查询 API ---------------------------------------------------------------


def test_manual_list_and_detail_api(ingest_client):
    start_manual(ingest_client, manual_id="man-c111111111111111", poi="POI_01",
                 operator_id="op-a")
    start_manual(ingest_client, manual_id="man-c222222222222222", poi="POI_02")
    ingest_client.post(
        "/api/ingest/manual-inspections/man-c222222222222222/finish",
        json={"status": "stopped", "error_message": "人工停止"}, headers=HEADERS,
    )

    listed = ingest_client.get("/api/manual-inspections").json()
    assert listed["total"] == 2

    running = ingest_client.get("/api/manual-inspections?status=running").json()
    assert running["total"] == 1
    assert running["items"][0]["poi"] == "POI_01"

    stopped = ingest_client.get("/api/manual-inspections?status=stopped").json()
    assert stopped["items"][0]["error_message"] == "人工停止"

    invalid = ingest_client.get("/api/manual-inspections?status=paused")
    assert invalid.status_code == 422

    missing = ingest_client.get("/api/manual-inspections/man-ffffffffffffffff")
    assert missing.status_code == 404

    # 手动记录不混入自动巡检列表
    rounds = ingest_client.get("/api/rounds").json()
    assert rounds["total"] == 0
