"""Bridge 数据接入 API 测试：令牌、幂等、原始字段保留、终态与 outcome。"""

from __future__ import annotations

from fastapi.testclient import TestClient

from app.auth import AuthManager
from app.main import create_app
from conftest import BRIDGE_TOKEN, make_settings

ROUND_ID = "run-0123456789abcdef"
HEADERS = {"X-Bridge-Token": BRIDGE_TOKEN}


def start_round(client, round_id: str = ROUND_ID, **extra):
    payload = {"round_id": round_id, **extra}
    return client.post("/api/ingest/rounds", json=payload, headers=HEADERS)


def add_result(client, round_id: str, **payload):
    body = {
        "event_id": "evt-" + "a" * 32,
        "poi": "POI_01",
        "status_code": 0,
        **payload,
    }
    return client.post(
        f"/api/ingest/rounds/{round_id}/results", json=body, headers=HEADERS
    )


def test_ingest_disabled_without_token(tmp_path):
    app = create_app(
        settings=make_settings(tmp_path / "no_token.db", token=""),
        auth=AuthManager.skip_auth(),
    )
    with TestClient(app) as client:
        resp = client.post("/api/ingest/rounds", json={"round_id": ROUND_ID})
    assert resp.status_code == 503


def test_invalid_token_rejected(ingest_client):
    resp = ingest_client.post(
        "/api/ingest/rounds",
        json={"round_id": ROUND_ID},
        headers={"X-Bridge-Token": "wrong"},
    )
    assert resp.status_code == 401


def test_round_start_is_idempotent_and_real(ingest_client):
    first = start_round(ingest_client)
    assert first.status_code == 201
    assert first.json()["created"] is True

    second = start_round(ingest_client)
    assert second.status_code == 200
    assert second.json()["created"] is False

    listed = ingest_client.get("/api/rounds").json()
    assert listed["total"] == 1
    item = listed["items"][0]
    assert item["round_id"] == ROUND_ID
    assert item["source"] == "real"
    assert item["is_demo"] is False
    assert item["status"] == "running"


def test_result_preserves_raw_fields(ingest_client):
    start_round(ingest_client)
    resp = add_result(
        ingest_client,
        ROUND_ID,
        status_code=5,
        simulated=True,
        detail="模型未加载",
        detections_json="[]",
        checked_at="2026-10-28T01:02:03+00:00",
    )
    assert resp.status_code == 200
    assert resp.json()["stored"] is True

    detail = ingest_client.get(f"/api/rounds/{ROUND_ID}").json()
    result = detail["results"][0]
    assert result["status_code"] == 5
    assert result["status_name"] == "NO_MODEL"
    assert result["simulated"] is True
    assert result["detail"] == "模型未加载"
    assert result["detections"] == []
    assert result["checked_at"] == "2026-10-28T01:02:03+00:00"
    assert result["sequence"] == 0


def test_duplicate_event_is_deduplicated(ingest_client):
    start_round(ingest_client)
    first = add_result(ingest_client, ROUND_ID)
    second = add_result(ingest_client, ROUND_ID)

    assert first.json()["duplicate"] is False
    assert second.json()["duplicate"] is True
    assert second.json()["stored"] is False
    detail = ingest_client.get(f"/api/rounds/{ROUND_ID}").json()
    assert len(detail["results"]) == 1


def test_result_requires_json_array(ingest_client):
    start_round(ingest_client)
    resp = add_result(ingest_client, ROUND_ID, detections_json='{"a": 1}')
    assert resp.status_code == 422

    resp = add_result(ingest_client, ROUND_ID, detections_json="not-json")
    assert resp.status_code == 422


def test_simulated_result_must_not_carry_detections(ingest_client):
    start_round(ingest_client)
    resp = add_result(
        ingest_client,
        ROUND_ID,
        simulated=True,
        detections_json='[{"cls": "smoke", "conf": 0.9}]',
    )
    assert resp.status_code == 422


def test_unknown_round_returns_404(ingest_client):
    assert add_result(ingest_client, "run-ffffffffffffffff").status_code == 404
    resp = ingest_client.post(
        "/api/ingest/rounds/run-ffffffffffffffff/state",
        json={"status": "paused"},
        headers=HEADERS,
    )
    assert resp.status_code == 404


def test_state_update(ingest_client):
    start_round(ingest_client)
    resp = ingest_client.post(
        f"/api/ingest/rounds/{ROUND_ID}/state",
        json={"status": "paused", "ros_state": "paused:inspect:POI_01"},
        headers=HEADERS,
    )
    assert resp.status_code == 200
    detail = ingest_client.get(f"/api/rounds/{ROUND_ID}").json()
    assert detail["status"] == "paused"
    assert detail["ros_state"] == "paused:inspect:POI_01"


def test_finish_computes_outcome_and_blocks_late_writes(ingest_client):
    start_round(ingest_client)
    add_result(ingest_client, ROUND_ID, event_id="evt-" + "b" * 32, status_code=0)
    add_result(ingest_client, ROUND_ID, event_id="evt-" + "c" * 32, status_code=1)

    resp = ingest_client.post(
        f"/api/ingest/rounds/{ROUND_ID}/finish",
        json={"status": "completed", "summary": "ROS 状态：done"},
        headers=HEADERS,
    )
    assert resp.status_code == 200
    assert resp.json()["outcome"] == "abnormal"
    assert resp.json()["idempotent"] is False

    detail = ingest_client.get(f"/api/rounds/{ROUND_ID}").json()
    assert detail["status"] == "completed"
    assert detail["finished_at"] is not None

    # 终态后拒绝再写结果 / 状态
    late = add_result(ingest_client, ROUND_ID, event_id="evt-" + "d" * 32)
    assert late.status_code == 409
    state = ingest_client.post(
        f"/api/ingest/rounds/{ROUND_ID}/state",
        json={"status": "running"},
        headers=HEADERS,
    )
    assert state.status_code == 409

    # finish 重复调用是幂等的
    again = ingest_client.post(
        f"/api/ingest/rounds/{ROUND_ID}/finish",
        json={"status": "aborted"},
        headers=HEADERS,
    )
    assert again.status_code == 200
    assert again.json()["idempotent"] is True
    assert again.json()["status"] == "completed"


def test_aborted_without_reason_gets_explicit_message(ingest_client):
    start_round(ingest_client)
    resp = ingest_client.post(
        f"/api/ingest/rounds/{ROUND_ID}/finish",
        json={"status": "aborted"},
        headers=HEADERS,
    )
    assert resp.status_code == 200
    detail = ingest_client.get(f"/api/rounds/{ROUND_ID}").json()
    assert detail["status"] == "aborted"
    assert "未提供结构化原因" in detail["error_message"]


def test_no_results_means_no_outcome(ingest_client):
    start_round(ingest_client)
    resp = ingest_client.post(
        f"/api/ingest/rounds/{ROUND_ID}/finish",
        json={"status": "stopped"},
        headers=HEADERS,
    )
    assert resp.status_code == 200
    assert resp.json()["outcome"] is None


def test_ingest_round_not_mixed_with_demo(ingest_client):
    start_round(ingest_client)
    demo = ingest_client.get("/api/rounds", params={"source": "demo"}).json()
    real = ingest_client.get("/api/rounds", params={"source": "real"}).json()
    assert demo["total"] == 0
    assert real["total"] == 1
