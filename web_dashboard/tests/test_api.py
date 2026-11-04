"""API 行为测试：离线状态、历史查询、详情、控制拒绝、演示数据标记、重启持久性。"""

from __future__ import annotations

from fastapi.testclient import TestClient

from app.auth import AuthManager
from app.db import connect
from app.main import create_app
from app.repository import finish_round, insert_event, insert_round
from app.ros_adapter import DisabledRosAdapter
from app.seed_demo import seed_demo


def test_health_ok(client):
    resp = client.get("/api/health")
    assert resp.status_code == 200
    body = resp.json()
    assert body["status"] == "ok"
    assert body["database"] == "ok"
    assert body["robot"]["online"] is False
    assert body["robot"]["adapter"] == "disabled"


def test_status_offline_without_round(client):
    resp = client.get("/api/status")
    assert resp.status_code == 200
    body = resp.json()
    assert body["robot"]["online"] is False
    assert body["current_round"] is None
    # 离线时不得出现任何伪造的状态文本
    assert body["robot"]["state_text"] is None


def test_rounds_empty(client):
    resp = client.get("/api/rounds")
    assert resp.status_code == 200
    body = resp.json()
    assert body["items"] == []
    assert body["total"] == 0
    assert body["limit"] == 20


def test_index_served(client):
    resp = client.get("/")
    assert resp.status_code == 200
    assert "消防巡检后台" in resp.text


def test_demo_data_is_marked_and_queryable(client, db_path):
    ids = seed_demo(db_path)
    assert len(ids) == 4

    # 演示数据成为"进行中"轮次，但必须带 demo 标记
    status = client.get("/api/status").json()
    assert status["current_round"] is not None
    assert status["current_round"]["is_demo"] is True
    assert status["current_round"]["source"] == "demo"
    assert all(item["simulated"] for item in status["current_round"]["results"])

    listing = client.get("/api/rounds").json()
    assert listing["total"] == 4
    assert all(item["is_demo"] for item in listing["items"])
    assert {item["status"] for item in listing["items"]} == {
        "completed",
        "completed",
        "stopped",
        "running",
    }

    # 演示数据不能混入"真实"来源筛选
    real = client.get("/api/rounds", params={"source": "real"}).json()
    assert real["total"] == 0

    stopped = client.get("/api/rounds", params={"status": "stopped"}).json()
    assert stopped["total"] == 1

    abnormal = client.get("/api/rounds", params={"outcome": "abnormal"}).json()
    assert abnormal["total"] == 1

    detail = client.get(f"/api/rounds/{abnormal['items'][0]['round_id']}").json()
    assert detail["outcome"] == "abnormal"
    assert detail["abnormal_count"] == 1
    abnormal_pois = [r for r in detail["results"] if r["status_code"] == 1]
    assert abnormal_pois
    assert abnormal_pois[0]["simulated"] is True
    assert abnormal_pois[0]["detections"]


def test_round_detail_not_found(client):
    resp = client.get("/api/rounds/not-exist")
    assert resp.status_code == 404
    assert "不存在" in resp.json()["detail"]


def test_control_offline_returns_503_and_writes_nothing(client):
    resp = client.post(
        "/api/control",
        json={"cmd": "start", "arg": "", "operator_id": "tester"},
    )
    assert resp.status_code == 503
    body = resp.json()
    assert body["success"] is False
    assert body["dispatched"] is False
    assert "未向机器人发送" in body["message"]

    # 失败的控制请求不得生成任何巡检记录
    assert client.get("/api/rounds").json()["total"] == 0


def test_control_rejects_invalid_payload(client):
    assert client.post("/api/control", json={"cmd": "fly"}).status_code == 422
    assert (
        client.post("/api/control", json={"cmd": "manual", "arg": ""}).status_code
        == 400
    )
    assert (
        client.post(
            "/api/control", json={"cmd": "skip", "arg": "POI 01"}
        ).status_code
        == 400
    )


def test_filters_reject_unknown_values(client):
    assert client.get("/api/rounds", params={"status": "bogus"}).status_code == 422
    assert (
        client.get("/api/rounds", params={"source": "bogus"}).status_code == 422
    )


def test_round_filters_combine_and_show_aborted(client, db_path):
    """筛选条件可组合；stopped / aborted 轮次能在历史中正常显示。"""
    conn = connect(db_path)
    try:
        done = insert_round(conn, round_id="mix-done", source="real")
        finish_round(conn, done, status="completed", outcome="normal")
        bad = insert_round(conn, round_id="mix-bad", source="real")
        finish_round(conn, bad, status="completed", outcome="abnormal")
        aborted = insert_round(conn, round_id="mix-aborted", source="real")
        finish_round(conn, aborted, status="aborted", outcome="uncertain")
        sim = insert_round(conn, round_id="mix-sim", source="simulated")
        finish_round(conn, sim, status="completed", outcome="normal")
    finally:
        conn.close()

    combo = client.get(
        "/api/rounds",
        params={"status": "completed", "outcome": "normal", "source": "real"},
    ).json()
    assert combo["total"] == 1
    assert combo["items"][0]["round_id"] == "mix-done"

    aborted_list = client.get("/api/rounds", params={"status": "aborted"}).json()
    assert aborted_list["total"] == 1
    assert aborted_list["items"][0]["round_id"] == "mix-aborted"

    stopped = client.get("/api/rounds", params={"status": "stopped"}).json()
    assert stopped["total"] == 0

    sim_page = client.get(
        "/api/rounds", params={"source": "simulated", "outcome": "normal"}
    ).json()
    assert sim_page["total"] == 1
    assert sim_page["items"][0]["is_demo"] is False


def test_health_reports_database_error(client, db_path):
    """数据库文件损坏时健康检查降级，而不是抛 500。"""
    db_path.write_bytes(b"this is not a sqlite database")
    resp = client.get("/api/health")
    assert resp.status_code == 200
    body = resp.json()
    assert body["database"] == "error"
    assert body["status"] == "degraded"

    # 其它接口遇到数据库错误返回 503，而不是未处理的 500
    rounds = client.get("/api/rounds")
    assert rounds.status_code == 503
    assert "数据库" in rounds.json()["detail"]


def test_rounds_and_events_persist_across_restart(tmp_path):
    """服务重启（重新 init_db）后历史轮次与事件仍然可查。"""
    db_path = tmp_path / "restart.db"
    conn = connect(db_path)
    try:
        from app.db import init_db

        init_db(db_path)
        rid = insert_round(
            conn,
            round_id="persist-round",
            source="real",
            status="completed",
            outcome="normal",
            total_pois=2,
        )
        insert_event(
            conn,
            event_id="evt-persist-1",
            event_type="comm_restored",
            source="bridge",
            message="通信恢复",
        )
    finally:
        conn.close()
    assert rid

    app = create_app(
        db_path=db_path,
        ros_adapter=DisabledRosAdapter(),
        auth=AuthManager.skip_auth(),
    )
    with TestClient(app) as restarted:
        rounds = restarted.get("/api/rounds").json()
        assert rounds["total"] == 1
        assert rounds["items"][0]["round_id"] == "persist-round"
        assert rounds["items"][0]["total_pois"] == 2
        events = restarted.get("/api/events").json()
        assert events["total"] == 1
        assert events["items"][0]["event_id"] == "evt-persist-1"
