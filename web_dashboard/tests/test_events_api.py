"""统一异常事件 + 开发模拟入口测试。

覆盖：
- 事件契约字段原样保存（event_id / event_type / timestamp / source / severity /
  message / details），模拟事件带 simulated 标记；
- 同一 event_id 重复上报不产生重复历史记录；
- 事件筛选、分页、详情、标记处理；
- 防空警报事件不自动生成"语音播报成功"事件（播报结果必须独立事件）；
- 开发模拟入口默认关闭，开启后场景写入的数据全部带模拟标记。
"""

from __future__ import annotations

from conftest import BRIDGE_TOKEN

ALARM_ID = "evt-test-alarm01"


def _event_payload(**overrides):
    payload = {
        "event_id": ALARM_ID,
        "event_type": "alarm_detected",
        "timestamp": "2026-10-29T10:00:00+00:00",
        "source": "alarm_monitor",
        "severity": "critical",
        "message": "检测到防空警报",
        "details": {"zone": "3F 东走廊", "voice_broadcast": "pending"},
        "simulated": False,
    }
    payload.update(overrides)
    return payload


# ---------------------------------------------------------------------------
# 事件接入（Bridge / 机器人端）
# ---------------------------------------------------------------------------


def test_ingest_event_requires_token(ingest_client):
    resp = ingest_client.post("/api/ingest/events", json=_event_payload())
    assert resp.status_code == 401


def test_ingest_event_preserves_contract_fields(ingest_client):
    resp = ingest_client.post(
        "/api/ingest/events",
        json=_event_payload(),
        headers={"X-Bridge-Token": BRIDGE_TOKEN},
    )
    assert resp.status_code == 200
    assert resp.json()["stored"] is True

    detail = ingest_client.get(f"/api/events/{ALARM_ID}").json()
    assert detail["event_id"] == ALARM_ID
    assert detail["event_type"] == "alarm_detected"
    assert detail["timestamp"] == "2026-10-29T10:00:00+00:00"
    assert detail["source"] == "alarm_monitor"
    assert detail["severity"] == "critical"
    assert detail["message"] == "检测到防空警报"
    assert detail["details"] == {"zone": "3F 东走廊", "voice_broadcast": "pending"}
    assert detail["simulated"] is False
    assert detail["status"] == "open"

    # 报警事件不自动生成语音播报事件
    voice = ingest_client.get(
        "/api/events", params={"event_type": "voice_broadcast"}
    ).json()
    assert voice["total"] == 0


def test_duplicate_event_id_is_idempotent(ingest_client):
    headers = {"X-Bridge-Token": BRIDGE_TOKEN}
    first = ingest_client.post(
        "/api/ingest/events", json=_event_payload(), headers=headers
    )
    second = ingest_client.post(
        "/api/ingest/events", json=_event_payload(), headers=headers
    )
    assert first.json()["stored"] is True
    assert second.json()["stored"] is False
    assert second.json()["duplicate"] is True
    listing = ingest_client.get("/api/events").json()
    assert listing["total"] == 1


def test_ingest_event_rejects_unknown_type(ingest_client):
    resp = ingest_client.post(
        "/api/ingest/events",
        json=_event_payload(event_type="meteor_strike"),
        headers={"X-Bridge-Token": BRIDGE_TOKEN},
    )
    assert resp.status_code == 422


def test_simulated_event_stays_marked(ingest_client):
    payload = _event_payload(event_id="evt-test-sim0001", simulated=True)
    ingest_client.post(
        "/api/ingest/events",
        json=payload,
        headers={"X-Bridge-Token": BRIDGE_TOKEN},
    )
    detail = ingest_client.get("/api/events/evt-test-sim0001").json()
    assert detail["simulated"] is True


# ---------------------------------------------------------------------------
# 事件查询 / 筛选 / 处理状态
# ---------------------------------------------------------------------------


def test_event_filters_and_pagination(ingest_client):
    headers = {"X-Bridge-Token": BRIDGE_TOKEN}
    for index in range(3):
        ingest_client.post(
            "/api/ingest/events",
            json=_event_payload(
                event_id=f"evt-test-page{index:04d}",
                severity="warning" if index == 0 else "info",
                event_type="vision_abnormal" if index == 0 else "comm_restored",
            ),
            headers=headers,
        )

    only_critical = ingest_client.get(
        "/api/events", params={"severity": "critical"}
    ).json()
    assert only_critical["total"] == 0

    vision = ingest_client.get(
        "/api/events", params={"event_type": "vision_abnormal"}
    ).json()
    assert vision["total"] == 1
    assert vision["items"][0]["source"] == "alarm_monitor"

    page = ingest_client.get("/api/events", params={"limit": 2, "offset": 0}).json()
    assert len(page["items"]) == 2
    assert page["total"] == 3

    bad = ingest_client.get("/api/events", params={"event_type": "bogus"})
    assert bad.status_code == 422


def test_event_detail_not_found(client):
    assert client.get("/api/events/evt-not-exist").status_code == 404


def test_events_empty_state(client):
    resp = client.get("/api/events")
    assert resp.status_code == 200
    body = resp.json()
    assert body["items"] == []
    assert body["total"] == 0
    assert body["limit"] == 20
    assert body["offset"] == 0


def test_event_ack_marks_handled(ingest_client):
    ingest_client.post(
        "/api/ingest/events",
        json=_event_payload(),
        headers={"X-Bridge-Token": BRIDGE_TOKEN},
    )
    resp = ingest_client.post(
        f"/api/events/{ALARM_ID}/ack", json={"operator_id": "tester"}
    )
    assert resp.status_code == 200
    body = resp.json()
    assert body["status"] == "acknowledged"
    assert body["handled_by"] == "tester"
    assert body["handled_at"]

    # 重复标记幂等，不报错
    again = ingest_client.post(f"/api/events/{ALARM_ID}/ack", json={})
    assert again.status_code == 200
    assert again.json()["status"] == "acknowledged"


def test_event_ack_not_found(client):
    resp = client.post("/api/events/evt-not-exist/ack", json={})
    assert resp.status_code == 404


# ---------------------------------------------------------------------------
# 开发模拟入口
# ---------------------------------------------------------------------------


def test_dev_entry_disabled_by_default(client):
    assert client.get("/api/dev/scenarios").status_code == 403
    assert client.post("/api/dev/scenarios/alarm").status_code == 403


def test_dev_scenarios_listing(dev_client):
    resp = dev_client.get("/api/dev/scenarios")
    assert resp.status_code == 200
    body = resp.json()
    assert body["simulated"] is True
    keys = {item["key"] for item in body["items"]}
    assert {
        "normal_round",
        "abnormal_round",
        "alarm",
        "voice_success",
        "voice_failure",
        "connection_lost",
        "connection_restored",
        "duplicate_event",
        "full_flow",
    } <= keys


def test_dev_control_never_reaches_robot(dev_client):
    """开发模拟模式下控制接口仍保持离线语义，不触发任何真实机器人动作。"""
    resp = dev_client.post("/api/control", json={"cmd": "start"})
    assert resp.status_code == 503
    assert resp.json()["dispatched"] is False
    assert dev_client.get("/api/status").json()["robot"]["online"] is False


def test_dev_full_flow_covers_whole_demo_chain(dev_client):
    body = dev_client.post("/api/dev/scenarios/full_flow").json()
    assert body["simulated"] is True
    round_id = body["round_id"]
    assert len(body["event_ids"]) == 3

    detail = dev_client.get(f"/api/rounds/{round_id}").json()
    assert detail["status"] == "completed"
    assert detail["outcome"] == "abnormal"
    assert detail["source"] == "simulated"
    assert detail["total_pois"] == 3
    assert detail["processed_count"] == 3
    assert [r["status_code"] for r in detail["results"]] == [0, 1, 0]
    assert all(item["simulated"] for item in detail["results"])

    events = dev_client.get("/api/events", params={"round_id": round_id}).json()
    assert {item["event_type"] for item in events["items"]} == {
        "vision_abnormal",
        "alarm_detected",
        "voice_broadcast",
    }
    assert all(item["simulated"] for item in events["items"])
    # 警报与播报是两条独立事件：警报不代表播报完成
    alarm = next(i for i in events["items"] if i["event_type"] == "alarm_detected")
    voice = next(i for i in events["items"] if i["event_type"] == "voice_broadcast")
    assert alarm["details"]["voice_broadcast"] == "pending"
    assert voice["details"]["result"] == "success"

    # 结束后历史记录可查，且不会混入真实来源
    history = dev_client.get("/api/rounds", params={"source": "simulated"}).json()
    assert any(item["round_id"] == round_id for item in history["items"])
    assert dev_client.get("/api/rounds", params={"source": "real"}).json()["total"] == 0


def test_dev_unknown_scenario(dev_client):
    assert dev_client.post("/api/dev/scenarios/nope").status_code == 404


def test_dev_alarm_creates_simulated_event_not_broadcast(dev_client):
    resp = dev_client.post("/api/dev/scenarios/alarm")
    assert resp.status_code == 200
    body = resp.json()
    assert body["simulated"] is True
    assert body["round_id"] is None

    events = dev_client.get("/api/events").json()
    assert events["total"] == 1
    alarm = events["items"][0]
    assert alarm["event_type"] == "alarm_detected"
    assert alarm["simulated"] is True
    assert alarm["details"]["voice_broadcast"] == "pending"

    # 收到警报不代表播报完成
    voice = dev_client.get(
        "/api/events", params={"event_type": "voice_broadcast"}
    ).json()
    assert voice["total"] == 0


def test_dev_voice_success_and_failure_are_separate_events(dev_client):
    dev_client.post("/api/dev/scenarios/voice_success")
    dev_client.post("/api/dev/scenarios/voice_failure")
    items = dev_client.get(
        "/api/events", params={"event_type": "voice_broadcast"}
    ).json()["items"]
    assert len(items) == 2
    results = {item["details"]["result"] for item in items}
    assert results == {"success", "failure"}
    assert all(item["simulated"] for item in items)


def test_dev_connection_events(dev_client):
    dev_client.post("/api/dev/scenarios/connection_lost")
    dev_client.post("/api/dev/scenarios/connection_restored")
    items = dev_client.get("/api/events").json()["items"]
    assert {item["event_type"] for item in items} == {"comm_failure", "comm_restored"}
    # 模拟通信事件不会让机器人状态变"在线"
    assert dev_client.get("/api/status").json()["robot"]["online"] is False


def test_dev_normal_round_progress_and_results(dev_client):
    body = dev_client.post("/api/dev/scenarios/normal_round").json()
    round_id = body["round_id"]

    status = dev_client.get("/api/status").json()
    # 轮次已结束，不应作为"进行中"轮次
    assert status["current_round"] is None

    detail = dev_client.get(f"/api/rounds/{round_id}").json()
    assert detail["source"] == "simulated"
    assert detail["status"] == "completed"
    assert detail["outcome"] == "normal"
    assert detail["total_pois"] == 3
    assert detail["processed_count"] == 3
    assert all(item["simulated"] for item in detail["results"])

    # 模拟轮次不会混进真实来源
    real = dev_client.get("/api/rounds", params={"source": "real"}).json()
    assert real["total"] == 0


def test_dev_abnormal_round_creates_vision_event(dev_client):
    body = dev_client.post("/api/dev/scenarios/abnormal_round").json()
    round_id = body["round_id"]

    detail = dev_client.get(f"/api/rounds/{round_id}").json()
    assert detail["outcome"] == "abnormal"
    abnormal = [r for r in detail["results"] if r["status_code"] == 1]
    assert len(abnormal) == 1

    events = dev_client.get(
        "/api/events", params={"round_id": round_id}
    ).json()
    assert events["total"] == 1
    event = events["items"][0]
    assert event["event_type"] == "vision_abnormal"
    assert event["poi"] == "POI_02"
    assert event["simulated"] is True


def test_dev_duplicate_event_scenario_keeps_one_record(dev_client):
    body = dev_client.post("/api/dev/scenarios/duplicate_event").json()
    assert body["duplicate"] == {"first_inserted": True, "second_inserted": False}

    dev_client.post("/api/dev/scenarios/duplicate_event")
    events = dev_client.get("/api/events").json()
    assert events["total"] == 2  # 两次运行各一条，每次运行内部只保留一条
    event_ids = [item["event_id"] for item in events["items"]]
    assert len(event_ids) == len(set(event_ids))
