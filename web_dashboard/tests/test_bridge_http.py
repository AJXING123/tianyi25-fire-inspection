"""Bridge 本机 HTTP 接口测试：令牌、在线判定、控制确认、去重与错误语义。"""

from __future__ import annotations

import json
import urllib.error
import urllib.request

import pytest

from bridge.gateway import GatewayReply, GatewayTimeout, GatewayUnavailable
from bridge.http_app import BridgeHttpServer

TOKEN = "bridge-test-token"


class FakeGateway:
    def __init__(self) -> None:
        self.calls: list[tuple[str, str, str, float]] = []
        self.reply: GatewayReply | Exception = GatewayReply(True, "已接受", "running:inspect:POI_01")
        self.status_payload = {
            "ros_ready": True,
            "control_service_available": True,
            "state_text": "running:inspect:POI_01",
            "detail": "",
        }
        self.status_error: Exception | None = None

    def status(self) -> dict:
        if self.status_error is not None:
            raise self.status_error
        return dict(self.status_payload)

    def control(self, cmd, arg, operator_id, timeout_s):
        self.calls.append((cmd, arg, operator_id, timeout_s))
        if isinstance(self.reply, Exception):
            raise self.reply
        return self.reply


@pytest.fixture
def gateway():
    return FakeGateway()


@pytest.fixture
def bridge(gateway):
    server = BridgeHttpServer(gateway, TOKEN, port=0, control_timeout_s=3.0)
    server.start()
    yield server
    server.stop()


def request(server, method: str, path: str, payload: dict | None = None, token: str = TOKEN):
    headers = {"X-Bridge-Token": token} if token else {}
    data = None
    if payload is not None:
        data = json.dumps(payload).encode("utf-8")
        headers["Content-Type"] = "application/json"
    req = urllib.request.Request(
        f"{server.url}{path}", data=data, headers=headers, method=method
    )
    try:
        with urllib.request.urlopen(req, timeout=5) as resp:
            return resp.status, json.loads(resp.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        return exc.code, json.loads(exc.read().decode("utf-8"))


def test_requires_token(bridge):
    assert request(bridge, "GET", "/status", token="")[0] == 401
    assert request(bridge, "GET", "/status", token="wrong")[0] == 401


def test_status_online_only_when_service_available(bridge, gateway):
    code, body = request(bridge, "GET", "/status")
    assert code == 200
    assert body["online"] is True
    assert body["control_service_available"] is True

    gateway.status_payload["control_service_available"] = False
    code, body = request(bridge, "GET", "/status")
    assert code == 200
    assert body["online"] is False
    assert "不可发现" in body["detail"]


def test_status_gateway_error_reports_offline(bridge, gateway):
    gateway.status_error = RuntimeError("rclpy 未就绪")
    code, body = request(bridge, "GET", "/status")
    assert code == 200
    assert body["online"] is False
    assert "rclpy 未就绪" in body["detail"]


def test_health(bridge):
    code, body = request(bridge, "GET", "/health")
    assert code == 200
    assert body["status"] == "ok"


def test_control_success_marks_dispatched(bridge, gateway):
    code, body = request(
        bridge,
        "POST",
        "/control",
        {"cmd": "start", "arg": "", "operator_id": "op-1", "request_id": "req-1"},
    )
    assert code == 200
    assert body["dispatched"] is True
    assert body["success"] is True
    assert body["current_state"] == "running:inspect:POI_01"
    assert gateway.calls == [("start", "", "op-1", 3.0)]


def test_control_unavailable_is_503(bridge, gateway):
    gateway.reply = GatewayUnavailable("未发现服务")
    code, body = request(bridge, "POST", "/control", {"cmd": "stop"})
    assert code == 503
    assert body["dispatched"] is False
    assert "未发现服务" in body["message"]


def test_control_timeout_is_504(bridge, gateway):
    gateway.reply = GatewayTimeout("未收到响应")
    code, body = request(bridge, "POST", "/control", {"cmd": "pause"})
    assert code == 504
    assert body["dispatched"] is False


def test_control_unknown_command_is_400(bridge, gateway):
    code, body = request(bridge, "POST", "/control", {"cmd": "explode"})
    assert code == 400
    assert body["dispatched"] is False
    assert gateway.calls == []


def test_manual_requires_arg(bridge, gateway):
    code, _ = request(bridge, "POST", "/control", {"cmd": "manual", "arg": ""})
    assert code == 400
    assert gateway.calls == []


def test_duplicate_request_id_is_deduplicated(bridge, gateway):
    payload = {"cmd": "start", "operator_id": "op", "request_id": "same-id"}
    first = request(bridge, "POST", "/control", payload)
    second = request(bridge, "POST", "/control", payload)

    assert first[0] == 200
    assert second[0] == 200
    assert second[1]["deduplicated"] is True
    assert len(gateway.calls) == 1


def test_unknown_path_404(bridge):
    assert request(bridge, "GET", "/nope")[0] == 404
    assert request(bridge, "POST", "/nope", {"cmd": "start"})[0] == 404


def test_invalid_json_body_400(bridge):
    req = urllib.request.Request(
        f"{bridge.url}/control",
        data=b"not-json",
        headers={"X-Bridge-Token": TOKEN, "Content-Type": "application/json"},
        method="POST",
    )
    try:
        urllib.request.urlopen(req, timeout=5)
        raise AssertionError("应当返回 400")
    except urllib.error.HTTPError as exc:
        assert exc.code == 400


def test_empty_token_cannot_start_server(gateway):
    with pytest.raises(ValueError):
        BridgeHttpServer(gateway, "")
