"""BridgeRosAdapter 测试：在线判定、控制确认、离线错误、配置校验。"""

from __future__ import annotations

import pytest

from app.config import Settings
from app.ros_adapter import (
    BridgeRosAdapter,
    DisabledRosAdapter,
    RosUnavailable,
    build_ros_adapter,
)
from bridge.gateway import GatewayReply, GatewayTimeout, GatewayUnavailable
from bridge.http_app import BridgeHttpServer

TOKEN = "adapter-test-token"


class CountingGateway:
    def __init__(self) -> None:
        self.status_calls = 0
        self.control_calls = 0
        self.online = True
        self.reply: GatewayReply | Exception = GatewayReply(True, "已接受", "running:inspect:POI_01")

    def status(self) -> dict:
        self.status_calls += 1
        return {
            "ros_ready": True,
            "control_service_available": self.online,
            "state_text": "running:inspect:POI_01" if self.online else None,
            "detail": "",
        }

    def control(self, cmd, arg, operator_id, timeout_s):
        self.control_calls += 1
        if isinstance(self.reply, Exception):
            raise self.reply
        return self.reply


@pytest.fixture
def gateway():
    return CountingGateway()


@pytest.fixture
def server(gateway):
    srv = BridgeHttpServer(gateway, TOKEN, port=0)
    srv.start()
    yield srv
    srv.stop()


def adapter(server, **kwargs) -> BridgeRosAdapter:
    return BridgeRosAdapter(server.url, TOKEN, status_timeout_s=2.0, **kwargs)


def test_online_when_bridge_reports_online(server, gateway):
    status = adapter(server).get_robot_status()
    assert status.online is True
    assert status.adapter == "bridge"
    assert status.state_text == "running:inspect:POI_01"


def test_offline_when_service_not_available(server, gateway):
    gateway.online = False
    status = adapter(server).get_robot_status()
    assert status.online is False
    assert "不可发现" in status.detail or "未就绪" in status.detail


def test_offline_when_bridge_unreachable(server):
    client = adapter(server)
    server.stop()
    status = client.get_robot_status()
    assert status.online is False
    assert "不可达" in status.detail


def test_status_cache_reduces_requests(server, gateway):
    client = adapter(server)
    client.get_robot_status()
    client.get_robot_status()
    assert gateway.status_calls == 1


def test_control_success_reports_dispatched(server, gateway):
    result = adapter(server).send_control("start", "", "op-1")
    assert result.success is True
    assert result.message == "已接受"
    assert result.current_state == "running:inspect:POI_01"
    assert gateway.control_calls == 1


def test_control_service_unavailable_raises(server, gateway):
    gateway.reply = GatewayUnavailable("未发现服务")
    with pytest.raises(RosUnavailable, match="未发现服务"):
        adapter(server).send_control("stop", "", "op-1")


def test_control_timeout_raises(server, gateway):
    gateway.reply = GatewayTimeout("未收到响应")
    with pytest.raises(RosUnavailable, match="未收到响应"):
        adapter(server).send_control("pause", "", "op-1")


def test_control_network_failure_is_explicit(server):
    client = adapter(server)
    server.stop()
    with pytest.raises(RosUnavailable, match="无法确认"):
        client.send_control("start", "", "op-1")


def test_build_adapter_defaults_to_disabled():
    settings = Settings(db_path=":memory:", ros_adapter="disabled", host="127.0.0.1", port=8000)
    assert isinstance(build_ros_adapter(settings), DisabledRosAdapter)


def test_build_bridge_requires_token():
    settings = Settings(db_path=":memory:", ros_adapter="bridge", host="127.0.0.1", port=8000)
    with pytest.raises(ValueError, match="FIRE_DASHBOARD_BRIDGE_TOKEN"):
        build_ros_adapter(settings)


def test_build_bridge_with_token():
    settings = Settings(
        db_path=":memory:",
        ros_adapter="bridge",
        host="127.0.0.1",
        port=8000,
        bridge_token="abc",
    )
    assert isinstance(build_ros_adapter(settings), BridgeRosAdapter)


def test_build_unknown_adapter_rejected():
    settings = Settings(db_path=":memory:", ros_adapter="jazzy-direct", host="127.0.0.1", port=8000)
    with pytest.raises(ValueError):
        build_ros_adapter(settings)
