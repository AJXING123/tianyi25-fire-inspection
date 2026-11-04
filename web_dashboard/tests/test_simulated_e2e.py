"""端到端模拟：真实 HTTP 后端 + 假 ROS 网关 + Bridge 全链路（无真实机器人）。

覆盖：控制确认、轮次建/状态/结果/结束、原始字段保留、演示数据隔离、
Bridge 停止后的离线语义。全程不导入 rclpy、不发送任何底盘指令。
"""

from __future__ import annotations

import json
import socket
import threading
import time
import urllib.error
import urllib.request

import pytest
import uvicorn

from app.auth import AuthManager
from app.main import create_app
from app.ros_adapter import BridgeRosAdapter, RosUnavailable
from bridge.backend_client import IngestClient, OutboxDispatcher
from bridge.gateway import GatewayReply
from bridge.http_app import BridgeHttpServer
from bridge.manual_tracker import ManualMarker, ManualTracker
from bridge.outbox import EventOutbox
from bridge.round_tracker import RoundMarker, RoundTracker
from conftest import BRIDGE_TOKEN, make_settings

ROUND_ID = "run-e2e0000000000001"
MANUAL_ID = "man-e2e0000000000001"


def free_port() -> int:
    sock = socket.socket()
    sock.bind(("127.0.0.1", 0))
    port = sock.getsockname()[1]
    sock.close()
    return port


def http_json(url: str, method: str = "GET", payload: dict | None = None) -> dict:
    data = json.dumps(payload).encode("utf-8") if payload is not None else None
    headers = {"Content-Type": "application/json"} if data else {}
    req = urllib.request.Request(url, data=data, headers=headers, method=method)
    with urllib.request.urlopen(req, timeout=3) as resp:
        return json.loads(resp.read().decode("utf-8"))


def wait_for(fn, timeout: float = 8.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = fn()
        if value:
            return value
        time.sleep(0.05)
    return None


def fetch_round(base: str, round_id: str) -> dict | None:
    """查询轮次；尚未建轮时返回 None，便于 wait_for 轮询。"""
    try:
        return http_json(f"{base}/api/rounds/{round_id}")
    except urllib.error.HTTPError as exc:
        if exc.code == 404:
            return None
        raise


@pytest.fixture
def backend(tmp_path):
    port = free_port()
    # e2e 关注数据链路；网页登录鉴权由 test_auth.py 专项覆盖
    app = create_app(
        settings=make_settings(tmp_path / "e2e.db"),
        auth=AuthManager.skip_auth(),
    )
    server = uvicorn.Server(
        uvicorn.Config(app, host="127.0.0.1", port=port, log_level="warning")
    )
    thread = threading.Thread(target=server.run, daemon=True)
    thread.start()
    base = f"http://127.0.0.1:{port}"

    def ready() -> bool:
        try:
            return http_json(f"{base}/api/health")["database"] == "ok"
        except (urllib.error.URLError, OSError, ValueError):
            return False

    assert wait_for(ready, 10.0) is not None, "后端未能启动"
    yield base
    server.should_exit = True
    thread.join(timeout=5.0)


class FakeRos:
    """模拟 rclpy 网关：只记录调用并返回固定响应，不接触真实 ROS。"""

    def __init__(self) -> None:
        self.online = True
        self.calls: list[tuple[str, str, str]] = []

    def status(self) -> dict:
        return {
            "ros_ready": True,
            "control_service_available": self.online,
            "state_text": "running:inspect:POI_01" if self.online else None,
            "detail": "",
        }

    def control(self, cmd, arg, operator_id, timeout_s):
        self.calls.append((cmd, arg, operator_id))
        return GatewayReply(True, f"模拟 ROS 已接受 {cmd}", "running:inspect:POI_01")


class Harness:
    def __init__(self, backend_url: str, tmp_path) -> None:
        self.outbox = EventOutbox(tmp_path / "outbox.json")
        self.tracker = RoundTracker(
            self.outbox,
            RoundMarker(tmp_path / "active_round.json"),
            clock=lambda: "2026-10-28T02:00:00+00:00",
            id_factory=lambda: ROUND_ID,
        )
        self.manual_tracker = ManualTracker(
            self.outbox,
            ManualMarker(tmp_path / "active_manual.json"),
            clock=lambda: "2026-10-28T02:00:00+00:00",
            id_factory=lambda: MANUAL_ID,
        )
        self.client = IngestClient(backend_url, BRIDGE_TOKEN, timeout_s=3.0)
        self.dispatcher = OutboxDispatcher(
            self.outbox, self.client, retry_interval_s=0.05, failure_backoff_s=0.2
        )
        self.ros = FakeRos()
        self.http = BridgeHttpServer(self.ros, BRIDGE_TOKEN, port=0, control_timeout_s=2.0)
        self.adapter: BridgeRosAdapter | None = None

    def emit(self, events: list[dict]) -> None:
        if not events:
            return
        self.outbox.append(events)
        self.dispatcher.notify()

    def state(self, text: str) -> None:
        """与 ros_node._on_state 一致：同一条状态流分别喂给两个跟踪器。"""
        events = self.tracker.observe_state(text)
        events += self.manual_tracker.observe_state(text)
        self.emit(events)

    def start(self) -> None:
        self.http.start()
        self.dispatcher.start()
        # 端口在 start() 后才确定，适配器必须此时创建
        self.adapter = BridgeRosAdapter(
            self.http.url,
            BRIDGE_TOKEN,
            status_timeout_s=2.0,
            control_timeout_s=3.0,
            status_cache_ttl_s=0.0,
        )

    def stop(self) -> None:
        self.http.stop()
        self.dispatcher.stop()


def test_full_simulated_round(backend, tmp_path):
    h = Harness(backend, tmp_path)
    h.start()
    try:
        assert h.adapter.get_robot_status().online is True

        # 1. 网页发起 start：Bridge 确认后才算 dispatched
        result = h.adapter.send_control("start", "", "op-e2e")
        assert result.success is True
        assert h.ros.calls == [("start", "", "op-e2e")]
        h.tracker.note_control("start", "op-e2e")

        # 2. ROS 进入巡检：建轮 / 状态变化
        h.emit(h.tracker.observe_state("running:inspect:POI_01"))
        h.emit(h.tracker.observe_state("paused:inspect:POI_01"))

        # 3. 正式点位结果（模拟 ROS 返回，保留原始字段）
        event = h.tracker.record_result(
            poi="POI_01",
            status_code=1,
            simulated=True,
            detail="模拟：检出异常",
            detections_json="[]",
            checked_at="2026-10-28T02:00:05+00:00",
        )
        assert event is not None
        h.emit([event])

        detail = wait_for(
            lambda: (
                data["results"] if (data := fetch_round(backend, ROUND_ID)) else None
            )
        )
        assert detail is not None, "点位结果未送达后端"

        # 4. 结束轮次
        h.emit(h.tracker.observe_state("done"))
        finished = wait_for(
            lambda: (
                data
                if (data := fetch_round(backend, ROUND_ID))
                and data["status"] == "completed"
                else None
            )
        )
        assert finished is not None

        round_data = http_json(f"{backend}/api/rounds/{ROUND_ID}")
        assert round_data["source"] == "real"
        assert round_data["is_demo"] is False
        assert round_data["status"] == "completed"
        assert round_data["outcome"] == "abnormal"
        assert h.ros.calls == [("start", "", "op-e2e")]

        poi = round_data["results"][0]
        assert poi["poi"] == "POI_01"
        assert poi["status_code"] == 1
        assert poi["status_name"] == "ABNORMAL"
        assert poi["simulated"] is True
        assert poi["detail"] == "模拟：检出异常"
        assert poi["detections"] == []
        assert poi["checked_at"] == "2026-10-28T02:00:05+00:00"

        # 5. 演示数据严格隔离
        demo = http_json(f"{backend}/api/rounds?source=demo")
        assert demo["total"] == 0
    finally:
        h.stop()


def fetch_manual(base: str, manual_id: str) -> dict | None:
    try:
        return http_json(f"{base}/api/manual-inspections/{manual_id}")
    except urllib.error.HTTPError as exc:
        if exc.code == 404:
            return None
        raise


def test_full_simulated_manual_inspection(backend, tmp_path):
    """手动点名检查全链路：建档 → 状态 → 正式结果 → 收尾，与自动巡检隔离。"""
    h = Harness(backend, tmp_path)
    h.start()
    try:
        # 1. 网页发起 manual 控制：Bridge 登记操作人（点位匹配才归属）
        h.manual_tracker.note_control("manual", "POI_03", "op-e2e")

        # 2. ROS 进入手动检查：建档 + 过程状态
        h.emit(h.manual_tracker.observe_state("running:manual:settle:POI_03"))
        h.emit(h.manual_tracker.observe_state("running:manual:inspect:POI_03"))

        created = wait_for(lambda: fetch_manual(backend, MANUAL_ID))
        assert created is not None, "手动检查建档未送达后端"
        assert created["status"] == "running"
        assert created["operator_id"] == "op-e2e"
        assert created["status_code"] is None  # 未完成检查没有视觉状态

        # 3. 正式点位结果（InspectPoi Action 返回值透传）
        event = h.manual_tracker.record_result(
            poi="POI_03",
            status_code=0,
            simulated=False,
            detail="未见异常",
            detections_json="[]",
            checked_at="2026-10-28T02:00:07+00:00",
        )
        assert event is not None
        h.emit([event])

        # 4. 手动检查结束（idle → completed）
        h.emit(h.manual_tracker.observe_state("idle"))
        finished = wait_for(
            lambda: (
                data
                if (data := fetch_manual(backend, MANUAL_ID))
                and data["status"] == "completed"
                else None
            )
        )
        assert finished is not None, "手动检查收尾未送达后端"
        assert finished["status_code"] == 0
        assert finished["status_name"] == "NORMAL"
        assert finished["simulated"] is False
        assert finished["detail"] == "未见异常"
        assert finished["checked_at"] == "2026-10-28T02:00:07+00:00"
        assert finished["finished_at"]

        # 5. 严格隔离：不产生自动巡检轮次，结果不进 poi_result
        rounds = http_json(f"{backend}/api/rounds")
        assert rounds["total"] == 0
        manual_list = http_json(f"{backend}/api/manual-inspections")
        assert manual_list["total"] == 1
        assert manual_list["items"][0]["manual_id"] == MANUAL_ID
    finally:
        h.stop()


def test_manual_full_flow_with_nav_and_intervention_creates_no_round(backend, tmp_path):
    """P0-2 回归：手动导航/介入全程不得创建 inspection_round，之后自动巡检正常。"""
    h = Harness(backend, tmp_path)
    h.start()
    try:
        h.manual_tracker.note_control("manual", "POI_01", "op-manual")
        for state in (
            "running:manual:POI_01",
            "running:manual:nav:POI_01",
            "running:manual:intervene:POI_01",
            "running:manual:settle:POI_01",
        ):
            h.state(state)
            assert http_json(f"{backend}/api/rounds")["total"] == 0

        result = h.manual_tracker.record_result(
            poi="POI_01",
            status_code=0,
            simulated=False,
            detail="未见异常",
            detections_json="[]",
            checked_at="2026-10-28T02:00:09+00:00",
        )
        assert result is not None

        for state in (
            "running:manual:inspect:POI_01",
            "running:manual:speak:POI_01",
        ):
            h.state(state)

        h.emit([result])
        h.state("idle")

        finished = wait_for(
            lambda: (
                data
                if (data := fetch_manual(backend, MANUAL_ID))
                and data["status"] == "completed"
                else None
            )
        )
        assert finished is not None
        assert finished["status_code"] == 0
        assert http_json(f"{backend}/api/rounds")["total"] == 0

        # 手动结束后自动巡检仍能正常建轮 / 结束
        h.tracker.note_control("start", "op-auto")
        h.state("running:wait")
        created = wait_for(
            lambda: (
                data
                if (data := fetch_round(backend, ROUND_ID)) and data["status"] == "running"
                else None
            )
        )
        assert created is not None, "手动结束后自动巡检未能建轮"

        auto_event = h.tracker.record_result(
            poi="POI_02",
            status_code=0,
            simulated=False,
            detail="正常",
            detections_json="[]",
            checked_at="2026-10-28T02:00:20+00:00",
        )
        h.emit([auto_event])
        h.state("done")

        done = wait_for(
            lambda: (
                data
                if (data := fetch_round(backend, ROUND_ID))
                and data["status"] == "completed"
                else None
            )
        )
        assert done is not None
        assert len(done["results"]) == 1
        assert done["results"][0]["poi"] == "POI_02"
        # 手动记录仍然独立，未被自动结果污染
        manual = http_json(f"{backend}/api/manual-inspections/{MANUAL_ID}")
        assert manual["poi"] == "POI_01"
    finally:
        h.stop()


def test_stop_button_and_offline_semantics(backend, tmp_path):
    h = Harness(backend, tmp_path)
    h.start()
    try:
        h.emit(h.tracker.observe_state("running:inspect:POI_01"))
        created = wait_for(
            lambda: (
                data
                if (data := fetch_round(backend, ROUND_ID)) and data["status"] == "running"
                else None
            )
        )
        assert created is not None

        # 网页 stop → Bridge 转发 → ROS 应进入 stopped；这里直接模拟状态回报
        h.tracker.note_control("stop", "op-e2e")
        h.emit(h.tracker.observe_state("stopped"))

        stopped = wait_for(
            lambda: (
                data
                if (data := fetch_round(backend, ROUND_ID)) and data["status"] == "stopped"
                else None
            )
        )
        assert stopped is not None

        # Bridge 停机后：状态必须离线，控制必须明确失败
        h.http.stop()
        status = h.adapter.get_robot_status()
        assert status.online is False
        with pytest.raises(RosUnavailable):
            h.adapter.send_control("start", "", "op-e2e")
    finally:
        h.stop()
