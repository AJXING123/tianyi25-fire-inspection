"""机器人端 ROS Bridge 入口（唯一 import rclpy 的文件）。

只在机器人 Ubuntu 24.04 + ROS 2 Jazzy 环境运行。开发机 Humble 环境不会、
也不应该导入本文件；后端与 Bridge 之间只有本机 HTTP，没有 ROS 依赖。
"""

from __future__ import annotations

import argparse
import logging
import os
import signal
import threading
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from fire_inspection_interfaces.msg import PoiResult
from fire_inspection_interfaces.srv import InspectionControl
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from .backend_client import IngestClient, OutboxDispatcher
from .gateway import (
    GatewayError,
    GatewayReply,
    GatewayTimeout,
    GatewayUnavailable,
    RosGateway,
)
from .http_app import BridgeHttpServer
from .manual_tracker import ManualMarker, ManualTracker
from .outbox import EventOutbox, OutboxFull
from .round_tracker import RoundMarker, RoundTracker

logger = logging.getLogger("bridge")

STATE_TOPIC = "/fire_inspection/state"
CONTROL_SERVICE = "/fire_inspection/control"
STACK_CONTROL_SERVICE = "/fire_inspection/stack_control"
# 正式点位结果话题：编排节点在 _inspect() 拿到 InspectPoi 结果后原样透传发布
POI_RESULT_TOPIC = "/fire_inspection/poi_result"


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def _stamp_to_iso(stamp) -> str | None:
    if stamp is None or (not stamp.sec and not stamp.nanosec):
        return None
    seconds = float(stamp.sec) + float(stamp.nanosec) / 1e9
    return datetime.fromtimestamp(seconds, timezone.utc).isoformat(timespec="seconds")


class RosBridgeNode(Node, RosGateway):
    """订阅状态与正式点位结果，并把控制请求转发给 /fire_inspection/control。"""

    def __init__(
        self,
        *,
        state_dir: Path,
        poi_topic: str = POI_RESULT_TOPIC,
        control_timeout_s: float = 5.0,
    ) -> None:
        super().__init__("fire_dashboard_bridge")
        state_dir.mkdir(parents=True, exist_ok=True)
        self._control_timeout = control_timeout_s
        self._state_text: str | None = None
        self._service_ready = False

        self._outbox = EventOutbox(state_dir / "outbox.json")
        self._tracker = RoundTracker(
            self._outbox,
            RoundMarker(state_dir / "active_round.json"),
            clock=_utc_now,
        )
        # 手动点名检查独立跟踪：与 RoundTracker 互不读写对方的会话
        self._manual_tracker = ManualTracker(
            self._outbox,
            ManualMarker(state_dir / "active_manual.json"),
            clock=_utc_now,
        )
        self._dispatcher = OutboxDispatcher(
            self._outbox,
            IngestClient(
                os.environ.get("FIRE_DASHBOARD_BACKEND_URL", "http://127.0.0.1:8000"),
                os.environ.get("FIRE_DASHBOARD_BRIDGE_TOKEN", ""),
            ),
        )

        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._state_sub = self.create_subscription(
            String, STATE_TOPIC, self._on_state, state_qos
        )
        self._poi_sub = self.create_subscription(
            PoiResult, poi_topic, self._on_poi_result, 10
        )
        self._control_client = self.create_client(InspectionControl, CONTROL_SERVICE)
        self._stack_client = self.create_client(
            InspectionControl, STACK_CONTROL_SERVICE
        )
        self._stack_service_ready = False
        self.create_timer(1.0, self._refresh_service_ready)
        logger.info(
            "ROS Bridge 已启动：state=%s poi=%s control=%s",
            STATE_TOPIC,
            poi_topic,
            CONTROL_SERVICE,
        )

    # -- 生命周期 -----------------------------------------------------------
    def start_delivery(self) -> None:
        self._dispatcher.start()

    def shutdown(self) -> None:
        self._dispatcher.stop()

    # -- 话题回调 -----------------------------------------------------------
    def _on_state(self, msg: String) -> None:
        self._state_text = msg.data
        # 同一条状态流分别喂给两个互相独立的跟踪器：
        # RoundTracker 只认自动巡检，ManualTracker 只认 manual:* 系列
        events = self._tracker.observe_state(msg.data)
        events += self._manual_tracker.observe_state(msg.data)
        self._publish_events(events)

    def _on_poi_result(self, msg: PoiResult) -> None:
        fields = {
            "poi": str(msg.poi),
            "status_code": int(msg.status),
            "simulated": bool(msg.simulated),
            "detail": str(msg.detail),
            "detections_json": str(msg.detections_json),
            "checked_at": _stamp_to_iso(getattr(msg, "header", None) and msg.header.stamp),
        }
        # 先手动后轮次：编排层 _busy 互斥保证两类任务不会同时进行；
        # 且 ManualTracker 只在手动会话有效（未被自动阶段判定失效）时接收，
        # 因此结果归属不只看点位名称，还要求对应任务会话仍然有效。
        # 残余歧义：PoiResult 无任务 ID，跨任务同点位迟到结果仍可能误收，
        # 根治需接口会签后给结果消息加任务标识。
        event = self._manual_tracker.record_result(**fields)
        if event is None:
            event = self._tracker.record_result(**fields)
        if event is None:
            logger.warning(
                "收到点位结果 %s，但当前没有活动轮次或手动检查，已忽略", msg.poi
            )
            return
        self._publish_events([event])

    def _publish_events(self, events: list[dict]) -> None:
        if not events:
            return
        try:
            self._outbox.append(events)
        except OutboxFull as exc:
            logger.error("outbox 已满，事件未能入队：%s", exc)
            return
        self._dispatcher.notify()

    # -- RosGateway ---------------------------------------------------------
    def _refresh_service_ready(self) -> None:
        self._service_ready = self._control_client.service_is_ready()
        self._stack_service_ready = self._stack_client.service_is_ready()

    def status(self) -> dict:
        ros_ready = rclpy.ok() and self.context.ok()
        available = ros_ready and (
            self._stack_service_ready or self._service_ready
        )
        return {
            "ros_ready": ros_ready,
            "control_service_available": available,
            "stack_service_available": ros_ready and self._stack_service_ready,
            "state_text": self._state_text,
            "detail": (
                "ROS 已就绪，统一 Launch 管理服务可用"
                if ros_ready and self._stack_service_ready
                else "ROS 已就绪，当前仅控制服务可用"
                if ros_ready and self._service_ready
                else "ROS 已就绪，但尚未发现 Launch 管理服务"
                if ros_ready
                else "rclpy 未就绪"
            ),
        }

    def control(
        self, cmd: str, arg: str, operator_id: str, timeout_s: float
    ) -> GatewayReply:
        if not rclpy.ok() or not self.context.ok():
            raise GatewayUnavailable("ROS 上下文不可用，未发送控制请求")
        # start/stop 管理整套 Launch；其他命令仍由巡检编排层处理。
        is_stack_command = cmd in ("start", "stop")
        client = self._stack_client if is_stack_command else self._control_client
        service_name = STACK_CONTROL_SERVICE if is_stack_command else CONTROL_SERVICE

        if not client.service_is_ready():
            if not client.wait_for_service(timeout_sec=1.0):
                raise GatewayUnavailable(
                    f"未发现 {service_name} 服务，未发送控制请求"
                )

        # 在发起调用前登记操作人：ROS 状态话题可能早于服务响应到达，
        # 建轮时需要能取到归属；note_control 只接受 start，manual 的操作人
        # 由独立手动检查机制管理，不会写入自动巡检轮次。
        self._tracker.note_control(cmd, operator_id)
        self._manual_tracker.note_control(cmd, arg, operator_id)

        request = InspectionControl.Request()
        request.cmd = cmd
        request.arg = arg
        request.operator_id = operator_id
        future = client.call_async(request)

        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout_s):
            # 超时结果未知：请求可能已生效、状态可能随后到达，登记保留，
            # 交给 note_control 的 TTL 兜底过期，避免误清后丢失归属。
            raise GatewayTimeout(
                f"已向 /fire_inspection/control 发起「{cmd}」，"
                f"{timeout_s:.1f}s 内未收到响应，结果未知"
            )
        exc = future.exception()
        if exc is not None:
            # 调用明确失败，清理待登记操作人，避免串到后续任务
            self._tracker.discard_control(operator_id)
            self._manual_tracker.discard_control(operator_id)
            raise GatewayError(f"ROS 控制服务调用失败：{exc}") from exc
        result = future.result()
        reply = GatewayReply(
            success=bool(result.success),
            message=str(result.message),
            current_state=str(result.current_state) or None,
        )
        if not reply.success:
            # 编排层明确拒绝（如 busy、点位非法），指令不会执行，清理登记
            self._tracker.discard_control(operator_id)
            self._manual_tracker.discard_control(operator_id)
        return reply


def _parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="消防巡检网页后台 ROS Bridge（Jazzy）")
    parser.add_argument("--host", default="127.0.0.1", help="HTTP 监听地址，默认回环")
    parser.add_argument("--port", type=int, default=8081, help="HTTP 端口，默认 8081")
    parser.add_argument(
        "--state-dir",
        default=os.environ.get("FIRE_DASHBOARD_BRIDGE_STATE_DIR", "data/bridge"),
        help="outbox 与当前轮次标记目录",
    )
    parser.add_argument(
        "--backend-url",
        default=os.environ.get(
            "FIRE_DASHBOARD_BACKEND_URL", "http://127.0.0.1:8000"
        ),
        help="网页后端地址（数据接入）",
    )
    parser.add_argument(
        "--poi-topic",
        default=os.environ.get("FIRE_DASHBOARD_POI_TOPIC", POI_RESULT_TOPIC),
        help="正式点位结果话题（需编排节点发布，待确认）",
    )
    parser.add_argument(
        "--control-timeout",
        type=float,
        default=float(os.environ.get("FIRE_DASHBOARD_CONTROL_TIMEOUT_S", "5.0")),
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s"
    )
    args = _parse_args(argv)

    token = os.environ.get("FIRE_DASHBOARD_BRIDGE_TOKEN", "").strip()
    if not token:
        logger.error("必须配置 FIRE_DASHBOARD_BRIDGE_TOKEN 才能启动 Bridge")
        return 2
    backend_url = args.backend_url.rstrip("/")
    os.environ["FIRE_DASHBOARD_BACKEND_URL"] = backend_url

    rclpy.init(args=None)
    node = RosBridgeNode(
        state_dir=Path(args.state_dir).expanduser(),
        poi_topic=args.poi_topic,
        control_timeout_s=args.control_timeout,
    )
    server = BridgeHttpServer(
        node,
        token,
        host=args.host,
        port=args.port,
        control_timeout_s=args.control_timeout,
    )

    def _handle_signal(_signum, _frame) -> None:
        logger.info("收到退出信号，正在关闭 Bridge")
        rclpy.try_shutdown()

    signal.signal(signal.SIGTERM, _handle_signal)
    signal.signal(signal.SIGINT, _handle_signal)

    server.start()
    node.start_delivery()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    logger.info("Bridge HTTP：%s → 后端 %s", server.url, backend_url)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        server.stop()
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
