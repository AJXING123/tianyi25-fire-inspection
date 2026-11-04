"""ROS Bridge 与 rclpy 之间的抽象接口（可在无 ROS 环境用假网关测试）。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol


class GatewayError(RuntimeError):
    """网关调用失败的基类。"""


class GatewayUnavailable(GatewayError):
    """/fire_inspection/control 服务不可发现或节点未就绪。"""


class GatewayTimeout(GatewayError):
    """已发起服务调用，但未在超时内收到响应（结果未知）。"""


@dataclass(frozen=True)
class GatewayReply:
    success: bool
    message: str
    current_state: str | None


class RosGateway(Protocol):
    def status(self) -> dict:
        """返回 {ros_ready, control_service_available, state_text, detail}。"""

    def control(
        self, cmd: str, arg: str, operator_id: str, timeout_s: float
    ) -> GatewayReply:
        ...
