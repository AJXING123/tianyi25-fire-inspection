"""ROS 适配层：网页后端与机器人之间唯一的交互边界。

实现：
- ``DisabledRosAdapter``：默认，明确离线、拒绝控制（第一阶段行为保持不变）。
- ``BridgeRosAdapter``：通过本机 HTTP 调用独立 ROS Bridge 进程。只有 Bridge
  确认收到 ROS 服务响应时才报告 dispatched=true；Bridge 不可达 / 超时时
  抛出 ``RosUnavailable``，绝不伪造成功。

本文件不得 import rclpy：网页后端跑在开发机（ROS 2 Humble）或机器人上，
都不直接加载 ROS 包。ROS 相关依赖只存在于 ``bridge/`` 目录。
"""

from __future__ import annotations

import json
import threading
import time
import urllib.error
import urllib.request
import uuid
from dataclasses import dataclass
from typing import Protocol

from .config import Settings
from .db import utc_now

CONTROL_COMMANDS = ("start", "pause", "resume", "skip", "stop", "manual")


class RosUnavailable(RuntimeError):
    """ROS 适配层不可用：调用方必须明确失败，不得当作成功。"""


@dataclass(frozen=True)
class RobotStatus:
    online: bool
    adapter: str
    state_text: str | None
    detail: str
    checked_at: str


@dataclass(frozen=True)
class ControlResult:
    success: bool
    message: str
    current_state: str | None


class RosAdapter(Protocol):
    name: str

    def get_robot_status(self) -> RobotStatus:
        ...

    def send_control(self, cmd: str, arg: str, operator_id: str) -> ControlResult:
        ...


class DisabledRosAdapter:
    """离线实现：明确报告离线，拒绝一切控制请求。"""

    name = "disabled"

    def get_robot_status(self) -> RobotStatus:
        return RobotStatus(
            online=False,
            adapter=self.name,
            state_text=None,
            detail="ROS 适配层未启用：当前仅展示本地数据库中的数据，未连接机器人。",
            checked_at=utc_now(),
        )

    def send_control(self, cmd: str, arg: str, operator_id: str) -> ControlResult:
        raise RosUnavailable(
            f"ROS 适配层未启用，未向机器人发送「{cmd}」指令；"
            "请配置 FIRE_DASHBOARD_ROS_ADAPTER=bridge 并启动 ROS Bridge。"
        )


class BridgeRosAdapter:
    """通过本机 HTTP 与 ROS Bridge 通信。

    online 判定完全以 Bridge 的 /status 为准（Bridge 只有在 rclpy 就绪且
    /fire_inspection/control 服务可发现时才返回 online=true）。
    """

    name = "bridge"

    def __init__(
        self,
        base_url: str,
        token: str,
        *,
        status_timeout_s: float = 1.5,
        control_timeout_s: float = 8.0,
        status_cache_ttl_s: float = 2.0,
    ) -> None:
        self._base_url = base_url.rstrip("/")
        self._token = token
        self._status_timeout = status_timeout_s
        self._control_timeout = control_timeout_s
        self._cache_ttl = status_cache_ttl_s
        self._lock = threading.Lock()
        self._cached: RobotStatus | None = None
        self._cached_at = 0.0

    # -- HTTP ---------------------------------------------------------------
    def _request(
        self, method: str, path: str, payload: dict | None, timeout: float
    ) -> tuple[int, dict]:
        data = None
        headers = {"X-Bridge-Token": self._token, "Accept": "application/json"}
        if payload is not None:
            data = json.dumps(payload).encode("utf-8")
            headers["Content-Type"] = "application/json"
        req = urllib.request.Request(
            f"{self._base_url}{path}", data=data, headers=headers, method=method
        )
        try:
            with urllib.request.urlopen(req, timeout=timeout) as resp:
                body = resp.read()
                parsed = json.loads(body.decode("utf-8")) if body else {}
                return int(resp.status), parsed if isinstance(parsed, dict) else {}
        except urllib.error.HTTPError as exc:
            body = exc.read()
            try:
                parsed = json.loads(body.decode("utf-8")) if body else {}
            except ValueError:
                parsed = {}
            return int(exc.code), parsed if isinstance(parsed, dict) else {}
        except (urllib.error.URLError, TimeoutError, OSError, ValueError) as exc:
            raise RosUnavailable(
                f"ROS Bridge 不可达（{self._base_url}）：{exc}"
            ) from exc

    # -- RosAdapter ---------------------------------------------------------
    def get_robot_status(self) -> RobotStatus:
        now = time.monotonic()
        with self._lock:
            if self._cached is not None and now - self._cached_at < self._cache_ttl:
                return self._cached

        try:
            code, body = self._request("GET", "/status", None, self._status_timeout)
        except RosUnavailable as exc:
            status = RobotStatus(
                online=False,
                adapter=self.name,
                state_text=None,
                detail=str(exc),
                checked_at=utc_now(),
            )
        else:
            online = code == 200 and bool(body.get("online"))
            if online:
                detail = str(body.get("detail") or "ROS Bridge 已连接，控制服务可用")
            elif code == 200:
                detail = str(
                    body.get("detail")
                    or "ROS Bridge 已连接，但 ROS 控制服务尚未就绪"
                )
            else:
                detail = f"ROS Bridge 返回 HTTP {code}"
            status = RobotStatus(
                online=online,
                adapter=self.name,
                state_text=body.get("state_text") if online else None,
                detail=detail,
                checked_at=utc_now(),
            )

        with self._lock:
            self._cached = status
            self._cached_at = now
        return status

    def send_control(self, cmd: str, arg: str, operator_id: str) -> ControlResult:
        request_id = uuid.uuid4().hex
        try:
            code, body = self._request(
                "POST",
                "/control",
                {
                    "cmd": cmd,
                    "arg": arg,
                    "operator_id": operator_id,
                    "request_id": request_id,
                },
                self._control_timeout,
            )
        except RosUnavailable as exc:
            raise RosUnavailable(
                f"未收到 ROS Bridge 的控制确认，无法确认「{cmd}」是否执行；"
                f"请检查机器人状态后再操作。（{exc}）"
            ) from exc

        message = str(body.get("message") or f"ROS Bridge 返回 HTTP {code}")
        if code != 200:
            raise RosUnavailable(message)
        if not body.get("dispatched"):
            raise RosUnavailable(f"ROS Bridge 未确认指令已下发：{message}")
        return ControlResult(
            success=bool(body.get("success")),
            message=message,
            current_state=body.get("current_state"),
        )


def build_ros_adapter(settings: Settings) -> RosAdapter:
    """按配置构造适配器；未知名称直接报错，避免静默降级成错误行为。"""
    normalized = (settings.ros_adapter or "disabled").strip().lower()
    if normalized == "disabled":
        return DisabledRosAdapter()
    if normalized == "bridge":
        if not settings.bridge_token:
            raise ValueError(
                "启用 bridge 适配器必须配置 FIRE_DASHBOARD_BRIDGE_TOKEN，"
                "用于后端与 ROS Bridge 之间的双向鉴权。"
            )
        return BridgeRosAdapter(
            settings.bridge_url,
            settings.bridge_token,
            control_timeout_s=settings.bridge_timeout_s,
        )
    raise ValueError(
        f"不支持的 ROS 适配器「{settings.ros_adapter}」。可用值：disabled、bridge。"
    )
