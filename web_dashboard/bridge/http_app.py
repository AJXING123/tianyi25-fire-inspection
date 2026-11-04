"""Bridge 的本机 HTTP 接口（标准库实现，机器人端无需 FastAPI）。

路由：
- GET  /status   供后端判断在线（仅 ROS 就绪且控制服务可发现才算在线）
- GET  /health   进程存活
- POST /control  把控制请求转给 ROS 服务；只有收到服务响应才返回 dispatched=true

所有请求必须带 X-Bridge-Token；Bridge 默认只监听 127.0.0.1。
"""

from __future__ import annotations

import json
import logging
import secrets
import threading
import time
from collections import OrderedDict
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Callable

from .gateway import GatewayTimeout, GatewayUnavailable, RosGateway

logger = logging.getLogger(__name__)

CONTROL_COMMANDS = ("start", "pause", "resume", "skip", "stop", "manual")
MAX_BODY_BYTES = 16 * 1024


class BridgeHttpServer:
    def __init__(
        self,
        gateway: RosGateway,
        token: str,
        *,
        host: str = "127.0.0.1",
        port: int = 8081,
        control_timeout_s: float = 5.0,
        dedupe_ttl_s: float = 120.0,
    ) -> None:
        if not token:
            raise ValueError("Bridge 必须配置非空令牌后才能启动")
        self._gateway = gateway
        self._token = token
        self._host = host
        self._port = port
        self._control_timeout = control_timeout_s
        self._dedupe_ttl = dedupe_ttl_s
        self._dedupe: OrderedDict[str, tuple[float, int, dict]] = OrderedDict()
        self._lock = threading.Lock()
        self._httpd: ThreadingHTTPServer | None = None
        self._thread: threading.Thread | None = None

    # -- 生命周期 -----------------------------------------------------------
    def start(self) -> None:
        handler = _make_handler(self)
        self._httpd = ThreadingHTTPServer((self._host, self._port), handler)
        self._httpd.daemon_threads = True
        self._port = int(self._httpd.server_address[1])
        self._thread = threading.Thread(
            target=self._httpd.serve_forever, name="bridge-http", daemon=True
        )
        self._thread.start()
        logger.info("Bridge HTTP 已监听 http://%s:%s", self._host, self._port)

    def stop(self) -> None:
        if self._httpd is not None:
            self._httpd.shutdown()
            self._httpd.server_close()
            self._httpd = None
        if self._thread is not None:
            self._thread.join(timeout=3.0)
            self._thread = None

    @property
    def port(self) -> int:
        return self._port

    @property
    def url(self) -> str:
        return f"http://{self._host}:{self._port}"

    # -- 逻辑 ---------------------------------------------------------------
    def handle_status(self) -> tuple[int, dict]:
        try:
            info = self._gateway.status()
        except Exception as exc:  # noqa: BLE001 - 网关异常必须反映为离线
            logger.warning("读取 ROS 状态失败：%s", exc)
            return 200, {
                "online": False,
                "ros_ready": False,
                "control_service_available": False,
                "state_text": None,
                "detail": f"ROS 状态不可用：{exc}",
                "bridge": "ros2-jazzy",
            }
        ros_ready = bool(info.get("ros_ready"))
        service_ready = bool(info.get("control_service_available"))
        online = ros_ready and service_ready
        detail = info.get("detail")
        if not detail:
            detail = (
                "ROS 已就绪，控制服务可发现"
                if online
                else "ROS 已就绪，但 /fire_inspection/control 尚不可发现"
            )
        return 200, {
            "online": online,
            "ros_ready": ros_ready,
            "control_service_available": service_ready,
            "state_text": info.get("state_text"),
            "detail": detail,
            "bridge": "ros2-jazzy",
        }

    def handle_control(self, body: dict) -> tuple[int, dict]:
        cmd = str(body.get("cmd") or "").strip().lower()
        arg = str(body.get("arg") or "").strip()
        operator = str(body.get("operator_id") or "").strip()
        request_id = str(body.get("request_id") or "").strip()

        if cmd not in CONTROL_COMMANDS:
            return 400, {
                "success": False,
                "message": f"未知指令「{cmd or '(空)'}」",
                "current_state": None,
                "dispatched": False,
            }
        if cmd == "manual" and not arg:
            return 400, {
                "success": False,
                "message": "manual 指令必须指定点位",
                "current_state": None,
                "dispatched": False,
            }

        if request_id:
            cached = self._cache_get(request_id)
            if cached is not None:
                code, payload = cached
                return code, {**payload, "deduplicated": True}

        try:
            reply = self._gateway.control(
                cmd, arg, operator, self._control_timeout
            )
        except GatewayUnavailable as exc:
            result = (503, {
                "success": False,
                "message": str(exc),
                "current_state": None,
                "dispatched": False,
            })
        except GatewayTimeout as exc:
            result = (504, {
                "success": False,
                "message": str(exc),
                "current_state": None,
                "dispatched": False,
            })
        except Exception as exc:  # noqa: BLE001 - Bridge 内部错误不得报成功
            logger.exception("控制调用异常")
            result = (500, {
                "success": False,
                "message": f"ROS Bridge 内部错误：{exc}",
                "current_state": None,
                "dispatched": False,
            })
        else:
            result = (200, {
                "success": bool(reply.success),
                "message": reply.message,
                "current_state": reply.current_state,
                "dispatched": True,
            })

        if request_id:
            self._cache_put(request_id, result)
        return result

    def _cache_get(self, request_id: str) -> tuple[int, dict] | None:
        now = time.monotonic()
        with self._lock:
            item = self._dedupe.get(request_id)
            if item is None:
                return None
            expires_at, code, payload = item
            if expires_at < now:
                self._dedupe.pop(request_id, None)
                return None
            return code, payload

    def _cache_put(self, request_id: str, result: tuple[int, dict]) -> None:
        now = time.monotonic()
        with self._lock:
            self._dedupe[request_id] = (now + self._dedupe_ttl, result[0], result[1])
            while len(self._dedupe) > 200:
                self._dedupe.popitem(last=False)


def _make_handler(server: BridgeHttpServer) -> type[BaseHTTPRequestHandler]:
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, fmt: str, *args) -> None:  # noqa: A003
            logger.debug("HTTP %s - " + fmt, self.address_string(), *args)

        def _send(self, code: int, payload: dict) -> None:
            body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _authorized(self) -> bool:
            token = self.headers.get("X-Bridge-Token", "")
            if secrets.compare_digest(token, server._token):
                return True
            self._send(401, {"detail": "Bridge 令牌无效"})
            return False

        def do_GET(self) -> None:  # noqa: N802
            if not self._authorized():
                return
            if self.path == "/status":
                self._send(*server.handle_status())
            elif self.path == "/health":
                self._send(200, {"status": "ok", "bridge": "ros2-jazzy"})
            else:
                self._send(404, {"detail": "Not Found"})

        def do_POST(self) -> None:  # noqa: N802
            if not self._authorized():
                return
            if self.path != "/control":
                self._send(404, {"detail": "Not Found"})
                return
            try:
                length = int(self.headers.get("Content-Length", "0"))
            except ValueError:
                length = 0
            if length <= 0 or length > MAX_BODY_BYTES:
                self._send(413, {"detail": "请求体为空或过大"})
                return
            raw = self.rfile.read(length)
            try:
                body = json.loads(raw.decode("utf-8"))
                if not isinstance(body, dict):
                    raise ValueError
            except ValueError:
                self._send(400, {"detail": "请求体必须是 JSON 对象"})
                return
            self._send(*server.handle_control(body))

    return Handler


def start_in_thread(server: BridgeHttpServer) -> Callable[[], None]:
    server.start()
    return server.stop
