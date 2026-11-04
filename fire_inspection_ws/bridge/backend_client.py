"""Bridge → 网页后端的数据投递（标准库 urllib，无额外依赖）。

失败语义：
- ``PermanentIngestError``（4xx）：后端已明确拒绝该事件，重试也不会成功，
  dispatcher 丢弃并记录日志，避免队列被毒事件堵死；
- ``TransientIngestError``（5xx / 网络错误 / 超时）：后端暂时不可用，
  事件留在 outbox，稍后重试。

事件按 ``kind`` 映射到 ``/api/ingest/*`` 接口；所有请求带 ``X-Bridge-Token``。
"""

from __future__ import annotations

import json
import logging
import threading
import time
import urllib.error
import urllib.request
from typing import Callable

from .events import (
    MANUAL_FINISH,
    MANUAL_RESULT,
    MANUAL_START,
    MANUAL_STATE,
    ALERT_EVENT,
    POI_RESULT,
    ROUND_FINISH,
    ROUND_START,
    ROUND_STATE,
)

logger = logging.getLogger(__name__)


class IngestError(RuntimeError):
    """投递失败的基类。"""


class PermanentIngestError(IngestError):
    """后端明确拒绝（4xx）：不应重试。"""


class TransientIngestError(IngestError):
    """后端暂时不可用（5xx / 网络）：应当重试。"""


class IngestClient:
    def __init__(
        self,
        base_url: str,
        token: str,
        *,
        timeout_s: float = 5.0,
    ) -> None:
        if not token:
            raise ValueError("IngestClient 必须配置非空令牌")
        self._base_url = base_url.rstrip("/")
        self._token = token
        self._timeout = timeout_s

    def _post(self, path: str, payload: dict) -> dict:
        data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        req = urllib.request.Request(
            f"{self._base_url}{path}",
            data=data,
            headers={
                "Content-Type": "application/json",
                "X-Bridge-Token": self._token,
                "Accept": "application/json",
            },
            method="POST",
        )
        try:
            with urllib.request.urlopen(req, timeout=self._timeout) as resp:
                body = resp.read()
                parsed = json.loads(body.decode("utf-8")) if body else {}
                return parsed if isinstance(parsed, dict) else {}
        except urllib.error.HTTPError as exc:
            detail = _read_detail(exc)
            if 400 <= exc.code < 500:
                raise PermanentIngestError(
                    f"后端拒绝事件（HTTP {exc.code}）：{detail}"
                ) from exc
            raise TransientIngestError(
                f"后端暂时不可用（HTTP {exc.code}）：{detail}"
            ) from exc
        except (urllib.error.URLError, TimeoutError, OSError, ValueError) as exc:
            raise TransientIngestError(f"后端不可达：{exc}") from exc

    def start_round(self, event: dict) -> dict:
        payload = {"round_id": event["round_id"], **event["payload"]}
        return self._post("/api/ingest/rounds", payload)

    def update_state(self, event: dict) -> dict:
        payload = dict(event["payload"])
        return self._post(f"/api/ingest/rounds/{event['round_id']}/state", payload)

    def add_result(self, event: dict) -> dict:
        payload = dict(event["payload"])
        # 结果事件的 event_id 在 payload 内（与事件 id 相同，用于后端幂等）
        payload.setdefault("event_id", event["event_id"])
        return self._post(f"/api/ingest/rounds/{event['round_id']}/results", payload)

    def finish_round(self, event: dict) -> dict:
        payload = dict(event["payload"])
        return self._post(
            f"/api/ingest/rounds/{event['round_id']}/finish", payload
        )

    # -- 手动点名检查（与自动巡检完全独立的接入接口） -------------------------
    def start_manual(self, event: dict) -> dict:
        payload = {"manual_id": event["manual_id"], **event["payload"]}
        return self._post("/api/ingest/manual-inspections", payload)

    def update_manual_state(self, event: dict) -> dict:
        payload = dict(event["payload"])
        return self._post(
            f"/api/ingest/manual-inspections/{event['manual_id']}/state", payload
        )

    def add_manual_result(self, event: dict) -> dict:
        payload = dict(event["payload"])
        # 结果事件的 event_id 在 payload 内（确定性 ID，用于后端幂等）
        payload.setdefault("event_id", event["event_id"])
        return self._post(
            f"/api/ingest/manual-inspections/{event['manual_id']}/result", payload
        )

    def finish_manual(self, event: dict) -> dict:
        payload = dict(event["payload"])
        return self._post(
            f"/api/ingest/manual-inspections/{event['manual_id']}/finish", payload
        )

    def add_alert_event(self, event: dict) -> dict:
        payload = dict(event["payload"])
        payload["event_id"] = event["event_id"]
        return self._post("/api/ingest/events", payload)

    def dispatch(self, event: dict) -> dict:
        kind = event.get("kind")
        if kind == ALERT_EVENT:
            return self.add_alert_event(event)
        if kind == ROUND_START:
            return self.start_round(event)
        if kind == ROUND_STATE:
            return self.update_state(event)
        if kind == POI_RESULT:
            return self.add_result(event)
        if kind == ROUND_FINISH:
            return self.finish_round(event)
        if kind == MANUAL_START:
            return self.start_manual(event)
        if kind == MANUAL_STATE:
            return self.update_manual_state(event)
        if kind == MANUAL_RESULT:
            return self.add_manual_result(event)
        if kind == MANUAL_FINISH:
            return self.finish_manual(event)
        raise PermanentIngestError(f"未知事件类型：{kind}")


class OutboxDispatcher:
    """后台线程：把 outbox 里的事件按顺序投递到后端。

    - 成功或 4xx → ack 移除；
    - 5xx / 网络错误 → 保留，等待下一次重试；
    - POST_FAILURE_BACKOFF_S 防止后端长时间不可用时疯狂重试。
    """

    def __init__(
        self,
        outbox,
        client: IngestClient,
        *,
        retry_interval_s: float = 2.0,
        failure_backoff_s: float = 10.0,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._outbox = outbox
        self._client = client
        self._retry_interval = retry_interval_s
        self._failure_backoff = failure_backoff_s
        self._clock = clock
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._wake = threading.Event()
        self._lock = threading.Lock()
        self.last_error: str | None = None
        self.delivered = 0
        self.dropped = 0

    # -- 生命周期 -----------------------------------------------------------
    def start(self) -> None:
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run, name="bridge-outbox", daemon=True
        )
        self._thread.start()

    def stop(self, timeout_s: float = 3.0) -> None:
        self._stop.set()
        self._wake.set()
        if self._thread is not None:
            self._thread.join(timeout=timeout_s)
            self._thread = None

    def notify(self) -> None:
        """新事件入队后唤醒投递线程（避免等待一个完整重试周期）。"""
        self._wake.set()

    # -- 主循环 -------------------------------------------------------------
    def _run(self) -> None:
        while not self._stop.is_set():
            pending = self._outbox.pending()
            if not pending:
                self._wake.wait(1.0)
                self._wake.clear()
                continue
            failed = False
            for event in pending:
                if self._stop.is_set():
                    return
                if not self._deliver(event):
                    failed = True
                    break
            if failed:
                self._stop.wait(self._failure_backoff)
            else:
                # 成功清空后稍等，避免空转
                self._stop.wait(0.05)

    def _deliver(self, event: dict) -> bool:
        try:
            self._client.dispatch(event)
        except PermanentIngestError as exc:
            # 毒事件：丢弃，避免堵住后续事件
            with self._lock:
                self.dropped += 1
                self.last_error = str(exc)
            logger.error("丢弃事件 %s：%s", event.get("event_id"), exc)
            self._outbox.ack(event["event_id"])
            return True
        except TransientIngestError as exc:
            with self._lock:
                self.last_error = str(exc)
            logger.warning("事件 %s 投递失败，稍后重试：%s", event.get("event_id"), exc)
            return False
        with self._lock:
            self.delivered += 1
            self.last_error = None
        self._outbox.ack(event["event_id"])
        return True

    def stats(self) -> dict:
        with self._lock:
            return {
                "pending": len(self._outbox),
                "delivered": self.delivered,
                "dropped": self.dropped,
                "last_error": self.last_error,
            }


def _read_detail(exc: urllib.error.HTTPError) -> str:
    try:
        body = exc.read().decode("utf-8", errors="replace")
    except OSError:
        return ""
    try:
        parsed = json.loads(body)
    except ValueError:
        return body.strip()
    if isinstance(parsed, dict):
        return str(parsed.get("detail") or body.strip())
    return body.strip()
