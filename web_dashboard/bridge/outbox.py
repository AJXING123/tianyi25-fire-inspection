"""持久化 outbox：Bridge 重启 / 后端暂时不可用时保住待投递事件。

实现为单个 JSON 文件 + 原子替换（os.replace）。事件量每轮只有几十条，
简单可靠优先；不引入额外的消息队列依赖。
"""

from __future__ import annotations

import json
import logging
import os
import threading
from pathlib import Path

logger = logging.getLogger(__name__)


class OutboxFull(RuntimeError):
    """待投递事件超过上限（防止无限堆积占满磁盘）。"""


class EventOutbox:
    def __init__(self, path: Path | str, *, max_events: int = 1000) -> None:
        self._path = Path(path)
        self._max_events = max_events
        self._lock = threading.RLock()
        self._path.parent.mkdir(parents=True, exist_ok=True)
        self._events = self._load()

    def _load(self) -> list[dict]:
        if not self._path.exists():
            return []
        try:
            data = json.loads(self._path.read_text(encoding="utf-8"))
        except (OSError, ValueError) as exc:
            corrupt = self._path.with_suffix(self._path.suffix + ".corrupt")
            try:
                os.replace(self._path, corrupt)
            except OSError:
                pass
            logger.error("outbox 文件损坏（%s），已移到 %s，按空队列继续", exc, corrupt)
            return []
        if not isinstance(data, list):
            logger.error("outbox 内容不是数组，按空队列继续")
            return []
        return [item for item in data if isinstance(item, dict) and "event_id" in item]

    def _persist(self) -> None:
        tmp = self._path.with_suffix(self._path.suffix + ".tmp")
        with open(tmp, "w", encoding="utf-8") as handle:
            json.dump(self._events, handle, ensure_ascii=False, indent=1)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(tmp, self._path)

    def append(self, events: list[dict]) -> None:
        if not events:
            return
        with self._lock:
            if len(self._events) + len(events) > self._max_events:
                raise OutboxFull(
                    f"outbox 待投递事件超过上限 {self._max_events}，"
                    "请检查后端 / 网络后重启 Bridge"
                )
            self._events.extend(events)
            self._persist()

    def pending(self) -> list[dict]:
        with self._lock:
            return list(self._events)

    def ack(self, event_id: str) -> None:
        with self._lock:
            remaining = [e for e in self._events if e.get("event_id") != event_id]
            if len(remaining) == len(self._events):
                return
            self._events = remaining
            self._persist()

    def __len__(self) -> int:
        return len(self._events)

    def clear(self) -> None:
        """仅用于测试 / 运维手动清理。"""
        with self._lock:
            self._events = []
            self._persist()
