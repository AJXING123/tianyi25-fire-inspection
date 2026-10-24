"""JSONL 巡检记录器。

每行一条记录：session 头 / poi 记录 / summary 尾。
只追加写，不覆盖历史文件；线程安全（内部加锁）。
"""

from __future__ import annotations

import json
import threading
from pathlib import Path

from .types import now_iso


class InspectionRecorder:
    def __init__(self, path, meta: dict):
        self._path = Path(path)
        self._path.parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()
        self._poi_count = 0
        self.write_raw({"record_type": "session", "ts": now_iso(), **meta})

    @property
    def path(self) -> Path:
        return self._path

    def write_raw(self, obj: dict) -> None:
        line = json.dumps(obj, ensure_ascii=False)
        with self._lock:
            with self._path.open("a", encoding="utf-8") as fp:
                fp.write(line + "\n")

    def write_poi(self, record) -> None:
        """写入一条 PoiRecord（types.PoiRecord）。"""
        self._poi_count += 1
        self.write_raw(record.to_dict())

    def write_summary(self, final_state: str, detail: str = "") -> None:
        self.write_raw({
            "record_type": "summary",
            "ts": now_iso(),
            "final_state": final_state,
            "poi_records": self._poi_count,
            "detail": detail,
        })
