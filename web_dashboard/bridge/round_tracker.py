"""巡检轮次跟踪：把 /fire_inspection/state 文本映射为可持久化的轮次事件。

设计目标：
- 每轮唯一 round_id，由 Bridge 生成并写入 marker 文件；
- Bridge 重启后读取 marker，用同一 round_id 重发幂等的建轮事件，避免串轮；
- 状态话题只用于判定轮次生命周期，**绝不**据此推断某点位检测结论；
- 正式点位结果只来自 InspectPoi Action 的返回值（经 Bridge 订阅的
  /fire_inspection/poi_result 话题转交），无结果就保持为空；
- 手动点名检查（running:manual:<poi> 起手）不属于巡检轮次：闩住手动会话，
  期间编排层还会发 settle/inspect/speak 等与自动巡检同形的状态，一律忽略，
  不建轮、不改轮、不发结束事件；手动操作人由独立手动检查机制管理。
"""

from __future__ import annotations

import json
import os
import re
import threading
import time
import uuid
from dataclasses import asdict, dataclass
from pathlib import Path

from .events import POI_RESULT, ROUND_FINISH, ROUND_START, ROUND_STATE, new_event

# 状态文本 -> 轮次终态；与 orchestrator_node.RunState 一致
TERMINAL_STATUS = {"done": "completed", "stopped": "stopped", "aborted": "aborted"}

# WATCH 模式不是巡检轮次（nav_enabled=false 时长期存在），不建轮
IGNORED_PHASES = {"watch"}

# 只会出现在自动巡检流程中的阶段（manual 流程不会经过），
# 用于识别手动会话已结束（例如 Bridge 错过了手动收尾的 idle/stopped）；
# speak:start / speak:done 同理（manual 只会发 speak:<poi>）。
AUTO_ONLY_PHASES = {"wait", "nav", "intervene", "return", "watch"}
AUTO_ONLY_SPEAK = {"speak:start", "speak:done"}

ROUND_ID_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]{7,63}$")


def _is_auto_only_phase(phase: str) -> bool:
    head = phase.split(":", 1)[0]
    if head == "speak":
        return phase in AUTO_ONLY_SPEAK
    return head in AUTO_ONLY_PHASES


@dataclass
class ActiveRound:
    round_id: str
    mode: str
    started_at: str
    status: str
    ros_state: str
    operator_id: str | None = None
    kind: str = "round"


@dataclass
class ManualSession:
    """手动点名检查会话标记：只用于忽略期间的状态，不代表任何巡检轮次。"""

    started_at: str
    ros_state: str
    kind: str = "manual"


class RoundMarker:
    """当前活动轮次 / 手动会话的持久化标记（原子写，同一文件互斥使用）。"""

    def __init__(self, path: Path | str) -> None:
        self._path = Path(path)

    @property
    def path(self) -> Path:
        return self._path

    def load(self) -> ActiveRound | ManualSession | None:
        if not self._path.exists():
            return None
        try:
            data = json.loads(self._path.read_text(encoding="utf-8"))
            # 旧版 marker 没有 kind 字段，只可能是轮次记录
            if str(data.get("kind") or "round") == "manual":
                return ManualSession(
                    started_at=str(data.get("started_at") or ""),
                    ros_state=str(data.get("ros_state") or ""),
                )
            round_id = str(data["round_id"])
            if not ROUND_ID_PATTERN.match(round_id):
                raise ValueError(f"非法 round_id: {round_id}")
            status = str(data["status"])
            if status not in ("running", "paused"):
                return None
            return ActiveRound(
                round_id=round_id,
                mode=str(data.get("mode") or "inspection"),
                started_at=str(data["started_at"]),
                status=status,
                ros_state=str(data.get("ros_state") or ""),
                operator_id=data.get("operator_id"),
            )
        except (OSError, ValueError, KeyError, TypeError):
            return None

    def save(self, active: ActiveRound | ManualSession) -> None:
        self._path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self._path.with_suffix(self._path.suffix + ".tmp")
        with open(tmp, "w", encoding="utf-8") as handle:
            json.dump(asdict(active), handle, ensure_ascii=False)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(tmp, self._path)

    def clear(self) -> None:
        try:
            self._path.unlink()
        except FileNotFoundError:
            pass


def split_state(text: str) -> tuple[str, str]:
    """``running:inspect:POI_01`` -> ("running", "inspect:POI_01")。"""
    clean = (text or "").strip()
    base, _, phase = clean.partition(":")
    return base, phase


class RoundTracker:
    def __init__(
        self,
        outbox,
        marker: RoundMarker,
        *,
        clock,
        id_factory=None,
        operator_ttl_s: float = 120.0,
    ) -> None:
        self._outbox = outbox
        self._marker = marker
        self._clock = clock
        self._id_factory = id_factory or (lambda: f"run-{uuid.uuid4().hex[:16]}")
        self._operator_ttl_s = operator_ttl_s
        # HTTP 控制线程与 ROS 回调线程都会访问本对象，统一用一把锁保护
        self._lock = threading.Lock()
        loaded = marker.load()
        if isinstance(loaded, ManualSession):
            self._manual: ManualSession | None = loaded
            self._active: ActiveRound | None = None
            self._needs_resync = False
        else:
            self._manual = None
            self._active = loaded
            self._needs_resync = loaded is not None
        self._pending_operator: str | None = None
        self._operator_at = 0.0

    # -- 控制请求辅助 -------------------------------------------------------
    def note_control(self, cmd: str, operator_id: str, now: float | None = None) -> None:
        """登记 start 的操作人，供随后创建的自动巡检轮次归属（有 TTL）。

        manual 不在这里登记：手动检查不属于巡检轮次，其操作人由独立的
        手动检查机制管理，避免串到后续的自动巡检轮次上。
        """
        if cmd != "start" or not operator_id:
            return
        with self._lock:
            self._pending_operator = operator_id
            self._operator_at = time.monotonic() if now is None else now

    def discard_control(self, operator_id: str) -> None:
        """控制请求明确失败 / 被拒时清理待登记操作人。

        仅当槽位里仍是同一操作人才清除，避免并发请求误清他人的登记。
        """
        if not operator_id:
            return
        with self._lock:
            if self._pending_operator == operator_id:
                self._pending_operator = None

    # -- 状态 ---------------------------------------------------------------
    def current_round(self) -> ActiveRound | None:
        with self._lock:
            return self._active

    def observe_state(self, text: str, now: str | None = None) -> list[dict]:
        with self._lock:
            return self._observe_state_locked(text, now)

    def _observe_state_locked(self, text: str, now: str | None) -> list[dict]:
        timestamp = now or self._clock()
        base, phase = split_state(text)
        events: list[dict] = []

        if base == "idle":
            # 手动会话正常结束（_run_manual 收尾发 idle），不产生任何轮次事件
            self._clear_manual()
            if self._active is not None:
                events.append(self._finish("stopped", "ROS 状态回到 idle", timestamp))
            return events

        if base in ("running", "paused"):
            phase_head = phase.split(":", 1)[0]
            if phase_head == "manual":
                # 手动点名检查不属于巡检轮次：闩住会话（已有活动轮次时
                # 按编排层互斥约束不会出现 manual，防御性忽略、不改轮）；
                # 已闩住时（含 manual:settle 等后续阶段）不重复写 marker
                if self._active is None and self._manual is None:
                    self._begin_manual(text.strip(), timestamp)
                return events
            if self._manual is not None:
                if not _is_auto_only_phase(phase):
                    # 手动流程中的 settle/inspect/speak:<poi> 与自动巡检同形，
                    # 一律忽略，不建轮、不改轮
                    return events
                # 出现自动巡检独有阶段：手动会话的终态曾被错过，闩锁失效
                self._clear_manual()
            if phase_head in IGNORED_PHASES:
                return events
            if self._active is None:
                self._active = ActiveRound(
                    round_id=self._id_factory(),
                    mode="inspection",
                    started_at=timestamp,
                    status=base,
                    ros_state=text.strip(),
                    operator_id=self._take_operator(),
                )
                self._marker.save(self._active)
                events.append(self._start_event(self._active, timestamp))
                self._needs_resync = False
                return events

            changed = (
                self._active.status != base or self._active.ros_state != text.strip()
            )
            self._active.status = base
            self._active.ros_state = text.strip()
            self._marker.save(self._active)
            if self._needs_resync:
                # Bridge 重启后先补发幂等建轮，确保后端有这条记录
                events.append(self._start_event(self._active, timestamp))
                self._needs_resync = False
            if changed:
                events.append(
                    new_event(
                        ROUND_STATE,
                        self._active.round_id,
                        {"status": base, "ros_state": text.strip()},
                        timestamp,
                    )
                )
            return events

        if base in TERMINAL_STATUS:
            # stopped / aborted:manual 也是手动会话的收尾，同样不产生轮次事件
            self._clear_manual()
            if self._active is not None:
                events.append(
                    self._finish(
                        TERMINAL_STATUS[base],
                        f"ROS 状态：{text.strip()}",
                        timestamp,
                    )
                )
            return events

        # 未知状态：只记录，不推断结论（也不解除手动会话闩锁）
        return events

    # -- 正式点位结果 -------------------------------------------------------
    def record_result(
        self,
        *,
        poi: str,
        status_code: int,
        simulated: bool,
        detail: str = "",
        detections_json: str = "",
        checked_at: str | None = None,
        duration_s: float | None = None,
        now: str | None = None,
    ) -> dict | None:
        """把 InspectPoi 正式结果转成事件；没有活动轮次时拒绝（返回 None）。

        手动检查期间没有活动轮次，其 InspectPoi 结果在此被丢弃，
        不会错误关联到任何自动巡检轮次。
        """
        with self._lock:
            if self._active is None:
                return None
            timestamp = now or self._clock()
            event = new_event(
                POI_RESULT,
                self._active.round_id,
                {
                    "poi": poi,
                    "status_code": int(status_code),
                    "simulated": bool(simulated),
                    "detail": detail or "",
                    "detections_json": detections_json or "",
                    "checked_at": checked_at or timestamp,
                    "duration_s": duration_s,
                },
                timestamp,
            )
            # 结果事件用同一个 event_id 做后端幂等键
            event["payload"]["event_id"] = event["event_id"]
            return event

    # -- 内部 ---------------------------------------------------------------
    def _start_event(self, active: ActiveRound, timestamp: str) -> dict:
        return new_event(
            ROUND_START,
            active.round_id,
            {
                "mode": active.mode,
                "operator_id": active.operator_id,
                "started_at": active.started_at,
                "ros_state": active.ros_state,
            },
            timestamp,
        )

    def _finish(self, status: str, summary: str, timestamp: str) -> dict:
        assert self._active is not None
        event = new_event(
            ROUND_FINISH,
            self._active.round_id,
            {"status": status, "summary": summary, "finished_at": timestamp},
            timestamp,
        )
        self._active = None
        self._needs_resync = False
        self._marker.clear()
        return event

    def _begin_manual(self, ros_state: str, timestamp: str) -> None:
        self._manual = ManualSession(started_at=timestamp, ros_state=ros_state)
        # 持久化闩锁：Bridge 在手动检查进行中重启后，仍能忽略后续同形状态
        self._marker.save(self._manual)

    def _clear_manual(self) -> None:
        if self._manual is None:
            return
        self._manual = None
        self._marker.clear()

    def _take_operator(self) -> str | None:
        if not self._pending_operator:
            return None
        if self._clock_monotonic() - self._operator_at > self._operator_ttl_s:
            self._pending_operator = None
            return None
        operator = self._pending_operator
        self._pending_operator = None
        return operator

    def _clock_monotonic(self) -> float:
        return time.monotonic()
