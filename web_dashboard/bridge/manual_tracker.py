"""手动点名检查跟踪：与 RoundTracker 完全独立的任务生命周期管理。

设计目标：
- 每次手动检查由 Bridge 生成唯一 manual_id，写入 marker 文件；
- Bridge 重启后读取 marker，用同一 manual_id 重发幂等的建档事件，避免串单；
- 只认 ``running:manual[:<phase>]:<poi>`` 系列状态与终态（idle/stopped/aborted），
  绝不创建或修改自动巡检轮次；旧版编排层（settle/inspect 不带前缀）只损失
  过程状态粒度，建档与结束依然正确；
- 正式结果只来自 /fire_inspection/poi_result（InspectPoi Action 返回值透传）；
  编排层先发结果再发 idle，outbox 又按 FIFO 投递，正常链路结果先于结束到达；
  另保留"刚结束会话短时窗"，容忍结果与状态话题跨话题乱序；
- 手动操作人由本模块的 note_control/discard_control 管理，不写入自动巡检轮次。
"""

from __future__ import annotations

import json
import os
import threading
import time
import uuid
from dataclasses import asdict, dataclass
from pathlib import Path

from .events import MANUAL_FINISH, MANUAL_RESULT, MANUAL_START, MANUAL_STATE, new_event
from .round_tracker import (
    ROUND_ID_PATTERN,
    _is_auto_only_phase,
    split_state,
)

# 手动会话的 ROS 终态 -> 记录状态
TERMINAL_STATUS = {
    "idle": "completed",
    "done": "completed",  # 防御：编排层互斥约束下不会出现，出现也按完成收尾
    "stopped": "stopped",
    "aborted": "aborted",
}


@dataclass
class ActiveManual:
    """进行中的手动点名检查会话。"""

    manual_id: str
    poi: str
    started_at: str
    ros_state: str
    operator_id: str | None = None
    result_recorded: bool = False


class ManualMarker:
    """当前活动手动检查的持久化标记（原子写，与 RoundMarker 同一目录不同文件）。"""

    def __init__(self, path: Path | str) -> None:
        self._path = Path(path)

    @property
    def path(self) -> Path:
        return self._path

    def load(self) -> ActiveManual | None:
        if not self._path.exists():
            return None
        try:
            data = json.loads(self._path.read_text(encoding="utf-8"))
            manual_id = str(data["manual_id"])
            if not ROUND_ID_PATTERN.match(manual_id):
                raise ValueError(f"非法 manual_id: {manual_id}")
            return ActiveManual(
                manual_id=manual_id,
                poi=str(data["poi"]),
                started_at=str(data["started_at"]),
                ros_state=str(data.get("ros_state") or ""),
                operator_id=data.get("operator_id"),
                result_recorded=bool(data.get("result_recorded")),
            )
        except (OSError, ValueError, KeyError, TypeError):
            return None

    def save(self, active: ActiveManual) -> None:
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


def parse_manual_state(text: str) -> tuple[str, str] | None:
    """``running:manual:settle:POI_01`` -> ("settle", "POI_01")；非手动状态返回 None。

    旧版编排层的 ``running:manual:<poi>``（无中间段）-> ("", "<poi>")。
    """
    base, phase = split_state(text)
    if base != "running":
        return None
    head, _, rest = phase.partition(":")
    if head != "manual":
        return None
    if not rest:
        return None  # 单独的 "running:manual" 没有点位，无法建档
    if ":" in rest:
        sub_phase, _, poi = rest.partition(":")
    else:
        sub_phase, poi = "", rest
    if not poi:
        return None
    return sub_phase, poi


class ManualTracker:
    def __init__(
        self,
        outbox,
        marker: ManualMarker,
        *,
        clock,
        id_factory=None,
        operator_ttl_s: float = 120.0,
        late_result_ttl_s: float = 60.0,
    ) -> None:
        self._outbox = outbox
        self._marker = marker
        self._clock = clock
        self._id_factory = id_factory or (lambda: f"man-{uuid.uuid4().hex[:16]}")
        self._operator_ttl_s = operator_ttl_s
        self._late_result_ttl_s = late_result_ttl_s
        # HTTP 控制线程与 ROS 回调线程都会访问本对象，统一用一把锁保护
        self._lock = threading.Lock()
        self._active: ActiveManual | None = marker.load()
        # 重启恢复：下一条手动状态先补发幂等建档，确保后端有这条记录
        self._needs_resync = self._active is not None
        # 刚结束的会话（结果话题可能晚于状态话题到达，短时窗内仍接受补录）
        self._last_finished: ActiveManual | None = None
        self._last_finished_at = 0.0
        # 待登记操作人：(operator_id, arg)，仅 manual 指令登记
        self._pending_operator: tuple[str, str] | None = None
        self._operator_at = 0.0

    # -- 控制请求辅助 -------------------------------------------------------
    def note_control(
        self, cmd: str, arg: str, operator_id: str, now: float | None = None
    ) -> None:
        """登记 manual 指令的操作人与点位，供随后创建的手动会话归属（有 TTL）。"""
        if cmd != "manual" or not operator_id:
            return
        with self._lock:
            self._pending_operator = (operator_id, (arg or "").strip())
            self._operator_at = time.monotonic() if now is None else now

    def discard_control(self, operator_id: str) -> None:
        """控制请求明确失败 / 被拒时清理待登记操作人（仅清同人，避免误清并发请求）。"""
        if not operator_id:
            return
        with self._lock:
            if self._pending_operator and self._pending_operator[0] == operator_id:
                self._pending_operator = None

    # -- 状态 ---------------------------------------------------------------
    def current_manual(self) -> ActiveManual | None:
        with self._lock:
            return self._active

    def observe_state(self, text: str, now: str | None = None) -> list[dict]:
        with self._lock:
            return self._observe_state_locked(text, now)

    def _observe_state_locked(self, text: str, now: str | None) -> list[dict]:
        timestamp = now or self._clock()
        clean = (text or "").strip()
        base, phase = split_state(clean)
        events: list[dict] = []

        parsed = parse_manual_state(clean)
        if parsed is not None:
            _sub_phase, poi = parsed
            if self._active is not None and self._active.poi != poi:
                # 防御：上一条会话的终态被错过，先按 aborted 收尾再建新会话
                events.append(
                    self._finish(
                        "aborted",
                        f"手动会话被新检查 {poi} 取代，原终态未收到",
                        timestamp,
                    )
                )
            if self._active is None:
                self._active = ActiveManual(
                    manual_id=self._id_factory(),
                    poi=poi,
                    started_at=timestamp,
                    ros_state=clean,
                    operator_id=self._take_operator(poi),
                )
                self._marker.save(self._active)
                events.append(self._start_event(self._active, timestamp))
                self._needs_resync = False
                return events
            changed = self._active.ros_state != clean
            self._active.ros_state = clean
            self._marker.save(self._active)
            if self._needs_resync:
                events.append(self._start_event(self._active, timestamp))
                self._needs_resync = False
            if changed:
                events.append(
                    new_event(
                        MANUAL_STATE,
                        self._active.manual_id,
                        {"ros_state": clean},
                        timestamp,
                    )
                )
            return events

        if base in TERMINAL_STATUS and self._active is not None:
            error = None
            if base == "aborted":
                error = f"ROS 状态：{clean}"
            events.append(self._finish(TERMINAL_STATUS[base], error, timestamp))
            return events

        # 编排层互斥约束下，手动会话期间不会出现自动巡检独有阶段
        # （wait/nav/intervene/return/watch、speak:start/done）。出现即说明
        # 手动收尾状态被错过（典型：Bridge 在手动过程中重启），旧会话必须失效：
        #  - 仍活动的会话按 aborted 收尾，不能再接收后续自动结果；
        #  - 刚结束会话的迟到结果窗口同步关闭，避免旧结果被新任务接收。
        if base in ("running", "paused") and _is_auto_only_phase(phase):
            self._last_finished = None
            if self._active is not None:
                events.append(
                    self._finish(
                        "aborted",
                        "自动巡检阶段出现，手动会话已失效",
                        timestamp,
                    )
                )
                # 强制作废的会话不保留迟到结果窗口：自动任务已开始，
                # 之后同点位的结果只可能属于新任务
                self._last_finished = None
                return events
            return events

        # 其他状态（自动巡检 / watch / 未知）与手动会话无关，一律忽略
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
        now: str | None = None,
    ) -> dict | None:
        """把 InspectPoi 正式结果转成事件；无匹配手动会话时拒绝（返回 None）。

        正常链路：结果先于 idle 到达，记入活动会话；
        乱序容忍：结果晚于结束状态到达时，短时窗内补录到刚结束的同点位会话，
        但一旦观察到自动巡检独有阶段，该窗口立即关闭。

        残余歧义：PoiResult.msg 没有 manual_id/round_id，只能依据"当前会话 +
        点位"推断归属。跨任务的迟到结果若恰好落在同点位新会话的活动期内，
        仍可能被错误接收；根治需在接口会签后给结果消息加任务标识。
        """
        with self._lock:
            target = self._match_session(poi)
            if target is None or target.result_recorded:
                return None
            timestamp = now or self._clock()
            event = new_event(
                MANUAL_RESULT,
                target.manual_id,
                {
                    "poi": poi,
                    "status_code": int(status_code),
                    "simulated": bool(simulated),
                    "detail": detail or "",
                    "detections_json": detections_json or "",
                    "checked_at": checked_at or timestamp,
                },
                timestamp,
            )
            # 结果幂等键用确定性 ID：Bridge 重启重发同一份结果时后端识别为重复
            event["payload"]["event_id"] = f"{target.manual_id}-result"
            target.result_recorded = True
            if target is self._active:
                self._marker.save(target)
            return event

    def _match_session(self, poi: str) -> ActiveManual | None:
        if self._active is not None and self._active.poi == poi:
            return self._active
        if (
            self._last_finished is not None
            and self._last_finished.poi == poi
            and time.monotonic() - self._last_finished_at <= self._late_result_ttl_s
        ):
            return self._last_finished
        return None

    # -- 内部 ---------------------------------------------------------------
    def _start_event(self, active: ActiveManual, timestamp: str) -> dict:
        return new_event(
            MANUAL_START,
            active.manual_id,
            {
                "poi": active.poi,
                "operator_id": active.operator_id,
                "started_at": active.started_at,
                "ros_state": active.ros_state,
            },
            timestamp,
        )

    def _finish(
        self, status: str, error_message: str | None, timestamp: str
    ) -> dict:
        assert self._active is not None
        event = new_event(
            MANUAL_FINISH,
            self._active.manual_id,
            {
                "status": status,
                "error_message": error_message,
                "finished_at": timestamp,
            },
            timestamp,
        )
        self._last_finished = self._active
        self._last_finished_at = time.monotonic()
        self._active = None
        self._needs_resync = False
        self._marker.clear()
        return event

    def _take_operator(self, poi: str) -> str | None:
        """取走与点位匹配的待登记操作人；点位不匹配的一律不取，避免串单。"""
        if self._pending_operator is None:
            return None
        operator, arg = self._pending_operator
        if time.monotonic() - self._operator_at > self._operator_ttl_s:
            self._pending_operator = None
            return None
        if arg != poi:
            return None
        self._pending_operator = None
        return operator
