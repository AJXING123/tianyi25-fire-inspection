"""巡检任务控制器：状态机 + 点位调度 + 异常处理。

状态机：
    IDLE → INIT(前置校验+限速) → TO_START
      → 每点: NAV_TO_POI → ALIGN_YAW → INSPECT → REPORT → 下一点
      → [RETURN_TO_START(可选)] → DONE
    任意状态 --取消/失败--> ABORTED → IDLE（清理后）

设计约定：
- 语音失败只记录、不阻塞巡检主流程（语音是辅助通道）；
- 视觉 UNCERTAIN/TIMEOUT/NO_FRAME 绝不落成"正常"；
- 导航失败默认终止（与 run_poi_sequence 行为一致），不自动跳下一点；
- 中断/取消语义（分阶段说明）：
  * 运动段内 Ctrl+C：由 _run_motion_safe（队友代码）转 WorkflowError
    "用户中断" → NavOutcome.CANCELLED → _abort，不经过 run() 的
    KeyboardInterrupt 分支；
  * 停留 dwell：_interruptible_sleep 每 0.1s 检查一次 cancel，
    期间 Ctrl+C 直接抛 KeyboardInterrupt；
  * TTS 播报 / 视觉取图 / 推理等待：均为可中断阻塞调用
    （time.sleep / future 等待 / queue.get 均可被 SIGINT 打断），
    KeyboardInterrupt 上抛至 run() 统一收口为 ABORTED；
  * 程序级 request_cancel() 不打断当前阻塞调用本身，在调用返回后的
    下一步骤间生效；
  * 无法被 Python 层中断的阻塞：TtsSpeaker.say 最长约 40s
    （submit 10s + query 30s）、YoloInspector 取图 frame_timeout 秒、
    推理 inference_timeout 秒（软超时，详见 inspector）。
"""

from __future__ import annotations

import enum
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Optional

from .types import (
    InspectionStatus,
    NavOutcome,
    PoiRecord,
    SpeechResult,
    SpeechState,
    now_iso,
)

__all__ = ["InspectionConfig", "InspectionController", "InspectionSummary", "State"]


class State(enum.Enum):
    IDLE = "IDLE"
    INIT = "INIT"
    TO_START = "TO_START"
    NAV_TO_POI = "NAV_TO_POI"
    ALIGN_YAW = "ALIGN_YAW"
    INSPECT = "INSPECT"
    REPORT = "REPORT"
    RETURN_TO_START = "RETURN_TO_START"
    DONE = "DONE"
    ABORTED = "ABORTED"


@dataclass
class InspectionConfig:
    start_poi: str = "start_point"
    pois: tuple = ("POI_01", "POI_02", "POI_03")
    start_timeout: float = 180.0
    segment_timeout: float = 180.0
    dwell_seconds: float = 3.0
    return_to_start: bool = False
    # 各阶段播报文案；None 表示该阶段不播报
    speech_texts: dict = field(default_factory=lambda: {
        "start": "消防巡检开始，前往起点",
        "arrive": "已到达{name}，开始检查",
        "normal": "{name}检查正常",
        "abnormal": "警告，{name}发现异常",
        "uncertain": "{name}检查结果不确定，请人工复核",
        "vision_failed": "{name}检查未完成",
        "nav_failed": "前往{name}失败，巡检终止",
        "done": "全部点位巡检完成",
        "cancelled": "巡检已取消",
    })
    # 视觉异常时是否中断巡检（True=播报警告后立即终止）
    abort_on_abnormal: bool = False


@dataclass
class InspectionSummary:
    final_state: State = State.IDLE
    detail: str = ""
    poi_finals: dict = field(default_factory=dict)  # poi -> final 字符串

    def to_dict(self) -> dict:
        return {
            "final_state": self.final_state.value,
            "detail": self.detail,
            "poi_finals": self.poi_finals,
        }


class InspectionController:
    def __init__(self, navigator, speaker, inspector, recorder,
                 config: Optional[InspectionConfig] = None,
                 on_transition: Optional[Callable] = None):
        self.nav = navigator
        self.spk = speaker
        self.vision = inspector
        self.rec = recorder
        self.cfg = config or InspectionConfig()
        self.state = State.IDLE
        self._cancel = threading.Event()
        self._on_transition = on_transition or (lambda s, d="": None)

    # ── 对外控制 ────────────────────────────────────────────────────

    def request_cancel(self) -> None:
        """请求取消：步骤之间生效；模拟 dwell 可被立即打断。"""
        self._cancel.set()

    @property
    def cancelled(self) -> bool:
        return self._cancel.is_set()

    # ── 内部工具 ────────────────────────────────────────────────────

    def _set(self, state: State, detail: str = "") -> None:
        self.state = state
        self._on_transition(state, detail)

    def _say(self, key: str, **fmt) -> SpeechResult:
        template = self.cfg.speech_texts.get(key)
        if not template:
            return SpeechResult(state=SpeechState.SKIPPED, text="")
        text = template.format(**fmt)
        interrupt = key in ("start", "abnormal", "nav_failed", "done",
                            "cancelled")
        return self.spk.say(text, interrupt=interrupt)

    def _interruptible_sleep(self, seconds: float) -> bool:
        """返回 True 表示正常睡完，False 表示被 cancel 打断。"""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if self.cancelled:
                return False
            time.sleep(min(0.1, max(0.0, deadline - time.monotonic())))
        return True

    # ── 主流程 ──────────────────────────────────────────────────────

    def run(self) -> InspectionSummary:
        summary = InspectionSummary()
        try:
            if not self._phase_init(summary):
                return summary
            if not self._phase_to_start(summary):
                return summary
            for poi in self.cfg.pois:
                if not self._run_one_poi(poi, summary):
                    return summary
            if self.cfg.return_to_start:
                if not self._phase_return(summary):
                    return summary
            # 终止状态先行落账，再做可能抛异常的播报/写盘（与 _abort 同规）
            summary.final_state = State.DONE
            summary.detail = "全部点位巡检完成"
            try:
                self._set(State.DONE, summary.detail)
            except Exception as exc:
                self.state = State.DONE
                print(f"[!!] 状态迁移回调异常: {exc}", flush=True)
            try:
                self._say("done")
            except Exception as exc:
                print(f"[!!] 完成播报失败: {exc}", flush=True)
            try:
                self.rec.write_summary(State.DONE.value, summary.detail)
            except Exception as exc:
                print(f"[!!] 写入完成 summary 失败: {exc}", flush=True)
            return summary
        except KeyboardInterrupt:
            # Ctrl+C 在 TTS/视觉/停留等非运动阶段抛出时在此统一收口，
            # 绝不能绕过 _abort 导致 summary 缺失或状态滞留。
            # （运动段的 Ctrl+C 已被 _run_motion_safe 转为 CANCELLED，
            #   不会到达本分支）
            self._cancel.set()
            self._abort(summary, "用户中断(Ctrl+C)", speak="cancelled")
            return summary
        except Exception as exc:  # 兜底：任何未预期异常也要清理
            self._abort(summary, f"控制器未预期异常: {exc}")
            return summary
        finally:
            self._cleanup()

    def _phase_init(self, summary) -> bool:
        self._set(State.INIT, "前置校验")
        r = self.nav.precheck(list(self.cfg.pois), self.cfg.start_poi)
        if not r.ok:
            return self._abort(summary, f"前置校验拒绝: {r.detail}", speak=None)
        r = self.nav.apply_speed_limit()
        if not r.ok:
            return self._abort(summary, f"限速失败: {r.detail}", speak=None)
        return True

    def _phase_to_start(self, summary) -> bool:
        self._set(State.TO_START, self.cfg.start_poi)
        self._say("start")
        r = self.nav.nav_to_start(self.cfg.start_poi, self.cfg.start_timeout)
        if self.cancelled or r.outcome is NavOutcome.CANCELLED:
            return self._abort(summary, "用户取消", speak="cancelled")
        if not r.ok:
            return self._abort(summary, f"前往起点失败: {r.detail}",
                               speak="nav_failed", name="起点")
        return True

    def _phase_return(self, summary) -> bool:
        self._set(State.RETURN_TO_START, self.cfg.start_poi)
        r = self.nav.nav_to_start(self.cfg.start_poi, self.cfg.start_timeout)
        if not r.ok:
            return self._abort(summary, f"返回起点失败: {r.detail}",
                               speak="nav_failed", name="起点")
        return True

    def _run_one_poi(self, poi_name: str, summary) -> bool:
        """包装：点位执行中被 Ctrl+C 打断时，尽力落盘 aborted 记录再上抛，
        由 run() 统一收口（终止 summary / cleanup）。"""
        record = PoiRecord(poi=poi_name)
        try:
            return self._run_one_poi_impl(poi_name, summary, record)
        except KeyboardInterrupt:
            record.final = "aborted"
            record.ts_end = now_iso()
            try:
                self.rec.write_poi(record)
            except Exception as exc:
                print(f"[!!] 写入中断点位记录失败: {exc}", flush=True)
            summary.poi_finals[record.poi] = record.final
            raise

    def _run_one_poi_impl(self, poi_name: str, summary, record) -> bool:
        rec_speech = record.speech

        if self.cancelled:
            return self._abort(summary, "用户取消", speak="cancelled",
                               record=record)

        # 1) 轨道导航到点
        self._set(State.NAV_TO_POI, poi_name)
        nav = self.nav.nav_track_to_poi(poi_name, self.cfg.segment_timeout)
        record.nav = nav.to_dict()
        if self.cancelled or nav.outcome is NavOutcome.CANCELLED:
            return self._abort(summary, "用户取消", speak="cancelled",
                               record=record)
        if not nav.ok:
            record.final = "nav_failed"
            rec_speech["nav_failed"] = self._say(
                "nav_failed", name=poi_name).to_dict()
            return self._finish_poi_fail(summary, record,
                                         f"前往{poi_name}失败: {nav.detail}")

        # 2) 对齐朝向
        self._set(State.ALIGN_YAW, poi_name)
        poi_desc = {"name": poi_name, **(self._poi_coords(poi_name) or {})}
        align = self.nav.align_yaw(poi_desc, self.cfg.segment_timeout)
        record.align = align.to_dict()
        if not align.ok:
            record.final = "align_failed"
            return self._finish_poi_fail(summary, record,
                                         f"{poi_name} 转向失败: {align.detail}")

        # 3) 到点播报 + 停留（可被打断） + 视觉检查
        rec_speech["arrive"] = self._say("arrive", name=poi_name).to_dict()
        self._set(State.INSPECT, poi_name)
        if not self._interruptible_sleep(self.cfg.dwell_seconds):
            return self._abort(summary, "用户取消", speak="cancelled",
                               record=record)
        result = self.vision.inspect(poi_name)
        record.inspection = result.to_dict()
        record.pose = self._safe_pose()

        # 4) 结果播报
        self._set(State.REPORT, poi_name)
        key = self._speech_key_for(result.status)
        rec_speech["report"] = self._say(key, name=poi_name).to_dict()

        if result.status is InspectionStatus.ABNORMAL and self.cfg.abort_on_abnormal:
            record.final = "abnormal_abort"
            return self._finish_poi_fail(summary, record,
                                         f"{poi_name} 检出异常，按配置终止")

        record.final = "done"
        record.ts_end = now_iso()
        self.rec.write_poi(record)
        summary.poi_finals[poi_name] = "done"
        return True

    def _poi_coords(self, poi_name: str) -> Optional[dict]:
        pois = getattr(self.nav, "_pois", None) or {}
        p = pois.get(poi_name)
        if isinstance(p, dict):
            return {"x": p.get("x"), "y": p.get("y"), "yaw": p.get("yaw")}
        return None

    @staticmethod
    def _speech_key_for(status: InspectionStatus) -> str:
        if status is InspectionStatus.SUCCESS:
            return "normal"
        if status is InspectionStatus.ABNORMAL:
            return "abnormal"
        if status is InspectionStatus.UNCERTAIN:
            return "uncertain"
        return "vision_failed"

    def _safe_pose(self) -> Optional[dict]:
        try:
            return self.nav.get_pose()
        except Exception:
            return None

    # ── 收尾 ────────────────────────────────────────────────────────

    def _finish_poi_fail(self, summary, record, detail: str) -> bool:
        record.ts_end = now_iso()
        self.rec.write_poi(record)
        summary.poi_finals[record.poi] = record.final
        return self._abort(summary, detail, speak=None)

    def _abort(self, summary, detail: str, speak: Optional[str],
               record=None, **fmt) -> bool:
        # 1) 终止状态先行落账（纯赋值，不会抛异常），确保后续播报/写盘
        #    失败也不会丢失或伪造终止原因
        summary.final_state = State.ABORTED
        summary.detail = detail
        try:
            self._set(State.ABORTED, detail)
        except Exception as exc:
            self.state = State.ABORTED  # 回调异常时的保底
            print(f"[!!] 状态迁移回调异常: {exc}", flush=True)
        # 2) 点位记录：失败仅打印错误，不影响终止流程
        if record is not None:
            record.final = "aborted"
            record.ts_end = now_iso()
            try:
                self.rec.write_poi(record)
            except Exception as exc:
                print(f"[!!] 写入中止点位记录失败: {exc}", flush=True)
            summary.poi_finals[record.poi] = record.final
        # 3) 终止播报：失败不阻塞 summary 落盘
        if speak:
            try:
                self._say(speak, **fmt)
            except Exception as exc:
                print(f"[!!] 终止播报失败: {exc}", flush=True)
        # 4) summary 落盘：失败记录错误，但不覆盖/伪造原始终止原因
        try:
            self.rec.write_summary(State.ABORTED.value, detail)
        except Exception as exc:
            print(f"[!!] 写入终止 summary 失败: {exc}", flush=True)
        return False

    def _cleanup(self) -> None:
        try:
            self.nav.restore_speed_limit()
        except Exception:
            pass
        try:
            self.nav.stop()
        except Exception:
            pass
        self._set(State.IDLE, "cleanup done")
