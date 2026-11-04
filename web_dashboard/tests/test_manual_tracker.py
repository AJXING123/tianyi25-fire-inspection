"""ManualTracker 单元测试：手动点名检查的独立生命周期、结果路由、操作人归属。

与 RoundTracker 完全隔离：手动状态绝不创建自动巡检轮次，
正式结果只关联手动会话（含结束后的乱序补录时窗）。
"""

from __future__ import annotations

import time

from bridge.events import MANUAL_FINISH, MANUAL_RESULT, MANUAL_START, MANUAL_STATE
from bridge.manual_tracker import (
    ManualMarker,
    ManualTracker,
    parse_manual_state,
)
from bridge.outbox import EventOutbox

MANUAL_ID = "man-aaaaaaaaaaaaaaa1"
MANUAL_ID_2 = "man-bbbbbbbbbbbbbbb2"


class Clock:
    def __init__(self) -> None:
        self.tick = 0

    def __call__(self) -> str:
        self.tick += 1
        return f"2026-10-31T00:00:{self.tick:02d}+00:00"


def make_tracker(tmp_path, ids=(MANUAL_ID,), **kwargs):
    outbox = EventOutbox(tmp_path / "outbox.json")
    marker = ManualMarker(tmp_path / "active_manual.json")
    counter = {"i": 0}

    def id_factory() -> str:
        value = ids[min(counter["i"], len(ids) - 1)]
        counter["i"] += 1
        return value

    tracker = ManualTracker(
        outbox, marker, clock=Clock(), id_factory=id_factory, **kwargs
    )
    return tracker, outbox, marker


# -- 状态文本解析 -----------------------------------------------------------


def test_parse_new_format_states():
    assert parse_manual_state("running:manual:POI_01") == ("", "POI_01")
    assert parse_manual_state("running:manual:settle:POI_01") == ("settle", "POI_01")
    assert parse_manual_state("running:manual:inspect:POI_02") == ("inspect", "POI_02")
    assert parse_manual_state("running:manual:speak:POI_02") == ("speak", "POI_02")


def test_parse_rejects_non_manual_states():
    assert parse_manual_state("running:inspect:POI_01") is None
    assert parse_manual_state("running:wait") is None
    assert parse_manual_state("paused:manual:POI_01") is None
    assert parse_manual_state("running:manual") is None
    assert parse_manual_state("running:manual:") is None
    assert parse_manual_state("idle") is None
    assert parse_manual_state("") is None


def test_parse_manual_nav_and_intervene_phases():
    """P0-2 新增的手动导航/介入阶段必须能被解析。"""
    assert parse_manual_state("running:manual:nav:POI_01") == ("nav", "POI_01")
    assert parse_manual_state("running:manual:intervene:POI_01") == (
        "intervene",
        "POI_01",
    )
    # 自动巡检的同类阶段不带 manual，不得被手动跟踪器误认
    assert parse_manual_state("running:nav:POI_01") is None
    assert parse_manual_state("running:intervene:POI_01") is None


# -- 建档与状态更新 ---------------------------------------------------------


def test_manual_state_creates_record(tmp_path):
    tracker, _, marker = make_tracker(tmp_path)
    events = tracker.observe_state("running:manual:POI_01")

    assert [e["kind"] for e in events] == [MANUAL_START]
    event = events[0]
    assert event["manual_id"] == MANUAL_ID
    assert "round_id" not in event
    assert event["payload"]["poi"] == "POI_01"
    assert event["payload"]["ros_state"] == "running:manual:POI_01"
    assert tracker.current_manual().poi == "POI_01"
    assert marker.path.exists()


def test_phase_updates_emit_manual_state(tmp_path):
    tracker, _, _ = make_tracker(tmp_path)
    tracker.observe_state("running:manual:settle:POI_01")
    events = tracker.observe_state("running:manual:inspect:POI_01")

    assert [e["kind"] for e in events] == [MANUAL_STATE]
    assert events[0]["manual_id"] == MANUAL_ID
    assert events[0]["payload"] == {"ros_state": "running:manual:inspect:POI_01"}
    assert tracker.current_manual().ros_state == "running:manual:inspect:POI_01"


def test_same_state_does_not_emit_duplicates(tmp_path):
    tracker, _, _ = make_tracker(tmp_path)
    tracker.observe_state("running:manual:inspect:POI_01")
    assert tracker.observe_state("running:manual:inspect:POI_01") == []


def test_auto_round_states_are_ignored(tmp_path):
    """自动巡检状态与手动会话无关：不建档、不发事件、不动 marker。"""
    tracker, _, marker = make_tracker(tmp_path)
    for text in (
        "running:wait",
        "running:inspect:POI_01",
        "paused:inspect:POI_01",
        "running:settle:POI_01",
        "done",
    ):
        assert tracker.observe_state(text) == []
    assert tracker.current_manual() is None
    assert not marker.path.exists()


def test_terminal_states_do_not_create_record(tmp_path):
    """没有活动会话时，idle/stopped 不产生任何手动事件。"""
    tracker, _, _ = make_tracker(tmp_path)
    for text in ("idle", "stopped", "aborted", "done"):
        assert tracker.observe_state(text) == []
    assert tracker.current_manual() is None


# -- 收尾 -------------------------------------------------------------------


def test_idle_finishes_completed(tmp_path):
    tracker, _, marker = make_tracker(tmp_path)
    tracker.observe_state("running:manual:inspect:POI_01")
    events = tracker.observe_state("idle")

    assert [e["kind"] for e in events] == [MANUAL_FINISH]
    assert events[0]["payload"]["status"] == "completed"
    assert events[0]["payload"]["error_message"] is None
    assert tracker.current_manual() is None
    assert not marker.path.exists()


def test_stopped_finishes_stopped(tmp_path):
    tracker, _, _ = make_tracker(tmp_path)
    tracker.observe_state("running:manual:POI_01")
    events = tracker.observe_state("stopped")

    assert [e["kind"] for e in events] == [MANUAL_FINISH]
    assert events[0]["payload"]["status"] == "stopped"


def test_aborted_finishes_with_error(tmp_path):
    tracker, _, _ = make_tracker(tmp_path)
    tracker.observe_state("running:manual:POI_01")
    events = tracker.observe_state("aborted")

    assert [e["kind"] for e in events] == [MANUAL_FINISH]
    assert events[0]["payload"]["status"] == "aborted"
    assert "aborted" in events[0]["payload"]["error_message"]


def test_new_manual_state_replaces_stale_session(tmp_path):
    """上一条会话终态被错过时：旧会话 aborted 收尾 + 新会话建档，不串单。"""
    tracker, _, _ = make_tracker(tmp_path, ids=(MANUAL_ID, MANUAL_ID_2))
    tracker.observe_state("running:manual:POI_01")
    events = tracker.observe_state("running:manual:POI_02")

    assert [e["kind"] for e in events] == [MANUAL_FINISH, MANUAL_START]
    assert events[0]["manual_id"] == MANUAL_ID
    assert events[0]["payload"]["status"] == "aborted"
    assert "POI_02" in events[0]["payload"]["error_message"]
    assert events[1]["manual_id"] == MANUAL_ID_2
    assert events[1]["payload"]["poi"] == "POI_02"
    assert tracker.current_manual().poi == "POI_02"


# -- 正式结果路由 -----------------------------------------------------------


def result_kwargs(**overrides):
    kwargs = {
        "poi": "POI_01",
        "status_code": 1,
        "simulated": True,
        "detail": "检出异常",
        "detections_json": '[{"cls": "fire", "conf": 0.9}]',
        "checked_at": "2026-10-31T00:00:05+00:00",
    }
    kwargs.update(overrides)
    return kwargs


def test_result_recorded_on_active_session(tmp_path):
    tracker, _, _ = make_tracker(tmp_path)
    tracker.observe_state("running:manual:inspect:POI_01")
    event = tracker.record_result(**result_kwargs())

    assert event is not None
    assert event["kind"] == MANUAL_RESULT
    assert event["manual_id"] == MANUAL_ID
    payload = event["payload"]
    # 确定性幂等键：Bridge 重发同一份结果时后端识别为重复
    assert payload["event_id"] == f"{MANUAL_ID}-result"
    assert payload["poi"] == "POI_01"
    assert payload["status_code"] == 1
    assert payload["simulated"] is True
    assert payload["detail"] == "检出异常"
    assert payload["detections_json"] == '[{"cls": "fire", "conf": 0.9}]'
    assert payload["checked_at"] == "2026-10-31T00:00:05+00:00"


def test_result_without_session_rejected(tmp_path):
    tracker, _, _ = make_tracker(tmp_path)
    assert tracker.record_result(**result_kwargs()) is None


def test_result_poi_mismatch_rejected(tmp_path):
    """自动巡检点位的正式结果绝不能关联到不同点位的手动会话。"""
    tracker, _, _ = make_tracker(tmp_path)
    tracker.observe_state("running:manual:POI_01")
    assert tracker.record_result(**result_kwargs(poi="POI_99")) is None
    assert tracker.current_manual().result_recorded is False


def test_duplicate_result_rejected(tmp_path):
    """一次手动点检只记录一份正式结果。"""
    tracker, _, _ = make_tracker(tmp_path)
    tracker.observe_state("running:manual:POI_01")
    assert tracker.record_result(**result_kwargs()) is not None
    assert tracker.record_result(**result_kwargs()) is None


def test_late_result_after_finish_within_window(tmp_path):
    """结果话题晚于 idle 到达（跨话题乱序）：短时窗内补录到刚结束的会话。"""
    tracker, _, _ = make_tracker(tmp_path)
    tracker.observe_state("running:manual:inspect:POI_01")
    tracker.observe_state("idle")

    event = tracker.record_result(**result_kwargs())
    assert event is not None
    assert event["manual_id"] == MANUAL_ID


def test_late_result_after_window_rejected(tmp_path):
    tracker, _, _ = make_tracker(tmp_path)
    tracker.observe_state("running:manual:inspect:POI_01")
    tracker.observe_state("idle")
    # 模拟结束时间远在时窗之外
    tracker._last_finished_at = time.monotonic() - 3600

    assert tracker.record_result(**result_kwargs()) is None


# -- 操作人归属 -------------------------------------------------------------


def test_operator_assigned_when_arg_matches(tmp_path):
    tracker, _, _ = make_tracker(tmp_path)
    tracker.note_control("manual", "POI_01", "op-alice")
    events = tracker.observe_state("running:manual:POI_01")

    assert events[0]["payload"]["operator_id"] == "op-alice"
    assert tracker.current_manual().operator_id == "op-alice"


def test_operator_arg_mismatch_not_assigned(tmp_path):
    """登记点位与实际点位不一致时不取走操作人，避免串单。"""
    tracker, _, _ = make_tracker(tmp_path)
    tracker.note_control("manual", "POI_99", "op-alice")
    events = tracker.observe_state("running:manual:POI_01")

    assert events[0]["payload"]["operator_id"] is None
    # 未被取走：随后匹配点位的会话仍可使用
    tracker.observe_state("idle")
    events = tracker.observe_state("running:manual:POI_99")
    assert events[0]["payload"]["operator_id"] == "op-alice"


def test_operator_ttl_expired(tmp_path):
    tracker, _, _ = make_tracker(tmp_path, operator_ttl_s=5.0)
    tracker.note_control("manual", "POI_01", "op-alice", now=time.monotonic() - 10.0)
    events = tracker.observe_state("running:manual:POI_01")

    assert events[0]["payload"]["operator_id"] is None


def test_discard_control_clears_same_operator_only(tmp_path):
    tracker, _, _ = make_tracker(tmp_path)
    tracker.note_control("manual", "POI_01", "op-alice")
    tracker.discard_control("op-bob")  # 不同操作人：不清
    tracker.discard_control("op-alice")
    events = tracker.observe_state("running:manual:POI_01")

    assert events[0]["payload"]["operator_id"] is None


def test_non_manual_control_not_registered(tmp_path):
    tracker, _, _ = make_tracker(tmp_path)
    tracker.note_control("start", "", "op-alice")
    tracker.note_control("stop", "", "op-alice")
    events = tracker.observe_state("running:manual:POI_01")

    assert events[0]["payload"]["operator_id"] is None


# -- 重启恢复 ---------------------------------------------------------------


def test_restart_resyncs_start_event(tmp_path):
    """Bridge 重启后从 marker 恢复会话，下一条状态重发幂等建档事件。"""
    tracker, _, _ = make_tracker(tmp_path)
    tracker.observe_state("running:manual:POI_01")

    # 模拟重启：同一 marker 路径建新 tracker
    revived, _, marker2 = make_tracker(tmp_path)
    assert revived.current_manual() is not None
    assert revived.current_manual().manual_id == MANUAL_ID

    events = revived.observe_state("running:manual:inspect:POI_01")
    kinds = [e["kind"] for e in events]
    assert MANUAL_START in kinds  # 重发建档（后端幂等，不会产生第二条记录）
    start_event = events[kinds.index(MANUAL_START)]
    assert start_event["manual_id"] == MANUAL_ID
    assert start_event["payload"]["poi"] == "POI_01"


def test_restart_without_marker_creates_new_id(tmp_path):
    """无 marker 的重启（会话已正常结束）不恢复任何会话。"""
    tracker, _, _ = make_tracker(tmp_path)
    tracker.observe_state("running:manual:POI_01")
    tracker.observe_state("idle")

    revived, _, _ = make_tracker(tmp_path)
    assert revived.current_manual() is None
    assert revived.observe_state("idle") == []


def test_marker_ignores_corrupt_file(tmp_path):
    tracker, _, marker = make_tracker(tmp_path)
    marker.path.parent.mkdir(parents=True, exist_ok=True)
    marker.path.write_text("not-json", encoding="utf-8")
    assert marker.load() is None

    marker.path.write_text('{"manual_id": "bad id!"}', encoding="utf-8")
    assert marker.load() is None


# -- 自动阶段对账（P1-2 / P1-3） -----------------------------------------


AUTO_ONLY_STATES = (
    "running:wait",
    "running:nav:POI_01",
    "running:intervene:POI_01",
    "running:return",
    "running:watch",
    "running:speak:start",
    "running:speak:done",
)


def test_restart_then_auto_phase_aborts_stale_manual(tmp_path):
    """Bridge 在手动过程中重启，随后自动巡检开始：旧手动会话必须失效。"""
    tracker, _, marker = make_tracker(tmp_path)
    tracker.observe_state("running:manual:nav:POI_01")

    revived, _, _ = make_tracker(tmp_path)
    assert revived.current_manual() is not None

    for state in AUTO_ONLY_STATES:
        events = revived.observe_state(state)
        if events:
            assert [e["kind"] for e in events] == [MANUAL_FINISH]
            assert events[0]["payload"]["status"] == "aborted"
            assert "失效" in events[0]["payload"]["error_message"]
            break
    assert revived.current_manual() is None
    assert not marker.path.exists()

    # 旧会话作废后，同点位自动结果不得被手动记录吞掉
    assert revived.record_result(**result_kwargs()) is None


def test_auto_phase_aborts_active_manual(tmp_path):
    """未重启但漏掉手动终态、直接看到自动阶段：同样按 aborted 收尾。"""
    tracker, _, marker = make_tracker(tmp_path)
    tracker.observe_state("running:manual:inspect:POI_01")
    events = tracker.observe_state("running:wait")

    assert [e["kind"] for e in events] == [MANUAL_FINISH]
    assert events[0]["payload"]["status"] == "aborted"
    assert tracker.current_manual() is None
    assert not marker.path.exists()


def test_auto_phase_closes_late_result_window(tmp_path):
    """自动任务开始后，刚结束手动会话的迟到结果窗口立即关闭。"""
    tracker, _, _ = make_tracker(tmp_path)
    tracker.observe_state("running:manual:inspect:POI_01")
    tracker.observe_state("idle")
    assert tracker.record_result(**result_kwargs()) is not None  # 窗口内可补录

    other, _, _ = make_tracker(tmp_path)
    other.observe_state("running:manual:inspect:POI_01")
    other.observe_state("idle")
    other.observe_state("running:wait")
    assert other.record_result(**result_kwargs()) is None


def test_manual_nav_intervene_keep_session(tmp_path):
    """手动自身的 nav/intervene 阶段只更新状态，不得误判为会话失效。"""
    tracker, _, marker = make_tracker(tmp_path)
    tracker.observe_state("running:manual:POI_01")

    nav_events = tracker.observe_state("running:manual:nav:POI_01")
    assert [e["kind"] for e in nav_events] == [MANUAL_STATE]
    intervene_events = tracker.observe_state("running:manual:intervene:POI_01")
    assert [e["kind"] for e in intervene_events] == [MANUAL_STATE]

    assert tracker.current_manual() is not None
    assert tracker.current_manual().ros_state == "running:manual:intervene:POI_01"
    assert marker.path.exists()


def test_paused_manual_state_does_not_abort_session(tmp_path):
    """`paused:manual:...` 属手动会话（编排层手动流程不暂停，仅防御）。"""
    tracker, _, _ = make_tracker(tmp_path)
    tracker.observe_state("running:manual:POI_01")
    assert tracker.observe_state("paused:manual:settle:POI_01") == []
    assert tracker.current_manual() is not None
