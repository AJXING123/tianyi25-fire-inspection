"""RoundTracker 单元测试：轮次唯一 ID、重启恢复、状态映射、结果归属。"""

from __future__ import annotations

import time

from bridge.events import POI_RESULT, ROUND_FINISH, ROUND_START, ROUND_STATE
from bridge.outbox import EventOutbox
from bridge.round_tracker import RoundMarker, RoundTracker


class Clock:
    def __init__(self) -> None:
        self.tick = 0

    def __call__(self) -> str:
        self.tick += 1
        return f"2026-10-28T00:00:{self.tick:02d}+00:00"


def make_tracker(tmp_path, ids=("run-aaaaaaaaaaaaaaa1",)):
    outbox = EventOutbox(tmp_path / "outbox.json")
    marker = RoundMarker(tmp_path / "active_round.json")
    counter = {"i": 0}

    def id_factory() -> str:
        value = ids[min(counter["i"], len(ids) - 1)]
        counter["i"] += 1
        return value

    tracker = RoundTracker(outbox, marker, clock=Clock(), id_factory=id_factory)
    return tracker, outbox, marker


def test_running_state_creates_one_round(tmp_path):
    tracker, _, marker = make_tracker(tmp_path)
    events = tracker.observe_state("running:inspect:POI_01")

    assert len(events) == 1
    assert events[0]["kind"] == ROUND_START
    assert events[0]["round_id"] == "run-aaaaaaaaaaaaaaa1"
    assert events[0]["payload"]["mode"] == "inspection"
    assert tracker.current_round().ros_state == "running:inspect:POI_01"
    assert marker.path.exists()


def test_running_wait_creates_round(tmp_path):
    tracker, _, marker = make_tracker(tmp_path)
    events = tracker.observe_state("running:wait")

    assert [e["kind"] for e in events] == [ROUND_START]
    assert events[0]["payload"]["mode"] == "inspection"
    assert events[0]["payload"]["ros_state"] == "running:wait"
    assert tracker.current_round().status == "running"
    assert marker.path.exists()


def test_running_inspect_updates_round(tmp_path):
    tracker, _, _ = make_tracker(tmp_path)
    tracker.observe_state("running:wait")
    events = tracker.observe_state("running:inspect:POI_01")

    assert [e["kind"] for e in events] == [ROUND_STATE]
    assert events[0]["payload"] == {
        "status": "running",
        "ros_state": "running:inspect:POI_01",
    }
    assert tracker.current_round().ros_state == "running:inspect:POI_01"


def test_state_change_emits_round_state(tmp_path):
    tracker, _, _ = make_tracker(tmp_path)
    tracker.observe_state("running:inspect:POI_01")
    events = tracker.observe_state("paused:inspect:POI_01")

    assert [e["kind"] for e in events] == [ROUND_STATE]
    assert events[0]["payload"] == {
        "status": "paused",
        "ros_state": "paused:inspect:POI_01",
    }
    assert tracker.current_round().status == "paused"


def test_restart_resyncs_same_round_id(tmp_path):
    tracker, outbox, marker = make_tracker(tmp_path)
    first = tracker.observe_state("running:inspect:POI_01")
    round_id = first[0]["round_id"]

    # 模拟 Bridge 重启：同一个 marker 文件、新的 tracker
    restarted = RoundTracker(
        outbox, marker, clock=Clock(), id_factory=lambda: "run-bbbbbbbbbbbbbbb2"
    )
    events = restarted.observe_state("running:inspect:POI_01")

    assert [e["kind"] for e in events] == [ROUND_START]
    assert events[0]["round_id"] == round_id
    assert restarted.current_round().round_id == round_id


def test_terminal_states_finish_round(tmp_path):
    for state, expected in (
        ("done", "completed"),
        ("stopped", "stopped"),
        ("aborted", "aborted"),
    ):
        case_dir = tmp_path / state
        tracker, _, marker = make_tracker(case_dir)
        tracker.observe_state("running:inspect:POI_01")
        events = tracker.observe_state(state)

        assert len(events) == 1
        assert events[0]["kind"] == ROUND_FINISH
        assert events[0]["payload"]["status"] == expected
        assert tracker.current_round() is None
        assert not marker.path.exists()


def test_idle_finishes_active_round_as_stopped(tmp_path):
    tracker, _, _ = make_tracker(tmp_path)
    tracker.observe_state("running:inspect:POI_01")
    events = tracker.observe_state("idle")

    assert [e["kind"] for e in events] == [ROUND_FINISH]
    assert events[0]["payload"]["status"] == "stopped"


def test_watch_phase_is_not_a_round(tmp_path):
    tracker, outbox, marker = make_tracker(tmp_path)
    assert tracker.observe_state("running:watch") == []
    assert tracker.current_round() is None
    assert len(outbox) == 0
    assert not marker.path.exists()


def test_unknown_state_is_ignored(tmp_path):
    tracker, outbox, _ = make_tracker(tmp_path)
    assert tracker.observe_state("something-new") == []
    assert len(outbox) == 0


def test_resync_after_restart_while_paused(tmp_path):
    tracker, outbox, marker = make_tracker(tmp_path)
    tracker.observe_state("running:inspect:POI_01")
    tracker.observe_state("paused:inspect:POI_01")

    restarted = RoundTracker(outbox, marker, clock=Clock())
    events = restarted.observe_state("paused:inspect:POI_01")
    assert [e["kind"] for e in events] == [ROUND_START]
    assert events[0]["payload"]["mode"] == "inspection"


def test_result_requires_active_round(tmp_path):
    tracker, _, _ = make_tracker(tmp_path)
    assert (
        tracker.record_result(poi="POI_01", status_code=1, simulated=False) is None
    )


def test_result_preserves_raw_fields(tmp_path):
    tracker, _, _ = make_tracker(tmp_path)
    tracker.observe_state("running:inspect:POI_01")
    event = tracker.record_result(
        poi="POI_01",
        status_code=5,
        simulated=True,
        detail="未加载模型",
        detections_json="[]",
        checked_at="2026-10-28T01:02:03+00:00",
    )

    assert event["kind"] == POI_RESULT
    assert event["payload"]["status_code"] == 5
    assert event["payload"]["simulated"] is True
    assert event["payload"]["detail"] == "未加载模型"
    assert event["payload"]["detections_json"] == "[]"
    assert event["payload"]["event_id"] == event["event_id"]


def test_manual_states_create_no_round_and_no_events(tmp_path):
    """手动点名检查全流程：manual → nav → intervene → settle → inspect → speak → idle，
    不建轮、不产生任何事件；结束后的 idle 不得补发 stopped。"""
    tracker, outbox, marker = make_tracker(tmp_path)
    tracker.note_control("manual", "operator-01")

    for state in (
        "running:manual:POI_02",
        "running:manual:nav:POI_02",
        "running:manual:intervene:POI_02",
        "running:manual:settle:POI_02",
        "running:manual:inspect:POI_02",
        "running:manual:speak:POI_02",
    ):
        assert tracker.observe_state(state) == []
        assert tracker.current_round() is None

    assert tracker.observe_state("idle") == []
    assert tracker.current_round() is None
    assert len(outbox) == 0
    assert not marker.path.exists()


def test_manual_nav_intervene_never_clear_latch_or_create_round(tmp_path):
    """P0 回归：手动导航/介入状态带 manual 前缀，不得被当成自动独有阶段。

    修复前 `running:nav:<poi>` 会清除手动闩锁并创建 inspection_round；
    现在 `running:manual:nav:<poi>` 必须被完全忽略。
    """
    tracker, outbox, marker = make_tracker(tmp_path)
    assert tracker.observe_state("running:manual:POI_01") == []
    assert tracker.observe_state("running:manual:nav:POI_01") == []
    assert tracker.observe_state("running:manual:intervene:POI_01") == []
    assert tracker.current_round() is None
    # 闩锁仍在：继续忽略后续手动阶段
    assert tracker.observe_state("running:manual:inspect:POI_01") == []
    assert tracker.observe_state("idle") == []
    assert len(outbox) == 0
    assert tracker.current_round() is None


def test_auto_round_works_after_manual_finishes(tmp_path):
    """手动结束后的 idle 不应误伤随后开始的自动巡检轮次。"""
    tracker, outbox, marker = make_tracker(tmp_path)
    tracker.observe_state("running:manual:POI_02")
    tracker.observe_state("idle")

    events = tracker.observe_state("running:wait")
    assert [e["kind"] for e in events] == ["round_start"]
    assert tracker.current_round() is not None
    tracker.observe_state("done")
    assert tracker.current_round() is None
    assert marker.path.exists() is False


def test_manual_aborted_and_stopped_emit_nothing(tmp_path):
    """手动检查被 stop / 失败中止：终态同样不产生轮次事件。"""
    for terminal in ("stopped", "aborted:manual"):
        case_dir = tmp_path / terminal.replace(":", "_")
        tracker, outbox, marker = make_tracker(case_dir)
        assert tracker.observe_state("running:manual:POI_02") == []
        assert tracker.observe_state(terminal) == []
        assert tracker.current_round() is None
        assert len(outbox) == 0
        assert not marker.path.exists()


def test_manual_session_survives_bridge_restart(tmp_path):
    """Bridge 在手动检查进行中重启：从 marker 恢复闩锁，继续忽略同形状态。"""
    tracker, outbox, marker = make_tracker(tmp_path)
    assert tracker.observe_state("running:manual:POI_02") == []
    assert marker.path.exists()

    restarted = RoundTracker(
        outbox, marker, clock=Clock(), id_factory=lambda: "run-bbbbbbbbbbbbbbb2"
    )
    assert restarted.observe_state("running:settle:POI_02") == []
    assert restarted.observe_state("running:inspect:POI_02") == []
    assert restarted.observe_state("idle") == []
    assert restarted.current_round() is None
    assert len(outbox) == 0
    assert not marker.path.exists()


def test_manual_latch_cleared_by_auto_only_phase(tmp_path):
    """手动会话闩锁期间出现自动巡检独有阶段：闩锁失效，正常建轮。"""
    tracker, _, _ = make_tracker(tmp_path)
    tracker.observe_state("running:manual:POI_02")

    events = tracker.observe_state("running:wait")
    assert [e["kind"] for e in events] == [ROUND_START]
    assert tracker.current_round().round_id == "run-aaaaaaaaaaaaaaa1"


def test_manual_state_does_not_touch_active_round(tmp_path):
    """已有活动轮次时收到 manual 状态（编排层互斥下不应出现）：不改轮。"""
    tracker, _, _ = make_tracker(tmp_path)
    tracker.observe_state("running:inspect:POI_01")

    assert tracker.observe_state("running:manual:POI_02") == []
    active = tracker.current_round()
    assert active is not None
    assert active.ros_state == "running:inspect:POI_01"
    assert active.status == "running"


def test_manual_result_is_not_attributed_to_any_round(tmp_path):
    """手动检查期间的正式点位结果不得关联到自动巡检轮次。"""
    tracker, outbox, _ = make_tracker(tmp_path)
    tracker.observe_state("running:manual:POI_02")
    tracker.observe_state("running:inspect:POI_02")

    assert (
        tracker.record_result(poi="POI_02", status_code=1, simulated=False) is None
    )
    assert tracker.current_round() is None
    assert len(outbox) == 0


def test_start_operator_attributed_to_round(tmp_path):
    tracker, _, _ = make_tracker(tmp_path)
    tracker.note_control("start", "operator-01")
    events = tracker.observe_state("running:wait")

    assert events[0]["payload"]["operator_id"] == "operator-01"
    assert tracker.current_round().operator_id == "operator-01"


def test_manual_operator_does_not_leak_into_next_round(tmp_path):
    """manual 的操作人不登记；后续自动巡检轮次不得继承该操作人。"""
    tracker, _, _ = make_tracker(tmp_path)
    tracker.note_control("manual", "operator-01")
    tracker.observe_state("running:manual:POI_02")
    tracker.observe_state("idle")

    tracker.note_control("start", "operator-02")
    tracker.observe_state("running:wait")

    assert tracker.current_round().operator_id == "operator-02"


def test_discard_control_clears_pending_operator(tmp_path):
    """控制请求明确失败后清理登记：随后建轮不带操作人。"""
    tracker, _, _ = make_tracker(tmp_path)
    tracker.note_control("start", "operator-01")
    tracker.discard_control("operator-01")
    tracker.observe_state("running:wait")

    assert tracker.current_round().operator_id is None


def test_discard_control_keeps_other_operator(tmp_path):
    """并发场景：失败请求的清理不得误清他人登记的操作人。"""
    tracker, _, _ = make_tracker(tmp_path)
    tracker.note_control("start", "operator-01")
    tracker.discard_control("operator-02")
    tracker.observe_state("running:wait")

    assert tracker.current_round().operator_id == "operator-01"


def test_operator_expires_after_ttl(tmp_path):
    tracker, _, _ = make_tracker(tmp_path)
    tracker.note_control("start", "operator-02", now=time.monotonic() - 300)
    tracker.observe_state("running:inspect:POI_01")

    assert tracker.current_round().operator_id is None
