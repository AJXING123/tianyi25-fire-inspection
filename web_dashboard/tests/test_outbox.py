"""Outbox 单元测试：持久化、ack、损坏恢复、容量上限。"""

from __future__ import annotations

import json

import pytest

from bridge.events import ROUND_START, new_event
from bridge.outbox import EventOutbox, OutboxFull


def event(idx: int) -> dict:
    return new_event(
        ROUND_START,
        f"run-{idx:016d}",
        {"mode": "inspection"},
        "2026-10-28T00:00:00+00:00",
    )


def test_append_and_persist_across_restart(tmp_path):
    path = tmp_path / "outbox.json"
    first = EventOutbox(path)
    first.append([event(1), event(2)])

    reopened = EventOutbox(path)
    assert [e["round_id"] for e in reopened.pending()] == [
        "run-0000000000000001",
        "run-0000000000000002",
    ]


def test_ack_removes_only_target(tmp_path):
    outbox = EventOutbox(tmp_path / "outbox.json")
    events = [event(1), event(2)]
    outbox.append(events)
    outbox.ack(events[0]["event_id"])

    assert [e["round_id"] for e in outbox.pending()] == ["run-0000000000000002"]
    assert len(EventOutbox(tmp_path / "outbox.json")) == 1


def test_corrupt_file_is_quarantined(tmp_path):
    path = tmp_path / "outbox.json"
    path.write_text("{not json", encoding="utf-8")

    outbox = EventOutbox(path)
    assert outbox.pending() == []
    assert (tmp_path / "outbox.json.corrupt").exists()


def test_non_list_payload_ignored(tmp_path):
    path = tmp_path / "outbox.json"
    path.write_text(json.dumps({"events": []}), encoding="utf-8")
    assert EventOutbox(path).pending() == []


def test_capacity_limit(tmp_path):
    outbox = EventOutbox(tmp_path / "outbox.json", max_events=2)
    outbox.append([event(1)])
    with pytest.raises(OutboxFull):
        outbox.append([event(2), event(3)])
    assert len(outbox) == 1


def test_empty_append_is_noop(tmp_path):
    path = tmp_path / "outbox.json"
    outbox = EventOutbox(path)
    outbox.append([])
    assert not path.exists()
