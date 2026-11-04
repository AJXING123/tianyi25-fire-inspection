"""开发用模拟联调场景。

⚠️ 仅用于本地开发与自动化测试，**不得用于真实机器人环境**：
- 生成的轮次 `source='simulated'`，页面显示「模拟数据」；
- 生成的事件 `simulated=1`，页面显示「模拟事件」；
- 不会伪造机器人在线状态，也不会调用任何控制接口。

场景覆盖 README「模拟联调」中的 7 项能力：正常巡检、点位正常/异常、
防空警报、语音播报成功/失败、通信断开与恢复、重复事件幂等。

命令行用法：
    python -m app.simulate --list
    python -m app.simulate normal_round --db /tmp/sim.db
"""

from __future__ import annotations

import argparse
import json
import uuid
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone

from .config import Settings
from .db import connect, init_db
from .repository import finish_round, insert_event, insert_poi_result, insert_round

# 模拟事件一律带 simulated=1；source 仍写事件的实际产生方（alarm_monitor 等），
# 便于验证真实的字段结构，同时用 simulated 明确区分真假。
DEV_OPERATOR = "dev-simulator"


@dataclass(frozen=True)
class Scenario:
    key: str
    title: str
    description: str


SCENARIOS: tuple[Scenario, ...] = (
    Scenario("normal_round", "正常巡检开始与结束", "3 个点位全部正常，自动完成"),
    Scenario("abnormal_round", "点位识别异常", "POI_02 检出异常并生成视觉异常事件"),
    Scenario("alarm", "检测到防空警报", "仅生成警报事件，不代表已语音播报"),
    Scenario("voice_success", "语音播报成功", "独立的语音播报结果事件（成功）"),
    Scenario("voice_failure", "语音播报失败", "独立的语音播报结果事件（失败）"),
    Scenario("connection_lost", "机器人离线 / Bridge 断开", "生成通信异常事件"),
    Scenario("connection_restored", "通信恢复", "生成通信恢复事件"),
    Scenario("duplicate_event", "重复上报同一事件", "同一 event_id 上报两次，只保留一条"),
    Scenario(
        "full_flow",
        "完整流程演示",
        "开始 → 点位正常 → 点位异常 → 防空警报 → 播报成功 → 巡检结束 → 历史可查",
    ),
)

SCENARIO_MAP = {item.key: item for item in SCENARIOS}


def _utc(offset_minutes: int = 0) -> str:
    moment = datetime.now(timezone.utc) - timedelta(minutes=offset_minutes)
    return moment.isoformat(timespec="seconds")


def _new_id(prefix: str) -> str:
    return f"{prefix}-{uuid.uuid4().hex[:12]}"


def _detections(cls: str, conf: float) -> str:
    import json

    return json.dumps(
        [{"cls": cls, "conf": conf, "cx": 0.5, "cy": 0.5, "w": 0.18, "h": 0.3}],
        ensure_ascii=False,
    )


def _add_round(conn, *, plan: list[str], ros_state: str | None = None) -> str:
    round_id = _new_id("sim")
    insert_round(
        conn,
        round_id=round_id,
        source="simulated",
        status="running",
        operator_id=DEV_OPERATOR,
        ros_state=ros_state,
        total_pois=len(plan),
        started_at=_utc(3),
    )
    return round_id


def _record(
    conn,
    round_id: str,
    poi: str,
    sequence: int,
    *,
    status_code: int,
    detail: str,
    cls: str = "",
    conf: float = 0.0,
) -> None:
    insert_poi_result(
        conn,
        round_id=round_id,
        poi=poi,
        sequence=sequence,
        status_code=status_code,
        simulated=True,
        detail=detail,
        detections_json=_detections(cls, conf) if cls else None,
        checked_at=_utc(2),
        duration_s=2.0,
    )


def _event(
    conn,
    *,
    event_type: str,
    source: str,
    message: str,
    severity: str,
    details: dict | None = None,
    round_id: str | None = None,
    poi: str | None = None,
    timestamp: str | None = None,
    event_id: str | None = None,
) -> tuple[str, bool]:
    eid = event_id or _new_id("evt")
    stored = insert_event(
        conn,
        event_id=eid,
        event_type=event_type,
        source=source,
        severity=severity,
        message=message,
        details=details,
        round_id=round_id,
        poi=poi,
        timestamp=timestamp or _utc(1),
        simulated=True,
    )
    return eid, stored


# ---------------------------------------------------------------------------
# 场景实现
# ---------------------------------------------------------------------------


def _scenario_normal_round(conn) -> dict:
    plan = ["POI_01", "POI_02", "POI_03"]
    round_id = _add_round(conn, plan=plan, ros_state="running:inspect:POI_01")
    _record(conn, round_id, "POI_01", 0, status_code=0,
            detail="模拟：灭火器在位", cls="fire_extinguisher", conf=0.9)
    _record(conn, round_id, "POI_02", 1, status_code=0,
            detail="模拟：安全出口指示牌正常", cls="safety_sign", conf=0.85)
    _record(conn, round_id, "POI_03", 2, status_code=0,
            detail="模拟：消防通道畅通", cls="clear_passage", conf=0.8)
    finish_round(
        conn, round_id, status="completed", outcome="normal",
        summary="模拟：3 个点位全部正常", finished_at=_utc(0),
    )
    return {"round_id": round_id, "event_ids": []}


def _scenario_abnormal_round(conn) -> dict:
    plan = ["POI_01", "POI_02", "POI_03"]
    round_id = _add_round(conn, plan=plan, ros_state="running:inspect:POI_02")
    _record(conn, round_id, "POI_01", 0, status_code=0,
            detail="模拟：灭火器在位", cls="fire_extinguisher", conf=0.91)
    _record(conn, round_id, "POI_02", 1, status_code=1,
            detail="模拟：检出吸烟人员", cls="smoking", conf=0.76)
    _record(conn, round_id, "POI_03", 2, status_code=0,
            detail="模拟：安全出口指示牌正常", cls="safety_sign", conf=0.83)
    eid, _ = _event(
        conn,
        event_type="vision_abnormal",
        source="vision",
        severity="warning",
        message="POI_02 视觉识别异常：检出吸烟人员",
        details={"poi": "POI_02", "status_code": 1, "class": "smoking", "confidence": 0.76},
        round_id=round_id,
        poi="POI_02",
        timestamp=_utc(1),
    )
    finish_round(
        conn, round_id, status="completed", outcome="abnormal",
        summary="模拟：POI_02 检出吸烟人员", finished_at=_utc(0),
    )
    return {"round_id": round_id, "event_ids": [eid]}


def _scenario_alarm(conn) -> dict:
    # 只生成警报事件；语音播报结果必须由独立的 voice_broadcast 事件表示，
    # 因此这里 details.voice_broadcast 明确写 "pending"（不代表已播报）。
    eid, _ = _event(
        conn,
        event_type="alarm_detected",
        source="alarm_monitor",
        severity="critical",
        message="检测到防空警报",
        details={
            "zone": "3F 东走廊",
            "confidence": 0.87,
            "trigger": "air_defense_siren",
            "voice_broadcast": "pending",
        },
        timestamp=_utc(0),
    )
    return {"round_id": None, "event_ids": [eid]}


def _scenario_voice(conn, success: bool) -> dict:
    if success:
        eid, _ = _event(
            conn,
            event_type="voice_broadcast",
            source="voice",
            severity="info",
            message="语音播报完成",
            details={
                "result": "success",
                "text": "检测到防空警报，请有序撤离",
                "target": "all",
            },
        )
    else:
        eid, _ = _event(
            conn,
            event_type="voice_broadcast",
            source="voice",
            severity="critical",
            message="语音播报失败",
            details={
                "result": "failure",
                "text": "检测到防空警报，请有序撤离",
                "error": "TTS 服务不可达",
            },
        )
    return {"round_id": None, "event_ids": [eid]}


def _scenario_connection(conn, restored: bool) -> dict:
    if restored:
        eid, _ = _event(
            conn,
            event_type="comm_restored",
            source="bridge",
            severity="info",
            message="ROS Bridge 通信已恢复",
            details={"bridge": "connected", "url": "http://127.0.0.1:8081"},
        )
    else:
        eid, _ = _event(
            conn,
            event_type="comm_failure",
            source="bridge",
            severity="critical",
            message="ROS Bridge 通信断开",
            details={"bridge": "disconnected", "reason": "Bridge HTTP 无响应"},
        )
    return {"round_id": None, "event_ids": [eid]}


def _scenario_duplicate_event(conn) -> dict:
    event_id = _new_id("evt")
    first_id, first = _event(
        conn,
        event_type="alarm_detected",
        source="alarm_monitor",
        severity="critical",
        message="检测到防空警报（重复上报测试）",
        details={"zone": "3F 东走廊", "voice_broadcast": "pending"},
        event_id=event_id,
    )
    _, second = _event(
        conn,
        event_type="alarm_detected",
        source="alarm_monitor",
        severity="critical",
        message="检测到防空警报（重复上报测试）",
        details={"zone": "3F 东走廊", "voice_broadcast": "pending"},
        event_id=event_id,
    )
    return {
        "round_id": None,
        "event_ids": [first_id],
        "duplicate": {"first_inserted": first, "second_inserted": second},
    }


def _scenario_full_flow(conn) -> dict:
    """一条命令演示完整流程，便于验收页面各面板联动。

    流程：开始巡检 → POI_01 正常 → POI_02 异常（视觉事件）→ 防空警报
    → 语音播报成功（独立事件）→ POI_03 正常 → 巡检结束（异常）→ 历史可查。
    """
    plan = ["POI_01", "POI_02", "POI_03"]
    round_id = _add_round(conn, plan=plan, ros_state="running:inspect:POI_01")
    _record(conn, round_id, "POI_01", 0, status_code=0,
            detail="模拟：灭火器在位", cls="fire_extinguisher", conf=0.92)
    _record(conn, round_id, "POI_02", 1, status_code=1,
            detail="模拟：检出吸烟人员", cls="smoking", conf=0.78)
    vision_id, _ = _event(
        conn,
        event_type="vision_abnormal",
        source="vision",
        severity="warning",
        message="POI_02 视觉识别异常：检出吸烟人员",
        details={"poi": "POI_02", "status_code": 1, "class": "smoking", "confidence": 0.78},
        round_id=round_id,
        poi="POI_02",
        timestamp=_utc(2),
    )
    alarm_id, _ = _event(
        conn,
        event_type="alarm_detected",
        source="alarm_monitor",
        severity="critical",
        message="检测到防空警报",
        details={
            "zone": "3F 东走廊",
            "confidence": 0.87,
            "trigger": "air_defense_siren",
            "voice_broadcast": "pending",
        },
        round_id=round_id,
        poi="POI_02",
        timestamp=_utc(1),
    )
    voice_id, _ = _event(
        conn,
        event_type="voice_broadcast",
        source="voice",
        severity="info",
        message="语音播报完成",
        details={"result": "success", "text": "检测到防空警报，请有序撤离", "target": "all"},
        round_id=round_id,
        timestamp=_utc(1),
    )
    _record(conn, round_id, "POI_03", 2, status_code=0,
            detail="模拟：消防通道畅通", cls="clear_passage", conf=0.81)
    finish_round(
        conn, round_id, status="completed", outcome="abnormal",
        summary="模拟：POI_02 检出吸烟人员，并检测到防空警报",
        finished_at=_utc(0),
    )
    return {"round_id": round_id, "event_ids": [vision_id, alarm_id, voice_id]}


_RUNNERS = {
    "normal_round": _scenario_normal_round,
    "abnormal_round": _scenario_abnormal_round,
    "alarm": _scenario_alarm,
    "voice_success": lambda conn: _scenario_voice(conn, True),
    "voice_failure": lambda conn: _scenario_voice(conn, False),
    "connection_lost": lambda conn: _scenario_connection(conn, False),
    "connection_restored": lambda conn: _scenario_connection(conn, True),
    "duplicate_event": _scenario_duplicate_event,
    "full_flow": _scenario_full_flow,
}


def run_scenario(conn, key: str) -> dict:
    """执行一个模拟场景并返回结果摘要；未知场景抛 KeyError。"""
    if key not in _RUNNERS:
        raise KeyError(key)
    scenario = SCENARIO_MAP[key]
    result = _RUNNERS[key](conn)
    return {
        "scenario": scenario.key,
        "title": scenario.title,
        "description": scenario.description,
        "simulated": True,
        **result,
    }


def list_scenarios() -> list[dict]:
    return [
        {"key": item.key, "title": item.title, "description": item.description}
        for item in SCENARIOS
    ]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="消防巡检网页后台 —— 开发模拟场景（不得用于真实机器人）"
    )
    parser.add_argument("scenario", nargs="?", help="场景 key，或使用 --list 查看")
    parser.add_argument("--list", action="store_true", help="列出所有场景")
    parser.add_argument(
        "--db",
        default=str(Settings.from_env().db_path),
        help="SQLite 数据库路径（默认 data/dashboard.db）",
    )
    args = parser.parse_args(argv)

    if args.list or not args.scenario:
        for item in list_scenarios():
            print(f"{item['key']:<20} {item['title']} —— {item['description']}")
        return 0
    if args.scenario not in _RUNNERS:
        print(f"未知场景「{args.scenario}」，可用：{', '.join(_RUNNERS)}")
        return 2

    init_db(args.db)
    conn = connect(args.db)
    try:
        result = run_scenario(conn, args.scenario)
    finally:
        conn.close()

    print("模拟场景已执行（source=simulated / simulated=1，非真实机器人数据）：")
    print(json.dumps(result, ensure_ascii=False, indent=2))
    print(f"数据库：{args.db}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
