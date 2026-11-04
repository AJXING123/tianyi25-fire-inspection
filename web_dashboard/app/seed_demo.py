"""写入演示数据（demo）。

🔴 演示数据仅用于本地验证页面展示，必须与真实巡检记录严格区分：
- 轮次 source='demo'、is_demo=1；
- 每个点位 simulated=1；
- round_id 带 demo- 前缀，页面上有醒目的「演示数据」标记。

默认数据库：data/dashboard.db（可用 --db 覆盖）。
用法：
    .venv/bin/python -m app.seed_demo
"""

from __future__ import annotations

import argparse
import json
import uuid
from datetime import datetime, timedelta, timezone

from .config import Settings
from .db import connect, init_db
from .repository import finish_round, insert_poi_result, insert_round

DEMO_PREFIX = "demo-"


def _ago(minutes: int) -> str:
    return (datetime.now(timezone.utc) - timedelta(minutes=minutes)).isoformat(
        timespec="seconds"
    )


def _dets(*items: dict) -> str:
    return json.dumps(items, ensure_ascii=False)


def seed_demo(db_path) -> list[str]:
    """写入 4 轮演示巡检，返回 round_id 列表。"""
    init_db(db_path)
    conn = connect(db_path)
    created: list[str] = []
    try:
        # 1) 正常完成
        rid = f"{DEMO_PREFIX}normal-{uuid.uuid4().hex[:8]}"
        insert_round(
            conn,
            round_id=rid,
            source="demo",
            status="running",
            operator_id="demo",
            ros_state="running:inspect:POI_01",
            started_at=_ago(120),
        )
        for seq, poi in enumerate(("POI_01", "POI_02", "POI_03")):
            insert_poi_result(
                conn,
                round_id=rid,
                poi=poi,
                sequence=seq,
                status_code=0,
                simulated=True,
                detail="演示数据：检出预期类别，设备外观正常",
                detections_json=_dets(
                    {
                        "cls": "fire_extinguisher" if poi == "POI_01" else "safety_sign",
                        "conf": 0.88,
                        "cx": 0.47,
                        "cy": 0.52,
                        "w": 0.18,
                        "h": 0.31,
                    }
                ),
                checked_at=_ago(115 - seq * 3),
                duration_s=2.4,
            )
        finish_round(
            conn,
            rid,
            status="completed",
            outcome="normal",
            summary="3 个巡检点均正常",
            finished_at=_ago(100),
        )
        created.append(rid)

        # 2) 正常完成但检测到异常
        rid = f"{DEMO_PREFIX}abnormal-{uuid.uuid4().hex[:8]}"
        insert_round(
            conn,
            round_id=rid,
            source="demo",
            status="running",
            operator_id="demo",
            ros_state="running:inspect:POI_02",
            started_at=_ago(90),
        )
        insert_poi_result(
            conn,
            round_id=rid,
            poi="POI_01",
            sequence=0,
            status_code=0,
            simulated=True,
            detail="演示数据：灭火器在位",
            detections_json=_dets(
                {"cls": "fire_extinguisher", "conf": 0.9, "cx": 0.44, "cy": 0.5, "w": 0.2, "h": 0.34}
            ),
            checked_at=_ago(86),
            duration_s=2.1,
        )
        insert_poi_result(
            conn,
            round_id=rid,
            poi="POI_02",
            sequence=1,
            status_code=1,
            simulated=True,
            detail="演示数据：检出吸烟人员",
            detections_json=_dets(
                {"cls": "smoking", "conf": 0.72, "cx": 0.62, "cy": 0.48, "w": 0.09, "h": 0.15}
            ),
            checked_at=_ago(82),
            duration_s=2.8,
        )
        insert_poi_result(
            conn,
            round_id=rid,
            poi="POI_03",
            sequence=2,
            status_code=0,
            simulated=True,
            detail="演示数据：安全出口指示牌正常",
            detections_json=_dets(
                {"cls": "safety_sign", "conf": 0.83, "cx": 0.5, "cy": 0.42, "w": 0.12, "h": 0.2}
            ),
            checked_at=_ago(78),
            duration_s=2.3,
        )
        finish_round(
            conn,
            rid,
            status="completed",
            outcome="abnormal",
            summary="POI_02 检出吸烟人员，其余点位正常",
            finished_at=_ago(72),
        )
        created.append(rid)

        # 3) 中途被人为停止，POI_02 被跳过
        rid = f"{DEMO_PREFIX}stopped-{uuid.uuid4().hex[:8]}"
        insert_round(
            conn,
            round_id=rid,
            source="demo",
            status="running",
            operator_id="demo",
            started_at=_ago(60),
        )
        insert_poi_result(
            conn,
            round_id=rid,
            poi="POI_01",
            sequence=0,
            status_code=0,
            simulated=True,
            detail="演示数据：灭火器在位",
            detections_json=_dets(
                {"cls": "fire_extinguisher", "conf": 0.86, "cx": 0.4, "cy": 0.5, "w": 0.19, "h": 0.32}
            ),
            checked_at=_ago(56),
            duration_s=2.2,
        )
        insert_poi_result(
            conn,
            round_id=rid,
            poi="POI_02",
            sequence=1,
            skipped=True,
            simulated=True,
            detail="演示数据：操作员跳过该点位",
            checked_at=_ago(52),
        )
        finish_round(
            conn,
            rid,
            status="stopped",
            summary="操作员在第 2 个点位前停止本轮巡检",
            finished_at=_ago(52),
        )
        created.append(rid)

        # 4) 进行中（用于验证状态卡片刷新）
        rid = f"{DEMO_PREFIX}running-{uuid.uuid4().hex[:8]}"
        insert_round(
            conn,
            round_id=rid,
            source="demo",
            status="running",
            operator_id="demo",
            ros_state="running:inspect:POI_02",
            started_at=_ago(3),
        )
        insert_poi_result(
            conn,
            round_id=rid,
            poi="POI_01",
            sequence=0,
            status_code=0,
            simulated=True,
            detail="演示数据：灭火器在位",
            detections_json=_dets(
                {"cls": "fire_extinguisher", "conf": 0.9, "cx": 0.42, "cy": 0.51, "w": 0.2, "h": 0.33}
            ),
            checked_at=_ago(2),
            duration_s=2.0,
        )
        created.append(rid)
    finally:
        conn.close()
    return created


def main() -> None:
    parser = argparse.ArgumentParser(description="写入消防巡检网页后台演示数据")
    parser.add_argument(
        "--db",
        default=str(Settings.from_env().db_path),
        help="SQLite 数据库路径（默认 data/dashboard.db）",
    )
    args = parser.parse_args()
    ids = seed_demo(args.db)
    print("已写入演示数据（source=demo, simulated=1，非真实巡检结果）：")
    for rid in ids:
        print(f"  - {rid}")
    print(f"数据库：{args.db}")


if __name__ == "__main__":
    main()
