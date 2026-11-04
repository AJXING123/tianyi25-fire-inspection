"""开发用模拟联调入口（默认关闭）。

⚠️ 仅在显式设置 ``FIRE_DASHBOARD_DEV_MODE=1`` 时启用；正式部署必须保持关闭。
所有场景写入的数据都带 simulated 标记（轮次 source=simulated、事件 simulated=1），
页面会显示「模拟数据 / 模拟事件」，不会与真实机器人数据混淆。
"""

from __future__ import annotations

import sqlite3
from typing import Annotated

from fastapi import APIRouter, Depends, HTTPException, Request

from .db import connect
from .simulate import list_scenarios, run_scenario

router = APIRouter(prefix="/api/dev")

DEV_HINT = (
    "开发模拟入口未启用。仅开发环境可设置 FIRE_DASHBOARD_DEV_MODE=1 后使用；"
    "正式部署必须保持关闭。"
)


def require_dev_mode(request: Request) -> bool:
    if not request.app.state.settings.dev_mode:
        raise HTTPException(status_code=403, detail=DEV_HINT)
    return True


def get_db(request: Request):
    conn: sqlite3.Connection = connect(request.app.state.db_path)
    try:
        yield conn
    finally:
        conn.close()


DevDep = Annotated[bool, Depends(require_dev_mode)]
DbDep = Annotated[sqlite3.Connection, Depends(get_db)]


@router.get("/scenarios")
def scenarios(_: DevDep) -> dict:
    return {
        "enabled": True,
        "dev": True,
        "simulated": True,
        "note": "模拟数据仅用于开发联调，不代表真实机器人行为",
        "items": list_scenarios(),
    }


@router.post("/scenarios/{key}")
def run(key: str, conn: DbDep, _: DevDep) -> dict:
    try:
        result = run_scenario(conn, key)
    except KeyError:
        raise HTTPException(status_code=404, detail=f"未知模拟场景「{key}」") from None
    return {"ok": True, "dev": True, "simulated": True, **result}
