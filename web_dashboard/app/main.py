"""FastAPI 应用装配。

用法（开发机 / 机器人端）：
    .venv/bin/uvicorn app.main:app --host 127.0.0.1 --port 8000

默认 ros_adapter=disabled（完全离线）；配置 bridge 后经 ROS Bridge 接入 Jazzy。
"""

from __future__ import annotations

import sqlite3
from contextlib import asynccontextmanager
from dataclasses import replace
from pathlib import Path
from urllib.parse import quote

from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse, RedirectResponse
from fastapi.staticfiles import StaticFiles

from .api import router
from .auth import (
    PUBLIC_PATHS,
    SESSION_COOKIE,
    AuthManager,
    auth_router,
)
from .config import BASE_DIR, Settings
from .db import init_db
from .dev import router as dev_router
from .ingest import router as ingest_router
from .ros_adapter import RosAdapter, build_ros_adapter

STATIC_DIR = BASE_DIR / "static"


def create_app(
    db_path: Path | str | None = None,
    ros_adapter: RosAdapter | None = None,
    settings: Settings | None = None,
    auth: AuthManager | None = None,
) -> FastAPI:
    settings = settings or Settings.from_env()
    if db_path is not None:
        settings = replace(settings, db_path=Path(db_path))
    adapter = ros_adapter or build_ros_adapter(settings)
    # 生产默认从环境变量构建（缺密码时 fail-closed）；测试可显式注入 AuthManager。
    auth = auth or AuthManager.from_env()

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        init_db(settings.db_path)
        yield

    app = FastAPI(
        title="天轶 2.5 消防巡检网页后台",
        version="0.3.0",
        description=(
            "第三阶段：统一异常事件（防空警报 / 视觉 / 语音 / 通信）+ 开发模拟入口。"
            "默认离线；启用 bridge 后经 ROS Bridge 接入机器人。"
        ),
        lifespan=lifespan,
        docs_url="/api/docs",
        openapi_url="/api/openapi.json",
    )
    app.state.settings = settings
    app.state.db_path = settings.db_path
    app.state.ros_adapter = adapter
    app.state.auth = auth

    app.include_router(auth_router)
    app.include_router(router)
    app.include_router(ingest_router)
    app.include_router(dev_router)

    @app.middleware("http")
    async def auth_middleware(request: Request, call_next):
        """网页登录门卫：保护页面、控制 API、开发模拟 API 与文档。

        - /api/ingest/* 是机器人 Bridge 数据接入通道，不走网页会话，
          仍由 ingest 模块的 X-Bridge-Token 校验（令牌不进浏览器）；
        - /login /logout 及登录页静态资源放行；
        - 其余路径要求有效会话：页面类 303 跳 /login，API 类 401。
        """
        auth: AuthManager = request.app.state.auth
        path = request.url.path
        if auth.skip or path in PUBLIC_PATHS or path.startswith("/api/ingest/"):
            return await call_next(request)
        if auth.validate_token(request.cookies.get(SESSION_COOKIE)):
            return await call_next(request)
        if path.startswith("/api/"):
            return JSONResponse(
                status_code=401,
                content={"detail": "未登录或会话已过期"},
            )
        target = "/login"
        if path != "/":
            target = f"{target}?next={quote(path, safe='/')}"
        return RedirectResponse(target, status_code=303)

    @app.exception_handler(sqlite3.Error)
    async def sqlite_error_handler(request: Request, exc: sqlite3.Error) -> JSONResponse:
        # 数据库暂不可用（损坏 / 被占用）时返回 503，避免把底层异常直接抛成 500
        return JSONResponse(
            status_code=503,
            content={"detail": "数据库暂时不可用，请稍后重试。"},
        )

    app.mount("/", StaticFiles(directory=str(STATIC_DIR), html=True), name="static")
    return app


app = create_app()
