"""登录鉴权：团队共享密码 + HMAC 签名 Session Cookie。

安全设计：
- 密码从环境变量 ``FIRE_DASHBOARD_LOGIN_PASSWORD`` 读取，不硬编码在源码里。
  启动时立即做 PBKDF2-HMAC-SHA256 派生，内存中只保留（盐, 迭代次数, 派生密钥），
  登录校验用 ``hmac.compare_digest`` 常量时间比较，之后不保留明文引用。
- Session 令牌 = base64url(payload) + "." + base64url(HMAC-SHA256 签名)，
  签名密钥来自 ``FIRE_DASHBOARD_SESSION_SECRET``；未配置时使用进程内随机密钥
  （重启即全体会话失效，多 worker 部署必须显式配置）并在启动日志中告警。
- Cookie 属性：HttpOnly + SameSite=Lax + Path=/；Secure 在 HTTPS 请求或
  ``FIRE_DASHBOARD_COOKIE_SECURE=1`` 时启用（开发环境 HTTP 下不启用，
  否则浏览器会丢弃 Cookie）。
- 未配置登录密码时整体fail-closed：登录接口返回 503，所有受保护路径一律拒绝，
  绝不因为漏配环境变量而放行。

放行范围：
- ``/login``、``/logout``、登录页静态资源（styles.css / auth.js）；
- ``/api/ingest/*``：机器人 Bridge 数据接入通道，仍由既有 X-Bridge-Token 校验，
  不受网页登录影响；
- 其余全部路径（含 /api/docs、/api/openapi.json、控制接口、开发模拟接口）
  都要求有效会话。
"""

from __future__ import annotations

import base64
import hashlib
import hmac
import json
import logging
import os
import secrets
import time
from dataclasses import dataclass, field
from pathlib import Path

from fastapi import APIRouter, HTTPException, Request
from fastapi.responses import FileResponse, JSONResponse, RedirectResponse

logger = logging.getLogger("fire_dashboard.auth")

PBKDF2_ITERATIONS = 200_000
SESSION_COOKIE = "fire_dashboard_session"
DEFAULT_SESSION_TTL_S = 12 * 3600
PASSWORD_ENV = "FIRE_DASHBOARD_LOGIN_PASSWORD"
SECRET_ENV = "FIRE_DASHBOARD_SESSION_SECRET"
SECURE_ENV = "FIRE_DASHBOARD_COOKIE_SECURE"
TTL_ENV = "FIRE_DASHBOARD_SESSION_TTL_S"

# 无需登录即可访问的路径（登录页自身与其静态资源、状态查询）
PUBLIC_PATHS = frozenset({"/login", "/logout", "/styles.css", "/auth.js", "/api/auth/status"})


def _b64encode(data: bytes) -> str:
    return base64.urlsafe_b64encode(data).decode("ascii").rstrip("=")


def _b64decode(text: str) -> bytes:
    return base64.urlsafe_b64decode(text + "=" * (-len(text) % 4))


def hash_password(
    password: str, salt: bytes | None = None, iterations: int = PBKDF2_ITERATIONS
):
    """PBKDF2 派生。返回 (盐, 迭代次数, 派生密钥)。"""
    salt = salt if salt is not None else secrets.token_bytes(16)
    derived = hashlib.pbkdf2_hmac("sha256", password.encode("utf-8"), salt, iterations)
    return salt, iterations, derived


def safe_redirect_target(target: str) -> str:
    """只允许站内相对路径，防开放重定向。"""
    if not target or not target.startswith("/") or target.startswith("//"):
        return "/"
    return target


@dataclass
class AuthManager:
    """共享密码校验 + 会话令牌签发/验证。"""

    salt: bytes
    iterations: int
    derived: bytes
    secret: bytes
    ttl_s: int = DEFAULT_SESSION_TTL_S
    enabled: bool = True
    secure_hint: bool = False
    skip: bool = field(default=False)

    @classmethod
    def from_password(
        cls,
        password: str,
        *,
        secret: str | bytes = "",
        ttl_s: int = DEFAULT_SESSION_TTL_S,
        secure_hint: bool = False,
    ) -> "AuthManager":
        """从明文密码构建（只用于测试与显式装配；生产走 from_env）。"""
        salt, iterations, derived = hash_password(password)
        key = secret.encode("utf-8") if isinstance(secret, str) else secret
        ephemeral = not key
        if ephemeral:
            key = secrets.token_bytes(32)
        mgr = cls(
            salt=salt,
            iterations=iterations,
            derived=derived,
            secret=key,
            ttl_s=ttl_s,
            enabled=bool(password),
            secure_hint=secure_hint,
        )
        if ephemeral and password:
            logger.warning("未配置 %s，使用进程内随机会话密钥（重启后所有会话失效）", SECRET_ENV)
        return mgr

    @classmethod
    def from_env(cls) -> "AuthManager":
        """生产路径：全部参数来自环境变量；缺密码时 fail-closed。"""
        password = os.environ.get(PASSWORD_ENV, "")
        secret = os.environ.get(SECRET_ENV, "").strip()
        raw_ttl = os.environ.get(TTL_ENV, "").strip()
        ttl_s = DEFAULT_SESSION_TTL_S
        if raw_ttl:
            try:
                ttl_s = int(raw_ttl)
            except ValueError:
                logger.warning("%s 不是整数，使用默认 %d 秒", TTL_ENV, DEFAULT_SESSION_TTL_S)
        if not password:
            logger.warning(
                "未配置 %s：网页登录保持关闭（fail-closed），所有页面与 API 均不可访问",
                PASSWORD_ENV,
            )
        secure_hint = os.environ.get(SECURE_ENV, "").strip().lower() in ("1", "true", "yes", "on")
        return cls.from_password(password, secret=secret, ttl_s=ttl_s, secure_hint=secure_hint)

    @classmethod
    def skip_auth(cls) -> "AuthManager":
        """完全跳过鉴权（仅用于单元测试 / 离线演示装配），生产不得使用。"""
        salt, iterations, derived = hash_password("disabled")
        return cls(
            salt=salt,
            iterations=iterations,
            derived=derived,
            secret=b"disabled",
            skip=True,
            enabled=False,
        )

    def verify(self, password: str) -> bool:
        if not self.enabled or not password:
            return False
        _, _, derived = hash_password(password, self.salt, self.iterations)
        return hmac.compare_digest(derived, self.derived)

    def issue_token(self) -> str:
        payload = {"exp": int(time.time()) + self.ttl_s, "n": _b64encode(secrets.token_bytes(12))}
        raw = _b64encode(json.dumps(payload, separators=(",", ":")).encode("utf-8"))
        sig = hmac.new(self.secret, raw.encode("ascii"), hashlib.sha256).digest()
        return f"{raw}.{_b64encode(sig)}"

    def validate_token(self, token: str | None) -> bool:
        if self.skip:
            return True
        if not token or "." not in token:
            return False
        raw, _, sig_text = token.rpartition(".")
        expected = hmac.new(self.secret, raw.encode("ascii"), hashlib.sha256).digest()
        try:
            sig_ok = hmac.compare_digest(expected, _b64decode(sig_text))
            payload = json.loads(_b64decode(raw))
        except (ValueError, json.JSONDecodeError):
            return False
        return (
            sig_ok
            and isinstance(payload, dict)
            and int(payload.get("exp", 0)) > int(time.time())
        )

    def set_cookie(self, response, *, secure: bool) -> None:
        response.set_cookie(
            SESSION_COOKIE,
            self.issue_token(),
            max_age=self.ttl_s,
            httponly=True,
            samesite="lax",
            secure=secure,
            path="/",
        )

    def clear_cookie(self, response) -> None:
        response.delete_cookie(SESSION_COOKIE, path="/")


auth_router = APIRouter()


async def _parse_credentials(request: Request) -> tuple[str, str, str]:
    """解析 (模式, 密码, next)。支持 JSON 与 urlencoded 表单，无需 multipart 依赖。"""
    content_type = request.headers.get("content-type", "").split(";")[0].strip().lower()
    if content_type == "application/json":
        try:
            data = await request.json()
        except ValueError as exc:
            raise HTTPException(status_code=422, detail="请求体不是合法 JSON") from exc
        if not isinstance(data, dict):
            raise HTTPException(status_code=422, detail="请求体必须是 JSON 对象")
        return "json", str(data.get("password") or ""), str(data.get("next") or "")
    import urllib.parse

    body = (await request.body()).decode("utf-8", "replace")
    fields = urllib.parse.parse_qs(body, keep_blank_values=True)
    return "form", fields.get("password", [""])[0], fields.get("next", [""])[0]


def _login_response(mode: str, *, ok: bool, detail: str, nxt: str, status: int):
    if mode == "form":
        if ok:
            return RedirectResponse(safe_redirect_target(nxt), status_code=303)
        # setup=服务端未配置登录密码；1=密码错误
        target = "/login?error=setup" if status == 503 else "/login?error=1"
        return RedirectResponse(target, status_code=303)
    if ok:
        return JSONResponse({"ok": True})
    return JSONResponse({"detail": detail}, status_code=status)


STATIC_DIR = Path(__file__).resolve().parent.parent / "static"


@auth_router.get("/login")
def login_page():
    return FileResponse(STATIC_DIR / "login.html", media_type="text/html")


@auth_router.post("/login")
async def login(request: Request):
    auth: AuthManager = request.app.state.auth
    mode, password, nxt = await _parse_credentials(request)
    if not auth.enabled:
        return _login_response(
            mode,
            ok=False,
            detail=f"登录未启用：请配置环境变量 {PASSWORD_ENV}",
            nxt=nxt,
            status=503,
        )
    if auth.verify(password):
        response = _login_response(mode, ok=True, detail="", nxt=nxt, status=200)
        secure = auth.secure_hint or request.url.scheme == "https"
        auth.set_cookie(response, secure=secure)
        return response
    return _login_response(mode, ok=False, detail="密码错误", nxt=nxt, status=401)


@auth_router.post("/logout")
def logout_json(request: Request):
    response = JSONResponse({"ok": True})
    request.app.state.auth.clear_cookie(response)
    return response


@auth_router.get("/logout")
def logout_page(request: Request):
    response = RedirectResponse("/login", status_code=303)
    request.app.state.auth.clear_cookie(response)
    return response


@auth_router.get("/api/auth/status")
def auth_status(request: Request):
    auth: AuthManager = request.app.state.auth
    return JSONResponse(
        {
            "authenticated": auth.validate_token(request.cookies.get(SESSION_COOKIE)),
            "enabled": auth.enabled,
        }
    )
