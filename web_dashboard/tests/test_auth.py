"""网页登录鉴权测试：共享密码登录、会话 Cookie、接口保护、Bridge 通道豁免。

全部通过 httpx/fastaip TestClient 模拟，不启动真实服务、不触碰 ROS。
"""

from __future__ import annotations

import json

from fastapi.testclient import TestClient

from app.auth import SESSION_COOKIE, AuthManager
from app.main import create_app
from conftest import BRIDGE_TOKEN, TEST_LOGIN_PASSWORD, make_settings


def make_app(tmp_path, **kw):
    return create_app(
        settings=make_settings(tmp_path / "auth_test.db"),
        auth=AuthManager.from_password(TEST_LOGIN_PASSWORD, secret="unit-secret"),
        **kw,
    )


def login(client: TestClient, password: str = TEST_LOGIN_PASSWORD):
    return client.post("/login", json={"password": password})


# -- 未登录访问 ---------------------------------------------------------------

def test_unauthenticated_page_redirects_to_login(tmp_path):
    with TestClient(make_app(tmp_path)) as client:
        resp = client.get("/", follow_redirects=False)
        assert resp.status_code == 303
        assert resp.headers["location"].startswith("/login")


def test_unauthenticated_deep_page_carries_next(tmp_path):
    with TestClient(make_app(tmp_path)) as client:
        resp = client.get("/index.html", follow_redirects=False)
        assert resp.status_code == 303
        assert resp.headers["location"] == "/login?next=/index.html"


def test_unauthenticated_api_returns_401(tmp_path):
    with TestClient(make_app(tmp_path)) as client:
        for method, url, payload in (
            ("GET", "/api/status", None),
            ("GET", "/api/rounds", None),
            ("GET", "/api/events", None),
            ("GET", "/api/manual-inspections", None),
            ("POST", "/api/control", {"cmd": "start"}),
            ("POST", "/api/dev/scenarios/demo_basic", {}),
            ("GET", "/api/docs", None),
            ("GET", "/api/openapi.json", None),
        ):
            resp = client.request(method, url, json=payload)
            assert resp.status_code == 401, f"{method} {url} 未被登录门卫拦截"
            assert "detail" in resp.json()


def test_login_page_publicly_accessible(tmp_path):
    with TestClient(make_app(tmp_path)) as client:
        resp = client.get("/login")
        assert resp.status_code == 200
        assert "text/html" in resp.headers["content-type"]
        assert "访问密码" in resp.text


# -- 登录成功 / 失败 -----------------------------------------------------------

def test_wrong_password_rejected(tmp_path):
    with TestClient(make_app(tmp_path)) as client:
        resp = login(client, "wrong-password")
        assert resp.status_code == 401
        assert resp.json()["detail"] == "密码错误"
        assert SESSION_COOKIE not in client.cookies


def test_wrong_password_form_redirects_with_error(tmp_path):
    with TestClient(make_app(tmp_path)) as client:
        resp = client.post(
            "/login",
            data={"password": "wrong", "next": "/"},
            follow_redirects=False,
        )
        assert resp.status_code == 303
        assert resp.headers["location"] == "/login?error=1"


def test_correct_password_sets_session_and_unlocks_site(tmp_path):
    with TestClient(make_app(tmp_path)) as client:
        resp = login(client)
        assert resp.status_code == 200
        assert resp.json() == {"ok": True}

        set_cookie = resp.headers.get("set-cookie", "")
        assert SESSION_COOKIE in set_cookie
        lowered = set_cookie.lower()
        assert "httponly" in lowered
        assert "samesite=lax" in lowered
        assert "path=/" in lowered

        # 登录后：页面与普通 API 都可用
        assert client.get("/", follow_redirects=False).status_code == 200
        assert client.get("/api/status").status_code == 200
        assert client.get("/api/rounds").status_code == 200


def test_correct_password_form_redirects_to_next(tmp_path):
    with TestClient(make_app(tmp_path)) as client:
        resp = client.post(
            "/login",
            data={"password": TEST_LOGIN_PASSWORD, "next": "/index.html"},
            follow_redirects=False,
        )
        assert resp.status_code == 303
        assert resp.headers["location"] == "/index.html"


def test_open_redirect_via_next_is_blocked(tmp_path):
    with TestClient(make_app(tmp_path)) as client:
        resp = client.post(
            "/login",
            data={"password": TEST_LOGIN_PASSWORD, "next": "//evil.example.com"},
            follow_redirects=False,
        )
        assert resp.status_code == 303
        assert resp.headers["location"] == "/"


def test_secure_cookie_flag_on_https(tmp_path):
    app = make_app(tmp_path)
    with TestClient(app, base_url="https://testserver") as client:
        resp = login(client)
        assert resp.status_code == 200
        assert "Secure" in resp.headers.get("set-cookie", "")


# -- 会话有效性 ----------------------------------------------------------------

def test_tampered_session_cookie_rejected(tmp_path):
    with TestClient(make_app(tmp_path)) as client:
        login(client)
        # 清掉真实会话后再塞伪造 Cookie，确保被测的是伪造值
        client.cookies.clear()
        client.cookies.set(SESSION_COOKIE, "forged-payload.forged-signature")
        assert client.get("/api/status").status_code == 401


def test_expired_session_rejected(tmp_path):
    # ttl_s=-1：签发即过期
    app = create_app(
        settings=make_settings(tmp_path / "expired.db"),
        auth=AuthManager.from_password(TEST_LOGIN_PASSWORD, secret="unit-secret", ttl_s=-1),
    )
    with TestClient(app) as client:
        assert login(client).status_code == 200
        expired = app.state.auth.issue_token()
        client.cookies.clear()
        client.cookies.set(SESSION_COOKIE, expired)
        assert client.get("/api/status").status_code == 401


def test_logout_invalidates_session(tmp_path):
    with TestClient(make_app(tmp_path)) as client:
        assert login(client).status_code == 200
        assert client.get("/api/status").status_code == 200

        resp = client.post("/logout")
        assert resp.status_code == 200
        assert resp.json() == {"ok": True}
        assert client.get("/api/status").status_code == 401


def test_auth_status_endpoint(tmp_path):
    with TestClient(make_app(tmp_path)) as client:
        assert client.get("/api/auth/status").json() == {
            "authenticated": False,
            "enabled": True,
        }
        login(client)
        assert client.get("/api/auth/status").json()["authenticated"] is True


# -- fail-closed：未配置登录密码 -----------------------------------------------

def test_fail_closed_when_password_not_configured(tmp_path):
    app = create_app(
        settings=make_settings(tmp_path / "no_pw.db"),
        auth=AuthManager.from_password("", secret="unit-secret"),
    )
    with TestClient(app) as client:
        # 任何密码都登录不了
        assert login(client, "anything").status_code == 503
        # 页面与 API 一律拒绝
        assert client.get("/", follow_redirects=False).status_code == 303
        assert client.get("/api/status").status_code == 401


# -- Bridge 数据接入通道豁免（但令牌校验保持不变） -------------------------------

def test_ingest_channel_works_without_login(tmp_path):
    app = make_app(tmp_path)
    with TestClient(app) as client:
        # 未登录，但携带正确 Bridge Token → 数据接入不受网页登录影响
        resp = client.post(
            "/api/ingest/rounds",
            json={"round_id": "run-auth000000000001"},
            headers={"X-Bridge-Token": BRIDGE_TOKEN},
        )
        assert resp.status_code == 201, resp.text
        assert resp.json()["round_id"] == "run-auth000000000001"


def test_ingest_channel_still_requires_bridge_token(tmp_path):
    app = make_app(tmp_path)
    with TestClient(app) as client:
        # 未登录且无令牌 → 401（Bridge 令牌校验，而非会话校验）
        resp = client.post("/api/ingest/rounds", json={"round_id": "run-auth000000000002"})
        assert resp.status_code == 401
        assert resp.json()["detail"] == "Bridge 令牌无效"

        # 已登录但无令牌 → 同样被 Bridge 令牌校验拦下
        login(client)
        resp = client.post("/api/ingest/rounds", json={"round_id": "run-auth000000000003"})
        assert resp.status_code == 401
        assert resp.json()["detail"] == "Bridge 令牌无效"


def test_wrong_bridge_token_still_rejected_after_login(tmp_path):
    app = make_app(tmp_path)
    with TestClient(app) as client:
        login(client)
        resp = client.post(
            "/api/ingest/rounds",
            json={"round_id": "run-auth000000000004"},
            headers={"X-Bridge-Token": "wrong-token"},
        )
        assert resp.status_code == 401


# -- 密码安全存储 ---------------------------------------------------------------

def test_password_not_stored_in_plaintext_memory(tmp_path):
    auth = AuthManager.from_password(TEST_LOGIN_PASSWORD, secret="unit-secret")
    # 内存中只保留派生密钥，不保留明文
    for plaintext_field in ("password", "plaintext"):
        assert not hasattr(auth, plaintext_field)
    assert auth.derived != TEST_LOGIN_PASSWORD.encode()
    assert auth.verify(TEST_LOGIN_PASSWORD)
    assert not auth.verify("wrong")


def test_session_token_signature_verified(tmp_path):
    import base64 as b64

    auth = AuthManager.from_password(TEST_LOGIN_PASSWORD, secret="unit-secret")
    token = auth.issue_token()
    assert auth.validate_token(token)
    # 篡改 payload 任何字节都应失效
    raw, _, sig = token.rpartition(".")
    payload = json.loads(b64.urlsafe_b64decode(raw + "=" * (-len(raw) % 4)))
    payload["exp"] += 99999
    forged_raw = b64.urlsafe_b64encode(
        json.dumps(payload, separators=(",", ":")).encode()
    ).decode().rstrip("=")
    assert not auth.validate_token(f"{forged_raw}.{sig}")
    assert not auth.validate_token("not-a-token")
    assert not auth.validate_token("")
