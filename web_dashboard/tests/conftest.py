"""测试夹具：每个用例使用独立的临时 SQLite 数据库。

网页登录鉴权：
- 共享夹具 `app`/`client`、`dev_app`/`dev_client` 显式启用登录并预登录；
- 数据接入夹具 `ingest_app`/`ingest_client` 只访问 /api/ingest/*（豁免会话，
  仍校验 X-Bridge-Token），无需登录；
- 直接构造 TestClient 的用例（如数据持久化、e2e）用 AuthManager.skip_auth()
  显式跳过鉴权，登录行为由 test_auth.py 专项覆盖。
"""

from __future__ import annotations

import pytest
from fastapi.testclient import TestClient

from app.auth import AuthManager
from app.config import Settings
from app.main import create_app
from app.ros_adapter import DisabledRosAdapter

BRIDGE_TOKEN = "test-bridge-token"
TEST_LOGIN_PASSWORD = "test-login-password"
TEST_SESSION_SECRET = "test-session-secret"


def make_settings(
    db_path,
    *,
    token: str = BRIDGE_TOKEN,
    adapter: str = "disabled",
    dev_mode: bool = False,
):
    return Settings(
        db_path=db_path,
        ros_adapter=adapter,
        host="127.0.0.1",
        port=8000,
        bridge_token=token,
        dev_mode=dev_mode,
    )


def make_auth() -> AuthManager:
    return AuthManager.from_password(
        TEST_LOGIN_PASSWORD, secret=TEST_SESSION_SECRET
    )


@pytest.fixture
def app(tmp_path):
    return create_app(
        db_path=tmp_path / "dashboard_test.db",
        ros_adapter=DisabledRosAdapter(),
        auth=make_auth(),
    )


def _login(test_client: TestClient) -> None:
    resp = test_client.post("/login", json={"password": TEST_LOGIN_PASSWORD})
    assert resp.status_code == 200, "测试预登录失败"


@pytest.fixture
def client(app):
    # with 上下文会触发 lifespan -> init_db
    with TestClient(app) as test_client:
        _login(test_client)
        yield test_client


@pytest.fixture
def db_path(app):
    return app.state.db_path


@pytest.fixture
def ingest_app(tmp_path):
    """启用了数据接入令牌的独立应用（模拟机器人端后端）。"""
    return create_app(settings=make_settings(tmp_path / "ingest_test.db"), auth=make_auth())


@pytest.fixture
def ingest_client(ingest_app):
    with TestClient(ingest_app) as test_client:
        # 该夹具既走 /api/ingest/*（仅令牌）也回读 /api/*（需登录）
        _login(test_client)
        yield test_client


@pytest.fixture
def dev_app(tmp_path):
    """启用开发模拟入口（FIRE_DASHBOARD_DEV_MODE=1 等价）的应用。"""
    return create_app(
        settings=make_settings(tmp_path / "dev_test.db", dev_mode=True),
        auth=make_auth(),
    )


@pytest.fixture
def dev_client(dev_app):
    with TestClient(dev_app) as test_client:
        _login(test_client)
        yield test_client
