"""运行配置：全部可通过环境变量覆盖，默认只监听本机回环地址。"""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path

# web_dashboard/ 目录（app/ 的上一级）
BASE_DIR = Path(__file__).resolve().parent.parent

DEFAULT_BRIDGE_URL = "http://127.0.0.1:8081"


def _int_env(name: str, default: int) -> int:
    raw = os.environ.get(name, "").strip()
    if not raw:
        return default
    try:
        return int(raw)
    except ValueError as exc:
        raise ValueError(f"环境变量 {name} 必须是整数，当前为「{raw}」") from exc


def _float_env(name: str, default: float) -> float:
    raw = os.environ.get(name, "").strip()
    if not raw:
        return default
    try:
        return float(raw)
    except ValueError as exc:
        raise ValueError(f"环境变量 {name} 必须是数字，当前为「{raw}」") from exc


@dataclass(frozen=True)
class Settings:
    """后端运行设置。

    ros_adapter=disabled（默认）时完全离线；=bridge 时经本机 ROS Bridge 接入 Jazzy。
    bridge_token 同时用于后端→Bridge 的控制鉴权与 Bridge→后端的数据接入鉴权。
    """

    db_path: Path
    ros_adapter: str
    host: str
    port: int
    bridge_url: str = DEFAULT_BRIDGE_URL
    bridge_token: str = ""
    bridge_timeout_s: float = 8.0
    dev_mode: bool = False

    @classmethod
    def from_env(cls) -> "Settings":
        return cls(
            db_path=Path(
                os.environ.get("FIRE_DASHBOARD_DB", "")
                or (BASE_DIR / "data" / "dashboard.db")
            ).expanduser(),
            ros_adapter=(
                os.environ.get("FIRE_DASHBOARD_ROS_ADAPTER", "disabled").strip().lower()
            ),
            host=os.environ.get("FIRE_DASHBOARD_HOST", "127.0.0.1").strip(),
            port=_int_env("FIRE_DASHBOARD_PORT", 8000),
            bridge_url=(
                os.environ.get("FIRE_DASHBOARD_BRIDGE_URL", DEFAULT_BRIDGE_URL).strip()
                or DEFAULT_BRIDGE_URL
            ).rstrip("/"),
            bridge_token=os.environ.get("FIRE_DASHBOARD_BRIDGE_TOKEN", "").strip(),
            bridge_timeout_s=_float_env("FIRE_DASHBOARD_BRIDGE_TIMEOUT_S", 8.0),
            dev_mode=os.environ.get("FIRE_DASHBOARD_DEV_MODE", "").strip().lower()
            in ("1", "true", "yes", "on"),
        )
