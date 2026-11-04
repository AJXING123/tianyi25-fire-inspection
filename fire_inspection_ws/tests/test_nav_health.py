#!/usr/bin/env python3
"""底盘健康校验（DTC 分级）离线测试。

背景：底盘 BaseHealthInfo 的 hasFatal/hasError 是聚合标志，只要存在任何
C 类告警（例如「WiFi 网络未连接」）也会把 hasFatal 置 True。实地巡检在
无 WiFi 情况下可以正常跑，因此判定必须看每条 DTC 的 severity：
    A类故障(FATAL) / B类故障(ERROR) -> 阻止运动
    C类故障(WARNING)                -> 仅告警

运行前需已 source ROS 环境与工作区 install：
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    python3 tests/test_nav_health.py
"""

from __future__ import annotations

import os
import sys
from types import SimpleNamespace

_HERE = os.path.dirname(os.path.abspath(__file__))
_WS = os.path.normpath(os.path.join(_HERE, ".."))
_SRC = os.path.join(_WS, "src")
for sub in ("fire_inspection_nav",):
    p = os.path.join(_SRC, sub)
    if p not in sys.path:
        sys.path.insert(0, p)

from fire_inspection_nav.navigate_to_server import (  # noqa: E402
    describe_dtc,
    is_blocking_dtc,
    split_health_errors,
)

TESTS = []


def case(fn):
    TESTS.append(fn)
    return fn


def _dtc(dtc_id, severity, desc="", solution=""):
    return SimpleNamespace(id=dtc_id, severity=severity,
                           description=desc, solution=solution)


# 现场实测的三条 DTC
MAINTENANCE = _dtc(0x01010117, "A类故障(FATAL)", "维护模式已启用",
                   "维护按钮弹起，设备退出手动模式后自动恢复")
WIFI_DOWN = _dtc(0x06030001, "C类故障(WARNING)", "WiFi网络未连接",
                 "WiFi连接恢复后自动恢复")
WIFI_FAIL = _dtc(0x06030002, "C类故障(WARNING)", "WiFi连接失败，请检查SSID和密码",
                 "检查WiFi配置和信号后手动重连")


@case
def test_maintenance_mode_blocks():
    """维护模式（A 类）必须阻止运动。"""
    assert is_blocking_dtc(MAINTENANCE) is True


@case
def test_wifi_warning_does_not_block():
    """WiFi 未连接是 C 类告警：不影响本地导航，不能拦车。"""
    assert is_blocking_dtc(WIFI_DOWN) is False
    assert is_blocking_dtc(WIFI_FAIL) is False


@case
def test_error_severity_blocks():
    """B 类(ERROR) 与英文 severity 同样阻止。"""
    for sev in ("B类故障(ERROR)", "A类故障(FATAL)", "error", "fatal"):
        assert is_blocking_dtc(_dtc(1, sev)) is True, sev


@case
def test_missing_severity_is_conservative():
    """老固件没有 severity 字段时不放行（保守）。"""
    assert is_blocking_dtc(_dtc(1, "")) is True
    assert is_blocking_dtc(_dtc(1, None)) is True
    assert is_blocking_dtc(SimpleNamespace(id=1)) is True


@case
def test_split_only_wifi_warnings_allows_motion():
    """只有 WiFi 告警时：阻断组为空 → 允许运动，告警单独记录。"""
    blocking, warnings = split_health_errors([WIFI_DOWN, WIFI_FAIL])
    assert blocking == [], blocking
    assert len(warnings) == 2, warnings


@case
def test_split_mixed_keeps_maintenance_blocking():
    """混合场景：维护模式仍阻断，WiFi 只进告警组。"""
    blocking, warnings = split_health_errors([MAINTENANCE, WIFI_DOWN, WIFI_FAIL])
    assert blocking == [MAINTENANCE], blocking
    assert warnings == [WIFI_DOWN, WIFI_FAIL], warnings


@case
def test_split_tolerates_empty_and_none():
    assert split_health_errors([]) == ([], [])
    assert split_health_errors(None) == ([], [])


@case
def test_describe_dtc_contains_id_severity_and_solution():
    text = describe_dtc(MAINTENANCE)
    assert "0x01010117" in text, text
    assert "A类故障" in text and "维护模式已启用" in text, text
    assert "维护按钮弹起" in text, text


@case
def test_describe_dtc_tolerates_missing_fields():
    """字段缺失/非法 id 不得抛异常。"""
    text = describe_dtc(SimpleNamespace(id="bad"))
    assert "0x????????" in text, text
    assert describe_dtc(SimpleNamespace()) != ""


def main():
    failed = []
    for fn in TESTS:
        try:
            fn()
        except AssertionError as exc:
            failed.append((fn.__name__, exc))
            print(f"[FAIL] {fn.__name__}: {exc}")
        else:
            print(f"[ok]   {fn.__name__}")
    print(f"\n{len(TESTS) - len(failed)}/{len(TESTS)} 通过")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
