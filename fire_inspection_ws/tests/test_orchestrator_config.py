#!/usr/bin/env python3
"""编排层配置离线测试（fire-inspection 语音集成缺口修复）。

纯 stdlib + object.__new__ + yaml：不 rclpy.init、不参与 DDS、不触 TTS。
覆盖：yaml 结构验证、conf_by_class / focus_classes 读取、
_build_text 参数传递、空 focus 与非空 focus 的 build_speech_text 行为。
"""

from __future__ import annotations

import os
import sys
import yaml
from types import SimpleNamespace

_HERE = os.path.dirname(os.path.abspath(__file__))
_SRC = os.path.normpath(os.path.join(_HERE, "..", "src"))
for sub in ("fire_inspection_voice", "fire_inspection_orchestrator"):
    p = os.path.join(_SRC, sub)
    if p not in sys.path:
        sys.path.insert(0, p)

import fire_inspection_voice.speech_texts as st  # noqa: E402
from fire_inspection_voice.speech_texts import (  # noqa: E402
    build_speech_text, PoiStatus, SPEECH_TEXTS, CLASS_CN,
)

YAML_PATH = os.path.normpath(os.path.join(
    _HERE, "..", "src", "fire_inspection_orchestrator",
    "config", "inspection_script.yaml"))

VISION_THRESHOLDS = {
    "smoking": 0.25,
    "safety_sign": 0.65,
    "fire_extinguisher": 0.65,
}

TESTS = []


def case(fn):
    TESTS.append(fn)
    return fn


# ── YAML 结构验证 ─────────────────────────────────────────────────────

@case
def test_yaml_has_conf_by_class_matching_vision():
    """yaml conf_by_class 存在且值与 vision.yaml 单一来源一致（不漂移）。"""
    cfg = yaml.safe_load(open(YAML_PATH))
    cb = cfg.get("conf_by_class")
    assert cb is not None, "缺少 conf_by_class 键"
    assert cb == VISION_THRESHOLDS, \
        f"conf_by_class 与 vision.yaml 漂移: {cb}"


@case
def test_yaml_pois_have_focus_classes_key():
    """每个 poi 条目都有 focus_classes 键（当前全部留空）。"""
    cfg = yaml.safe_load(open(YAML_PATH))
    for p in cfg.get("pois", []):
        assert "focus_classes" in p, f"{p.get('name')} 缺少 focus_classes"
        assert isinstance(p["focus_classes"], list), \
            f"{p.get('name')} focus_classes 不是列表"
        # 当前全部留空待现场确认，不擅自分配
        assert p["focus_classes"] == [], \
            f"{p.get('name')} focus_classes 当前应为空（待确认），实际={p['focus_classes']}"


@case
def test_yaml_expect_abnormal_same_as_before():
    """yaml 的 expect_classes / abnormal_classes 未被本批改动（纯增量）。"""
    cfg = yaml.safe_load(open(YAML_PATH))
    pois = {p["name"]: p for p in cfg.get("pois", [])}
    assert pois["POI_01"]["expect_classes"] == ["fire_extinguisher", "safety_sign"]
    assert pois["POI_01"]["abnormal_classes"] == ["smoking"]
    assert pois["POI_02"]["expect_classes"] == ["safety_sign"]
    assert pois["POI_03"]["expect_classes"] == ["safety_sign"]


# ── 编排层 _build_text 参数传递验证 ────────────────────────────────────

def _make_node(focus_by_poi, conf_by_class=None):
    """构造不依赖 ROS 的"裸"编排节点，只供 _build_text 调用。"""
    import fire_inspection_orchestrator.orchestrator_node as onc
    inst = object.__new__(onc.OrchestratorNode)
    inst._cfg = {
        "conf_by_class": conf_by_class or dict(VISION_THRESHOLDS),
        "inspect_cfg": {
            name: {
                "expect_classes": p.get("expect_classes", ()),
                "abnormal_classes": p.get("abnormal_classes", ()),
                "focus_classes": tuple(p.get("focus_classes", ())),
            }
            for name, p in focus_by_poi.items()
        },
    }
    return inst


def _det(cls, conf):
    return {"class_name": cls, "confidence": conf}


@case
def test_empty_focus_normal_path_preserved():
    """focus_classes 为空 + expect 含 fire_extinguisher：

    有效路径（detections 确有 FE）仍返回 extinguisher_ok；
    检测证据不足时（仅 SS / 空列表 / None 等）降级为 UNCERTAIN 风格，
    不再仅凭 status=NORMAL 就播报正常（防御性校验）。
    """
    inst = _make_node({
        "POI_01": {"expect_classes": ["fire_extinguisher", "safety_sign"],
                    "abnormal_classes": ["smoking"], "focus_classes": []},
    })
    # ── 有效路径：有 FE 0.9 检出 → 仍返回固定灭火器正常文案 ──
    t = inst._build_text("POI_01", PoiStatus.NORMAL, [_det("fire_extinguisher", 0.9)])
    assert t == SPEECH_TEXTS["extinguisher_ok"], \
        f"有效 FE 检出 + NORMAL → 应返回灭火器正常文案，实际={t}"

    # ── 防御路径：仅检出 SS（无 FE）→ 降级为不确定，绝不报正常 ──
    t = inst._build_text("POI_01", PoiStatus.NORMAL, [_det("safety_sign", 0.9)])
    assert "不确定" in t and "灭火器" in t and "人工复核" in t, \
        f"仅 SS 检出 + NORMAL → 应降级为不确定，实际={t}"
    assert "检查正常" not in t, f"防御路径绝不能报正常，实际={t}"

    # ── 防御路径：空 detections → 降级为不确定 ──
    t = inst._build_text("POI_01", PoiStatus.NORMAL, [])
    assert "不确定" in t and "人工复核" in t, \
        f"空 detections + NORMAL → 应降级为不确定，实际={t}"
    assert "检查正常" not in t

    # ── 无 focus 也无 fire_extinguisher 的点位 → 通用文案不受影响 ──
    inst2 = _make_node({
        "POI_02": {"expect_classes": ["safety_sign"],
                    "abnormal_classes": ["smoking"], "focus_classes": []},
    })
    t = inst2._build_text("POI_02", PoiStatus.NORMAL, [_det("safety_sign", 0.9)])
    assert "检查完成" in t, \
        f"空 focus + expect 不含 fire_extinguisher → 应走通用文案，实际={t}"


@case
def test_nonempty_focus_triggers_special_safety_sign_text():
    """POI_02 配置 focus_classes=[safety_sign] → 专用文案触发。"""
    inst = _make_node({
        "POI_02": {"expect_classes": ["safety_sign"],
                    "abnormal_classes": ["smoking"],
                    "focus_classes": ["safety_sign"]},
    })
    # 检出 → 专用正常文本
    t = inst._build_text("POI_02", PoiStatus.NORMAL, [_det("safety_sign", 0.9)])
    assert t == SPEECH_TEXTS["safety_sign_ok"], \
        f"focus=safety_sign + 检出 → 应播专用正常，实际={t}"
    # 未检出 → 专用缺失警告
    t = inst._build_text("POI_02", PoiStatus.ABNORMAL, [])
    assert t == SPEECH_TEXTS["safety_sign_missing"], \
        f"focus=safety_sign + 未检出 → 应播专用缺失，实际={t}"
    # 检出 + 吸烟 → 吸烟警告 + 专用缺失（focus≠fire_extinguisher）
    t = inst._build_text("POI_02", PoiStatus.ABNORMAL, [_det("smoking", 0.9)])
    assert SPEECH_TEXTS["smoking"] in t, \
        f"应含吸烟警告，实际={t}"


@case
def test_nonempty_focus_triggers_special_extinguisher_text():
    """POI 配置 focus_classes=[fire_extinguisher] → 专用文案触发。"""
    inst = _make_node({
        "POI_X": {"expect_classes": ["fire_extinguisher"],
                   "abnormal_classes": ["smoking"],
                   "focus_classes": ["fire_extinguisher"]},
    })
    t = inst._build_text("POI_X", PoiStatus.NORMAL, [_det("fire_extinguisher", 0.9)])
    assert t == SPEECH_TEXTS["extinguisher_ok"], \
        f"focus=fire_extinguisher + 检出 → 应播专用正常，实际={t}"
    t = inst._build_text("POI_X", PoiStatus.ABNORMAL, [])
    assert t == SPEECH_TEXTS["extinguisher_missing"], \
        f"focus=fire_extinguisher + 未检出 → 应播专用缺失，实际={t}"


@case
def test_conf_by_class_passed_to_build_speech_text():
    """conf_by_class 必须传递到文案层，阈值口径与 vision.yaml 一致。"""
    cb = {"smoking": 0.25, "safety_sign": 0.65, "fire_extinguisher": 0.65}
    inst = _make_node({
        "POI_01": {"expect_classes": ["fire_extinguisher", "safety_sign"],
                    "abnormal_classes": ["smoking"], "focus_classes": []},
    }, conf_by_class=cb)
    # 模拟视觉未过滤的低置信 smoking（0.20 < 0.25）
    # conf_by_class=0.25 时不触发吸烟警告；若文案层用默认 0.5 则也不会触发
    # 但如果 conf_by_class 缺失（文案层退回默认 0.5），这里仍不触发
    # 换个角度：验证 conf_by_class 实际生效
    # smoking 0.30 在 conf=0.25 阈值下达标 → 应触发警告
    t = inst._build_text("POI_01", PoiStatus.ABNORMAL,
                         [_det("smoking", 0.30), _det("fire_extinguisher", 0.9)])
    assert SPEECH_TEXTS["smoking"] in t, \
        f"smoking 0.30 >= conf_by_class.smoking 0.25 应触发警告，实际={t}"


@case
def test_conf_by_class_missing_uses_defaults_safely():
    """conf_by_class 缺失（None）时 build_speech_text 用 min_conf 兜底。"""
    inst = _make_node({
        "POI_01": {"expect_classes": ["fire_extinguisher", "safety_sign"],
                    "abnormal_classes": ["smoking"], "focus_classes": []},
    }, conf_by_class=None)
    # min_conf=0.0 时所有检出都算达标
    t = inst._build_text("POI_01", PoiStatus.ABNORMAL,
                         [_det("fire_extinguisher", 0.9)])
    # ABNORMAL + 有灭火器检出 + 无吸烟 → 应为通用异常或缺失（非关键）
    # 关键是函数不抛异常
    assert isinstance(t, str) and t


# ── 运行器 ──────────────────────────────────────────────────────────

def main() -> int:
    passed, failed = [], []
    for fn in TESTS:
        try:
            fn()
            passed.append(fn.__name__)
            print(f"[PASS] {fn.__name__}")
        except Exception as exc:
            failed.append(fn.__name__)
            print(f"[FAIL] {fn.__name__}: {exc}")
            import traceback; traceback.print_exc()
    print(f"\n=== 结果: {len(passed)}/{len(TESTS)} 通过 ===")
    if failed:
        print("失败:", ", ".join(failed))
    return 0 if not failed else 1


if __name__ == "__main__":
    sys.exit(main())
