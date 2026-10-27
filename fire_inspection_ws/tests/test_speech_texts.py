#!/usr/bin/env python3
"""speech_texts.py 离线测试（fire-inspection 批次 1）。

纯 stdlib 断言脚本，无 pytest 依赖、不导入 ROS 消息、不触硬件。
预期值全部来自原 robot_controller/src/inspection/report.py 的业务规则
与已确认的冻结接口定义（PoiResult.msg 状态注释），不为通过而改预期。

运行（工作目录任意，无需 source ROS 环境）：
    python3 /home/nvidia/fire-inspection/fire_inspection_ws/tests/test_speech_texts.py
"""

from __future__ import annotations

import os
import sys
import traceback
from types import SimpleNamespace

_HERE = os.path.dirname(os.path.abspath(__file__))
_VOICE_SRC = os.path.normpath(os.path.join(_HERE, "..", "src",
                                            "fire_inspection_voice"))
if _VOICE_SRC not in sys.path:
    sys.path.insert(0, _VOICE_SRC)

from fire_inspection_voice.speech_texts import (  # noqa: E402
    CLASS_CN,
    SPEECH_TEXTS,
    PoiStatus,
    STATUS_SKIPPED,
    build_speech_text,
    parse_poi_focus,
)

# 与队友确认的逐类阈值口径：smoking 0.25 / 其他 0.65
CONF = {"smoking": 0.25, "safety_sign": 0.65, "fire_extinguisher": 0.65}


class R:
    """最小结果载体：模拟冻结接口解析后的形态。

    status 对应 PoiResult.msg 的 uint8 字段；detections 对应
    detections_json 解析后的 dict 列表（{"class_name", "confidence"}）。
    """

    def __init__(self, status, detections=()):
        self.status = status
        self.detections = list(detections)


def det(cls: str, conf: float) -> dict:
    return {"class_name": cls, "confidence": conf}


TESTS = []


def case(fn):
    TESTS.append(fn)
    return fn


# ── 冻结接口与固定文本钉死 ──────────────────────────────────────────

@case
def test_frozen_status_constants():
    """PoiStatus 数值必须与 PoiResult.msg 注释逐字一致。"""
    assert PoiStatus.NORMAL == 0
    assert PoiStatus.ABNORMAL == 1
    assert PoiStatus.UNCERTAIN == 2
    assert PoiStatus.TIMEOUT == 3
    assert PoiStatus.NO_FRAME == 4
    assert PoiStatus.NO_MODEL == 5
    assert PoiStatus.ERROR == 6


@case
def test_frozen_speech_texts_literals():
    """7 条固定文本必须与已确认文案逐字一致（含安全出口专用两条）。"""
    assert SPEECH_TEXTS["start"] == "消防巡检开始，请注意现场安全。"
    assert SPEECH_TEXTS["extinguisher_ok"] == "灭火器检查正常。"
    assert SPEECH_TEXTS["extinguisher_missing"] == \
        "警告，未检测到灭火器，请工作人员检查。"
    assert SPEECH_TEXTS["safety_sign_ok"] == "安全出口指示灯检查正常。"
    assert SPEECH_TEXTS["safety_sign_missing"] == \
        "警告，未检测到安全出口指示灯，请工作人员检查。"
    assert SPEECH_TEXTS["smoking"] == "警告，发现吸烟行为，请立即停止吸烟。"
    assert SPEECH_TEXTS["done"] == "本次消防巡检已完成。"


@case
def test_class_cn_mapping():
    assert CLASS_CN["smoking"] == "吸烟行为"
    assert CLASS_CN["safety_sign"] == "安全指示牌"
    assert CLASS_CN["fire_extinguisher"] == "灭火器"


# ── 正常（NORMAL）───────────────────────────────────────────────────

@case
def test_normal_extinguisher_global():
    """非专用点 + 期望含灭火器 → 固定灭火器正常文本。"""
    t = build_speech_text("POI_01", R(PoiStatus.NORMAL,
                                      [det("fire_extinguisher", 0.9)]),
                          expect_classes=("fire_extinguisher",),
                          conf_by_class=CONF)
    assert t == SPEECH_TEXTS["extinguisher_ok"]
    # 不得因期望类别里没有 safety_sign 就串播安全出口结论
    assert "安全出口" not in t


@case
def test_normal_safety_sign_focus_1_target():
    t = build_speech_text("安全出口", R(PoiStatus.NORMAL,
                                       [det("safety_sign", 0.9)]),
                          focus_classes=("safety_sign",),
                          conf_by_class=CONF)
    assert t == SPEECH_TEXTS["safety_sign_ok"]


@case
def test_normal_safety_sign_focus_2_targets():
    t = build_speech_text("安全出口", R(PoiStatus.NORMAL,
                                       [det("safety_sign", 0.88),
                                        det("safety_sign", 0.91)]),
                          focus_classes=("safety_sign",),
                          conf_by_class=CONF)
    assert t == SPEECH_TEXTS["safety_sign_ok"]


@case
def test_normal_safety_sign_focus_3_targets():
    t = build_speech_text("安全出口", R(PoiStatus.NORMAL,
                                       [det("safety_sign", 0.8),
                                        det("safety_sign", 0.9),
                                        det("safety_sign", 0.95)]),
                          focus_classes=("safety_sign",),
                          conf_by_class=CONF)
    assert t == SPEECH_TEXTS["safety_sign_ok"]


@case
def test_normal_safety_sign_one_valid_one_low_conf():
    """一有效 + 一低置信（<0.65）仍算正常，不设数量上限。"""
    t = build_speech_text("安全出口", R(PoiStatus.NORMAL,
                                       [det("safety_sign", 0.9),
                                        det("safety_sign", 0.3)]),
                          focus_classes=("safety_sign",),
                          conf_by_class=CONF)
    assert t == SPEECH_TEXTS["safety_sign_ok"]


@case
def test_normal_focus_order_safety_sign_first():
    """focus 同时含两类时按原分支顺序：safety_sign 优先。"""
    t = build_speech_text("综合点", R(PoiStatus.NORMAL,
                                     [det("safety_sign", 0.9),
                                      det("fire_extinguisher", 0.9)]),
                          focus_classes=("safety_sign", "fire_extinguisher"),
                          conf_by_class=CONF)
    assert t == SPEECH_TEXTS["safety_sign_ok"]


@case
def test_normal_generic_text_no_fire_extinguisher_expect():
    """期望不含灭火器且无 focus → 通用"均在位"文案（原逻辑可达路径）。"""
    t = build_speech_text("POI_03", R(PoiStatus.NORMAL,
                                      [det("safety_sign", 0.9)]),
                          expect_classes=("safety_sign",),
                          conf_by_class=CONF)
    assert t == "POI_03检查完成，安全指示牌均在位，未发现异常"


@case
def test_normal_attribute_style_detections():
    """detections 为属性对象（非 dict）时同样兼容。"""
    obj = SimpleNamespace(class_name="fire_extinguisher", confidence=0.9)
    t = build_speech_text("POI_01", R(PoiStatus.NORMAL, [obj]),
                          expect_classes=("fire_extinguisher",),
                          conf_by_class=CONF)
    assert t == SPEECH_TEXTS["extinguisher_ok"]


# ── 异常（ABNORMAL）─────────────────────────────────────────────────

@case
def test_abnormal_extinguisher_missing():
    t = build_speech_text("POI_01", R(PoiStatus.ABNORMAL),
                          expect_classes=("fire_extinguisher",),
                          conf_by_class=CONF)
    assert t == SPEECH_TEXTS["extinguisher_missing"]


@case
def test_abnormal_safety_sign_missing_focus():
    t = build_speech_text("安全出口", R(PoiStatus.ABNORMAL),
                          focus_classes=("safety_sign",),
                          conf_by_class=CONF)
    assert t == SPEECH_TEXTS["safety_sign_missing"]


@case
def test_abnormal_safety_sign_only_low_conf_is_missing():
    """仅剩低于阈值框 → 不计为检出 → 播缺失警告。"""
    t = build_speech_text("安全出口", R(PoiStatus.ABNORMAL,
                                       [det("safety_sign", 0.3)]),
                          focus_classes=("safety_sign",),
                          conf_by_class=CONF)
    assert t == SPEECH_TEXTS["safety_sign_missing"]


@case
def test_abnormal_non_focus_safety_sign_never_announced():
    """非专用点缺失 safety_sign：不得凭空出现安全出口结论（原规则）。"""
    t = build_speech_text("POI_01", R(PoiStatus.ABNORMAL),
                          expect_classes=("safety_sign",),
                          conf_by_class=CONF)
    assert t == "警告，POI_01发现异常，请现场人员立即确认处理"
    assert "安全出口" not in t and "安全指示牌" not in t


@case
def test_abnormal_smoking_warning_no_fire_word():
    t = build_speech_text("POI_02", R(PoiStatus.ABNORMAL,
                                      [det("smoking", 0.9)]),
                          expect_classes=("fire_extinguisher",),
                          abnormal_classes=("smoking",),
                          conf_by_class=CONF)
    # 吸烟警告 + 灭火器缺失（期望含灭火器且未检出）按原规则拼接
    assert t == SPEECH_TEXTS["smoking"] + SPEECH_TEXTS["extinguisher_missing"]
    assert "火灾" not in t


@case
def test_abnormal_focus_smoking_plus_sign_missing_no_extinguisher():
    """专用点：吸烟 + 缺失安全指示牌；focus 隔离全局期望，不提灭火器。"""
    t = build_speech_text("安全出口", R(PoiStatus.ABNORMAL,
                                       [det("smoking", 0.9)]),
                          expect_classes=("fire_extinguisher", "safety_sign"),
                          abnormal_classes=("smoking",),
                          focus_classes=("safety_sign",),
                          conf_by_class=CONF)
    assert t == SPEECH_TEXTS["smoking"] + SPEECH_TEXTS["safety_sign_missing"]
    assert "灭火器" not in t


@case
def test_abnormal_low_conf_smoking_no_warning():
    """低置信度吸烟（0.1 < 0.25）不触发吸烟警告。"""
    t = build_speech_text("POI_02", R(PoiStatus.ABNORMAL,
                                      [det("smoking", 0.1)]),
                          expect_classes=("fire_extinguisher",),
                          abnormal_classes=("smoking",),
                          conf_by_class=CONF)
    assert t == SPEECH_TEXTS["extinguisher_missing"]
    assert "吸烟" not in t


@case
def test_abnormal_unmatched_fallback():
    """判定异常但阈值口径对不上 → 兜底文案，不编造类别。"""
    t = build_speech_text("POI_02", R(PoiStatus.ABNORMAL),
                          abnormal_classes=("smoking",),
                          conf_by_class=CONF)
    assert t == "警告，POI_02发现异常，请现场人员立即确认处理"


# ── NORMAL 分支防御性校验：检测证据不足时绝不能报正常 ──────────────────

def _assert_uncertain_not_normal(text, *expected_keywords):
    """防御路径断言：必须含"不确定"或"人工复核"，绝不能含"检查正常"。"""
    assert "检查正常" not in text, \
        f"防御路径绝不能报正常，实际={text}"
    assert ("不确定" in text or "人工复核" in text), \
        f"防御路径应降级为不确定/人工复核，实际={text}"
    for kw in expected_keywords:
        assert kw in text, f"防御路径应提到'{kw}'，实际={text}"


@case
def test_normal_focus_empty_no_fe_in_dets_not_normal():
    """空 focus + expect 含 FE + NORMAL + 仅 SS 检出 → 降级不确定，不报正常。"""
    t = build_speech_text("POI_01", R(PoiStatus.NORMAL,
                                       [det("safety_sign", 0.9)]),
                          expect_classes=("fire_extinguisher", "safety_sign"),
                          conf_by_class=CONF)
    _assert_uncertain_not_normal(t, "灭火器")


@case
def test_normal_focus_empty_empty_dets_not_normal():
    """空 focus + expect 含 FE + NORMAL + 空 detections → 降级不确定。"""
    t = build_speech_text("POI_01", R(PoiStatus.NORMAL, []),
                          expect_classes=("fire_extinguisher", "safety_sign"),
                          conf_by_class=CONF)
    _assert_uncertain_not_normal(t)


@case
def test_normal_focus_empty_fe_low_conf_not_normal():
    """空 focus + expect 含 FE + NORMAL + FE 置信度 0.5 < 0.65 → 降级不确定。"""
    t = build_speech_text("POI_01", R(PoiStatus.NORMAL,
                                       [det("fire_extinguisher", 0.5),
                                        det("safety_sign", 0.9)]),
                          expect_classes=("fire_extinguisher", "safety_sign"),
                          conf_by_class=CONF)
    # FE 被 filter 掉，detections 里只剩 SS → 降级不确定，提 FE
    _assert_uncertain_not_normal(t, "灭火器")


@case
def test_normal_focus_fe_no_fe_in_dets_not_normal():
    """focus=[FE] + NORMAL + 空 detections → 降级不确定，不报正常。"""
    t = build_speech_text("POI_X", R(PoiStatus.NORMAL, []),
                          focus_classes=("fire_extinguisher",),
                          conf_by_class=CONF)
    _assert_uncertain_not_normal(t, "灭火器")


@case
def test_normal_focus_fe_only_ss_in_dets_not_normal():
    """focus=[FE] + NORMAL + 仅 SS 检出 → 降级不确定。"""
    t = build_speech_text("POI_X", R(PoiStatus.NORMAL,
                                      [det("safety_sign", 0.9)]),
                          focus_classes=("fire_extinguisher",),
                          conf_by_class=CONF)
    _assert_uncertain_not_normal(t, "灭火器")


@case
def test_normal_focus_ss_no_ss_in_dets_not_normal():
    """focus=[SS] + NORMAL + 空 detections → 降级不确定。"""
    t = build_speech_text("安全出口", R(PoiStatus.NORMAL, []),
                          focus_classes=("safety_sign",),
                          conf_by_class=CONF)
    _assert_uncertain_not_normal(t, "安全指示牌")


@case
def test_normal_focus_ss_only_fe_in_dets_not_normal():
    """focus=[SS] + NORMAL + 仅 FE 检出 → 降级不确定。"""
    t = build_speech_text("安全出口", R(PoiStatus.NORMAL,
                                        [det("fire_extinguisher", 0.9)]),
                          focus_classes=("safety_sign",),
                          conf_by_class=CONF)
    _assert_uncertain_not_normal(t, "安全指示牌")


@case
def test_normal_detections_none_safe_no_crash():
    """detections=None 绝不能崩溃，也不能报正常。"""
    result = R(PoiStatus.NORMAL, [])
    result.detections = None  # 强制 None，绕过 R.__init__
    t = build_speech_text("POI_01", result,
                          expect_classes=("fire_extinguisher",),
                          conf_by_class=CONF)
    _assert_uncertain_not_normal(t, "灭火器")


@case
def test_normal_detections_malformed_safe_no_crash():
    """元素缺 class_name / confidence → 跳过异常项，不报正常。"""
    t = build_speech_text("POI_01", R(PoiStatus.NORMAL,
                                       [{"confidence": 0.9},   # 缺 class_name
                                        {"class_name": "fire_extinguisher"},  # 缺 confidence
                                        {"class_name": "fire_extinguisher",
                                         "confidence": None}]),
                          expect_classes=("fire_extinguisher",),
                          conf_by_class=CONF)
    # 全部元素被跳过 → 无达标 FE → 降级不确定
    _assert_uncertain_not_normal(t, "灭火器")


# ── 通用 NORMAL 分支 fallback 防御：items 非空但未全部达标时降级 ─────────

@case
def test_normal_generic_fallback_expect_ss_no_dets():
    """expect=[SS] + NORMAL + 空 detections → 降级不确定，不报正常。"""
    t = build_speech_text("POINT", R(PoiStatus.NORMAL, []),
                          expect_classes=("safety_sign",),
                          conf_by_class=CONF)
    _assert_uncertain_not_normal(t, "安全指示牌")


@case
def test_normal_generic_fallback_expect_ss_dets_fe_only():
    """expect=[SS] + NORMAL + 只有 FE 检出 → 降级不确定。"""
    t = build_speech_text("POINT", R(PoiStatus.NORMAL,
                                      [det("fire_extinguisher", 0.9)]),
                          expect_classes=("safety_sign",),
                          conf_by_class=CONF)
    _assert_uncertain_not_normal(t, "安全指示牌")


@case
def test_normal_generic_fallback_expect_ss_low_conf():
    """expect=[SS] + NORMAL + SS 0.5<0.65 → filter 后无达标 SS → 降级不确定。"""
    t = build_speech_text("POINT", R(PoiStatus.NORMAL,
                                      [det("safety_sign", 0.5)]),
                          expect_classes=("safety_sign",),
                          conf_by_class=CONF)
    _assert_uncertain_not_normal(t, "安全指示牌")


@case
def test_normal_generic_items_empty_keeps_current_behavior():
    """items 空（expect 空 + focus 空）→ 保留通用文案（无明确检查目标）。

    这是业务规则待确认的场景：无 expect 时 speech_texts 不主动检查任何目标，
    仅依赖视觉层 judge() 的判定。本测试锁定当前"保留通用文案"的行为。
    """
    t = build_speech_text("POINT", R(PoiStatus.NORMAL, []),
                          expect_classes=(), focus_classes=(),
                          conf_by_class=CONF)
    assert t == "POINT检查完成，未发现异常"
    # 允许含"未发现异常"——items 空 + 无异常检出 = 可接受的通用文案


@case
def test_normal_generic_items_empty_but_smoking_detected():
    """items 空 + 有 smoking 检出 + NORMAL → 当前行为记录（超出本批范围）。

    生产中视觉 judge() 对 smoking 检出会判 ABNORMAL（非 NORMAL），
    所以这条路径不可达。但防御纵深上，speech_texts NORMAL 分支不检查
    abnormal_classes——如果视觉层被绕过，这里会输出"未发现异常"。
    本测试只记录当前行为，不尝试修复（上一批 AdvisorTool 标记为
    "防御纵深缺口"，但本轮明确排除 abnormal 规则修改）。
    """
    t = build_speech_text("POINT", R(PoiStatus.NORMAL,
                                      [det("smoking", 0.9)]),
                          expect_classes=(), focus_classes=(),
                          abnormal_classes=("smoking",),
                          conf_by_class=CONF)
    # 当前行为：items 空 → L191 "未发现异常"（不检查 abnormal_classes）
    # 这条路径生产不可达（视觉会判 ABNORMAL），本测试仅记录当前行为
    assert t == "POINT检查完成，未发现异常"


# ── 不确定 / 失败态：绝不播确定性结论 ───────────────────────────────

@case
def test_uncertain_focus_low_conf_asks_review():
    t = build_speech_text("POI_02", R(PoiStatus.UNCERTAIN,
                                      [det("safety_sign", 0.3)]),
                          focus_classes=("safety_sign",),
                          conf_by_class=CONF)
    assert t == "POI_02检查结果不确定，未能确认安全指示牌，请人工复核"
    assert "检查正常" not in t
    assert "未检测到安全出口指示灯" not in t


@case
def test_uncertain_no_missing_generic_review():
    """focus/expect 均为空时无可比目标 → 通用"请人工复核"文案（原规则）。"""
    t = build_speech_text("POI_02", R(PoiStatus.UNCERTAIN),
                          conf_by_class=CONF)
    assert t == "POI_02检查结果不确定，请人工复核"
    assert "正常" not in t


@case
def test_uncertain_focus_without_detections():
    """focus 点位零检出 → UNCERTAIN 文案点名该点目标（原规则）。"""
    t = build_speech_text("POI_02", R(PoiStatus.UNCERTAIN),
                          focus_classes=("safety_sign",),
                          conf_by_class=CONF)
    assert t == "POI_02检查结果不确定，未能确认安全指示牌，请人工复核"


def _assert_review_text(name, status):
    t = build_speech_text(name, R(status), conf_by_class=CONF)
    assert "检查正常" not in t
    assert "未检测到安全出口指示灯" not in t
    return t


@case
def test_no_frame_asks_manual_check():
    t = _assert_review_text("POI_02", PoiStatus.NO_FRAME)
    assert t == "POI_02未能获取相机图像，检查未完成，请人工检查"


@case
def test_timeout_asks_manual_check():
    t = _assert_review_text("POI_02", PoiStatus.TIMEOUT)
    assert t == "POI_02视觉识别超时，检查未完成，请人工检查"


@case
def test_no_model_asks_manual_check():
    t = _assert_review_text("POI_02", PoiStatus.NO_MODEL)
    assert t == "POI_02视觉模型未配置，检查未完成，请人工检查"


@case
def test_error_asks_manual_check():
    t = _assert_review_text("POI_02", PoiStatus.ERROR)
    assert t == "POI_02视觉识别出现异常，检查未完成，请人工检查"


@case
def test_skipped_placeholder_branch():
    """冻结接口无 SKIPPED；None 占位时原分支保持原文案（当前不可达）。"""
    assert STATUS_SKIPPED is None
    t = build_speech_text("POI_02", R(None), conf_by_class=CONF)
    assert t == "POI_02检查已跳过"


@case
def test_unknown_status_never_normal():
    t = build_speech_text("POI_02", R(99), conf_by_class=CONF)
    assert t == "POI_02检查状态未知，请人工复核"
    assert "正常" not in t


# ── parse_poi_focus ────────────────────────────────────────────────

@case
def test_parse_poi_focus_valid():
    assert parse_poi_focus(["POI_02=safety_sign"]) == \
        {"POI_02": ("safety_sign",)}
    assert parse_poi_focus(["POI_02=safety_sign,fire_extinguisher"]) == \
        {"POI_02": ("safety_sign", "fire_extinguisher")}
    assert parse_poi_focus([" POI_02 = safety_sign "]) == \
        {"POI_02": ("safety_sign",)}
    assert parse_poi_focus(["POI_02=safety_sign",
                            "POI_03=fire_extinguisher"]) == \
        {"POI_02": ("safety_sign",), "POI_03": ("fire_extinguisher",)}
    assert parse_poi_focus([]) == {}
    assert parse_poi_focus(None) == {}
    assert parse_poi_focus(["", "   "]) == {}


@case
def test_parse_poi_focus_invalid():
    for bad in (["POI_02"],            # 缺 =
                ["=safety_sign"],      # 缺点位名
                ["POI_02="],           # 缺类别
                ["POI_02 =  , ,"]):    # 类别全为空
        try:
            parse_poi_focus(bad)
        except ValueError:
            pass
        else:
            raise AssertionError(f"应拒绝非法格式: {bad!r}")


# ── 运行器 ──────────────────────────────────────────────────────────

def main() -> int:
    passed, failed = [], []
    for fn in TESTS:
        try:
            fn()
            passed.append(fn.__name__)
            print(f"[PASS] {fn.__name__}")
        except Exception:
            failed.append(fn.__name__)
            print(f"[FAIL] {fn.__name__}")
            traceback.print_exc()
    print(f"\n=== 结果: {len(passed)}/{len(TESTS)} 通过 ===")
    if failed:
        print("失败用例:", ", ".join(failed))
    return 0 if not failed else 1


if __name__ == "__main__":
    sys.exit(main())
