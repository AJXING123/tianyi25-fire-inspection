"""视觉结果 → 中文播报文案映射层（fire-inspection 批次 1 迁移版）。

来源：robot_controller/src/inspection/report.py（逻辑逐字迁移，仅做接口适配）。
职责单一：把巡检结论转成准确、保守的中文播报文本。
不做检测、不调语音、不写盘，纯函数便于离线测试。
固定文本集中在 SPEECH_TEXTS（唯一来源），供所有巡检入口引用。

与冻结接口的对齐（接口契约冻结表 v0.1，PoiResult.msg / InspectPoi.action）：
- status 为 uint8，数值取自 PoiResult.msg 原文注释：
  0 NORMAL / 1 ABNORMAL / 2 UNCERTAIN / 3 TIMEOUT / 4 NO_FRAME /
  5 NO_MODEL / 6 ERROR。msg 未定义具名常量（生成的 Python 类只有
  SLOT_TYPES），故以 PoiStatus(IntEnum) 镜像，离线测试中钉死数值。
- 冻结接口的 NORMAL(0) 与原实现中的 SUCCESS 语义一致（检查正常）。
- 冻结接口没有 SKIPPED 状态：原实现的 SKIPPED 分支原样保留，
  以 STATUS_SKIPPED = None 占位（任何冻结数值都不会命中该分支），
  不自行发明数值；待接口扩展或业务确认后再补齐/删除。
- 冻结接口通过 detections_json 字符串携带检出（无类型化 Detection
  msg）：本模块不负责 JSON 解析；调用方解析后以
  {"class_name": ..., "confidence": ...} 字典列表传入，也兼容带
  同名属性的普通对象。

播报红线（与项目安全约定一致）：
- 检出 smoking 只播"发现吸烟行为"，绝不播报"发生火灾"——
  目标检出 ≠ 火灾确认，是否起火必须由人工现场判断；
- UNCERTAIN / TIMEOUT / NO_FRAME / ERROR / NO_MODEL 一律不播报"正常"，
  统一引导人工复核；
- 文案只引用实际检出且过阈值的类别，不编造未检出的内容。

类别中文名对齐队友 robot_test.py 的 CLASS_CN（三分类模型）。
"""

from __future__ import annotations

from enum import IntEnum
from typing import Any, Optional, Sequence

# ── 状态常量（镜像 PoiResult.msg / InspectPoi.action 冻结定义）───────


class PoiStatus(IntEnum):
    """与 PoiResult.msg 注释一一对应；禁止在此之外新增数值。"""

    NORMAL = 0      # 检查正常（原实现称 SUCCESS，语义一致）
    ABNORMAL = 1    # 检出异常（如吸烟）或必查目标缺失
    UNCERTAIN = 2   # 无法确认，需人工复核
    TIMEOUT = 3     # 推理超时
    NO_FRAME = 4    # 取图失败
    NO_MODEL = 5    # 模型未配置
    ERROR = 6       # 推理异常


# 冻结接口未定义 SKIPPED；None 占位保证原分支不被任何冻结数值命中
STATUS_SKIPPED: Any = None

# 与队友 robot_test.py CLASS_CN 一致；未知类别原样保留英文名
CLASS_CN = {
    "smoking": "吸烟行为",
    "safety_sign": "安全指示牌",
    "fire_extinguisher": "灭火器",
}

# ── 固定中文播报文本（唯一来源）────────────────────────────────────
# 所有巡检入口都必须引用这里，禁止在其他文件重复书写同一句话。
# 统一固定文本、不调用大语言模型临时生成；"未检出吸烟"没有对应文本，
# 即不播报吸烟警告。
SPEECH_TEXTS = {
    "start": "消防巡检开始，请注意现场安全。",
    "extinguisher_ok": "灭火器检查正常。",
    "extinguisher_missing": "警告，未检测到灭火器，请工作人员检查。",
    "safety_sign_ok": "安全出口指示灯检查正常。",
    "safety_sign_missing": "警告，未检测到安全出口指示灯，请工作人员检查。",
    "smoking": "警告，发现吸烟行为，请立即停止吸烟。",
    "done": "本次消防巡检已完成。",
}


def _field(d: Any, name: str) -> Any:
    """读取检出项字段：兼容 dict（detections_json 解析结果）与属性对象。

    冻结接口以 detections_json 字符串携带检出，解析后为 dict 列表；
    原实现按属性访问。此适配不改变任何判定逻辑。
    """
    if isinstance(d, dict):
        return d.get(name)
    return getattr(d, name, None)


def _cn(name: str, class_cn: dict) -> str:
    return class_cn.get(name, name)


def _strong_detections(result: Any, min_conf: float,
                       conf_by_class: Optional[dict]) -> list:
    """按与判定层相同的阈值语义过滤达标检出。

    安全处理 detections=None、元素字段缺失或置信度非数值等异常输入，
    绝不抛出 TypeError / AttributeError。
    """
    out = []
    dets = getattr(result, "detections", None)
    if not dets:
        return out
    for d in dets:
        cls = _field(d, "class_name")
        if not cls:
            continue
        conf = _field(d, "confidence")
        if conf is None:
            continue
        try:
            thr = (conf_by_class or {}).get(cls, min_conf)
            if float(conf) >= float(thr):
                out.append(d)
        except (TypeError, ValueError):
            continue
    return out


def _uncertain_text(name: str, found_en: set, focus: tuple,
                    expect_classes: Sequence[str],
                    cn: dict) -> str:
    """生成 UNCERTAIN 风格的人工复核提示（内部辅助）。

    接收已达标的 found_en（英文类名集合），计算缺失目标并生成文本。
    供 status=UNCERTAIN 分支和 NORMAL 分支校验失败时复用，
    避免两处复制相同的缺失计算逻辑。
    """
    target_classes = focus or tuple(expect_classes)
    missing = [_cn(c, cn) for c in target_classes if c not in found_en]
    if missing:
        return (f"{name}检查结果不确定，未能确认"
                f"{'、'.join(missing)}，请人工复核")
    return f"{name}检查结果不确定，请人工复核"


def build_speech_text(poi_name: str, result: Any,
                      expect_classes: Sequence[str] = (),
                      abnormal_classes: Sequence[str] = (),
                      min_conf: float = 0.5,
                      conf_by_class: Optional[dict] = None,
                      class_cn: Optional[dict] = None,
                      focus_classes: Sequence[str] = ()) -> str:
    """生成单个点位的播报文本。

    result 需带有 status（uint8，取值见 PoiStatus）与 detections
    （dict 列表或等价属性对象）两个字段。

    阈值参数应与视觉判定层实际使用的取值保持一致，否则
    文案中引用的"达标检出"会与判定结论不一致。

    focus_classes 是调用方显式指定的"本检查点目标类别"（如某个点位
    专查 safety_sign）。为空时沿用原有通用文案，绝不按 POI 名称猜测；
    专用点位只播该点位的专用文案，不会把专用结论扩散到其他点位。
    """
    cn = class_cn or CLASS_CN
    name = poi_name or "当前点位"
    status = result.status
    focus = tuple(focus_classes)

    if status == PoiStatus.NORMAL:
        # 先安全过滤达标检出：None / 字段缺失 / 非法置信度已在 _strong_detections 内处理
        strong = _strong_detections(result, min_conf, conf_by_class)
        found_en = {_field(d, "class_name") for d in strong}  # 英文类名集合

        # ── 三个专用 / 固定正常播报分支都必须校验 detections 证据 ──
        # 禁止仅凭 status=NORMAL 就播报"正常"；上游视觉判定已足够保守，
        # 此处防御性校验用来阻断检测结果与状态矛盾时的假阳性播报。

        if "safety_sign" in focus:
            if "safety_sign" in found_en:
                return SPEECH_TEXTS["safety_sign_ok"]
            # focus 指定了安全指示牌但 detections 无法证明其存在 → 降级为不确定
            return _uncertain_text(name, found_en, focus, expect_classes, cn)

        if "fire_extinguisher" in focus:
            if "fire_extinguisher" in found_en:
                return SPEECH_TEXTS["extinguisher_ok"]
            return _uncertain_text(name, found_en, focus, expect_classes, cn)

        if not focus and "fire_extinguisher" in expect_classes:
            if "fire_extinguisher" in found_en:
                return SPEECH_TEXTS["extinguisher_ok"]
            # 视觉判 NORMAL 但 detections 里没有灭火器 → 降级为不确定
            return _uncertain_text(name, found_en, focus, expect_classes, cn)

        # 通用分支：fall through 到此处说明三个 early-return 分支都未命中或校验失败
        found_cn = {_cn(c, cn) for c in found_en}
        target_classes = focus or tuple(expect_classes)
        items = [_cn(c, cn) for c in target_classes]

        # items 非空且全部达标 → 正常文本（已有 detections 校验）
        if items and all(i in found_cn for i in items):
            return f"{name}检查完成，{'、'.join(items)}均在位，未发现异常"

        # items 非空但未全部达标 → 降级为不确定（防御纵深：目标未能确认，不能仅凭 status=NORMAL 就说"未发现异常"）
        if items:
            return _uncertain_text(name, found_en, focus, expect_classes, cn)

        # items 空（focus 空 + expect 空）→ 无明确检查目标，保留通用文案
        return f"{name}检查完成，未发现异常"

    if status == PoiStatus.ABNORMAL:
        strong = _strong_detections(result, min_conf, conf_by_class)
        # 1) 异常类：吸烟用固定警告，其他异常类按类名播报
        present_abn = {_field(d, "class_name") for d in strong
                       if _field(d, "class_name") in abnormal_classes}
        parts = []
        for cls in sorted(present_abn):
            if cls == "smoking":
                parts.append(SPEECH_TEXTS["smoking"])
            else:
                parts.append(
                    f"警告，{name}发现{_cn(cls, cn)}，请现场人员立即确认处理")
        # 2) 目标缺失：专用点位只看该点目标，通用点位沿用期望类别
        found = {_field(d, "class_name") for d in strong}
        for cls in (focus or tuple(expect_classes)):
            if cls in found:
                continue
            if cls == "fire_extinguisher":
                parts.append(SPEECH_TEXTS["extinguisher_missing"])
            elif cls == "safety_sign" and "safety_sign" in focus:
                # 仅专用检查点播报，避免普通点位凭空出现安全出口结论
                parts.append(SPEECH_TEXTS["safety_sign_missing"])
        if parts:
            return "".join(parts)
        # 判定为异常但阈值口径对不上时，不编造具体类别
        return f"警告，{name}发现异常，请现场人员立即确认处理"

    if status == PoiStatus.UNCERTAIN:
        strong = _strong_detections(result, min_conf, conf_by_class)
        found_en = {_field(d, "class_name") for d in strong}
        return _uncertain_text(name, found_en, focus, expect_classes, cn)

    if status == PoiStatus.NO_FRAME:
        return f"{name}未能获取相机图像，检查未完成，请人工检查"
    if status == PoiStatus.TIMEOUT:
        return f"{name}视觉识别超时，检查未完成，请人工检查"
    if status == PoiStatus.NO_MODEL:
        return f"{name}视觉模型未配置，检查未完成，请人工检查"
    if status == PoiStatus.ERROR:
        return f"{name}视觉识别出现异常，检查未完成，请人工检查"
    if status == STATUS_SKIPPED:
        # 冻结接口无此状态；原逻辑分支保留，STATUS_SKIPPED=None 时不可达
        return f"{name}检查已跳过"
    # 未知状态兜底：绝不报"正常"
    return f"{name}检查状态未知，请人工复核"


def parse_poi_focus(items: Sequence[str]) -> dict:
    """解析 --poi-focus "POI=CLASS[,CLASS...]" 为 {POI: (CLASS, ...)}。

    只做显式配置解析，不按名称推断任何点位的检查目标；格式错误抛
    ValueError，由入口拒绝启动而不是静默降级。
    """
    out = {}
    for raw in items or ():
        item = str(raw).strip()
        if not item:
            continue
        if "=" not in item:
            raise ValueError(f"--poi-focus 需要 POI=CLASS 形式: {item!r}")
        poi, classes = item.split("=", 1)
        poi = poi.strip()
        cls = tuple(c for c in classes.replace(",", " ").split() if c)
        if not poi or not cls:
            raise ValueError(f"--poi-focus 缺少点位名或类别: {item!r}")
        out[poi] = cls
    return out
