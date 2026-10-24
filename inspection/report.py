"""视觉结果 → 中文播报文案映射层。

职责单一：把 PoiInspection 结论转成准确、保守的中文播报文本。
不做检测、不调语音、不写盘，纯函数便于离线测试。

播报红线（与项目安全约定一致）：
- 检出 smoking 只播"发现吸烟行为"，绝不播报"发生火灾"——
  目标检出 ≠ 火灾确认，是否起火必须由人工现场判断；
- UNCERTAIN / TIMEOUT / NO_FRAME / ERROR / NO_MODEL 一律不播报"正常"，
  统一引导人工复核；
- 文案只引用实际检出且过阈值的类别，不编造未检出的内容。

类别中文名对齐队友 robot_test.py 的 CLASS_CN（三分类模型
runs/train_3cls_v2/weights/best.pt，本仓库 models/fire/best.pt）。
"""

from __future__ import annotations

from typing import Optional, Sequence

from .types import InspectionStatus, PoiInspection

# 与队友 robot_test.py CLASS_CN 一致；未知类别原样保留英文名
CLASS_CN = {
    "smoking": "吸烟行为",
    "safety_sign": "安全指示牌",
    "fire_extinguisher": "灭火器",
}


def _cn(name: str, class_cn: dict) -> str:
    return class_cn.get(name, name)


def _strong_detections(result: PoiInspection, min_conf: float,
                       conf_by_class: Optional[dict]) -> list:
    """按与 YoloInspector 相同的阈值语义过滤达标检出。"""
    out = []
    for d in result.detections:
        thr = (conf_by_class or {}).get(d.class_name, min_conf)
        if d.confidence >= thr:
            out.append(d)
    return out


def build_speech_text(poi_name: str, result: PoiInspection,
                      expect_classes: Sequence[str] = (),
                      abnormal_classes: Sequence[str] = (),
                      min_conf: float = 0.5,
                      conf_by_class: Optional[dict] = None,
                      class_cn: Optional[dict] = None) -> str:
    """生成单个点位的播报文本。

    阈值参数应与 YoloInspector 实际使用的取值保持一致，否则
    文案中引用的"达标检出"会与判定结论不一致。
    """
    cn = class_cn or CLASS_CN
    name = poi_name or "当前点位"
    status = result.status

    if status is InspectionStatus.SUCCESS:
        strong = _strong_detections(result, min_conf, conf_by_class)
        found = {_cn(d.class_name, cn) for d in strong}
        items = [_cn(c, cn) for c in expect_classes]
        if items and all(i in found for i in items):
            return f"{name}检查完成，{'、'.join(items)}均在位，未发现异常"
        return f"{name}检查完成，未发现异常"

    if status is InspectionStatus.ABNORMAL:
        strong = _strong_detections(result, min_conf, conf_by_class)
        abn = {_cn(d.class_name, cn) for d in strong
               if d.class_name in abnormal_classes}
        if abn:
            return (f"警告，{name}发现{'、'.join(sorted(abn))}，"
                    f"请现场人员立即确认处理")
        # 判定为异常但阈值口径对不上时，不编造具体类别
        return f"警告，{name}发现异常，请现场人员立即确认处理"

    if status is InspectionStatus.UNCERTAIN:
        strong = _strong_detections(result, min_conf, conf_by_class)
        found = {d.class_name for d in strong}
        missing = [_cn(c, cn) for c in expect_classes if c not in found]
        if missing:
            return (f"{name}检查结果不确定，未能确认"
                    f"{'、'.join(missing)}，请人工复核")
        return f"{name}检查结果不确定，请人工复核"

    if status is InspectionStatus.NO_FRAME:
        return f"{name}未能获取相机图像，检查未完成，请人工检查"
    if status is InspectionStatus.TIMEOUT:
        return f"{name}视觉识别超时，检查未完成，请人工检查"
    if status is InspectionStatus.NO_MODEL:
        return f"{name}视觉模型未配置，检查未完成，请人工检查"
    if status is InspectionStatus.ERROR:
        return f"{name}视觉识别出现异常，检查未完成，请人工检查"
    if status is InspectionStatus.SKIPPED:
        return f"{name}检查已跳过"
    # 未知状态兜底：绝不报"正常"
    return f"{name}检查状态未知，请人工复核"
