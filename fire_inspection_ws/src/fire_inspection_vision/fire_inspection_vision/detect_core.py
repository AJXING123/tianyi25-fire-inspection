"""消防巡检检测核心：模型加载、推理、阈值过滤、结果判定。

从 tianyi_see/robot_test.py 抽取，去掉 GUI / CSV / 存图等调试逻辑，
只保留「一帧图像 -> 结构化检出 -> PoiResult 语义」这条主链。

阈值策略沿用现场实测结论：
  - 吸烟 0.60：过低（0.25/0.45）会把只框到裤腿、远处人体的误检当确定结论
  - 安全指示牌 / 灭火器 0.65：收紧阈值压制误检
  - 吸烟额外加框几何校验（见 SMOKING_GEOMETRY），作为阈值之外的第二道闸
"""

from __future__ import annotations

import json
import os
import threading
from concurrent.futures import ThreadPoolExecutor, TimeoutError as FuturesTimeout

# PoiResult.status 枚举（与 fire_inspection_interfaces/msg/PoiResult.msg 一致）
STATUS_NORMAL = 0
STATUS_ABNORMAL = 1
STATUS_UNCERTAIN = 2
STATUS_TIMEOUT = 3
STATUS_NO_FRAME = 4
STATUS_NO_MODEL = 5
STATUS_ERROR = 6

STATUS_CN = {
    STATUS_NORMAL: "正常",
    STATUS_ABNORMAL: "异常",
    STATUS_UNCERTAIN: "不确定",
    STATUS_TIMEOUT: "超时",
    STATUS_NO_FRAME: "无图像",
    STATUS_NO_MODEL: "无模型",
    STATUS_ERROR: "错误",
}

# 配置里的别名统一映射到模型输出的英文类名
ALIASES = {
    "fire": "fire_extinguisher",
    "extinguisher": "fire_extinguisher",
    "fire_extinguisher": "fire_extinguisher",
    "smoke": "smoking",
    "smoking": "smoking",
    "sign": "safety_sign",
    "safety_sign": "safety_sign",
}


def canonical(name: str) -> str:
    key = str(name).strip().lower()
    return ALIASES.get(key, key)


# 吸烟框几何下限（归一化到整幅画面）：只框到裤腿/手等局部、或极小的远处
# 目标，几何上就不像一次完整的吸烟行为，直接丢弃。阈值挡不住的边缘误检
# （如 0.52 的裤腿框）在这里再压一层；现场可按需在 vision.yaml 覆盖。
SMOKING_GEOMETRY = {
    "min_w": 0.03,       # 框宽下限
    "min_h": 0.10,       # 框高下限
    "min_area": 0.01,    # 框面积下限（w*h）
    "min_aspect": 0.20,  # 宽/高 下限
    "max_aspect": 1.80,  # 宽/高 上限
}


def smoking_geometry_ok(det: dict, limits: dict | None = None) -> bool:
    """吸烟框的几何合理性校验；字段缺失/非法一律按不合理处理。"""
    cfg = dict(SMOKING_GEOMETRY)
    if isinstance(limits, dict):
        for key in cfg:
            if key in limits:
                try:
                    cfg[key] = float(limits[key])
                except (TypeError, ValueError):
                    return False
    try:
        width = float(det["w"])
        height = float(det["h"])
    except (KeyError, TypeError, ValueError):
        return False
    if width <= 0.0 or height <= 0.0:
        return False
    if (
        width < cfg["min_w"]
        or height < cfg["min_h"]
        or width * height < cfg["min_area"]
    ):
        return False
    aspect = width / height
    return cfg["min_aspect"] <= aspect <= cfg["max_aspect"]


class Detector:
    """YOLO 检测器：懒加载模型，单线程串行推理，超时即弃用实例。"""

    def __init__(self, model_path, imgsz=1280, conf_smoking=0.60,
                 conf_other=0.65, device="0", smoking_geometry=None):
        self.model_path = str(model_path or "")
        self.imgsz = int(imgsz)
        self.conf_smoking = float(conf_smoking)
        self.conf_other = float(conf_other)
        self.device = str(device)
        self.smoking_geometry = (
            dict(smoking_geometry) if isinstance(smoking_geometry, dict) else None)
        self._model = None
        self.names = {}
        self._err = ""
        self._lock = threading.Lock()
        self._pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="yolo")

    # ---------- 模型 ----------
    def ensure(self):
        """按需加载模型，返回 (ok, err)。加载失败会被记住，不重复尝试。"""
        with self._lock:
            if self._model is not None:
                return True, ""
            if self._err:
                return False, self._err
            if not self.model_path or not os.path.isfile(self.model_path):
                self._err = f"模型文件不存在: {self.model_path!r}"
                return False, self._err
            try:
                from ultralytics import YOLO
            except ImportError as exc:
                self._err = f"ultralytics 未安装: {exc}"
                return False, self._err
            try:
                self._model = YOLO(self.model_path)
                self.names = {int(k): str(v) for k, v in self._model.names.items()}
            except Exception as exc:  # 模型损坏 / ABI 不匹配等
                self._err = f"模型加载失败: {exc}"
                return False, self._err
            return True, ""

    # ---------- 推理 ----------
    def infer(self, frame, timeout_s=None):
        """返回 (dets, err, status)。dets 为 None 表示失败，status 为 None 表示成功。"""
        ok, err = self.ensure()
        if not ok:
            return None, err, STATUS_NO_MODEL

        infer_conf = min(self.conf_smoking, self.conf_other)
        future = self._pool.submit(self._predict, frame, infer_conf)
        try:
            dets = future.result(timeout=timeout_s) if timeout_s else future.result()
            return dets, "", None
        except FuturesTimeout:
            # 推理卡死：弃用当前模型实例，下次重新加载
            with self._lock:
                self._model = None
                self._err = f"推理超时（>{timeout_s:.1f}s），已弃用模型实例"
            return None, self._err, STATUS_TIMEOUT
        except Exception as exc:
            return None, f"推理失败: {exc}", STATUS_ERROR

    def _predict(self, frame, infer_conf):
        result = self._model.predict(
            frame, imgsz=self.imgsz, conf=infer_conf,
            device=self.device, verbose=False,
        )[0]
        h, w = frame.shape[:2]
        dets = []
        for box in result.boxes:
            cls_id = int(box.cls)
            name = self.names.get(cls_id, str(cls_id))
            x1, y1, x2, y2 = [float(v) for v in box.xyxy[0]]
            dets.append({
                "cls": name,
                "conf": float(box.conf),
                # 归一化坐标（契约 §6.1）：cx/cy 为中心点，w/h 为宽高，均除以整幅画面
                "cx": (x1 + x2) / 2.0 / w,
                "cy": (y1 + y2) / 2.0 / h,
                "w": (x2 - x1) / w,
                "h": (y2 - y1) / h,
                "xyxy": [int(x1), int(y1), int(x2), int(y2)],
            })
        return dets

    # ---------- 判定 ----------
    def threshold_of(self, name):
        return self.conf_smoking if canonical(name) == "smoking" else self.conf_other

    def filter(self, dets):
        """按每类阈值过滤（推理用的统一低阈值，这里再按类别收紧）。

        吸烟额外做框几何校验：只框到裤腿/手等局部或极小的远处目标直接丢弃。
        """
        out = []
        for det in dets:
            if det["conf"] < self.threshold_of(det["cls"]):
                continue
            if (
                canonical(det["cls"]) == "smoking"
                and not smoking_geometry_ok(det, self.smoking_geometry)
            ):
                continue
            out.append(det)
        return out

    def judge(self, dets, expect_classes, abnormal_classes):
        """返回 (status, detail)。结果保守：不确定绝不判正常。"""
        names = {canonical(d["cls"]) for d in dets}
        expect = {canonical(e) for e in expect_classes}
        abnormal = {canonical(a) for a in abnormal_classes}

        hits = sorted(names & abnormal)
        if hits:
            return STATUS_ABNORMAL, "检出异常类别: " + ", ".join(hits)

        if expect:
            missing = sorted(expect - names)
            if missing:
                return STATUS_UNCERTAIN, "期望目标未全部检出，缺: " + ", ".join(missing)
            return STATUS_NORMAL, "期望目标全部检出"

        if abnormal:
            return STATUS_NORMAL, "未检出异常类别"
        if names:
            return STATUS_UNCERTAIN, "有检出但未配置判定规则"
        return STATUS_UNCERTAIN, "画面内无检出"

    @staticmethod
    def to_json(dets, limit=20):
        """按契约 §6.1 序列化：cls/conf/cx/cy/w/h，按 conf 降序，最多 limit 条。

        range_m / bearing_rad / range_valid / look_pitch_rad 为可选扩展字段
        （吸烟人员介入用），由持续检测节点按深度与头部姿态补齐；
        缺失时不写，保持旧 schema 兼容。
        """
        ordered = sorted(dets, key=lambda d: d["conf"], reverse=True)[:limit]
        items = []
        for d in ordered:
            item = {
                "cls": d["cls"], "conf": round(d["conf"], 3),
                "cx": round(d["cx"], 4), "cy": round(d["cy"], 4),
                "w": round(d["w"], 4), "h": round(d["h"], 4),
            }
            if "range_m" in d:
                item["range_m"] = round(float(d["range_m"]), 3)
            if "bearing_rad" in d:
                item["bearing_rad"] = round(float(d["bearing_rad"]), 4)
            if "look_pitch_rad" in d:
                item["look_pitch_rad"] = round(float(d["look_pitch_rad"]), 4)
            if "range_valid" in d:
                item["range_valid"] = bool(d["range_valid"])
            items.append(item)
        return json.dumps(items, ensure_ascii=False, separators=(",", ":"))
