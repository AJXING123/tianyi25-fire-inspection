"""视觉巡检适配层。

真实模式：robot.camera_get_rgb_depth(...) 取图 + YoloLoad.detect(frame) 推理。
模拟模式：SimInspector 按剧本返回，detections 一律 simulated=True 且类名
带 sim_ 前缀，绝不允许伪装成真实识别结果。

判定规则（保守，宁可不确定也不误报正常）：
- 检出 abnormal_classes 中任一类（如 smoking）且置信度达标 → ABNORMAL
- 期望目标（expect_classes）全部检出且置信度达标 → SUCCESS
- absence_abnormal（默认仅 fire_extinguisher）中的类别缺失 → ABNORMAL：
  灭火器"至少一个有效检出即正常"，不设数量上限；0 个有效检出
  （含仅剩低于阈值的框）判定异常，低于阈值的框不计为检出成功
- 其余期望目标缺失 → UNCERTAIN（不是"正常"）
- 取图超时 / 推理超时 → 对应失败态（TIMEOUT / NO_FRAME）

推理超时诚实声明（重要）：
Python 无法安全强杀线程，因此推理超时是"软超时"——超时后后台线程
可能仍在 GPU 上执行；本实现将超时实例标记为不可用（poisoned），
后续检测直接返回 ERROR，避免同一模型被多线程并发访问。
守护线程不阻止进程退出，但若线程挂在 CUDA 内核中，进程退出仍可能
等待驱动清理——这是无法从 Python 层消除的限制。
"""

from __future__ import annotations

import queue
import threading
import time
from typing import Mapping, Optional, Sequence

from .types import Detection, InspectionStatus, PoiInspection


class InspectorBase:
    def inspect(self, poi_name: str) -> PoiInspection:
        raise NotImplementedError

    def close(self) -> None:
        pass


# ── 模拟实现 ────────────────────────────────────────────────────────

class SimInspector(InspectorBase):
    """离线模拟视觉检查。

    mode: normal / abnormal / uncertain / timeout / no_frame
    """

    def __init__(self, mode: str = "normal", latency_s: float = 0.02,
                 expect_classes: Sequence[str] = ("sim_fire_extinguisher",)):
        self.mode = mode
        self.latency_s = latency_s
        self.expect_classes = tuple(expect_classes)
        self.calls: list = []

    def inspect(self, poi_name: str) -> PoiInspection:
        self.calls.append(poi_name)
        t0 = time.monotonic()
        time.sleep(self.latency_s)
        ms = (time.monotonic() - t0) * 1000.0

        def det(name: str, conf: float) -> Detection:
            return Detection(class_name=name, confidence=conf,
                             xyxy=[0, 0, 10, 10], simulated=True)

        if self.mode == "abnormal":
            return PoiInspection(
                status=InspectionStatus.ABNORMAL, poi=poi_name,
                detections=[det("smoking", 0.91)],
                detail="simulated abnormal", inference_ms=ms, simulated=True)
        if self.mode == "uncertain":
            return PoiInspection(
                status=InspectionStatus.UNCERTAIN, poi=poi_name,
                detections=[det("sim_fire_extinguisher", 0.31)],
                detail="simulated low confidence", inference_ms=ms,
                simulated=True)
        if self.mode == "timeout":
            return PoiInspection(
                status=InspectionStatus.TIMEOUT, poi=poi_name,
                detail="simulated timeout", inference_ms=ms, simulated=True)
        if self.mode == "no_frame":
            return PoiInspection(
                status=InspectionStatus.NO_FRAME, poi=poi_name,
                detail="simulated no frame", inference_ms=ms, simulated=True)
        return PoiInspection(
            status=InspectionStatus.SUCCESS, poi=poi_name,
            detections=[det(c, 0.95) for c in self.expect_classes],
            detail="simulated normal", inference_ms=ms, simulated=True)


# ── 真实实现 ────────────────────────────────────────────────────────

class YoloInspector(InspectorBase):
    """YoloLoad + 头部相机的真实检查。

    model_path 必须由真实配置/队友资料给出；缺文件直接 FileNotFoundError，
    绝不回退到虚构模型。abnormal_classes / expect_classes 同样必须来自
    队友确认的类别清单。
    """

    def __init__(self, robot, model_path: str,
                 expect_classes: Sequence[str],
                 abnormal_classes: Sequence[str] = (),
                 confidence: float = 0.5,
                 camera_name: str = "head",
                 frame_timeout: float = 3.0,
                 inference_timeout: float = 10.0,
                 min_conf: float = 0.5,
                 conf_by_class: Optional[dict] = None,
                 to_bgr: bool = False,
                 absence_abnormal: Sequence[str] = ("fire_extinguisher",),
                 poi_focus_classes: Optional[Mapping[str, Sequence[str]]] = None):
        if not model_path:
            raise ValueError(
                "YoloInspector 需要真实模型路径；模型未到位请用 SimInspector")
        from utils.camera_detect.yolo_load import YoloLoad  # 延迟导入

        self._robot = robot
        # 推理阈值取所有判定阈值的最低档，避免低阈值目标在模型侧被丢弃；
        # 逐类过滤在判定阶段按 conf_by_class 执行（对齐队友双阈值语义：
        # smoking 0.25 / 其他 0.65）。conf_by_class 缺省时行为与原来一致。
        self._class_conf = dict(conf_by_class) if conf_by_class else None
        if self._class_conf:
            infer_conf = min(list(self._class_conf.values()) + [min_conf])
        else:
            infer_conf = confidence
        self._yolo = YoloLoad(model_path=model_path, confidence=infer_conf,
                              auto_load=True)  # 文件不存在时 validate 抛错
        self._expect = tuple(expect_classes)
        self._abnormal = tuple(abnormal_classes)
        # 这些类别"至少一个有效检出即正常、缺失即异常"（默认灭火器）
        self._absence_abnormal = tuple(absence_abnormal)
        # 显式配置的"点位 → 专用检查目标"；未配置的点位沿用全局规则。
        # 禁止按 POI 名称猜测，映射只能由调用方明确给出。
        self._poi_focus = {str(k): tuple(v)
                           for k, v in (poi_focus_classes or {}).items()}
        self._camera_name = camera_name
        self._frame_timeout = frame_timeout
        self._inference_timeout = float(inference_timeout)
        self._min_conf = min_conf
        # robot_controller 出图为 RGB 序，ultralytics 对 numpy 输入按 BGR
        # 处理且队友模型按 BGR 帧训练；to_bgr=True 时在喂模型前翻转通道。
        self._to_bgr = bool(to_bgr)
        # 推理软超时后置为 True：不再信任该实例，后续直接返回 ERROR
        self._poisoned = False

    # ── 推理软超时包装 ─────────────────────────────────────────────

    def _detect_with_timeout(self, rgb):
        """在守护线程中执行推理，返回 (outcome, payload)。

        outcome: "ok" / "error" / "timeout"。
        超时返回后，后台线程可能仍在运行（无法强杀），因此调用方
        必须将本实例 poisoned，禁止再次复用同一模型。
        """
        q = queue.Queue(maxsize=1)

        def _work():
            try:
                q.put(("ok", self._yolo.detect(rgb)))
            except Exception as exc:
                q.put(("error", exc))

        t = threading.Thread(target=_work, name="yolo-detect", daemon=True)
        t.start()
        try:
            return q.get(timeout=self._inference_timeout)
        except queue.Empty:
            return ("timeout", None)

    def _conf_for(self, class_name: str) -> float:
        """该类别的判定阈值：conf_by_class 未列出时回退 min_conf。"""
        if self._class_conf:
            return self._class_conf.get(class_name, self._min_conf)
        return self._min_conf

    def _focus_classes_for(self, poi_name: str) -> tuple:
        """本点位显式配置的专用检查目标；未配置返回空（不猜名称）。"""
        return self._poi_focus.get(poi_name, ())

    # ── 判定参数只读暴露（供播报层按同一阈值口径生成文案）─────────

    @property
    def expect_classes(self) -> tuple:
        return self._expect

    @property
    def abnormal_classes(self) -> tuple:
        return self._abnormal

    @property
    def min_conf(self) -> float:
        return self._min_conf

    @property
    def conf_by_class(self) -> Optional[dict]:
        return dict(self._class_conf) if self._class_conf else None

    # ── 主入口 ─────────────────────────────────────────────────────

    def inspect(self, poi_name: str) -> PoiInspection:
        if self._poisoned:
            return PoiInspection(
                status=InspectionStatus.ERROR, poi=poi_name,
                detail="推理曾超时，视觉检测已禁用（保守策略，"
                       "避免并发访问同一模型）")

        t0 = time.monotonic()

        # 1) 取图：区分超时与通用异常，分别记录
        try:
            frame = self._robot.camera_get_rgb_depth(
                timeout=self._frame_timeout, require_new=True,
                camera_name=self._camera_name)
        except TimeoutError as exc:
            return PoiInspection(
                status=InspectionStatus.NO_FRAME, poi=poi_name,
                detail=f"取图超时（>{self._frame_timeout}s）: {exc}")
        except Exception as exc:
            return PoiInspection(
                status=InspectionStatus.NO_FRAME, poi=poi_name,
                detail=f"取图异常: {exc}")

        rgb = frame.get("rgb") if isinstance(frame, dict) else None
        if rgb is None:
            return PoiInspection(
                status=InspectionStatus.NO_FRAME, poi=poi_name,
                detail="camera_get_rgb_depth 返回无 rgb")
        if self._to_bgr:
            import numpy as np
            rgb = np.ascontiguousarray(rgb[..., ::-1])  # RGB -> BGR

        # 2) 推理：软超时包装
        outcome, payload = self._detect_with_timeout(rgb)
        ms = (time.monotonic() - t0) * 1000.0
        if outcome == "timeout":
            self._poisoned = True
            return PoiInspection(
                status=InspectionStatus.TIMEOUT, poi=poi_name,
                detail=f"推理超过 {self._inference_timeout}s 未返回"
                       f"（软超时，后台线程可能仍在运行，后续视觉已禁用）",
                inference_ms=ms)
        if outcome == "error":
            return PoiInspection(
                status=InspectionStatus.ERROR, poi=poi_name,
                detail=f"推理异常: {payload}", inference_ms=ms)

        rv = payload
        infer_ms = float(rv.get("inference_ms", ms)) if isinstance(rv, dict) else ms

        detections = []
        for d in (rv.get("detections") or []):
            detections.append(Detection(
                class_name=str(d.get("class_name")),
                confidence=float(d.get("confidence", 0.0)),
                xyxy=d.get("xyxy"),
                simulated=False,
            ))

        strong = [d for d in detections if d.confidence >= self._conf_for(d.class_name)]
        hit_abnormal = [d for d in strong if d.class_name in self._abnormal]
        if hit_abnormal:
            return PoiInspection(
                status=InspectionStatus.ABNORMAL, poi=poi_name,
                detections=detections,
                detail="检出异常: " + ",".join(d.class_name for d in hit_abnormal),
                inference_ms=ms)

        # 期望目标按"至少一个有效检出"判定：found 是类名集合，
        # 同一目标的重复框天然去重，且不设数量上限（1/2/3/N 个都算在位）。
        # 显式配置了专用目标的点位只按该目标判定，避免全局 expect
        # 把其他点位的目标混进来。
        focus = self._focus_classes_for(poi_name)
        targets = focus if focus else self._expect
        found = {d.class_name for d in strong}
        missing = [c for c in targets if c not in found]
        if missing:
            # 专用点位目标缺失即异常；非专用点沿用原 absence_abnormal 规则
            critical = list(missing) if focus else [
                c for c in missing if c in self._absence_abnormal]
            if critical:
                # 目标缺失 = 异常（不是"不确定"）。低于阈值的框不计成功，
                # 但仍写进 detail 便于日志/记录区分"没看到"与"看到了但不够可信"。
                rejected = [d for d in detections
                            if d.class_name in critical
                            and d.confidence < self._conf_for(d.class_name)]
                if rejected:
                    top = max(d.confidence for d in rejected)
                    why = (f"仅检出低于阈值的框，最高 {top:.2f} < "
                           f"{self._conf_for(rejected[0].class_name):.2f}")
                else:
                    why = "无检出"
                return PoiInspection(
                    status=InspectionStatus.ABNORMAL, poi=poi_name,
                    detections=detections,
                    detail=f"未检出有效目标 {critical}（{why}）",
                    inference_ms=ms)
            return PoiInspection(
                status=InspectionStatus.UNCERTAIN, poi=poi_name,
                detections=detections,
                detail=f"期望目标缺失或置信度不足: {missing}",
                inference_ms=ms)
        return PoiInspection(status=InspectionStatus.SUCCESS, poi=poi_name,
                             detections=detections, detail="期望目标均在位",
                             inference_ms=infer_ms)
