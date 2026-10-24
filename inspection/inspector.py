"""视觉巡检适配层。

真实模式：robot.camera_get_rgb_depth(...) 取图 + YoloLoad.detect(frame) 推理。
模拟模式：SimInspector 按剧本返回，detections 一律 simulated=True 且类名
带 sim_ 前缀，绝不允许伪装成真实识别结果。

判定规则（保守，宁可不确定也不误报正常）：
- 检出 abnormal_classes 中任一类（如 smoking）且置信度达标 → ABNORMAL
- 期望目标（expect_classes）全部检出且置信度达标 → SUCCESS
- 有检出但置信度不足 / 期望目标缺失 → UNCERTAIN（不是"正常"）
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
from typing import Optional, Sequence

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
                detections=[det("sim_smoking", 0.91)],
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
                 min_conf: float = 0.5):
        if not model_path:
            raise ValueError(
                "YoloInspector 需要真实模型路径；模型未到位请用 SimInspector")
        from utils.camera_detect.yolo_load import YoloLoad  # 延迟导入

        self._robot = robot
        self._yolo = YoloLoad(model_path=model_path, confidence=confidence,
                              auto_load=True)  # 文件不存在时 validate 抛错
        self._expect = tuple(expect_classes)
        self._abnormal = tuple(abnormal_classes)
        self._camera_name = camera_name
        self._frame_timeout = frame_timeout
        self._inference_timeout = float(inference_timeout)
        self._min_conf = min_conf
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

        strong = [d for d in detections if d.confidence >= self._min_conf]
        hit_abnormal = [d for d in strong if d.class_name in self._abnormal]
        if hit_abnormal:
            return PoiInspection(
                status=InspectionStatus.ABNORMAL, poi=poi_name,
                detections=detections,
                detail="检出异常: " + ",".join(d.class_name for d in hit_abnormal),
                inference_ms=ms)

        found = {d.class_name for d in strong}
        missing = [c for c in self._expect if c not in found]
        if missing:
            return PoiInspection(
                status=InspectionStatus.UNCERTAIN, poi=poi_name,
                detections=detections,
                detail=f"期望目标缺失或置信度不足: {missing}",
                inference_ms=ms)
        return PoiInspection(status=InspectionStatus.SUCCESS, poi=poi_name,
                             detections=detections, detail="期望目标均在位",
                             inference_ms=infer_ms)
