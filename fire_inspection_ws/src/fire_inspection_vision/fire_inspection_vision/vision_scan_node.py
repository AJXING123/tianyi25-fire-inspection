"""消防巡检常驻扫描节点：持续订阅相机 -> 推理 -> 发布检测结果。

话题：/fire_inspection/detection  fire_inspection_interfaces/msg/PoiResult
  QoS：best_effort / depth 1（只要最新结果，允许丢帧）
  detections_json 按契约 §6.1：cls/conf/cx/cy/w/h（归一化，按 conf 降序，最多 20 条）
  status：0 无检出 / 1 有检出；simulated=true 时 detections_json 恒为 []

设计要点：
- 推理在独立线程里跑，不阻塞订阅回调；
- 只处理「最新一帧」，不排队，避免积压导致结果延迟；
- 两次推理之间按 scan_rate_hz 限速，避免长时间占满 GPU。
"""

from __future__ import annotations

import os
import threading

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import CompressedImage

from fire_inspection_interfaces.msg import PoiResult
from fire_inspection_vision.detect_core import (STATUS_ABNORMAL, STATUS_NO_FRAME,
                                               STATUS_NORMAL, Detector,
                                               canonical)


def load_config():
    share = get_package_share_directory("fire_inspection_vision")
    path = os.path.join(share, "config", "vision.yaml")
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def sensor_qos():
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        durability=DurabilityPolicy.VOLATILE,
    )


class VisionScanNode(Node):
    def __init__(self):
        super().__init__("vision_scan")
        cfg = load_config()
        scan = cfg.get("scan") or {}

        self.declare_parameter("dry_run", bool(cfg.get("dry_run", False)))
        self.dry_run = bool(self.get_parameter("dry_run").value)

        topic = cfg.get("topic", "/ob_camera_head/color/image_raw/compressed")
        out_topic = scan.get("topic", "/fire_inspection/detection")
        self.rate = max(0.2, float(scan.get("rate_hz", 2.0)))
        self.interval = 1.0 / self.rate
        self.infer_timeout = float((cfg.get("timeouts") or {}).get(
            "inference_timeout_s", 5.0))

        th = cfg.get("thresholds") or {}
        self.detector = Detector(
            model_path=cfg.get("model_path", ""),
            imgsz=int(cfg.get("imgsz", 1280)),
            conf_smoking=float(th.get("conf_smoking", 0.25)),
            conf_other=float(th.get("conf_other", 0.65)),
            device=str(cfg.get("device", "0")),
        )

        self._pub = self.create_publisher(PoiResult, out_topic, sensor_qos())
        self.create_subscription(CompressedImage, topic, self._on_image,
                                 sensor_qos())

        self._frame = None
        self._seq = 0
        self._lock = threading.Lock()
        self._new_frame = threading.Event()
        self._stop = threading.Event()
        self._worker = threading.Thread(target=self._loop, name="scan_infer",
                                        daemon=True)
        self._worker.start()

        self.get_logger().info(
            f"常驻扫描就绪 相机={topic} 发布={out_topic} "
            f"{self.rate:.1f}Hz dry_run={self.dry_run}")

    # ---------- 相机 ----------
    def _on_image(self, msg):
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8),
                             cv2.IMREAD_COLOR)
        if frame is None:
            return
        with self._lock:
            self._frame = frame
            self._seq += 1
        self._new_frame.set()

    # ---------- 推理线程 ----------
    def _loop(self):
        while rclpy.ok() and not self._stop.is_set():
            if not self._new_frame.wait(0.5):
                continue
            self._new_frame.clear()

            if self.dry_run:
                self._publish(STATUS_NORMAL, True, "dry_run 未推理", "[]")
                self._stop.wait(self.interval)
                continue

            with self._lock:
                frame = self._frame
            if frame is None:
                self._stop.wait(self.interval)
                continue

            dets, err, fail_status = self.detector.infer(frame, self.infer_timeout)
            if dets is None:
                status = fail_status if fail_status is not None else STATUS_NO_FRAME
                self._publish(status, False, err or "推理失败", "[]")
                self._stop.wait(self.interval)
                continue

            kept = self.detector.filter(dets)
            names = sorted({canonical(d["cls"]) for d in kept})
            if kept:
                detail = "检出: " + ", ".join(names)
                status = STATUS_ABNORMAL
            else:
                detail = "无检出"
                status = STATUS_NORMAL
            self._publish(status, False, detail, self.detector.to_json(kept))
            self._stop.wait(self.interval)

    def _publish(self, status, simulated, detail, dets_json):
        msg = PoiResult()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "head"
        msg.poi = "scan"
        msg.status = int(status)
        msg.simulated = bool(simulated)
        msg.detail = detail
        msg.detections_json = dets_json
        self._pub.publish(msg)

    def shutdown(self):
        self._stop.set()
        self._new_frame.set()
        self._worker.join(timeout=3.0)


def main(argv=None):
    rclpy.init(args=argv)
    node = VisionScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
