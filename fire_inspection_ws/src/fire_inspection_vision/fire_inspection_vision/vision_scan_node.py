"""T3 持续检测节点：订阅头部相机帧，持续推理并发 /fire_inspection/detection。

为编排层（orchestrator）提供实时检测流，使其能够基于「连续 N 帧同类命中」
做防误报确认，并在第一次命中时就锁定头部转向目标。

与 InspectPoi Action 的关系：
  - InspectPoi 是「单次点名检测」，供 orchestrator 在 NAV_TO_POI → INSPECT 阶段调用
  - vision_scan_node 是「持续检测流」，供 orchestrator 在 WATCH 阶段实时响应

配置来自 config/vision.yaml 的 scan 段：
  topic: /fire_inspection/detection   发布话题（契约 T3，best_effort / depth 1）
  rate_hz: 2.0                        推理限速，太低漏目标，太高占满 GPU

发布：fire_inspection_interfaces/msg/PoiResult
  poi: "scan"                         持续检测固定标记为 scan
  status: 0-6                         NORMAL/ABNORMAL/UNCERTAIN/...
  simulated: true = dry_run           编排层对 simulated=true 的结果一律忽略
  detections_json: cx/cy/w/h 归一化   契约 §6.1 schema，conf 降序最多 20 条
"""

from __future__ import annotations

import os
import time

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header

from fire_inspection_interfaces.msg import PoiResult
from fire_inspection_vision.detect_core import (Detector, STATUS_NO_FRAME,
                                                 STATUS_NO_MODEL, STATUS_ERROR)


def load_config():
    share = get_package_share_directory("fire_inspection_vision")
    path = os.path.join(share, "config", "vision.yaml")
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def detection_qos():
    """契约 T3：best_effort / depth 1，编排层订阅端必须一致。"""
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

        self.declare_parameter("dry_run", bool(cfg.get("dry_run", False)))
        self.dry_run = bool(self.get_parameter("dry_run").value)

        # 模型 / 阈值
        self.model_path = str(cfg.get("model_path", ""))
        th = cfg.get("thresholds") or {}
        self.detector = Detector(
            model_path=self.model_path,
            imgsz=int(cfg.get("imgsz", 1280)),
            conf_smoking=float(th.get("conf_smoking", 0.25)),
            conf_other=float(th.get("conf_other", 0.65)),
            device=str(cfg.get("device", "0")),
        )

        # 相机
        cam_topic = str(cfg.get("topic", "/ob_camera_head/color/image_raw/compressed"))
        self._latest_frame = None
        self._frame_lock = __import__("threading").Lock()

        # T3 发布
        scan = cfg.get("scan") or {}
        scan_topic = str(scan.get("topic", "/fire_inspection/detection"))
        self.rate_hz = float(scan.get("rate_hz", 2.0))
        self._period = 1.0 / max(0.5, self.rate_hz)
        self._pub = self.create_publisher(PoiResult, scan_topic, detection_qos())

        # 订阅相机（best_effort / depth 1，只留最新帧）
        self.create_subscription(CompressedImage, cam_topic,
                                 self._on_frame, detection_qos())

        # 定时推理
        self._timer = self.create_timer(self._period, self._scan_once)

        self.get_logger().info(
            f"持续检测节点就绪 topic={scan_topic} rate={self.rate_hz:.1f}Hz "
            f"相机={cam_topic} dry_run={self.dry_run} model={self.model_path}")

    # ---------- 相机帧 ----------
    def _on_frame(self, msg):
        try:
            buf = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        except Exception:
            return
        if img is None:
            return
        with self._frame_lock:
            self._latest_frame = img

    # ---------- 单次推理 ----------
    def _scan_once(self):
        frame = None
        with self._frame_lock:
            frame = self._latest_frame
        if frame is None:
            self._publish(STATUS_NO_FRAME, "无图像", "[]")
            return

        if self.dry_run:
            self._publish(2, "dry_run 模拟", "[]", simulated=True)
            return

        dets, err, status = self.detector.infer(frame, timeout_s=5.0)
        if dets is None:
            self._publish(status or STATUS_ERROR, err or "推理失败", "[]")
            return

        filtered = self.detector.filter(dets)
        detections_json = Detector.to_json(filtered)
        # 持续检测流不做判定（N/ABNORMAL/UNCERTAIN），只把检出列表交给编排层自己定夺
        # 编排层 WATCH/LOCKED 状态下只关心"有没有命中过目标"
        self._publish(2 if not filtered else 0,
                      f"检出 {len(filtered)} 个目标" if filtered else "无检出",
                      detections_json)

    def _publish(self, status, detail, detections_json, simulated=None):
        msg = PoiResult()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.poi = "scan"
        msg.status = int(status)
        msg.detail = str(detail)
        msg.detections_json = str(detections_json)
        msg.simulated = bool(self.dry_run if simulated is None else simulated)
        self._pub.publish(msg)


def main(argv=None):
    rclpy.init(args=argv)
    node = VisionScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
