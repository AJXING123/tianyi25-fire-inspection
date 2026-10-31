"""消防巡检视觉服务端：InspectPoi Action 服务端（机器人端，本机直连相机）。

流程：goal -> capturing(等一帧) -> inferring(YOLO) -> judging -> result
判定语义与 fire_inspection_interfaces/msg/PoiResult.msg 一致：
  0 NORMAL / 1 ABNORMAL / 2 UNCERTAIN / 3 TIMEOUT / 4 NO_FRAME / 5 NO_MODEL / 6 ERROR

dry_run=True 时立即返回 UNCERTAIN + simulated=True，绝不冒充真实结论。
"""

from __future__ import annotations

import os
import threading
import time

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import CompressedImage

from fire_inspection_interfaces.action import InspectPoi
from fire_inspection_vision.detect_core import (STATUS_CN, STATUS_ERROR,
                                                STATUS_NO_FRAME, STATUS_NORMAL,
                                                STATUS_UNCERTAIN, Detector)


def load_config():
    share = get_package_share_directory("fire_inspection_vision")
    path = os.path.join(share, "config", "vision.yaml")
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


class InspectActionServer(Node):
    def __init__(self):
        super().__init__("fire_inspect_poi")
        cfg = load_config()

        self.topic = cfg.get("topic", "/ob_camera_head/color/image_raw/compressed")
        self.action_name = cfg.get("action_name", "/fire_inspection/inspect_poi")
        thresholds = cfg.get("thresholds", {})
        timeouts = cfg.get("timeouts", {})

        self.detector = Detector(
            model_path=cfg.get("model_path", ""),
            imgsz=cfg.get("imgsz", 1280),
            conf_smoking=thresholds.get("conf_smoking", 0.60),
            conf_other=thresholds.get("conf_other", 0.65),
            device=str(cfg.get("device", "0")),
            smoking_geometry=cfg.get("smoking_geometry"),
        )
        self.frame_timeout = float(timeouts.get("frame_timeout_s", 3.0))
        self.infer_timeout = float(timeouts.get("inference_timeout_s", 5.0))

        self.dry_run = bool(cfg.get("dry_run", True))
        self.declare_parameter("dry_run", self.dry_run)
        self.dry_run = bool(self.get_parameter("dry_run").value)

        # 相机帧缓存：只保留最新一帧，旧帧直接丢弃
        self._cv = threading.Condition()
        self._payload = None
        self._seq = 0

        cbg = ReentrantCallbackGroup()
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(CompressedImage, self.topic, self._on_image,
                                 qos, callback_group=cbg)

        self._server = ActionServer(
            self,
            InspectPoi,
            self.action_name,
            execute_callback=self.execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=cbg,
        )

        self.get_logger().info(
            f"视觉服务端就绪 action={self.action_name} 相机={self.topic} "
            f"模型={self.detector.model_path or '(未配置)'} dry_run={self.dry_run}")

    # ---------- 相机 ----------
    def _on_image(self, msg):
        with self._cv:
            self._payload = bytes(msg.data)
            self._seq += 1
            self._cv.notify_all()

    def _wait_frame(self, start_seq, timeout_s):
        deadline = time.time() + timeout_s
        with self._cv:
            while self._seq == start_seq and rclpy.ok():
                remain = deadline - time.time()
                if remain <= 0:
                    return None
                self._cv.wait(min(remain, 0.1))
            return self._payload

    # ---------- Action ----------
    def _on_goal(self, goal_request):
        if not goal_request.poi:
            self.get_logger().warn("拒绝 goal：poi 为空")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_cancel(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _feedback(self, goal_handle, phase):
        fb = InspectPoi.Feedback()
        fb.phase = phase
        goal_handle.publish_feedback(fb)

    def _finish(self, goal_handle, status, simulated, detail, dets_json=""):
        result = InspectPoi.Result()
        result.status = int(status)
        result.simulated = bool(simulated)
        result.detail = detail
        result.detections_json = dets_json
        goal_handle.succeed()
        self.get_logger().info(
            f"InspectPoi 完成: {STATUS_CN.get(status, status)} | {detail}")
        return result

    def execute(self, goal_handle):
        req = goal_handle.request
        self.get_logger().info(f"收到 InspectPoi: poi={req.poi}")

        if self.dry_run:
            self._feedback(goal_handle, "capturing")
            return self._finish(goal_handle, STATUS_UNCERTAIN, True,
                                "dry_run 模拟结果，不代表真实识别")

        start_seq = self._seq
        timeout = float(req.timeout_s) if req.timeout_s > 0 else self.frame_timeout

        self._feedback(goal_handle, "capturing")
        payload = self._wait_frame(start_seq, timeout)
        if payload is None:
            return self._finish(goal_handle, STATUS_NO_FRAME, False,
                                f"{timeout:.1f}s 内没有收到新的相机帧")
        if goal_handle.is_cancel_requested:
            return self._finish(goal_handle, STATUS_UNCERTAIN, False, "已取消")

        frame = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8),
                             cv2.IMREAD_COLOR)
        if frame is None:
            return self._finish(goal_handle, STATUS_NO_FRAME, False, "图像解码失败")

        self._feedback(goal_handle, "inferring")
        dets, err, fail_status = self.detector.infer(frame, self.infer_timeout)
        if dets is None:
            status = fail_status if fail_status is not None else STATUS_ERROR
            return self._finish(goal_handle, status, False, err)

        self._feedback(goal_handle, "judging")
        dets = self.detector.filter(dets)
        status, detail = self.detector.judge(
            dets, req.expect_classes, req.abnormal_classes)
        return self._finish(goal_handle, status, False, detail,
                            self.detector.to_json(dets))


def main(argv=None):
    rclpy.init(args=argv)
    node = InspectActionServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
