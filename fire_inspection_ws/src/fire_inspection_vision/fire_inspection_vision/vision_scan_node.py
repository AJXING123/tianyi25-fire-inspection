"""T3 持续检测节点：订阅头部相机帧，持续推理并发 /fire_inspection/detection。

为编排层（orchestrator）提供实时检测流，使其能够基于「连续 N 帧同类命中」
做防误报确认，并在第一次命中时就锁定头部转向目标。

与 InspectPoi Action 的关系：
  - InspectPoi 是「单次点名检测」，供 orchestrator 在 NAV_TO_POI → INSPECT 阶段调用
  - vision_scan_node 是「持续检测流」，供 orchestrator 在 WATCH 阶段实时响应

配置来自 config/vision.yaml：
  topic: /fire_inspection/detection   发布话题（契约 T3，best_effort / depth 1）
  rate_hz: 2.0                        推理限速，太低漏目标，太高占满 GPU
  depth.*                             深度测距（见下）
  hfov_deg / vfov_deg                 视场角，供像素偏移换算方位角

发布：fire_inspection_interfaces/msg/PoiResult
  poi: "scan"                         持续检测固定标记为 scan
  status: 0-6                         NORMAL/ABNORMAL/UNCERTAIN/...
  simulated: true = dry_run           编排层对 simulated=true 的结果一律忽略
  detections_json: cx/cy/w/h 归一化   契约 §6.1 schema，conf 降序最多 20 条
    可选扩展字段（吸烟人员介入用）：
      range_m      目标到相机的水平距离（m，按头部 pitch 做余弦修正）
      bearing_rad  目标相对车头方向（rad，+左 / -右）= 头部 yaw + 像素偏移，
                   即把头部转到该 yaw 正好对准目标
      look_pitch_rad 把头部俯仰转到该值正好对准目标（绝对姿态，需当前头部 pitch）
      range_valid  该组值能否用于接近（深度有效且头部姿态新鲜）
    缺字段时下游按「无有效测距」处理，旧消费方不受影响。
    像素偏移优先用 camera_info 内参（fx/fy/主点）精确换算，内参未到达时
    才退回 config 的 hfov/vfov 线性近似。

本节点只做感知，不碰底盘，也不做任何运动决策。
"""

from __future__ import annotations

import math
import os
import threading
import time

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from ros2_bridge_msgs.msg import RobotState
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import Header

from fire_inspection_interfaces.msg import PoiResult
from fire_inspection_vision.detect_core import (Detector, STATUS_ERROR,
                                                STATUS_NO_FRAME, canonical)

# 头部关节 ID：1=roll，2=pitch（正=低头），3=yaw（正=向左）
HEAD_PITCH_JOINT = 2
HEAD_YAW_JOINT = 3


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
            conf_smoking=float(th.get("conf_smoking", 0.60)),
            conf_other=float(th.get("conf_other", 0.65)),
            device=str(cfg.get("device", "0")),
            smoking_geometry=cfg.get("smoking_geometry"),
        )

        # 相机
        cam_topic = str(cfg.get("topic", "/ob_camera_head/color/image_raw/compressed"))
        self._latest_frame = None
        self._frame_lock = threading.Lock()

        # 视场角：camera_info 缺失时的兜底（与编排层 hfov_deg/vfov_deg 同源）
        self._hfov = math.radians(float(cfg.get("hfov_deg", 60.0)))
        self._vfov = math.radians(float(cfg.get("vfov_deg", 34.0)))
        # 相机内参：到达后用精确换算替代视场角线性近似
        self._cam_fx = None
        self._cam_fy = None
        self._cam_w = 0
        self._cam_h = 0

        # 深度测距（吸烟人员介入用）
        depth_cfg = cfg.get("depth") or {}
        self._depth_topic = str(
            depth_cfg.get("topic", "/ob_camera_head/depth/image_raw"))
        self._camera_info_topic = str(
            depth_cfg.get("camera_info_topic",
                          "/ob_camera_head/color/camera_info"))
        self._depth_sample_ratio = float(depth_cfg.get("sample_ratio", 0.5))
        self._depth_min_m = float(depth_cfg.get("min_m", 0.3))
        self._depth_max_m = float(depth_cfg.get("max_m", 12.0))
        self._depth_unit = str(depth_cfg.get("unit", "auto")).strip().lower()
        # 一致性校验：有效占比过低或深度离散过大时不发布距离
        self._depth_min_valid_ratio = float(
            depth_cfg.get("min_valid_ratio", 0.5))
        self._depth_max_spread_m = float(depth_cfg.get("max_spread_m", 1.0))
        # 框尺寸交叉验证：按先验高度反推距离，偏差过大判为不可用
        raw_prior = depth_cfg.get("prior_height_m") or {}
        self._depth_prior_height = (
            raw_prior if isinstance(raw_prior, dict) else {})
        self._depth_size_tol = float(
            depth_cfg.get("size_check_tolerance", 0.5))
        if not 0.0 < self._depth_min_valid_ratio <= 1.0:
            raise RuntimeError(
                f"depth.min_valid_ratio 非法: {self._depth_min_valid_ratio}")
        if not (math.isfinite(self._depth_max_spread_m)
                and self._depth_max_spread_m > 0.0):
            raise RuntimeError(
                f"depth.max_spread_m 非法: {self._depth_max_spread_m}")
        if not 0.0 < self._depth_size_tol <= 5.0:
            raise RuntimeError(
                f"depth.size_check_tolerance 非法: {self._depth_size_tol}")
        self._depth = None
        self._depth_stamp = 0.0
        self._depth_scale = None
        self._depth_lock = threading.Lock()
        self._camera_info_logged = False
        self._last_size_warn = 0.0    # 尺寸交叉校验告警节流（2s 一次）

        # 头部姿态（/robot_state）：方位角 = 头部 yaw + 像素偏移
        self._head_state_topic = str(
            cfg.get("head_state_topic", "/robot_state"))
        self._head_pose_timeout_s = float(
            cfg.get("head_pose_timeout_s", 1.0))
        self._head_yaw = None
        self._head_pitch = None
        self._head_stamp = 0.0
        self._head_lock = threading.Lock()

        # T3 发布
        scan = cfg.get("scan") or {}
        scan_topic = str(scan.get("topic", "/fire_inspection/detection"))
        self.rate_hz = float(scan.get("rate_hz", 2.0))
        self._period = 1.0 / max(0.5, self.rate_hz)
        self._pub = self.create_publisher(PoiResult, scan_topic, detection_qos())

        # 订阅相机（best_effort / depth 1，只留最新帧）
        qos = detection_qos()
        self.create_subscription(CompressedImage, cam_topic,
                                 self._on_frame, qos)
        self.create_subscription(Image, self._depth_topic,
                                 self._on_depth, qos)
        self.create_subscription(CameraInfo, self._camera_info_topic,
                                 self._on_camera_info, qos)
        self.create_subscription(RobotState, self._head_state_topic,
                                 self._on_robot_state, 10)

        # 定时推理
        self._timer = self.create_timer(self._period, self._scan_once)

        self.get_logger().info(
            f"持续检测节点就绪 topic={scan_topic} rate={self.rate_hz:.1f}Hz "
            f"相机={cam_topic} 深度={self._depth_topic} dry_run={self.dry_run} "
            f"model={self.model_path}")

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

    # ---------- 深度 / 内参 / 头部姿态 ----------
    def _on_depth(self, msg):
        depths = self._decode_depth(msg)
        if depths is None:
            return
        with self._depth_lock:
            self._depth = depths
            self._depth_stamp = time.monotonic()

    def _decode_depth(self, msg):
        """把 Image 解成「米」为单位的 float32 深度图；不支持/损坏返回 None。"""
        enc = str(getattr(msg, "encoding", "") or "").lower()
        height = int(getattr(msg, "height", 0) or 0)
        width = int(getattr(msg, "width", 0) or 0)
        if height <= 0 or width <= 0:
            return None
        if "32f" in enc or "float" in enc:
            dtype = np.float32
            unit_scale = 1.0
        elif "16u" in enc or enc in ("mono16", "16uc1"):
            dtype = np.uint16
            unit_scale = None      # 由 unit 配置或数据量级决定
        else:
            return None

        itemsize = np.dtype(dtype).itemsize
        row_bytes = width * itemsize
        step = int(getattr(msg, "step", 0) or 0) or row_bytes
        raw = np.frombuffer(msg.data, dtype=np.uint8)
        if raw.size < step * height:
            return None
        try:
            if step == row_bytes:
                data = raw[: row_bytes * height].view(dtype).reshape(height, width)
            else:
                rows = raw[: step * height].reshape(height, step)[:, :row_bytes]
                data = np.ascontiguousarray(rows).view(dtype).reshape(height, width)
        except (ValueError, TypeError):
            return None

        # 必须拷贝：float32 编码时 data 是 msg.data 的视图，后面清零会改写消息内存
        depths = np.array(data, dtype=np.float32, copy=True)
        if unit_scale is None:
            unit_scale = self._resolve_unit_scale(depths)
        if unit_scale != 1.0:
            depths *= unit_scale
        self._depth_scale = unit_scale
        # NaN / inf 统一清成 0（后续按 0 视为无效）
        depths[~np.isfinite(depths)] = 0.0
        return depths

    def _resolve_unit_scale(self, depths) -> float:
        """16 位深度单位：mm 还是 m。

        配置写死优先；auto 时按有效值量级判断（本任务量程 0.3~12m，
        毫米编码的中位数远大于 50，米编码不会超过 50）。
        """
        if self._depth_unit == "mm":
            return 0.001
        if self._depth_unit == "m":
            return 1.0
        sample = depths[depths > 0]
        median = float(np.median(sample)) if sample.size else 0.0
        return 0.001 if median > 50.0 else 1.0

    def _on_camera_info(self, msg):
        """缓存内参（fx/fy/画面尺寸）：像素偏移优先用内参精确换算。"""
        raw_k = getattr(msg, "k", None)
        k = list(raw_k) if raw_k is not None else []
        width = int(getattr(msg, "width", 0) or 0)
        height = int(getattr(msg, "height", 0) or 0)
        if len(k) >= 6 and k[0] > 0 and k[4] > 0 and width > 0 and height > 0:
            self._cam_fx = float(k[0])
            self._cam_fy = float(k[4])
            self._cam_w = width
            self._cam_h = height
        if self._camera_info_logged:
            return
        if self._cam_fx is None:
            return
        self._camera_info_logged = True
        hfov = math.degrees(2.0 * math.atan(width / (2.0 * self._cam_fx)))
        vfov = math.degrees(2.0 * math.atan(height / (2.0 * self._cam_fy)))
        self.get_logger().info(
            f"相机内参 {width}x{height} fx={self._cam_fx:.1f} "
            f"fy={self._cam_fy:.1f}，标称视场角 hfov={hfov:.1f}° vfov={vfov:.1f}°；"
            f"方位角按内参换算（内参缺失时退回配置 "
            f"hfov={math.degrees(self._hfov):.1f}° vfov={math.degrees(self._vfov):.1f}°）"
        )

    def _yaw_offset(self, cx) -> float:
        """像素横坐标 → 相对光轴的方位角偏移（rad，+左）。"""
        if self._cam_fx and self._cam_w:
            return math.atan2((0.5 - float(cx)) * self._cam_w, self._cam_fx)
        return (0.5 - float(cx)) * self._hfov

    def _pitch_offset(self, cy) -> float:
        """像素纵坐标 → 相对光轴的俯仰偏移（rad，+低头）。"""
        if self._cam_fy and self._cam_h:
            return math.atan2((float(cy) - 0.5) * self._cam_h, self._cam_fy)
        return (float(cy) - 0.5) * self._vfov

    def _on_robot_state(self, msg):
        try:
            joints = {int(s.name): float(s.pos) for s in msg.head.status}
        except (AttributeError, TypeError, ValueError):
            return
        if HEAD_YAW_JOINT not in joints or HEAD_PITCH_JOINT not in joints:
            return
        with self._head_lock:
            self._head_yaw = joints[HEAD_YAW_JOINT]
            self._head_pitch = joints[HEAD_PITCH_JOINT]
            self._head_stamp = time.monotonic()

    def _latest_depth(self):
        with self._depth_lock:
            depth, stamp = self._depth, self._depth_stamp
        if depth is None:
            return None
        if time.monotonic() - stamp > max(1.0, 2.0 * self._period):
            return None      # 深度断流时宁可不给数据，也不要用旧距离
        return depth

    def _head_pose(self):
        """返回 (yaw, pitch) rad；姿态过期或无 /robot_state 时返回 None。"""
        with self._head_lock:
            yaw, pitch = self._head_yaw, self._head_pitch
            stamp = self._head_stamp
        if yaw is None or pitch is None:
            return None
        if time.monotonic() - stamp > self._head_pose_timeout_s:
            return None
        return yaw, pitch

    # ---------- 测距标注 ----------
    def _annotate_ranges(self, dets) -> None:
        """给每条检出补 range_m / bearing_rad / range_valid（就地修改）。

        保守策略：深度或头部姿态任一不可用就只写 range_valid=false，
        不写假的 0 距离，避免编排层把「不知道」当成「很近」。
        """
        depth = self._latest_depth()
        head = self._head_pose()
        for det in dets:
            if depth is None:
                det["range_valid"] = False
                continue
            try:
                range_m = self._measure_range(
                    depth, float(det["cx"]), float(det["cy"]),
                    float(det["w"]), float(det["h"]),
                    None if head is None else head[1],
                )
            except (KeyError, TypeError, ValueError):
                range_m = None
            if range_m is None:
                det["range_valid"] = False
                continue
            det["range_m"] = range_m
            # 深度与「框高反推距离」交叉验证：偏差过大说明框内取到了近处
            # 地面/遮挡（实测人 2.5m 外深度给 0.61m），本帧不作为接近依据
            size_range = self._size_range_m(
                det.get("cls"), det.get("h"), depth.shape[0])
            if size_range is not None:
                denom = max(range_m, size_range)
                if denom > 0.0 and (
                    abs(range_m - size_range) / denom
                ) > self._depth_size_tol:
                    now = time.monotonic()
                    if now - self._last_size_warn >= 2.0:
                        self._last_size_warn = now
                        self.get_logger().warn(
                            f"{det.get('cls')} 深度 {range_m:.2f}m 与框尺寸"
                            f"估算 {size_range:.2f}m 偏差过大，本帧不作为接近依据")
                    det["range_valid"] = False
                    continue
            if head is None:
                # 有距离但没有头部朝向，方位角无从谈起，不允许用于接近
                det["range_valid"] = False
                continue
            bearing = head[0] + self._yaw_offset(det["cx"])
            det["bearing_rad"] = max(-math.pi, min(math.pi, bearing))
            # 头部俯仰是绝对姿态：编排层据此发 look_at，与方位角口径一致
            det["look_pitch_rad"] = head[1] + self._pitch_offset(det["cy"])
            det["range_valid"] = True

    def _measure_range(self, depth, cx, cy, box_w, box_h, pitch):
        """检测框中心区域有效深度的中位数，按头部 pitch 修正成水平距离。

        额外做两项一致性校验，任何一项不过都返回 None（下游只播报不接近）：
          - 有效深度占比不低于 depth.min_valid_ratio；
          - 框内深度 10%~90% 分位差不超过 depth.max_spread_m，
            框同时盖住近处地面/门框和远处目标时，中心值没有代表性。
        """
        height, width = depth.shape[:2]
        if width <= 0 or height <= 0:
            return None
        ratio = min(max(self._depth_sample_ratio, 0.05), 1.0)
        patch_w = max(2, int(round(box_w * width * ratio)))
        patch_h = max(2, int(round(box_h * height * ratio)))
        u = int(round(cx * (width - 1)))
        v = int(round(cy * (height - 1)))
        u0 = max(0, min(width - 1, u - patch_w // 2))
        v0 = max(0, min(height - 1, v - patch_h // 2))
        u1 = max(u0 + 1, min(width, u0 + patch_w))
        v1 = max(v0 + 1, min(height, v0 + patch_h))
        patch = depth[v0:v1, u0:u1]
        if patch.size == 0:
            return None
        valid = patch[(patch >= self._depth_min_m) & (patch <= self._depth_max_m)]
        # 有效点太少说明框内有遮挡/反光/背景，按无效处理
        if valid.size < self._depth_min_valid_ratio * patch.size:
            return None
        spread = float(np.percentile(valid, 90) - np.percentile(valid, 10))
        if spread > self._depth_max_spread_m:
            return None
        range_m = float(np.median(valid))
        if pitch is not None and math.isfinite(pitch):
            # 深度沿光轴测量；低头时投影到水平面的距离约为 z*cos(pitch)
            range_m *= math.cos(pitch)
        if not math.isfinite(range_m) or range_m <= 0.0:
            return None
        return range_m

    def _size_range_m(self, cls, box_h, image_height):
        """按先验高度和框高反推光轴距离；未配置该类或内参缺失返回 None。

        针孔模型：Z = fy * 真实高度 / 框高像素。只用于「深度是否离谱」的
        交叉校验，不直接当作测距结果。
        """
        if not self._cam_fy or image_height <= 0:
            return None
        prior = self._depth_prior_height.get(canonical(str(cls or "")))
        try:
            height_m = float(prior)
            box_h = float(box_h)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(height_m) or height_m <= 0.0:
            return None
        pixels = box_h * float(image_height)
        if pixels <= 0.0:
            return None
        return float(self._cam_fy) * height_m / pixels

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
        self._annotate_ranges(filtered)
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
