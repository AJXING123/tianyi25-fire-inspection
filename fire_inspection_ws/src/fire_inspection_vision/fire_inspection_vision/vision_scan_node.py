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
from ultralytics import YOLO

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
            conf_smoking=float(th.get("conf_smoking", 0.45)),
            conf_other=float(th.get("conf_other", 0.65)),
            device=str(cfg.get("device", "0")),
        )

        # 人体检测（巡检途中避让行人）：独立 COCO 模型，只取 person 类。
        # 与消防模型并行推理，任一模型失败都不影响另一个继续发布。
        person_cfg = cfg.get("person") or {}
        self.person_detector = None
        self.smoke_crop_detector = None
        self._last_person_warn = 0.0
        if bool(person_cfg.get("enabled", False)):
            person_path = str(person_cfg.get("model_path", ""))
            if not person_path or not os.path.isfile(person_path):
                self.get_logger().warn(
                    f"人体检测已启用但模型不存在：{person_path!r}，"
                    "本轮跳过人体检测（消防检测不受影响）")
            else:
                person_conf = float(person_cfg.get("conf", 0.5))
                self.person_detector = Detector(
                    model_path=person_path,
                    imgsz=int(person_cfg.get("imgsz", 640)),
                    conf_smoking=person_conf,
                    conf_other=person_conf,
                    device=str(cfg.get("device", "0")),
                    keep_classes=["person"],
                )
                # 上半身裁图补检专用：小尺寸更快、对大目标更敏感；
                # 与 1280 的整幅/裁图结果取最高分，覆盖远近两种尺度。
                if self.model_path and os.path.isfile(self.model_path):
                    self.smoke_crop_detector = Detector(
                        model_path=self.model_path,
                        imgsz=640,
                        conf_smoking=float(th.get("conf_smoking", 0.45)),
                        conf_other=float(th.get("conf_other", 0.65)),
                        device=str(cfg.get("device", "0")),
                    )

        # 专用香烟检测（只检 cigarette）：检出后映射成 smoking。
        cigarette_cfg = cfg.get("cigarette") or {}
        self.cigarette_detector = None
        self._cigarette_person_margin = 0.25
        self._cigarette_max_width_ratio = 0.12
        self._last_cig_size_log = 0.0
        if bool(cigarette_cfg.get("enabled", False)):
            cig_path = str(cigarette_cfg.get("model_path", ""))
            if not cig_path or not os.path.isfile(cig_path):
                self.get_logger().warn(
                    f"香烟检测已启用但模型不存在：{cig_path!r}，"
                    "本轮跳过香烟检测（消防/人体检测不受影响）")
            else:
                cig_conf = float(cigarette_cfg.get("conf", 0.5))
                self.cigarette_detector = Detector(
                    model_path=cig_path,
                    imgsz=int(cigarette_cfg.get("imgsz", 1280)),
                    conf_smoking=cig_conf,
                    conf_other=cig_conf,
                    device=str(cfg.get("device", "0")),
                    keep_classes=["cigarette"],
                )
                self._cigarette_person_margin = max(
                    0.0, float(cigarette_cfg.get("person_margin", 0.25)))
                self._cigarette_max_width_ratio = max(
                    0.0, float(cigarette_cfg.get("max_width_ratio", 0.12)))

        # 吸烟姿态门控：手腕必须在脸/嘴附近，过滤脸/海报/手机误报。
        gate_cfg = cfg.get("smoking_gate") or {}
        self.pose_model = None
        self._gate_imgsz = 640
        self._gate_conf = 0.50
        self._gate_wrist_face_ratio = 0.30
        self._gate_cigarette_wrist_ratio = 0.50
        self._gate_cigarette_nose_ratio = 0.12
        self._gate_cigarette_min_aspect = 1.2
        self._gate_fire_smoke_face_ratio = 0.28
        self._gate_wrist_conf = 0.50
        self._gate_nose_conf = 0.30
        self._gate_min_hits = 3
        self._gate_window_s = 2.0
        self._gate_pass_times = []
        if bool(gate_cfg.get("enabled", False)):
            gate_path = str(gate_cfg.get("model_path", ""))
            if not gate_path or not os.path.isfile(gate_path):
                self.get_logger().warn(
                    f"吸烟姿态门控已启用但模型不存在：{gate_path!r}，"
                    "本轮不做姿态门控（误报可能增多）")
            else:
                self.pose_model = YOLO(gate_path)
                self._gate_imgsz = int(gate_cfg.get("imgsz", 640))
                self._gate_conf = float(gate_cfg.get("conf", 0.50))
                self._gate_wrist_face_ratio = float(
                    gate_cfg.get("wrist_face_ratio", 0.30))
                self._gate_cigarette_wrist_ratio = float(
                    gate_cfg.get("cigarette_wrist_ratio", 0.20))
                self._gate_cigarette_nose_ratio = float(
                    gate_cfg.get("cigarette_nose_ratio", 0.12))
                self._gate_cigarette_min_aspect = float(
                    gate_cfg.get("cigarette_min_aspect", 1.2))
                self._gate_fire_smoke_face_ratio = float(
                    gate_cfg.get("fire_smoke_face_ratio", 0.15))
                self._gate_wrist_conf = float(
                    gate_cfg.get("wrist_conf", 0.50))
                self._gate_nose_conf = float(
                    gate_cfg.get("nose_conf", 0.30))
                self._gate_min_hits = max(
                    1, int(gate_cfg.get("min_hits", 3)))
                self._gate_window_s = max(
                    0.2, float(gate_cfg.get("window_s", 2.0)))

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
        if not 0.0 < self._depth_min_valid_ratio <= 1.0:
            raise RuntimeError(
                f"depth.min_valid_ratio 非法: {self._depth_min_valid_ratio}")
        if not (math.isfinite(self._depth_max_spread_m)
                and self._depth_max_spread_m > 0.0):
            raise RuntimeError(
                f"depth.max_spread_m 非法: {self._depth_max_spread_m}")
        self._depth = None
        self._depth_stamp = 0.0
        self._depth_scale = None
        self._depth_lock = threading.Lock()
        self._camera_info_logged = False
        self._last_smoke_diag = 0.0   # smoking 原始打分诊断日志节流（2s）
        self._last_gate_log = 0.0     # 姿态门控日志节流（2s）

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
            f"model={self.model_path} "
            f"人体检测={'开' if self.person_detector else '关'} "
            f"香烟检测={'开' if self.cigarette_detector else '关'} "
            f"吸烟门控={'开' if self.pose_model else '关'}")

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
        person_dets = self._infer_person(frame)
        if dets is None and person_dets is None:
            self._publish(status or STATUS_ERROR, err or "推理失败", "[]")
            return
        if dets is None:
            # 消防模型失败也要保留人体检测：避让行人是安全动作，不能一起丢
            self.get_logger().warn(
                f"消防模型推理失败（人体检测继续）：{err or status}")
            dets = []

        person_filtered = []
        if self.person_detector is not None and person_dets is not None:
            person_filtered = self.person_detector.filter(person_dets)
        # 小目标补检：远处的人体框里烟只占几个像素，整幅 1280 推理看不到；
        # 用人体上半身裁图再跑一次消防模型，能把 0.3x 的分拉到 0.6+。
        crop_smoke = self._smoking_from_person_crops(frame, person_filtered)
        cig_smoke = self._smoking_from_cigarette(frame, person_filtered)
        all_dets = dets + (person_dets or []) + crop_smoke + cig_smoke
        fire_filtered = self.detector.filter(dets)
        filtered = fire_filtered + person_filtered
        self._merge_smoking(filtered, crop_smoke)
        self._merge_smoking(filtered, cig_smoke)
        # 消防三分类模型自己是否看到 smoking（作为香烟模型漏检时的兜底）
        fire_smoke_seen = any(
            d.get("cls") == "smoking" for d in fire_filtered + crop_smoke)
        # 姿态门控：先要求「长条形香烟框」，再要求它靠近鼻子（手在嘴边）
        # 或靠近手腕（手拿烟）。现场误报（手机/脸/手）的烟框都接近正方形，
        # 长宽比≈0.9~1.0，会被这里直接排除。
        if (self.pose_model is not None
                and any(d.get("cls") == "smoking" for d in filtered)):
            pose = self._pose_points(frame, person_filtered)
            cig_all = []
            cig_long = []
            for det in cig_smoke:
                xy = det.get("xyxy") or []
                if len(xy) != 4:
                    continue
                bw = abs(float(xy[2]) - float(xy[0]))
                bh = max(1e-6, abs(float(xy[3]) - float(xy[1])))
                point = ((float(xy[0]) + float(xy[2])) / 2.0,
                         (float(xy[1]) + float(xy[3])) / 2.0)
                cig_all.append(point)
                if bw / bh >= self._gate_cigarette_min_aspect:
                    cig_long.append(point)
            face_ratio = None
            cig_wrist_ratio = None
            cig_nose_ratio = None
            if pose is not None:
                if pose["nose"] is not None:
                    face_ratio = self._min_ratio(
                        pose["wrists"], [pose["nose"]], pose["person_h"])
                    cig_nose_ratio = self._min_ratio(
                        cig_long, [pose["nose"]], pose["person_h"])
                cig_wrist_ratio = self._min_ratio(
                    pose["wrists"], cig_long, pose["person_h"])
            ok = (
                # 烟在嘴边：长条烟框贴近鼻子/嘴就算。3m 外姿态关键点常把
                # 手腕位置带偏（现场烟-嘴=0.02~0.07 却因烟-腕=0.3+ 被拒），
                # 这条能保证「人拿着/吸着烟」被识别，而不是只报人员避让。
                (cig_nose_ratio is not None
                 and cig_nose_ratio <= self._gate_cigarette_nose_ratio)
                # 手在嘴边吸：手靠近嘴 + 长条烟框也在嘴附近
                or (face_ratio is not None
                    and face_ratio <= self._gate_wrist_face_ratio
                    and cig_nose_ratio is not None
                    and cig_nose_ratio <= self._gate_cigarette_nose_ratio)
                # 手拿烟：长条烟框紧贴手腕
                or (cig_wrist_ratio is not None
                    and cig_wrist_ratio <= self._gate_cigarette_wrist_ratio)
                # 兜底：香烟模型完全没框，但消防模型明确看到吸烟且手靠近嘴
                or (fire_smoke_seen and not cig_all
                    and face_ratio is not None
                    and face_ratio <= self._gate_fire_smoke_face_ratio)
            )
            # 单帧通过不算：站姿偶然抬手/姿态抖动也会瞬时通过。
            # 只保留窗口内的通过帧，窗口内累计够 min_hits 帧才确认吸烟。
            now = time.monotonic()
            self._gate_pass_times = [
                t for t in self._gate_pass_times
                if now - t <= self._gate_window_s
            ]
            if ok:
                self._gate_pass_times.append(now)
            hits = len(self._gate_pass_times)
            confirmed = ok and hits >= self._gate_min_hits
            log_it = now - self._last_gate_log >= 2.0
            if log_it:
                self._last_gate_log = now
            shown_face = "n/a" if face_ratio is None else f"{face_ratio:.2f}"
            shown_wrist = (
                "n/a" if cig_wrist_ratio is None else f"{cig_wrist_ratio:.2f}")
            shown_nose = (
                "n/a" if cig_nose_ratio is None else f"{cig_nose_ratio:.2f}")
            if not confirmed:
                if log_it:
                    reason = (
                        f"待确认 {hits}/{self._gate_min_hits} 帧"
                        if ok else "未通过")
                    self.get_logger().info(
                        f"[diag] 吸烟姿态门控未确认（{reason}）：腕-脸={shown_face}"
                        f"/烟-腕={shown_wrist}/烟-嘴={shown_nose}，长条烟框 "
                        f"{len(cig_long)}/{len(cig_all)}，消防吸烟="
                        f"{int(fire_smoke_seen)}")
                filtered = [d for d in filtered if d.get("cls") != "smoking"]
                all_dets = [d for d in all_dets if d.get("cls") != "smoking"]
            elif log_it:
                if (cig_nose_ratio is not None
                        and cig_nose_ratio
                        <= self._gate_cigarette_nose_ratio):
                    how = "烟在嘴边"
                elif (face_ratio is not None
                        and face_ratio <= self._gate_wrist_face_ratio
                        and cig_nose_ratio is not None
                        and cig_nose_ratio <= self._gate_cigarette_nose_ratio):
                    how = "手在嘴边"
                elif (cig_wrist_ratio is not None
                      and cig_wrist_ratio
                      <= self._gate_cigarette_wrist_ratio):
                    how = "手拿烟"
                else:
                    how = "消防模型兜底"
                self.get_logger().info(
                    f"[diag] 吸烟姿态确认（{how}）：腕-脸={shown_face}"
                    f"/烟-腕={shown_wrist}/烟-嘴={shown_nose}，"
                    f"窗口内 {hits}/{self._gate_min_hits} 帧")
        self._log_smoking_diag(all_dets)
        self._annotate_ranges(filtered)
        detections_json = Detector.to_json(filtered)
        # 持续检测流不做判定（N/ABNORMAL/UNCERTAIN），只把检出列表交给编排层自己定夺
        # 编排层 WATCH/LOCKED 状态下只关心"有没有命中过目标"
        self._publish(2 if not filtered else 0,
                      f"检出 {len(filtered)} 个目标" if filtered else "无检出",
                      detections_json)

    def _infer_person(self, frame):
        """人体检测：失败返回 None（不影响消防检测），日志 5s 节流。"""
        if self.person_detector is None:
            return None
        dets, err, status = self.person_detector.infer(frame, timeout_s=5.0)
        if dets is None:
            now = time.monotonic()
            if now - self._last_person_warn >= 5.0:
                self._last_person_warn = now
                self.get_logger().warn(
                    f"人体检测失败（消防检测继续）：{err or status}")
        return dets

    @staticmethod
    def _primary_persons(person_dets, conf_slack=0.10, limit=2):
        """取置信度接近最高分的行人，排除海报/广告里低分人像。

        现场真实行人通常 conf 高且相近；背景海报人像分数明显更低。
        只保留与最高分相差不超过 conf_slack 的前 limit 个人。
        """
        if not person_dets:
            return []
        try:
            max_conf = max(float(d.get("conf", 0.0)) for d in person_dets)
        except (TypeError, ValueError):
            return []
        persons = [
            d for d in person_dets
            if float(d.get("conf", 0.0)) >= max_conf - conf_slack
        ]
        persons.sort(key=lambda d: float(d.get("conf", 0.0)), reverse=True)
        return persons[:limit]

    def _smoking_from_person_crops(self, frame, person_dets):
        """用主要人体框的上半身裁图补检吸烟（小目标）。

        远处人的烟只占几个像素，整幅 1280 推理分数常低于阈值；裁出上半身
        再放大到模型的 imgsz 后，烟占的画面比例显著变大。只保留 smoking 类，
        并把坐标映射回整幅画面，后续测距/发布口径不变。
        """
        if self.person_detector is None or not person_dets:
            return []
        height, width = frame.shape[:2]
        if height <= 0 or width <= 0:
            return []
        # 取置信度最高的前两个人（真实行人），低分海报人像不参与裁图。
        persons = self._primary_persons(person_dets)
        threshold = float(self.detector.conf_smoking)
        detectors = [self.detector]
        if self.smoke_crop_detector is not None:
            detectors.append(self.smoke_crop_detector)
        out = []
        for person in persons:
            try:
                cx = float(person["cx"])
                cy = float(person["cy"])
                bw = float(person["w"])
                bh = float(person["h"])
            except (KeyError, TypeError, ValueError):
                continue
            if bw <= 0.0 or bh <= 0.0:
                continue
            # 左右各扩 10%，上方扩 15%，向下取到人体框 65% 高度（覆盖手/嘴）
            x1 = max(0, int((cx - bw / 2.0 - 0.10 * bw) * width))
            x2 = min(width, int((cx + bw / 2.0 + 0.10 * bw) * width))
            y1 = max(0, int((cy - bh / 2.0 - 0.15 * bh) * height))
            y2 = min(height, int((cy - bh / 2.0 + 0.65 * bh) * height))
            if x2 - x1 < 16 or y2 - y1 < 16:
                continue
            crop = frame[y1:y2, x1:x2]
            if crop.size == 0:
                continue
            for detector in detectors:
                dets, _err, _status = detector.infer(crop, timeout_s=3.0)
                if dets is None:
                    continue
                for det in dets:
                    if canonical(str(det.get("cls", ""))) != "smoking":
                        continue
                    try:
                        conf = float(det.get("conf", 0.0))
                    except (TypeError, ValueError):
                        continue
                    if conf < threshold:
                        continue
                    xy = det.get("xyxy") or []
                    if len(xy) != 4:
                        continue
                    fx1 = x1 + int(xy[0])
                    fy1 = y1 + int(xy[1])
                    fx2 = x1 + int(xy[2])
                    fy2 = y1 + int(xy[3])
                    box = {
                        "cls": "smoking",
                        "conf": conf,
                        "cx": (fx1 + fx2) / 2.0 / width,
                        "cy": (fy1 + fy2) / 2.0 / height,
                        "w": (fx2 - fx1) / width,
                        "h": (fy2 - fy1) / height,
                        "xyxy": [fx1, fy1, fx2, fy2],
                    }
                    # 640/1280 两种裁图可能检出同一个烟，按 IoU 去重
                    if any(self._box_iou(box, prev) > 0.3 for prev in out):
                        continue
                    out.append(box)
        return out

    def _smoking_from_cigarette(self, frame, person_dets):
        """专用香烟模型：检到 cigarette 且落在最大人体框附近才映射成 smoking。

        专用模型对远处香烟召回明显更好，但也会在海报/广告人像上误报；
        用“最大人体框（外扩 person_margin）内”作为门控，避免把背景图案
        当成现场人员吸烟。
        """
        if self.cigarette_detector is None or not person_dets:
            return []
        height, width = frame.shape[:2]
        if height <= 0 or width <= 0:
            return []
        persons = self._primary_persons(person_dets)
        if not persons:
            return []
        margin = self._cigarette_person_margin
        boxes = []
        for person in persons:
            try:
                px1 = (float(person["cx"]) - float(person["w"]) / 2.0) * width
                px2 = (float(person["cx"]) + float(person["w"]) / 2.0) * width
                py1 = (float(person["cy"]) - float(person["h"]) / 2.0) * height
                py2 = (float(person["cy"]) + float(person["h"]) / 2.0) * height
            except (KeyError, TypeError, ValueError):
                continue
            pw, ph = px2 - px1, py2 - py1
            boxes.append((
                px1 - margin * pw, py1 - margin * ph,
                px2 + margin * pw, py2 + margin * ph, ph,
            ))
        if not boxes:
            return []
        dets, _err, _status = self.cigarette_detector.infer(
            frame, timeout_s=3.0)
        if dets is None:
            return []
        threshold = float(self.cigarette_detector.conf_other)
        out = []
        for det in dets:
            if canonical(str(det.get("cls", ""))) != "cigarette":
                continue
            try:
                conf = float(det.get("conf", 0.0))
            except (TypeError, ValueError):
                continue
            if conf < threshold:
                continue
            xy = det.get("xyxy") or []
            if len(xy) != 4:
                continue
            cx = (float(xy[0]) + float(xy[2])) / 2.0
            cy = (float(xy[1]) + float(xy[3])) / 2.0
            matched_h = [
                ph for x1, y1, x2, y2, ph in boxes
                if x1 <= cx <= x2 and y1 <= cy <= y2
            ]
            if not matched_h:
                continue
            # 物理尺度校验：真香烟只有几厘米，框长边占人体高度不到一成；
            # 眼镜/脸颊/衣领整块被误检成香烟时框宽 0.15~0.31 人体高。
            # 用最近的人体框高（取最大）做尺度基准，超限直接丢弃。
            person_h = max(matched_h)
            bw = float(xy[2]) - float(xy[0])
            if (person_h > 0.0
                    and bw > self._cigarette_max_width_ratio * person_h):
                now = time.monotonic()
                if now - self._last_cig_size_log >= 2.0:
                    self._last_cig_size_log = now
                    self.get_logger().info(
                        f"[diag] 香烟框过大 {bw:.0f}px/{person_h:.0f}px 人体高"
                        f"（>{self._cigarette_max_width_ratio:.2f}），"
                        "按脸/眼镜误检忽略")
                continue
            out.append({
                "cls": "smoking",
                "conf": conf,
                "cx": cx / width,
                "cy": cy / height,
                "w": (float(xy[2]) - float(xy[0])) / width,
                "h": (float(xy[3]) - float(xy[1])) / height,
                "xyxy": [int(v) for v in xy],
            })
        return out

    def _pose_points(self, frame, person_dets):
        """返回主行人的鼻子/手腕像素坐标与姿态框高；无可用手腕返回 None。

        真吸烟有两种姿态：手抬到嘴边吸（手腕靠近鼻子），或只是手拿烟
        （烟框靠近手腕）。鼻子允许缺失（头顶贴边被裁），只是算不了「手在
        嘴边」；手腕必须至少一个可用，两种姿态都靠它。
        """
        if self.pose_model is None or not person_dets:
            return None
        try:
            result = self.pose_model.predict(
                frame, imgsz=self._gate_imgsz, conf=self._gate_conf,
                verbose=False)[0]
        except Exception as exc:
            self.get_logger().warn(f"姿态门控推理失败：{exc}")
            return None
        boxes = getattr(result, "boxes", None)
        kps = getattr(result, "keypoints", None)
        if boxes is None or kps is None or len(boxes) == 0:
            return None
        height, width = frame.shape[:2]
        if height <= 0 or width <= 0:
            return None
        # 主行人框（像素坐标），用于把姿态结果和行人对上号
        person_xyxy = []
        for person in self._primary_persons(person_dets):
            try:
                px1 = (float(person["cx"]) - float(person["w"]) / 2.0) * width
                py1 = (float(person["cy"]) - float(person["h"]) / 2.0) * height
                px2 = (float(person["cx"]) + float(person["w"]) / 2.0) * width
                py2 = (float(person["cy"]) + float(person["h"]) / 2.0) * height
            except (KeyError, TypeError, ValueError):
                continue
            person_xyxy.append((px1, py1, px2, py2))
        if not person_xyxy:
            return None
        pose_xyxy = boxes.xyxy.cpu().numpy().tolist()
        pose_conf = boxes.conf.cpu().numpy().tolist()
        pose_kps = kps.xy.cpu().numpy()
        pose_kconf = (kps.conf.cpu().numpy() if kps.conf is not None
                      else None)
        best = None
        best_iou = 0.0
        for i, box in enumerate(pose_xyxy):
            for pbox in person_xyxy:
                iou = self._iou_xyxy(box, pbox)
                if iou > best_iou:
                    best_iou = iou
                    best = i
        if best is None:
            # 没配上（例如人体框和姿态框差异较大）就退回置信度最高的姿态
            best = max(range(len(pose_xyxy)), key=lambda i: pose_conf[i])
        box = pose_xyxy[best]
        person_h = float(box[3]) - float(box[1])
        if person_h <= 0.0:
            return None
        pts = pose_kps[best]
        confs = (pose_kconf[best] if pose_kconf is not None
                 else np.ones(len(pts), dtype=np.float32))
        nose = None
        if float(confs[0]) >= self._gate_nose_conf:
            nose = (float(pts[0][0]), float(pts[0][1]))
        wrists = []
        for idx in (9, 10):   # left_wrist / right_wrist
            if float(confs[idx]) < self._gate_wrist_conf:
                continue
            wrists.append((float(pts[idx][0]), float(pts[idx][1])))
        if not wrists:
            return None
        return {"nose": nose, "wrists": wrists, "person_h": person_h}

    @staticmethod
    def _min_ratio(anchors, targets, person_h):
        """锚点（手腕）到目标点的最小像素距离 / 人体框高。"""
        if not anchors or not targets or person_h <= 0.0:
            return None
        return min(
            math.hypot(ax - tx, ay - ty)
            for ax, ay in anchors for tx, ty in targets
        ) / person_h

    def _merge_smoking(self, filtered, extra):
        """合并补检的 smoking：同一位置保留置信度更高的那一框。"""
        for det in extra or []:
            overlaps = [
                prev for prev in filtered
                if prev.get("cls") == "smoking"
                and self._box_iou(det, prev) > 0.3
            ]
            if overlaps:
                best = max(overlaps, key=lambda p: float(p.get("conf", 0.0)))
                if float(det.get("conf", 0.0)) <= float(best.get("conf", 0.0)):
                    continue
                for prev in overlaps:
                    filtered.remove(prev)
            filtered.append(det)

    @staticmethod
    def _iou_xyxy(a, b):
        """像素坐标框 IoU，用于把姿态框和人体框配对。"""
        ax1, ay1, ax2, ay2 = (float(v) for v in a)
        bx1, by1, bx2, by2 = (float(v) for v in b)
        iw = max(0.0, min(ax2, bx2) - max(ax1, bx1))
        ih = max(0.0, min(ay2, by2) - max(ay1, by1))
        inter = iw * ih
        union = ((ax2 - ax1) * (ay2 - ay1)
                 + (bx2 - bx1) * (by2 - by1) - inter)
        return inter / union if union > 0.0 else 0.0

    @staticmethod
    def _box_iou(a, b):
        """归一化框 IoU，用于裁图补检与整幅检测去重。"""
        try:
            ax1 = float(a["cx"]) - float(a["w"]) / 2.0
            ay1 = float(a["cy"]) - float(a["h"]) / 2.0
            ax2 = ax1 + float(a["w"])
            ay2 = ay1 + float(a["h"])
            bx1 = float(b["cx"]) - float(b["w"]) / 2.0
            by1 = float(b["cy"]) - float(b["h"]) / 2.0
            bx2 = bx1 + float(b["w"])
            by2 = by1 + float(b["h"])
        except (KeyError, TypeError, ValueError):
            return 0.0
        iw = max(0.0, min(ax2, bx2) - max(ax1, bx1))
        ih = max(0.0, min(ay2, by2) - max(ay1, by1))
        inter = iw * ih
        union = a["w"] * a["h"] + b["w"] * b["h"] - inter
        return inter / union if union > 0 else 0.0

    def _log_smoking_diag(self, dets) -> None:
        """每 2 秒打印一次本帧检出汇总，用于判断「模型没看到」还是「看到但分数低」。

        只看 smoking 无法区分两种完全不同的故障：
          - 画面里什么都没有（相机断流/头部朝向不对）→ 所有类别都是 0
          - 画面里有东西但没认出烟（小目标/角度问题）→ 有其它类别、无 smoking
        因此这里同时输出各类别最高分。
        """
        now = time.monotonic()
        if now - self._last_smoke_diag < 2.0:
            return
        self._last_smoke_diag = now
        best_by_cls = {}
        for det in dets:
            cls = canonical(str(det.get("cls", "")))
            if cls not in best_by_cls or det["conf"] > best_by_cls[cls]:
                best_by_cls[cls] = det["conf"]
        if not best_by_cls:
            self.get_logger().info(
                f"[diag] 本帧检出 0 个（画面内无任何目标）；"
                f"smoking 阈值 {self.detector.conf_smoking:.2f}")
            return
        summary = ", ".join(
            f"{cls}={conf:.2f}" for cls, conf in
            sorted(best_by_cls.items(), key=lambda kv: kv[1], reverse=True)
        )
        smoke = best_by_cls.get("smoking")
        if smoke is None:
            tail = "smoking 无候选"
        else:
            tail = (f"smoking={smoke:.2f}"
                    f"（阈值 {self.detector.conf_smoking:.2f}，"
                    f"{'过' if smoke >= self.detector.conf_smoking else '未过'}）")
        self.get_logger().info(
            f"[diag] 本帧检出 {len(dets)} 个：{summary}；{tail}")

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
