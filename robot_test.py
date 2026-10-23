"""
机器人检测测试（在笔记本端运行）

两种取流方式:
  TCP 中继模式（推荐，跨机组播不通时用）:
    先 ssh 到机器人运行 robot_image_relay.py，然后:
    python3 robot_test.py --tcp 192.168.22.50:5000

  ROS 直连模式（两边 DDS 能互相发现时用）:
    source /opt/ros/jazzy/setup.bash
    export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    python3 robot_test.py [--topic /ob_camera_head/color/image_raw/compressed]

通用参数: --conf 0.65（指示牌/灭火器） --conf-smoking 0.25（吸烟） --imgsz 640 --roi 0 --save-every 10 --max-frames 0 --no-show
按 q 或 ESC 退出，点窗口叉号也可退出。
"""
import argparse
import os
import socket
import struct
import threading
import time
from collections import Counter

import cv2
import numpy as np
from PIL import Image as PILImage, ImageDraw, ImageFont

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                           DurabilityPolicy)
    from sensor_msgs.msg import CompressedImage, Image
    ROS_OK = True
except ImportError:
    ROS_OK = False

CLASS_CN = {
    "smoking": "吸烟",
    "safety_sign": "安全指示牌",
    "fire_extinguisher": "灭火器",
}
COLORS = [(0, 0, 255), (0, 200, 0), (255, 128, 0), (255, 0, 255)]

FONT_PATH = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
FONT = ImageFont.truetype(FONT_PATH, 20) if os.path.exists(FONT_PATH) else None

# ===== 检测阈值配置（部署后现场微调只改这里，也可用命令行参数临时覆盖）=====
CONF_SMOKING = 0.25   # 吸烟：放宽阈值优先保证能检出，代价是弱误框可能变多
CONF_OTHER = 0.65     # 安全指示牌 / 灭火器：收紧阈值压制误检


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--model",
                   default="runs/train_3cls_v2/weights/best.pt")
    p.add_argument("--tcp", default="", help="TCP 中继地址，如 192.168.22.50:5000")
    p.add_argument("--topic", default="", help="ROS 相机话题，留空自动发现")
    p.add_argument("--conf", type=float, default=CONF_OTHER,
                   help="指示牌、灭火器的显示阈值，调高可压误检")
    p.add_argument("--conf-smoking", type=float, default=CONF_SMOKING,
                   help="吸烟的显示阈值，调低更灵敏（代价是纸盒等弱误框可能变多）")
    p.add_argument("--imgsz", type=int, default=640)
    p.add_argument("--roi", type=float, default=0.0,
                   help="只检测画面中央区域（如 0.5 = 中央一半宽高），相当于数字变焦，利于远处的烟头等小目标")
    p.add_argument("--device", default="0", help="0 为 GPU，cpu 为 CPU")
    p.add_argument("--save-dir", default="robot_test_output")
    p.add_argument("--save-every", type=int, default=10)
    p.add_argument("--no-show", action="store_true")
    p.add_argument("--max-frames", type=int, default=0)
    return p.parse_args()


if ROS_OK:
    def sensor_qos():
        return QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )

    class DetectNode(Node):
        def __init__(self):
            super().__init__("yolo_robot_test")
            self.is_compressed = False
            self.latest = None

        def callback(self, msg):
            self.latest = msg

        def subscribe(self, topic, is_compressed):
            self.is_compressed = is_compressed
            msg_type = CompressedImage if is_compressed else Image
            self.sub = self.create_subscription(msg_type, topic, self.callback,
                                                sensor_qos())
            self.get_logger().info(
                f"订阅: {topic} ({'compressed' if is_compressed else 'raw'})")

        def decode(self, msg):
            if self.is_compressed:
                return cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8),
                                    cv2.IMREAD_COLOR)
            h, w = msg.height, msg.width
            if msg.encoding in ("rgb8", "bgr8"):
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                    h, msg.step // 3, 3)[:, :w]
                return (cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                        if msg.encoding == "rgb8" else img.copy())
            if msg.encoding in ("mono8", "8UC1"):
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, msg.step)[:, :w]
                return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
            return None


def find_camera_topic(node):
    topics = dict(node.get_topic_names_and_types())
    cands = [
        (t, types) for t, types in topics.items()
        if "camera" in t.lower() and any("Image" in ty for ty in types)
    ]
    if not cands:
        return None, False
    compressed = [t for t, _ in cands if "compressed" in t.lower() and "depth" not in t.lower()]
    if compressed:
        return sorted(compressed, key=len)[0], True
    raw = [t for t, _ in cands if "compressed" not in t.lower() and "depth" not in t.lower()]
    if raw:
        return sorted(raw, key=len)[0], False
    return None, False


def draw(img, result, names, conf_by_class, offset=(0, 0)):
    counts = Counter()
    boxes = []
    ox, oy = offset
    for box in result.boxes:
        cls_id = int(box.cls)
        conf = float(box.conf)
        name = names[cls_id]
        # 每类单独阈值：吸烟放宽，其他类收紧
        if conf < conf_by_class.get(name, 0.0):
            continue
        counts[name] += 1
        x1, y1, x2, y2 = [int(v) for v in box.xyxy[0]]
        boxes.append((cls_id, conf, name, x1 + ox, y1 + oy, x2 + ox, y2 + oy))

    if FONT is not None:
        # PIL 渲染中文标签（cv2.putText 不支持中文）
        pil = PILImage.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        drawer = ImageDraw.Draw(pil)
        for cls_id, conf, name, x1, y1, x2, y2 in boxes:
            color = COLORS[cls_id % len(COLORS)]
            rgb = (color[2], color[1], color[0])
            drawer.rectangle([x1, y1, x2, y2], outline=rgb, width=2)
            label = f"{CLASS_CN.get(name, name)} {conf:.2f}"
            tb = drawer.textbbox((0, 0), label, font=FONT)
            tw, th = tb[2] - tb[0], tb[3] - tb[1]
            ty = max(y1 - th - 8, 0)
            drawer.rectangle([x1, ty, x1 + tw + 8, ty + th + 8], fill=rgb)
            drawer.text((x1 + 4, ty + 2), label, font=FONT, fill=(255, 255, 255))
        return counts, cv2.cvtColor(np.array(pil), cv2.COLOR_RGB2BGR), boxes

    for cls_id, conf, name, x1, y1, x2, y2 in boxes:
        color = COLORS[cls_id % len(COLORS)]
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
        label = f"{name} {conf:.2f}"
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        cv2.rectangle(img, (x1, max(y1 - th - 6, 0)), (x1 + tw + 4, y1), color, -1)
        cv2.putText(img, label, (x1 + 2, max(y1 - 4, th)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    return counts, img, boxes


class Output:
    """负责推理结果的画框、存图、CSV 记录和统计"""

    def __init__(self, args, model, names):
        os.makedirs(args.save_dir, exist_ok=True)
        self.args = args
        self.model = model
        self.names = names
        # 每类显示阈值：吸烟单独放宽
        self.conf_by_class = {
            name: (args.conf_smoking if name == "smoking" else args.conf)
            for name in names.values()
        }
        # 推理阈值取最低的一档，再由 draw 按类过滤
        self.infer_conf = min(self.conf_by_class.values())
        self.csv_path = os.path.join(args.save_dir, "detections.csv")
        self.csv_file = open(self.csv_path, "w", newline="", encoding="utf-8")
        self.writer = csv_writer(self.csv_file)
        self.frame_idx = 0
        self.saved = 0
        self.total = Counter()
        self.t0 = time.time()
        self.fps_t0 = time.time()
        self.fps_frames = 0
        self.fps = 0.0

    def handle(self, img):
        offset = (0, 0)
        infer_img = img
        if self.args.roi > 0:
            h, w = img.shape[:2]
            cw, ch = int(w * self.args.roi), int(h * self.args.roi)
            ox, oy = (w - cw) // 2, (h - ch) // 2
            offset = (ox, oy)
            infer_img = img[oy:oy + ch, ox:ox + cw]

        result = self.model.predict(infer_img, imgsz=self.args.imgsz, conf=self.infer_conf,
                                    device=self.args.device, verbose=False)[0]
        counts, img, boxes = draw(img, result, self.names, self.conf_by_class, offset)
        self.total.update(counts)

        now = time.strftime("%H:%M:%S")
        for _, conf, name, x1, y1, x2, y2 in boxes:
            self.writer.writerow([self.frame_idx, now, name,
                                  f"{conf:.3f}", x1, y1, x2, y2])

        self.frame_idx += 1
        self.fps_frames += 1
        elapsed = time.time() - self.fps_t0
        if elapsed >= 1.0:
            self.fps = self.fps_frames / elapsed
            self.fps_t0, self.fps_frames = time.time(), 0

        cv2.putText(img, f"FPS {self.fps:.1f}  frame {self.frame_idx}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

        if self.args.save_every > 0 and self.frame_idx % self.args.save_every == 0:
            cv2.imwrite(os.path.join(self.args.save_dir, f"{self.saved:05d}.jpg"), img)
            self.saved += 1

        if not self.args.no_show:
            cv2.imshow(WINDOW_NAME, img)
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                return False
            try:
                # 点窗口右上角叉号关闭时，可见性会变成 0
                if cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
                    return False
            except cv2.error:
                return False
        return True

    def close(self):
        self.csv_file.close()
        cv2.destroyAllWindows()
        dur = time.time() - self.t0
        print("\n" + "=" * 50)
        print(f"处理帧数: {self.frame_idx}  时长: {dur:.1f}s  "
              f"平均 FPS: {self.frame_idx / dur if dur else 0:.1f}")
        print(f"各类检出帧数: {dict(self.total)}")
        print(f"带框图保存: {self.saved} 张 -> {self.args.save_dir}")
        print(f"检测日志: {self.csv_path}")
        print("=" * 50)


WINDOW_NAME = "YOLO Robot Test (q to quit)"


def csv_writer(f):
    import csv
    w = csv.writer(f)
    w.writerow(["frame", "timestamp", "class", "conf", "x1", "y1", "x2", "y2"])
    return w


def recv_exact(sock, n):
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf


def run_tcp(args, out):
    host, port = args.tcp.rsplit(":", 1)
    port = int(port)
    sock = None
    while sock is None:
        try:
            sock = socket.create_connection((host, port), timeout=5)
            sock.settimeout(10)
            print(f"已连接机器人中继 {host}:{port}")
        except OSError as e:
            print(f"连接 {host}:{port} 失败({e})，2 秒后重试…（确认机器人上 relay 已启动）")
            time.sleep(2)

    try:
        sock.settimeout(None)  # 读帧线程用阻塞模式，保证整帧完整
        latest = [None]
        lock = threading.Lock()
        stop = threading.Event()

        def reader():
            while not stop.is_set():
                try:
                    header = recv_exact(sock, 4)
                    if header is None:
                        print("中继连接已断开")
                        break
                    size = struct.unpack(">I", header)[0]
                    payload = recv_exact(sock, size)
                    if payload is None:
                        print("中继连接中断")
                        break
                except OSError:
                    break
                with lock:
                    latest[0] = payload  # 只保留最新帧，旧的直接丢弃
            stop.set()

        threading.Thread(target=reader, daemon=True).start()

        while not stop.is_set():
            with lock:
                payload, latest[0] = latest[0], None
            if payload is None:
                time.sleep(0.002)
                continue
            img = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                continue
            if not out.handle(img):
                break
            if args.max_frames and out.frame_idx >= args.max_frames:
                break
    finally:
        sock.close()


def run_ros(args, out):
    if not ROS_OK:
        print("❌ ROS 环境未加载：先 source /opt/ros/jazzy/setup.bash")
        return
    node = DetectNode()
    if args.topic:
        node.subscribe(args.topic, "compressed" in args.topic)
    else:
        topic, is_compressed = find_camera_topic(node)
        if not topic:
            print("❌ 没发现相机话题：检查 ROS_DOMAIN_ID / RMW / 相机是否开启")
            node.destroy_node()
            return
        print(f"自动发现并订阅: {topic}")
        node.subscribe(topic, is_compressed)

    print("等待相机数据……")
    waited = 0
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            msg = node.latest
            if msg is None:
                waited += 1
                if waited % 100 == 0:
                    print(f"  仍未收到数据（已等 {waited * 0.05:.0f}s）…")
                continue
            node.latest = None
            img = node.decode(msg)
            if img is None or img.size == 0:
                continue
            if not out.handle(img):
                break
            if args.max_frames and out.frame_idx >= args.max_frames:
                break
    except Exception as e:
        if e.__class__.__name__ != "ExternalShutdownException":
            raise
    finally:
        node.destroy_node()


def main():
    args = parse_args()
    from ultralytics import YOLO
    model = YOLO(args.model)
    print(f"模型加载完成，类别: {model.names}")
    print(f"阈值: 吸烟 {args.conf_smoking}，指示牌/灭火器 {args.conf}")

    if not args.tcp and ROS_OK:
        rclpy.init()

    out = Output(args, model, model.names)
    try:
        if args.tcp:
            run_tcp(args, out)
        else:
            run_ros(args, out)
    except KeyboardInterrupt:
        pass
    finally:
        out.close()
        if not args.tcp and ROS_OK and rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
