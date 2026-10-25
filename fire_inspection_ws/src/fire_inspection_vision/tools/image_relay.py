#!/usr/bin/env python3
"""
机器人端图像中继（在机器人上运行）

订阅本地压缩图像话题，通过 TCP 把 JPEG 帧推给笔记本，绕开 DDS 跨机组播不通的问题。
每个客户端只保留最新一帧：网络慢时丢弃积压帧，避免画面延迟越积越大。

用法（机器人上）:
    source /opt/ros/jazzy/setup.bash
    source /home/nvidia/xos/setup.bash
    python3 robot_image_relay.py --port 5000
    python3 robot_image_relay.py --port 5000 --scale 0.5   # WiFi 带宽紧张时降分辨率
"""
import argparse
import socket
import struct
import threading

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import CompressedImage


def sensor_qos():
    return QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        durability=DurabilityPolicy.VOLATILE,
    )


class TcpClient:
    """单个笔记本连接：只发最新帧，发送阻塞时新帧直接覆盖旧帧"""

    def __init__(self, conn, addr):
        self.conn = conn
        self.addr = addr
        self.alive = True
        self._packet = None
        self._lock = threading.Lock()
        self._wake = threading.Event()
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        threading.Thread(target=self._send_loop, daemon=True).start()

    def push(self, packet):
        with self._lock:
            self._packet = packet
        self._wake.set()

    def _send_loop(self):
        while self.alive:
            self._wake.wait()
            self._wake.clear()
            with self._lock:
                packet, self._packet = self._packet, None
            if packet is None:
                continue
            try:
                self.conn.sendall(packet)
            except OSError:
                self.close()
                return

    def close(self):
        self.alive = False
        self._wake.set()
        try:
            self.conn.close()
        except OSError:
            pass


class Relay(Node):
    def __init__(self, topic, port, scale, quality):
        super().__init__("image_relay")
        self.tcp_clients = []
        self.lock = threading.Lock()
        self.count = 0
        self.scale = scale
        self.quality = quality

        self.sub = self.create_subscription(
            CompressedImage, topic, self.on_image, sensor_qos())

        self.srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind(("0.0.0.0", port))
        self.srv.listen(5)
        threading.Thread(target=self.accept_loop, daemon=True).start()

        hint = "" if scale == 1.0 else f"，缩放 {scale}"
        self.get_logger().info(f"订阅 {topic}，TCP 监听 0.0.0.0:{port}{hint}")

    def accept_loop(self):
        while True:
            try:
                conn, addr = self.srv.accept()
            except OSError:
                return
            client = TcpClient(conn, addr)
            with self.lock:
                self.tcp_clients.append(client)
            print(f"[relay] 笔记本已连接: {addr}")

    def on_image(self, msg):
        payload = bytes(msg.data)

        if self.scale != 1.0:
            img = cv2.imdecode(np.frombuffer(payload, np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                return
            img = cv2.resize(img, None, fx=self.scale, fy=self.scale,
                             interpolation=cv2.INTER_AREA)
            ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
            if not ok:
                return
            payload = buf.tobytes()

        packet = struct.pack(">I", len(payload)) + payload
        with self.lock:
            clients = list(self.tcp_clients)
        for c in clients:
            if c.alive:
                c.push(packet)

        alive = [c for c in clients if c.alive]
        if len(alive) != len(clients):
            with self.lock:
                self.tcp_clients = [c for c in self.tcp_clients if c.alive]
            print(f"[relay] 清理断开连接，当前客户端 {len(alive)}")

        self.count += 1
        if self.count % 30 == 0:
            print(f"[relay] 已转发 {self.count} 帧, 在线客户端 {len(alive)}")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--topic", default="/ob_camera_head/color/image_raw/compressed")
    p.add_argument("--port", type=int, default=5000)
    p.add_argument("--scale", type=float, default=1.0, help="送图前缩放比例，如 0.5")
    p.add_argument("--quality", type=int, default=80, help="缩放后重新编码的 JPEG 质量")
    args = p.parse_args()

    rclpy.init()
    node = Relay(args.topic, args.port, args.scale, args.quality)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.srv.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
