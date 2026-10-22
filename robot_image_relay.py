#!/usr/bin/env python3
"""
机器人端图像中继（在机器人上运行）

订阅本地压缩图像话题，通过 TCP 把 JPEG 帧推给笔记本，绕开 DDS 跨机组播不通的问题。

用法（机器人上）:
    source /opt/ros/jazzy/setup.bash
    source /home/nvidia/xos/setup.bash
    python3 robot_image_relay.py --port 5000
    python3 robot_image_relay.py --topic /ob_camera_head/color/image_raw/compressed
"""
import argparse
import socket
import struct
import threading

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


class Relay(Node):
    def __init__(self, topic, port):
        super().__init__("image_relay")
        self.tcp_clients = []
        self.lock = threading.Lock()
        self.count = 0

        self.sub = self.create_subscription(
            CompressedImage, topic, self.on_image, sensor_qos())

        self.srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind(("0.0.0.0", port))
        self.srv.listen(5)
        threading.Thread(target=self.accept_loop, daemon=True).start()

        self.get_logger().info(f"订阅 {topic}，TCP 监听 0.0.0.0:{port}")

    def accept_loop(self):
        while True:
            try:
                conn, addr = self.srv.accept()
            except OSError:
                return
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            with self.lock:
                self.tcp_clients.append(conn)
            print(f"[relay] 笔记本已连接: {addr}")

    def on_image(self, msg):
        payload = bytes(msg.data)
        packet = struct.pack(">I", len(payload)) + payload
        dead = []
        with self.lock:
            clients = list(self.tcp_clients)
        for c in clients:
            try:
                c.sendall(packet)
            except OSError:
                dead.append(c)
        if dead:
            with self.lock:
                for c in dead:
                    if c in self.tcp_clients:
                        self.tcp_clients.remove(c)
                    try:
                        c.close()
                    except OSError:
                        pass
            print(f"[relay] 断开 {len(dead)} 个客户端")

        self.count += 1
        if self.count % 30 == 0:
            print(f"[relay] 已转发 {self.count} 帧, 在线客户端 {len(clients)}")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--topic", default="/ob_camera_head/color/image_raw/compressed")
    p.add_argument("--port", type=int, default=5000)
    args = p.parse_args()

    rclpy.init()
    node = Relay(args.topic, args.port)
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
