"""消防巡检语音服务端（骨架）。

后续实现要点（平台语义已确认，不得改写）：
- cmd="stop" 只能带空 text，用于打断；新文本必须用 cmd="append"；
- 完成判定只认 status == "playing"，并加 >=0.4s 非播放防抖；
- 加文本时长估算兜底（约 0.22s/字），避免未起播时等满超时。
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node


class SpeakActionServer(Node):
    """骨架占位：只声明参数，不注册 Action、不调用 TTS。"""

    def __init__(self) -> None:
        super().__init__("fire_speak")
        self.declare_parameter("tts_service", "/intelligent_interaction/tts/play")
        self.declare_parameter("audio_id", "audio_guide")
        self.declare_parameter("query_timeout_s", 30.0)
        self.declare_parameter("dry_run", True)
        self.get_logger().warn("speak_action_server 骨架就绪：尚未实现 Speak Action")


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = SpeakActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
