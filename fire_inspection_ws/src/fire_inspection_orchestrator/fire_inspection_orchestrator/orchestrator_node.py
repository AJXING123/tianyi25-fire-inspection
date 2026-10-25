"""消防巡检编排层（骨架）—— 唯一指挥者 / 唯一状态机。

状态机：
    IDLE -> INIT(前置校验+限速) -> TO_START
      -> 每个点位: NAV_TO_POI -> INSPECT -> REPORT
      -> [RETURN_TO_START] -> DONE
    任意状态 --取消/失败--> ABORTED -> 清理 -> IDLE

后续实现要点：
- 只由本节点调用三个 Action，模块之间禁止互调；
- 先落终止状态，再执行播报/记录，避免异常打断导致状态丢失；
- 导航失败默认终止，不自动跳下一点；
- 视觉 UNCERTAIN/TIMEOUT/NO_FRAME 绝不落成 NORMAL。
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node


class OrchestratorNode(Node):
    """骨架占位：只声明参数，不调用任何 Action。"""

    def __init__(self) -> None:
        super().__init__("fire_inspection_orchestrator")
        self.declare_parameter("script_file", "")
        self.declare_parameter("dwell_s", 3.0)
        self.declare_parameter("return_to_start", False)
        self.declare_parameter("dry_run", True)
        self.get_logger().warn("orchestrator_node 骨架就绪：尚未实现状态机")


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = OrchestratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
