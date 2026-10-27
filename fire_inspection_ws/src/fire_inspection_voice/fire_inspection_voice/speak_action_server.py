"""消防巡检语音服务端：/fire_inspection/speak -> 本体 TTS。

把契约 §4 A3 的 Speak Action 落到本体 TTS 服务
/intelligent_interaction/tts/play（interaction_msgs/srv/TtsService）：

    1. cmd='query' 先查播报状态（playing 时 append 会自动排队）；
    2. cmd='append', type='text', audio_id=<audio_id> 排队播报；
    3. 轮询 cmd='query' 直到播报结束（或文本时长估算兜底），期间发 Feedback；
    4. 目标被取消时用 cmd='stop', audio_id=<audio_id> 真正打断音频。

🔴 audio_id 必须是 audio_config.json 里开启 stop 权限的文本型实体（当前 audio_guide），
   纯 stop 不带 audio_id 会被平台忽略，音频会播完。

🔴 执行回调里会阻塞轮询，必须 ReentrantCallbackGroup + MultiThreadedExecutor，
   否则 cancel 和其它回调无法被处理。

语义约定（2026-10-27 第二批定点修订，均不改动冻结接口）：
- query_timeout_s 是播放查询硬上限，文本估算兜底不得突破该上限；
- 从未观测到 playing / 查询超时 → success=False, state='timeout'
  （无法确认播报完成，不声称 TTS 一定失败）；成功完成才 success=True, state='done'；
- 同一时间只允许一个语音 Goal：并发 Goal 在受理与执行两处都会被拒绝；
- 主动取消走 ROS Action 取消终态（canceled），并真正 stop 音频。
"""

from __future__ import annotations

import os
import threading
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from interaction_msgs.srv import TtsService
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from fire_inspection_interfaces.action import Speak


def load_config():
    share = get_package_share_directory("fire_inspection_voice")
    path = os.path.join(share, "config", "voice.yaml")
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


class SpeakActionServer(Node):
    def __init__(self) -> None:
        super().__init__("fire_speak")
        cfg = load_config()
        timing = cfg.get("timing") or {}

        self.declare_parameter("dry_run", bool(cfg.get("dry_run", False)))
        self.declare_parameter("tts_service", cfg.get("tts_service",
                                                      "/intelligent_interaction/tts/play"))
        self.declare_parameter("audio_id", cfg.get("audio_id", "audio_guide"))
        self.declare_parameter("query_timeout_s",
                               float(timing.get("query_timeout_s", 30.0)))

        self.dry_run = bool(self.get_parameter("dry_run").value)
        self.tts = str(self.get_parameter("tts_service").value)
        self.audio_id = str(self.get_parameter("audio_id").value)
        self.timeout = float(self.get_parameter("query_timeout_s").value)

        self.poll = float(timing.get("query_poll_s", 0.2))
        self.playing_status = str(timing.get("playing_status", "playing"))
        self.settle = float(timing.get("settle_s", 0.4))
        self.s_per_char = float(timing.get("est_char_rate_s", 0.22))
        self.min_duration = float(timing.get("est_min_duration_s", 1.5))

        self.cli = self.create_client(TtsService, self.tts)
        self.group = ReentrantCallbackGroup()
        # 并发 Goal 互斥：_on_goal 快速拒绝 + _execute 原子保护（acquire 失败
        # 直接拒绝；无论正常结束、取消还是异常，finally 必然释放）
        self._exec_lock = threading.Lock()
        self.server = ActionServer(
            self, Speak, "/fire_inspection/speak",
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=self.group,
        )
        self.get_logger().info(
            f"语音服务端就绪 /fire_inspection/speak -> {self.tts} "
            f"audio_id={self.audio_id} dry_run={self.dry_run}")

    # ---------------- 回调 ----------------
    def _on_goal(self, goal_request: Speak.Goal) -> GoalResponse:
        if not goal_request.text.strip():
            self.get_logger().warn("拒绝空文本的播报目标")
            return GoalResponse.REJECT
        # G6: 已有 Goal 执行中则快速拒绝（权威保护仍在 _execute 的
        # acquire(blocking=False)，此处只是受理阶段的快速路径）
        if self._exec_lock.locked():
            self.get_logger().warn("已有语音 Goal 执行中，拒绝新 Goal")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_cancel(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle) -> Speak.Result:
        # 并发保护：同一时间只允许一个语音 Goal 执行；拒绝并发 Goal。
        # 无论正常结束、取消、超时还是异常，finally 都会释放锁。
        if not self._exec_lock.acquire(blocking=False):
            self.get_logger().warn("已有语音 Goal 执行中，拒绝并发 Goal")
            goal_handle.abort()
            return self._result(False, "已有语音 Goal 执行中", "rejected")
        try:
            return self._execute_locked(goal_handle)
        finally:
            self._exec_lock.release()

    def _execute_locked(self, goal_handle) -> Speak.Result:
        req = goal_handle.request
        text = req.text.strip()

        if self.dry_run:
            self.get_logger().info(f"[dry_run] 播报: {text}")
            goal_handle.succeed()
            return self._result(True, "dry_run 未调用 TTS", "done")

        if not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"TTS 服务 {self.tts} 未就绪")
            goal_handle.abort()
            return self._result(False, f"TTS 服务 {self.tts} 未就绪", "rejected")

        if req.interrupt:
            self.get_logger().info("interrupt=true：先打断当前播报")
            self._stop_tts()

        self._feedback(goal_handle, 0.0, "submitting")
        q = self._call("query", "")
        if q is not None and q.success and q.status == self.playing_status:
            self.get_logger().info("当前正在播报，本次文本将排队")

        appended = self._call("append", text, self.audio_id)
        if appended is None or not appended.success:
            self.get_logger().error("TTS append 未成功")
            goal_handle.abort()
            return self._result(False, "TTS append 未成功", "rejected")

        est = max(self.min_duration, len(text) * self.s_per_char)
        # G1: query_timeout_s 是播放查询硬上限，文本估算兜底不得突破该上限
        hard = min(est * 2.0 + 5.0, self.timeout)
        est_deadline = min(est + 1.0, self.timeout)
        t0 = time.monotonic()
        seen_playing = False
        idle_since = None
        phase = "playing"

        while True:
            elapsed = time.monotonic() - t0
            if goal_handle.is_cancel_requested:
                self._stop_tts()
                goal_handle.canceled()
                msg = f"播报被取消，已打断音频（已等 {elapsed:.1f}s）"
                self.get_logger().warn(msg)
                return self._result(False, msg, "rejected")

            q = self._call("query", "")
            playing = bool(q is not None and q.success
                           and q.status == self.playing_status)
            if playing:
                seen_playing = True
                idle_since = None
                phase = "playing"
            elif idle_since is None:
                idle_since = elapsed
                phase = "settling"

            done = (seen_playing and idle_since is not None
                    and elapsed - idle_since >= self.settle)
            if not done and not seen_playing and elapsed >= est_deadline:
                # 从未观测到 playing：无法确认播报完成（不声称 TTS 一定失败，
                # 可能瞬播完/被抢占/未合成），不得伪装为成功
                msg = (f"无法确认播报完成（从未观测到播放状态，"
                       f"已等待 {elapsed:.1f}s）")
                self.get_logger().warn(msg)
                goal_handle.abort()
                return self._result(False, msg, "timeout")
            if not done and elapsed >= hard:
                # 播放查询超时 → success=False, state="timeout"
                msg = f"播放查询超时（超过 {hard:.1f}s 上限）"
                self.get_logger().warn(msg)
                goal_handle.abort()
                return self._result(False, msg, "timeout")
            if done:
                break

            self._feedback(goal_handle, min(elapsed / est, 1.0), phase)
            time.sleep(self.poll)

        total = time.monotonic() - t0
        goal_handle.succeed()
        msg = f"播报结束（{total:.1f}s，seen_playing={seen_playing}）"
        self.get_logger().info(msg)
        return self._result(True, msg, "done")

    # ---------------- 工具 ----------------
    @staticmethod
    def _feedback(goal_handle, progress, phase):
        fb = Speak.Feedback()
        fb.progress = float(progress)
        fb.phase = phase
        goal_handle.publish_feedback(fb)

    @staticmethod
    def _result(success, message, state):
        result = Speak.Result()
        result.success = bool(success)
        result.message = message
        result.state = state
        return result

    def _call(self, cmd: str, text: str, audio_id: str = ""):
        req = TtsService.Request()
        req.text = text
        req.type = "text" if text else ""
        req.cmd = cmd
        req.audio_id = audio_id
        req.extra = ""
        future = self.cli.call_async(req)
        deadline = time.monotonic() + 5.0
        while not future.done():
            if time.monotonic() > deadline:
                self.get_logger().warn(f"TTS {cmd} 调用超时")
                return None
            time.sleep(0.02)
        try:
            return future.result()
        except Exception as exc:
            self.get_logger().error(f"TTS {cmd} 调用异常: {exc}")
            return None

    def _stop_tts(self):
        # stop 是 best-effort：调用异常不得中断播报主流程
        try:
            resp = self._call("stop", "", self.audio_id)
        except Exception as exc:
            self.get_logger().warn(f"TTS stop 调用异常: {exc}")
            return
        if resp is None:
            self.get_logger().warn(f"TTS stop(audio_id={self.audio_id}) 无应答")
        elif not resp.success:
            self.get_logger().warn(f"TTS stop 未成功: status={resp.status}")


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = SpeakActionServer()
    executor = MultiThreadedExecutor()
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

