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

云端 TTS 兜底（2026-10-28）：
- 提交前先做云端可达性预检（DNS + TCP），不可达最多等 tts_cloud.wait_s 秒；
- 平台合成失败会播放固定 1.806s 的 tts_failed.wav 且状态同为 playing，
  用有效播放时长识别，首次重试一次，仍短则 success=False, state='tts_failed'。

离线语音缓存（2026-10-28）：
- 巡检点位（POI_02/POI_03）WiFi 丢路由时云端合成不可达，等待也无济于事；
- audio_cache.dir 下按文本 sha1 命中的 wav 用 type='file' 直接播本地音频，
  全程不依赖网络；未命中或本地播放失败才回退云端合成。
"""

from __future__ import annotations

import hashlib
import os
import socket
import threading
import time
import wave

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from interaction_msgs.srv import TtsService
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool

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

        cloud = cfg.get("tts_cloud") or {}
        self.cloud_host = str(cloud.get("host", "cosyvoice.x-humanoid-cloud.com"))
        self.cloud_port = int(cloud.get("port", 9962))
        self.cloud_wait = float(cloud.get("wait_s", 15.0))

        cache = cfg.get("audio_cache") or {}
        self.declare_parameter("audio_cache_enabled", bool(cache.get("enabled", True)))
        self.declare_parameter("audio_cache_dir", str(cache.get("dir", "")))
        self.cache_enabled = bool(self.get_parameter("audio_cache_enabled").value)
        self.cache_dir = str(self.get_parameter("audio_cache_dir").value)

        self.poll = float(timing.get("query_poll_s", 0.2))
        self.playing_status = str(timing.get("playing_status", "playing"))
        self.settle = float(timing.get("settle_s", 0.4))
        self.s_per_char = float(timing.get("est_char_rate_s", 0.22))
        self.min_duration = float(timing.get("est_min_duration_s", 1.5))

        self.cli = self.create_client(TtsService, self.tts)
        # 播报状态：播报时麦克风会收到机器人自己的声音。听觉节点订阅此话题
        # 做半双工，播报期间不识别，避免把 TTS 误当成防空警报。
        self._speaking_pub = self.create_publisher(
            Bool, "/fire_inspection/speaking", 10)
        self.speaking_flag = False   # 当前是否真的在出声（dry_run 不算）
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
        cache_note = (f"本地缓存={self.cache_dir}" if self.cache_enabled
                      else "本地缓存=关闭")
        self.get_logger().info(
            f"语音服务端就绪 /fire_inspection/speak -> {self.tts} "
            f"audio_id={self.audio_id} dry_run={self.dry_run} {cache_note}")

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
            # 无论成功/取消/异常都广播"播报结束"，避免听觉节点一直屏蔽
            if self.speaking_flag:
                self.speaking_flag = False
                self._speaking_pub.publish(Bool(data=False))

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

        # 从这里开始真的出声，通知听觉节点进入半双工屏蔽
        self.speaking_flag = True
        self._speaking_pub.publish(Bool(data=True))

        # 1) 本地缓存优先：POI_02/POI_03 等位置 WiFi 丢路由时云端合成只能播
        #    失败提示音；命中预录缓存则用 type=file 播本地音频，全程不依赖网络。
        cache_file = self._cache_file(text)
        if cache_file is not None:
            seconds = self._wav_seconds(cache_file)
            est = max(0.8, seconds)
            # G1: query_timeout_s 仍是播放查询硬上限
            hard = min(est * 1.8 + 4.0, self.timeout)
            est_deadline = min(est + 1.2, self.timeout)
            min_span = max(0.8, est * 0.7)
            self.get_logger().info(
                f"命中本地语音缓存（{seconds:.1f}s），离线播报：{cache_file}")
            outcome, detail, span, total = self._play_and_wait(
                goal_handle, cache_file, "file", est, hard, est_deadline)
            if outcome == "canceled":
                return self._result(False, detail, "rejected")
            if outcome == "ok" and span >= min_span:
                goal_handle.succeed()
                msg = f"播报结束（{total:.1f}s，本地缓存）"
                self.get_logger().info(msg)
                return self._result(True, msg, "done")
            self.get_logger().warn(
                f"本地缓存播报未确认（{outcome}：{detail}），回退云端 TTS")

        # 2) 云端合成（无缓存或缓存播放失败）
        est = max(self.min_duration, len(text) * self.s_per_char)
        # G1: query_timeout_s 是播放查询硬上限，文本估算兜底不得突破该上限
        hard = min(est * 2.0 + 5.0, self.timeout)
        est_deadline = min(est + 1.0, self.timeout)
        # 平台在云端 TTS 失败（如 DNS 解析失败）时会回退播放 1.806s 的
        # tts_failed.wav，状态同样是 playing；用有效播放时长识别这种
        # “只放了失败提示音”，并重试一次。该兜底音固定 1.8s，而最短播报
        # 文案（8~12 字）实际语音也在 2.2s 以上，故下界取 1.9s；
        # 长文本再叠加 est×0.55 的相对下界，避免短文本误判。
        min_span = max(1.9, est * 0.55)

        for attempt in (1, 2):
            self._feedback(goal_handle, 0.0, "submitting")
            # 云端 TTS 不可达（DNS 解析失败/端口不通）时 append 必然只播
            # tts_failed.wav（现场听到的「音频生成失败」）。这里不再「等满
            # 时间也照常提交」，而是直接跳过本次播报：宁可不出声，也不播
            # 平台失败提示音干扰巡检。
            wait_deadline = time.monotonic() + self.cloud_wait
            waiting = False
            ready = self._cloud_ready()
            while not ready:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    msg = "等待云端 TTS 恢复期间被取消"
                    self.get_logger().warn(msg)
                    return self._result(False, msg, "rejected")
                if time.monotonic() >= wait_deadline:
                    msg = (
                        f"云端 TTS {self.cloud_host}:{self.cloud_port} 不可达"
                        f"（已等 {self.cloud_wait:.0f}s），跳过本次播报，"
                        "不播放失败提示音")
                    self.get_logger().error(msg)
                    goal_handle.abort()
                    return self._result(False, msg, "tts_failed")
                if not waiting:
                    self.get_logger().warn(
                        f"云端 TTS {self.cloud_host}:{self.cloud_port} 不可达"
                        "（疑似 DNS 异常），等待恢复…")
                    waiting = True
                self._feedback(goal_handle, 0.0, "waiting_network")
                time.sleep(0.5)
                ready = self._cloud_ready()

            outcome, detail, span, total = self._play_and_wait(
                goal_handle, text, "text", est, hard, est_deadline)
            if outcome == "canceled":
                return self._result(False, detail, "rejected")
            if outcome == "append_failed":
                goal_handle.abort()
                return self._result(False, detail, "rejected")
            if outcome in ("no_playing", "timeout"):
                goal_handle.abort()
                return self._result(False, detail, "timeout")
            if span < min_span:
                msg = (f"疑似播放了 TTS 失败提示音（有效播放 {span:.1f}s < "
                       f"下界 {min_span:.1f}s（估算 {est:.1f}s），第 {attempt} 次）")
                if attempt == 1:
                    if not self._cloud_ready():
                        # 云端已断：重试只会再播一次失败提示音，直接放弃
                        self.get_logger().error(msg + "；云端不可达，不再重试")
                        goal_handle.abort()
                        return self._result(False, msg, "tts_failed")
                    self.get_logger().warn(msg + "，重试一次")
                    time.sleep(0.5)
                    continue
                self.get_logger().error(msg + "，判定播报失败")
                goal_handle.abort()
                return self._result(False, msg, "tts_failed")

            goal_handle.succeed()
            self.get_logger().info(detail)
            return self._result(True, detail, "done")

    def _play_and_wait(self, goal_handle, payload: str, msg_type: str,
                       est: float, hard: float, est_deadline: float):
        """提交一次播报并轮询到结束。

        返回 (outcome, detail, span, total)：
        - outcome: ok / canceled / append_failed / no_playing / timeout
        - span: 观测到的有效播放时长（用于失败提示音识别）
        """
        q = self._call("query", "")
        if q is not None and q.success and q.status == self.playing_status:
            self.get_logger().info("当前正在播报，本次文本将排队")

        appended = self._call("append", payload, self.audio_id, msg_type)
        if appended is None or not appended.success:
            msg = f"TTS append 未成功（type={msg_type}）"
            self.get_logger().error(msg)
            return "append_failed", msg, 0.0, 0.0

        t0 = time.monotonic()
        seen_playing = False
        idle_since = None
        phase = "playing"
        first_playing = None
        last_playing = None

        while True:
            elapsed = time.monotonic() - t0
            if goal_handle.is_cancel_requested:
                self._stop_tts()
                goal_handle.canceled()
                msg = f"播报被取消，已打断音频（已等 {elapsed:.1f}s）"
                self.get_logger().warn(msg)
                return "canceled", msg, 0.0, elapsed

            q = self._call("query", "")
            playing = bool(q is not None and q.success
                           and q.status == self.playing_status)
            if playing:
                seen_playing = True
                if first_playing is None:
                    first_playing = elapsed
                last_playing = elapsed
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
                return "no_playing", msg, 0.0, elapsed
            if not done and elapsed >= hard:
                # 播放查询超时 → success=False, state="timeout"
                msg = f"播放查询超时（超过 {hard:.1f}s 上限）"
                self.get_logger().warn(msg)
                return "timeout", msg, 0.0, elapsed
            if done:
                break

            self._feedback(goal_handle, min(elapsed / est, 1.0), phase)
            time.sleep(self.poll)

        total = time.monotonic() - t0
        span = (last_playing - first_playing
                if first_playing is not None and last_playing is not None
                else 0.0)
        return "ok", (f"播报结束（{total:.1f}s，seen_playing={seen_playing}）"), span, total

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

    def _cloud_ready(self) -> bool:
        """云端 TTS 端口是否可达（DNS 解析 + TCP 连接）。"""
        if self.cloud_wait <= 0.0:
            return True
        try:
            with socket.create_connection((self.cloud_host, self.cloud_port), 2.0):
                return True
        except OSError:
            return False

    def _cache_file(self, text: str):
        """按文本内容定位本地缓存音频（sha1 前 16 位），未命中返回 None。"""
        if not self.cache_enabled or not self.cache_dir:
            return None
        key = hashlib.sha1(text.encode("utf-8")).hexdigest()[:16]
        path = os.path.join(self.cache_dir, key + ".wav")
        return path if os.path.isfile(path) else None

    @staticmethod
    def _wav_seconds(path: str) -> float:
        """读取 PCM wav 时长；无法解析时返回 0（仅影响进度与判定门槛）。"""
        try:
            with wave.open(path, "rb") as f:
                rate = f.getframerate()
                frames = f.getnframes()
            return float(frames) / rate if rate else 0.0
        except (OSError, wave.Error, EOFError):
            return 0.0

    def _call(self, cmd: str, text: str, audio_id: str = "",
              msg_type: str = ""):
        req = TtsService.Request()
        req.text = text
        req.type = msg_type or ("text" if text else "")
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

