"""TTS 语音适配层。

真实链路：/intelligent_interaction/tts/play (interaction_msgs/srv/TtsService)
    → audio_node → /play_content/tts/play → adaptive_tts。

已核实的字段：Request{text, type, cmd, audio_id, extra} → Response{success, status}。
已核实的 cmd：stop=打断并播 / append=排队 / query=查询播放状态。

已知限制（如实声明，不伪造确定性）：
1. success=true 只表示 audio_node 接受并转发请求（audio_manager 日志
   decision=1 后即返回），不代表合成成功、不代表播放完成。
2. query 返回的是全局播放状态（日志观察到 playing/idle 两种），
   无法严格关联到「本条请求」：队列里有多条时，idle 可能是别条播完；
   从未观测到 playing 也可能是瞬播完 / 被高优先级抢占 / 实际未合成。
   因此 wait_done 只能给出 SpeechState 级别的可信结论。
3. query 响应 status 的具体字符串集合未逐一实测，本实现对
   "playing" 之外的值一律按「非播放中」处理并记录原始值。
"""

from __future__ import annotations

import time

from .types import SpeechResult, SpeechState

TTS_SERVICE = "/intelligent_interaction/tts/play"
# audio_config.json 中定义：audio_guide priority=4 interruptible=true（巡检播报用）
DEFAULT_AUDIO_ID = "audio_guide"

QUERY_POLL_S = 0.2
PLAYING_KEYWORD = "playing"  # 防御式匹配：status 含该子串才认为在播


class SpeakerBase:
    """统一接口：say(text, interrupt) -> SpeechResult。"""

    def say(self, text: str, interrupt: bool = True) -> SpeechResult:
        raise NotImplementedError

    def close(self) -> None:
        pass


# ── 模拟实现 ────────────────────────────────────────────────────────

class SimSpeaker(SpeakerBase):
    """离线模拟 TTS。mode: ok / submit_fail / query_timeout / never_playing。"""

    def __init__(self, mode: str = "ok", latency_s: float = 0.02):
        self.mode = mode
        self.latency_s = latency_s
        self.calls: list = []  # [(text, interrupt)]，供测试断言

    def say(self, text: str, interrupt: bool = True) -> SpeechResult:
        self.calls.append((text, interrupt))
        t0 = time.monotonic()
        time.sleep(self.latency_s)
        elapsed = time.monotonic() - t0

        if self.mode == "submit_fail":
            return SpeechResult(
                state=SpeechState.SUBMIT_FAILED, text=text,
                submit_success=False, submit_status="simulated submit failure",
                elapsed_s=elapsed, simulated=True,
            )
        if self.mode == "query_timeout":
            return SpeechResult(
                state=SpeechState.QUERY_TIMEOUT, text=text,
                submit_success=True, submit_status="success",
                elapsed_s=elapsed, simulated=True,
            )
        if self.mode == "never_playing":
            return SpeechResult(
                state=SpeechState.SUBMIT_UNCONFIRMED, text=text,
                submit_success=True, submit_status="success",
                elapsed_s=elapsed, simulated=True,
            )
        return SpeechResult(
            state=SpeechState.DONE_OBSERVED, text=text,
            submit_success=True, submit_status="success",
            observed_transition=True, elapsed_s=elapsed, simulated=True,
        )


# ── 真实实现 ────────────────────────────────────────────────────────

class TtsSpeaker(SpeakerBase):
    """走 /intelligent_interaction/tts/play 的真实播报。

    robot 是 robot_controller 单例（其 impl 是 rclpy Node，executor 在后台
    线程 spinning），因此 call_async 的 future 不需要主线程 spin。
    """

    def __init__(self, robot, service_name: str = TTS_SERVICE,
                 audio_id: str = DEFAULT_AUDIO_ID,
                 done_timeout_s: float = 30.0,
                 service_wait_s: float = 5.0):
        from interaction_msgs.srv import TtsService  # 延迟导入，模拟模式无 ROS

        self._robot = robot
        self._audio_id = audio_id
        self._done_timeout_s = done_timeout_s
        self._cli = robot.create_client(TtsService, service_name)
        if not self._cli.wait_for_service(timeout_sec=service_wait_s):
            raise RuntimeError(f"TTS 服务不可用: {service_name}")

    def _call(self, text: str, cmd: str):
        """同步等待 call_async 完成，返回 (ok, response_or_exc)。"""
        req = self._make_request(text, cmd)
        future = self._cli.call_async(req)
        deadline = time.monotonic() + 10.0
        while not future.done():
            if time.monotonic() > deadline:
                return False, TimeoutError("TTS service call timeout")
            time.sleep(0.02)
        try:
            return True, future.result()
        except Exception as exc:
            return False, exc

    def _make_request(self, text: str, cmd: str):
        raise NotImplementedError  # 由 _make 注入，避免持有 srv 模块引用

    def _is_playing(self) -> tuple:
        """query 当前播放状态。返回 (ok, is_playing, raw_status)。"""
        ok, resp = self._call("", "query")
        if not ok:
            return False, False, str(resp)
        status = (getattr(resp, "status", "") or "").strip()
        return True, PLAYING_KEYWORD in status.lower(), status

    def say(self, text: str, interrupt: bool = True) -> SpeechResult:
        t0 = time.monotonic()
        cmd = "stop" if interrupt else "append"

        ok, resp = self._call(text, cmd)
        if not ok:
            return SpeechResult(
                state=SpeechState.SUBMIT_FAILED, text=text,
                submit_success=False, submit_status=f"call exception: {resp}",
                elapsed_s=time.monotonic() - t0,
            )
        if not getattr(resp, "success", False):
            return SpeechResult(
                state=SpeechState.SUBMIT_FAILED, text=text,
                submit_success=False,
                submit_status=getattr(resp, "status", ""),
                elapsed_s=time.monotonic() - t0,
            )

        submit_status = getattr(resp, "status", "")
        # 提交成功 ≠ 在播。轮询 query 观察 playing→非playing 跳变。
        saw_playing = False
        deadline = time.monotonic() + self._done_timeout_s
        while time.monotonic() < deadline:
            ok, playing, raw = self._is_playing()
            if not ok:
                return SpeechResult(
                    state=SpeechState.QUERY_ERROR, text=text,
                    submit_success=True, submit_status=submit_status,
                    elapsed_s=time.monotonic() - t0,
                )
            if playing:
                saw_playing = True
            elif saw_playing:
                return SpeechResult(
                    state=SpeechState.DONE_OBSERVED, text=text,
                    submit_success=True, submit_status=submit_status,
                    observed_transition=True,
                    elapsed_s=time.monotonic() - t0,
                )
            time.sleep(QUERY_POLL_S)

        if saw_playing:
            return SpeechResult(
                state=SpeechState.QUERY_TIMEOUT, text=text,
                submit_success=True, submit_status=submit_status,
                observed_transition=True,
                elapsed_s=time.monotonic() - t0,
            )
        return SpeechResult(
            state=SpeechState.SUBMIT_UNCONFIRMED, text=text,
            submit_success=True, submit_status=submit_status,
            elapsed_s=time.monotonic() - t0,
        )


def make_tts_speaker(robot, **kwargs) -> TtsSpeaker:
    """工厂：补全 _make_request（延迟绑定 srv 类型）。"""
    from interaction_msgs.srv import TtsService

    speaker = TtsSpeaker(robot, **kwargs)

    def _make_request(text: str, cmd: str):
        req = TtsService.Request()
        req.text = text
        req.type = "text"
        req.cmd = cmd
        req.audio_id = speaker._audio_id
        req.extra = ""
        return req

    speaker._make_request = _make_request
    return speaker
