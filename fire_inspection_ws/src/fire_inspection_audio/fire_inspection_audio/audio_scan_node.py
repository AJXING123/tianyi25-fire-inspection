"""消防巡检听觉检测节点：本地音频指纹匹配防空警报。

每 hop_s 秒取一段 record_chunk_s 秒滑窗的麦克风音频，提取 MFCC 指纹，
和 config 里列出的样本做余弦相似度比对。连续命中 >= min_hits 帧时，
在 /fire_inspection/audio_alert 发一条 String（data 为样本纯名，如
'air_raid'）。冷却 cooldown_s 秒内不重复触发。

音频源（audio.source 配置）：
  - lyre       ：订阅平台头部 8 阵麦 /lyre/audio_stream（推荐，远场拾音）。
                 节点启动时调用 /lyre/audio_control 打开流，退出时关闭。
  - sounddevice：用 sounddevice 直采本地 USB 麦（audio.device 指定），兜底。

半双工：订阅 /fire_inspection/speaking，机器人自己播报期间不识别，
        避免把自己的 TTS 误当成警报。

发布：
  /fire_inspection/audio_alert  std_msgs/String   data = 命中的样本纯名
"""

from __future__ import annotations

import os
import threading
import time

import yaml
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

# lyre_msgs 是平台包（/home/nvidia/xos/lyre_msgs），source 不到时优雅降级
try:
    from lyre_msgs.msg import AudioFrame
    from lyre_msgs.srv import AudioControl
    _LYRE_OK = True
except ImportError:
    _LYRE_OK = False


def load_config():
    share = get_package_share_directory("fire_inspection_audio")
    path = os.path.join(share, "config", "audio.yaml")
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


try:
    import numpy as np
    import librosa
    _LIBS_OK = True
    _LIBS_MISSING = ""
except ImportError as exc:          # noqa: N816 - 记一条警告后优雅降级
    np = None  # type: ignore
    _LIBS_OK = False
    _LIBS_MISSING = str(exc)


def extract_mfcc(samples: "np.ndarray", sr: int) -> "np.ndarray":
    """MFCC 指纹向量（26 维：13 均值 + 13 标准差）。

    对警报的音色/时频结构建模，区分度远高于 chroma（chroma 噪声下
    sim 也≈0.94 无法区分）。实机巡检全程实测：静止环境 0.79~0.81、
    行走噪声 ≤0.839、手机放警报 0.84~0.893，阈值 0.85 + 响度门切开。
    """
    m = librosa.feature.mfcc(y=samples, sr=sr, n_mfcc=13)
    return np.concatenate([m.mean(axis=1), m.std(axis=1)])


def cosine_vec(a: "np.ndarray", b: "np.ndarray") -> float:
    """两 1-D 向量余弦相似度。"""
    na, nb = np.linalg.norm(a), np.linalg.norm(b)
    if na < 1e-9 or nb < 1e-9:
        return 0.0
    return float(np.dot(a, b) / (na * nb))


N_FFT = 2048
HOP = 512


def spectral_rolloff(samples: "np.ndarray", sr: int) -> float:
    """谱滚降点（Hz）：覆盖 85% 能量的最高频率，仅作参考日志。

    干净信号下区分度好（警报≈3000Hz vs 环境噪声≈5500Hz+），
    但低信噪比下噪声把能量抬到高频会失效，故不做硬门。
    """
    S = np.abs(librosa.stft(samples, n_fft=N_FFT, hop_length=HOP))
    return float(librosa.feature.spectral_rolloff(S=S, sr=sr).mean())


def spectral_concentration(S: "np.ndarray", sr: int) -> float:
    """谱集中度：每帧 300~3000Hz 带内 top-3 频点功率占带内总功率比，取时间均值。

    物理门：警报是窄带扫频音（能量集中在一两个滑动峰上），语音/宽带噪声
    能量摊开。实机实测（2026-10-30）：警报样本 0.406，当前环境人声
    0.084~0.091，差 4.5 倍；而 MFCC 对人声会给出 0.86+ 的高 sim（与警报
    重叠），必须靠这道门挡住。
    """
    freqs = librosa.fft_frequencies(sr=sr, n_fft=(S.shape[0] - 1) * 2)
    band = (freqs >= 300) & (freqs <= 3000)
    Sb = np.abs(S[band]) ** 2
    total = Sb.sum(axis=0) + 1e-12
    top3 = np.sort(Sb, axis=0)[-3:].sum(axis=0)
    return float((top3 / total).mean())


class AudioScanNode(Node):
    def __init__(self):
        super().__init__("fire_audio_scan")
        cfg = load_config()
        au = cfg.get("audio") or {}

        self.declare_parameter("dry_run", bool(au.get("dry_run", False)))
        self.dry_run = bool(self.get_parameter("dry_run").value)

        self.sr = int(au.get("sample_rate", 16000))
        self.channels = int(au.get("channels", 1))
        self.device = au.get("device", "") or None
        self.source = str(au.get("source", "lyre")).lower()
        self.chunk_s = float(au.get("record_chunk_s", 2.0))
        self.hop_s = float(au.get("hop_s", 0.5))
        self.threshold = float(au.get("match_threshold", 0.86))
        self.min_conc = float(au.get("min_band_concentration", 0.20))
        _pb = au.get("peak_band_hz") or [300, 3000]
        self.peak_lo, self.peak_hi = float(_pb[0]), float(_pb[1])
        self.min_rms = float(au.get("min_rms", 0.008))
        self.cooldown_s = float(au.get("cooldown_s", 15.0))
        self.min_hits = int(au.get("min_hits", 5))
        self.speaking_guard_s = float(au.get("speaking_guard_s", 1.5))
        # lyre 打开流时的可选 cae 模式，留空用平台默认
        self.cae_mode = str(au.get("cae_mode", "") or "")

        self._muted_ticks = 0         # 半双工屏蔽日志降频
        self._quiet_ticks = 0         # 安静跳过日志降频
        self._mute_until = 0.0
        self._stream_opened = False   # 本节点是否打开了 lyre 流（负责关闭）

        # 加载样本（MFCC 指纹，启动时一次加载，运行时免磁盘 I/O）
        share = get_package_share_directory("fire_inspection_audio")
        self.samples: dict[str, "np.ndarray"] = {}
        self._display_names: dict[str, str] = {}
        for name in au.get("samples") or []:
            path = os.path.join(share, "config", name)
            if not os.path.isfile(path):
                self.get_logger().warn(f"样本文件不存在：{path}，跳过")
                continue
            try:
                y, _ = librosa.load(path, sr=self.sr, mono=True)
                # 关键：trim 掉样本头尾静音，只保留有声主体段参与比对，
                # 否则静音帧会稀释指纹导致匹配不稳。
                y_trimmed, (start, end) = librosa.effects.trim(
                    y, top_db=18, frame_length=2048, hop_length=512)
                if len(y_trimmed) < self.sr // 2:
                    self.get_logger().warn(f"样本 {name} 有效声音太短，跳过")
                    continue
                self.samples[name] = extract_mfcc(y_trimmed, self.sr)
                # 展示名：去掉 .wav 后缀，让编排层 SPEECH_TEXTS 按纯名命中
                display = os.path.splitext(name)[0]
                self._display_names[name] = display
                self.get_logger().info(
                    f"加载样本 {name} → trim [{start/self.sr:.2f}s, {end/self.sr:.2f}s]"
                    f" mfcc 指纹 {self.samples[name].shape} → 展示名 '{display}'")
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f"加载样本 {name} 失败：{exc}")

        # 运行时状态
        self._ring = np.zeros(int(self.chunk_s * self.sr), dtype=np.float32)
        self._ring_lock = threading.Lock()
        self._hits: dict[str, int] = {k: 0 for k in self.samples}
        self._last_alert: dict[str, float] = {}

        self._pub = self.create_publisher(String,
                                          "/fire_inspection/audio_alert", 10)

        if not _LIBS_OK:
            self.get_logger().error(
                f"依赖库缺失：{_LIBS_MISSING}，听觉节点无法工作。"
                f" 请执行 pip install librosa numpy sounddevice")
            return

        if not self.samples:
            self.get_logger().error(
                "没有加载到任何有效样本，听觉节点无法触发任何警报。"
                " 请在 config/audio.yaml 的 samples 下配置 .wav 文件")
            return

        # 半双工：机器人自己播报期间不识别
        self.create_subscription(Bool, "/fire_inspection/speaking",
                                 self._on_speaking, 10)

        # 启动音频采集源
        if not self._start_source(au):
            return

        # 定时器：每隔 hop_s 秒取 ring 末尾 chunk_s 做一次匹配
        self._timer = self.create_timer(max(0.05, self.hop_s), self._scan_tick)

    # ---------------- 音频源 ----------------
    def _start_source(self, au) -> bool:
        if self.source == "lyre":
            return self._start_lyre(au)
        return self._start_sounddevice()

    def _start_lyre(self, au) -> bool:
        if not _LYRE_OK:
            self.get_logger().error(
                "source=lyre 但 lyre_msgs 不可见（未 source /home/nvidia/xos/"
                "setup.bash），听觉节点无法工作。")
            return False
        # 打开平台头部麦音频流
        cli = self.create_client(AudioControl, "/lyre/audio_control")
        if not cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("/lyre/audio_control 服务不可达，平台语音未就绪")
            return False
        req = AudioControl.Request()
        req.enable = True
        if self.cae_mode:
            try:
                req.cae_mode = self.cae_mode
            except Exception:  # noqa: BLE001 - 老固件无此字段
                pass
        try:
            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
            if fut.result() is not None and not fut.result().success:
                self.get_logger().warn(f"打开 lyre 音频流返回失败：{fut.result()}")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"调用 /lyre/audio_control 异常：{exc}（仍尝试订阅）")
        self._stream_opened = True
        # 订阅音频流，QoS 用 best_effort 适配高帧率传感器
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(AudioFrame, "/lyre/audio_stream",
                                 self._on_audio_frame, qos)
        self.get_logger().info(
            f"听觉源=lyre 头部麦 /lyre/audio_stream sr={self.sr} "
            f"chunk={self.chunk_s}s hop={self.hop_s}s 阈值={self.threshold} "
            f"dry_run={self.dry_run} 样本={list(self.samples.keys())}")
        return True

    def _start_sounddevice(self) -> bool:
        try:
            import sounddevice as sd
        except ImportError:
            self.get_logger().error("sounddevice 未安装，无法直采 USB 麦")
            return False
        try:
            self._stream = sd.InputStream(
                samplerate=self.sr, channels=1, dtype="float32",
                device=self.device, blocksize=0, latency="low",
                callback=self._sd_cb)
            self._stream.start()
            self.get_logger().info(
                f"听觉源=sounddevice device={self.device or 'default'} sr={self.sr}"
                f" dry_run={self.dry_run} 样本={list(self.samples.keys())}")
            return True
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f"sounddevice 录音启动失败：{exc}")
            return False

    # ---------------- 音频回调 ----------------
    def _on_audio_frame(self, msg) -> None:
        """lyre 头部麦帧：16bit 单声道，拼入环形缓冲。"""
        try:
            a = np.frombuffer(bytes(msg.data), dtype=np.int16).astype(np.float32) / 32768.0
            if msg.channels > 1:
                a = a.reshape(-1, msg.channels).mean(axis=1)
            if a.size == 0:
                return
            with self._ring_lock:
                n = len(a)
                self._ring = np.roll(self._ring, -n)
                self._ring[-n:] = a
        except Exception:  # noqa: BLE001 - 回调里不抛
            pass

    def _sd_cb(self, indata, _frames, _time, status):
        del status
        with self._ring_lock:
            n = len(indata)
            self._ring = np.roll(self._ring, -n)
            self._ring[-n:] = indata[:, 0]

    # ---------------- 半双工 ----------------
    def _on_speaking(self, msg) -> None:
        if msg.data:
            self._mute_until = time.monotonic() + 60.0
        else:
            self._mute_until = time.monotonic() + self.speaking_guard_s

    # ---------------- 定时扫描 ----------------
    def _scan_tick(self):
        if not self.samples:
            return
        # 半双工屏蔽：机器人自己正在播报
        if time.monotonic() < self._mute_until:
            self._muted_ticks += 1
            if self._muted_ticks % 8 == 1:
                self.get_logger().info("[audio] 机器人在播报，暂停听觉识别（半双工）")
            return
        self._muted_ticks = 0

        with self._ring_lock:
            frame = self._ring.copy()

        # 门限 1：响度门，只挡数字静音。被挡时也降频打日志，
        # 避免节点"哑巴"（2026-10-30 教训：0.025 门曾把整条链路静默）。
        rms = float(np.sqrt(np.mean(frame ** 2)))
        if rms < self.min_rms:
            self._quiet_ticks += 1
            if self._quiet_ticks % 8 == 1:
                self.get_logger().info(
                    f"[audio] 环境安静 rms={rms:.5f} < {self.min_rms}，跳过")
            return
        self._quiet_ticks = 0

        now = time.monotonic()
        # 一次 STFT 同时出 rolloff（参考）、谱集中度、主峰频带（两道门）
        S = np.abs(librosa.stft(frame, n_fft=N_FFT, hop_length=HOP))
        rolloff = float(librosa.feature.spectral_rolloff(S=S, sr=self.sr).mean())
        conc = spectral_concentration(S, self.sr)
        freqs = librosa.fft_frequencies(sr=self.sr, n_fft=N_FFT)
        peaks = freqs[np.argmax(S, axis=0)]
        peak_med = float(np.median(peaks))
        frame_mfcc = extract_mfcc(frame, self.sr)
        for name, samp_mfcc in self.samples.items():
            display = self._display_names.get(name, name)
            sim = cosine_vec(frame_mfcc, samp_mfcc)
            # 三门判定：MFCC 指纹 + 谱集中度（窄带音）+ 主峰频带（警报扫频在中频）
            ok = (sim >= self.threshold and conc >= self.min_conc
                  and self.peak_lo <= peak_med <= self.peak_hi)
            self.get_logger().info(
                f"[audio] {display} mfcc_sim={sim:.3f} conc={conc:.3f} "
                f"peak={peak_med:.0f}Hz rolloff={rolloff:.0f}Hz rms={rms:.5f} "
                f"阈值={self.threshold}/{self.min_conc}/{self.peak_lo}-{self.peak_hi}Hz "
                f"连续={self._hits[name]}")
            if ok:
                self._hits[name] += 1
            else:
                self._hits[name] = 0
            if self._hits[name] >= self.min_hits:
                last = self._last_alert.get(name, 0.0)
                if now - last >= self.cooldown_s:
                    self._last_alert[name] = now
                    self._hits[name] = 0
                    self._fire(display, sim)

    def _fire(self, display: str, sim: float) -> None:
        if self.dry_run:
            self.get_logger().warn(
                f"[audio] 命中 {display}（dry_run，不发 alert） sim={sim:.3f}")
            return
        msg = String()
        msg.data = display
        self._pub.publish(msg)
        self.get_logger().warn(
            f"[audio] 命中 {display} → 已发 /fire_inspection/audio_alert sim={sim:.3f}")

    def destroy_node(self):
        # 退出时关闭本节点打开的 lyre 流，恢复平台语音模式
        if getattr(self, "_stream_opened", False) and _LYRE_OK:
            try:
                cli = self.create_client(AudioControl, "/lyre/audio_control")
                if cli.wait_for_service(timeout_sec=2.0):
                    req = AudioControl.Request()
                    req.enable = False
                    fut = cli.call_async(req)
                    rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
            except Exception:  # noqa: BLE001
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AudioScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
