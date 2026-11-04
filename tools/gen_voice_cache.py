#!/usr/bin/env python3
"""生成离线语音缓存：云端 TTS 合成 → 录默认声卡 monitor → 裁静音 → 存 wav。

背景：POI_02/POI_03 位置 WiFi 丢路由，云端合成不可达，平台只播 1.8s
“音频生成失败”提示音。巡检文案是固定的模板句，提前在有网环境下合成并
录成 wav 后，speak_action_server 命中缓存即用 type=file 离线播放。

用法（需先 source ROS 与工作区环境）：
    source /opt/ros/jazzy/setup.bash
    source /home/nvidia/fire-inspection/fire_inspection_ws/install/setup.bash
    python3 tools/gen_voice_cache.py            # 只补缺失项
    python3 tools/gen_voice_cache.py --force    # 全部重录
    python3 tools/gen_voice_cache.py --list     # 只打印待生成文案

录到的音频会通过扬声器外放，生成前请确认现场无其它播报干扰。
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import signal
import subprocess
import sys
import tempfile
import time
import wave

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from interaction_msgs.srv import TtsService

RATE = 44100
CHANNELS = 2


# ---------------- 文案枚举 ----------------
class _Result:
    """模拟 PoiResult：build_speech_text 只需 status + detections。"""

    def __init__(self, status, detections):
        self.status = int(status)
        self.detections = detections


def _det(cls: str, conf: float = 0.9) -> dict:
    return {"class_name": cls, "confidence": conf}


def collect_texts(script_cfg: dict) -> list:
    """枚举巡检流程所有可能的播报文案（与 speech_texts 逻辑完全一致）。"""
    from fire_inspection_voice.speech_texts import (
        CLASS_CN, PoiStatus, SPEECH_TEXTS, build_speech_text)

    texts = list(SPEECH_TEXTS.values())
    # 行进中检出目标（orchestrator._watch_speak_and_resume）
    texts += [f"发现目标：{cn}" for cn in CLASS_CN.values()]

    conf_by_class = script_cfg.get("conf_by_class") or {}
    for poi in script_cfg.get("pois") or []:
        name = str(poi.get("name") or "")
        if not name:
            continue
        expect = list(poi.get("expect_classes") or [])
        abnormal = list(poi.get("abnormal_classes") or [])
        focus = list(poi.get("focus_classes") or [])
        subsets = [tuple(expect), ()] + [(c,) for c in expect]
        for sub in subsets:
            dets = [_det(c) for c in sub]
            for status in (PoiStatus.NORMAL, PoiStatus.UNCERTAIN):
                texts.append(build_speech_text(
                    name, _Result(status, dets), expect, abnormal,
                    conf_by_class=conf_by_class, focus_classes=focus))
            # 异常：仅检出异常类 / 异常类 + 目标子集
            texts.append(build_speech_text(
                name, _Result(PoiStatus.ABNORMAL,
                              dets + [_det(a) for a in abnormal]),
                expect, abnormal, conf_by_class=conf_by_class,
                focus_classes=focus))
            texts.append(build_speech_text(
                name, _Result(PoiStatus.ABNORMAL, dets), expect, abnormal,
                conf_by_class=conf_by_class, focus_classes=focus))
        for status in (PoiStatus.TIMEOUT, PoiStatus.NO_FRAME,
                       PoiStatus.NO_MODEL, PoiStatus.ERROR):
            texts.append(build_speech_text(
                name, _Result(status, []), expect, abnormal,
                conf_by_class=conf_by_class, focus_classes=focus))

    seen, unique = set(), []
    for t in texts:
        t = (t or "").strip()
        if t and t not in seen:
            seen.add(t)
            unique.append(t)
    return unique


# ---------------- TTS 调用 ----------------
class TtsClient:
    def __init__(self, service: str, audio_id: str):
        self.node = rclpy.create_node("gen_voice_cache")
        self.cli = self.node.create_client(TtsService, service)
        self.audio_id = audio_id

    def wait_ready(self) -> bool:
        return self.cli.wait_for_service(timeout_sec=10.0)

    def call(self, cmd: str, text: str = "", msg_type: str = "", timeout: float = 8.0):
        req = TtsService.Request()
        req.cmd = cmd
        req.text = text
        req.type = msg_type or ("text" if text else "")
        req.audio_id = self.audio_id
        req.extra = ""
        future = self.cli.call_async(req)
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                return None
            rclpy.spin_once(self.node, timeout_sec=0.05)
        try:
            return future.result()
        except Exception as exc:  # noqa: BLE001 现场诊断用
            print(f"  ! TTS {cmd} 调用异常: {exc}")
            return None

    def status(self) -> str:
        resp = self.call("query", timeout=5.0)
        return str(getattr(resp, "status", "")) if resp is not None else ""

    def wait_idle(self, timeout: float = 20.0) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.status() != "playing":
                return True
            time.sleep(0.2)
        return False

    def wait_played(self, timeout: float = 25.0,
                    idle_need: float = 2.5) -> bool:
        """等本次播报结束；返回是否真的观测到播放。

        云端合成中途会有网络停顿，状态可能短暂变为非 playing；若按 0.6s
        空闲判结束会把后半句切掉（实测只录到约 70%）。这里要求连续
        idle_need 秒非 playing 才认为整句播完。
        """
        deadline = time.monotonic() + timeout
        seen = False
        idle_since = None
        while time.monotonic() < deadline:
            if self.status() == "playing":
                seen = True
                idle_since = None
            elif seen:
                if idle_since is None:
                    idle_since = time.monotonic()
                elif time.monotonic() - idle_since >= idle_need:
                    return True
            time.sleep(0.15)
        return seen


# ---------------- 录音与裁剪 ----------------
def start_record(path: str):
    """录默认 sink 的 monitor：输出裸 PCM。

    不用 --file-format=wav：parec 被中断时不会回填 wav 头里的帧数，
    sox 会把整段录音当成 0 采样点读入。
    """
    return subprocess.Popen([
        "parec", "--device=@DEFAULT_MONITOR@",
        f"--rate={RATE}", f"--channels={CHANNELS}", "--format=s16le", path,
    ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def stop_record(proc) -> None:
    if proc.poll() is None:
        proc.send_signal(signal.SIGINT)
        try:
            proc.wait(timeout=3)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=2)


def trim(src: str, dst: str) -> bool:
    """裸 PCM -> 44.1k 立体声 16bit wav，并两端去静音（-45dB/0.1s）。

    SoX 不支持用 ':' 串联效果：写成 `reverse : silenceremove ...` 时只有
    第一个 reverse 生效，会把整段音频倒放（缓存播出来就是"乱码"）。
    正确写法是把效果依次列出，用 silence 效果两端裁剪：
        silence(前) -> reverse -> silence(原尾部) -> reverse
    """
    cmd = [
        "sox", "-t", "raw", "-r", str(RATE), "-c", str(CHANNELS),
        "-e", "signed-integer", "-b", "16", src, dst,
        "silence", "1", "0.1", "-45dB",
        "reverse", "silence", "1", "0.1", "-45dB", "reverse",
    ]
    proc = subprocess.run(cmd, capture_output=True, text=True)
    if proc.returncode != 0:
        print(f"  ! sox 裁剪失败: {proc.stderr.strip()[:200]}")
        return False
    return True


def wav_seconds(path: str) -> float:
    try:
        with wave.open(path, "rb") as f:
            rate, frames = f.getframerate(), f.getnframes()
        return float(frames) / rate if rate else 0.0
    except (OSError, wave.Error, EOFError):
        return 0.0


def expected_seconds(text: str) -> float:
    return max(1.0, len(text) * 0.22)


def gen_one(tts: TtsClient, text: str, out_path: str, force: bool) -> bool:
    if os.path.isfile(out_path) and not force:
        return True
    for attempt in (1, 2):
        if not tts.wait_idle():
            print("  ! 平台一直在播报，跳过本次尝试")
            return False
        tmp_wav = tempfile.mktemp(suffix=".pcm")
        proc = start_record(tmp_wav)
        time.sleep(0.5)
        resp = tts.call("append", text, "text", timeout=10.0)
        if resp is None or not resp.success:
            print(f"  ! append 失败（第 {attempt} 次）")
            stop_record(proc)
            os.path.exists(tmp_wav) and os.unlink(tmp_wav)
            continue
        played = tts.wait_played()
        time.sleep(0.3)
        stop_record(proc)
        if not played:
            print(f"  ! 未观测到播放（第 {attempt} 次）")
            os.path.exists(tmp_wav) and os.unlink(tmp_wav)
            continue
        if not trim(tmp_wav, out_path):
            os.path.exists(tmp_wav) and os.unlink(tmp_wav)
            continue
        os.path.exists(tmp_wav) and os.unlink(tmp_wav)
        seconds = wav_seconds(out_path)
        exp = expected_seconds(text)
        # 兜底失败音固定 1.8s；过短/过长都判为无效：过长通常是把上一段
        # 音频尾巴或同一句重复播放也录了进来（实测语速 0.22~0.28 s/字）
        if seconds < 1.2 or not (exp * 0.5 <= seconds <= exp * 1.7 + 0.8):
            print(f"  ! 时长异常 {seconds:.1f}s（预期约 {exp:.1f}s），"
                  f"疑似失败提示音或录音污染（第 {attempt} 次）")
            os.unlink(out_path)
            continue
        return True
    return False


def main() -> int:
    ap = argparse.ArgumentParser(description="生成巡检离线语音缓存")
    ap.add_argument("--dir", default="/home/nvidia/fire-inspection/audio_cache")
    ap.add_argument("--force", action="store_true", help="已存在也重新录制")
    ap.add_argument("--list", action="store_true", help="只列出待生成文案")
    ap.add_argument("--tts-service", default="/intelligent_interaction/tts/play")
    ap.add_argument("--audio-id", default="audio_guide")
    args = ap.parse_args()

    share = get_package_share_directory("fire_inspection_voice")
    voice_cfg = yaml.safe_load(
        open(os.path.join(share, "config", "voice.yaml"), encoding="utf-8")) or {}
    cache_dir = (voice_cfg.get("audio_cache") or {}).get("dir") or args.dir
    os.makedirs(cache_dir, exist_ok=True)

    script_path = "/home/nvidia/fire-inspection/fire_inspection_ws/src/" \
                  "fire_inspection_orchestrator/config/inspection_script.yaml"
    script_cfg = yaml.safe_load(open(script_path, encoding="utf-8")) or {}
    texts = collect_texts(script_cfg)

    def key(t: str) -> str:
        return hashlib.sha1(t.encode("utf-8")).hexdigest()[:16] + ".wav"

    todo = [t for t in texts
            if args.force or not os.path.isfile(os.path.join(cache_dir, key(t)))]
    print(f"缓存目录: {cache_dir}")
    print(f"文案总数 {len(texts)}，待生成 {len(todo)}")
    if args.list:
        for t in todo:
            print(f"  {key(t)}  {t}")
        return 0

    rclpy.init()
    tts = TtsClient(args.tts_service, args.audio_id)
    try:
        if not tts.wait_ready():
            print(f"! TTS 服务 {args.tts_service} 未就绪")
            return 1
        manifest_path = os.path.join(cache_dir, "manifest.json")
        manifest = {}
        if os.path.isfile(manifest_path):
            try:
                manifest = json.load(open(manifest_path, encoding="utf-8"))
            except (OSError, ValueError):
                manifest = {}

        ok, failed = 0, []
        for i, text in enumerate(todo, 1):
            out = os.path.join(cache_dir, key(text))
            print(f"[{i}/{len(todo)}] {text}")
            if gen_one(tts, text, out, args.force):
                manifest[key(text)] = {
                    "text": text,
                    "file": os.path.basename(out),
                    "seconds": round(wav_seconds(out), 2),
                }
                with open(manifest_path, "w", encoding="utf-8") as f:
                    json.dump(manifest, f, ensure_ascii=False, indent=2)
                ok += 1
                print(f"  ✓ {wav_seconds(out):.1f}s")
            else:
                failed.append(text)
                print("  ✗ 生成失败")
        print(f"\n完成：成功 {ok}，失败 {len(failed)}，缓存目录 {cache_dir}")
        for t in failed:
            print(f"  失败: {t}")
        return 0 if not failed else 2
    finally:
        tts.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
