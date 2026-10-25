"""消防巡检总控入口。

用法（在 src 目录下）：
    # 默认：离线模拟，不动机器人、不出声、不需要 ROS/底盘
    python3 chassis/run_inspection.py
    python3 chassis/run_inspection.py --scenario nav_fail --record /tmp/insp.jsonl

    # 实机（会移动机器人、会真实播报！必须显式 --real，且需二次确认：
    #   设置环境变量 INSPECTION_REAL_CONFIRM=1，或在交互终端输入 YES。
    #   未确认将安全退出（退出码 2），且不连接机器人、不触发任何硬件）
    INSPECTION_REAL_CONFIRM=1 python3 chassis/run_inspection.py --real

模拟场景（--scenario）：
    success          全流程正常（默认）
    nav_fail         POI_02 轨道导航失败
    nav_timeout      POI_02 轨道导航超时
    vision_abnormal  每个点都检出 smoking（模拟异常）
    vision_uncertain 每个点都低置信度（模拟不确定）
    vision_timeout   每个点视觉超时
    vision_no_frame  每个点取图失败
    speech_fail      所有 TTS 提交失败（验证不阻塞主流程）
    speech_timeout   所有 TTS 查询超时
    user_cancel      运行 0.4s 后外部线程请求取消
    return_start     正常流程 + 结束后返回起点

实机模式额外参数：
    --model PATH     消防模型权重（.pt/.onnx/.engine），缺省=视觉 NO_MODEL 拒绝
    --expect CLS...  期望检出类别（来自队友类别清单，缺省空=不启用真实视觉判定）
    --abnormal CLS   异常类别（如 smoking），可多次
    --poi-focus POI=CLASS[,CLASS]
                     显式指定某点位的专用检查目标（如 安全出口=safety_sign）；
                     可多次，未配置的点位沿用全局 --expect 口径
"""

import argparse
import os
import sys
import time

_SRC_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SRC_DIR not in sys.path:
    sys.path.insert(0, _SRC_DIR)

from inspection.controller import InspectionConfig, InspectionController, State
from inspection.recorder import InspectionRecorder
from inspection.report import parse_poi_focus
from inspection.types import now_iso


def _default_record_path() -> str:
    stamp = now_iso().replace(":", "-").replace("+", "_")
    return os.path.join("inspection_records", f"inspection_{stamp}.jsonl")


def _print_transition(state: State, detail: str = "") -> None:
    print(f"  [STATE] -> {state.value}" + (f"  ({detail})" if detail else ""),
          flush=True)


# 实机二次确认环境变量：必须显式且严格等于 "1" 才放行（无默认通过路径）
REAL_CONFIRM_ENV = "INSPECTION_REAL_CONFIRM"


def _confirm_real(cfg, args, input_fn=None) -> bool:
    """实机巡检二次确认门控。

    放行条件（任一满足）：
    1. 环境变量 INSPECTION_REAL_CONFIRM 严格等于 "1"；
    2. 交互终端中用户严格输入 "YES"。

    其余任何情况（变量未设置 / 值不符 / 输入错误 / 非交互终端 / EOF /
    输入异常）一律拒绝。确认前仅打印本次将实际使用的配置
    （全部来自 cfg/args 真实取值，不伪造），不接触任何硬件接口。
    """
    print("=" * 60, flush=True)
    print("!! 实机巡检确认：机器人将【实际移动】并【真实播报语音】 !!",
          flush=True)
    print("请确认现场人员安全、路径无障碍后再放行。", flush=True)
    print(f"  起点 POI : {cfg.start_poi}", flush=True)
    print(f"  巡检 POI : {list(cfg.pois)}", flush=True)
    print(f"  移动限速 : {args.max_speed} m/s", flush=True)
    print(f"  返回起点 : {'是' if cfg.return_to_start else '否'}", flush=True)
    print(f"  视觉模型 : {args.model or '未配置（视觉一律记 no_model）'}",
          flush=True)
    print(f"  期望类别 : {list(args.expect) if args.expect else '未配置'}",
          flush=True)
    print(f"  异常类别 : {list(args.abnormal) if args.abnormal else '未配置'}",
          flush=True)
    print(f"  专用点位 : {getattr(args, 'poi_focus_map', None) or '未配置'}",
          flush=True)
    print(f"  记录文件 : {args.record}", flush=True)
    print("=" * 60, flush=True)

    env_val = os.environ.get(REAL_CONFIRM_ENV, "").strip()
    if env_val == "1":
        print(f"已通过环境变量 {REAL_CONFIRM_ENV}=1 确认。", flush=True)
        return True

    if input_fn is None:
        isatty = getattr(sys.stdin, "isatty", None)
        if callable(isatty) and not isatty():
            print(f"[X] 非交互终端且未设置 {REAL_CONFIRM_ENV}=1，"
                  f"拒绝实机运行。", flush=True)
            return False
        input_fn = input
    try:
        ans = input_fn("确认以上配置无误并启动实机巡检？请输入 YES 继续: ")
    except EOFError:
        print(f"[X] 确认输入不可用（EOF/非交互终端），"
              f"请设置 {REAL_CONFIRM_ENV}=1 后重试。", flush=True)
        return False
    except Exception as exc:
        print(f"[X] 确认输入异常: {exc}，拒绝实机运行。", flush=True)
        return False
    if str(ans).strip() != "YES":
        print("[X] 未获得有效确认（需严格输入 YES），已安全退出。",
              flush=True)
        return False
    print("已确认，启动实机巡检。", flush=True)
    return True


# ── 模拟模式 ────────────────────────────────────────────────────────

def build_sim(scenario: str, poi_focus=None):
    from inspection.inspector import SimInspector
    from inspection.navigator import SimNavigator
    from inspection.speaker import SimSpeaker

    nav = SimNavigator(delay_s=0.02)
    vision = SimInspector(mode="normal", latency_s=0.02)
    spk = SimSpeaker(mode="ok", latency_s=0.01)
    cfg = InspectionConfig(dwell_seconds=0.05,  # 模拟用短停留
                           poi_focus_classes=dict(poi_focus or {}))

    if scenario == "nav_fail":
        nav = SimNavigator(fail_at="POI_02", outcome="failed", delay_s=0.02)
    elif scenario == "nav_timeout":
        nav = SimNavigator(fail_at="POI_02", outcome="timeout", delay_s=0.02)
    elif scenario == "vision_abnormal":
        vision = SimInspector(mode="abnormal", latency_s=0.02)
    elif scenario == "vision_uncertain":
        vision = SimInspector(mode="uncertain", latency_s=0.02)
    elif scenario == "vision_timeout":
        vision = SimInspector(mode="timeout", latency_s=0.02)
    elif scenario == "vision_no_frame":
        vision = SimInspector(mode="no_frame", latency_s=0.02)
    elif scenario == "speech_fail":
        spk = SimSpeaker(mode="submit_fail", latency_s=0.01)
    elif scenario == "speech_timeout":
        spk = SimSpeaker(mode="query_timeout", latency_s=0.01)
    elif scenario == "return_start":
        cfg = InspectionConfig(dwell_seconds=0.05, return_to_start=True,
                               poi_focus_classes=dict(poi_focus or {}))
    elif scenario == "user_cancel":
        pass  # 取消由 run_sim 里的线程注入
    elif scenario != "success":
        raise SystemExit(f"未知场景: {scenario}")

    return nav, spk, vision, cfg


def run_sim(args) -> int:
    nav, spk, vision, cfg = build_sim(
        args.scenario, getattr(args, "poi_focus_map", None))
    rec = InspectionRecorder(args.record, meta={
        "mode": "simulate", "scenario": args.scenario, "simulated": True,
        "pois": list(cfg.pois), "start_poi": cfg.start_poi,
        "poi_focus": {k: list(v) for k, v in cfg.poi_focus_classes.items()},
    })
    ctl = InspectionController(nav, spk, vision, rec, cfg,
                               on_transition=_print_transition)

    cancel_thread = None
    if args.scenario == "user_cancel":
        import threading
        cancel_thread = threading.Thread(
            target=lambda: (time.sleep(0.4), ctl.request_cancel()),
            daemon=True)

    print(f"=== 模拟巡检 scenario={args.scenario} record={args.record} ===",
          flush=True)
    if cancel_thread:
        cancel_thread.start()
    summary = ctl.run()
    if cancel_thread:
        cancel_thread.join(timeout=2.0)

    print("\n=== 汇总 ===", flush=True)
    print(f"  final_state: {summary.final_state.value}", flush=True)
    print(f"  detail: {summary.detail}", flush=True)
    print(f"  poi_finals: {summary.poi_finals}", flush=True)
    print(f"  nav.calls: {nav.calls}", flush=True)
    print(f"  speech.calls: {len(spk.calls)} 条", flush=True)
    print(f"  vision.calls: {vision.calls}", flush=True)
    return 0 if summary.final_state is State.DONE else 1


# ── 实机模式 ────────────────────────────────────────────────────────

def run_real(args) -> int:
    from inspection.inspector import SimInspector, YoloInspector
    from inspection.navigator import ChassisNavigator
    from inspection.speaker import make_tts_speaker

    poi_focus = dict(getattr(args, "poi_focus_map", None) or {})
    cfg = InspectionConfig(return_to_start=args.return_start,
                           poi_focus_classes=poi_focus)

    # 二次确认门控：未通过前不导入/连接 robot_controller，不触任何硬件
    if not _confirm_real(cfg, args):
        print("[X] 实机巡检未通过二次确认，已安全退出"
              "（未连接机器人、未下发任何指令）。", flush=True)
        return 2

    from robot_controller import robot_controller

    robot = robot_controller()
    try:
        nav = ChassisNavigator(robot, max_speed=args.max_speed)
        spk = make_tts_speaker(robot)

        if args.model:
            vision = YoloInspector(
                robot, model_path=args.model,
                expect_classes=args.expect, abnormal_classes=args.abnormal,
                poi_focus_classes=poi_focus)
        else:
            print("[!] 未提供 --model，视觉使用 NO_MODEL 拒绝桩"
                  "（每个点都会记为 no_model，不会误报正常）", flush=True)

            class NoModelInspector(SimInspector):
                def inspect(self, poi_name):
                    from inspection.types import InspectionStatus, PoiInspection
                    return PoiInspection(status=InspectionStatus.NO_MODEL,
                                         poi=poi_name,
                                         detail="未配置模型", simulated=False)
            vision = NoModelInspector(mode="no_frame")

        rec = InspectionRecorder(args.record, meta={
            "mode": "real", "simulated": False,
            "pois": list(cfg.pois), "start_poi": cfg.start_poi,
            "model": args.model,
            "poi_focus": {k: list(v) for k, v in poi_focus.items()},
        })
        ctl = InspectionController(nav, spk, vision, rec, cfg,
                                   on_transition=_print_transition)
        print("=== 实机巡检（会移动、会播报） ===", flush=True)
        summary = ctl.run()
        print(f"final_state={summary.final_state.value} "
              f"detail={summary.detail}", flush=True)
        return 0 if summary.final_state is State.DONE else 1
    finally:
        try:
            robot.stop()
        except Exception:
            pass


def main() -> int:
    ap = argparse.ArgumentParser(description="消防巡检总控（默认模拟模式）")
    ap.add_argument("--real", action="store_true",
                    help="实机模式：会移动机器人并真实播报，需定位就绪")
    ap.add_argument("--scenario", default="success", help="模拟场景名")
    ap.add_argument("--record", default=_default_record_path(),
                    help="JSONL 记录输出路径")
    ap.add_argument("--model", default="", help="实机模式：模型权重路径")
    ap.add_argument("--expect", nargs="*", default=[],
                    help="实机模式：期望检出类别清单")
    ap.add_argument("--abnormal", nargs="*", default=[],
                    help="实机模式：异常类别清单（如 smoking）")
    ap.add_argument("--max-speed", type=float, default=0.4)
    ap.add_argument("--return-start", action="store_true",
                    help="实机模式：结束后返回起点")
    ap.add_argument("--poi-focus", action="append", default=[],
                    metavar="POI=CLASS[,CLASS]",
                    help="显式指定某点位的专用检查目标，可多次；"
                         "未配置的点位沿用全局 --expect/--abnormal 口径")
    args = ap.parse_args()

    try:
        args.poi_focus_map = parse_poi_focus(args.poi_focus)
    except ValueError as exc:
        print(f"[X] --poi-focus 配置无效: {exc}", flush=True)
        return 2

    if args.real:
        return run_real(args)
    return run_sim(args)


if __name__ == "__main__":
    sys.exit(main())
