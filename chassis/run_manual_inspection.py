"""人工遥控单点巡检闭环入口（视觉 + 语音 + 记录，不含任何底盘控制）。

工作流：
    人工遥控机器人到点 → 终端按回车触发 → 视觉识别 → 生成中文播报文本
    → TTS 播报 → JSONL 落盘（点位/视觉/语音/异常）。

安全约定：
- 本脚本绝不调用 Navigator / 底盘接口，不影响人工遥控；
- 语音失败只记录、不阻塞；视觉任何非正常态都不会被播报成"正常"；
- 检出 smoking 只播"发现吸烟行为"，不播报"发生火灾"；
- 默认离线模拟（SimInspector + SimSpeaker），不连接机器人、不出声；
- --real 需二次确认（环境变量 INSPECTION_REAL_CONFIRM=1 或终端输入 YES），
  确认前不创建机器人实例、不触任何硬件。

用法（在 src 目录下）：
    # 离线模拟
    python3 chassis/run_manual_inspection.py
    python3 chassis/run_manual_inspection.py --scenario abnormal

    # 实机（触发相机取图 + YOLO 推理 + 真实 TTS 播报，不动底盘）
    INSPECTION_REAL_CONFIRM=1 python3 chassis/run_manual_inspection.py --real
"""

import argparse
import os
import sys

_SRC_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SRC_DIR not in sys.path:
    sys.path.insert(0, _SRC_DIR)
_REPO_ROOT = os.path.dirname(_SRC_DIR)

from inspection.recorder import InspectionRecorder
from inspection.report import build_speech_text
from inspection.types import PoiRecord, SpeechResult, SpeechState, now_iso

REAL_CONFIRM_ENV = "INSPECTION_REAL_CONFIRM"
DEFAULT_MODEL = os.path.join(_REPO_ROOT, "models", "fire", "best.pt")


def _confirm_real(args, input_fn=None) -> bool:
    """实机二次确认门控：与 run_inspection.py 同一约定，无默认通过路径。"""
    print("=" * 60, flush=True)
    print("!! 实机单点巡检确认：将【真实取图 + 推理 + 播报语音】!!", flush=True)
    print("本脚本不会控制底盘，请保持人工遥控接管。", flush=True)
    print(f"  视觉模型 : {args.model}", flush=True)
    print(f"  期望类别 : {list(args.expect)}", flush=True)
    print(f"  异常类别 : {list(args.abnormal)}", flush=True)
    print(f"  吸烟阈值 : {args.conf_smoking}  其他阈值 : {args.conf_other}",
          flush=True)
    print(f"  记录文件 : {args.record}", flush=True)
    print("=" * 60, flush=True)

    if os.environ.get(REAL_CONFIRM_ENV, "").strip() == "1":
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
        ans = input_fn("确认以上配置无误？请输入 YES 继续: ")
    except Exception as exc:
        print(f"[X] 确认输入不可用（{type(exc).__name__}），拒绝实机运行。",
              flush=True)
        return False
    if str(ans).strip() != "YES":
        print("[X] 未获得有效确认（需严格输入 YES），已安全退出。", flush=True)
        return False
    print("已确认，启动实机单点巡检。", flush=True)
    return True


class _NullSpeaker:
    """TTS 服务不可用时的保底：播报一律记为失败，绝不影响视觉与记录。"""

    def __init__(self, reason: str):
        self.reason = reason

    def say(self, text, interrupt=True) -> SpeechResult:
        return SpeechResult(state=SpeechState.SUBMIT_FAILED, text=text,
                            submit_success=False, submit_status=self.reason)

    def close(self) -> None:
        pass


def _build_sim(args):
    from inspection.inspector import SimInspector
    from inspection.speaker import SimSpeaker

    vision_modes = {"normal", "abnormal", "uncertain", "timeout", "no_frame"}
    speech_modes = {"speech_fail", "speech_timeout"}
    # 用真实类名做模拟检出，使模拟播报文案与实机口径一致
    expect = tuple(getattr(args, "expect", None) or ("sim_fire_extinguisher",))
    if args.scenario in vision_modes:
        vision = SimInspector(mode=args.scenario, latency_s=0.02,
                              expect_classes=expect)
        spk = SimSpeaker(mode="ok", latency_s=0.01)
    elif args.scenario in speech_modes:
        vision = SimInspector(mode="normal", latency_s=0.02,
                              expect_classes=expect)
        spk = SimSpeaker(
            mode="submit_fail" if args.scenario == "speech_fail"
            else "query_timeout", latency_s=0.01)
    else:
        raise SystemExit(f"未知场景: {args.scenario}（可选: "
                         f"{sorted(vision_modes | speech_modes)}）")
    return vision, spk


def _build_real(args):
    from inspection.inspector import YoloInspector

    from robot_controller import robot_controller

    robot = robot_controller()
    # 队友三分类模型的按类阈值：smoking 放宽、其他收紧（对齐 robot_test.py）
    conf_by_class = {"smoking": args.conf_smoking}
    vision = YoloInspector(
        robot, model_path=args.model,
        expect_classes=args.expect, abnormal_classes=args.abnormal,
        confidence=args.conf_other, min_conf=args.conf_other,
        conf_by_class=conf_by_class,
        to_bgr=True,  # robot_controller 出图 RGB 序，模型按 BGR 训练
        camera_name=args.camera)
    try:
        from inspection.speaker import make_tts_speaker
        spk = make_tts_speaker(robot)
    except Exception as exc:
        # 语音是辅助通道：TTS 不可用不阻止视觉巡检，只记为失败
        print(f"[!] TTS 服务不可用（{exc}），后续播报将全部记为失败", flush=True)
        spk = _NullSpeaker(f"tts unavailable: {exc}")
    return robot, vision, spk, conf_by_class


def run_loop(vision, spk, rec, args, conf_by_class=None, input_fn=input,
             output_fn=print) -> int:
    """单点闭环主循环：回车触发检测，q 退出。返回进程退出码。"""
    count = 0
    aborted = False
    output_fn("=== 单点巡检就绪：回车=触发检测，q+回车=退出 ===")
    while True:
        try:
            line = input_fn(f"[{count}] 点位名称(回车用默认，q 退出): ")
        except EOFError:
            # 输入流结束（如管道关闭）：正常收尾，不算中断
            output_fn("\n输入结束，正常退出巡检。")
            break
        except KeyboardInterrupt:
            aborted = True
            output_fn("\n用户中断(Ctrl+C)，结束巡检。")
            break
        name = str(line).strip()
        if name.lower() == "q":
            break
        if not name:
            name = f"POI_MANUAL_{count + 1:02d}"

        record = PoiRecord(poi=name)
        try:
            # 1) 视觉识别
            result = vision.inspect(name)
            record.inspection = result.to_dict()
            output_fn(f"  视觉: {result.status.value}  {result.detail}")

            # 2) 生成播报文本（只引用实际达标检出）
            text = build_speech_text(
                name, result,
                expect_classes=args.expect, abnormal_classes=args.abnormal,
                min_conf=args.conf_other, conf_by_class=conf_by_class)
            output_fn(f"  播报: {text}")

            # 3) TTS 播报：失败只记录，不影响视觉结论落盘
            try:
                speech = spk.say(text, interrupt=True)
            except Exception as exc:
                speech = SpeechResult(state=SpeechState.SUBMIT_FAILED,
                                      text=text, submit_success=False,
                                      submit_status=f"say exception: {exc}")
            record.speech["report"] = speech.to_dict()
            output_fn(f"  语音: {speech.state.value}"
                      + (f"  {speech.submit_status}" if speech.submit_status
                         else ""))
            record.final = "done"
        except KeyboardInterrupt:
            record.final = "aborted"
            record.ts_end = now_iso()
            try:
                rec.write_poi(record)
            except Exception as exc:
                output_fn(f"[!!] 写入中断点位记录失败: {exc}")
            aborted = True
            break
        except Exception as exc:
            record.final = "error"
            record.speech["loop_error"] = {"error": str(exc)}
            output_fn(f"[!!] 点位处理异常: {exc}")

        record.ts_end = now_iso()
        try:
            rec.write_poi(record)
        except Exception as exc:
            output_fn(f"[!!] 写入点位记录失败: {exc}")
        count += 1

    final = "aborted" if aborted else "done"
    try:
        rec.write_summary(final, f"manual loop finished, {count} poi(s)")
    except Exception as exc:
        output_fn(f"[!!] 写入 summary 失败: {exc}")
    output_fn(f"=== 结束：完成 {count} 个点位，记录 -> {rec.path} ===")
    return 1 if aborted else 0


def main() -> int:
    ap = argparse.ArgumentParser(
        description="人工遥控单点巡检闭环（默认离线模拟，不动底盘）")
    ap.add_argument("--real", action="store_true",
                    help="实机模式：真实取图+推理+播报（不控制底盘）")
    ap.add_argument("--scenario", default="normal",
                    help="模拟场景: normal/abnormal/uncertain/timeout/"
                         "no_frame/speech_fail/speech_timeout")
    ap.add_argument("--record", default=os.path.join(
        "inspection_records",
        f"manual_{now_iso().replace(':', '-').replace('+', '_')}.jsonl"))
    ap.add_argument("--model", default=DEFAULT_MODEL,
                    help="实机模式：模型权重路径（默认 models/fire/best.pt）")
    ap.add_argument("--expect", nargs="*",
                    default=["safety_sign", "fire_extinguisher"],
                    help="期望检出类别")
    ap.add_argument("--abnormal", nargs="*", default=["smoking"],
                    help="异常类别")
    ap.add_argument("--conf-smoking", type=float, default=0.25,
                    help="smoking 判定阈值（对齐队友实测）")
    ap.add_argument("--conf-other", type=float, default=0.65,
                    help="其他类别判定阈值（对齐队友实测）")
    ap.add_argument("--camera", default="head", help="实机模式：相机名")
    args = ap.parse_args()

    if args.real:
        if not _confirm_real(args):
            print("[X] 实机巡检未通过二次确认，已安全退出"
                  "（未连接机器人、未触发任何硬件）。", flush=True)
            return 2
        robot, vision, spk, conf_by_class = _build_real(args)
        rec = InspectionRecorder(args.record, meta={
            "mode": "real_manual", "simulated": False, "model": args.model,
            "expect": list(args.expect), "abnormal": list(args.abnormal),
            "conf_smoking": args.conf_smoking, "conf_other": args.conf_other,
        })
        try:
            return run_loop(vision, spk, rec, args, conf_by_class)
        finally:
            try:
                robot.stop()
            except Exception:
                pass

    vision, spk = _build_sim(args)
    rec = InspectionRecorder(args.record, meta={
        "mode": "simulate_manual", "scenario": args.scenario,
        "simulated": True,
    })
    # 模拟模式阈值参数仅用于文案一致性展示
    return run_loop(vision, spk, rec, args,
                    conf_by_class={"smoking": args.conf_smoking})


if __name__ == "__main__":
    sys.exit(main())
