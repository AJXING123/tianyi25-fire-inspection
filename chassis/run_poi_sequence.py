"""一键导航：到 start_point -> 沿唯一轨道依次经过 POI_01 / POI_02 / POI_03。

运行（在 src 目录下）：
    python3 chassis/run_poi_sequence.py

无需任何交互和模式选择：脚本启动后自动选当前地图下唯一的轨道并开始执行。
每个 POI 的流程：沿轨道到达 -> 转到该 POI 配置的朝向（yaw）-> 原地停留 3 秒 -> 继续轨道。

安全行为复用 run_POI 的 _run_motion_safe：
每段运动前开启 AEB、轮询 action 状态到终态、Ctrl+C 急停，
每段结束停车并关闭 AEB；任意一段失败即终止，不再继续后面的点。

启动后把底盘最大线速度限制为 0.4 m/s（角速度保持不变），
退出时（正常结束、失败或中断）恢复为原来的速度参数。
"""

import os
import sys
import time

# robot_controller 包在本脚本的上一级目录 src/ 下
_SRC_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _SRC_DIR not in sys.path:
    sys.path.insert(0, _SRC_DIR)

from robot_controller import robot_controller
from robot_controller.chassis import ChassisNavError
from chassis.run_POI import WorkflowError, _run_motion_safe, _unwrap_list


# ── 配置 ────────────────────────────────────────────────────────────
START_POI = "start_point"
POI_SEQUENCE = ("POI_01", "POI_02", "POI_03")

START_TIMEOUT = 180.0          # 前往起点的总超时
SEGMENT_TIMEOUT = 180.0        # 轨道每一段、每个转向动作的总超时
DWELL_SECONDS = 3.0            # 到点转向后的停留时间
MAX_MOVING_SPEED = 0.4      # 底盘最大线速度，单位 m/s
SPEED_APPLY_TIMEOUT = 5.0      # 等待限速真正生效的超时


def _ensure_chassis(robot):
    if robot.chassis is None:
        raise WorkflowError(
            "底盘未配置（robot_config.yaml 缺 chassis 段），本脚本必须依赖底盘，终止。"
        )


def _poi_map(robot):
    """返回当前地图下 {名称: POI dict}。"""
    rv = robot.chassis_get_all_poi()
    pois = _unwrap_list(rv, "pois", "poi", "data")
    result = {}
    for poi in pois:
        if not isinstance(poi, dict):
            continue
        name = poi.get("name") or poi.get("point_name")
        if name:
            result[str(name)] = poi
    return result


def _wait_speed(robot, target):
    """轮询直到底盘线速度回读等于目标值，超时返回 False。"""
    deadline = time.monotonic() + SPEED_APPLY_TIMEOUT
    while time.monotonic() < deadline:
        current = robot.chassis_get_motion_param().get("move_spd")
        if current is not None and abs(float(current) - float(target)) < 1e-3:
            return True
        time.sleep(0.2)
    return False


def _apply_speed_limit(robot):
    """把底盘最大线速度限制为 MAX_MOVING_SPEED，返回原参数供退出时恢复。

    set_speed_params 是异步任务，接口返回 accepted 不代表已经生效，
    必须轮询回读确认，否则第一段可能仍按原速度行驶。
    """
    previous = robot.chassis_get_motion_param()
    if previous.get("move_spd") is None:
        raise WorkflowError("读取底盘速度参数失败，无法确认限速，终止。")

    robot.chassis_set_motion_param(move_spd=MAX_MOVING_SPEED)
    if not _wait_speed(robot, MAX_MOVING_SPEED):
        raise WorkflowError(
            f"限速 {MAX_MOVING_SPEED} m/s 未在 {SPEED_APPLY_TIMEOUT}s 内生效，终止。"
        )
    return previous


def _restore_speed(robot, previous):
    """提交恢复请求并确认线速度已回到原值。"""
    target = previous.get("move_spd")
    robot.chassis_set_motion_param(move_spd=target, rot_spd=previous.get("rot_spd"))
    if _wait_speed(robot, target):
        print(f"[恢复] 底盘线速度已还原: {target} m/s", flush=True)
    else:
        print(f"[!] 底盘线速度恢复未确认，目标 {target} m/s，请检查。", flush=True)


def _find_track(robot):
    """自动选取当前地图下唯一的轨道，返回含 track_id 的轨道 dict。"""
    rv = robot.chassis_get_track()
    tracks = [
        t for t in _unwrap_list(rv, "tracks", "track", "data")
        if isinstance(t, dict)
    ]
    if len(tracks) != 1:
        names = [t.get("track_name") for t in tracks]
        raise WorkflowError(
            f"当前地图应只有一条轨道，实际 {len(tracks)} 条: {names}，终止。"
        )
    track = tracks[0]
    if track.get("track_id") is None:
        raise WorkflowError("轨道缺少 track_id，终止。")
    return track


def main():
    robot = robot_controller()
    previous_speed = None
    try:
        _ensure_chassis(robot)

        # 前置校验：点位与轨道都必须存在，避免下发一半才报错
        pois = _poi_map(robot)
        required = (START_POI,) + POI_SEQUENCE
        missing = [name for name in required if name not in pois]
        if missing:
            raise WorkflowError(f"当前地图缺少 POI: {missing}，终止。")
        for name in required:
            if any(pois[name].get(k) is None for k in ("x", "y", "yaw")):
                raise WorkflowError(f"POI {name} 缺少 x/y/yaw，终止。")

        track = _find_track(robot)
        track_id = track["track_id"]
        track_name = track.get("track_name") or str(track_id)

        print("\n=== 导航序列 ===", flush=True)
        print(f"  起点: {START_POI}", flush=True)
        print(f"  轨道: {track_name} (track_id={track_id})", flush=True)
        print(f"  目标: {' -> '.join(POI_SEQUENCE)}", flush=True)
        print(f"  到点: 转向该点 yaw 后停留 {DWELL_SECONDS:.0f}s", flush=True)
        print(f"  线速度: {MAX_MOVING_SPEED} m/s（结束后恢复原值）", flush=True)

        # 0) 限速：必须在第一段运动之前生效
        previous_speed = _apply_speed_limit(robot)
        print(
            f"-> 已限速为 {MAX_MOVING_SPEED} m/s"
            f"（原线速度 {previous_speed.get('move_spd')} m/s），开始执行",
            flush=True,
        )

        # 1) 先导航到 start_point（点对点）
        _run_motion_safe(
            robot,
            f"前往起点 {START_POI}",
            lambda: robot.chassis_nav_to_poi(START_POI),
            START_TIMEOUT,
        )

        # 2) 沿唯一轨道依次到三个 POI：到达 -> 转向该点 yaw -> 停留 3s -> 继续
        for poi_name in POI_SEQUENCE:
            _run_motion_safe(
                robot,
                f"轨道 {track_name} -> {poi_name}",
                lambda poi=poi_name: robot.chassis_track_nav_to_poi(track_id, poi),
                SEGMENT_TIMEOUT,
            )

            poi = pois[poi_name]
            yaw = float(poi["yaw"])
            _run_motion_safe(
                robot,
                f"{poi_name} 转到朝向 yaw={yaw:.3f}",
                lambda x=float(poi["x"]), y=float(poi["y"]), yaw=yaw:
                    robot.chassis_nav_to_point(x, y, yaw, flags=0),
                SEGMENT_TIMEOUT,
            )

            print(
                f"-> {poi_name} 已到位，停留 {DWELL_SECONDS:.0f}s",
                flush=True,
            )
            time.sleep(DWELL_SECONDS)

        print("\n=== 执行完毕确认 ===", flush=True)
        try:
            print(f"  当前位姿: {robot.chassis_get_pose()}", flush=True)
        except Exception as exc:
            print(f"  [!] 读取当前位姿失败: {exc}", flush=True)
        print("  全部导航完成。", flush=True)

    except WorkflowError as exc:
        print(f"\n[ERROR] {exc}", file=sys.stderr, flush=True)
        sys.exit(1)
    except ChassisNavError as exc:
        print(
            f"\n[ERROR] [{exc.code.name}] {exc.message}",
            file=sys.stderr,
            flush=True,
        )
        sys.exit(1)
    except KeyboardInterrupt:
        print("\n[中断] 收到 Ctrl+C，退出。", flush=True)
        sys.exit(130)
    finally:
        if previous_speed is not None:
            try:
                _restore_speed(robot, previous_speed)
            except Exception as exc:
                print(f"[!] 恢复底盘速度失败: {exc}", flush=True)
        try:
            robot.stop()
        except Exception as exc:
            print(f"[!] robot.stop() 异常: {exc}", flush=True)


if __name__ == "__main__":
    main()
