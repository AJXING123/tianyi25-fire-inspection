"""导航适配层。

真实模式复用队友代码：
- chassis.run_POI: WorkflowError / _run_motion_safe / _unwrap_list
  （含 AEB 开关、Ctrl+C 急停、状态轮询、终态分类，全部不复制不重写）
- robot_controller 的 chassis_* API（REST 转发，不自行构造 HTTP 请求）

前置校验（实机导航前必须通过，否则拒绝执行）：
1. robot.chassis 已配置；
2. chassis_pose_quality() -> {"ok": slam_status==0, ...}（定位就绪）；
3. 所需 POI 全部存在且 x/y/yaw 非空；
4. 当前地图恰好一条轨道且带 track_id。

已声明的脆弱点：
- CANCELLED 目前只能靠 WorkflowError 消息里 "用户中断" 字样区分，
  与「执行失败」共用异常类型。底盘 REST 是否有正式取消/续行语义未确认，
  本层把 CANCELLED 单列，不当普通失败，也不假设可恢复。
"""

from __future__ import annotations

import time
from typing import Optional

from .types import NavOutcome, NavResult


class NavigatorBase:
    """统一接口。所有方法不得抛异常到 controller，统一落 NavResult。"""

    def precheck(self, pois: list, start_poi: str) -> NavResult:
        raise NotImplementedError

    def apply_speed_limit(self) -> NavResult:
        raise NotImplementedError

    def restore_speed_limit(self) -> None:
        raise NotImplementedError

    def nav_to_start(self, start_poi: str, timeout: float) -> NavResult:
        raise NotImplementedError

    def nav_track_to_poi(self, poi_name: str, timeout: float) -> NavResult:
        raise NotImplementedError

    def align_yaw(self, poi: dict, timeout: float) -> NavResult:
        raise NotImplementedError

    def get_pose(self) -> dict:
        raise NotImplementedError

    def stop(self) -> None:
        raise NotImplementedError

    def close(self) -> None:
        pass


# ── 模拟实现 ────────────────────────────────────────────────────────

class SimNavigator(NavigatorBase):
    """离线模拟导航。

    fail_at: 指定在哪个环节失败，如 "POI_02" / "POI_02.align" / "start_point"。
    outcome: 失败类型 failed / timeout。
    delay_s: 每段模拟耗时。
    """

    def __init__(self, fail_at: Optional[str] = None, outcome: str = "failed",
                 delay_s: float = 0.02):
        self.fail_at = fail_at
        self.outcome = outcome
        self.delay_s = delay_s
        self.calls: list = []
        self._x = 0.0

    def _run(self, key: str, label: str) -> NavResult:
        self.calls.append(key)
        t0 = time.monotonic()
        time.sleep(self.delay_s)
        elapsed = time.monotonic() - t0
        if self.fail_at == key:
            outcome = (NavOutcome.TIMEOUT if self.outcome == "timeout"
                       else NavOutcome.FAILED)
            return NavResult(outcome=outcome, label=label,
                             detail=f"simulated {self.outcome} at {key}",
                             elapsed_s=elapsed, simulated=True)
        return NavResult(outcome=NavOutcome.SUCCESS, label=label,
                         detail="simulated", elapsed_s=elapsed, simulated=True)

    def precheck(self, pois: list, start_poi: str) -> NavResult:
        return NavResult(outcome=NavOutcome.SUCCESS, label="precheck",
                         detail="simulated ok", simulated=True)

    def apply_speed_limit(self) -> NavResult:
        return NavResult(outcome=NavOutcome.SUCCESS, label="speed_limit",
                         detail="simulated", simulated=True)

    def restore_speed_limit(self) -> None:
        self.calls.append("restore_speed")

    def nav_to_start(self, start_poi: str, timeout: float) -> NavResult:
        return self._run(start_poi, f"前往起点 {start_poi}")

    def nav_track_to_poi(self, poi_name: str, timeout: float) -> NavResult:
        return self._run(poi_name, f"轨道 -> {poi_name}")

    def align_yaw(self, poi: dict, timeout: float) -> NavResult:
        return self._run(f"{poi.get('name', '?')}.align", "对齐朝向")

    def get_pose(self) -> dict:
        return {"x": 0.0, "y": 0.0, "yaw": 0.0, "simulated": True}

    def stop(self) -> None:
        self.calls.append("stop")


# ── 真实实现 ────────────────────────────────────────────────────────

class ChassisNavigator(NavigatorBase):
    """基于 robot_controller + run_POI._run_motion_safe 的真实导航。"""

    def __init__(self, robot, max_speed: float = 0.4,
                 speed_apply_timeout: float = 5.0):
        self._robot = robot
        self._max_speed = max_speed
        self._speed_apply_timeout = speed_apply_timeout
        self._track_id: Optional[int] = None
        self._prev_motion_param: Optional[dict] = None
        self._pois: dict = {}

    # ── 前置校验 ────────────────────────────────────────────────────

    def precheck(self, pois: list, start_poi: str) -> NavResult:
        from chassis.run_POI import WorkflowError, _unwrap_list

        robot = self._robot
        try:
            if robot.chassis is None:
                return self._refused("底盘未配置（robot_config.yaml 缺 chassis 段）")

            # chassis_pose_quality() -> {"ok": slam_status==0, "slam_status": int}
            q = robot.chassis_pose_quality()
            if not q.get("ok"):
                return self._refused(
                    f"定位未就绪: slam_status={q.get('slam_status')} "
                    f"(0=OK, 5=waiting_reloc)，禁止实机导航"
                )

            rv = robot.chassis_get_all_poi()
            poi_map = {}
            for p in _unwrap_list(rv, "pois", "poi", "data"):
                if isinstance(p, dict):
                    name = p.get("name") or p.get("point_name")
                    if name:
                        poi_map[str(name)] = p
            required = [start_poi] + list(pois)
            missing = [n for n in required if n not in poi_map]
            if missing:
                return self._refused(f"当前地图缺少 POI: {missing}")
            for n in required:
                if any(poi_map[n].get(k) is None for k in ("x", "y", "yaw")):
                    return self._refused(f"POI {n} 缺少 x/y/yaw")
            self._pois = poi_map

            tracks = [t for t in _unwrap_list(robot.chassis_get_track(),
                                              "tracks", "track", "data")
                      if isinstance(t, dict)]
            if len(tracks) != 1:
                return self._refused(
                    f"当前地图应只有一条轨道，实际 {len(tracks)} 条")
            self._track_id = tracks[0].get("track_id")
            if self._track_id is None:
                return self._refused("轨道缺少 track_id")

            return NavResult(outcome=NavOutcome.SUCCESS, label="precheck",
                             detail=f"slam_ok, {len(poi_map)} pois, "
                                    f"track_id={self._track_id}")
        except WorkflowError as exc:
            return self._refused(f"前置校验异常: {exc}")
        except Exception as exc:
            return self._refused(f"前置校验异常: {exc}")

    @staticmethod
    def _refused(detail: str) -> NavResult:
        return NavResult(outcome=NavOutcome.PRECHECK_REFUSED,
                         label="precheck", detail=detail)

    # ── 限速 ────────────────────────────────────────────────────────

    def apply_speed_limit(self) -> NavResult:
        robot = self._robot
        # 1. 先读原始参数；读不出就没有可靠备份，直接拒绝写入
        #    （不得猜测默认值、不得用“安全默认速度”冒充回滚）
        try:
            prev = robot.chassis_get_motion_param()
        except Exception as exc:
            return NavResult(outcome=NavOutcome.FAILED, label="speed_limit",
                             detail=f"读取底盘速度参数异常: {exc}")
        if not isinstance(prev, dict) or prev.get("move_spd") is None:
            return NavResult(outcome=NavOutcome.FAILED, label="speed_limit",
                             detail="读取底盘速度参数失败（move_spd 缺失），"
                                    "未写入限速")

        # 2. 备份先行登记（写入之前），保证写入异常/验证失败/中途取消
        #    等任何后续失败路径都能回滚；重复调用时保留最早的真实备份
        if self._prev_motion_param is None:
            self._prev_motion_param = prev

        # 3. 写入限速；写入抛异常视为“可能已部分生效”，立即尝试回滚
        try:
            robot.chassis_set_motion_param(move_spd=self._max_speed)
        except Exception as exc:
            restore = self._try_restore()
            return NavResult(outcome=NavOutcome.FAILED, label="speed_limit",
                             detail=f"写入限速异常: {exc}; {restore}")

        # 4. 轮询验证生效；验证读取异常或超时同样回滚
        deadline = time.monotonic() + self._speed_apply_timeout
        while time.monotonic() < deadline:
            try:
                cur = robot.chassis_get_motion_param().get("move_spd")
            except Exception as exc:
                restore = self._try_restore()
                return NavResult(outcome=NavOutcome.FAILED, label="speed_limit",
                                 detail=f"限速验证读取异常: {exc}; {restore}")
            if cur is not None and abs(float(cur) - self._max_speed) < 1e-3:
                return NavResult(outcome=NavOutcome.SUCCESS,
                                 label="speed_limit",
                                 detail=f"{prev.get('move_spd')} -> "
                                        f"{self._max_speed} m/s")
            time.sleep(0.2)

        restore = self._try_restore()
        return NavResult(outcome=NavOutcome.FAILED, label="speed_limit",
                         detail=f"限速未在 {self._speed_apply_timeout}s 内生效"
                                f"（写入可能已生效）; {restore}")

    def _try_restore(self) -> str:
        """尽力回滚已登记的原始速度参数，返回如实描述字符串。

        成功：清除备份（后续 cleanup 不再重复回滚）。
        失败：保留备份（cleanup 的 restore_speed_limit 会重试），
              并明确打印错误，绝不假装恢复成功。
        """
        prev = self._prev_motion_param
        if prev is None:
            return "无原始参数备份，未执行回滚"
        try:
            self._robot.chassis_set_motion_param(
                move_spd=prev.get("move_spd"),
                rot_spd=prev.get("rot_spd"))
        except Exception as exc:
            msg = (f"回滚原始速度参数失败: {exc}；"
                   f"底盘速度状态未知，需人工核查")
            print(f"[!!] {msg}", flush=True)
            return msg
        self._prev_motion_param = None
        return (f"已回滚原始速度参数 move_spd={prev.get('move_spd')} "
                f"rot_spd={prev.get('rot_spd')}")

    def restore_speed_limit(self) -> None:
        # 恢复失败会在 _try_restore 内明确打印；备份保留以便重复调用重试
        self._try_restore()

    # ── 运动段（复用 _run_motion_safe，异常 → NavResult） ────────────

    def _motion(self, label: str, dispatch_fn, timeout: float) -> NavResult:
        from chassis.run_POI import WorkflowError, _run_motion_safe
        from robot_controller.chassis import ChassisNavError

        t0 = time.monotonic()
        try:
            _run_motion_safe(self._robot, label, dispatch_fn, timeout)
            return NavResult(outcome=NavOutcome.SUCCESS, label=label,
                             elapsed_s=time.monotonic() - t0)
        except WorkflowError as exc:
            return self._classify(label, str(exc), t0)
        except ChassisNavError as exc:
            return self._classify(label, f"[{exc.code.name}] {exc.message}", t0)
        except Exception as exc:
            return self._classify(label, f"unexpected: {exc}", t0)

    @staticmethod
    def _classify(label: str, message: str, t0: float) -> NavResult:
        """脆弱文本匹配：与队友确认 REST 取消语义前应替换为正式协议。"""
        if "用户中断" in message:
            outcome = NavOutcome.CANCELLED
        elif "超时" in message:
            outcome = NavOutcome.TIMEOUT
        else:
            outcome = NavOutcome.FAILED
        return NavResult(outcome=outcome, label=label, detail=message,
                         elapsed_s=time.monotonic() - t0)

    def nav_to_start(self, start_poi: str, timeout: float) -> NavResult:
        return self._motion(
            f"前往起点 {start_poi}",
            lambda: self._robot.chassis_nav_to_poi(start_poi),
            timeout)

    def nav_track_to_poi(self, poi_name: str, timeout: float) -> NavResult:
        if self._track_id is None:
            return NavResult(outcome=NavOutcome.PRECHECK_REFUSED,
                             label=f"轨道 -> {poi_name}",
                             detail="precheck 未通过或无轨道")
        return self._motion(
            f"轨道 -> {poi_name}",
            lambda: self._robot.chassis_track_nav_to_poi(self._track_id, poi_name),
            timeout)

    def align_yaw(self, poi: dict, timeout: float) -> NavResult:
        x, y, yaw = float(poi["x"]), float(poi["y"]), float(poi["yaw"])
        return self._motion(
            f"{poi.get('name', '?')} 转到 yaw={yaw:.3f}",
            lambda: self._robot.chassis_nav_to_point(x, y, yaw, flags=0),
            timeout)

    def get_pose(self) -> dict:
        try:
            return self._robot.chassis_get_pose()
        except Exception as exc:
            return {"error": str(exc)}

    def stop(self) -> None:
        try:
            self._robot.chassis_stop_motion()
        except Exception:
            pass
