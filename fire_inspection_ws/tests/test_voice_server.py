#!/usr/bin/env python3
"""speak_action_server.py 离线 mock 测试（fire-inspection 第二批）。

纯 mock：不 rclpy.init、不参与 DDS、不调用真实 TTS 服务。
通过 object.__new__ 绕过 Node 构造，注入 fake logger/client/clock/goal_handle，
直接驱动 _on_goal 与 _execute，验证真实业务行为。
所有时间相关测试使用可控 mock 时钟（sleep 即推进时钟），无真实等待。

运行：
    source ~/xos/setup.bash
    source /home/nvidia/fire-inspection/fire_inspection_ws/install/setup.bash
    python3 /home/nvidia/fire-inspection/fire_inspection_ws/tests/test_voice_server.py
"""

from __future__ import annotations

import ast
import builtins
import os
import sys
import threading
import traceback
from contextlib import contextmanager
from types import SimpleNamespace

_HERE = os.path.dirname(os.path.abspath(__file__))
_VOICE_SRC = os.path.normpath(os.path.join(_HERE, "..", "src",
                                            "fire_inspection_voice"))
if _VOICE_SRC not in sys.path:
    sys.path.insert(0, _VOICE_SRC)

import fire_inspection_voice.speak_action_server as sas  # noqa: E402

AUDIO_ID = "audio_guide"


# ── 测试替身 ────────────────────────────────────────────────────────

class FakeTime:
    """可控时钟：sleep 即推进，单调不减。"""

    def __init__(self, step: float = 0.2):
        self.now = 0.0
        self.step = step

    def monotonic(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.now += seconds


@contextmanager
def fake_clock(step: float = 0.2):
    ft = FakeTime(step)
    orig = sas.time
    sas.time = ft
    try:
        yield ft
    finally:
        sas.time = orig


class FakeCalls:
    """替代 TtsSpeaker._call 的假服务调用，记录调用序列。

    script: query 响应的 status 序列（耗尽后回退 idle_status）；
    append_ok/append_none: 控制 append 成功/失败/调用无应答；
    fail_query: query 一律返回 None（调用超时）；
    raise_on_stop: stop 调用时抛异常（验证 best-effort 不炸主流程）。
    """

    def __init__(self, script=(), append_ok=True, append_none=False,
                 fail_query=False, idle_status="idle", raise_on_stop=False):
        self.log = []
        self.script = list(script)
        self.append_ok = append_ok
        self.append_none = append_none
        self.fail_query = fail_query
        self.idle_status = idle_status
        self.raise_on_stop = raise_on_stop

    def __call__(self, cmd, text, audio_id=""):
        if cmd == "stop":
            if self.raise_on_stop:
                raise RuntimeError("stop boom")
            self.log.append((cmd, text, audio_id))
            return SimpleNamespace(success=True, status="")
        self.log.append((cmd, text, audio_id))
        if cmd == "append":
            if self.append_none:
                return None
            return SimpleNamespace(success=self.append_ok,
                                   status="ok" if self.append_ok else "busy")
        # query
        if self.fail_query:
            return None
        if self.script:
            return SimpleNamespace(success=True, status=self.script.pop(0))
        return SimpleNamespace(success=True, status=self.idle_status)


class FakeClient:
    def __init__(self, ready=True):
        self.ready = ready

    def wait_for_service(self, timeout_sec=None):
        return self.ready

    def call_async(self, req):  # 测试中 _call 已被替换，不应走到这里
        raise AssertionError("真实 call_async 不应在离线测试中被调用")


class FakeGoal:
    """模拟 goal_handle（.request/.is_cancel_requested/终态/feedback）。

    _on_goal 接收的是 goal request（真实接口为 Speak.Goal，直接带
    .text/.interrupt），该处使用 SimpleNamespace(text=...) 构造。
    """

    def __init__(self, text, interrupt=False, cancel=False):
        self.request = SimpleNamespace(text=text, interrupt=interrupt)
        self.is_cancel_requested = cancel
        self.terminals = []
        self.feedbacks = []

    def publish_feedback(self, fb):
        self.feedbacks.append((fb.progress, fb.phase))

    def succeed(self):
        self.terminals.append("succeed")

    def abort(self):
        self.terminals.append("abort")

    def canceled(self):
        self.terminals.append("canceled")


def make_server(calls, ready=True, dry_run=False, timeout_s=30.0):
    """绕过 Node.__init__ 构造实例，注入全部依赖。"""
    inst = object.__new__(sas.SpeakActionServer)
    inst.get_logger = lambda: SimpleNamespace(
        info=lambda *a, **k: None,
        warn=lambda *a, **k: None,
        error=lambda *a, **k: None)
    inst.dry_run = dry_run
    inst.tts = "/intelligent_interaction/tts/play"
    inst.audio_id = AUDIO_ID
    inst.timeout = timeout_s
    inst.poll = 0.2
    inst.playing_status = "playing"
    inst.settle = 0.4
    inst.s_per_char = 0.22
    inst.min_duration = 1.5
    inst.cli = FakeClient(ready)
    inst._exec_lock = threading.Lock()
    inst._call = calls
    return inst


# playing→idle 防抖完成的标准脚本（text=2 字 → est=1.5s, settle=0.4s）：
# 预查询 idle + playing×2 + idle×3（0.4s 防抖）
DONE_SCRIPT = ["idle", "playing", "playing", "idle", "idle", "idle"]


# ── 用例 ────────────────────────────────────────────────────────────

TESTS = []


def case(fn):
    TESTS.append(fn)
    return fn


@case
def test_empty_and_blank_text_rejected():
    """空文本 / 纯空白文本在 Goal 阶段拒绝，不触 TTS（G4 保持）。"""
    calls = FakeCalls()
    inst = make_server(calls)
    assert sas.SpeakActionServer._on_goal(
        inst, SimpleNamespace(text="")) == sas.GoalResponse.REJECT
    assert sas.SpeakActionServer._on_goal(
        inst, SimpleNamespace(text="   ")) == sas.GoalResponse.REJECT
    assert sas.SpeakActionServer._on_goal(
        inst, SimpleNamespace(text="你好")) == sas.GoalResponse.ACCEPT
    assert calls.log == []  # Goal 阶段不产生任何 TTS 调用


@case
def test_dry_run_direct_done():
    """dry_run：不调用任何 TTS，直接 succeed + state=done。"""
    calls = FakeCalls()
    inst = make_server(calls, dry_run=True)
    gh = FakeGoal("测试播报")
    with fake_clock():
        r = inst._execute(gh)
    assert r.success is True and r.state == "done"
    assert gh.terminals == ["succeed"]
    assert calls.log == []


@case
def test_service_not_ready_rejected():
    """TTS 服务未就绪 → abort + success=False + state=rejected，无调用。"""
    calls = FakeCalls()
    inst = make_server(calls, ready=False)
    gh = FakeGoal("测试播报")
    with fake_clock():
        r = inst._execute(gh)
    assert r.success is False and r.state == "rejected"
    assert gh.terminals == ["abort"]
    assert calls.log == []


@case
def test_interrupt_order_stop_then_query_append():
    """interrupt=true：调用顺序必须是 stop → (预查询) → append。"""
    calls = FakeCalls(script=["idle", "playing", "idle", "idle", "idle"])
    inst = make_server(calls)
    gh = FakeGoal("你好", interrupt=True)
    with fake_clock():
        r = inst._execute(gh)
    assert r.success is True and r.state == "done"
    assert [c[0] for c in calls.log[:3]] == ["stop", "query", "append"]
    # stop 必须携带 audio_id 且空文本（平台语义：带 text 的 stop 被忽略）
    assert calls.log[0] == ("stop", "", AUDIO_ID)


@case
def test_playing_idle_settle_done():
    """playing → 连续非 playing 达防抖窗口 → success=True, state=done。"""
    calls = FakeCalls(script=DONE_SCRIPT)
    inst = make_server(calls)
    gh = FakeGoal("你好")
    with fake_clock():
        r = inst._execute(gh)
    assert r.success is True and r.state == "done"
    assert gh.terminals == ["succeed"]
    assert "播报结束" in r.message and "seen_playing=True" in r.message


@case
def test_never_playing_returns_timeout_not_done():
    """从未观测到 playing → success=False, state=timeout（G2 修订）。"""
    calls = FakeCalls()  # 查询永远 idle
    inst = make_server(calls)
    gh = FakeGoal("你好")
    with fake_clock():
        r = inst._execute(gh)
    assert r.success is False and r.state == "timeout"
    assert "无法确认播报完成" in r.message
    assert gh.terminals == ["abort"]
    # append 仍然发生了（提交成功，只是无法确认播放）
    assert ("append", "你好", AUDIO_ID) in calls.log


@case
def test_hard_cap_playing_forever():
    """G1：query_timeout_s 为硬上限——一直 playing 也必须在上限处结束。"""
    calls = FakeCalls(idle_status="playing")  # 永远 playing
    inst = make_server(calls, timeout_s=1.0)
    gh = FakeGoal("你好")
    with fake_clock() as ft:
        r = inst._execute(gh)
    assert r.success is False and r.state == "timeout"
    assert "播放查询超时" in r.message
    assert gh.terminals == ["abort"]
    # 耗时上界：模拟时钟只累计轮询 sleep（_call 已被 FakeCalls 替换，
    # 0 耗时），退出时刻 ∈ [hard, hard+poll]。移除 min(..., self.timeout)
    # 封顶后 hard=8.0，此断言必然失败（G1 回归捕获）。
    assert ft.now <= inst.timeout + inst.poll + 1e-6, \
        f"超时未在硬上限处触发: ft.now={ft.now}"


@case
def test_hard_cap_limits_estimate():
    """G1：文本估算兜底同样不得突破 query_timeout_s。"""
    # 30 字 → est=6.6s、估算兜底原为 7.6s，但上限 1.0s 必须先到
    calls = FakeCalls()  # 永远 idle
    inst = make_server(calls, timeout_s=1.0)
    gh = FakeGoal("一" * 30)
    with fake_clock() as ft:
        r = inst._execute(gh)
    assert r.success is False and r.state == "timeout"
    assert "无法确认播报完成" in r.message
    assert gh.terminals == ["abort"]
    # 耗时上界：移除 est_deadline 的 min(..., self.timeout) 封顶后
    # est_deadline=7.6，此断言必然失败（G1 回归捕获）。
    assert ft.now <= inst.timeout + inst.poll + 1e-6, \
        f"估算兜底未受硬上限约束: ft.now={ft.now}"


@case
def test_query_call_none_is_timeout():
    """播放查询超时（调用无应答）→ success=False, state=timeout。"""
    calls = FakeCalls(fail_query=True)
    inst = make_server(calls)
    gh = FakeGoal("你好")
    with fake_clock():
        r = inst._execute(gh)
    assert r.success is False and r.state == "timeout"
    assert "无法确认播报完成" in r.message
    assert gh.terminals == ["abort"]


@case
def test_cancel_stops_audio_and_uses_canceled_terminal():
    """执行中取消：真正 stop 音频 + ROS 取消终态 canceled()（G3）。"""
    calls = FakeCalls(script=["playing"] * 20)
    inst = make_server(calls)
    gh = FakeGoal("你好", cancel=True)
    with fake_clock():
        r = inst._execute(gh)
    assert r.success is False and r.state == "rejected"
    assert gh.terminals == ["canceled"]
    assert ("stop", "", AUDIO_ID) in calls.log
    # 取消后互斥锁必须释放，可以再次受理新 Goal
    assert not inst._exec_lock.locked()
    assert sas.SpeakActionServer._on_goal(
        inst, SimpleNamespace(text="再来")) == sas.GoalResponse.ACCEPT


@case
def test_feedback_phases_and_progress():
    """Feedback 阶段含 submitting/playing/settling，进度在 [0,1]。"""
    calls = FakeCalls(script=DONE_SCRIPT)
    inst = make_server(calls)
    gh = FakeGoal("你好")
    with fake_clock():
        inst._execute(gh)
    phases = [p for _, p in gh.feedbacks]
    progresses = [v for v, _ in gh.feedbacks]
    assert phases[0] == "submitting"
    assert "playing" in phases and "settling" in phases
    assert all(0.0 <= v <= 1.0 for v in progresses)


@case
def test_stop_exception_swallowed():
    """stop 调用异常不中断主流程（best-effort），播报仍正常完成。"""
    calls = FakeCalls(script=["idle", "playing", "idle", "idle", "idle"],
                      raise_on_stop=True)
    inst = make_server(calls)
    gh = FakeGoal("你好", interrupt=True)
    with fake_clock():
        r = inst._execute(gh)  # 不应抛出异常
    assert r.success is True and r.state == "done"


@case
def test_append_failure_rejected():
    """append 返回失败 → abort + state=rejected。"""
    calls = FakeCalls(append_ok=False)
    inst = make_server(calls)
    gh = FakeGoal("你好")
    with fake_clock():
        r = inst._execute(gh)
    assert r.success is False and r.state == "rejected"
    assert "append 未成功" in r.message
    assert gh.terminals == ["abort"]


@case
def test_append_call_none_rejected():
    """append 调用无应答（超时）→ abort + state=rejected。"""
    calls = FakeCalls(append_none=True)
    inst = make_server(calls)
    gh = FakeGoal("你好")
    with fake_clock():
        r = inst._execute(gh)
    assert r.success is False and r.state == "rejected"
    assert gh.terminals == ["abort"]


@case
def test_concurrent_goal_mutex():
    """G6：并发 Goal 被拒绝；执行结束后锁释放、可再次受理新 Goal。"""
    calls = FakeCalls()
    inst = make_server(calls)
    # 模拟已有 Goal 执行中（持有互斥锁）
    inst._exec_lock.acquire()
    try:
        assert sas.SpeakActionServer._on_goal(
            inst, SimpleNamespace(text="新文本")) == sas.GoalResponse.REJECT
        gh = FakeGoal("新文本")
        with fake_clock():
            r = inst._execute(gh)
        assert r.success is False and r.state == "rejected"
        assert "已有语音 Goal 执行中" in r.message
        assert gh.terminals == ["abort"]
        assert calls.log == []  # 并发 Goal 不产生任何 TTS 调用
    finally:
        inst._exec_lock.release()
    # 释放后可再次受理并正常执行
    assert sas.SpeakActionServer._on_goal(
        inst, SimpleNamespace(text="再来一次")) == sas.GoalResponse.ACCEPT
    calls2 = FakeCalls(script=DONE_SCRIPT)
    inst._call = calls2
    gh2 = FakeGoal("再来一次")
    with fake_clock():
        r2 = inst._execute(gh2)
    assert r2.success is True and r2.state == "done"
    assert not inst._exec_lock.locked()


@case
def test_whitespace_text_stripped_before_tts():
    """前后空格在提交前剥离；纯空白由 Goal 校验拦截（边界行为）。"""
    calls = FakeCalls(script=["idle", "playing", "idle", "idle", "idle"])
    inst = make_server(calls)
    gh = FakeGoal("  你好  ")
    with fake_clock():
        r = inst._execute(gh)
    assert r.success is True and r.state == "done"
    assert ("append", "你好", AUDIO_ID) in calls.log


@case
def test_lock_released_after_timeout_paths():
    """超时/取消等提前返回后互斥锁必须释放（G6 泄漏防护）。"""
    for make_calls, timeout in ((lambda: FakeCalls(), 30.0),   # 从未 playing
                                (lambda: FakeCalls(append_ok=False), 30.0),
                                (lambda: FakeCalls(idle_status="playing"), 1.0)):
        calls = make_calls()
        inst = make_server(calls, timeout_s=timeout)
        gh = FakeGoal("你好")
        with fake_clock():
            inst._execute(gh)
        assert not inst._exec_lock.locked(), \
            f"提前返回后锁未释放: {calls.log}"


# ── P1 回归：构造路径真实执行 + 全局名静态检查 ──────────────────────

def _ast_bound_target(node, out):
    """递归收集赋值目标绑定的名字（Attribute/Subscript 不引入新名）。"""
    if isinstance(node, ast.Name):
        out.add(node.id)
    elif isinstance(node, ast.Starred):
        _ast_bound_target(node.value, out)
    elif isinstance(node, (list, tuple)):
        for item in node:
            _ast_bound_target(item, out)


def _ast_locals_of(func):
    """收集函数作用域绑定的局部名（含嵌套层与推导式，保守防误报）。"""
    local = set()
    a = func.args
    for arg in list(a.posonlyargs) + list(a.args) + list(a.kwonlyargs):
        local.add(arg.arg)
    if a.vararg:
        local.add(a.vararg.arg)
    if a.kwarg:
        local.add(a.kwarg.arg)
    for stmt in ast.walk(func):
        if isinstance(stmt, ast.Assign):
            for t in stmt.targets:
                _ast_bound_target(t, local)
        elif isinstance(stmt, (ast.AugAssign, ast.AnnAssign)):
            _ast_bound_target(stmt.target, local)
        elif isinstance(stmt, (ast.For, ast.AsyncFor, ast.comprehension)):
            _ast_bound_target(stmt.target, local)
        elif isinstance(stmt, (ast.With, ast.AsyncWith)):
            for item in stmt.items:
                if item.optional_vars is not None:
                    _ast_bound_target(item.optional_vars, local)
        elif isinstance(stmt, ast.ExceptHandler) and stmt.name:
            local.add(stmt.name)
        elif isinstance(stmt, ast.NamedExpr):
            local.add(stmt.target.id)
        elif isinstance(stmt, (ast.FunctionDef, ast.AsyncFunctionDef,
                               ast.ClassDef)):
            local.add(stmt.name)
        elif isinstance(stmt, (ast.Global, ast.Nonlocal)):
            local.update(stmt.names)  # 外层名，视作已解析防误报
    return local


@case
def test_real_constructor_runs_offline():
    """P1 回归：真实实例化 SpeakActionServer，执行 __init__ 全部构造体。

    object.__new__ 会绕过构造函数，P1（缺 import threading → L77
    NameError）曾因此漏检。本用例临时把 Node 基类的 5 个构造期方法
    替换为离线替身（不 rclpy.init、不参与 DDS、不触 TTS），随后真实
    执行 __init__ 每一行，断言 _exec_lock 真实创建可用；finally 必然
    恢复被替换的 Node 方法与模块属性。
    """
    node_cls = sas.Node
    saved = {k: node_cls.__dict__[k] for k in
             ("__init__", "declare_parameter", "get_parameter",
              "create_client", "get_logger") if k in node_cls.__dict__}
    declared = {}

    def _fake_init(self, name, **kw):
        self._node_name = name

    def _fake_declare(self, name, value):
        declared[name] = value

    def _fake_get(self, name):
        return SimpleNamespace(value=declared[name])

    def _fake_client(self, srv_type, srv_name, **kw):
        return FakeClient(True)

    def _fake_logger(self):
        return SimpleNamespace(
            info=lambda *a, **k: None,
            warn=lambda *a, **k: None,
            error=lambda *a, **k: None)

    real_load_config, real_action_server = sas.load_config, sas.ActionServer
    sas.load_config = lambda: {}
    sas.ActionServer = lambda *a, **k: SimpleNamespace()
    node_cls.__init__ = _fake_init
    node_cls.declare_parameter = _fake_declare
    node_cls.get_parameter = _fake_get
    node_cls.create_client = _fake_client
    node_cls.get_logger = _fake_logger
    try:
        inst = sas.SpeakActionServer()  # 真实签名无参，节点名硬编码
    finally:
        sas.load_config = real_load_config
        sas.ActionServer = real_action_server
        for k, v in saved.items():
            setattr(node_cls, k, v)
    # 构造体真实执行成功：默认参数与关键属性齐备
    assert getattr(inst, "_node_name", None) == "fire_speak"
    assert inst.dry_run is False
    assert inst.audio_id == AUDIO_ID
    assert inst.timeout == 30.0
    assert inst.tts == "/intelligent_interaction/tts/play"
    lock = getattr(inst, "_exec_lock", None)
    assert lock is not None and hasattr(lock, "acquire"), \
        "_exec_lock 未在真实构造中创建（P1 类缺陷）"
    assert not lock.locked()
    lock.acquire()
    assert lock.locked()
    lock.release()
    assert not lock.locked()


@case
def test_ast_all_referenced_names_resolved():
    """P1 兜底：AST 静态检查所有函数引用的全局名已导入/定义。

    覆盖构造执行用例之外的盲区（任意方法内新增未导入全局名）。
    局部名完整收集（参数/赋值/for/with/except as/推导式/walrus/
    嵌套 def/global 声明），仅对真正无法解析的 Load 名字报错。
    """
    with open(sas.__file__, "r", encoding="utf-8") as f:
        tree = ast.parse(f.read())

    module_names = set()
    for stmt in tree.body:
        if isinstance(stmt, ast.Import):
            for a in stmt.names:
                module_names.add(a.asname or a.name.split(".")[0])
        elif isinstance(stmt, ast.ImportFrom):
            for a in stmt.names:
                if a.name != "*":
                    module_names.add(a.asname or a.name)
        elif isinstance(stmt, ast.Assign):
            for t in stmt.targets:
                _ast_bound_target(t, module_names)
        elif isinstance(stmt, ast.AnnAssign):
            _ast_bound_target(stmt.target, module_names)
        elif isinstance(stmt, (ast.FunctionDef, ast.AsyncFunctionDef,
                               ast.ClassDef)):
            module_names.add(stmt.name)

    allowed = module_names | set(dir(builtins)) | {
        "__name__", "__file__", "__doc__", "__package__", "__spec__",
        "__loader__", "__builtins__", "__debug__"}
    problems = []
    # 所有函数（含类方法）：Load 名必须落在 局部/模块/builtins/运行时名 内
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            local = _ast_locals_of(node)
            for n in ast.walk(node):
                if (isinstance(n, ast.Name) and isinstance(n.ctx, ast.Load)
                        and n.id not in local and n.id not in allowed):
                    problems.append(f"{n.id} @ line {n.lineno}")
    # 模块级顶层可执行语句（如 if __name__ == "__main__"）
    skip = (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef,
            ast.Import, ast.ImportFrom)
    for stmt in tree.body:
        if isinstance(stmt, skip):
            continue
        for n in ast.walk(stmt):
            if (isinstance(n, ast.Name) and isinstance(n.ctx, ast.Load)
                    and n.id not in allowed):
                problems.append(f"{n.id} @ line {n.lineno}")
    assert not problems, \
        f"存在未导入/未定义的全局名引用: {sorted(set(problems))}"


# ── 运行器 ──────────────────────────────────────────────────────────

def main() -> int:
    passed, failed = [], []
    for fn in TESTS:
        try:
            fn()
            passed.append(fn.__name__)
            print(f"[PASS] {fn.__name__}")
        except Exception:
            failed.append(fn.__name__)
            print(f"[FAIL] {fn.__name__}")
            traceback.print_exc()
    print(f"\n=== 结果: {len(passed)}/{len(TESTS)} 通过 ===")
    if failed:
        print("失败用例:", ", ".join(failed))
    return 0 if not failed else 1


if __name__ == "__main__":
    sys.exit(main())
