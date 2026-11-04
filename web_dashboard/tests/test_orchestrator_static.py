"""编排层结构冒烟检查（只读源码，不导入 rclpy）。

开发机是 ROS 2 Humble，无法导入机器人端 Jazzy 的消息包，因此这里用 AST
校验“接线”是否真实存在：Publisher 创建、QoS、话题参数、锁释放路径、
手动状态前缀。它只能证明代码结构正确，**不能替代 Jazzy 环境联调**。
"""

from __future__ import annotations

import ast
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
ORCHESTRATOR = (
    REPO_ROOT
    / "fire_inspection_ws"
    / "src"
    / "fire_inspection_orchestrator"
    / "fire_inspection_orchestrator"
    / "orchestrator_node.py"
)


def _source() -> str:
    return ORCHESTRATOR.read_text(encoding="utf-8")


def _tree() -> ast.Module:
    return ast.parse(_source())


def _method(tree: ast.Module, name: str) -> ast.FunctionDef:
    for node in tree.body:
        if isinstance(node, ast.ClassDef) and node.name == "OrchestratorNode":
            for item in node.body:
                if isinstance(item, ast.FunctionDef) and item.name == name:
                    return item
    raise AssertionError(f"未找到方法 OrchestratorNode.{name}")


def _calls(node: ast.AST, label: str) -> list[ast.Call]:
    """收集 func 文本等于 label 的调用，例如 'self._release_busy'。"""
    found: list[ast.Call] = []
    for candidate in ast.walk(node):
        if (
            isinstance(candidate, ast.Call)
            and isinstance(candidate.func, ast.Attribute)
            and ast.unparse(candidate.func) == label
        ):
            found.append(candidate)
    return found


def _string_constants(node: ast.AST) -> set[str]:
    return {
        item.value
        for item in ast.walk(node)
        if isinstance(item, ast.Constant) and isinstance(item.value, str)
    }


def _assignments(node: ast.AST, attr: str) -> list[ast.Assign]:
    result: list[ast.Assign] = []
    for candidate in ast.walk(node):
        if isinstance(candidate, ast.Assign):
            for target in candidate.targets:
                if isinstance(target, ast.Attribute) and target.attr == attr:
                    result.append(candidate)
    return result


# -- P0-1：PoiResult 发布器 -------------------------------------------------


def test_poi_result_topic_parameter_declared():
    init = _method(_tree(), "__init__")
    declared = [
        call
        for call in _calls(init, "self.declare_parameter")
        if call.args and _string_constants(call.args[0]) == {"poi_result_topic"}
    ]
    assert declared, "缺少 poi_result_topic 参数声明"
    assert any(
        "/fire_inspection/poi_result" in _string_constants(call)
        for call in declared
    ), "poi_result_topic 默认值必须为 /fire_inspection/poi_result"


def test_poi_result_publisher_created_exactly_once():
    init = _method(_tree(), "__init__")
    assignments = _assignments(init, "_poi_result_pub")
    assert len(assignments) == 1, "PoiResult 发布器必须且只能创建一次"

    value = assignments[0].value
    assert isinstance(value, ast.Call), "发布器必须由 create_publisher 创建"
    assert ast.unparse(value.func) == "self.create_publisher"

    msg_type, topic_expr, qos_expr = value.args[0], value.args[1], value.args[2]
    assert ast.unparse(msg_type) == "PoiResult"
    assert "poi_result_topic" in ast.unparse(topic_expr), "话题必须来自参数"
    assert ast.unparse(qos_expr) == "POI_RESULT_QOS"


def test_poi_result_qos_is_reliable_volatile_depth_10():
    keywords = {}
    for node in _tree().body:
        if isinstance(node, ast.Assign) and ast.unparse(node.targets[0]) == "POI_RESULT_QOS":
            assert isinstance(node.value, ast.Call)
            keywords = {kw.arg: kw.value for kw in node.value.keywords}
            break
    assert keywords, "缺少 POI_RESULT_QOS 定义"
    assert ast.unparse(keywords["depth"]) == "10"
    assert ast.unparse(keywords["reliability"]).endswith("RELIABLE")
    assert ast.unparse(keywords["durability"]).endswith("VOLATILE")


def test_publish_poi_result_uses_created_publisher():
    method = _method(_tree(), "_publish_poi_result")
    publishes = _calls(method, "self._poi_result_pub.publish")
    assert len(publishes) == 1, "必须通过已创建的发布器发布结果"
    fields = _string_constants(method)
    # 原样透传 Action 返回字段，不改语义
    for field_name in ("simulated", "detail", "detections_json", "status"):
        assert field_name in fields or any(
            isinstance(n, ast.Attribute) and n.attr == field_name for n in ast.walk(method)
        ), f"缺少字段 {field_name} 的透传"
    # 发布失败必须显式 error 日志，不能静默
    assert _calls(method, "self.get_logger().error"), "发布失败必须记录 error 日志"


# -- P1-1：任何退出路径都释放 _busy ----------------------------------------


def test_run_releases_busy_for_all_exit_paths():
    run = _method(_tree(), "_run")
    tries = [n for n in run.body if isinstance(n, ast.Try)]
    assert tries, "_run 必须用最外层 try/finally 统一管理锁"
    assert _calls(tries[0], "self._release_busy"), "_run 的 finally 必须释放 _busy"

    run_round = _method(_tree(), "_run_round")
    # WATCH 模式：先 return，靠外层 finally 释放锁
    watch_if = [
        n
        for n in ast.walk(run_round)
        if isinstance(n, ast.If) and ast.unparse(n.test) == "not self._nav_enabled"
    ]
    assert watch_if, "缺少 nav_enabled=False 的 WATCH 分支"
    assert any(isinstance(n, ast.Return) for n in ast.walk(watch_if[0])), (
        "WATCH 分支必须保留提前 return（由外层 finally 释放锁）"
    )
    assert not _calls(run_round, "self._release_busy"), (
        "_run_round 不得重复释放 _busy（统一由 _run 的 finally 管理）"
    )

    manual = _method(_tree(), "_run_manual")
    assert _calls(manual, "self._release_busy"), "手动流程也必须释放 _busy"


# -- P0-2：手动导航/介入状态带 manual 前缀 ---------------------------------


def test_navigate_supports_manual_flag():
    navigate = _method(_tree(), "_navigate")
    args = navigate.args
    positions = [a.arg for a in args.args + args.kwonlyargs]
    assert "manual" in positions, "_navigate 必须支持 manual 标记"
    defaults = [ast.unparse(d) for d in args.defaults]
    assert "False" in defaults, "manual 默认必须为 False（自动巡检行为不变）"

    # 状态文本必须能拼出 manual:nav:<poi>
    assert "manual:" in _string_constants(navigate), "_navigate 必须定义 manual 前缀"
    for call in _calls(navigate, "self._set_state"):
        if "nav:" in _string_constants(call):
            assert "phase_prefix" in ast.unparse(call), (
                "_navigate 的状态文本必须使用 manual 前缀变量"
            )
            break
    else:  # pragma: no cover - 结构变了就明确失败
        raise AssertionError("未找到 _navigate 的状态发布点")


def test_manual_run_navigates_with_manual_flag():
    manual = _method(_tree(), "_run_manual")
    navigate_calls = _calls(manual, "self._navigate")
    assert navigate_calls, "手动流程必须复用 _navigate"
    assert any(
        any(
            kw.arg == "manual" and ast.unparse(kw.value) == "True"
            for kw in call.keywords
        )
        for call in navigate_calls
    ), "手动流程调用 _navigate 必须传 manual=True"


def test_intervention_supports_manual_prefix():
    intervention = _method(_tree(), "_handle_intervention")
    positions = [a.arg for a in intervention.args.args + intervention.args.kwonlyargs]
    assert "manual" in positions, "_handle_intervention 必须支持 manual 标记"
    for call in _calls(intervention, "self._set_state"):
        if "intervene:" in _string_constants(call):
            assert "manual:" in _string_constants(call), (
                "介入状态必须能拼出 manual:intervene:<poi>"
            )
            break
    else:  # pragma: no cover
        raise AssertionError("未找到介入状态发布点")


def test_manual_navigate_calls_forward_manual_flag():
    navigate = _method(_tree(), "_navigate")
    forwarded = [
        call
        for call in _calls(navigate, "self._handle_intervention")
        if any(kw.arg == "manual" and ast.unparse(kw.value) == "manual" for kw in call.keywords)
    ]
    assert len(forwarded) == len(_calls(navigate, "self._handle_intervention")) >= 1, (
        "导航内的所有介入调用都必须透传 manual 标记"
    )


# -- auto_start：Dashboard 安全启动模式 -------------------------------------


def _auto_start_if(init: ast.FunctionDef) -> ast.If | None:
    """找到 __init__ 中以 auto_start 为条件的 If 节点。"""
    for node in ast.walk(init):
        if isinstance(node, ast.If) and "auto_start" in ast.unparse(node.test):
            return node
    return None


def test_auto_start_parameter_declared_with_default_false():
    init = _method(_tree(), "__init__")
    declared = [
        call
        for call in _calls(init, "self.declare_parameter")
        if call.args and _string_constants(call.args[0]) == {"auto_start"}
    ]
    assert declared, "缺少 auto_start 参数声明"
    # 默认值必须为 False（安全模式：不自动开跑）
    assert any(
        len(call.args) >= 2
        and isinstance(call.args[1], ast.Constant)
        and call.args[1].value is False
        for call in declared
    ), "auto_start 默认值必须为 False"


def test_auto_start_true_creates_start_timer():
    init = _method(_tree(), "__init__")
    auto_if = _auto_start_if(init)
    assert auto_if is not None, "缺少以 auto_start 为条件的分支"
    # auto_start=True 分支内必须创建启动定时器（回调为 _start_once）
    create_calls = [
        c
        for c in _calls(auto_if, "self.create_timer")
        if any(ast.unparse(a) == "self._start_once" for a in c.args)
    ]
    assert create_calls, "auto_start=True 时必须创建 _start_once 定时器"


def test_auto_start_false_does_not_create_timer():
    init = _method(_tree(), "__init__")
    auto_if = _auto_start_if(init)
    assert auto_if is not None, "缺少以 auto_start 为条件的分支"
    # auto_start=False 分支（orelse）不得创建定时器，且应把 _start_timer 置 None
    orelse_text = ast.unparse(auto_if.orelse)
    assert "create_timer" not in orelse_text, (
        "auto_start=False 分支不得创建启动定时器"
    )
    assert "self._start_timer=None" in orelse_text.replace(" ", ""), (
        "auto_start=False 时应将 _start_timer 置 None，明确无自动启动"
    )


def test_control_service_and_state_publisher_unconditional():
    """auto_start 不影响控制服务与状态发布者的创建。"""
    init = _method(_tree(), "__init__")
    auto_if = _auto_start_if(init)
    assert auto_if is not None

    # 控制服务：create_service(InspectionControl, control_service, _on_control)
    service_calls = _calls(init, "self.create_service")
    assert service_calls, "缺少 /fire_inspection/control 控制服务创建"
    for call in service_calls:
        assert not _is_inside(call, auto_if), (
            "控制服务创建必须在 auto_start 分支之外（两种模式都要提供）"
        )

    # 状态发布者：self._state_pub = self.create_publisher(String, state_topic, STATE_QOS)
    state_pub_assigns = _assignments(init, "_state_pub")
    assert state_pub_assigns, "缺少 /fire_inspection/state 发布者"
    for assign in state_pub_assigns:
        assert not _is_inside(assign, auto_if), (
            "状态发布者创建必须在 auto_start 分支之外（两种模式都要发布）"
        )


def test_initial_idle_state_published_before_auto_start_branch():
    init = _method(_tree(), "__init__")
    auto_if = _auto_start_if(init)
    assert auto_if is not None
    # 初始 IDLE 状态发布必须存在且不在 auto_start 分支内
    idle_calls = [
        c
        for c in _calls(init, "self._set_state")
        if "IDLE" in ast.unparse(c)
    ]
    assert idle_calls, "初始化时必须发布一次 IDLE 状态"
    for call in idle_calls:
        assert not _is_inside(call, auto_if), (
            "初始 IDLE 状态发布必须在 auto_start 分支之外"
        )


def _is_inside(node: ast.AST, container: ast.AST) -> bool:
    """node 是否位于 container 的子树中。"""
    for child in ast.walk(container):
        if child is node:
            return True
    return False


# -- auto_start 安全修复：_round_active 活动轮次门控 --------------------------

def _on_detection_method() -> ast.FunctionDef:
    return _method(_tree(), "_on_detection")


def test_round_active_guard_is_first_check_inside_watch_lock():
    """_on_detection 持有 _watch_lock 后，首个判断必须是 _round_active。"""
    m = _on_detection_method()
    # 找到 with self._watch_lock: 节点
    with_nodes = [
        n for n in ast.walk(m)
        if isinstance(n, ast.With)
        and any("watch_lock" in ast.unparse(item.context_expr)
                for item in n.items)
    ]
    assert with_nodes, "_on_detection 必须使用 with self._watch_lock"
    body = with_nodes[0].body
    # 第一条语句必须是 if not self._round_active: return
    first = body[0]
    assert isinstance(first, ast.If), "锁内首条必须是 if 判断"
    assert "self._round_active" in ast.unparse(first.test), (
        "首个判断必须基于 self._round_active"
    )
    assert "not" in ast.unparse(first.test), (
        "判断必须是 if not self._round_active"
    )
    assert any(isinstance(s, ast.Return) for s in ast.walk(first)), (
        "_round_active 为 False 时必须 return"
    )


def _assignments(node: ast.AST, attr: str) -> list[ast.Assign]:
    result: list[ast.Assign] = []
    for candidate in ast.walk(node):
        if isinstance(candidate, ast.Assign):
            for target in candidate.targets:
                if (isinstance(target, ast.Attribute)
                        and target.attr == attr):
                    result.append(candidate)
    return result


def _ancestor_with_lock(node: ast.AST, root: ast.AST) -> ast.With | None:
    """找到 node 最近的、以 self._watch_lock 为上下文的 with 祖先。"""
    # 构建 parent 映射
    parent = {}
    for child in ast.walk(root):
        for sub in ast.iter_child_nodes(child):
            parent[sub] = child
    cur = parent.get(node)
    while cur is not None:
        if (isinstance(cur, ast.With)
                and any("watch_lock" in ast.unparse(item.context_expr)
                        for item in cur.items)):
            return cur
        cur = parent.get(cur)
    return None


def test_run_and_run_manual_toggle_round_active():
    """_run / _run_manual 中所有 _round_active 赋值都必须在 _watch_lock 内。

    入口置 True、finally 置 False，且都要持锁，确保读写双方受同一把锁保护。
    """
    tree = _tree()
    for name in ("_run", "_run_manual"):
        m = _method(tree, name)
        assigns = _assignments(m, "_round_active")
        assert assigns, f"{name} 必须赋值 _round_active"
        values = [ast.unparse(a.value) for a in assigns]
        assert "True" in values, f"{name} 必须将 _round_active 置 True"
        # 所有赋值都必须位于 with self._watch_lock 块内
        for a in assigns:
            assert _ancestor_with_lock(a, m) is not None, (
                f"{name} 中 _round_active 赋值必须在 with self._watch_lock 内"
            )
        # finally 块内必须有置 False（可能嵌套在 with 块内，需递归查找）
        finally_blocks = [
            n.finalbody for n in ast.walk(m)
            if isinstance(n, ast.Try) and n.finalbody
        ]
        false_in_finally = False
        for body in finally_blocks:
            for stmt in body:
                for s in ast.walk(stmt):
                    if (isinstance(s, ast.Assign)
                            and any(isinstance(t, ast.Attribute)
                                    and t.attr == "_round_active"
                                    for t in s.targets)
                            and ast.unparse(s.value) == "False"):
                        false_in_finally = True
        assert false_in_finally, (
            f"{name} 的 finally 必须将 _round_active 置 False"
        )


def test_watch_finally_closes_gate_before_stop():
    """WATCH 分支 finally：_round_active=False 与 _send_head_cmd('stop')
    必须在同一个 with self._watch_lock 块内，且 False 在 stop 之前。

    保证 stop 是检测路径发出的最后一条头部指令。
    """
    run_round = _method(_tree(), "_run_round")
    # 收集所有 _send_head_cmd('stop') 调用，找到位于 with self._watch_lock 内的那个
    # （WATCH finally 的；导航 finally 的 stop 不在锁内）。
    stop_calls_in_lock = []
    for call in _calls(run_round, "self._send_head_cmd"):
        if not any(isinstance(a, ast.Constant) and a.value == "stop"
                   for a in call.args):
            continue
        lock_block = _ancestor_with_lock(call, run_round)
        if lock_block is not None:
            stop_calls_in_lock.append((call, lock_block))
    assert stop_calls_in_lock, (
        "必须存在一个位于 with self._watch_lock 内的 _send_head_cmd('stop')（WATCH finally）"
    )
    for call, lock_block in stop_calls_in_lock:
        # 同一锁块内必须有 _round_active=False 赋值
        false_assign = any(
            isinstance(s, ast.Assign)
            and any(isinstance(t, ast.Attribute)
                    and t.attr == "_round_active" for t in s.targets)
            and ast.unparse(s.value) == "False"
            for s in ast.walk(lock_block)
        )
        assert false_assign, (
            "WATCH finally 锁块内必须先置 _round_active=False"
        )
        # 顺序：False 赋值必须在 stop 调用之前（同锁块内语句顺序）
        false_idx = None
        stop_idx = None
        for i, s in enumerate(ast.iter_child_nodes(lock_block)):
            if (isinstance(s, ast.Assign)
                    and any(isinstance(t, ast.Attribute)
                            and t.attr == "_round_active" for t in s.targets)):
                false_idx = i
            if (isinstance(s, ast.Expr)
                    and isinstance(s.value, ast.Call)
                    and ast.unparse(s.value.func) == "self._send_head_cmd"):
                stop_idx = i
        assert false_idx is not None and stop_idx is not None, (
            "WATCH finally 锁块内必须同时有 _round_active=False 与 _send_head_cmd('stop')"
        )
        assert false_idx < stop_idx, (
            "WATCH finally 中 _round_active=False 必须在 _send_head_cmd('stop') 之前"
        )
