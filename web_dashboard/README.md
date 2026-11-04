# 天轶 2.5 消防巡检网页后台

独立的 `web_dashboard/` 子项目，提供巡检状态、控制入口、巡检点结果与历史记录的网页展示。

- **第一阶段（已完成）**：FastAPI + SQLite + 单页前端，全程离线，不连接机器人。
- **第二阶段（本机模拟已通过）**：独立的 ROS Bridge 进程（机器人端 Jazzy）经本机 HTTP
  与后端对接；后端默认仍是 `disabled` 适配器，**只有真实 Bridge 验证在线后才报告在线**。
  演示数据（demo）与真实数据严格隔离；模拟测试结果不等同于真实机器人测试。
- **第三阶段（已完成）**：统一异常事件（防空警报 / 视觉异常 / 语音播报 / 通信异常）、
  巡检进度、事件处理状态，以及默认关闭的开发模拟联调入口。事件字段为**网页侧建议契约**，
  待机器人端确认后再对接真实上报。
- **第五阶段（已完成）**：验收补测（空状态、异常/中止记录、进度一致性、老库迁移、
  重启持久性、数据库错误降级、筛选组合）、接口契约冻结（见 [接口契约.md](接口契约.md)）、
  新增「完整流程演示」模拟场景。

> 网页端可以完全独立运行和测试：不配置 `FIRE_DASHBOARD_ROS_ADAPTER=bridge` 时，
> 机器人恒为离线，控制接口一律 503，不会伪造任何成功结果。

## 目录结构

```
web_dashboard/
├── app/
│   ├── main.py          # FastAPI 应用装配、静态文件挂载
│   ├── config.py        # 环境变量配置（DB 路径 / 监听地址 / Bridge 地址与令牌）
│   ├── db.py            # SQLite 连接、建表、状态枚举（与 ROS 契约对齐）
│   ├── repository.py    # 巡检轮次 / 点位结果的读写与查询
│   ├── schemas.py       # API 响应模型 + 数据接入模型
│   ├── api.py           # /api/health /status /rounds /rounds/{id} /control /events
│   ├── ingest.py        # /api/ingest/*：Bridge 专用写入（回环 + 令牌）
│   ├── dev.py           # /api/dev/*：开发模拟入口（默认 403，需 DEV_MODE=1）
│   ├── simulate.py      # 9 个开发模拟场景 + 命令行入口
│   ├── ros_adapter.py   # DisabledRosAdapter（默认）/ BridgeRosAdapter（本机 HTTP）
│   └── seed_demo.py     # 演示数据写入脚本
├── bridge/              # 机器人端独立 Bridge（仅 ros_node.py import rclpy）
│   ├── __main__.py      # python -m bridge 入口
│   ├── ros_node.py      # rclpy 节点：状态 / 结果订阅 + 控制服务客户端 + HTTP 服务
│   ├── http_app.py      # Bridge 本机 HTTP：/status /health /control（令牌校验）
│   ├── gateway.py       # 与 rclpy 解耦的网关协议与错误类型
│   ├── round_tracker.py # 状态 → 轮次事件；唯一 round_id；重启恢复
│   ├── backend_client.py# 事件投递 + 持久化 outbox 重试
│   ├── outbox.py        # JSON 原子写 outbox
│   └── events.py        # 事件结构与类型
├── static/
│   ├── index.html       # 单页后台
│   ├── styles.css
│   └── app.js
├── tests/               # pytest：API / 数据层 / Bridge / 模拟端到端
├── data/                # SQLite 数据库与 Bridge 状态（已被 .gitignore 忽略）
├── requirements.txt
├── requirements-dev.txt
└── pyproject.toml       # pytest / ruff 配置
```

## 安装与启动

要求 Python 3.10+（开发机 Ubuntu 22.04 / ROS 2 Humble 可用，不依赖 rclpy）。

```bash
cd web_dashboard
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt

# 配置登录密码与会话密钥后启动（密码不要写进源码或脚本）
export FIRE_DASHBOARD_LOGIN_PASSWORD='团队共享密码'
export FIRE_DASHBOARD_SESSION_SECRET='一串足够长的随机密钥'
.venv/bin/uvicorn app.main:app --host 127.0.0.1 --port 8000
```

打开 <http://127.0.0.1:8000/>，未登录会先跳转登录页，输入团队共享密码后进入看板；
右上角「退出登录」可结束会话。API 文档 <http://127.0.0.1:8000/api/docs> 同样受登录保护。

> 未配置 `FIRE_DASHBOARD_LOGIN_PASSWORD` 时登录保持关闭（fail-closed）：所有页面与
> API（除 `/api/ingest/*` 与登录页外）都会被拒绝，不会静默放行。

如果 8000 端口已被其他程序占用，换一个端口即可，例如 `--port 8010`。

首次启动会自动创建 `data/dashboard.db` 与表结构。

### 可选配置（环境变量）

| 变量 | 默认值 | 说明 |
| --- | --- | --- |
| `FIRE_DASHBOARD_DB` | `data/dashboard.db` | SQLite 文件路径 |
| `FIRE_DASHBOARD_HOST` | `127.0.0.1` | 监听地址 |
| `FIRE_DASHBOARD_PORT` | `8000` | 监听端口 |
| `FIRE_DASHBOARD_ROS_ADAPTER` | `disabled` | `disabled`（离线）或 `bridge`（经本机 ROS Bridge） |
| `FIRE_DASHBOARD_BRIDGE_URL` | `http://127.0.0.1:8081` | Bridge 的 HTTP 地址 |
| `FIRE_DASHBOARD_BRIDGE_TOKEN` | 空 | 双向共享令牌；启用 `bridge` 时为必填 |
| `FIRE_DASHBOARD_BRIDGE_TIMEOUT_S` | `8.0` | 后端调用 Bridge 的超时（秒） |
| `FIRE_DASHBOARD_DEV_MODE` | 关闭 | 设为 `1` 才启用 `/api/dev/*` 模拟联调入口；**正式部署必须保持关闭** |
| `FIRE_DASHBOARD_LOGIN_PASSWORD` | 空 | 网页团队共享登录密码；**未配置时登录保持关闭（fail-closed），页面与 API 全部拒绝** |
| `FIRE_DASHBOARD_SESSION_SECRET` | 空（随机会话密钥） | 会话 Cookie 的 HMAC 签名密钥；多 worker / 需要重启后会话不失效时必填 |
| `FIRE_DASHBOARD_SESSION_TTL_S` | `43200` | 会话有效期（秒），默认 12 小时 |
| `FIRE_DASHBOARD_COOKIE_SECURE` | 关闭 | 设为 `1` 强制 `Secure` Cookie；HTTPS 反代后建议开启（HTTP 请求自动启用 Secure 反而会导致浏览器丢弃 Cookie） |

Bridge 进程另有：

| 变量 | 默认值 | 说明 |
| --- | --- | --- |
| `FIRE_DASHBOARD_BRIDGE_TOKEN` | 无（必填） | 与后端相同的令牌 |
| `FIRE_DASHBOARD_BACKEND_URL` | `http://127.0.0.1:8000` | 数据接入目标后端 |
| `FIRE_DASHBOARD_BRIDGE_STATE_DIR` | `data/bridge` | outbox 与当前轮次标记目录 |
| `FIRE_DASHBOARD_POI_TOPIC` | `/fire_inspection/poi_result` | 正式点位结果话题（待编排节点发布） |
| `FIRE_DASHBOARD_CONTROL_TIMEOUT_S` | `5.0` | 等待控制服务响应的超时 |

> 默认只监听 `127.0.0.1`；如需局域网访问，请自行评估安全边界，不要使用 `0.0.0.0` 直接暴露控制接口。

## 演示数据（demo）

```bash
.venv/bin/python -m app.seed_demo            # 写入默认数据库
.venv/bin/python -m app.seed_demo --db /tmp/demo.db
```

写入 4 轮演示巡检（正常完成 / 检出异常 / 中途停止 / 进行中），全部：

- `source = demo`、`is_demo = 1`，页面上显示橙色「演示数据」标记；
- 每个点位 `simulated = 1`，详情中显示「模拟」标记；
- 演示数据不会被 `?source=real` 查询命中，不会与真实记录混淆。

删除演示数据：直接删除对应数据库文件即可（不要删除真实数据）。

## 开发模拟联调（第三阶段，默认关闭）

用于在没有真实机器人的情况下验证全部网页功能。**仅供开发/测试，正式部署必须关闭。**

```bash
# 启动开发模式（服务端）
FIRE_DASHBOARD_DEV_MODE=1 .venv/bin/uvicorn app.main:app --host 127.0.0.1 --port 8000

# 或命令行写入指定数据库（不需要启动服务）
.venv/bin/python -m app.simulate --list
.venv/bin/python -m app.simulate alarm --db /tmp/sim.db
```

未设置 `FIRE_DASHBOARD_DEV_MODE=1` 时，`/api/dev/*` 一律返回 **403**，页面上的模拟面板不显示。

| 场景 key | 覆盖能力 |
| --- | --- |
| `normal_round` | 正常巡检开始与结束（3 点位全正常，进度 3/3） |
| `abnormal_round` | 点位识别异常（POI_02 异常 + 视觉异常事件） |
| `alarm` | 检测到防空警报（只写警报事件，`details.voice_broadcast = "pending"`） |
| `voice_success` | 语音播报成功（独立 `voice_broadcast` 事件） |
| `voice_failure` | 语音播报失败（独立 `voice_broadcast` 事件，severity=critical） |
| `connection_lost` | 机器人离线 / Bridge 断开（通信异常事件） |
| `connection_restored` | 通信恢复事件 |
| `duplicate_event` | 同一 `event_id` 上报两次，只保留一条（幂等） |
| `full_flow` | 完整流程：开始 → 点位正常 → 点位异常 → 防空警报 → 播报成功 → 巡检结束 → 历史可查 |

模拟数据的安全约束：

- 轮次 `source = simulated`（页面显示「模拟数据」），事件 `simulated = true`（页面显示「模拟」）；
- 不会改变 `robot.online`，不会调用任何控制接口，不会伪造控制成功；
- 收到 `alarm_detected` **不代表已完成语音播报**，播报结果必须由独立的 `voice_broadcast` 事件表示。

## 测试与静态检查

```bash
cd web_dashboard
.venv/bin/pip install -r requirements-dev.txt
.venv/bin/python -m pytest
.venv/bin/ruff check .
```

测试使用临时数据库，不读写 `data/` 下的真实数据。

第二阶段新增测试（全部为本机模拟，不启动导航、不调用真实控制服务）：

- `test_round_tracker.py`：轮次唯一 ID、重启补发、状态映射、结果归属；
- `test_outbox.py`：持久化、ack、损坏隔离、容量上限；
- `test_bridge_http.py`：令牌、在线判定、控制确认、去重、503/504/400；
- `test_ingest_api.py`：令牌、幂等建轮、原始字段保留、终态 409、outcome 推导；
- `test_bridge_adapter.py`：Bridge 在线/离线、控制确认、配置校验；
- `test_simulated_e2e.py`：真实 HTTP 后端 + 假网关 + Bridge 全链路，含演示数据隔离。

第三阶段新增测试（`tests/test_events_api.py`）：

- 事件字段保真（`event_id / event_type / timestamp / source / severity / message / details`）；
- `simulated` 标记、`event_id` 幂等去重、非法类型 422；
- 事件筛选 / 分页 / 详情 404 / 标记处理与重复标记；
- `/api/dev/*` 默认 403、场景列表、未知场景 404；
- `alarm` 不产生播报事件、`voice_success` / `voice_failure` 独立事件；
- 通信事件不改变机器人在线状态、`normal_round` 进度与结果、`abnormal_round` 视觉事件、
  `duplicate_event` 只落一条。

第五阶段验收补测（只补真实缺口，不重复已有用例）：

- 首页空状态：无轮次、无事件时 `current_round=null`、列表 `total=0`；
- 中止（`aborted`）记录可显示，`stopped` 筛选为空时行为正确；
- 进度一致性：跳过点位计入 `processed_count`、不计入 `recorded_count`；
- 老库迁移：缺少 `total_pois` / `event_id` / `event_record` 的旧库无损升级，迁移后新功能可用；
- 重启持久性：重新 `init_db` 后轮次与事件仍可查；
- 数据库错误：`/api/health` 降级为 `degraded`，其它接口返回 503 而非 500；
- 筛选组合：`status + outcome + source` 组合筛选；
- 模拟隔离：开发模拟模式下 `/api/control` 仍返回 503，不会触发真实机器人动作；
- `full_flow` 场景：一条命令覆盖完整演示链条，事件全部带 `simulated`。

## 验收清单（第五阶段）

| 验收项 | 结论 | 依据 |
| --- | --- | --- |
| 无数据空状态 | 通过 | `test_status_offline_without_round`、`test_rounds_empty`、`test_events_empty_state` |
| 正常/异常/停止/中止记录 | 通过 | `test_demo_data_is_marked_and_queryable`、`test_round_filters_combine_and_show_aborted` |
| 点位结果与进度一致 | 通过 | `test_progress_counts_skipped_but_not_recorded`、`test_dev_normal_round_progress_and_results` |
| 事件查询/详情/确认/历史保留 | 通过 | `test_events_api.py` 事件用例 + `test_rounds_and_events_persist_across_restart` |
| 重复上报幂等 | 通过 | `test_duplicate_event_id_is_idempotent`、`test_dev_duplicate_event_scenario_keeps_one_record` |
| 老库迁移与重启持久性 | 通过 | `test_legacy_database_migration_keeps_old_rows`、`test_rounds_and_events_persist_across_restart` |
| 超时/503/404/无效请求/数据库错误 | 通过 | `test_control_offline_returns_503_and_writes_nothing`、`test_round_detail_not_found`、`test_filters_reject_unknown_values`、`test_bridge_http.py`（504 超时）、`test_health_reports_database_error` |
| 分页与筛选组合 | 通过 | `test_event_filters_and_pagination`、`test_round_filters_combine_and_show_aborted`、`test_pagination_and_order` |
| 模拟/真实模式区分 | 通过 | `test_dev_normal_round_progress_and_results`、`test_simulated_event_stays_marked`、`test_dev_control_never_reaches_robot` |
| 开发入口默认关闭 | 通过 | `test_dev_entry_disabled_by_default` |

> 所有结论来自本机自动化测试与开发模式下的浏览器验证，**不代表真实机器人联调已通过**。

## API 一览

| 方法 | 路径 | 说明 |
| --- | --- | --- |
| GET | `/api/health` | 数据库与 ROS 适配层健康状态 |
| GET | `/api/status` | 机器人连接状态 + 当前进行中的巡检轮次（含点位结果） |
| GET | `/api/rounds` | 历史记录列表，支持 `status` / `outcome` / `source` / `limit` / `offset` |
| GET | `/api/rounds/{round_id}` | 单轮详情（点位、结果、说明、检测数据、时间） |
| POST | `/api/control` | 控制指令入口；离线或 Bridge 未确认时返回 **503**，不写库、不伪造成功 |
| POST | `/api/ingest/rounds` | Bridge 建轮（按 `round_id` 幂等，201 新建 / 200 已存在） |
| POST | `/api/ingest/rounds/{id}/state` | Bridge 更新 running / paused |
| POST | `/api/ingest/rounds/{id}/results` | Bridge 写入正式点位结果（按 `event_id` 幂等） |
| POST | `/api/ingest/rounds/{id}/finish` | Bridge 结束轮次（completed / stopped / aborted） |
| GET | `/api/events` | 异常事件列表；支持 `event_type` / `severity` / `status` / `source` / `round_id` / `limit` / `offset` |
| GET | `/api/events/{event_id}` | 单条事件详情（事件不存在返回 404） |
| POST | `/api/events/{event_id}/ack` | 标记已处理：`{"operator_id": "", "status": "acknowledged|resolved"}` |
| POST | `/api/ingest/events` | Bridge 上报事件（需 `X-Bridge-Token`；按 `event_id` 幂等） |
| GET | `/api/dev/scenarios` | 列出开发模拟场景；**默认 403**，仅 `FIRE_DASHBOARD_DEV_MODE=1` 可用 |
| POST | `/api/dev/scenarios/{key}` | 执行一个模拟场景（模拟数据，非真实机器人结果） |

`/api/ingest/rounds` 可额外携带 `total_pois`（计划点位总数，可选）；不提供时页面不显示
进度百分比，避免臆造。`/api/rounds` 与详情响应新增 `total_pois` / `processed_count`。

`/api/control` 请求体：`{"cmd": "start|pause|resume|skip|stop|manual", "arg": "", "operator_id": ""}`。
`/api/ingest/*` 必须携带 `X-Bridge-Token`；后端未配置令牌时整组接口返回 503。

## 数据模型

与 ROS 接口契约（`接口契约冻结表.md`）对齐；完整字段级契约（含「已实现 / 网页端约定 /
待机器人端确认」标注）见 [接口契约.md](接口契约.md)：

- `inspection_round`：一轮巡检。`status` ∈ `running / paused / completed / stopped / aborted`；
  `outcome` ∈ `normal / abnormal / uncertain`（`NULL` 表示尚未得出）；`source` ∈ `real / demo / simulated`。
- `poi_result`：单个巡检点结果。`status_code` 与 `PoiResult.msg` 一致：
  `0 NORMAL / 1 ABNORMAL / 2 UNCERTAIN / 3 TIMEOUT / 4 NO_FRAME / 5 NO_MODEL / 6 ERROR`；
  包含 `simulated`、`detail`、`detections_json`（页面解析为检测标签）、`checked_at`、`duration_s`。

跳过（skip）不写视觉状态码，而是 `skipped = 1` + `status_name = SKIPPED`，
避免把"跳过"伪装成某种检测结论；`NO_MODEL` / `NO_FRAME` 等也不会被判为正常。

- `event_record`：统一异常事件。`event_id` 全局唯一（重复上报不产生第二条）；
  `event_type` ∈ `alarm_detected / vision_abnormal / voice_broadcast / comm_failure / comm_restored`；
  `severity` ∈ `info / warning / critical`；处理状态 `status` ∈ `open / acknowledged / resolved`。

### 事件字段契约（网页侧建议，⚠️ 待机器人端确认）

```json
{
  "event_id": "evt-20261029-0001",
  "event_type": "alarm_detected",
  "timestamp": "2026-10-29T07:41:19+00:00",
  "source": "alarm_monitor",
  "severity": "critical",
  "message": "检测到防空警报",
  "details": { "zone": "3F 东走廊", "voice_broadcast": "pending" }
}
```

| 字段 | 说明 |
| --- | --- |
| `event_id` | 全局唯一；机器人端重试必须复用同一 ID（契约字段，待确认） |
| `event_type` | 五种类型，见上；新增类型需前后端同步（待确认） |
| `timestamp` | ISO 8601；缺省时后端用接收时间填充（待确认） |
| `source` | 1–32 位字母数字/`_.-`，如 `alarm_monitor` / `vision` / `voice` / `bridge`（待确认） |
| `severity` | `info / warning / critical`（待确认） |
| `message` | 面向操作员的一句话说明（待确认） |
| `details` | 附加结构，随类型不同（待确认） |
| `round_id` / `poi` | 可选，关联巡检轮次与点位（待确认） |
| `simulated` | 网页侧自有标记，模拟数据恒为 `true` |

接口侧另有网页自用字段：`status`（处理状态）、`handled_by` / `handled_at`（处理人与时间）。

> **播报与警报解耦**：`alarm_detected` 只表示"检测到警报"，不代表已完成语音播报；
> 播报结果必须由独立的 `voice_broadcast` 事件（`details.result = success/failure`）表示。

## 当前限制

1. **默认仍是 `DisabledRosAdapter`**：没有配置 `FIRE_DASHBOARD_ROS_ADAPTER=bridge`
   并启动 Bridge 时，在线状态恒为离线，控制请求一律 503。
2. 控制按钮在离线时禁用；即使直接调用 API 也只会收到 503，且不会写数据库。
3. 首页的"当前巡检状态"来自数据库中最近一条 `running / paused` 记录。
4. 没有登录鉴权。仅监听 `127.0.0.1`，不要把控制接口直接暴露到网络。
5. 正式点位结果依赖编排节点发布 `/fire_inspection/poi_result`；该发布点已在第三阶段加入
   `orchestrator_node.py`，但**尚未在真实机器人 Jazzy 环境验证**。
6. 异常事件目前只由开发模拟入口产生；机器人端的防空警报 / 语音播报 / 视觉异常 / 通信
   事件上报接口**尚未实现**（见文末「机器人端接口清单」）。
7. 本仓库内的测试全部是本机模拟（假网关 / 假 ROS），**不代表真实机器人测试已通过**。

## 第二阶段架构与数据流

```
机器人（Ubuntu 24.04 + ROS 2 Jazzy）
┌──────────────────────────────────────────────────────────────┐
│  orchestrator / navigation / vision / voice（现有，不改动）  │
│        ▲ /fire_inspection/control (InspectionControl)        │
│        │ /fire_inspection/state (std_msgs/String,            │
│        │   reliable + transient_local, depth 1)              │
│        │ /fire_inspection/poi_result（编排节点已发布，待实机验证）    │
│  ┌─────┴──────────────────────────────────────────────────┐  │
│  │ bridge/ros_node.py（独立进程，唯一 import rclpy 的文件）│  │
│  │  - 状态 → RoundTracker → 事件                            │  │
│  │  - 控制 → InspectionControl 服务客户端                   │  │
│  │  - HTTP 127.0.0.1:8081（/status /health /control）       │  │
│  └─────┬──────────────────────────────────────────────────┘  │
└────────┼─────────────────────────────────────────────────────┘
         │ 本机 HTTP + X-Bridge-Token（控制、状态）
         │ 本机 HTTP + X-Bridge-Token（事件投递 /api/ingest/*）
┌────────┴─────────────────────────────────────────────────────┐
│ FastAPI 后端（app/）：BridgeRosAdapter + /api/ingest + SQLite │
└──────────────────────────────────────────────────────────────┘
```

### 为什么选本机 HTTP

- 两端都在同一台机器人上，HTTP 调试成本最低（curl 即可验证），不需要额外依赖。
- FastAPI 与 Bridge 完全解耦：Bridge 只依赖 Python 标准库（机器人端无需安装 FastAPI）。
- 回环地址 + 共享令牌满足当前单机安全边界；未引入 Unix socket 的权限/路径复杂性。

### 事件协议（Bridge → 后端）

| kind | 接口 | payload |
| --- | --- | --- |
| `round_start` | `POST /api/ingest/rounds` | `mode` / `operator_id` / `started_at` / `ros_state` |
| `round_state` | `POST /api/ingest/rounds/{id}/state` | `status`(running/paused) / `ros_state` |
| `poi_result` | `POST /api/ingest/rounds/{id}/results` | `event_id` / `poi` / `status_code` / `simulated` / `detail` / `detections_json` / `checked_at` |
| `round_finish` | `POST /api/ingest/rounds/{id}/finish` | `status`(completed/stopped/aborted) / `summary` / `finished_at` |

每个事件都带 `event_id`；结果记录按 `(round_id, event_id)` 唯一索引去重，
Bridge 重试或后端重复收到同一事件都不会产生重复点位。

### 每轮任务 ID 与结果关联

- `round_id` 由 Bridge 生成（`run-<16 位十六进制>`），首次进入
  `running:*` / `paused:*`（`watch` 除外）时创建，并写入 `data/bridge/active_round.json`。
- 所有事件和点位结果都携带同一个 `round_id`，后端按它写入对应巡检记录，
  **不存在按时间或序号猜测归属**。
- Bridge 重启后读取 marker，用同一 `round_id` 补发幂等建轮事件，继续写入同一轮，
  避免重启后串轮。
- `done / stopped / aborted` 或回到 `idle` 时结束轮次并删除 marker；下一轮重新生成 ID。

### 失败与边界语义

| 场景 | 行为 |
| --- | --- |
| Bridge 不可达 / 超时 | 后端 503，`dispatched=false`，消息明确「无法确认是否执行」 |
| ROS 控制服务不可发现 | Bridge 返回 503，`dispatched=false` |
| 控制服务无响应 | Bridge 返回 504，`dispatched=false`（结果未知） |
| 重复控制请求 | 同一 `request_id` 在 Bridge 内去重，返回 `deduplicated=true`，不重复调用服务 |
| 后端暂时不可用 | 事件留在 `outbox.json`，后台线程重试；4xx 视为毒事件丢弃并记录日志 |
| 后端 SQLite 写失败 | 接口返回 5xx → Bridge 视为暂时失败并重试，不丢事件 |
| outbox 超过 1000 条 | 拒绝入队并报错，防止占满磁盘（需排查后端/网络） |
| 终态轮次再收到结果 | 后端 409，拒绝污染已结束记录 |
| 状态话题无结构化结果 | 不写任何点位结果，绝不根据状态文本推断检测结论 |

`simulated=true` 的正式结果保留原始 `simulated` 标记，页面照实显示；
演示数据只能由 `seed_demo` 写入（`source=demo`），接入接口固定写 `source=real`。

## 第三阶段已实施的机器人侧改动（未在真实机器人验证）

外部 Bridge 无法直接获取 `InspectPoi` Action 的结果，因此在
`fire_inspection_ws/src/fire_inspection_orchestrator/fire_inspection_orchestrator/orchestrator_node.py`
增加了最小改动（约 5–8 行）：

1. 在 `__init__` 中创建一个 `PoiResult` 发布者（话题 `/fire_inspection/poi_result`，
   QoS `reliable + volatile`、depth 10，不使用 `transient_local` 以避免旧结果重放串轮）；
2. 在 `_inspect()` 拿到 Action `result` 后（`simulated` 判断分支之前）发布一条
   `PoiResult`，字段直接来自 `result.status / simulated / detail / detections_json`；
3. 跳过（skip）、Action 调用失败、超时无有效结果时**不发布**，保持「跳过不是视觉结论」；
4. 发布失败只记录告警，不阻断巡检主流程。

> 该改动尚未在真实机器人 Jazzy 环境验证。未修改导航、视觉、语音代码。

## 机器人端接口清单（待提供）

网页端已完成事件的数据模型、查询、处理和模拟入口；机器人端需要补充：

| 能力 | 建议提供方 | 说明 |
| --- | --- | --- |
| 防空警报事件 | 警报监听节点（待开发） | 检测到防空警报后上报 `alarm_detected`，不得同时冒充播报完成 |
| 语音播报结果 | 语音节点 | 独立上报 `voice_broadcast`，`details.result = success / failure` |
| 视觉异常事件 | 视觉 / 编排节点 | 点位异常时可上报 `vision_abnormal`（点位结果仍走 `poi_result`） |
| 通信异常 / 恢复事件 | Bridge | Bridge 检测到与后端或 ROS 的断开与恢复后上报 `comm_failure` / `comm_restored` |
| 事件上报通道 | Bridge | 复用 `POST /api/ingest/events` + `X-Bridge-Token`，`event_id` 幂等 |
| 计划点位总数 | 编排 / Bridge | 建轮时通过 `/api/ingest/rounds` 携带 `total_pois`，页面才能显示进度百分比 |
| 事件字段确认 | 机器人端 | 逐项确认上文「事件字段契约」中的契约字段 |

联调顺序建议：

1. 先在网页端用 `FIRE_DASHBOARD_DEV_MODE=1` 跑通 9 个模拟场景（本机、无机器人）；
2. 机器人端确认事件字段契约，并在 Bridge 中实现事件上报（`event_id` 必须幂等）；
3. 联调前关闭 `FIRE_DASHBOARD_DEV_MODE`，确认 `/api/dev/*` 已返回 403；
4. 仅观察状态与事件，不发送底盘运动指令；确认无伪造数据后再进入正式使用。

## 真实机器人部署步骤（尚未执行）

1. 机器人端安装依赖并构建接口：`colcon build --packages-select fire_inspection_interfaces`；
2. 生成两端一致的强随机令牌（如 `openssl rand -hex 32`），写入两个服务的环境变量；
3. **编排层以 Dashboard 安全模式启动**（不自动巡检，只提供控制服务与状态话题）：
   必须与导航 / 视觉 / 语音 / 头部一起作为**唯一一份常驻链路**启动，否则网页
   「启动」后会出现「头动了但底盘不动」：
   ```bash
   ros2 launch fire_inspection_orchestrator inspection_bringup.launch.py \
     dry_run:=false start_head:=true start_audio:=true \
     auto_start:=false nav_enabled:=true
   ```
   推荐由 `fire-inspection.service` 开机常驻（见项目根目录的
   `fire-inspection.service`，安装方法见下文「开机自启」）。
   **不要**只单独启动 `orchestrator.launch.py` 或 `orchestrator_node`，
   那样网页虽然有控制服务，但导航 / 视觉 Action 往往并不存在。

   > **关于视觉检测与防空警报**：`auto_start:=false` 且无活动巡检轮次时，编排层
   > 会丢弃视觉检测流（`_on_detection` 直接返回），不会触发头部锁定、语音播报或
   > 吸烟介入线程。**但防空警报（`_on_audio_alert`）属于独立安全事件，不受
   > `auto_start` 控制**：只要 audio 节点发布 `/fire_inspection/audio_alert`，
   > 即使巡检处于 IDLE 也会抓拍画面、暂停头部扫视并播报警报文案。这是有意设计，
   > 安全警报不能因没有巡检任务而被忽略。
4. 后端：`FIRE_DASHBOARD_ROS_ADAPTER=bridge`、`FIRE_DASHBOARD_BRIDGE_TOKEN=...`、
   `uvicorn app.main:app --host 127.0.0.1 --port 8000`；
5. Bridge：`FIRE_DASHBOARD_BRIDGE_TOKEN=... python -m bridge`（在 Jazzy 环境下）；
6. 验证：`curl -H 'X-Bridge-Token: ...' http://127.0.0.1:8081/status` 显示
   `online=true` 前，网页不会报告在线；
7. 先只观察状态与日志，确认未误发指令，再逐步验证 `pause` / `resume` 等非运动指令；
8. 用 systemd 托管后端、Bridge 与巡检链路，保持 `127.0.0.1` 监听。

### 开机自启（巡检全链路 + 遥控器 B 键）

网页「启动」和遥控器 B 键共用 `/fire_inspection/control` 这一个入口，
因此开机只允许存在一份巡检链路：

```bash
# 机器人端（项目根目录）
sudo cp fire-inspection.service button-trigger.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now fire-inspection.service
sudo systemctl enable --now button-trigger.service
# 全链路已自带头部节点，停用旧的独立头部自启，避免两个节点同时下发 /head/cmd
sudo systemctl disable --now head-sweep.service
```

启动顺序与依赖已写在单元里：`fire-inspection.service` 等
`robot-control.service` / `proc_manager.service`；`button-trigger.service`
`Wants=` / `After=` `fire-inspection.service`。

验证链路是否就绪（不运动）：

```bash
systemctl status fire-inspection.service --no-pager
ros2 service list | grep -x /fire_inspection/control
ros2 action list -t | grep fire_inspection
ros2 topic echo --once /fire_inspection/state std_msgs/String   # 期望 idle
```

此时网页「启动」或按 B 键才会真正开始巡检；两者最终都调用编排层
`_begin_round()` → `_run()` 和同一个 `NavigateTo` Action，不会再拉起第二套服务。
