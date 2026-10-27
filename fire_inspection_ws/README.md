# fire_inspection_ws —— 天轶 2.5 消防巡检 ROS 2 工作空间

消防轨道巡检场景的 ROS 2 工作空间：天轶 2.5 人形机器人沿**预设轨道**巡航，
在**固定点位**停下，用**视觉**检查灭火器、消防设施、安全出口是否正常，
用**语音**播报巡检状态与异常提示。

- **ROS 版本**：ROS 2 Jazzy
- **本体**：天轶 2.5 实机（头部相机 + 移动底盘）
- **导航**：底盘 REST `http://192.168.11.10:9090`，点位以底盘 POI 服务端为唯一真源
- **构建**：colcon（`ament_cmake` + `ament_python`）
- **语言**：Python 3（`rclpy`）

> ⚠️ **当前阶段：导航（dry_run + 实机运动）、视觉检查、头部扫视已实现；语音与编排层为骨架**
>
> | 模块 | 状态 |
> | --- | --- |
> | `fire_inspection_nav` | ✅ dry_run 只读链路 + 实机轨道运动（AEB、到点转向、限速备份回滚、取消、超时、返回起点） |
> | `fire_inspection_vision` | ✅ InspectPoi 服务端（头部相机取图 + YOLO 推理 + 判定，无模型/无图/超时绝不判正常） |
> | `fire_inspection_head` | ✅ 头部扫视服务端（`/fire_inspection/head_sweep_cmd` 控制、手柄急停、systemd 自启） |
> | `fire_inspection_voice` | ⬜ 骨架，未注册 Action、不调 TTS |
> | `fire_inspection_orchestrator` | ⬜ 骨架，未实现状态机 |
>
> **实机运动已实现**：起点与过点点位（`POI_04`）走点对点，巡检段沿 `tracks.outbound`，
> 返程沿 `tracks.return`；轨道到点后原地转向到点位朝向（`POI_03` 以地图中的 -60° 为准），
> 校验实际朝向，偏差超过 0.10 rad 重试一次；若轨道末航点朝向已一致则不补转。
> 每个巡检点位转向完成后停留 `dwell_s`（默认 5s），`dwell_overrides` 可按点位覆盖
> （`POI_01`/`POI_02` 停 5s、`POI_04` 停 3s），起点与返程不停留。
> 注意：底盘固件会忽略 `move_to_locations` 的 `target_position.yaw`，且跨全图的点对点返程会规划失败，因此返程必须走轨道。
> 每段运动前开 AEB、轮询动作状态到终态，成功/失败/取消都会停车并关 AEB。
> 实机模式启动时严格校验定位（`slam_status != 0` 直接拒绝所有 goal）与底盘健康（A 类故障直接拒绝并输出 DTC），
> 每段运动前按 `speed.outbound_mps` / `speed.return_mps`（默认均为 0.4 m/s）下发限速，
> 首次写入前备份底盘原值，节点退出时恢复。
> 导航服务端只使用底盘 SDK，不初始化 `robot_controller` 门面，避免连带拉起手臂 / 手部 / 相机。
> dry_run 下定位状态异常（`slam_status != 0`）只告警不拒绝，因为不下发运动；实机模式必须严格通过。

---

## 1. 快速开始

```bash
# 0) 进入工作空间
cd /home/nvidia/fire-inspection/fire_inspection_ws

# 1) 前置：source ROS 与实机平台环境（后者提供 interaction_msgs）
source /opt/ros/jazzy/setup.bash
source /home/nvidia/xos/setup.bash

# 2) 构建（首次或改动 package.xml / setup.py / CMakeLists.txt 后）
colcon build --symlink-install

# 3) source 工作空间（每个新终端都要执行）
source install/setup.bash

# 4) 一键拉起全链路（导航 + 语音 + 视觉 + 编排层），默认模拟模式
ros2 launch fire_inspection_orchestrator inspection_bringup.launch.py

# 5) 实机模式（显式关闭 dry_run，需有人守护、急停可达）
ros2 launch fire_inspection_orchestrator inspection_bringup.launch.py dry_run:=false
```

> 🔴 实机安全：首次调试务必低速、空载、有人守护、急停可达；
> 限速参数写入前必须备份原值，写入或验证失败必须回滚。

---

## 2. 目录结构

```
fire_inspection_ws/
├── src/
│   ├── fire_inspection_interfaces/      # ① 接口（msg/srv/action），依赖图最底层
│   ├── fire_inspection_nav/             # ② 轨道导航服务端（NavigateTo Action）
│   ├── fire_inspection_voice/           # ③ 语音播报服务端（Speak Action）
│   ├── fire_inspection_vision/          # ④ 视觉检查服务端（InspectPoi Action）
│   ├── fire_inspection_head/            # ⑤ 头部扫视服务端（/head/cmd + 手柄急停）
│   └── fire_inspection_orchestrator/    # ⑥ 编排层（唯一指挥者 / 唯一状态机 + 一键 bringup）
├── build/  install/  log/               # colcon 产物（不入库）
└── README.md
```

---

## 3. 包一览与依赖分层

| # | 包 | 类型 | 职责 | 可执行入口 |
| --- | --- | --- | --- | --- |
| ① | `fire_inspection_interfaces` | `ament_cmake` | 7 个自建接口（3 msg / 1 srv / 3 action） | —（接口包） |
| ② | `fire_inspection_nav` | `ament_python` | 底盘 REST 导航服务端，沿轨道逐点到点 | `navigate_to_server` |
| ③ | `fire_inspection_voice` | `ament_python` | 平台 TTS 播报服务端 | `speak_action_server` |
| ④ | `fire_inspection_vision` | `ament_python` | 取图 + YOLO，判定点位设施是否正常 | `inspect_action_server` |
| ⑤ | `fire_inspection_head` | `ament_python` | 头部扫视，按配置序列下发 `/head/cmd`，手柄可急停 | `head_sweep_server` |
| ⑥ | `fire_inspection_orchestrator` | `ament_python` | 状态机，按点位顺序调用三个 Action | `orchestrator_node` |

### 自建接口

| 名称 | 类型 | 用途 |
| --- | --- | --- |
| `ConsoleCommand` | msg | 控制指令：`start / pause / resume / skip / stop / manual` |
| `SubsystemHealth` | msg | 底盘、语音、相机、模型、POI 表、定位状态汇总 |
| `PoiResult` | msg | 单点位检查结果（含 `simulated` 标记） |
| `InspectionControl` | srv | 巡检启停控制 |
| `NavigateTo` | action | 前往指定 POI |
| `Speak` | action | 播报一段文本 |
| `InspectPoi` | action | 检查指定 POI 的设施 |

### 依赖与调用层级

```
fire_inspection_interfaces ──（所有包的接口依赖，最底层）
        │
        ├──► fire_inspection_nav      （NavigateTo）
        ├──► fire_inspection_voice    （Speak）
        ├──► fire_inspection_vision   （InspectPoi）
        │
        └──► fire_inspection_orchestrator（唯一指挥者）

fire_inspection_head ──（独立链路：直接下发 /head/cmd，不经过接口包）
```

- 构建顺序由包依赖自动决定；`--symlink-install` 便于改 Python / 配置后免重建。
- `interaction_msgs`（voice）、`ros2_bridge_msgs`（head）来自 `/home/nvidia/xos`，
  `rosdep` 无法解析，只能通过 `source` 提供。

---

## 4. 功能架构与数据流

```
                      ┌──────────────────────────────────┐
                      │   fire_inspection_orchestrator   │  唯一指挥者 / 唯一状态机
                      └───┬────────────┬────────────┬────┘
              Action 客户端 │            │            │
                          ▼            ▼            ▼
                    NavigateTo       Speak       InspectPoi
                          │            │            │
              ┌───────────┘            │            └────────────┐
              ▼                        ▼                         ▼
    fire_inspection_nav     fire_inspection_voice      fire_inspection_vision
      （轨道导航）              （语音播报）                （取图 + YOLO）
              │                        │                         │
              ▼                        ▼                         ▼
   底盘 REST :9090          /intelligent_interaction        头部相机话题
   （POI 表唯一真源）          /tts/play → /audio_node        → 推理 → 判定
```

**导航**：`fire_inspection_nav` → 底盘 REST `:9090`，发起前校验定位正常、目标 POI 存在。
**语音**：`fire_inspection_voice` → 平台 TTS 服务，说完一句再进入下一点位，避免抢播。
**视觉**：`fire_inspection_vision` → 取一帧图像做推理，按“异常优先、不确定不判正常”的规则给出结论。

### 巡检流程

```
IDLE → INIT（前置校验 + 限速设置） → TO_START
     → 每个点位循环： NAV_TO_POI → INSPECT → REPORT（语音播报）
     → [RETURN_TO_START] → DONE
任意状态 ──取消/失败──► ABORTED ──清理──► IDLE
```

默认点位顺序（可在 `fire_inspection_nav/config/pois.yaml` 调整）：

```
start_point → POI_01 → POI_02 → POI_03
```

### 判定规则（视觉）

| 情况 | 结论 |
| --- | --- |
| 检出异常类别且置信度达标 | `ABNORMAL` |
| 期望目标全部检出 | `NORMAL` |
| 有检出但置信度不足 / 缺目标 | `UNCERTAIN`（**不得判正常**） |
| 无模型 / 无图像 / 超时 / 推理异常 | `NO_MODEL` / `NO_FRAME` / `TIMEOUT` / `ERROR` |

模拟模式下所有结果必须带 `simulated=true`，不得当作真实识别结论上报。

---

## 5. 启动入口

| 命令 | 作用 | 归属包 |
| --- | --- | --- |
| `ros2 launch fire_inspection_orchestrator inspection_bringup.launch.py` | **一键全链路**（导航 + 语音 + 视觉 + 编排） | orchestrator |
| `ros2 launch fire_inspection_nav navigate_to.launch.py` | 仅导航服务端 | nav |
| `ros2 launch fire_inspection_voice speak.launch.py` | 仅语音服务端 | voice |
| `ros2 launch fire_inspection_vision inspect.launch.py` | 仅视觉服务端 | vision |
| `ros2 launch fire_inspection_head head_sweep.launch.py` | 仅头部扫视（`auto_start:=true` 可延时自启） | head |
| `ros2 launch fire_inspection_orchestrator orchestrator.launch.py` | 仅编排层（调试用） | orchestrator |

所有 launch 均支持 `dry_run:=true|false`；`inspection_bringup.launch.py` 会把该参数透传给四个子链路。

### 头部扫视开机自启（systemd）

模板在 `fire_inspection_head/scripts/head-sweep.service`，安装：

```bash
sudo cp src/fire_inspection_head/scripts/head-sweep.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now head-sweep.service
```

> ⚠️ 该模板是**本机部署文件**，`User=nvidia`、`HOME=/home/nvidia`、
> `/home/nvidia/xos`、`/home/nvidia/fire-inspection/fire_inspection_ws/install` 都是本机绝对路径；
> 换机器直接使用会启动失败，需按实际用户与安装路径修改。
> 它同时 `After=` / `Wants=` `robot-control.service`，若你的底盘服务名不同要一并改。
> 自启时通过 `auto_start:=true` 延时（`auto_start_delay_s`，默认 30s）等 `/robot_state` 就绪。

### 5.1 导航 dry_run 验证

```bash
# 终端 1：启动导航服务端（只读，不动车）
ros2 launch fire_inspection_nav navigate_to.launch.py

# 终端 2：按顺序发 goal
ros2 action send_goal /fire_inspection/navigate_to fire_inspection_interfaces/action/NavigateTo "{poi: 'start_point'}" --feedback
ros2 action send_goal /fire_inspection/navigate_to fire_inspection_interfaces/action/NavigateTo "{poi: 'POI_01'}"
ros2 action send_goal /fire_inspection/navigate_to fire_inspection_interfaces/action/NavigateTo "{poi: 'POI_02'}"
ros2 action send_goal /fire_inspection/navigate_to fire_inspection_interfaces/action/NavigateTo "{poi: 'POI_03'}"
# 返回起点：is_return 必须为 true，且 poi 固定为 start_point
ros2 action send_goal /fire_inspection/navigate_to fire_inspection_interfaces/action/NavigateTo "{poi: 'start_point', is_return: true}"
```

预期行为：
- 顺序不对（如重复请求 `POI_01`）会返回 `success: false`，提示"点位顺序不匹配"；
- `is_return: true` 只用于"返回起点"：`poi` 非 `start_poi` 会被拒绝；未开始巡检时不能返回；
  已在起点时重复返回会被拒绝；中途返回后，下一个正常 goal 从原进度继续，不重走已完成点位；
- 首个 `start_point` 与 `POI_04`（`transit_pois`）走点对点，巡检段走 `tracks.outbound`，
  返程走 `tracks.return`（专门从 POI_04 回起点），到点后转为起点朝向；
- `timeout_s` 传 0 或缺省时：`is_return: true` 用 `per_poi_s`，首次前往起点用 `to_start_s`，
  其余用 `per_poi_s`（返程不再借用 `to_start_s`）；
- 配置自检：`start_poi` 不得出现在 `poi_order`，`poi_order` 不得有重复项，
  违规时前置校验失败并拒绝所有 goal；
- `range_limit` 生效：POI 坐标超出 `pois.yaml` 的软限位、限位格式非法、或出现未知键，
  都会让前置校验失败；
- POI 与轨道航点的位置偏差只做告警（>0.5m 时提示），不拒绝，避免影响现场微调点位；
- 前置校验失败不会永久锁死：后台按 `preflight_retry_interval_s`（默认 5s）重试，
  重试期间 goal 立即返回失败而不是卡住客户端；底盘晚启动时不必重启节点；
- 一轮走完并回到起点后，请求 `start_point` 或直接请求首个点位 `POI_01`（`is_return: false`）
  都会开始新一轮并自动复位进度；
- 刚到达起点、尚未出发时 `is_return: true` 会被拒绝（无意义的原返回）；
- POI 名会去除首尾空白，`"POI_01 "` 与 `"POI_01"` 等价；空名字直接拒绝；
- 同一时刻只接受一个 goal，并发请求会被直接拒绝；
- `cancel` 在 50ms 内生效，结果 `phase=cancelled`；
- feedback 的 `phase` 为纯枚举（`navigating` / `arrived`），导航方式在 `mode` 字段（`point` / `track`），
  `distance_remaining_m` 是当前位姿到目标的**直线距离**（不是沿轨道剩余里程，`arrived` 时为 0）；
- 结果 `is_return` 回显本次 goal 类型，编排层据此区分"前往"与"返回"，不要解析 `message`；
- `message` 会带进度快照（已开始 / 已返回 / 已完成数 / 下一站），仅作人类可读说明；
- 结果里 `[dry_run]` 前缀表示未下发任何运动指令。

### 5.2 实机运动

```bash
# 前置：完成重定位，确认 slam_status=0（curl 或底盘工具查询）
curl -s http://192.168.11.10:9090/api/core/slam/v1/localization/status

# 一键跑一轮（默认实机）
/home/nvidia/fire-inspection/run_nav.sh

# 只校验不动车
/home/nvidia/fire-inspection/run_nav.sh --dry-run
```

实机行为：
- 启动时定位异常（`slam_status != 0`）直接拒绝所有 goal，等重定位后无需重启节点（后台每 5s 重试）；
- 每段运动前读取并备份底盘限速，按 `speed.outbound_mps`（巡检）/ `speed.return_mps`（返程，
  默认均为 0.4 m/s）写入并回读确认；与当前值相同则不重复写；写入/验证失败立即回滚并终止；
  节点退出时恢复原值；
- 每段运动前开 AEB，轮询 `action_id` 状态到终态；失败/超时/取消都会先取消动作、停车再关 AEB；
- 轨道段用 `move_to_locations_poi` 沿 `outbound`（巡检）或 `return`（返程）轨道到点，
  再 `move_to(x, y, yaw, flags=0)` 原地转向；固件会忽略 `target_position.yaw`，这一步不能省；
  若到点朝向已与点位朝向误差 ≤0.10 rad（轨道末航点朝向已对齐），则跳过补转；
- 转向后回读位姿校验朝向，误差超过 0.10 rad 自动重试一次，仍超限则 goal 失败；
- 到点后校验实际位置与 POI 的偏差，超过 `arrival_tolerance_m`（默认 0.5m）则 goal 失败，
  避免固件报成功但车停在别处；
- 点位朝向以底盘地图为唯一真源（`POI_03` 为地图中的 -60° / -1.0472 rad），本地不再覆盖；
  前置校验会告警"轨道航点朝向 vs 点位朝向"不一致：想让车到点即朝向规定角度，
  需在图中把对应轨道末端航点朝向改成点位朝向（不依赖到点补转）；
- 每个点位到点后按 `_dwell_for` 停留：起点与返程 0s；`dwell_overrides` 优先
  （`POI_04` 为 3s）；未单独配置的 `transit_pois` 0s；其余巡检点位 `dwell_s`（默认 5s）；
  停留期间可被取消；
- 返程沿 `tracks.return`（POI_04 → 起点）返回，到点后同样转向到起点朝向；
  跨全图的点对点返程在底盘上会规划失败，不要使用。

> ⚠️ 实机调试必须有人守护、急停可达、轨道无障碍；首次测试建议保持 0.4 m/s 以下。

---

## 6. 配置真源

| 文件 | 内容 | 唯一真源约束 |
| --- | --- | --- |
| `fire_inspection_nav/config/pois.yaml` | 点位顺序、巡检/返程轨道名、过点点位、停留时间、分段限速、超时、应用层软限位 | 坐标与朝向以底盘 POI 服务端为准，**禁止写第二份坐标/朝向** |
| `fire_inspection_voice/config/voice.yaml` | TTS 服务名、`audio_id`、完成判定时序 | 完成判定只认 `status == "playing"` |
| `fire_inspection_vision/config/vision.yaml` | 相机、模型路径、置信度、超时 | 模型路径留空即视为 `NO_MODEL` |
| `fire_inspection_head/config/head_sweep.yaml` | 扫视序列、节奏、发布频率、手柄急停键 | 改参数需重启节点，不用重编译 |
| `fire_inspection_orchestrator/config/inspection_script.yaml` | 点位检查项、播报文案、流程开关 | 流程编排的唯一真源 |

> **模型权重不入库**（见仓库根 `.gitignore`）：手工把权重放到仓库根的 `models/`，
> 例如 `models/fire_3cls_v2.pt`，并让 `vision.yaml` 的 `model_path` 指向它。
> 权重缺失时视觉结果为 `NO_MODEL`，不会把点位判为正常。

---

## 7. 环境要求

| 项 | 要求 |
| --- | --- |
| 系统 | Ubuntu 24.04 |
| ROS | ROS 2 Jazzy（`/opt/ros/jazzy`） |
| 实机平台 | `/home/nvidia/xos`（提供 `interaction_msgs`、`ros2_bridge_msgs`），编译与运行前需 `source` |
| 底盘 | knewbots 固件 REST API，默认 `http://192.168.11.10:9090` |
| Python 依赖 | `python3-requests`（底盘 REST）、`python3-yaml`（配置解析）、`opencv` / `numpy`（视觉） |
| 视觉 | `ultralytics` / 推理运行时（模型权重路径由 `vision.yaml` 指定） |

```bash
# 若 RSP 因日志目录权限拒绝启动，先改日志目录
export ROS_LOG_DIR=$HOME/.ros/log
```

---

## 8. 常见问题

| 现象 | 原因 | 处理 |
| --- | --- | --- |
| `Package 'fire_inspection_*' not found` | 未构建或未 source | `colcon build` + `source install/setup.bash` |
| 编译报 `Failed to find package 'interaction_msgs'` | 未 source 实机平台环境 | `source /home/nvidia/xos/setup.bash` 后重建 |
| 改了 `setup.py` / `CMakeLists.txt` 没生效 | `--symlink-install` 不覆盖元数据变更 | 重新 `colcon build` |
| `build/ install/` 在编辑器里看不到 | 文件树未刷新 / 在 `src/` 下查看 | 刷新文件树；产物在 `src` 的**上一级** |
| 底盘 REST 不可达 | 网络 / IP 白名单 / 固件未运行 | `curl` 验证；检查 `host` |
| 导航一直超时 | 距离过远 / 障碍 / 速度过慢 | 提高 `timeout_s`；检查路径与限速 |
| 语音返回成功但没声音 | `cmd="stop"` 携带了文本（平台会忽略文本） | 打断用 `stop` + 空文本，新文本必须用 `cmd="append"` |

---

## 9. 开发约定

- 🔴 **接口冻结**：`fire_inspection_interfaces` 的字段顺序、语义变更必须同步所有使用方，禁止单侧改动。
- 🔴 **单一指挥者**：只有 `fire_inspection_orchestrator` 调用三个 Action，模块之间禁止互调。
- 🔴 **判定保守**：视觉结果不确定、超时、无图、无模型，一律不得落成 `NORMAL`。
- 🔴 **语音平台语义**：`stop` 只能带空文本打断；新文本用 `append`；完成判定需防抖 + 时长估算兜底。
- 🔴 **限速可回滚**：限速参数写入前备份原值，失败必须回滚，禁止猜测原值。
- 🔴 **终止优先**：中断时先落终止状态，再执行播报 / 记录，播报失败不得阻止状态落盘。
- 参数一律落 YAML，禁止硬编码在代码里。
- 新增点位只改 `pois.yaml` 与 `inspection_script.yaml`，不改代码。
