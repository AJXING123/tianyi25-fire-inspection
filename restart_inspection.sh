#!/bin/bash
# 一键重启消防巡检全链路（后台运行，日志写文件）
# 先拉起各服务端并确认 Action 就绪，最后才启动编排层，避免空等 60s 后报
# 「导航/视觉 Action 服务端不可用」。

WS=/home/nvidia/fire-inspection/fire_inspection_ws
LOG_DIR=/home/nvidia/logs
NAV_PID=""
VISION_PID=""

echo "=== 1. 清场：杀掉所有冲突进程 ==="
pkill -9 -f head_sweep 2>/dev/null
pkill -9 -f vision_scan 2>/dev/null
pkill -9 -f inspect_action_server 2>/dev/null
pkill -9 -f speak_action 2>/dev/null
pkill -9 -f orchestrator_node 2>/dev/null
pkill -9 -f audio_scan 2>/dev/null
pkill -9 -f navigate_to_server 2>/dev/null
sleep 2
echo "清场完成"

echo ""
echo "=== 2. colcon build 确保最新 ==="
cd "$WS" || exit 1
source /opt/ros/jazzy/setup.bash
colcon build --packages-select fire_inspection_interfaces fire_inspection_nav fire_inspection_head fire_inspection_vision fire_inspection_voice fire_inspection_audio fire_inspection_orchestrator 2>&1 | tail -5
echo "编译完成"

echo ""
echo "=== 3. source + 后台起各服务端 ==="
source install/setup.bash
mkdir -p "$LOG_DIR"

action_ready() {
    timeout 3 ros2 action list 2>/dev/null | grep -qx "$1"
}

service_ready() {
    timeout 3 ros2 service list 2>/dev/null | grep -qx "$1"
}

wait_action() {
    local action="$1" timeout_s="${2:-60}" waited=0
    while (( waited < timeout_s )); do
        action_ready "$action" && return 0
        sleep 1
        ((waited++))
    done
    return 1
}

wait_service() {
    local service="$1" timeout_s="${2:-20}" waited=0
    while (( waited < timeout_s )); do
        service_ready "$service" && return 0
        sleep 1
        ((waited++))
    done
    return 1
}

start_nav() {
    nohup ros2 launch fire_inspection_nav navigate_to.launch.py \
        dry_run:=false > "$LOG_DIR/nav.log" 2>&1 &
    NAV_PID=$!
    echo "nav    PID=$NAV_PID"
}

start_vision() {
    # inspect_action_server + vision_scan_node 一起拉起：
    # 只起 vision_scan_node 会缺 /fire_inspection/inspect_poi 服务端
    nohup ros2 launch fire_inspection_vision inspect.launch.py \
        dry_run:=false start_scan:=true > "$LOG_DIR/vision.log" 2>&1 &
    VISION_PID=$!
    echo "vision PID=$VISION_PID"
}

# 头部
nohup ros2 launch fire_inspection_head head_sweep.launch.py > "$LOG_DIR/head.log" 2>&1 &
echo "head   PID=$!"

# 导航（之前本脚本漏起，导致编排层一直报导航服务不可用）
start_nav

# 视觉
start_vision

# 语音
nohup ros2 launch fire_inspection_voice speak.launch.py > "$LOG_DIR/voice.log" 2>&1 &
echo "voice  PID=$!"

# 听觉检测（防空警报，默认走头部麦 /lyre/audio_stream）
nohup ros2 launch fire_inspection_audio audio_scan.launch.py > "$LOG_DIR/audio.log" 2>&1 &
echo "audio  PID=$!"

echo ""
echo "=== 4. 等待关键 Action 服务就绪（不就绪则自动重启一次） ==="
if ! wait_action "/fire_inspection/navigate_to" 90; then
    echo "!! 导航服务 90s 未就绪，重启 nav 后重试"
    kill "$NAV_PID" 2>/dev/null
    sleep 2
    start_nav
    if ! wait_action "/fire_inspection/navigate_to" 90; then
        echo "!! 导航 Action 服务端仍不可用，日志如下：" >&2
        tail -30 "$LOG_DIR/nav.log" >&2
        exit 1
    fi
fi
echo "OK 导航 Action 就绪"

if ! wait_action "/fire_inspection/inspect_poi" 60; then
    echo "!! 视觉服务 60s 未就绪，重启 vision 后重试"
    kill "$VISION_PID" 2>/dev/null
    sleep 2
    start_vision
    if ! wait_action "/fire_inspection/inspect_poi" 60; then
        echo "!! 视觉 Action 服务端仍不可用，日志如下：" >&2
        tail -30 "$LOG_DIR/vision.log" >&2
        exit 1
    fi
fi
echo "OK 视觉 Action 就绪"

if ! wait_action "/fire_inspection/approach_target" 10; then
    echo "!! 警告：接近 Action 不可用，吸烟介入将只停车播报、不靠近人员"
fi
if ! wait_action "/fire_inspection/speak" 10; then
    echo "!! 警告：语音 Action 不可用，本轮巡检不会播报"
fi

echo ""
echo "=== 5. 启动编排层（单实例锁会拒绝重复启动） ==="
nohup ros2 run fire_inspection_orchestrator orchestrator_node \
    --ros-args -p dry_run:=false > "$LOG_DIR/orchestrator.log" 2>&1 &
echo "orch   PID=$!"

if ! wait_service "/fire_inspection/control" 20; then
    echo "!! 编排层控制服务未就绪，日志如下：" >&2
    tail -30 "$LOG_DIR/orchestrator.log" >&2
    exit 1
fi
echo "OK 编排层就绪"

echo ""
echo "=== 6. 验证状态 ==="
echo "-- 节点列表 --"
ros2 node list 2>&1 | grep -E "fire_head|vision_scan|orchestrator|fire_speak|fire_navigate_to|fire_inspect_poi"
echo ""
echo "-- Action 服务器 --"
ros2 action list -t 2>&1 | grep -E "fire_inspection"
echo ""
echo "-- 起头开始扫 --"
ros2 topic pub --once /fire_inspection/head_sweep_cmd std_msgs/String "{data: start}" 2>&1

echo ""
echo "=== 日志实时看 ==="
echo "tail -f $LOG_DIR/orchestrator.log   # 编排层（最重要）"
echo "tail -f $LOG_DIR/nav.log            # 导航"
echo "tail -f $LOG_DIR/vision.log         # 视觉"
echo "tail -f $LOG_DIR/head.log           # 头部"
echo "tail -f $LOG_DIR/voice.log          # 语音"
echo "tail -f $LOG_DIR/audio.log          # 听觉（防空警报命中）"
