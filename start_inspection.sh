#!/bin/bash
# 巡检一键启动脚本
# 杀旧进程 → 起三个节点 → 验证 → 开始测试

set -e

echo "=== 1. 杀旧进程 ==="
pkill -9 -f audio_aiui 2>/dev/null || true
sudo pkill -9 -f audio_aiui 2>/dev/null || true
pkill -9 -f head_sweep 2>/dev/null || true
pkill -9 -f vision_scan 2>/dev/null || true
pkill -9 -f inspect_action_server 2>/dev/null || true
pkill -9 -f orchestrator 2>/dev/null || true
pkill -9 -f speak_action 2>/dev/null || true
sleep 2
echo "杀完"

echo "=== 2. source 环境 ==="
source /opt/ros/jazzy/setup.bash
source /home/nvidia/fire-inspection/fire_inspection_ws/install/setup.bash

echo "=== 3. 起三个节点 ==="
nohup ros2 launch fire_inspection_head head_sweep.launch.py > /home/nvidia/logs/head.log 2>&1 &
sleep 2
# 必须同时起 inspect_action_server，只起 vision_scan_node 会缺
# /fire_inspection/inspect_poi 服务端，编排层等 60s 后报「视觉不可用」
nohup ros2 launch fire_inspection_vision inspect.launch.py \
    dry_run:=false start_scan:=true > /home/nvidia/logs/vision.log 2>&1 &
sleep 2

echo "=== 4. 等待视觉 Action 就绪 ==="
ready=0
for _ in $(seq 1 60); do
    if ros2 action list 2>/dev/null | grep -qx "/fire_inspection/inspect_poi"; then
        ready=1
        break
    fi
    sleep 1
done
if [[ "$ready" -ne 1 ]]; then
    echo "!! 视觉 Action 服务端 60s 未就绪，日志如下：" >&2
    tail -30 /home/nvidia/logs/vision.log >&2
    exit 1
fi
echo "OK 视觉 Action 就绪"

nohup ros2 run fire_inspection_orchestrator orchestrator_node \
    --ros-args -p dry_run:=false -p nav_enabled:=false -p speak_enabled:=false \
    > /home/nvidia/logs/orchestrator.log 2>&1 &
sleep 3

echo "=== 5. 验证 /head/cmd 发布者 ==="
ros2 topic info /head/cmd -v 2>&1 | grep -E "Publisher|Node" | head -10

echo "=== 6. 发 start 让头部开始扫 ==="
ros2 topic pub --once /fire_inspection/head_sweep_cmd std_msgs/String "{data: start}" 2>/dev/null || true

echo ""
echo "=== 7. 测试中... ==="
echo "把灭火器放到相机前，然后看日志："
echo "  tail -f /home/nvidia/logs/orchestrator.log"
echo ""
echo "预期：头部锁定不动 → 三次确认 → pause 锁住 2s → resume → 继续扫视"
echo "（语音队友在弄，当前 speak_enabled=false 跳过播报）"
tail -f /home/nvidia/logs/orchestrator.log
