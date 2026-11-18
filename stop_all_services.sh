#!/bin/bash
# 停止消防巡检项目的全部服务：
#   1) 取消开机自启并停止常驻 systemd 服务；
#   2) 清理按键临时拉起的 inspection_bringup 与各服务端节点。
# 对应启动脚本：start_all_services.sh（会重新 enable 并启动）。
set -uo pipefail

# 先停按键触发，避免停其它服务期间又被拉起一轮
UNITS=(
  button-trigger.service
  fire-inspection-bridge.service
  fire-inspection-orchestrator.service
  navigate-to-server.service
  head-sweep.service
)

# 按键临时拉起 / 手工遗留的进程（按可执行路径匹配，避免误杀同名进程）
PATTERNS=(
  'ros2 launch fire_inspection_orchestrator inspection_bringup.launch.py'
  'ros2 launch fire_inspection_nav navigate_to.launch.py'
  '/fire_inspection_orchestrator/orchestrator_node'
  '/fire_inspection_orchestrator/button_trigger_node'
  '/fire_inspection_nav/navigate_to_server'
  '/fire_inspection_head/head_sweep_server'
  '/fire_inspection_vision/inspect_action_server'
  '/fire_inspection_vision/vision_scan_node'
  '/fire_inspection_voice/speak_action_server'
  '/fire_inspection_audio/audio_scan_node'
  'python -m bridge'
)

echo "=== 1. 停止服务并取消开机自启 ==="
for u in "${UNITS[@]}"; do
  state=$(systemctl is-active "$u" 2>/dev/null || true)
  printf '  %-38s %s -> disable --now\n' "$u" "${state:-unknown}"
  sudo systemctl disable --now "$u" >/dev/null 2>&1 || true
done

echo "=== 2. 清理按键临时拉起的进程 ==="
for pat in "${PATTERNS[@]}"; do
  pids=$(pgrep -f "$pat" 2>/dev/null || true)
  if [ -n "$pids" ]; then
    echo "  SIGINT  $pat  (pids: $(echo "$pids" | tr '\n' ' '))"
    sudo pkill -INT -f "$pat" 2>/dev/null || true
  fi
done
sleep 3
for pat in "${PATTERNS[@]}"; do
  pids=$(pgrep -f "$pat" 2>/dev/null || true)
  if [ -n "$pids" ]; then
    echo "  SIGKILL $pat  (pids: $(echo "$pids" | tr '\n' ' '))"
    sudo pkill -KILL -f "$pat" 2>/dev/null || true
  fi
done
sleep 1

echo "=== 3. 结果 ==="
left=0
for u in "${UNITS[@]}"; do
  enabled=$(systemctl is-enabled "$u" 2>/dev/null || true)
  active=$(systemctl is-active "$u" 2>/dev/null || true)
  printf '  %-38s enabled=%-9s active=%s\n' "$u" "${enabled:-unknown}" "${active:-unknown}"
  [ "$active" = "active" ] && left=$((left + 1))
done
for pat in "${PATTERNS[@]}"; do
  pids=$(pgrep -f "$pat" 2>/dev/null || true)
  if [ -n "$pids" ]; then
    echo "  残留进程: $(echo "$pids" | tr '\n' ' ')  ($pat)"
    left=$((left + 1))
  fi
done

if [ "$left" -eq 0 ]; then
  echo "全部已停止，且开机自启已关闭。"
else
  echo "仍有 $left 项残留，请检查上面的输出。"
  exit 1
fi
