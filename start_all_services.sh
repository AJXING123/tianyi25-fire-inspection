#!/bin/bash
# 启动消防巡检项目的全部常驻服务，并恢复开机自启。
# 只拉起服务，不自动开始巡检；按键触发（G 左 + B）会拉起语音/视觉/听觉。
# 对应停止脚本：stop_all_services.sh。
set -uo pipefail

WS=/home/nvidia/fire-inspection/fire_inspection_ws

# 启动顺序：先服务端，再编排层，最后按键触发
UNITS=(
  navigate-to-server.service
  head-sweep.service
  fire-inspection-orchestrator.service
  fire-inspection-bridge.service
  button-trigger.service
)

wait_active() {
  local unit=$1 timeout=${2:-25} i=0
  while [ "$i" -lt $((timeout * 2)) ]; do
    if systemctl is-active --quiet "$unit" 2>/dev/null; then
      return 0
    fi
    sleep 0.5
    i=$((i + 1))
  done
  return 1
}

echo "=== 1. 恢复开机自启 ==="
for u in "${UNITS[@]}"; do
  sudo systemctl enable "$u" >/dev/null 2>&1 || true
  printf '  %-38s enabled=%s\n' "$u" "$(systemctl is-enabled "$u" 2>/dev/null || true)"
done

echo "=== 2. 启动服务 ==="
# 记录重启次数：bridge 这类启动即崩的服务会短暂显示 active，
# 只有重启次数不再增长才算真正起来了。
declare -A restarts_before=()
for u in "${UNITS[@]}"; do
  restarts_before["$u"]=$(systemctl show -p NRestarts --value "$u" 2>/dev/null || echo 0)
done
for u in "${UNITS[@]}"; do
  echo "  start $u"
  sudo systemctl start "$u" >/dev/null 2>&1 || true
  if wait_active "$u" 20; then
    echo "    -> active"
  else
    echo "    -> 未进入 active（查看 journalctl -u $u -n 50）"
  fi
done

echo "=== 3. 等待编排层控制服务 ==="
# ROS 的 setup.bash 会引用未定义变量，临时关掉 -u 再 source
set +u
if [ -f /opt/ros/jazzy/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
fi
if [ -f /home/nvidia/xos/setup.bash ]; then
  # shellcheck disable=SC1091
  source /home/nvidia/xos/setup.bash
fi
if [ -f "$WS/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$WS/install/setup.bash"
fi
set -u
ready=0
# --no-daemon：避免 ros2 daemon 状态陈旧时列不到服务
for _ in $(seq 1 20); do
  if timeout 10 ros2 service list --no-daemon 2>/dev/null \
      | grep -q '^/fire_inspection/control$'; then
    ready=1
    break
  fi
  sleep 1
done
if [ "$ready" -eq 1 ]; then
  echo "  /fire_inspection/control 已就绪"
else
  echo "  /fire_inspection/control 未出现（编排层仍在启动或异常）"
fi
state=$(timeout 3 ros2 topic echo /fire_inspection/state --once 2>/dev/null | head -1 || true)
[ -n "$state" ] && echo "  当前状态: $state"

echo "=== 4. 结果 ==="
# 等几秒，让启动失败后进入重启循环的服务（如 bridge 绑定地址错误）暴露出来
sleep 3
bad=0
for u in "${UNITS[@]}"; do
  enabled=$(systemctl is-enabled "$u" 2>/dev/null || true)
  active=$(systemctl is-active "$u" 2>/dev/null || true)
  before=${restarts_before["$u"]:-0}
  after=$(systemctl show -p NRestarts --value "$u" 2>/dev/null || echo 0)
  if [ "$after" -gt "$before" ] 2>/dev/null; then
    note="（重启 $((after - before)) 次：启动即失败）"
    bad=$((bad + 1))
  elif [ "$active" != "active" ]; then
    note="（未处于 active）"
    bad=$((bad + 1))
  else
    note=""
  fi
  printf '  %-38s enabled=%-9s active=%-11s%s\n' \
    "$u" "${enabled:-unknown}" "${active:-unknown}" "$note"
done

if [ "$bad" -eq 0 ]; then
  echo "全部服务已启动。按 G 左 + B 开始巡检。"
else
  echo "有 $bad 个服务未稳定运行；bridge 若因 --host 地址不存在而崩溃，请先改 unit 里的监听地址。"
  exit 1
fi
