#!/usr/bin/env bash
# 消防巡检导航一键联调脚本
#
# 自动完成：source 环境 → 启动导航服务端 → 按顺序下发一轮 goal → 关闭服务端。
# 默认实机运动：起点 → POI_01 → POI_02 → POI_03（停 5s）→ POI_04（停 3s）
# → 沿返程轨道回起点。
#
# 用法：
#   ./run_nav.sh                      # 实机运动，使用 pois.yaml 默认配置
#   ./run_nav.sh --dry-run            # 只校验不下发运动
#   ./run_nav.sh config.yaml          # 指定配置文件
#   DRY_RUN=true ./run_nav.sh         # 等价于 --dry-run
#
# 实机前确认：车已停在起点 start_point（自动重定位的初始位姿取自该点）、
# 轨道无遮挡、有人守护、急停可达。定位异常（slam_status 1/2/3/5）时，
# 导航服务端会按 pois.yaml 的 relocalize 配置自动重定位一次，失败则拒绝导航。

set -o pipefail

WS="${WS:-/home/nvidia/fire-inspection/fire_inspection_ws}"
CONFIG=""
DRY_RUN="${DRY_RUN:-false}"
LOG="/tmp/fire_nav_$(date +%Y%m%d_%H%M%S).log"
SERVER_PID=""

for arg in "$@"; do
  case "$arg" in
    --dry-run|-n) DRY_RUN=true ;;
    -h|--help)
      sed -n '2,13p' "$0"
      exit 0
      ;;
    *)
      if [[ -n "$CONFIG" ]]; then
        echo "!! 只能指定一个配置文件" >&2
        exit 1
      fi
      CONFIG="$arg"
      ;;
  esac
done

source /opt/ros/jazzy/setup.bash
source /home/nvidia/xos/setup.bash
source "$WS/install/setup.bash"

# Trae 终端会把自己的 libstdc++ 注入 LD_LIBRARY_PATH，遮蔽系统库导致 rclpy 导入失败
export LD_LIBRARY_PATH="$(
  printf '%s' "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v 'trae-cn-server' | paste -sd: -
)"

cleanup() {
  if [[ -n "$SERVER_PID" ]] && kill -0 "$SERVER_PID" 2>/dev/null; then
    echo "==> 关闭导航服务端"
    kill -INT "$SERVER_PID" 2>/dev/null
    # 留足时间：节点要停车、回滚限速（最多 5s 回读确认）后才退出
    for _ in {1..60}; do
      kill -0 "$SERVER_PID" 2>/dev/null || break
      sleep 0.2
    done
    kill -TERM "$SERVER_PID" 2>/dev/null
  fi
}
trap cleanup EXIT
trap 'exit 130' INT TERM

# 已存在服务端时直接退出，避免两个 Action 服务端争抢 goal
if ros2 action list 2>/dev/null | grep -qx "/fire_inspection/navigate_to"; then
  echo "!! 已存在 /fire_inspection/navigate_to 服务端，请先关闭后再运行" >&2
  exit 1
fi

if [[ "$DRY_RUN" != "true" ]]; then
  echo "!! 即将驱动底盘沿轨道运动，请确认："
  echo "   1) 车已停在起点 start_point（自动重定位的初始位姿取自该点）"
  echo "   2) 轨道与点位周围无人、无遮挡"
  echo "   3) 有人守护，急停可达"
  echo "   3 秒后开始，Ctrl-C 可取消"
  for i in 3 2 1; do
    printf '   %s...\n' "$i"
    sleep 1
  done
fi

node_args=(-p "dry_run:=$DRY_RUN")
[[ -n "$CONFIG" ]] && node_args+=(-p "config_file:=$CONFIG")

# 直接执行安装后的节点，不经 ros2 run / launch 包装，脚本退出时可可靠回收
NODE_BIN="$WS/install/fire_inspection_nav/lib/fire_inspection_nav/navigate_to_server"
echo "==> 启动导航服务端（dry_run=$DRY_RUN），日志：$LOG"
if [[ "$DRY_RUN" != "true" ]]; then
  echo "    定位异常时会先自动重定位（最长约 60s），等 Action 服务端就绪期间请勿断电"
fi
"$NODE_BIN" --ros-args "${node_args[@]}" >"$LOG" 2>&1 &
SERVER_PID=$!

ready=0
# 节点先做前置校验（含自动重定位，最长 relocalize.timeout_s=60s）才创建 Action 服务端，
# 因此等待窗口按 120s 给足，避免重定位尚未结束就误判为启动失败并杀掉进程
for _ in {1..240}; do
  if ros2 action list 2>/dev/null | grep -qx "/fire_inspection/navigate_to"; then
    ready=1
    break
  fi
  kill -0 "$SERVER_PID" 2>/dev/null || break
  sleep 0.5
done

if [[ "$ready" -ne 1 ]]; then
  echo "!! 导航服务端启动失败或超时，日志如下：" >&2
  tail -20 "$LOG" >&2
  exit 1
fi

if grep -q "前置校验失败" "$LOG"; then
  echo "!! 前置校验失败，goal 会被全部拒绝：" >&2
  grep "前置校验失败" "$LOG" | tail -1 >&2
  if grep -q "定位状态异常" "$LOG"; then
    echo "!! 自动重定位未成功：确认车在起点 start_point，定位状态（slam_status）恢复为 0 后再运行" >&2
  fi
  exit 1
fi

run_goal() {
  local poi="$1" is_return="$2" label="$3" out
  echo "==> $label"
  out=$(timeout 200 ros2 action send_goal /fire_inspection/navigate_to \
    fire_inspection_interfaces/action/NavigateTo \
    "{poi: '$poi', is_return: $is_return}" 2>&1)
  echo "$out" | grep -E "success:|phase:|message:|Goal finished" || echo "$out"
  echo "$out" | grep -q "Goal finished with status: SUCCEEDED"
}

run_goal start_point false "前往起点" || exit 1
run_goal POI_01 false "前往 POI_01" || exit 1
run_goal POI_02 false "前往 POI_02" || exit 1
run_goal POI_03 false "前往 POI_03（停留 5s）" || exit 1
run_goal POI_04 false "前往 POI_04（停留 3s，上返程轨道）" || exit 1
run_goal start_point true "沿返程轨道返回起点" || exit 1

if [[ "$DRY_RUN" == "true" ]]; then
  echo "==> 一轮导航校验全部通过（dry_run：未下发运动指令）"
else
  echo "==> 一轮实机导航完成（限速将在服务端退出时恢复）"
fi
echo "==> 服务端日志：$LOG"
