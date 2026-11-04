#!/bin/bash
# 把 build/ 里的 Python 包元数据同步到 install 的 site-packages。
#
# 背景：工作区用 `colcon build --symlink-install` 时，ament_python 包只在
# install/<pkg>/lib/python3.12/site-packages 下留一个 *.egg-link，真正的
# *.egg-info 留在 build/<pkg>/，靠各包的 develop hook 把 build 目录加入
# PYTHONPATH 才能被 importlib.metadata 找到。systemd 服务只在启动时读一次
# 环境，若服务启动早于某次 build，setuptools 生成的入口脚本会直接以
#   PackageNotFoundError: No package metadata was found for fire-inspection-xxx
# 退出（表现为「按 B 后什么都没发生」）。
#
# 这里把 egg-info 复制进 install，让入口脚本不再依赖运行时 PYTHONPATH；
# 已存在真实安装元数据（*.egg-info / *.dist-info）的包自动跳过。
#
# 用法：tools/sync_egg_info.sh [工作区路径]
set -u

WS=${1:-/home/nvidia/fire-inspection/fire_inspection_ws}
PYVER=python3.12

shopt -s nullglob
for build_dir in "$WS"/build/fire_inspection_*; do
    pkg=$(basename "$build_dir")
    eggs=("$build_dir"/"$pkg".egg-info)
    if [ ${#eggs[@]} -eq 0 ]; then
        continue        # 未构建 / 已清理
    fi
    dest="$WS/install/$pkg/lib/$PYVER/site-packages"
    if [ ! -d "$dest" ]; then
        continue
    fi
    have=("$dest"/*.egg-info "$dest"/*.dist-info)
    if [ ${#have[@]} -gt 0 ]; then
        continue        # 已有安装元数据，不动
    fi
    cp -a "${eggs[0]}" "$dest/" && echo "[sync] $pkg -> $dest"
done
exit 0
