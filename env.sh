#!/bin/bash
# smart-wheelchair 环境变量（所有路径的唯一定义点）
# 用法: source env.sh  （在仓库根执行）
export WS_ROOT="${WS_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
export MODELS_ROOT="${MODELS_ROOT:-$WS_ROOT/models}"
export LADAR_AI_SRC="$WS_ROOT/src"
# ROS2 环境
source /opt/ros/humble/setup.bash
source "$WS_ROOT/third_party/teb_ws_src/../teb_ws_install/setup.bash" 2>/dev/null || true
echo "[env] WS_ROOT=$WS_ROOT"
echo "[env] MODELS_ROOT=$MODELS_ROOT"
