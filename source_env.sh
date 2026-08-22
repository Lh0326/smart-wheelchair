#!/bin/bash
# smart-wheelchair ROS 2 环境源
# 用法: source source_env.sh
WS_ROOT="${WS_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
export WS_ROOT
MODELS_ROOT="${MODELS_ROOT:-$WS_ROOT/models}"
export MODELS_ROOT

source /opt/ros/humble/setup.bash

# FastDDS: 强制 UDPv4,避免 SHM 锁文件在节点异常退出后阻塞 DDS 发现层
export FASTRTPS_BUILTIN_TRANSPORTS=UDPv4

# 本仓库九包(构建后)
[ -f "$WS_ROOT/install/setup.bash" ] && source "$WS_ROOT/install/setup.bash" 2>/dev/null || true

# TEB 本地规划器(第三方,需先构建到 third_party/teb_ws_install)
[ -f "$WS_ROOT/third_party/teb_ws_install/setup.bash" ] && source "$WS_ROOT/third_party/teb_ws_install/setup.bash" 2>/dev/null || true

# N10P 雷达驱动(lslidar)
for pkg in lslidar_driver lslidar_msgs; do
    P="$WS_ROOT/third_party/lidar_n10p_install/$pkg"
    [ -d "$P" ] && export AMENT_PREFIX_PATH="$P:$AMENT_PREFIX_PATH"
done

echo "[env] WS_ROOT=$WS_ROOT  MODELS_ROOT=$MODELS_ROOT"
