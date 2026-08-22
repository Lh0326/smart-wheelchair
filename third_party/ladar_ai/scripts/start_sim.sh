#!/bin/bash
# 一键启动自主避障仿真
set -e

SOURCE_CMD='source /opt/ros/humble/setup.bash
source /mnt/ssd/wheelchair_nav_ws/install/setup.bash 2>/dev/null || true
source /mnt/ssd/ladar-ai/install/setup.bash 2>/dev/null || true'

echo "=== 1. 构建 ladar_ai（若已构建则很快） ==="
bash -c "$SOURCE_CMD
cd /mnt/ssd/ladar-ai && colcon build --packages-select ladar_ai --symlink-install 2>&1 | tail -3"

echo "=== 2. 启动 Gazebo + bcr_bot + 桥接（xterm 窗口） ==="
if command -v xterm >/dev/null 2>&1; then
    xterm -e bash -c "$SOURCE_CMD
ros2 launch ladar_ai sim_gazebo.launch.py
exec bash" &
    GAZEBO_PID=$!
else
    echo "[warn] xterm 未安装，改用后台启动（日志: /tmp/ladar-ai-sim/gazebo.log）"
    mkdir -p /tmp/ladar-ai-sim
    bash -c "$SOURCE_CMD
ros2 launch ladar_ai sim_gazebo.launch.py > /tmp/ladar-ai-sim/gazebo.log 2>&1" &
    GAZEBO_PID=$!
fi
sleep 10

echo "=== 3. 启动避障栈（teleop 在 xterm 窗口） ==="
if command -v xterm >/dev/null 2>&1; then
    xterm -e bash -c "$SOURCE_CMD
ros2 launch ladar_ai obstacle_avoidance.launch.py
exec bash" &
    AVOID_PID=$!
else
    bash -c "$SOURCE_CMD
ros2 launch ladar_ai obstacle_avoidance.launch.py > /tmp/ladar-ai-sim/avoidance.log 2>&1" &
    AVOID_PID=$!
fi

echo ""
echo "=== 启动完成 ==="
echo "在 teleop 窗口按键盘控制轮椅："
echo "  i=前进  ,=左转  .=右转  k=停止  o=退后"
echo "VFH 会自动绕开障碍物（VFH 安全栅栏会拦截不安全的方向）"
echo ""
echo "Ctrl+C 退出本脚本（xterm 窗口需手动关闭）"
wait
