#!/bin/bash
# 键盘遥控启动脚本（避障仿真专用）
# 把 teleop 的 /cmd_vel 重定向到 /teleop_cmd_vel，让避障节点消费
set -e

source /opt/ros/humble/setup.bash
source /mnt/ssd/ladar-ai/install/setup.bash 2>/dev/null || true

echo "=== 键盘遥控启动 ==="
echo "按键说明："
echo "  i = 前进        k = 停止"
echo "  j = 左转        l = 右转"
echo "  , = 退后        o = 全后退"
echo "  u/o/m/. = 弧线前进/后退"
echo "  q/z = 加/减速"
echo ""
echo "VFH 会自动避障。按 Ctrl+C 退出。"
echo ""

exec ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r /cmd_vel:=/teleop_cmd_vel
