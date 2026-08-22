#!/bin/bash
# WS_ROOT: 仓库根(自动探测,可用环境变量覆盖)
WS_ROOT="${WS_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)}"
# 对src/xxx/scripts下的脚本,向上找env.sh定位仓库根
if [ ! -f "$WS_ROOT/env.sh" ]; then
  D="$WS_ROOT"
  for i in 1 2 3 4 5; do D=$(dirname "$D"); [ -f "$D/env.sh" ] && WS_ROOT="$D" && break; done
fi
export WS_ROOT
MODELS_ROOT="${MODELS_ROOT:-$WS_ROOT/models}"
export MODELS_ROOT

# ============================================================
# chassis_serial_node Layer 3 桌面验证脚本
# ============================================================
# 用途：轮椅架起来轮子悬空时，人工逐项验证下位机响应方向是否正确
# 前置条件：
#   1. 下位机串口已接入（默认 /dev/wheelchair_chassis，可改 $SERIAL_PORT）
#   2. HWT906P IMU 已启动（USE_REAL_IMU=1）
#   3. 轮椅架起，轮子悬空（防意外飞出）
#   4. 安全员就位，物理急停按钮就位
#
# 用法：
#   SERIAL_PORT=/dev/ttyUSB0 ./run_chassis_serial_test.sh
#
# 这是交互式脚本，会逐步提示用户观察轮椅响应。
# ============================================================

set -u

RTK_DIR="$WS_ROOT"
SERIAL_PORT="${SERIAL_PORT:-/dev/wheelchair_chassis}"
SOURCE_ENV="${RTK_DIR}/source_env.sh"

if [ ! -f "$SOURCE_ENV" ]; then
    echo "✗ 找不到 source_env.sh: $SOURCE_ENV"
    exit 1
fi

if [ ! -e "$SERIAL_PORT" ]; then
    echo "✗ 串口设备不存在: $SERIAL_PORT"
    echo "  临时方案：SERIAL_PORT=/dev/ttyUSB0 $0"
    exit 1
fi

echo "=========================================="
echo "  chassis_serial_node Layer 3 桌面验证"
echo "=========================================="
echo "  串口：$SERIAL_PORT"
echo "  ⚠️  确保轮椅架起，轮子悬空！"
echo "  ⚠️  安全员就位，物理急停按钮就位！"
echo ""
read -p "按回车继续，Ctrl-C 取消..." _

# 激活环境
source "$SOURCE_ENV"
source "${RTK_DIR}/install/setup.bash" 2>/dev/null || true

# 启动 chassis_serial_node（不依赖整套 sim_navigation_teb，独立验证串口）
echo ""
echo "[1/6] 启动 chassis_serial_node（独立模式）..."
ros2 run rtk_perception chassis_serial_node &
CHASSIS_PID=$!
sleep 3
if ! kill -0 $CHASSIS_PID 2>/dev/null; then
    echo "  ✗ 启动失败"
    exit 1
fi
echo "  ✓ PID: $CHASSIS_PID"

# 测试 1：零速基线
echo ""
echo "[2/6] 测试 1：发零速 Twist，轮椅应不动"
echo "  ros2 topic pub --once /cmd_vel_safe geometry_msgs/Twist '{linear: {x: 0}, angular: {z: 0}}'"
ros2 topic pub --once /cmd_vel_safe geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" 2>/dev/null
read -p "  轮椅动了吗？（应该不动）按回车继续..."

# 测试 2：低速前进
echo ""
echo "[3/6] 测试 2：发低速前进（0.3 m/s），轮椅应向前慢慢转"
ros2 topic pub --once /cmd_vel_safe geometry_msgs/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}" 2>/dev/null
read -p "  轮椅向前转了吗？按回车停（停在原地等下一题）..."
ros2 topic pub --once /cmd_vel_safe geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" 2>/dev/null

# 测试 3：左转
echo ""
echo "[4/6] 测试 3：发左转（angular.z=+0.5），轮椅应逆时针转"
ros2 topic pub --once /cmd_vel_safe geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}" 2>/dev/null
read -p "  轮椅逆时针转了吗？若顺时针转，说明 heading_sign=-1 反了，应改 launch 参数为 +1。按回车停..."
ros2 topic pub --once /cmd_vel_safe geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" 2>/dev/null

# 测试 4：右转
echo ""
echo "[5/6] 测试 4：发右转（angular.z=-0.5），轮椅应顺时针转"
ros2 topic pub --once /cmd_vel_safe geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: -0.5}}" 2>/dev/null
read -p "  轮椅顺时针转了吗？按回车停..."
ros2 topic pub --once /cmd_vel_safe geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" 2>/dev/null

# 测试 5：低速后退
echo ""
echo "[6/6] 测试 5：发低速后退（-0.3 m/s），轮椅应向后转"
ros2 topic pub --once /cmd_vel_safe geometry_msgs/Twist "{linear: {x: -0.3}, angular: {z: 0.0}}" 2>/dev/null
read -p "  轮椅向后转了吗？按回车停..."
ros2 topic pub --once /cmd_vel_safe geometry_msgs/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" 2>/dev/null

# 收尾
echo ""
echo "=========================================="
echo "  桌面验证完成"
echo "=========================================="
echo "  若所有 5 项都符合预期 → 符号方向正确，可进入 Layer 4 TEB 闭环"
echo "  若左/右转方向反了 → 改 launch 参数 heading_sign 为 +1，重跑此脚本"
echo ""
echo "  停止 chassis_serial_node..."
kill $CHASSIS_PID 2>/dev/null
# 等 _shutdown_safe 完成（连发 3 帧零速 + 关串口，~100ms）
wait $CHASSIS_PID 2>/dev/null || true
echo "  ✓ 完成"
