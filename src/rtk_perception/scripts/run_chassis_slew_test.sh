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
# ChassisSlewLimiter Layer 3 桌面验证脚本（不接电机）
# ============================================================
# 用途：开发期验证限速器行为（mock 串口输出到 stdout），不依赖实物底盘
# 前置条件：
#   1. rtk 工作空间已 build（colcon build --packages-select rtk_perception）
#   2. source install/setup.bash
#
# 用法：
#   $WS_ROOT/src/rtk_perception/scripts/run_chassis_slew_test.sh
# ============================================================

set -u

RTK_DIR="$WS_ROOT"
SOURCE_ENV="${RTK_DIR}/source_env.sh"

if [ ! -f "$SOURCE_ENV" ]; then
    echo "✗ 找不到 source_env.sh: $SOURCE_ENV"
    exit 1
fi

source "$SOURCE_ENV"
source "${RTK_DIR}/install/setup.bash" 2>/dev/null || true

echo "=========================================="
echo "  ChassisSlewLimiter Layer 3 桌面验证"
echo "=========================================="
echo "  本脚本不接电机，仅启动节点用 mock 串口验证限速器行为"
echo ""

# 启动 chassis_serial_node（独立模式，USE_REAL_CHASSIS=false）
echo "[1/5] 启动 chassis_serial_node（mock 模式）..."
USE_REAL_CHASSIS=0 ros2 run rtk_perception chassis_serial_node &
CHASSIS_PID=$!
sleep 3
if ! kill -0 $CHASSIS_PID 2>/dev/null; then
    echo "  ✗ 启动失败"
    exit 1
fi
echo "  ✓ PID: $CHASSIS_PID"

# 测试 1：单次 EEG FORWARD→LEFT 切换
echo ""
echo "[2/5] 测试 1：EEG FORWARD→LEFT 单次切换（应触发 COOLDOWN）"
echo "  发布 FORWARD 帧（持续 500ms）..."
ros2 topic pub --once /cmd_vel_eeg geometry_msgs/Twist "{linear: {x: 0.5}}" &
sleep 0.5
echo "  发布 LEFT 帧..."
ros2 topic pub --once /cmd_vel_eeg geometry_msgs/Twist "{angular: {z: 0.5}}" &
sleep 1
echo "  ✓ 观察节点日志：应看到 state=COOLDOWN，direction 保持 0"

# 测试 2：连续 LEFT/RIGHT 摇头 2 秒（应触发 LONG_COOLDOWN）
echo ""
echo "[3/5] 测试 2：连续 LEFT/RIGHT 摇头 2 秒（应触发 LONG_COOLDOWN）"
for i in $(seq 1 8); do
    if [ $((i % 2)) -eq 0 ]; then
        ros2 topic pub --once /cmd_vel_eeg geometry_msgs/Twist "{angular: {z: 0.5}}" &
    else
        ros2 topic pub --once /cmd_vel_eeg geometry_msgs/Twist "{angular: {z: -0.5}}" &
    fi
    sleep 0.25
done
sleep 1
echo "  ✓ 观察节点日志：应看到 state=LONG_COOLDOWN"

# 测试 3：Nav2 cmd_vel 阶跃（应被 slew rate 平滑）
echo ""
echo "[4/5] 测试 3：Nav2 cmd_vel 阶跃 0.6 m/s（应被 slew rate 平滑）"
ros2 topic pub --once /cmd_vel_safe geometry_msgs/Twist "{linear: {x: 0.6}}" &
sleep 1
echo "  ✓ 观察节点日志：forward_speed 应递增（每帧 +20）"

# 测试 4：运行时禁用限速器
echo ""
echo "[5/5] 测试 4：运行时禁用限速器（应直通 target）"
ros2 param set /chassis_serial_node slew_limiter_enabled false
sleep 0.5
ros2 topic pub --once /cmd_vel_safe geometry_msgs/Twist "{linear: {x: 0.6}}" &
sleep 1
echo "  ✓ 观察节点日志：forward_speed 立即顶到 mode_speed（无 ramp）"

# 清理
kill $CHASSIS_PID 2>/dev/null
wait 2>/dev/null

echo ""
echo "=========================================="
echo "  桌面验证完成"
echo "=========================================="
echo "实物测试请参考 docs/chassis-slew-rate-acceptance.md"
