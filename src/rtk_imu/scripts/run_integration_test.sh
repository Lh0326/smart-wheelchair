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

# JY901 IMU 集成测试：6 步验证
# 用法：./run_integration_test.sh

set -e
SOURCE_ENV=${SOURCE_ENV:-$WS_ROOT/source_env.sh}
source $SOURCE_ENV
source $WS_ROOT/install/setup.bash

echo "===== 步骤 1: udev 别名 ====="
if [ -e /dev/ttyIMU ]; then
    echo "✅ /dev/ttyIMU 存在 ($(readlink -f /dev/ttyIMU))"
else
    echo "❌ /dev/ttyIMU 不存在"
    echo "  检查：ls /dev/ttyUSB*"
    echo "  修复：sudo cp src/rtk_imu/udev/99-jy901.rules /etc/udev/rules.d/"
    echo "        sudo udevadm control --reload-rules && sudo udevadm trigger"
    exit 1
fi

echo ""
echo "===== 步骤 2: 串口读数测试 ====="
echo "请观察是否出现 55 51/52/53/54 包头"
timeout 3 python3 -c "
import serial
s = serial.Serial('/dev/ttyIMU', 115200, timeout=1.0)
data = s.read(50)
print('收到', len(data), '字节：', ' '.join(f'{b:02x}' for b in data[:20]))
has_55 = 0x55 in data
print('包含 0x55 包头：', has_55)
s.close()
" || echo "⚠ 串口读数超时"

echo ""
echo "===== 步骤 3: 驱动启动测试 ====="
echo "（手动启动，观察日志是否显示校准完成）"
echo "  ros2 launch rtk_imu jy901.launch.py"

echo ""
echo "===== 步骤 4-6：手动验证 ====="
echo "  步骤 4: ros2 topic echo /imu/data  （看 orientation 是否变化）"
echo "  步骤 5: ros2 run tf2_tools view_frames  （看 TF 链完整性）"
echo "  步骤 6: ros2 topic echo /heading_imu  （看 0-360° 范围）"
