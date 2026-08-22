#!/bin/bash
# 安装 LD14P udev 别名 /dev/LD14P
#
# 实测硬件信息（2026-06-23）：
#   芯片: CH9102 (vid=1a86 pid=55d4)
#   serial: 5A6C086938
#   驱动: cdc_acm（内核 ch341 不识别 55d4，由 cdc_acm 接管生成 ttyACM*）
#   设备节点: /dev/ttyACM1
#
# 注意：标准 ch341 内核驱动 alias 列表中不含 1a86:55d4，所以不会生成 ttyUSB*。
# LD14P 实测通过 USB CDC-ACM 协议被 cdc_acm 驱动识别为 /dev/ttyACM*。
# 因此本规则用 SUBSYSTEM=="tty" 而非 KERNEL=="ttyUSB*"，以同时兼容两种驱动路径。
#
# 如未来更换芯片为 CP2102（vid=10c4 pid=ea60），可启用文件末尾备选规则。
set -e

RULE_FILE=/etc/udev/rules.d/ld14p_lidar.rules
LD14P_SERIAL="${LD14P_SERIAL:-5A6C086938}"

echo "安装 LD14P udev 规则（CH9102, serial=$LD14P_SERIAL）..."

# 写入规则：匹配 vid/pid/serial，软链接到 /dev/LD14P
echo "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"1a86\", ATTRS{idProduct}==\"55d4\", ATTRS{serial}==\"$LD14P_SERIAL\", MODE:=\"0777\", GROUP:=\"dialout\", SYMLINK+=\"LD14P\"" | sudo tee "$RULE_FILE" > /dev/null

sudo udevadm control --reload-rules
sudo udevadm trigger

echo "完成。运行 ls -l /dev/LD14P 验证软链接。"
echo ""
echo "如需查找当前 LD14P 的 serial："
echo "  ls /dev/ttyACM* 后对每个设备运行："
echo "  udevadm info -q property -n /dev/ttyACM0 | grep ID_SERIAL_SHORT"

# === 备选：CP2102 芯片 ===
# 如检测到 CP2102 (vid=10c4 pid=ea60)，将上面规则替换为：
# echo "SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"10c4\", ATTRS{idProduct}==\"ea60\", ATTRS{serial}==\"<YOUR_SERIAL>\", MODE:=\"0777\", GROUP:=\"dialout\", SYMLINK+=\"LD14P\"" | sudo tee "$RULE_FILE"
