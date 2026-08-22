#!/bin/bash
# EC20 GPS 一站式诊断脚本
# 用法：./scripts/diagnose_gps.sh
# 输出：USB 拓扑 / udev 符号链接 / AT 响应 / gnssconfig / QGPS 状态 / NMEA 摘要
# 不需要 ROS 环境，仅依赖 pyserial + sudo
#
# 排错顺序（按本脚本输出顺序）：
#   1. USB 设备识别（2c7c:0125）
#   2. udev 符号链接（ttyUSB_AT / ttyUSB_NMEA 是否指向 EC20 iface）
#   3. AT 端口响应（OK）
#   4. gnssconfig=1（不是 5）
#   5. QGPS=1（GNSS 引擎已开）
#   6. NMEA GSV 真实跟踪 vs 占位 SNR（关键诊断）
#
# 详见 docs/gps-troubleshooting.md

set -u
SUDO="sudo -S"

# 需要 sudo 的操作请手动输入密码
SUDO_PW=""
if [ -z "${SUDO_PW:-}" ]; then
    echo "需要 sudo 密码（输入后回车）："
    SUDO_PIPED=""
else
    SUDO_PIPED="echo '$SUDO_PW' | "
fi
# 用 bash -c 包装，让 echo | sudo -S 能在 eval 中正常工作
SUDO="${SUDO_PIPED}sudo -S"

AT_PORT="/dev/ttyUSB_AT"
NMEA_PORT="/dev/ttyUSB_NMEA"

echo "================================================"
echo "  EC20 GPS 诊断 - $(date '+%Y-%m-%d %H:%M:%S')"
echo "================================================"

echo ""
echo "【1】USB 设备识别"
echo "------------------------------------------------"
lsusb | grep "2c7c" && echo "  ✅ EC20 已识别" || { echo "  ❌ EC20 未识别（检查 USB 连接）"; exit 1; }

echo ""
echo "【2】udev 符号链接"
echo "------------------------------------------------"
ls -la /dev/ttyUSB_AT /dev/ttyUSB_NMEA /dev/ttyUSB_GNSS /dev/ttyUSB_MODEM 2>&1 | awk '{print "  " $0}'

echo ""
echo "【3】AT 端口响应 ($AT_PORT)"
echo "------------------------------------------------"
AT_RESP=$(eval "$SUDO timeout 3 python3 -c \"
import serial, time
s = serial.Serial('$AT_PORT', 115200, timeout=0.3)
s.reset_input_buffer()
s.write(b'AT\r\n')
time.sleep(0.5)
print(s.read(256).decode(errors='replace'))
s.close()
\"" 2>/dev/null | grep -v "^\[sudo\]")
if echo "$AT_RESP" | grep -q "OK"; then
    echo "  ✅ AT 响应正常 (OK)"
else
    echo "  ❌ AT 无响应"
    echo "  原因可能：① fwudev/ModemManager 抢占 ② AT 端口符号链接错"
    echo "  排查：sudo systemctl stop fwupd ModemManager"
    exit 1
fi

echo ""
echo "【4】gnssconfig 配置 (期望=1, GPS-only)"
echo "------------------------------------------------"
CFG=$(eval "$SUDO timeout 3 python3 -c \"
import serial, time
s = serial.Serial('$AT_PORT', 115200, timeout=0.3)
s.reset_input_buffer()
s.write(b'AT+QGPSCFG=\\\"gnssconfig\\\"\r\n')
time.sleep(0.5)
print(s.read(256).decode(errors='replace'))
s.close()
\"" 2>/dev/null | grep -v "^\[sudo\]")
CFG_VAL=$(echo "$CFG" | grep -oP 'gnssconfig\",\K[0-9]+' || echo "?")
if [ "$CFG_VAL" = "1" ]; then
    echo "  ✅ gnssconfig=1（正确，GPS-only）"
else
    echo "  ❌ gnssconfig=$CFG_VAL（应为 1）"
    echo "  ⚠️  修复：AT+QGPSCFG=\"gnssconfig\",1"
    echo "  根因：固件 EC20CEFILGR06A09M1G 上 gnssconfig=5 会触发 SNR 占位 34 bug"
fi

echo ""
echo "【5】QGPS 状态 (期望=1, GNSS 引擎已开)"
echo "------------------------------------------------"
QGPS=$(eval "$SUDO timeout 3 python3 -c \"
import serial, time
s = serial.Serial('$AT_PORT', 115200, timeout=0.3)
s.reset_input_buffer()
s.write(b'AT+QGPS?\r\n')
time.sleep(0.5)
print(s.read(256).decode(errors='replace'))
s.close()
\"" 2>/dev/null | grep -v "^\[sudo\]")
QGPS_VAL=$(echo "$QGPS" | grep -oP 'QGPS: \K[0-9]+' || echo "?")
if [ "$QGPS_VAL" = "1" ]; then
    echo "  ✅ QGPS=1（引擎已开）"
else
    echo "  ❌ QGPS=$QGPS_VAL（引擎未开）"
    echo "  ⚠️  修复：AT+QGPS=1"
fi

echo ""
echo "【6】NMEA 卫星跟踪摘要（关键诊断，采样 30 秒）"
echo "------------------------------------------------"
# 临时停止 ec20_gnss 节点释放 NMEA 端口（节点会自动重启或用户手动重启）
EC20_PIDS=$(pgrep -f ec20_gnss | tr '\n' ' ' | xargs)
if [ -n "$EC20_PIDS" ]; then
    echo "  发现 ec20_gnss 节点 PID=[$EC20_PIDS]，临时停止以释放 NMEA 端口"
    eval "$SUDO kill $EC20_PIDS" 2>/dev/null
    sleep 1
fi
echo "  监听 $NMEA_PORT 30 秒（区分真实跟踪 vs 占位 SNR）..."
echo ""
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
eval "$SUDO python3 $SCRIPT_DIR/watch_nmea.py 30 $NMEA_PORT" 2>&1 | grep -v "^\[sudo\]"

echo ""
echo "================================================"
echo "  诊断完成。详细排错流程见 docs/gps-troubleshooting.md"
echo "================================================"
