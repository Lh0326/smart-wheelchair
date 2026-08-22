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
# 智慧轮椅 RTK 全部服务一键启动
# ============================================================
# 用法：$WS_ROOT/start/start_all.sh
#
# 启动内容：
#   1. 底层栈 sim_navigation_teb.launch.py（27 节点，含 RViz）
#   2. PyQt5 主窗口 wheelchair_app main
#   3. ASR 语音节点 voice_node
#
# 当前默认模式：全实物（USE_REAL_CHASSIS=1, USE_REAL_IMU=1）
#   - 实物：底盘 chassis_serial_node + IMU HWT906P /heading_imu + GPS EC20 /fix
#   - 雷达/相机/ASR/YOLO 不受此开关影响，始终启用
#
# 切换回仿真模式：
#   USE_REAL_CHASSIS=0 USE_REAL_IMU=0 ./start_all.sh   # 全仿真（sim_chassis 提供 fix/heading/odom）
#   USE_REAL_IMU=0 ./start_all.sh                       # 仅底盘仿真（仍要实物 IMU 时无效，因互锁）
#
# 停止：$WS_ROOT/start/stop_all.sh
# 详细文档：$WS_ROOT/start/README.md
# ============================================================

set -u

RTK_DIR="$WS_ROOT"
LOG_DIR="/tmp/rtk_logs"
SOURCE_ENV="${RTK_DIR}/source_env.sh"
DISPLAY_ENV="${DISPLAY:-:0}"

# 实物 IMU 模式：USE_REAL_IMU=1 启用 HWT906P 真实 IMU + EC20 实物 GPS + navsat_transform + EKF
# 默认 1=实物（HWT906P + EC20），0=纯仿真（sim_chassis 提供 /heading_imu + /fix）
# 用法：USE_REAL_IMU=0 $WS_ROOT/start/start_all.sh（切回仿真 IMU/GPS）
# 注意：USE_REAL_CHASSIS=1 会强制 USE_REAL_IMU=1（current_angle 必须真实）
USE_REAL_IMU="${USE_REAL_IMU:-1}"

# 实物底盘模式：USE_REAL_CHASSIS=1 启用 chassis_serial_node 替代 sim_chassis_node
# 互锁：USE_REAL_CHASSIS=1 时强制 USE_REAL_IMU=1（current_angle 必须真实）
# 默认 1=实物底盘（chassis_serial_node），0=仿真底盘（sim_chassis_node）
# 切回仿真：USE_REAL_CHASSIS=0 $WS_ROOT/start/start_all.sh
USE_REAL_CHASSIS="${USE_REAL_CHASSIS:-1}"
if [ "$USE_REAL_CHASSIS" = "1" ]; then
    USE_REAL_IMU=1  # 互锁
fi

if [ "$USE_REAL_IMU" = "1" ]; then
    IMU_LAUNCH_ARG="use_real_imu:=true"
    IMU_MODE_TAG="[实物 IMU]"
else
    IMU_LAUNCH_ARG="use_real_imu:=false"
    IMU_MODE_TAG="[仿真]"
fi

if [ "$USE_REAL_CHASSIS" = "1" ]; then
    CHASSIS_LAUNCH_ARG="use_real_chassis:=true"
    CHASSIS_MODE_TAG="[实物底盘]"
else
    CHASSIS_LAUNCH_ARG="use_real_chassis:=false"
    CHASSIS_MODE_TAG="[仿真底盘]"
fi

# 可选：CHASSIS_SERIAL_PORT 覆盖默认 /dev/wheelchair_chassis（如 udev 规则未配）
CHASSIS_SERIAL_PORT="${CHASSIS_SERIAL_PORT:-}"
CHASSIS_PORT_EXPLICIT=0
[ -n "$CHASSIS_SERIAL_PORT" ] && CHASSIS_PORT_EXPLICIT=1
if [ "$USE_REAL_CHASSIS" = "1" ]; then
    CHASSIS_BY_PATH="/dev/serial/by-path/pci-0000:00:14.0-usb-0:6.3:1.0-port0"
    if [ -z "$CHASSIS_SERIAL_PORT" ]; then
        if [ -e "$CHASSIS_BY_PATH" ]; then
            CHASSIS_SERIAL_PORT="$CHASSIS_BY_PATH"
        else
            CHASSIS_SERIAL_PORT="/dev/wheelchair_chassis"
        fi
    fi
    export CHASSIS_SERIAL_PORT
    CHASSIS_LAUNCH_ARG="$CHASSIS_LAUNCH_ARG chassis_serial_port:=$CHASSIS_SERIAL_PORT"
fi

# N10P 串口：优先 udev 别名，其次稳定 by-id，最后使用当前枚举端口。
N10P_SERIAL_PORT="${N10P_SERIAL_PORT:-}"
if [ -z "$N10P_SERIAL_PORT" ]; then
    if [ -e /dev/lidar_n10p ]; then
        N10P_SERIAL_PORT=/dev/lidar_n10p
    elif [ -e /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 ]; then
        N10P_SERIAL_PORT=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
    else
        N10P_SERIAL_PORT=/dev/lidar_n10p
    fi
fi
N10P_LAUNCH_ARG="n10p_serial_port:=$N10P_SERIAL_PORT"

# 可选：IMU_SERIAL_PORT 覆盖默认 /dev/ttyIMU。
# 当前 COM 口重排后，HWT906P 位于 by-path 3.3；udev 规则未安装时用它兜底。
IMU_SERIAL_PORT="${IMU_SERIAL_PORT:-}"
IMU_PORT_EXPLICIT=0
[ -n "$IMU_SERIAL_PORT" ] && IMU_PORT_EXPLICIT=1
if [ "$USE_REAL_IMU" = "1" ]; then
    IMU_BY_PATH="/dev/serial/by-path/pci-0000:00:14.0-usb-0:3.3:1.0-port0"
    if [ -z "$IMU_SERIAL_PORT" ]; then
        if [ -e "$IMU_BY_PATH" ]; then
            IMU_SERIAL_PORT="$IMU_BY_PATH"
        elif [ -e /dev/ttyIMU ]; then
            IMU_SERIAL_PORT="/dev/ttyIMU"
        else
            IMU_SERIAL_PORT="/dev/ttyIMU"
        fi
    fi
    export IMU_SERIAL_PORT
    IMU_LAUNCH_ARG="$IMU_LAUNCH_ARG imu_serial_port:=$IMU_SERIAL_PORT"
fi

mkdir -p "$LOG_DIR"

wait_for_command() {
    local label="$1"
    local timeout_sec="$2"
    shift 2
    local started=$SECONDS
    while (( SECONDS - started < timeout_sec )); do
        if "$@" >/dev/null 2>&1; then
            echo "  ✓ $label（$((SECONDS - started))s）"
            return 0
        fi
        sleep 1
    done
    echo "  ✗ 等待 $label 超时（${timeout_sec}s）"
    return 1
}

controller_ready() {
    set +u
    source "$SOURCE_ENV" >/dev/null 2>&1
    local source_rc=$?
    set -u
    [ "$source_rc" -eq 0 ] &&
        ros2 lifecycle get /controller_server 2>/dev/null | grep -q "active"
}

core_topics_ready() {
    set +u
    source "$SOURCE_ENV" >/dev/null 2>&1
    local source_rc=$?
    set -u
    [ "$source_rc" -eq 0 ] || return 1
    local topics
    topics=$(ros2 topic list 2>/dev/null) || return 1
    grep -qx '/cmd_vel_safe' <<<"$topics" &&
        grep -qx '/global_plan' <<<"$topics" &&
        grep -qx '/fix' <<<"$topics"
}

frontend_ready() {
    curl -fsS --max-time 1 http://127.0.0.1:8000/nav/index.html >/dev/null
}

# ---------- 前置检查 ----------
echo "=========================================="
echo "  智慧轮椅 RTK 全部服务启动"
echo "=========================================="
echo ""

echo "[1/5] 前置硬件检查"
check() {
    local label="$1"
    local cmd="$2"
    local expected="$3"
    local result
    result=$(eval "$cmd" 2>/dev/null | head -1)
    if [ -n "$result" ]; then
        echo "  ✓ $label: $result"
    else
        echo "  ✗ $label: 未检测到（$expected）"
    fi
}

tty_usb_property() {
    local port="$1"
    local property="$2"
    [ -c "$port" ] || return 1
    udevadm info -q property -n "$port" 2>/dev/null |
        sed -n "s/^${property}=//p" | head -1
}

detect_tty_usb() {
    local port="$1"
    local expected_vid="$2"
    local expected_pid="$3"
    local expected_serial="${4:-}"
    local expected_path_fragment="${5:-}"
    local vid pid serial devpath

    [ -c "$port" ] || return 1
    vid=$(tty_usb_property "$port" ID_VENDOR_ID) || return 1
    pid=$(tty_usb_property "$port" ID_MODEL_ID) || return 1
    [ "$vid" = "$expected_vid" ] && [ "$pid" = "$expected_pid" ] || return 1

    if [ -n "$expected_serial" ]; then
        serial=$(tty_usb_property "$port" ID_SERIAL_SHORT) || return 1
        [ "$serial" = "$expected_serial" ] || return 1
    fi
    if [ -n "$expected_path_fragment" ]; then
        devpath=$(udevadm info -q path -n "$port" 2>/dev/null) || return 1
        case "$devpath" in
            *"$expected_path_fragment"*) ;;
            *) return 1 ;;
        esac
    fi

    printf '%s (%s:%s%s)\n' "$port" "$vid" "$pid" \
        "${expected_serial:+ serial=$expected_serial}"
}

detect_usb_capture_device() {
    local usbid_file card_dir card_index card_name
    for usbid_file in /proc/asound/card*/usbid; do
        [ -r "$usbid_file" ] || continue
        card_dir=${usbid_file%/usbid}
        card_index=${card_dir##*/card}
        compgen -G "$card_dir/pcm*c" >/dev/null || continue
        card_name=$(sed -n "s/^ *${card_index} \[\([^]]*\).*/\1/p" /proc/asound/cards |
            head -1 | tr -d ' ')
        printf 'USB录音设备 card %s%s [%s]\n' "$card_index" \
            "${card_name:+ ($card_name)}" "$(cat "$usbid_file")"
        return 0
    done
    return 1
}

check "Orbbec Gemini 335L" \
    "lsusb | grep '2bc5:0804'" \
    "请接 USB 3.0 蓝色口"
check "N10P 雷达" \
    "detect_tty_usb '$N10P_SERIAL_PORT' 1a86 55d4 5B8E671052" \
    "请检查 udev 规则 / 设备接入"
check "LD14P 雷达" \
    "detect_tty_usb /dev/LD14P 1a86 55d4 5A6C086938" \
    "请检查 udev 规则 / 设备接入"
if [ "$USE_REAL_IMU" = "1" ]; then
    IMU_EXPECTED_PATH='3-3.3'
    [ "$IMU_PORT_EXPLICIT" = "1" ] && IMU_EXPECTED_PATH=''
    check "HWT906P IMU" \
        "detect_tty_usb '$IMU_SERIAL_PORT' 1a86 7523 '' '$IMU_EXPECTED_PATH'" \
        "请检查 HWT906P 接线 / udev 规则 / 改用 IMU_SERIAL_PORT=/dev/ttyUSBx"
fi
check "麦克风" \
    "detect_usb_capture_device" \
    "请接入带录音输入的 USB 麦克风/USB 声卡"

# ModemManager / brltty 自动停（已 disable/mask 后不会再自启，这里只做兜底）
# 这两个服务会抢占 EC20 AT 端口（/dev/ttyUSB_AT）和 CH340 串口（brltty），
# 不停掉会导致 ec20_gnss 启动失败 + jy901_driver 反复重连。
if systemctl is-active --quiet ModemManager 2>/dev/null; then
    echo "  ⚠ ModemManager 仍在运行（抢占 EC20 AT 端口），尝试自动停..."
    if sudo -n systemctl stop ModemManager 2>/dev/null; then
        echo "  ✓ ModemManager 已停（passwordless sudo）"
    else
        echo "  ✗ ModemManager 无法自动停，请手动执行："
        echo "    sudo systemctl stop ModemManager"
        echo "  或一劳永逸：sudo systemctl disable --now ModemManager && sudo systemctl mask ModemManager"
    fi
fi
if systemctl is-active --quiet brltty 2>/dev/null; then
    echo "  ⚠ brltty 仍在运行（抢占 CH340 串口），尝试自动停..."
    if sudo -n systemctl stop brltty 2>/dev/null; then
        echo "  ✓ brltty 已停（passwordless sudo）"
    else
        echo "  ✗ brltty 无法自动停，请手动执行："
        echo "    sudo systemctl stop brltty"
    fi
fi
check "ModemManager 已停" \
    "systemctl is-active ModemManager 2>/dev/null | grep inactive" \
    "sudo systemctl stop ModemManager"
check "brltty 已停" \
    "systemctl is-active brltty 2>/dev/null | grep inactive" \
    "sudo systemctl stop brltty && sudo systemctl mask brltty-udev"

if [ "$USE_REAL_CHASSIS" = "1" ]; then
    DEFAULT_CHASSIS_PORT="${CHASSIS_SERIAL_PORT:-/dev/wheelchair_chassis}"
    CHASSIS_EXPECTED_PATH='3-6.3'
    [ "$CHASSIS_PORT_EXPLICIT" = "1" ] && CHASSIS_EXPECTED_PATH=''
    check "下位机底盘串口" \
        "detect_tty_usb '$DEFAULT_CHASSIS_PORT' 1a86 7523 '' '$CHASSIS_EXPECTED_PATH'" \
        "请检查 udev 规则 / 改用 CHASSIS_SERIAL_PORT=/dev/ttyUSBx 启动"

    # 串口占用检查：chassis_serial / ttyUSB_AT 被占用 → 启动必失败
    for port in "$DEFAULT_CHASSIS_PORT" /dev/ttyUSB_AT; do
        if [ -e "$port" ]; then
            holder=$(fuser "$port" 2>/dev/null | tr -d ' ')
            if [ -n "$holder" ]; then
                echo "  ✗ $port 被进程 $holder 占用（先 $WS_ROOT/start/stop_all.sh 再启动）"
            fi
        fi
    done
fi
echo ""

# ---------- 清理残留 ----------
echo "[2/5] 清理可能残留的旧进程"
$RTK_DIR/start/stop_all.sh --quiet 2>/dev/null || true
sleep 2

# pkill 兜底：stop_all.sh 的 pattern 可能漏掉新加节点，导致串口冲突。
# 这里按"目录前缀 + 已知节点名"全量清理，与 stop_all.sh 互为补充。
# 不依赖 pattern 列表，直接按 rtk 安装路径杀，更稳。
sudo -n pkill -9 -f "$WS_ROOT/install/.*/lib/" 2>/dev/null || true
pkill -9 -f "$WS_ROOT/install/.*/lib/" 2>/dev/null || true
pkill -9 -f "ros2 launch rtk_perception" 2>/dev/null || true
pkill -9 -f "ros2 launch rtk_bringup" 2>/dev/null || true
pkill -9 -f "n10p_python_driver.py" 2>/dev/null || true
pkill -9 -f "rosbridge_websocket" 2>/dev/null || true
pkill -9 -f "QtWebEngineProcess.*application-name=main" 2>/dev/null || true
sleep 1

# 最终检查：残留进程数
REMAINING=$(ps -eo pid,cmd | grep -E "rtk_perception/lib|rtk_gnss/lib|rtk_planner/lib|rtk_frontend/lib|rtk_map/lib|rtk_imu/lib|wheelchair_app/lib|jy901_driver|chassis_serial_node|navsat_transform_node|ekf_node|ec20_gnss|n10p_python_driver|ldlidar_publisher|component_container|rosbridge_websocket|[/]rviz2" | grep -v grep | wc -l)
if [ "$REMAINING" -gt 0 ]; then
    echo "  ⚠ 仍有 $REMAINING 个残留进程，可能影响启动："
    ps -eo pid,cmd | grep -E "rtk_perception/lib|rtk_gnss/lib|rtk_planner/lib|rtk_frontend/lib|rtk_map/lib|rtk_imu/lib|wheelchair_app/lib|jy901_driver|chassis_serial_node|navsat_transform_node|ekf_node|ec20_gnss|n10p_python_driver|ldlidar_publisher|component_container|rosbridge_websocket|[/]rviz2" | grep -v grep | head -5
else
    echo "  ✓ 残留清理完成（0 残留）"
fi

# FastDDS SHM 残留兜底清理（双保险，stop_all.sh 已清过一次）。
# 残留会导致 sim_navigation_teb.launch.py 启动时所有节点报
# "RTPS_TRANSPORT_SHM Error: open_and_lock_file failed"，lifecycle_manager
# 无法激活 controller_server → [3/5] 阶段闪退。
SHM_LEFT=$(ls /dev/shm/fastrtps_* 2>/dev/null | wc -l)
if [ "$SHM_LEFT" -gt 0 ]; then
    rm -f /dev/shm/fastrtps_* /dev/shm/fastrtps_*_el 2>/dev/null
    echo "  ✓ 清理 $SHM_LEFT 个 fastrtps SHM 残留"
fi
echo ""

# ---------- 启动底层栈 ----------
echo "[3/5] 启动底层栈 sim_navigation_teb.launch.py $IMU_MODE_TAG $CHASSIS_MODE_TAG"
nohup bash -c "source '$SOURCE_ENV' && cd '$RTK_DIR' && export DISPLAY=$DISPLAY_ENV && \
    exec ros2 launch rtk_perception sim_navigation_teb.launch.py $IMU_LAUNCH_ARG $CHASSIS_LAUNCH_ARG $N10P_LAUNCH_ARG" \
    > "$LOG_DIR/sim_nav.log" 2>&1 &
SIM_PID=$!
disown
echo "  PID: $SIM_PID"
echo "  日志: $LOG_DIR/sim_nav.log"
if ! kill -0 $SIM_PID 2>/dev/null; then
    echo "  ✗ 底层栈启动失败，请查看日志"
    tail -20 "$LOG_DIR/sim_nav.log"
    exit 1
fi
echo "  等待 controller_server 和核心 Topic 就绪..."
if ! wait_for_command "controller_server active" 45 controller_ready; then
    tail -40 "$LOG_DIR/sim_nav.log"
    echo "  ⚠ 导航控制器尚未 active，继续启动前端供观察和诊断"
fi
if ! wait_for_command "核心 Topic 可用" 20 core_topics_ready; then
    tail -40 "$LOG_DIR/sim_nav.log"
    echo "  ⚠ 核心 Topic 尚未全部注册，继续启动前端供观察和诊断"
fi
wait_for_command "前端静态服务可用" 15 frontend_ready || true
echo "  ✓ 底层栈运行中且导航链路已就绪"
echo ""

# ---------- 启动 PyQt5 主窗口 ----------
echo "[4/5] 启动 PyQt5 主窗口 wheelchair_app"

# QtWebEngine 黑屏根因修复（2026-07-06 v2，前两次仅修症状都未持久）：
#   GNOME Wayland 会话（XDG_SESSION_TYPE=wayland, gdm-autologin）+ XWayland 下，
#   PyQt5 QtWebEngine 5.15.9（Chromium 87）即使设了 QTWEBENGINE_DISABLE_SANDBOX=1
#   和 --no-sandbox 也只能跳过 setuid sandbox，**zygote 进程模型仍在工作**。
#   每次渲染进程 fork 都走 zygote IPC，在 XWayland 下 IPC 通信失败：
#     "Failed to send GetTerminationStatus message to zygote"
#   → QtWebEngineProcess 崩溃/不启动 → 自主导航 + 小智陪伴 Tab 黑屏。
#   脑电控制 Tab 是纯 PyQt5 原生无 QtWebEngine，所以不受影响。
#
#   关键 flag（前两次修复缺这个，所以重启后复发）：
#     --no-zygote           彻底跳过 zygote 进程模型，渲染进程直接 fork
#   配合 flag：
#     --no-sandbox          双保险跳 sandbox
#     --disable-gpu-compositing  Intel Arc i915 在 XWayland 下合成层不稳
#
#   仅在 PyQt5 段落 export，不污染底层栈（rviz2）和 voice_node。
#   防御层：navigation_tab.py / companion_tab.py 的 renderProcessTerminated
#   信号触发自动 reload，万一崩溃也能自愈。
export QT_QPA_PLATFORM=xcb
export QTWEBENGINE_DISABLE_SANDBOX=1
export QTWEBENGINE_CHROMIUM_FLAGS="--no-sandbox --no-zygote --disable-gpu-compositing"

nohup bash -c "source '$SOURCE_ENV' && export DISPLAY=$DISPLAY_ENV && \
    exec ros2 run wheelchair_app main" \
    > "$LOG_DIR/wheelchair_main.log" 2>&1 &
QT_PID=$!
disown
echo "  PID: $QT_PID"
echo "  日志: $LOG_DIR/wheelchair_main.log"
sleep 2
if ! kill -0 $QT_PID 2>/dev/null; then
    echo "  ✗ PyQt5 启动失败"
    tail -10 "$LOG_DIR/wheelchair_main.log"
else
    echo "  ✓ PyQt5 主窗口已启动"
fi
echo ""

# ---------- 启动 ASR ----------
echo "[5/5] 启动 ASR voice_node"
nohup bash -c "source '$SOURCE_ENV' && exec ros2 run wheelchair_app voice_node" \
    > "$LOG_DIR/voice_node.log" 2>&1 &
VOICE_PID=$!
disown
echo "  PID: $VOICE_PID"
echo "  日志: $LOG_DIR/voice_node.log"
echo "  等待 voice_node ROS 节点注册..."
wait_for_command "voice_node 注册" 20 bash -c \
    "source '$SOURCE_ENV' >/dev/null 2>&1 && ros2 node list 2>/dev/null | grep -qx '/voice_node'" || true
if ! kill -0 $VOICE_PID 2>/dev/null; then
    echo "  ✗ voice_node 启动失败"
    tail -10 "$LOG_DIR/voice_node.log"
else
    echo "  ✓ voice_node 运行中"
fi
echo ""

# ---------- 完成汇总 ----------
echo "=========================================="
echo "  全部服务启动完成"
echo "=========================================="
echo ""
echo "▌ 主进程 PID"
echo "  底层栈    : $SIM_PID"
echo "  PyQt5     : $QT_PID"
echo "  voice_node: $VOICE_PID"
echo ""
echo "▌ 桌面窗口（应该都已出现）"
echo "  - 智慧轮椅 — BrainControl（PyQt5 主窗口，3 个 tab）"
echo "  - RViz（perception_teb.rviz）"
echo ""
echo "▌ HTTP 服务"
echo "  http://localhost:8000/nav/index.html        (前端 - 自主导航)"
echo "  http://localhost:8000/companion/index.html  (前端 - 小智陪伴)"
echo "  http://localhost:8080/health                (mbtiles 瓦片)"
echo "  http://localhost:8086                       (相机 JPEG 流)"
echo "  ws://localhost:9091                         (rosbridge WebSocket)"
echo ""
echo "▌ 测试入口"
echo "  1. PyQt5 → Tab 1 → 地图点击终点 → 自动导航 + RViz 避障"
echo "  2. PyQt5 → Tab 2 → 视频 + YOLO + 雷达图 + 说「小智你好」测 ASR"
echo "  3. PyQt5 → Tab 3 → 脑电控制（需 main_eeg.py 训练）"
echo ""
echo "▌ 停止服务"
echo "  $WS_ROOT/start/stop_all.sh"
echo ""
