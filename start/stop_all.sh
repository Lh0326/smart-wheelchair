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
# 智慧轮椅 RTK 全部服务一键停止
# ============================================================
# 用法：$WS_ROOT/start/stop_all.sh
#       $WS_ROOT/start/stop_all.sh --quiet   （静默模式，供 start_all.sh 内部调用）
#
# 停止内容：
#   1. PyQt5 主窗口 + voice_node（wheelchair_app）
#   2. 底层栈 sim_navigation_teb.launch.py 全部 27 节点（含实物模式 chassis_serial /
#      jy901_driver / ec20_gnss / navsat_transform / ekf_node）
#   3. 释放端口 8000 / 8080 / 8085 / 8086 / 9091
#   4. 释放串口 /dev/wheelchair_chassis / /dev/ttyIMU / /dev/ttyUSB_{AT,GNSS,DIAG}
# ============================================================

QUIET=0
[ "${1:-}" = "--quiet" ] && QUIET=1

log() {
    [ $QUIET -eq 0 ] && echo "$@"
}

# 匹配 rtk 全栈进程的关键词（不匹配 braincontrol 等其他项目）
# 实物模式新增（2026-07-06）：chassis_serial_node / jy901_driver / ec20_gnss /
# navsat_transform_node / ekf_node —— 这些是 USE_REAL_CHASSIS=1 / USE_REAL_IMU=1 时启动的
# 关键节点。漏停任何一个 → 下次启动串口冲突 → 整栈雪崩崩溃。
PATTERNS=(
    "ros2 launch rtk_perception"
    "ros2 launch rtk_bringup"
    "rtk_perception/lib/rtk_perception/"
    "rtk_gnss/lib/rtk_gnss/"
    "rtk_planner/lib/rtk_planner/"
    "rtk_frontend/lib/rtk_frontend/"
    "rtk_map/lib/rtk_map/"
    "rtk_imu/lib/rtk_imu/"
    "wheelchair_app/lib/wheelchair_app/main"
    "wheelchair_app/lib/wheelchair_app/voice_node"
    "wheelchair_app/lib/wheelchair_app/hw_monitor_node"
    "[/]rosbridge_websocket"
    "[/]rviz2"
    "[/]controller_server"
    "[/]lifecycle_manager"
    "[/]robot_state_publisher"
    "[/]static_transform_publisher.*base_link"
    "[/]static_transform_publisher.*map"
    "[/]mbtiles_server"
    "[/]frontend_server"
    "[/]web_video_server"
    "[/]camera_http_streamer"
    "[/]depthimage_to_laserscan"
    "[/]camera_detect_node"
    "[/]fusion_scan_node"
    "[/]scan_min_range_filter"
    "[/]path_feeder_node"
    "[/]path_to_baselink_node"
    "[/]safety_chain_node"
    "[/]teb_debug_node"
    "[/]sim_chassis_node"
    "[/]chassis_serial_node"
    "[/]local_costmap"
    "[/]component_container"
    "[/]jy901_driver"
    "[/]ec20_gnss"
    "[/]navsat_transform_node"
    "[/]ekf_node"
    "[/]ekf_local_node"
    "[/]robot_localization"
    # 新加节点（语音播报 + costmap 周期清理）
    "[/]voice_announce_node"
    "[/]tts_node"
    "[/]costmap_periodic_clear"
    "[/]networkx_planner"
    "[/]networkx_planner_real"
    "[/]rosapi_node"
    "n10p_python_driver.py"
    "ldlidar_publisher_ld14p"
    "QtWebEngineProcess.*application-name=main"
    "rqt_plot"
)

log "=========================================="
log "  停止智慧轮椅 RTK 全部服务"
log "=========================================="

# 收集所有匹配 PID
PIDS=""
for pattern in "${PATTERNS[@]}"; do
    found=$(ps -eo pid,cmd | grep -E "$pattern" | grep -v "grep\|vscode-server\|stop_all" | awk '{print $1}')
    if [ -n "$found" ]; then
        PIDS="$PIDS $found"
    fi
done

# 去重
PIDS=$(echo "$PIDS" | tr ' ' '\n' | sort -u | tr '\n' ' ')

if [ -z "$(echo $PIDS | tr -d ' ')" ]; then
    log "  没有运行中的 rtk 服务"
    exit 0
fi

log ""
log "  即将停止的 PID: $PIDS"
log ""

# 先 SIGTERM 给清理机会
for pid in $PIDS; do
    kill -TERM $pid 2>/dev/null
done
sleep 3

# SIGKILL 兜底
for pid in $PIDS; do
    kill -9 $pid 2>/dev/null
done
sleep 2

# 最终检查
REMAINING=""
for pattern in "${PATTERNS[@]}"; do
    found=$(ps -eo pid,cmd | grep -E "$pattern" | grep -v "grep\|vscode-server\|stop_all" | awk '{print $1}')
    [ -n "$found" ] && REMAINING="$REMAINING $found"
done

log "  端口检查："
PORTS_OK=1
for port in 8000 8080 8085 8086 9091; do
    if ss -tlnp 2>/dev/null | grep -q ":$port "; then
        log "    ✗ 端口 $port 仍被占用"
        PORTS_OK=0
    fi
done
[ $PORTS_OK -eq 1 ] && log "    ✓ 所有端口已释放"

# 清理 FastDDS 共享内存段残留（fastrtps_*）。
# 不正常退出会留下锁文件，下次 ros2 launch 时所有节点报
# "RTPS_TRANSPORT_SHM Error: open_and_lock_file failed"，导致 lifecycle_manager
# 无法激活 controller_server → 整个 nav2 栈崩溃。
SHM_LEFT=$(ls /dev/shm/fastrtps_* 2>/dev/null | wc -l)
if [ "$SHM_LEFT" -gt 0 ]; then
    rm -f /dev/shm/fastrtps_* /dev/shm/fastrtps_*_el 2>/dev/null
    log "  ✓ 清理 $SHM_LEFT 个 fastrtps SHM 残留（避免下次启动 RTPS_SHM 锁冲突）"
fi

if [ -n "$(echo $REMAINING | tr -d ' ')" ]; then
    log ""
    log "  ⚠ 仍有残留进程: $REMAINING"
    log "  请手动 kill -9"
    exit 1
fi

log ""
log "  ✓ 全部服务已停止"
log ""
