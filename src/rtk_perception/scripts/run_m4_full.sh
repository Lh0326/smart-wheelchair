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

# M4 感知与避障系统 - 完整启动（含 GPS + IMU + 路径规划 + 前端）
# 启动顺序：
#   1. m1_full.launch.py（mbtiles + 前端 + rosbridge + EC20 GNSS + IMU heading + NetworkX planner + orbbec_camera）
#   2. N10P 雷达驱动
#   3. LD14P 雷达驱动
#   4. m4_perception.launch.py（robot_state_publisher + 5 感知节点 + path_to_baselink_node + RViz）
# 用法：./run_m4_full.sh
# 停止：./stop_m4_full.sh

set -e

# 路径常量
RTK_DIR="$WS_ROOT"
N10P_SDK="$WS_ROOT/third_party/lidar_n10p_install/install"
LD14P_SDK="$WS_ROOT/third_party/ldlidar_ws/install"
LOG_DIR="/tmp/m4_perception"
mkdir -p "${LOG_DIR}"

# source 所有工作空间
source /opt/ros/humble/setup.bash
source "${N10P_SDK}/setup.bash"
source "${LD14P_SDK}/setup.bash"
source "${RTK_DIR}/install/setup.bash"

echo "=========================================="
echo "M4 感知与避障系统 - 完整启动（含 GPS 联动）"
echo "=========================================="
echo "日志目录: ${LOG_DIR}/"
echo ""

# === 硬件检查 ===
HARDWARE_OK=true
if [ ! -e /dev/lidar_n10p ]; then
    echo "[WARN] /dev/lidar_n10p 不存在！N10P 雷达未连接"
    HARDWARE_OK=false
fi
if [ ! -e /dev/LD14P ]; then
    echo "[WARN] /dev/LD14P 不存在！LD14P 雷达未连接"
    HARDWARE_OK=false
fi
if [ "${HARDWARE_OK}" = true ]; then
    echo "[OK] 雷达硬件检查通过: /dev/lidar_n10p + /dev/LD14P"
fi

# 检查 EC20 GNSS dongle（ttyUSB2/ttyUSB3）
if ls /dev/ttyUSB* 2>/dev/null | grep -q "ttyUSB"; then
    echo "[OK] EC20 GNSS dongle 检测到 (/dev/ttyUSB*)"
    # 停止 ModemManager（否则抢占 EC20 AT 端口）
    sudo -n systemctl stop ModemManager 2>/dev/null && echo "  [stopped] ModemManager" || echo "  [info] ModemManager 未停止（需要 sudo 或已停）"
else
    echo "[WARN] EC20 GNSS dongle 未检测到（/fix 不会有数据）"
fi

# === 1. 启动 m1_full.launch.py（前端 + GNSS + IMU + planner） ===
echo ""
echo "[1/4] 启动 m1_full.launch.py（前端 + EC20 GNSS + IMU + NetworkX planner）..."
ros2 launch rtk_bringup m1_full.launch.py > "${LOG_DIR}/m1_full.log" 2>&1 &
M1_PID=$!
echo "    PID=${M1_PID}, 日志=${LOG_DIR}/m1_full.log"
sleep 8  # 等待 GNSS/IMU/planner 初始化

# === 2. 启动 N10P 雷达驱动 ===
echo "[2/4] 启动 N10P 雷达驱动..."
ros2 launch lslidar_driver lsn10p_launch.py > "${LOG_DIR}/n10p_driver.log" 2>&1 &
N10P_PID=$!
echo "    PID=${N10P_PID}, 日志=${LOG_DIR}/n10p_driver.log"
sleep 5

# === 3. 启动 LD14P 雷达驱动（用 ros2 run 跳过 launch 自带的冲突 static_transform_publisher） ===
echo "[3/4] 启动 LD14P 雷达驱动..."
ros2 run ldlidar ldlidar --ros-args \
    -p product_name:=LDLiDAR_LD14P \
    -p topic_name:=scan_ld14p \
    -p port_name:=/dev/LD14P \
    -p frame_id:=ld14p_link \
    -p laser_scan_dir:=true \
    -p enable_angle_crop_func:=false \
    -p angle_crop_min:=135.0 \
    -p angle_crop_max:=225.0 \
    -p truncated_mode_:=0 \
    -r __node:=ldlidar_publisher_ld14p \
    > "${LOG_DIR}/ld14p_driver.log" 2>&1 &
LD14P_PID=$!
echo "    PID=${LD14P_PID}, 日志=${LOG_DIR}/ld14p_driver.log"
sleep 3

# === 4. 启动 m4_perception.launch.py ===
echo "[4/4] 启动 m4_perception.launch.py（含 path_to_baselink_node + RViz）..."
ros2 launch rtk_perception m4_perception.launch.py > "${LOG_DIR}/m4_perception.log" 2>&1 &
M4_PID=$!
echo "    PID=${M4_PID}, 日志=${LOG_DIR}/m4_perception.log"
sleep 5

# === 验证关键 topic ===
echo ""
echo "=== 验证关键 topic 频率（5 秒采样）==="
for t in /scan /scan_ld14p /scan_fused /fix /heading_imu /global_plan /target_heading /cmd_vel_safe; do
    printf "%-22s " "$t"
    timeout 4 ros2 topic hz "$t" 2>&1 | grep "average rate" | head -1 || echo "(无数据)"
done

echo ""
echo "=========================================="
echo "全部启动完成"
echo ""
echo "下一步操作："
echo "  1. 浏览器开 http://localhost:8000 看融合图"
echo "  2. 在融合图点击终点 → NetworkX 自动算路径"
echo "  3. 命令行设 IMU 朝向参考（用手机指南针）："
echo "     ros2 service call /set_yaw_reference rtk_msgs/srv/SetYawReference '{direction_deg: 0}'"
echo "     （direction_deg: 0=北, 90=东, 180=南, 270=西）"
echo "  4. 看 RViz 中绿色 vfh_candidate 箭头是否朝目标方向"
echo ""
echo "停止所有: ./stop_m4_full.sh"
echo "=========================================="
echo ""
echo "按 Ctrl+C 停止本脚本（不会停止后台进程）..."
wait
