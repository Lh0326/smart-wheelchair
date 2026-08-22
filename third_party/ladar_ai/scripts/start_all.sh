#!/bin/bash
# ladar-ai 统一启动脚本
# 优化版：ASR 流式 + TTS 句级流式 + YOLO INT8 NPU 异步
# 算力分配：GPU→YOLO, CPU→KWS/ASR/TTS/Web
# 注：NPU (Intel AI Boost) 不兼容当前 YOLO 模型，不使用
set -e

source /opt/ros/humble/setup.bash
source /mnt/ssd/N10P/lidar_ros2_ws/install/setup.bash 2>/dev/null || true
source /mnt/ssd/ladar-ai/third_party/ldlidar_ws/install/setup.bash 2>/dev/null || echo "[warn] LD14P SDK not built, /scan_ld14p 不可用"
source /mnt/ssd/ladar-ai/install/setup.bash 2>/dev/null || true
export PYTHONPATH=/mnt/ssd/ladar-ai/install/ladar_ai/local/lib/python3.10/dist-packages:$PYTHONPATH

LOGDIR=/tmp/ladar-ai
mkdir -p $LOGDIR

echo "=== 启动 ladar-ai 系统 ==="

# 0. 麦克风 AGC 开启（确保唤醒词检测灵敏度）
amixer -c 1 sset 'Auto Gain Control' on > /dev/null 2>&1 || true

# 1. N10P 雷达驱动
echo "[1/7] 启动 N10P 雷达驱动..."
ros2 launch lslidar_driver lsn10p_launch.py > $LOGDIR/lidar_driver.log 2>&1 &
LIDAR_PID=$!
sleep 6

# 2. LD14P 雷达驱动（路沿检测，下方安装）
echo "[2/7] 启动 LD14P 雷达驱动..."
if [ -e /dev/LD14P ]; then
    ros2 launch ldlidar ld14p.launch.py > $LOGDIR/ld14p_driver.log 2>&1 &
    LD14P_PID=$!
    sleep 3
else
    echo "[warn] /dev/LD14P 不存在，跳过 LD14P 启动（仅影响路沿检测显示）"
fi

# 3. Orbbec 深度摄像头驱动
echo "[3/7] 启动 Orbbec Gemini 335L 摄像头..."
ros2 launch orbbec_camera gemini_330_series.launch.py enable_color:=true color_width:=640 color_height:=480 color_fps:=15 depth_width:=640 depth_height:=480 depth_fps:=15 > $LOGDIR/orbbec.log 2>&1 &
CAM_PID=$!
sleep 6

# 4. lidar_zone_node
echo "[4/7] 启动 lidar_zone_node..."
ros2 run ladar_ai lidar_zone_node > $LOGDIR/lidar_zone.log 2>&1 &
sleep 2

# 5. fusion_decision_node
echo "[5/7] 启动 fusion_decision_node..."
ros2 run ladar_ai fusion_decision_node > $LOGDIR/fusion.log 2>&1 &
sleep 2

# 6. camera_detect_node（YOLO 在 Intel iGPU 上运行）
echo "[6/7] 启动 camera_detect_node (YOLO @ NPU)..."
ros2 run ladar_ai camera_detect_node > $LOGDIR/camera_detect.log 2>&1 &
CAMDET_PID=$!
sleep 5

# 7. web_node + voice_node + tts_node
echo "[7/7] 启动 web/voice/tts 节点..."
ros2 run ladar_ai web_node > $LOGDIR/web_node.log 2>&1 &
ros2 run ladar_ai voice_node > $LOGDIR/voice_node.log 2>&1 &
ros2 run ladar_ai tts_node > $LOGDIR/tts_node.log 2>&1 &
sleep 3

echo ""
echo "=== 系统启动完成 ==="
echo "Web: http://localhost:5000"
echo "日志目录: $LOGDIR/"
echo ""
echo "运行中的进程:"
ps aux | grep -E "lidar_zone|fusion_decision|camera_detect|web_node|voice_node|tts_node|lsn10p|gemini|ldlidar" | grep python | grep -v grep | awk '{printf "  %-8s PID=%-6s CPU=%-5s MEM=%-5s %s\n", $11, $2, $3"%", $4"%", $12}'
echo ""
echo "按 Ctrl+C 停止所有服务"

# 等待
wait
