#!/bin/bash
# 停止所有 ladar-ai 相关服务
echo "停止 ladar-ai 所有服务..."
pkill -f "camera_detect_node" 2>/dev/null
pkill -f "fusion_decision_node" 2>/dev/null
pkill -f "lidar_zone_node" 2>/dev/null
pkill -f "web_node" 2>/dev/null
pkill -f "voice_node" 2>/dev/null
pkill -f "tts_node" 2>/dev/null
pkill -f "lsn10p_launch" 2>/dev/null
pkill -f "lslidar_driver" 2>/dev/null
pkill -f "component_container" 2>/dev/null
sleep 2
# 强制清理残留
ps aux | grep -E "ladar_ai|lslidar_driver|orbbec_camera" | grep python | grep -v grep | awk '{print $2}' | xargs kill -9 2>/dev/null
echo "所有服务已停止"
