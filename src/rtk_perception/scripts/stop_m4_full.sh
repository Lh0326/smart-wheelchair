#!/bin/bash
# 停止 M4 感知与避障系统所有进程
echo "停止所有 M4 进程..."
pkill -f "ros2 launch lslidar_driver" 2>/dev/null && echo "  [stopped] N10P driver" || echo "  [skip] N10P driver 未运行"
pkill -f "ros2 launch ldlidar" 2>/dev/null && echo "  [stopped] LD14P driver" || echo "  [skip] LD14P driver 未运行"
pkill -f "ros2 launch rtk_perception" 2>/dev/null && echo "  [stopped] m4_perception" || echo "  [skip] m4_perception 未运行"
pkill -f "rviz2" 2>/dev/null && echo "  [stopped] rviz2" || echo "  [skip] rviz2 未运行"
pkill -f "robot_state_publisher" 2>/dev/null && echo "  [stopped] robot_state_publisher" || true
pkill -f "lslidar_driver_node" 2>/dev/null && echo "  [stopped] lslidar_driver_node" || true
pkill -f "ldlidar_publisher_ld14p" 2>/dev/null && echo "  [stopped] ldlidar_publisher" || true
pkill -f "fusion_scan_node\|curb_detector_node\|target_heading_node\|vfh_avoidance_node\|safety_chain_node" 2>/dev/null && echo "  [stopped] 5 perception nodes" || true
echo ""
echo "=== 剩余 ros2 进程 ==="
pgrep -af "ros2|rviz" | grep -v "ros2cli.daemon\|grep" || echo "(无)"
