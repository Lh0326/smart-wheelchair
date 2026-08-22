#!/bin/bash
# Ladar-AI 一键启动脚本
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

echo "=== Ladar-AI System Starter ==="

source /opt/ros/humble/setup.bash

if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    source "$WORKSPACE_DIR/install/setup.bash"
else
    echo "Workspace not built. Building..."
    cd "$WORKSPACE_DIR"
    colcon build --packages-select ladar_ai
    source install/setup.bash
fi

ros2 launch ladar_ai ladar_ai.launch.py
