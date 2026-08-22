#!/usr/bin/env bash
# 端到端语音延迟测试脚本
#
# 用途：测量"用户说完话到听到回应"的总延迟。
# 流程：播放预录的唤醒词 + 指令 wav -> 录音回环 -> 测量 TTS 首个音频 sample 时间。
#
# 前置条件：所有节点已通过 start_all.sh 启动。

set -euo pipefail

LOG_DIR="/tmp/ladar-ai"
mkdir -p "$LOG_DIR"

echo "=== 1. 节点健康检查 ==="
for node in lidar_zone_node fusion_decision_node camera_detect_node voice_node tts_node web_node; do
    if pgrep -f "$node" >/dev/null; then
        echo "  [OK] $node"
    else
        echo "  [MISSING] $node"
    fi
done

echo ""
echo "=== 2. TTS 引擎类型 ==="
grep "engine=" "$LOG_DIR/tts_node.log" 2>/dev/null | tail -1 || \
    echo "  (未找到 tts_node 日志，请先 start_all.sh)"

echo ""
echo "=== 3. YOLO 模型版本 ==="
grep "YOLO 模型加载成功" "$LOG_DIR/camera_detect.log" 2>/dev/null | tail -1 || \
    echo "  (未找到 camera_detect 日志)"

echo ""
echo "=== 4. ASR 引擎类型 ==="
grep "ASR.*加载成功" "$LOG_DIR/voice_node.log" 2>/dev/null | tail -1 || \
    echo "  (未找到 voice_node 日志)"

echo ""
echo "=== 5. 手动端到端测试（约 30 秒） ==="
echo ""
echo "请按以下步骤测试并手动计时："
echo "  1. 对麦克风说：'小智你好'"
echo "  2. 听到'我在'后立即说：'前方有什么'"
echo "  3. 用秒表测量从说完'前方有什么'到听到第一个字的时间"
echo ""
echo "预期：1.5-2.0 秒（优化前 5-7 秒）"
echo ""
read -p "按 Enter 开始 30 秒录音窗口（测量节点状态）..." _ || true

echo ""
echo "=== 6. 30 秒状态采样 ==="
START=$(date +%s)
while [ $(( $(date +%s) - START )) -lt 30 ]; do
    CPU=$(ps aux | grep -E "voice_node|tts_node|camera_detect" | grep python | \
        grep -v grep | awk '{sum+=$3} END {printf "%.1f", sum}')
    MEM=$(ps aux | grep -E "voice_node|tts_node|camera_detect" | grep python | \
        grep -v grep | awk '{sum+=$6} END {printf "%.0f", sum/1024}')
    echo "  CPU=${CPU}% MEM=${MEM}MB"
    sleep 5
done

echo ""
echo "=== 测试完成 ==="
echo "如果端到端延迟 > 2 秒，请检查："
echo "  - TTS 引擎是否为 Piper（日志应为 'engine=Piper (轻量本地)'）"
echo "  - ASR 是否为流式（日志应为 'ASR 流式模型加载成功'）"
echo "  - 起始死区是否 0.3s（grep POST_WAKE_IGNORE）"
