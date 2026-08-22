#!/bin/bash
# 4 个 Gazebo 端到端验收场景
# 前置：start_sim.sh 已运行，Gazebo 中场景已加载
set -e

SOURCE_CMD='source /opt/ros/humble/setup.bash
source /mnt/ssd/wheelchair_nav_ws/install/setup.bash 2>/dev/null || true
source /mnt/ssd/ladar-ai/install/setup.bash 2>/dev/null || true'

RESULT_DIR="/tmp/ladar-ai-acceptance"
mkdir -p $RESULT_DIR

check_topic_alive() {
    local topic=$1
    bash -c "$SOURCE_CMD
    timeout 3 ros2 topic echo --once $topic > /dev/null 2>&1"
    return $?
}

wait_for_topic() {
    local topic=$1
    local name=$2
    echo "[setup] 等待 $topic 可用 ..."
    for i in {1..30}; do
        if check_topic_alive $topic; then
            echo "[setup] $topic 可用"
            return 0
        fi
        sleep 1
    done
    echo "[FAIL] $topic 不可用"
    return 1
}

record_cmd_vel() {
    local duration=$1
    local out=$2
    bash -c "$SOURCE_CMD
    timeout $duration ros2 topic echo /cmd_vel > $out 2>&1" &
}

### S1：直线无障碍 ###
run_s1() {
    echo ""
    echo "=== S1：直线无障碍 ==="
    echo "请在 teleop 窗口按 i 让机器人前进"
    record_cmd_vel 20 "$RESULT_DIR/s1_cmd_vel.txt"
    wait $!
    if grep -q "linear" "$RESULT_DIR/s1_cmd_vel.txt"; then
        echo "[S1] PASS: /cmd_vel 有输出"
        return 0
    else
        echo "[S1] FAIL: /cmd_vel 无输出"
        return 1
    fi
}

### S2：单一障碍绕行 ###
run_s2() {
    echo ""
    echo "=== S2：单一障碍绕行 ==="
    echo "请把机器人朝障碍物方向开，观察 VFH 是否自动绕开"
    record_cmd_vel 30 "$RESULT_DIR/s2_cmd_vel.txt"
    wait $!
    if grep -q "angular" "$RESULT_DIR/s2_cmd_vel.txt"; then
        echo "[S2] PASS: /cmd_vel 包含角速度变化"
        return 0
    else
        echo "[S2] FAIL: 未观察到角速度变化"
        return 1
    fi
}

### S3：双障碍夹道 ###
run_s3() {
    echo ""
    echo "=== S3：双障碍夹道 ==="
    echo "请朝两个障碍物中间的通道前进"
    record_cmd_vel 30 "$RESULT_DIR/s3_cmd_vel.txt"
    wait $!
    echo "[S3] PASS（人工检查机器人是否未碰撞）"
    return 0
}

### S4：死胡同刹车 ###
run_s4() {
    echo ""
    echo "=== S4：死胡同刹车 ==="
    echo "请朝 U 形障碍物开口前进（或墙壁）"
    record_cmd_vel 15 "$RESULT_DIR/s4_cmd_vel.txt"
    wait $!
    if grep -q "0.0" "$RESULT_DIR/s4_cmd_vel.txt"; then
        echo "[S4] PASS: 检测到 0 速度（刹车）"
        return 0
    else
        echo "[S4] FAIL: 未检测到刹车"
        return 1
    fi
}

echo "================ 验收测试开始 ================"
wait_for_topic /scan_fused "融合雷达" || exit 1
wait_for_topic /cmd_vel "速度指令" || exit 1

PASS=0
FAIL=0

run_s1 && PASS=$((PASS+1)) || FAIL=$((FAIL+1))
run_s2 && PASS=$((PASS+1)) || FAIL=$((FAIL+1))
run_s3 && PASS=$((PASS+1)) || FAIL=$((FAIL+1))
run_s4 && PASS=$((PASS+1)) || FAIL=$((FAIL+1))

echo ""
echo "================ 验收结果 ================"
echo "PASS: $PASS / 4"
echo "FAIL: $FAIL / 4"
echo "结果文件: $RESULT_DIR/"
