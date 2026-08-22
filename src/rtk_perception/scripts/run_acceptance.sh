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

# M4 A 阶段验证测试脚本
# 用法：./run_acceptance.sh [scenario_name]
# scenario_name: s1_straight / s2_obstacle / s3_curve / s4_curb / all

set -e

SCENARIO=${1:-all}
BAG_DIR="$WS_ROOT/log/m4_bags"
RTK_DIR="$WS_ROOT"

source /opt/ros/humble/setup.bash
source "${RTK_DIR}/install/setup.bash"

mkdir -p "${BAG_DIR}"

# 公共 topic 列表
TOPICS="/scan /scan_ld14p /scan_fused /cmd_vel /cmd_vel_safe \
/curb_left_marker /curb_right_marker /curb_polygon \
/vfh_histogram /vfh_candidate /target_heading"

record_scenario() {
    local name=$1
    local out="${BAG_DIR}/${name}"
    echo "=========================================="
    echo "录制场景：${name}"
    echo "输出：${out}"
    echo "=========================================="
    echo "请在另一个终端启动 m4_perception.launch.py，"
    echo "然后在 RViz 设置 2D Goal Pose，推轮椅前进。"
    echo "录制 30 秒后自动停止。"
    echo ""
    read -p "按回车开始录制..." dummy

    timeout 30 ros2 bag record -o "${out}" ${TOPICS}
    echo "录制完成：${out}"
    echo ""
    ros2 bag info "${out}" | head -20
}

case "${SCENARIO}" in
    s1_straight|s2_obstacle|s3_curve|s4_curb)
        record_scenario "${SCENARIO}"
        ;;
    all)
        record_scenario "s1_straight"
        record_scenario "s2_obstacle"
        record_scenario "s3_curve"
        record_scenario "s4_curb"
        ;;
    *)
        echo "未知场景：${SCENARIO}"
        echo "用法：$0 [s1_straight|s2_obstacle|s3_curve|s4_curb|all]"
        exit 1
        ;;
esac

echo ""
echo "=========================================="
echo "所有场景录制完成"
echo "rosbag 路径：${BAG_DIR}"
echo "请填写 docs/m4-acceptance.md 报告"
echo "=========================================="
