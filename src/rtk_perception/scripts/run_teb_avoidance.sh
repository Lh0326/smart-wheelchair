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

# TEB 避障完整测试栈一键启动
# 启动：主 launch（含 RViz）+ rqt_plot（速度曲线）+ cmd_vel echo（数字反馈）

set -e
RTK_DIR="$WS_ROOT"

# === source 所有依赖 ===
source /opt/ros/humble/setup.bash
source $WS_ROOT/third_party/teb_ws_install/install/setup.bash
source $WS_ROOT/third_party/lidar_n10p_install/install/setup.bash
source $WS_ROOT/third_party/ldlidar_ws/install/setup.bash
source ${RTK_DIR}/install/setup.bash

echo "=========================================="
echo "TEB 避障测试栈启动"
echo "=========================================="
echo "工作目录: ${RTK_DIR}"
echo ""

# === 终端 1：主 launch（含 RViz + teb_debug_node + sim_chassis 闭环） ===
echo "[1/3] 启动 sim_navigation_teb.launch.py..."
gnome-terminal --title="TEB Main Launch" -- bash -c \
  "source /opt/ros/humble/setup.bash && \
   source $WS_ROOT/third_party/teb_ws_install/install/setup.bash && \
   source $WS_ROOT/third_party/lidar_n10p_install/install/setup.bash && \
   source $WS_ROOT/third_party/ldlidar_ws/install/setup.bash && \
   source ${RTK_DIR}/install/setup.bash && \
   ros2 launch rtk_perception sim_navigation_teb.launch.py; \
   exec bash" &

echo "    等待 lifecycle_manager 完成 activate (8 秒)..."
sleep 8

# === 终端 2：rqt_plot 速度曲线 ===
if [ -f "${RTK_DIR}/src/rtk_perception/rviz/cmd_vel_plot.xml" ]; then
    echo "[2/3] 启动 rqt_plot 速度曲线..."
    gnome-terminal --title="rqt_plot (cmd_vel + deviation)" -- bash -c \
      "source /opt/ros/humble/setup.bash && \
       ros2 run rqt_plot rqt_plot --config-file \
         ${RTK_DIR}/src/rtk_perception/rviz/cmd_vel_plot.xml; \
       exec bash" &
else
    echo "[2/3] 跳过 rqt_plot（cmd_vel_plot.xml 不存在）"
fi

# === 终端 3：cmd_vel echo（数字反馈） ===
echo "[3/3] 启动 cmd_vel echo..."
gnome-terminal --title="/cmd_vel echo" -- bash -c \
  "source /opt/ros/humble/setup.bash && \
   source ${RTK_DIR}/install/setup.bash && \
   ros2 topic echo /cmd_vel; \
   exec bash" &

echo ""
echo "=========================================="
echo "全部启动完成"
echo "=========================================="
echo ""
echo "浏览器测试："
echo "  http://localhost:8000/index.html"
echo ""
echo "RViz 应自动打开，关注以下 display："
echo "  - NavPath (绿色全局路径)"
echo "  - LocalCostmap (紫色障碍填充)"
echo "  - CmdVelArrow (base_link 前方速度向量)"
echo "  - Trail (蓝色历史尾迹)"
echo "  - DeviationLine (偏离距离连线)"
echo "  - ModeText (HUD 文字)"
echo "  - TebFeedback (TEB 局部轨迹)"
echo ""
echo "测试流程："
echo "  1. 前端点击 5-10m 远的目标点"
echo "  2. 在轮椅模型前进路径上摆障碍物（如纸箱/椅子）"
echo "  3. 观察 CmdVelArrow 缩短 → 转向 → DeviationLine 变红 → 绕过 → 变绿"
echo "  4. rqt_plot 曲线显示避障瞬间的 angular.z 峰值"
echo ""
echo "停止：关闭 3 个 gnome-terminal 窗口，或 killall -9 ros2 ros2launch rviz2"
