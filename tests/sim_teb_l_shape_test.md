# TEB L 形路径仿真验证清单

**测试目标**：验证 TEB 替换 VFH+ 后，L 形路径拐弯不再震荡。
**核心指标**：`cmd_vel.angular.z` 在拐角窗口（±2m）过零次数 ≤ 2（VFH+ 实测 5-10 次）。

## 准备

```bash
# 终端 1：source 所有依赖
source /opt/ros/humble/setup.bash
source /home/intel/SSD/teb_ws/install/setup.bash
source /mnt/ssd/N10P/lidar_ros2_ws/install/setup.bash
source /mnt/ssd/ladar-ai/third_party/ldlidar_ws/install/setup.bash
source /home/intel/SSD/rtk/install/setup.bash

# 终端 2：录制 rosbag
ros2 bag record -o /tmp/teb_test.bag /cmd_vel /nav_path /scan /scan_ld14p
```

## 启动

```bash
# 终端 1
ros2 launch rtk_perception sim_navigation_teb.launch.py
```

**预期 5 秒内**：
- [ ] 无错误（红色 ERROR 输出）
- [ ] `lifecycle_manager` 输出 "Managed nodes are active"
- [ ] `controller_server` 输出 "Activated"
- [ ] `local_costmap` 输出 "Activated"
- [ ] `ros2 node list` 包含：controller_server, local_costmap, lifecycle_manager, path_feeder_node, sim_chassis_node

## TF 验证

```bash
# 终端 3
ros2 run tf2_ros tf2_echo map base_link
```
- [ ] 输出非零 translation（轮椅静止时 ~0）
- [ ] 输出 quaternion 接近 identity（轮椅静止时）

## 雷达验证

- [ ] `ros2 topic hz /scan` 输出 10Hz 左右（N10P）
- [ ] `ros2 topic hz /scan_ld14p` 输出 10Hz 左右（LD14P）
- [ ] RViz 中 `LocalCostmap` 显示障碍点（雷达扫到的物体）

## L 形路径测试

1. 打开浏览器访问 `http://localhost:8080`（前端）
2. 在地图上找一个 L 形路径的终点（如北 30m 后东 30m）
3. 点击终点

**预期**：
- [ ] RViz 中 `NavPath` 显示完整 L 形路径（绿色）
- [ ] `ros2 topic echo /nav_path` 输出 nav_msgs/Path（frame_id=odom）
- [ ] 虚拟轮椅开始沿第一段路径前进（北上）
- [ ] `ros2 topic echo /cmd_vel` 输出非零 linear.x

## 关键验证：拐弯平滑性

**到达第一个拐角时**（约 5-10 秒后，取决于速度）：
- [ ] `cmd_vel.angular.z` 符号一致 ≥ 1.5 秒（不左右横跳）
- [ ] RViz 中 `TebFeedback` 显示局部优化轨迹（不是来回 S 形）
- [ ] 虚拟轮椅平滑转弯到 90°
- [ ] 转弯后沿第二段路径前进（东向）

## 到达终点

- [ ] 虚拟轮椅到达终点附近
- [ ] `path_to_baselink_node` 输出 "✅ 已到达终点（COMPLETED）"
- [ ] `/nav_path` 停止发布（COMPLETED 模式不发布）
- [ ] `path_feeder_node` 输出 "Path expired, canceling current goal"
- [ ] `cmd_vel` 归零

## 后处理：震荡检测

```bash
# 终端 2 停止录制（Ctrl+C）
python3 /home/intel/SSD/rtk/scripts/check_oscillation.py /tmp/teb_test.bag
```

**预期**：
- [ ] 输出 "Found N turning window(s):"
- [ ] 每个窗口的 Zero crossings ≤ 2
- [ ] Overall: PASS

## 失败诊断

如果窗口 Zero crossings > 2（震荡）：
1. 调小 `dt_ref`（0.3 → 0.2）
2. 调大 `weight_kinematics_turning_radius`（1.0 → 5.0）
3. 调大 `obstacle_cost_exponent`（4 → 6）
4. 重新跑测试

参考：https://github.com/rst-tu-dortmund/teb_local_planner

## 验收

- [ ] 所有以上 checkbox 勾选
- [ ] 震荡检测脚本输出 PASS
- [ ] 与 VFH+ 旧版对比（如有 /tmp/vfh_test.bag）过零次数显著下降
