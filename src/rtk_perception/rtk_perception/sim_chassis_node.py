"""虚拟底盘节点（仿真用）。

订阅 /cmd_vel_safe，用差速驱动模型更新虚拟 GPS + 朝向。
发布 /sim_fix + /sim_heading_imu（不覆盖真实 /fix + /heading_imu）。

仿真模式专用——与实物模式完全隔离。
"""
from __future__ import annotations

import math
import signal

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, Float64, Empty
from tf2_ros import TransformBroadcaster


class SimChassisNode(Node):
    def __init__(self):
        super().__init__("sim_chassis_node")

        self.declare_parameter("initial_lat", 24.8551)
        self.declare_parameter("initial_lon", 102.8553)
        self.declare_parameter("initial_heading_deg", 0.0)
        self.declare_parameter("update_rate_hz", 50.0)
        self.declare_parameter("cmd_timeout_sec", 1.0)
        self.declare_parameter("eeg_mode_active", False)
        self.declare_parameter("eeg_override_hold_sec", 1.5)
        self.declare_parameter("eeg_motion_linear_deadband_mps", 0.001)
        self.declare_parameter("eeg_motion_angular_deadband_rad_s", 0.001)
        # 是否发布 /heading_imu + /fix（默认 True 仿真兼容）。
        # 实物模式下 use_real_imu=true 时设 False，让 HWT906P/GPS 独占这两个 topic。
        self.declare_parameter("publish_real_topics", True)

        self._lat = float(self.get_parameter("initial_lat").value)
        self._lon = float(self.get_parameter("initial_lon").value)
        self._heading_rad = math.radians(float(self.get_parameter("initial_heading_deg").value))
        rate = float(self.get_parameter("update_rate_hz").value)
        self._dt = 1.0 / rate
        self._cmd_timeout = float(self.get_parameter("cmd_timeout_sec").value)

        self._vx = 0.0
        self._wz = 0.0
        self._last_cmd_sec = 0.0
        self._nav_vx = 0.0
        self._nav_wz = 0.0
        self._last_nav_cmd_sec = 0.0
        self._eeg_vx = 0.0
        self._eeg_wz = 0.0
        self._last_eeg_cmd_sec = 0.0
        self._last_eeg_motion_sec = 0.0
        self._eeg_override_active = False
        self._last_tick_sec = 0.0  # 实际 wall-clock dt 用于积分
        self._force_stop_active = False  # /clear_goal 触发的永久强制 zero 标志

        # 脑控多路 mux 状态：True 只表示脑控待命；非零 /cmd_vel_eeg
        # 触发临时接管，保护时间结束后自动回到后台持续运行的 Nav2。
        self.eeg_mode_active = bool(self.get_parameter("eeg_mode_active").value)
        self._last_eeg_mode_msg_time = self.get_clock().now().nanoseconds * 1e-9
        self._publish_real_topics = bool(self.get_parameter("publish_real_topics").value)

        self.create_subscription(Twist, "/cmd_vel_safe", self._cmd_cb, 10)
        # 订阅 /clear_goal:前端按"清除终点"时立即强制 zero,
        # 不依赖 controller_server cancel 反应(path_feeder cancel 可能异步延迟)
        self.create_subscription(Empty, "/clear_goal", self._on_clear_goal, 10)
        # 脑控信号源
        self.create_subscription(Twist, "/cmd_vel_eeg", self._on_cmd_vel_eeg, 1)
        self.create_subscription(Bool, "/eeg_mode_active", self._on_eeg_mode_active, 10)

        # 发布 4 个 topic：
        # /sim_fix + /sim_heading_imu → path_to_baselink（remap 订阅）
        # /fix + /heading_imu → 前端（直接订阅，仿真模式无 EC20 竞争）
        self._sim_fix_pub = self.create_publisher(NavSatFix, "/sim_fix", 10)
        self._fix_pub = self.create_publisher(NavSatFix, "/fix", 10)
        self._sim_heading_pub = self.create_publisher(Float64, "/sim_heading_imu", 10)
        self._heading_pub = self.create_publisher(Float64, "/heading_imu", 10)

        # TEB: 发布 odom→base_link TF（里程计）
        # 原点 = 启动时的 (initial_lat, initial_lon)
        self._initial_lat = self._lat
        self._initial_lon = self._lon
        self._tf_broadcaster = TransformBroadcaster(self)

        self.create_timer(self._dt, self._tick)
        # 脑控 fallback 监测：3 秒无 /eeg_mode_active 心跳 → 自动回 Nav2
        self.create_timer(1.0, self._check_eeg_mode_fallback)
        self.get_logger().info(
            f"SimChassisNode started at ({self._lat:.6f}, {self._lon:.6f}), "
            f"heading={math.degrees(self._heading_rad):.1f}°, rate={rate}Hz"
        )

    def _cmd_cb(self, msg: Twist):
        now = self.get_clock().now().nanoseconds * 1e-9
        self._nav_vx = float(msg.linear.x)
        self._nav_wz = float(msg.angular.z)
        self._last_nav_cmd_sec = now
        # 强制 zero 期间:检查 cmd_vel_safe 是否真正 zero(controller 真停了)
        # 如果 zero,解除强制(用户可设新 goal 继续走);否则继续强制 zero
        if self._force_stop_active:
            if abs(msg.linear.x) < 0.001 and abs(msg.angular.z) < 0.001:
                # controller 真的停了(cmd_vel_safe = 0),解除强制
                self._force_stop_active = False
                self.get_logger().info(
                    "cmd_vel_safe 转 zero,解除强制停止(允许新 goal)"
                )
            return  # 强制期间忽略 cmd_vel_safe
        if not self._should_use_eeg_override(now):
            self._vx = self._nav_vx
            self._wz = self._nav_wz
            self._last_cmd_sec = now

    def _on_clear_goal(self, msg: Empty):
        """前端"清除终点"时永久强制 zero,直到 cmd_vel_safe 真正变为 zero。

        策略:
        - 按按钮 → 设 _force_stop_active = True,永久强制 zero
        - controller cancel 成功 → cmd_vel 停 → safety_chain 发 zero →
          _cmd_cb 检测到 zero → 解除强制(用户可设新 goal 继续走)
        - controller cancel 失败(cmd_vel 持续非零) → 永久保持强制 zero

        这样无论 cancel 是否成功,轮椅都会真正停下。
        """
        self._force_stop_active = True
        self._vx = 0.0
        self._wz = 0.0
        self.get_logger().info(
            "clear_goal 收到,永久强制 zero 直到 cmd_vel_safe 转 zero"
        )

    def _on_cmd_vel_eeg(self, msg: Twist):
        """脑控速度回调：仅 eeg_mode_active=True 时消费。"""
        if self.eeg_mode_active:
            now = self.get_clock().now().nanoseconds * 1e-9
            self._eeg_vx = float(msg.linear.x)
            self._eeg_wz = float(msg.angular.z)
            self._last_eeg_cmd_sec = now
            if self._is_eeg_motion_cmd(msg):
                self._last_eeg_motion_sec = now
            if self._should_use_eeg_override(now):
                self._eeg_override_active = True
                self._vx = self._eeg_vx
                self._wz = self._eeg_wz
                self._last_cmd_sec = now

    def _on_eeg_mode_active(self, msg: Bool):
        """切换脑控待命状态。

        - True: 脑控待命，但不立即打断 Nav2
        - False: 清空脑控接管状态，回 Nav2
        每次都刷新心跳时间，供 fallback 监测。
        """
        prev = self.eeg_mode_active
        self.eeg_mode_active = bool(msg.data)
        self._last_eeg_mode_msg_time = self.get_clock().now().nanoseconds * 1e-9
        if prev != self.eeg_mode_active:
            self.get_logger().info(
                f"eeg_mode_active: {prev} -> {self.eeg_mode_active}"
            )
            self._eeg_vx = 0.0
            self._eeg_wz = 0.0
            self._last_eeg_cmd_sec = 0.0
            self._last_eeg_motion_sec = 0.0
            self._eeg_override_active = False
            if not self.eeg_mode_active:
                self._select_velocity(self._last_eeg_mode_msg_time)

    def _check_eeg_mode_fallback(self):
        """3 秒无 /eeg_mode_active 心跳 → fallback 到 Nav2 模式。

        脑控进程崩溃 / EEGLogger 掉线 → 心跳停止 → 自动交还控制权，
        防止用户卡在脑控模式无法接管。
        """
        if not self.eeg_mode_active:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_eeg_mode_msg_time > 3.0:
            self.get_logger().warn(
                "eeg_mode_active 3s 无更新,fallback 到 Nav2 模式"
            )
            self.eeg_mode_active = False
            self._eeg_override_active = False
            self._last_eeg_motion_sec = 0.0

    def _is_eeg_motion_cmd(self, msg: Twist) -> bool:
        linear_deadband = max(
            0.0,
            float(self.get_parameter("eeg_motion_linear_deadband_mps").value),
        )
        angular_deadband = max(
            0.0,
            float(self.get_parameter("eeg_motion_angular_deadband_rad_s").value),
        )
        return (
            abs(float(msg.linear.x)) >= linear_deadband
            or abs(float(msg.angular.z)) >= angular_deadband
        )

    def _should_use_eeg_override(self, now_sec: float) -> bool:
        if not self.eeg_mode_active:
            return False
        if self._last_eeg_motion_sec <= 0.0:
            return False
        hold_sec = max(0.0, float(self.get_parameter("eeg_override_hold_sec").value))
        return (now_sec - self._last_eeg_motion_sec) <= hold_sec

    def _select_velocity(self, now_sec: float):
        """按 force_stop > EEG 临时接管 > Nav2 选出当前仿真速度。"""
        self._eeg_override_active = False
        if self._force_stop_active:
            self._vx = 0.0
            self._wz = 0.0
            self._last_cmd_sec = now_sec
            return

        if self._should_use_eeg_override(now_sec):
            self._eeg_override_active = True
            if (
                self._last_eeg_cmd_sec > 0.0
                and (now_sec - self._last_eeg_cmd_sec) <= self._cmd_timeout
            ):
                self._vx = self._eeg_vx
                self._wz = self._eeg_wz
                self._last_cmd_sec = self._last_eeg_cmd_sec
            else:
                self._vx = 0.0
                self._wz = 0.0
                self._last_cmd_sec = now_sec
            return

        if (
            self._last_nav_cmd_sec > 0.0
            and (now_sec - self._last_nav_cmd_sec) <= self._cmd_timeout
        ):
            self._vx = self._nav_vx
            self._wz = self._nav_wz
            self._last_cmd_sec = self._last_nav_cmd_sec
        else:
            self._vx = 0.0
            self._wz = 0.0
            self._last_cmd_sec = now_sec

    def _tick(self):
        now_sec = self.get_clock().now().nanoseconds * 1e-9

        # /clear_goal 触发的永久强制 zero:无视 cmd_vel_safe
        if self._force_stop_active:
            self._eeg_override_active = False
            self._vx = 0.0
            self._wz = 0.0
            self._last_cmd_sec = now_sec
        else:
            self._select_velocity(now_sec)

        # 实际 wall-clock dt（CPU 紧张时 timer 可能延迟，用固定 dt 会让位移低估）
        if self._last_tick_sec == 0.0:
            self._last_tick_sec = now_sec
            return  # 第一帧只记录时间，不积分
        actual_dt = min(now_sec - self._last_tick_sec, 0.2)  # 上限 0.2s 防大跳
        self._last_tick_sec = now_sec

        # compass heading 顺时针为正（0=北, π/2=东）
        # ROS cmd_vel.angular.z 逆时针为正（左转正值）
        # 积分时取反：compass -= wz * dt
        self._heading_rad -= self._wz * actual_dt
        self._heading_rad %= 2 * math.pi

        # heading 是指南针角度（0=北, π/2=东, π=南）
        # cos(heading) 对应北方向（lat），sin(heading) 对应东方向（lon）
        d_north = self._vx * math.cos(self._heading_rad) * actual_dt
        d_east = self._vx * math.sin(self._heading_rad) * actual_dt

        self._lat += d_north / 111320.0
        self._lon += d_east / (111320.0 * max(math.cos(math.radians(self._initial_lat)), 0.01))

        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = "wgs84"
        fix.latitude = self._lat
        fix.longitude = self._lon
        fix.status.status = 0
        self._sim_fix_pub.publish(fix)
        if self._publish_real_topics:
            # 仿真模式：sim_chassis 给前端发 /fix（无真实 GPS 时独占）
            # 实物模式：让 EC20/DGPS 独占 /fix
            self._fix_pub.publish(fix)

        heading_deg = math.degrees(self._heading_rad) % 360.0
        hdg = Float64()
        hdg.data = heading_deg
        self._sim_heading_pub.publish(hdg)
        if self._publish_real_topics:
            # 仿真模式：sim_chassis 同时给前端发 /heading_imu（无真实 IMU 时独占）
            # 实物模式（publish_real_topics=False）：让 HWT906P 独占 /heading_imu
            self._heading_pub.publish(hdg)

        # 发布 odom→base_link TF
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_link"

        # 笛卡尔偏移（米）：以北/东为轴（与 GPS 推算使用同一 initial_lat 基准）
        dy_north_m = (self._lat - self._initial_lat) * 111320.0
        dx_east_m = (self._lon - self._initial_lon) * 111320.0 * math.cos(
            math.radians(self._initial_lat)
        )

        # 指南针角度 → ROS yaw（绕 Z 逆时针）
        # 指南针：0=北, π/2=东（顺时针为正）
        # ROS yaw：0=东, π/2=北（逆时针为正）
        # 转换：yaw_ros = π/2 - heading_compass
        # 四元数用绕 Z 轴闭式公式（避免 tf_transformations 在新 numpy 上的兼容性问题）
        yaw_ros = math.pi / 2 - self._heading_rad
        half_yaw = yaw_ros / 2.0
        qz = math.sin(half_yaw)
        qw = math.cos(half_yaw)

        tf.transform.translation.x = dx_east_m
        tf.transform.translation.y = dy_north_m
        tf.transform.translation.z = 0.0
        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = float(qz)
        tf.transform.rotation.w = float(qw)
        self._tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = SimChassisNode()
    signal.signal(signal.SIGINT, lambda *_: rclpy.shutdown())
    signal.signal(signal.SIGTERM, lambda *_: rclpy.shutdown())
    try:
        rclpy.spin(node)
    except Exception:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
