#!/usr/bin/env python3
"""
IMU 航向积分节点（向量投影法 + 静态 bias 校准）

核心算法：
1. 启动时采集 3 秒静态数据，计算：
   - 三轴 gyro bias（每秒漂移）
   - 重力向量（accel 平均），归一化得到垂直方向单位向量
2. 每帧：
   - gyro 减去 bias
   - 将 gyro 投影到重力方向（垂直轴），得到偏航角速度
   - 积分得到 yaw

优势：无论 IMU 怎么安装（水平/倾斜/任意朝向），都能正确测偏航。
限制：假设转动期间 IMU 倾角不变（水平转轮椅成立）。
"""
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Float64, Header

from rtk_msgs.srv import SetYawReference


class ImuHeadingNode(Node):
    def __init__(self):
        super().__init__('imu_heading')

        self.declare_parameter('calibration_time', 3.0)
        self.declare_parameter('initial_yaw', 0.0)
        self.declare_parameter('invert', False)  # 转动方向反向（如箭头反向）

        self.calibration_time = float(self.get_parameter('calibration_time').value)
        self.yaw_rad = math.radians(float(self.get_parameter('initial_yaw').value))
        self.invert = bool(self.get_parameter('invert').value)

        # Yaw 人工对齐 offset（替代磁力计）
        # 启动时为 0，用户通过 /set_yaw_reference service 设置
        self.yaw_offset_deg = 0.0

        self.bias = [0.0, 0.0, 0.0]  # gx, gy, gz bias
        self.gravity = [0.0, 0.0, -1.0]  # 默认 z 朝下
        self._last_time = None
        self._lock = threading.Lock()
        self._calibrating = True
        self._calibration_samples = []

        self.heading_pub = self.create_publisher(Float64, '/heading_imu', 10)
        # 额外发布标准 Imu 消息（带 orientation），供 robot_localization EKF 使用
        # 内容：原始 angular_velocity + linear_acceleration + 由 yaw 生成的 orientation
        self.imu_data_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.create_subscription(
            Imu,
            '/camera/gyro_accel/sample',
            self._imu_callback,
            qos_profile_sensor_data,
        )

        # Yaw 人工对齐 service
        self.create_service(SetYawReference, '/set_yaw_reference', self._handle_set_yaw)

        self.create_timer(0.01, self._publish_heading)
        # 每秒打印调试信息（转动时能看到 yaw_rate 变化）
        self.create_timer(1.0, self._debug_log)

        self._last_yaw_rate = 0.0
        self._debug_count = 0

        self.get_logger().info(
            f"启动静态校准（请保持静止 {self.calibration_time:.1f} 秒）..."
        )

    def _handle_set_yaw(self, request, response):
        """人工校准 yaw：用户告知当前实际朝向，节点记录 offset"""
        with self._lock:
            current_raw_deg = math.degrees(self.yaw_rad)

        if self._calibrating:
            response.success = False
            response.message = "IMU 尚在校准中，请稍候"
            response.offset_applied = 0.0
            self.get_logger().warn(f"校准请求被拒：IMU 仍在静态校准")
            return response

        # IMU raw_yaw 是 ROS 标准（逆时针为正）
        # 转为指南针角度（顺时针为正，北=0/东=90/南=180/西=270）
        current_compass_deg = (-current_raw_deg) % 360.0
        self.yaw_offset_deg = (request.direction_deg - current_compass_deg) % 360.0

        direction_names = {0: "北", 90: "东", 180: "南", 270: "西"}
        dir_name = direction_names.get(request.direction_deg, f"{request.direction_deg}°")

        response.success = True
        response.message = f"已校准：当前朝向为{dir_name}（offset={self.yaw_offset_deg:.1f}°）"
        response.offset_applied = self.yaw_offset_deg

        self.get_logger().info(
            f"Yaw 人工对齐：raw={current_raw_deg:.1f}° → compass={current_compass_deg:.1f}° "
            f"→ 目标={dir_name}({request.direction_deg}°)，offset={self.yaw_offset_deg:.1f}°"
        )
        return response

    def _debug_log(self):
        """每秒打印一次当前偏航角速度（用于诊断是否检测到转动）"""
        self._debug_count += 1
        rate_deg = math.degrees(self._last_yaw_rate)
        # 判断状态
        if abs(rate_deg) < 1.0:
            status = "静止"
        elif abs(rate_deg) < 10.0:
            status = "缓慢转动"
        elif abs(rate_deg) < 60.0:
            status = "正常转动"
        else:
            status = "快速转动"
        # 每 2 秒打印一次，避免日志太吵
        if self._debug_count % 2 == 0 and not self._calibrating:
            self.get_logger().info(
                f"[调试] yaw_rate={rate_deg:+6.2f}°/s ({status}) | yaw={math.degrees(self.yaw_rad):.1f}°"
            )

    def _imu_callback(self, msg):
        now_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        # 保存原始 IMU 数据用于发布 /imu/data
        self._last_raw_imu = msg

        if self._calibrating:
            self._calibration_samples.append((gx, gy, gz, ax, ay, az, now_sec))
            if len(self._calibration_samples) > 10:
                elapsed = now_sec - self._calibration_samples[0][6]
                if elapsed >= self.calibration_time:
                    self._finish_calibration()
            self._last_time = now_sec
            return

        if self._last_time is None:
            self._last_time = now_sec
            return
        dt = now_sec - self._last_time
        self._last_time = now_sec
        if dt <= 0 or dt > 0.5:
            return

        # 减去 bias
        gx_c = gx - self.bias[0]
        gy_c = gy - self.bias[1]
        gz_c = gz - self.bias[2]

        # 投影到重力方向（垂直轴），得到偏航角速度
        # yaw_rate = gyro · gravity_normalized
        g_norm = math.sqrt(sum(c * c for c in self.gravity))
        if g_norm < 0.001:
            yaw_rate = gz_c  # fallback
        else:
            gn = [c / g_norm for c in self.gravity]
            yaw_rate = gx_c * gn[0] + gy_c * gn[1] + gz_c * gn[2]

        if self.invert:
            yaw_rate = -yaw_rate

        self._last_yaw_rate = yaw_rate  # 供调试日志使用

        with self._lock:
            self.yaw_rad += yaw_rate * dt
            while self.yaw_rad < 0:
                self.yaw_rad += 2 * math.pi
            while self.yaw_rad >= 2 * math.pi:
                self.yaw_rad -= 2 * math.pi

    def _finish_calibration(self):
        n = len(self._calibration_samples)
        sgx = sum(s[0] for s in self._calibration_samples) / n
        sgy = sum(s[1] for s in self._calibration_samples) / n
        sgz = sum(s[2] for s in self._calibration_samples) / n
        sax = sum(s[3] for s in self._calibration_samples) / n
        say = sum(s[4] for s in self._calibration_samples) / n
        saz = sum(s[5] for s in self._calibration_samples) / n

        self.bias = [sgx, sgy, sgz]
        self.gravity = [sax, say, saz]

        g_norm = math.sqrt(sax * sax + say * say + saz * saz)
        self._calibrating = False
        self._last_time = None

        self.get_logger().info(
            f"✅ 校准完成（{n} 采样）:\n"
            f"  gyro bias: x={sgx:+.4f}, y={sgy:+.4f}, z={sgz:+.4f} rad/s\n"
            f"  重力向量: [{sax:+.3f}, {say:+.3f}, {saz:+.3f}] m/s² (|g|={g_norm:.2f})\n"
            f"  → 垂直轴单位向量: [{sax/g_norm:+.3f}, {say/g_norm:+.3f}, {saz/g_norm:+.3f}]\n"
            f"现在水平转动相机/轮椅，方向箭头会跟随（每完整一圈 = 360°）"
        )

    def _publish_heading(self):
        with self._lock:
            raw_deg = math.degrees(self.yaw_rad)
        # IMU raw_yaw 是 ROS 标准（逆时针为正）
        # 转为指南针角度（顺时针为正），再应用人工对齐 offset
        compass_deg = (-raw_deg) % 360.0
        yaw_deg = (compass_deg + self.yaw_offset_deg) % 360.0
        msg = Float64()
        msg.data = yaw_deg
        self.heading_pub.publish(msg)

        # 同时发布标准 Imu 到 /imu/data（供 EKF 使用）
        # 仅在校准完成后才发布
        if not self._calibrating and hasattr(self, '_last_raw_imu'):
            raw_msg = self._last_raw_imu
            imu_out = Imu()
            imu_out.header = Header()
            imu_out.header.stamp = raw_msg.header.stamp
            imu_out.header.frame_id = 'imu_link'
            # 偏航角（绕 Z 轴）转 quaternion（ROS 标准 ENU: yaw 绕 Z）
            # 注：相机 IMU 实际偏航绕 Y 轴，但这里我们用积分后的 yaw 作为 Z 轴 yaw
            yaw_rad = math.radians(yaw_deg)
            # Z 轴 yaw 转 quaternion（z 轴 0，y 轴 0，仅 yaw）
            cy = math.cos(yaw_rad * 0.5)
            sy = math.sin(yaw_rad * 0.5)
            imu_out.orientation.x = 0.0
            imu_out.orientation.y = 0.0
            imu_out.orientation.z = sy
            imu_out.orientation.w = cy
            # orientation_covariance 对角线（让 EKF 知道这个 orientation 可信）
            # 只填 yaw 项（最后一项），其他设大值表示不可信
            imu_out.orientation_covariance = [
                9999.0, 0.0, 0.0,
                0.0, 9999.0, 0.0,
                0.0, 0.0, 0.01,  # yaw 方向 σ² = 0.01 ≈ 5.7°
            ]
            # 原始角速度 + 加速度（保留 EKF 用于过程模型）
            imu_out.angular_velocity = raw_msg.angular_velocity
            imu_out.angular_velocity_covariance = raw_msg.angular_velocity_covariance
            imu_out.linear_acceleration = raw_msg.linear_acceleration
            imu_out.linear_acceleration_covariance = raw_msg.linear_acceleration_covariance
            self.imu_data_pub.publish(imu_out)


def main():
    rclpy.init()
    node = ImuHeadingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
