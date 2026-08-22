"""JY901 IMU 驱动主节点。

数据流：
    /dev/ttyIMU (USB 串口)
        ↓ pyserial 读线程（1024 字节块）
    PacketParser.feed()
        ↓ List[PacketResult]
    BiasCalibrator.feed() / apply_to_gyro()
        ↓ 校准后 gyro
    累积最新 angle/gyro/acc/mag（按 reg 更新对应字段）
    REG_ANGLE 包到达 → 触发一次发布 /imu/data + /imu/mag + /heading_imu
"""
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Float64
from geometry_msgs.msg import TransformStamped
import tf2_ros
import serial

from rtk_imu.packet_parser import PacketParser, PacketResult
from rtk_imu.calibration import BiasCalibrator
from rtk_imu.message_assembly import (
    build_imu_message,
    build_mag_message,
    build_heading_message,
    euler_to_quaternion,
)
from rtk_imu.jy901_protocol import REG_ACC, REG_GYRO, REG_ANGLE, REG_MAG


class Jy901DriverNode(Node):
    def __init__(self):
        super().__init__('jy901_driver')

        # === 参数声明 ===
        self.declare_parameter('port', '/dev/ttyIMU')
        self.declare_parameter('baud', 921600)  # HWT906P 默认
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('calibration_duration', 3.0)
        # 静态 TF 参数
        self.declare_parameter('imu_position_x', 0.0)
        self.declare_parameter('imu_position_y', 0.0)
        self.declare_parameter('imu_position_z', 0.4)
        self.declare_parameter('imu_rotation_r', 0.0)
        self.declare_parameter('imu_rotation_p', 0.0)
        self.declare_parameter('imu_rotation_y', 0.0)
        # 父 frame
        self.declare_parameter('parent_frame', 'base_link')

        self._port = str(self.get_parameter('port').value)
        self._baud = int(self.get_parameter('baud').value)
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._parent_frame = str(self.get_parameter('parent_frame').value)

        # === Publishers ===
        self._imu_pub = self.create_publisher(Imu, '/imu/data', qos_profile_sensor_data)
        self._mag_pub = self.create_publisher(MagneticField, '/imu/mag', qos_profile_sensor_data)
        self._heading_pub = self.create_publisher(Float64, '/heading_imu', 10)

        # === 静态 TF（base_link → imu_link）===
        self._publish_static_tf()

        # === 解析器 + 校准器 ===
        self._parser = PacketParser()
        cal_duration = float(self.get_parameter('calibration_duration').value)
        self._calibrator = BiasCalibrator(duration_sec=cal_duration)

        # === 最新累积数据（按 reg 更新）===
        self._latest_angle = None       # [roll, pitch, yaw]
        self._latest_acc = [0.0, 0.0, 0.0]
        self._latest_gyro_corrected = [0.0, 0.0, 0.0]
        self._latest_mag = [0, 0, 0]
        self._lock = threading.Lock()

        # === 串口线程 ===
        self._running = True
        self._serial = None
        self._serial_thread = threading.Thread(target=self._serial_loop, daemon=True)
        self._serial_thread.start()

        # === 调试统计 ===
        self._packet_count = 0
        self._create_debug_timer()

        self.get_logger().info(
            f"JY901 driver 启动：port={self._port}, baud={self._baud}, "
            f"frame_id={self._frame_id}, parent={self._parent_frame}"
        )
        self.get_logger().info(
            f"启动静态校准（请保持静止 {cal_duration:.1f} 秒）..."
        )

    def _publish_static_tf(self):
        """发布 base_link → imu_link 静态 TF。"""
        x = float(self.get_parameter('imu_position_x').value)
        y = float(self.get_parameter('imu_position_y').value)
        z = float(self.get_parameter('imu_position_z').value)
        r = float(self.get_parameter('imu_rotation_r').value)
        p = float(self.get_parameter('imu_rotation_p').value)
        yaw = float(self.get_parameter('imu_rotation_y').value)

        qx, qy, qz, qw = euler_to_quaternion(r, p, yaw)
        self._tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self._parent_frame
        t.child_frame_id = self._frame_id
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self._tf_static_broadcaster.sendTransform(t)

    def _serial_loop(self):
        """串口读线程主循环（含自动重连）。

        HWT906P 在 USB autosuspend/物理接触不良时偶尔会断开，
        这里捕获异常后等 1 秒重新打开串口，避免节点退出。
        """
        while self._running and rclpy.ok():
            try:
                self._serial = serial.Serial(self._port, self._baud, timeout=1.0)
            except (serial.SerialException, PermissionError, OSError) as e:
                self.get_logger().error(f"串口打开失败 {self._port}: {e}")
                time.sleep(1.0)
                continue

            self.get_logger().info(f"串口打开成功：{self._port} @ {self._baud} bps")

            while self._running and rclpy.ok():
                try:
                    chunk = self._serial.read(1024)
                    if chunk:
                        self._process_bytes(chunk)
                except (serial.SerialException, OSError) as e:
                    self.get_logger().warn(f"串口读异常（将重连）：{e}")
                    break

            try:
                if self._serial:
                    self._serial.close()
            except Exception:
                pass
            time.sleep(1.0)  # 重连前等 1 秒

        self.get_logger().info("串口线程退出")

    def _process_bytes(self, data: bytes):
        """处理串口读到的字节流。"""
        results = self._parser.feed(data)

        should_publish = False
        for result in results:
            self._packet_count += 1

            # 喂给校准器（启动 3 秒收集 bias）
            self._calibrator.feed(result)

            # 累积最新数据
            with self._lock:
                if result.reg == REG_ACC and result.acc is not None:
                    self._latest_acc = result.acc
                elif result.reg == REG_GYRO and result.gyro is not None:
                    if self._calibrator.is_calibrating():
                        self._latest_gyro_corrected = result.gyro
                    else:
                        self._latest_gyro_corrected = self._calibrator.apply_to_gyro(result.gyro)
                elif result.reg == REG_ANGLE and result.angle is not None:
                    self._latest_angle = result.angle
                    should_publish = True  # 标记发布，但不在锁内发布
                elif result.reg == REG_MAG and result.mag is not None:
                    self._latest_mag = result.mag

        # 锁外发布（_publish_all 内部会自己 acquire 锁读取最新数据）
        if should_publish:
            self._publish_all()

    def _publish_all(self):
        """发布 /imu/data + /imu/mag + /heading_imu（在 angle 包到达时触发）。"""
        if self._calibrator.is_calibrating():
            return  # 校准期不发布

        if self._latest_angle is None:
            return

        with self._lock:
            angle = list(self._latest_angle)
            gyro = list(self._latest_gyro_corrected)
            acc = list(self._latest_acc)
            mag = list(self._latest_mag)
            # 构造 PacketResult（用于 build_*_message）
            angle_result = PacketResult(reg=REG_ANGLE, angle=angle)
            mag_result = PacketResult(reg=REG_MAG, mag=mag)

        stamp = self.get_clock().now().to_msg()
        sec = stamp.sec
        nanosec = stamp.nanosec

        imu_msg = build_imu_message(
            angle_result=angle_result,
            corrected_gyro=gyro,
            acc=acc,
            frame_id=self._frame_id,
            stamp_sec=sec,
            stamp_nanosec=nanosec,
        )
        mag_msg = build_mag_message(
            mag_result=mag_result,
            frame_id=self._frame_id,
            stamp_sec=sec,
            stamp_nanosec=nanosec,
        )
        heading_msg = build_heading_message(yaw_rad=angle[2])

        self._imu_pub.publish(imu_msg)
        self._mag_pub.publish(mag_msg)
        self._heading_pub.publish(heading_msg)

    def _create_debug_timer(self):
        """每 5 秒打印一次状态。"""
        self.create_timer(5.0, self._debug_log)
        self._debug_count = 0

    def _debug_log(self):
        self._debug_count += 1
        with self._lock:
            calibrating = self._calibrator.is_calibrating()
            yaw_deg = math.degrees(self._latest_angle[2]) if self._latest_angle else 0.0
        dropped = self._parser.dropped_count
        status = '校准中' if calibrating else '发布中'
        self.get_logger().info(
            f"[状态] {status} | 包计数={self._packet_count} | "
            f"丢包={dropped} | yaw={yaw_deg:+.1f}°"
        )


def main():
    rclpy.init()
    node = Jy901DriverNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        # 外部信号（Ctrl+C / timeout SIGTERM / ros2 daemon 关闭）→ 优雅退出
        pass
    finally:
        node._running = False
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            # 外部已 shutdown 时再次调用会抛错，忽略
            pass


if __name__ == '__main__':
    main()
