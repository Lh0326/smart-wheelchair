"""ROS 消息装配：PacketResult → sensor_msgs/Imu, MagneticField, std_msgs/Float64。

Euler 角约定（维特 JY901 输出，ROS REP-103 ENU）：
    roll  = 绕 X 轴
    pitch = 绕 Y 轴
    yaw   = 绕 Z 轴（垂直）

heading_deg 约定（与 imu_heading_node 输出契约一致）：
    compass 角度，0=北，顺时针为正（与 ROS yaw 相反）
    compass_deg = (-yaw_deg) % 360
"""
import math

from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Float64, Header
from builtin_interfaces.msg import Time

from rtk_imu.packet_parser import PacketResult


# Covariance 默认值（严格精度指标）
# orientation_covariance[2,2] = 6e-5 rad² ≈ σ=0.5°（标定后）
DEFAULT_ORIENTATION_COVARIANCE = [
    9999.0, 0.0, 0.0,
    0.0, 9999.0, 0.0,
    0.0, 0.0, 6e-5,
]

# angular_velocity σ = 1e-3 rad/s（σ² = 1e-6）
DEFAULT_GYRO_COVARIANCE = [
    1e-6, 0.0, 0.0,
    0.0, 1e-6, 0.0,
    0.0, 0.0, 1e-6,
]

# linear_acceleration σ = 0.1 m/s²（σ² = 0.01）
DEFAULT_ACCEL_COVARIANCE = [
    0.01, 0.0, 0.0,
    0.0, 0.01, 0.0,
    0.0, 0.0, 0.01,
]


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> list:
    """欧拉角（rad，ZYX 顺序）→ 四元数 [x, y, z, w]。"""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    w = cr * cp * cy + sr * sp * sy
    return [x, y, z, w]


def _make_time(sec: int, nanosec: int) -> Time:
    t = Time()
    t.sec = sec
    t.nanosec = nanosec
    return t


def _make_header(frame_id: str, sec: int, nanosec: int) -> Header:
    h = Header()
    h.stamp = _make_time(sec, nanosec)
    h.frame_id = frame_id
    return h


def build_imu_message(
    angle_result: PacketResult,
    corrected_gyro: list,
    acc: list,
    frame_id: str,
    stamp_sec: int,
    stamp_nanosec: int,
    orientation_covariance: list = None,
    gyro_covariance: list = None,
    accel_covariance: list = None,
) -> Imu:
    """构造 sensor_msgs/Imu 消息。

    Args:
        angle_result: REG_ANGLE 包，含 [roll, pitch, yaw]
        corrected_gyro: 校准后的 [wx, wy, wz] rad/s
        acc: 原始 [ax, ay, az] m/s²（含重力，EKF 会自动减）
        frame_id: TF frame，默认 imu_link
        stamp_sec/nanosec: 时间戳
        orientation_covariance/gyro_covariance/accel_covariance: 自定义协方差矩阵（9 元素）
    """
    if orientation_covariance is None:
        orientation_covariance = DEFAULT_ORIENTATION_COVARIANCE
    if gyro_covariance is None:
        gyro_covariance = DEFAULT_GYRO_COVARIANCE
    if accel_covariance is None:
        accel_covariance = DEFAULT_ACCEL_COVARIANCE

    msg = Imu()
    msg.header = _make_header(frame_id, stamp_sec, stamp_nanosec)

    # orientation（从欧拉角转换）
    roll, pitch, yaw = angle_result.angle
    qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)
    msg.orientation.x = qx
    msg.orientation.y = qy
    msg.orientation.z = qz
    msg.orientation.w = qw
    msg.orientation_covariance = orientation_covariance

    # angular_velocity（校准后）
    msg.angular_velocity.x = float(corrected_gyro[0])
    msg.angular_velocity.y = float(corrected_gyro[1])
    msg.angular_velocity.z = float(corrected_gyro[2])
    msg.angular_velocity_covariance = gyro_covariance

    # linear_acceleration（原始，含重力）
    msg.linear_acceleration.x = float(acc[0])
    msg.linear_acceleration.y = float(acc[1])
    msg.linear_acceleration.z = float(acc[2])
    msg.linear_acceleration_covariance = accel_covariance

    return msg


def build_mag_message(
    mag_result: PacketResult,
    frame_id: str,
    stamp_sec: int,
    stamp_nanosec: int,
) -> MagneticField:
    """构造 sensor_msgs/MagneticField 消息（原始 int16 → float）。"""
    msg = MagneticField()
    msg.header = _make_header(frame_id, stamp_sec, stamp_nanosec)
    mx, my, mz = mag_result.mag
    msg.magnetic_field.x = float(mx)
    msg.magnetic_field.y = float(my)
    msg.magnetic_field.z = float(mz)
    return msg


def build_heading_message(yaw_rad: float) -> Float64:
    """构造 /heading_imu 消息（compass deg，0=北，顺时针正）。

    Float64 无 header 字段，时间戳由发布者节点header自然处理。
    与 imu_heading_node 输出契约一致（path_to_baselink_node 订阅）。
    """
    yaw_deg = math.degrees(yaw_rad)
    compass_deg = (-yaw_deg) % 360.0
    msg = Float64()
    msg.data = compass_deg
    return msg
