"""ROS 消息装配单测"""
import math

import pytest

from rtk_imu.message_assembly import (
    build_imu_message,
    build_mag_message,
    build_heading_message,
    euler_to_quaternion,
    DEFAULT_ORIENTATION_COVARIANCE,
    DEFAULT_GYRO_COVARIANCE,
    DEFAULT_ACCEL_COVARIANCE,
)
from rtk_imu.packet_parser import PacketResult
from rtk_imu.jy901_protocol import REG_ANGLE, REG_MAG


# ========== Covariance 默认值 ==========

def test_default_orientation_covariance_shape():
    assert len(DEFAULT_ORIENTATION_COVARIANCE) == 9
    # 对角线 [2,2] 应该是 yaw 方差（小值，标定后）
    assert DEFAULT_ORIENTATION_COVARIANCE[8] < 1e-3


def test_default_gyro_covariance_shape():
    assert len(DEFAULT_GYRO_COVARIANCE) == 9


def test_default_accel_covariance_shape():
    assert len(DEFAULT_ACCEL_COVARIANCE) == 9


# ========== 欧拉角 → 四元数 ==========

def test_euler_to_quaternion_identity():
    """0,0,0 → 单位四元数"""
    q = euler_to_quaternion(0.0, 0.0, 0.0)
    assert q == pytest.approx([0.0, 0.0, 0.0, 1.0], abs=1e-6)


def test_euler_to_quaternion_yaw_90():
    """yaw=π/2 → 绕 Z 轴 90°"""
    q = euler_to_quaternion(0.0, 0.0, math.pi / 2)
    assert q[0] == pytest.approx(0.0, abs=1e-6)   # x
    assert q[1] == pytest.approx(0.0, abs=1e-6)   # y
    assert q[2] == pytest.approx(math.sin(math.pi / 4), abs=1e-6)  # z
    assert q[3] == pytest.approx(math.cos(math.pi / 4), abs=1e-6)  # w


def test_euler_to_quaternion_roll_90():
    """roll=π/2 → 绕 X 轴 90°"""
    q = euler_to_quaternion(math.pi / 2, 0.0, 0.0)
    assert q[0] == pytest.approx(math.sin(math.pi / 4), abs=1e-6)
    assert q[1] == pytest.approx(0.0, abs=1e-6)
    assert q[2] == pytest.approx(0.0, abs=1e-6)
    assert q[3] == pytest.approx(math.cos(math.pi / 4), abs=1e-6)


# ========== IMU 消息 ==========

def test_build_imu_message_basic_fields(rclpy_init):
    """构造 Imu 消息：所有字段填齐"""
    angle_result = PacketResult(
        reg=REG_ANGLE,
        angle=[0.1, 0.2, 0.3],  # roll/pitch/yaw
    )
    corrected_gyro = [0.01, 0.02, 0.03]
    acc = [0.0, 0.0, 9.8]

    msg = build_imu_message(
        angle_result=angle_result,
        corrected_gyro=corrected_gyro,
        acc=acc,
        frame_id='imu_link',
        stamp_sec=12345,
        stamp_nanosec=67890,
    )

    assert msg.header.frame_id == 'imu_link'
    assert msg.header.stamp.sec == 12345
    assert msg.header.stamp.nanosec == 67890
    # orientation 应该是 [0.1, 0.2, 0.3] 对应的 quaternion
    expected_q = euler_to_quaternion(0.1, 0.2, 0.3)
    assert msg.orientation.x == pytest.approx(expected_q[0], abs=1e-6)
    assert msg.orientation.y == pytest.approx(expected_q[1], abs=1e-6)
    assert msg.orientation.z == pytest.approx(expected_q[2], abs=1e-6)
    assert msg.orientation.w == pytest.approx(expected_q[3], abs=1e-6)
    # angular_velocity
    assert msg.angular_velocity.x == pytest.approx(0.01)
    assert msg.angular_velocity.y == pytest.approx(0.02)
    assert msg.angular_velocity.z == pytest.approx(0.03)
    # linear_acceleration
    assert msg.linear_acceleration.z == pytest.approx(9.8)
    # covariance 长度 = 9
    assert len(msg.orientation_covariance) == 9
    assert len(msg.angular_velocity_covariance) == 9
    assert len(msg.linear_acceleration_covariance) == 9


def test_build_imu_message_covariance_from_param(rclpy_init):
    """传入自定义 covariance：应该被采用"""
    custom_orient = [1.0] * 9
    custom_gyro = [2.0] * 9
    custom_accel = [3.0] * 9

    angle_result = PacketResult(reg=REG_ANGLE, angle=[0, 0, 0])
    msg = build_imu_message(
        angle_result=angle_result,
        corrected_gyro=[0, 0, 0],
        acc=[0, 0, 0],
        frame_id='imu_link',
        stamp_sec=0,
        stamp_nanosec=0,
        orientation_covariance=custom_orient,
        gyro_covariance=custom_gyro,
        accel_covariance=custom_accel,
    )
    assert list(msg.orientation_covariance) == custom_orient
    assert list(msg.angular_velocity_covariance) == custom_gyro
    assert list(msg.linear_acceleration_covariance) == custom_accel


# ========== Mag 消息 ==========

def test_build_mag_message(rclpy_init):
    mag_result = PacketResult(reg=REG_MAG, mag=[100, -200, 300])
    msg = build_mag_message(
        mag_result=mag_result,
        frame_id='imu_link',
        stamp_sec=1,
        stamp_nanosec=2,
    )
    assert msg.header.frame_id == 'imu_link'
    assert msg.header.stamp.sec == 1
    assert msg.header.stamp.nanosec == 2
    assert msg.magnetic_field.x == 100.0
    assert msg.magnetic_field.y == -200.0
    assert msg.magnetic_field.z == 300.0


# ========== Heading 消息 ==========

def test_build_heading_message_range_0_360(rclpy_init):
    """yaw rad → compass deg（0-360，0=北，顺时针正）"""
    # yaw=0 rad → compass 0°
    msg = build_heading_message(yaw_rad=0.0)
    assert msg.data == pytest.approx(0.0, abs=1e-3)

    # yaw=π/2 rad（ROS 标准 = 西方 = 逆时针 90°）
    # compass 角度 = -yaw_deg % 360 = -90 % 360 = 270°
    msg = build_heading_message(yaw_rad=math.pi / 2)
    assert msg.data == pytest.approx(270.0, abs=1e-3)

    # yaw=π rad → compass 180°
    msg = build_heading_message(yaw_rad=math.pi)
    assert msg.data == pytest.approx(180.0, abs=1e-3)
