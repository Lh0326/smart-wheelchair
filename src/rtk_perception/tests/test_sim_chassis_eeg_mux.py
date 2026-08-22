"""sim_chassis_node 多路 mux 测试：Nav2 vs 脑控模式切换。

测试直接构造 SimChassisNode 实例（已 rclpy.init），
调用回调方法并断言 _vx/_wz/eeg_mode_active 字段——
贴合实际接口（_cmd_cb 直接赋值 _vx/_wz，无 _apply_vel 中间层）。
"""
import math
import time

import pytest
import rclpy
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Bool


@pytest.fixture(scope='module')
def rclpy_init():
    rclpy.init()
    yield
    rclpy.shutdown()


def _twist(linear_x=0.0, angular_z=0.0):
    return Twist(linear=Vector3(x=linear_x), angular=Vector3(z=angular_z))


def test_default_mode_consumes_cmd_vel_safe(rclpy_init):
    """默认 eeg_mode_active=False，/cmd_vel_safe 正常消费（_vx 跟随）。"""
    from rtk_perception.sim_chassis_node import SimChassisNode

    chassis = SimChassisNode()
    try:
        assert chassis.eeg_mode_active is False
        chassis._cmd_cb(_twist(linear_x=0.5, angular_z=0.1))
        assert chassis._vx == pytest.approx(0.5)
        assert chassis._wz == pytest.approx(0.1)
    finally:
        chassis.destroy_node()


def test_eeg_mode_armed_without_motion_consumes_cmd_vel_safe(rclpy_init):
    """eeg_mode_active=True 但未出现脑控动作时，Nav2 继续控制底盘。"""
    from rtk_perception.sim_chassis_node import SimChassisNode

    chassis = SimChassisNode()
    try:
        chassis.eeg_mode_active = True
        chassis._vx = 0.0
        chassis._wz = 0.0
        chassis._cmd_cb(_twist(linear_x=0.99, angular_z=0.5))
        # 脑控只是待命，尚未非零接管；Nav2 指令应继续生效。
        assert chassis._vx == pytest.approx(0.99)
        assert chassis._wz == pytest.approx(0.5)
    finally:
        chassis.destroy_node()


def test_eeg_mode_consumes_cmd_vel_eeg(rclpy_init):
    """eeg_mode_active=True 且 /cmd_vel_eeg 非零时触发脑控接管。"""
    from rtk_perception.sim_chassis_node import SimChassisNode

    chassis = SimChassisNode()
    try:
        chassis.eeg_mode_active = True
        chassis._on_cmd_vel_eeg(_twist(linear_x=0.3, angular_z=-0.2))
        assert chassis._vx == pytest.approx(0.3)
        assert chassis._wz == pytest.approx(-0.2)
        assert chassis._eeg_override_active is True
    finally:
        chassis.destroy_node()


def test_eeg_mode_off_ignores_cmd_vel_eeg(rclpy_init):
    """eeg_mode_active=False 时 /cmd_vel_eeg 被忽略（Nav2 模式下脑控无效）。"""
    from rtk_perception.sim_chassis_node import SimChassisNode

    chassis = SimChassisNode()
    try:
        chassis.eeg_mode_active = False
        chassis._vx = 0.0
        chassis._on_cmd_vel_eeg(_twist(linear_x=0.7))
        assert chassis._vx == pytest.approx(0.0)
    finally:
        chassis.destroy_node()


def test_toggle_eeg_mode_via_callback_does_not_clear_nav2(rclpy_init):
    """收到 Bool(True) 只让脑控待命，不打断正在运行的 Nav2 速度。"""
    from rtk_perception.sim_chassis_node import SimChassisNode

    chassis = SimChassisNode()
    try:
        # 模拟 Nav2 模式下轮椅正在前进
        chassis._cmd_cb(_twist(linear_x=0.5, angular_z=0.2))

        # 收到激活信号
        chassis._on_eeg_mode_active(Bool(data=True))

        assert chassis.eeg_mode_active is True
        assert chassis._eeg_override_active is False
        assert chassis._vx == pytest.approx(0.5)
        assert chassis._wz == pytest.approx(0.2)
    finally:
        chassis.destroy_node()


def test_eeg_zero_command_before_motion_does_not_preempt_nav2(rclpy_init):
    """脑控待命后的 STOP 帧不算用户操控，不能让自主导航停车。"""
    from rtk_perception.sim_chassis_node import SimChassisNode

    chassis = SimChassisNode()
    try:
        chassis._cmd_cb(_twist(linear_x=0.4, angular_z=0.1))
        chassis._on_eeg_mode_active(Bool(data=True))
        chassis._on_cmd_vel_eeg(_twist(linear_x=0.0, angular_z=0.0))
        chassis._select_velocity(chassis.get_clock().now().nanoseconds * 1e-9)

        assert chassis._eeg_override_active is False
        assert chassis._vx == pytest.approx(0.4)
        assert chassis._wz == pytest.approx(0.1)
    finally:
        chassis.destroy_node()


def test_eeg_override_holds_zero_then_releases_to_nav2(rclpy_init):
    """非零脑控动作触发接管；用户松开后保护期内零速，超时回 Nav2。"""
    from rtk_perception.sim_chassis_node import SimChassisNode

    chassis = SimChassisNode()
    try:
        chassis._cmd_cb(_twist(linear_x=0.6, angular_z=0.0))
        chassis._on_eeg_mode_active(Bool(data=True))
        chassis._on_cmd_vel_eeg(_twist(linear_x=0.3, angular_z=0.0))
        assert chassis._vx == pytest.approx(0.3)

        # Nav2 继续在后台刷新，但脑控保护期内优先级更高。
        chassis._cmd_cb(_twist(linear_x=0.9, angular_z=0.0))
        assert chassis._vx == pytest.approx(0.3)

        # 用户松开头姿后，STOP 帧在保护期内仍由脑控执行。
        chassis._on_cmd_vel_eeg(_twist(linear_x=0.0, angular_z=0.0))
        chassis._select_velocity(chassis.get_clock().now().nanoseconds * 1e-9)
        assert chassis._eeg_override_active is True
        assert chassis._vx == pytest.approx(0.0)

        # 保护期结束后自动恢复到最新 Nav2 指令。
        now = chassis.get_clock().now().nanoseconds * 1e-9
        hold = float(chassis.get_parameter("eeg_override_hold_sec").value)
        chassis._last_eeg_motion_sec = now - hold - 0.1
        chassis._select_velocity(now)
        assert chassis._eeg_override_active is False
        assert chassis._vx == pytest.approx(0.9)
    finally:
        chassis.destroy_node()


def test_toggle_off_restores_nav2(rclpy_init):
    """收到 /eeg_mode_active Bool(False) 切回 Nav2 模式。"""
    from rtk_perception.sim_chassis_node import SimChassisNode

    chassis = SimChassisNode()
    try:
        chassis.eeg_mode_active = True
        chassis._on_eeg_mode_active(Bool(data=False))
        assert chassis.eeg_mode_active is False
    finally:
        chassis.destroy_node()


def test_fallback_after_3s_no_update(rclpy_init):
    """3 秒无 /eeg_mode_active 更新 → fallback 到 Nav2 模式。"""
    from rtk_perception.sim_chassis_node import SimChassisNode

    chassis = SimChassisNode()
    try:
        chassis.eeg_mode_active = True
        # 模拟 _last_eeg_mode_msg_time 是 4 秒前
        now = chassis.get_clock().now().nanoseconds * 1e-9
        chassis._last_eeg_mode_msg_time = now - 4.0
        # 触发 fallback 检查
        chassis._check_eeg_mode_fallback()
        assert chassis.eeg_mode_active is False, "应 fallback 到 Nav2 模式"
    finally:
        chassis.destroy_node()


def test_fallback_skipped_when_recent(rclpy_init):
    """1 秒前刚更新 → 不 fallback。"""
    from rtk_perception.sim_chassis_node import SimChassisNode

    chassis = SimChassisNode()
    try:
        chassis.eeg_mode_active = True
        now = chassis.get_clock().now().nanoseconds * 1e-9
        chassis._last_eeg_mode_msg_time = now - 1.0
        chassis._check_eeg_mode_fallback()
        assert chassis.eeg_mode_active is True, "1s 内不应 fallback"
    finally:
        chassis.destroy_node()
