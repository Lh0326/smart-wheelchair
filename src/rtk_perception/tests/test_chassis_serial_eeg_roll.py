"""chassis_serial_node EEG roll 补偿 Layer 2 集成测试（spec § 7.2）。

测试直接构造 ChassisSerialNode 实例（mock 串口，不 spin），覆盖：
- 参数声明与默认值
- 参数校验 fail-fast
- /eeg_head_pose 订阅缓存
- _tick 内 roll 接入 FORWARD/BACKWARD 帧
- LEFT/RIGHT 帧不消费 roll
- roll 缓存超时回零
"""
import pytest
import rclpy
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Bool, Float64


class MockSerial:
    """模拟 serial.Serial，捕获写入字节供断言。"""

    def __init__(self, *args, **kwargs):
        self.writes: list[bytes] = []
        self.is_open = True

    def write(self, data: bytes) -> int:
        self.writes.append(data)
        return len(data)

    def close(self):
        self.is_open = False


@pytest.fixture(scope='module')
def rclpy_init():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture(autouse=True, scope='module')
def _disable_slew_for_legacy_tests():
    """既有测试基于"无限速器"行为设计，自动禁用避免回归。

    限速器行为由 test_chassis_slew_limiter.py 单独验证。
    """
    import rtk_perception.chassis_serial_node as csn
    orig_set_enabled = csn.ChassisSlewLimiter.set_enabled

    def force_disabled(self, enabled):
        orig_set_enabled(self, False)

    csn.ChassisSlewLimiter.set_enabled = force_disabled
    yield
    csn.ChassisSlewLimiter.set_enabled = orig_set_enabled


def _make_node(overrides=None):
    """构造 ChassisSerialNode（注入 MockSerial，不真开串口）。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode

    mock = MockSerial()
    node = ChassisSerialNode(
        serial_factory=lambda **kwargs: mock,
        open_serial=False,
        overrides=overrides or {},
    )
    return node, mock


def test_roll_params_default_values(rclpy_init):
    """5 个新参数都声明，默认值与 spec § 3 一致。"""
    node, _ = _make_node()
    try:
        assert node.get_parameter('eeg_roll_compensation_enabled').value is True
        assert node.get_parameter('eeg_roll_gain_deg_per_deg').value == 0.5
        assert node.get_parameter('eeg_roll_saturation_deg').value == 20.0
        assert node.get_parameter('eeg_roll_polarity').value == -1
        assert node.get_parameter('eeg_head_pose_timeout_sec').value == 3.0
    finally:
        node.destroy_node()


def test_roll_polarity_invalid_raises(rclpy_init):
    """eeg_roll_polarity=0 → ValueError（必须 ∈ {-1, 1}）。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode

    with pytest.raises(ValueError, match="eeg_roll_polarity"):
        ChassisSerialNode(
            serial_factory=lambda **kwargs: MockSerial(),
            open_serial=False,
            overrides={'eeg_roll_polarity': 0},
        )


def test_roll_saturation_zero_raises(rclpy_init):
    """eeg_roll_saturation_deg=0 → ValueError（必须 > 0）。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode

    with pytest.raises(ValueError, match="eeg_roll_saturation_deg"):
        ChassisSerialNode(
            serial_factory=lambda **kwargs: MockSerial(),
            open_serial=False,
            overrides={'eeg_roll_saturation_deg': 0.0},
        )


def test_roll_gain_negative_raises(rclpy_init):
    """eeg_roll_gain_deg_per_deg=-0.1 → ValueError（必须 ≥ 0）。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode

    with pytest.raises(ValueError, match="eeg_roll_gain_deg_per_deg"):
        ChassisSerialNode(
            serial_factory=lambda **kwargs: MockSerial(),
            open_serial=False,
            overrides={'eeg_roll_gain_deg_per_deg': -0.1},
        )


# ===== _head_pose_cb + _tick 集成测试（任务 5）=====


def _twist(linear_x=0.0, angular_z=0.0):
    return Twist(linear=Vector3(x=linear_x), angular=Vector3(z=angular_z))


def _arm_eeg_override(node, linear_x=0.5):
    """让节点进入 EEG override 状态：mode 心跳 + 非零 cmd_vel_eeg。"""
    node.eeg_mode_active = True
    node._last_eeg_mode_msg_time = node._now_sec()
    node._cmd_vel_eeg_cb(_twist(linear_x=linear_x))
    return node._now_sec()


def test_head_pose_cb_caches_roll(rclpy_init):
    """/eeg_head_pose 回调缓存 roll + 刷新时间戳。"""
    node, _ = _make_node()
    try:
        node._head_pose_cb(Float64(data=20.0))
        assert node._latest_roll_deg == 20.0
        assert node._latest_head_pose_sec > 0.0
    finally:
        node.destroy_node()


def test_tick_forward_with_roll_outputs_offset_direction(rclpy_init):
    """FORWARD 帧 + roll=20° + HWT=0 → direction_angle=-10（默认 polarity=-1, gain=0.5）。"""
    node, mock = _make_node()
    try:
        _arm_eeg_override(node, linear_x=0.5)
        node._heading_cb(Float64(data=0.0))
        node._head_pose_cb(Float64(data=20.0))
        node._tick()

        assert len(mock.writes) > 0
        last_frame = mock.writes[-1].decode('ascii')
        # 帧格式 "\n<direction>,<current>,<speed>"
        # direction = wrap_deg(current=0 + frame_offset=0 + roll_offset=-10) = -10
        # current=HWT=0, speed=900（FORWARD 默认）
        assert last_frame == "\n-10,0,900"
    finally:
        node.destroy_node()


def test_tick_forward_saturates_on_large_roll(rclpy_init):
    """FORWARD + roll=100° + HWT=0 → direction 饱和到 -20。"""
    node, mock = _make_node()
    try:
        _arm_eeg_override(node, linear_x=0.5)
        node._heading_cb(Float64(data=0.0))
        node._head_pose_cb(Float64(data=100.0))
        node._tick()

        last_frame = mock.writes[-1].decode('ascii')
        assert last_frame == "\n-20,0,900"
    finally:
        node.destroy_node()


def test_tick_left_ignores_roll(rclpy_init):
    """LEFT 帧 + roll=20° + HWT=0 → direction=wrap_deg(0-90)=-90（不消费 roll）。"""
    node, mock = _make_node()
    try:
        _arm_eeg_override(node, linear_x=0.0)
        # LEFT 是 omega_rad_s > 0；先发布 LEFT cmd_vel_eeg
        node._cmd_vel_eeg_cb(_twist(linear_x=0.0, angular_z=0.5))
        node._heading_cb(Float64(data=0.0))
        node._head_pose_cb(Float64(data=20.0))
        node._tick()

        last_frame = mock.writes[-1].decode('ascii')
        # direction = wrap_deg(current=0 + left_offset=-90) = -90, current=0, speed=300
        assert last_frame == "\n-90,0,300"
    finally:
        node.destroy_node()


def test_tick_roll_timeout_falls_back_to_zero(rclpy_init):
    """roll 缓存 > 3s 未更新 → direction_offset 回 0；HWT=0 → direction=0。"""
    node, mock = _make_node()
    try:
        _arm_eeg_override(node, linear_x=0.5)
        node._heading_cb(Float64(data=0.0))
        node._head_pose_cb(Float64(data=20.0))
        # 模拟 4 秒前收到的 roll
        node._latest_head_pose_sec = node._now_sec() - 4.0
        node._tick()

        last_frame = mock.writes[-1].decode('ascii')
        # roll 回 0 → direction = wrap_deg(0 + 0 + 0) = 0
        assert last_frame == "\n0,0,900"
    finally:
        node.destroy_node()


def test_tick_roll_compensation_disabled(rclpy_init):
    """eeg_roll_compensation_enabled=False + roll=20° + HWT=0 → direction=0。"""
    node, mock = _make_node(overrides={'eeg_roll_compensation_enabled': False})
    try:
        _arm_eeg_override(node, linear_x=0.5)
        node._heading_cb(Float64(data=0.0))
        node._head_pose_cb(Float64(data=20.0))
        node._tick()

        last_frame = mock.writes[-1].decode('ascii')
        assert last_frame == "\n0,0,900"
    finally:
        node.destroy_node()
