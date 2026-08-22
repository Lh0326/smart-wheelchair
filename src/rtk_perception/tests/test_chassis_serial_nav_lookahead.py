"""chassis_serial_node Nav2 lookahead + 档位 Layer 2 集成测试（spec § 5.2）。

测试直接构造 ChassisSerialNode 实例（mock 串口，不 spin），覆盖：
- 9 个新 ROS 参数默认值
- 参数校验 fail-fast
- /local_plan 订阅 + lookahead direction_angle
- /nav_speed_mode 订阅 + 档位 forward_speed
- fallback 链
"""
import math
import pytest
import rclpy
from geometry_msgs.msg import Pose, PoseArray, Point
from std_msgs.msg import Float64, Int8


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
    node._cmd_vel_ramp_rate = 0.0
    return node, mock


def test_nav_lookahead_params_default_values(rclpy_init):
    """9 个新参数都声明，默认值与 spec § 3 一致。"""
    node, _ = _make_node()
    try:
        assert node.get_parameter('nav_use_local_plan_lookahead').value is True
        assert node.get_parameter('nav_lookahead_distance_m').value == 0.5
        assert node.get_parameter('nav_lookahead_lpf_alpha').value == 0.3
        assert node.get_parameter('nav_local_plan_timeout_sec').value == 0.5
        assert node.get_parameter('nav_teb_max_vel_x').value == 0.6
        assert node.get_parameter('nav_speed_mode_default').value == 2
        assert node.get_parameter('nav_speed_slow').value == 400
        assert node.get_parameter('nav_speed_medium').value == 700
        assert node.get_parameter('nav_speed_fast').value == 1000
    finally:
        node.destroy_node()


def test_nav_lookahead_distance_invalid_raises(rclpy_init):
    """nav_lookahead_distance_m=0 → ValueError（必须 > 0）。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode

    with pytest.raises(ValueError, match="nav_lookahead_distance_m"):
        ChassisSerialNode(
            serial_factory=lambda **kwargs: MockSerial(),
            open_serial=False,
            overrides={'nav_lookahead_distance_m': 0.0},
        )


def test_nav_lookahead_lpf_alpha_out_of_range_raises(rclpy_init):
    """nav_lookahead_lpf_alpha=1.5 → ValueError（必须 ≤ 1）。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode

    with pytest.raises(ValueError, match="nav_lookahead_lpf_alpha"):
        ChassisSerialNode(
            serial_factory=lambda **kwargs: MockSerial(),
            open_serial=False,
            overrides={'nav_lookahead_lpf_alpha': 1.5},
        )


def test_nav_speed_mode_invalid_raises(rclpy_init):
    """nav_speed_mode_default=4 → ValueError（必须 ∈ {1,2,3}）。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode

    with pytest.raises(ValueError, match="nav_speed_mode_default"):
        ChassisSerialNode(
            serial_factory=lambda **kwargs: MockSerial(),
            open_serial=False,
            overrides={'nav_speed_mode_default': 4},
        )


def test_nav_speed_ordering_raises(rclpy_init):
    """nav_speed_slow >= medium → ValueError（必须 slow<medium<fast）。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode

    with pytest.raises(ValueError, match="nav_speed_slow|nav_speed_medium|nav_speed_fast"):
        ChassisSerialNode(
            serial_factory=lambda **kwargs: MockSerial(),
            open_serial=False,
            overrides={'nav_speed_slow': 800, 'nav_speed_medium': 700},
        )


def test_nav_local_plan_timeout_invalid_raises(rclpy_init):
    """nav_local_plan_timeout_sec=0 → ValueError（必须 > 0）。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode

    with pytest.raises(ValueError, match="nav_local_plan_timeout_sec"):
        ChassisSerialNode(
            serial_factory=lambda **kwargs: MockSerial(),
            open_serial=False,
            overrides={'nav_local_plan_timeout_sec': 0.0},
        )


def test_nav_teb_max_vel_x_invalid_raises(rclpy_init):
    """nav_teb_max_vel_x=0 → ValueError（必须 > 0）。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode

    with pytest.raises(ValueError, match="nav_teb_max_vel_x"):
        ChassisSerialNode(
            serial_factory=lambda **kwargs: MockSerial(),
            open_serial=False,
            overrides={'nav_teb_max_vel_x': 0.0},
        )


# ===== /local_plan 订阅 + _tick lookahead 集成测试（任务 4）=====

from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Bool


def test_geometry_lookahead_ignores_noisy_pose_orientation():
    from rtk_perception.chassis_serial_node import extract_lookahead_bearing

    plan = PoseArray()
    plan.header.frame_id = 'base_link'
    first = _pose_at(0.0, -35.0)
    second = _pose_at(1.0, 40.0)
    second.position.y = 0.2
    plan.poses = [first, second]

    bearing = extract_lookahead_bearing(plan, 1.0)
    assert math.degrees(bearing) == pytest.approx(math.degrees(math.atan2(0.2, 1.0)))


def test_gps_required_gate_rejects_invalid_and_stale_fix(rclpy_init):
    from rclpy.parameter import Parameter
    from sensor_msgs.msg import NavSatFix

    node, _ = _make_node(overrides={
        'gps_required_for_motion': True,
        'gps_fix_timeout_sec': 1.5,
    })
    try:
        invalid = NavSatFix()
        invalid.status.status = -1
        node._gps_fix_cb(invalid)
        assert not node._is_gps_ready(node._now_sec())

        valid = NavSatFix()
        valid.status.status = 0
        valid.latitude = 24.85
        valid.longitude = 102.85
        node._gps_fix_cb(valid)
        assert node._is_gps_ready(node._now_sec())

        node._latest_gps_sec = node._now_sec() - 2.0
        assert not node._is_gps_ready(node._now_sec())
    finally:
        node.destroy_node()


def _twist(linear_x=0.0, angular_z=0.0):
    return Twist(linear=Vector3(x=linear_x), angular=Vector3(z=angular_z))


def _pose_at(x_m: float, yaw_deg: float) -> Pose:
    p = Pose()
    p.position = Point(x=x_m, y=0.0, z=0.0)
    half = math.radians(yaw_deg) / 2.0
    p.orientation.z = math.sin(half)
    p.orientation.w = math.cos(half)
    return p


def _pose_at_bearing(distance_m: float, bearing_deg: float) -> Pose:
    p = _pose_at(0.0, bearing_deg)
    bearing_rad = math.radians(bearing_deg)
    p.position.x = distance_m * math.cos(bearing_rad)
    p.position.y = distance_m * math.sin(bearing_rad)
    return p


def _pose_array(poses: list, frame_id: str = 'base_link') -> PoseArray:
    pa = PoseArray()
    pa.header.frame_id = frame_id
    pa.poses = poses
    return pa


def _arm_nav(node, linear_x=0.5, omega=0.0):
    """让节点进入 Nav2 模式：nav_control_active 心跳 + cmd_vel_safe + heading。"""
    node._nav_control_active = True
    node._latest_nav_control_sec = node._now_sec()
    node._latest_heading_deg = 0.0  # 假设朝北
    node._latest_heading_sec = node._now_sec()
    from rclpy.parameter import Parameter
    node.set_parameters([Parameter('nav_chassis_control_enabled', value=True)])
    node._cmd_vel_cb(_twist(linear_x=linear_x, angular_z=omega))


def test_local_plan_cb_caches(rclpy_init):
    """/local_plan 回调缓存 PoseArray + 时间戳。"""
    node, _ = _make_node()
    try:
        pa = _pose_array([_pose_at(0.5, 30.0)])
        node._local_plan_cb(pa)
        assert node._latest_local_plan is not None
        assert node._latest_local_plan_sec > 0.0
    finally:
        node.destroy_node()


def test_tick_with_local_plan_uses_lookahead(rclpy_init):
    """有 /local_plan 时 direction_angle 反映 lookahead 朝向（HWT_yaw=0 + 30°偏角）。"""
    node, mock = _make_node()
    try:
        _arm_nav(node, linear_x=0.6, omega=0.0)
        pa = _pose_array([_pose_at(0.0, 0.0), _pose_at_bearing(0.5, 30.0)])
        node._local_plan_cb(pa)
        node._tick()

        last_frame = mock.writes[-1].decode('ascii')
        parts = last_frame.strip().split(',')
        # LPF 首帧 filtered = 0 + 0.3*(30-0) = 9
        assert int(parts[0]) == 9  # direction
        assert int(parts[1]) == 0  # current = HWT_yaw = 0
    finally:
        node.destroy_node()


def test_tick_no_local_plan_falls_back_to_angular_z(rclpy_init):
    """无 /local_plan → fallback 到 angular.z 前馈（既有逻辑）。"""
    node, mock = _make_node()
    try:
        _arm_nav(node, linear_x=0.6, omega=0.5)
        # 不发 /local_plan
        node._tick()

        last_frame = mock.writes[-1].decode('ascii')
        parts = last_frame.strip().split(',')
        # omega 先 cap 到 0.3：current(0) + 0.3*30*1*(-1) = -9
        assert int(parts[0]) == -9
    finally:
        node.destroy_node()


def test_tick_stale_local_plan_falls_back(rclpy_init):
    """/local_plan > 0.5s 未更新 → fallback。"""
    node, mock = _make_node()
    try:
        _arm_nav(node, linear_x=0.6, omega=0.5)
        pa = _pose_array([_pose_at(0.0, 0.0), _pose_at(0.5, 30.0)])
        node._local_plan_cb(pa)
        node._latest_local_plan_sec = node._now_sec() - 4.0
        node._tick()

        last_frame = mock.writes[-1].decode('ascii')
        parts = last_frame.strip().split(',')
        # fallback 到 angular.z 前馈（omega cap=0.3）→ direction=-9
        assert int(parts[0]) == -9
    finally:
        node.destroy_node()


def test_tick_local_plan_wrong_frame_falls_back(rclpy_init):
    """frame_id != 'base_link' → fallback。"""
    node, mock = _make_node()
    try:
        _arm_nav(node, linear_x=0.6, omega=0.5)
        pa = _pose_array([_pose_at(0.0, 0.0), _pose_at(0.5, 30.0)], frame_id='odom')
        node._local_plan_cb(pa)
        node._tick()

        last_frame = mock.writes[-1].decode('ascii')
        parts = last_frame.strip().split(',')
        # fallback（omega cap=0.3）→ direction=-9
        assert int(parts[0]) == -9
    finally:
        node.destroy_node()


def test_tick_lookahead_disabled_param(rclpy_init):
    """nav_use_local_plan_lookahead=False → 完全旁路。"""
    node, mock = _make_node(overrides={'nav_use_local_plan_lookahead': False})
    try:
        _arm_nav(node, linear_x=0.6, omega=0.5)
        pa = _pose_array([_pose_at(0.0, 0.0), _pose_at(0.5, 30.0)])
        node._local_plan_cb(pa)
        node._tick()

        last_frame = mock.writes[-1].decode('ascii')
        parts = last_frame.strip().split(',')
        # 完全旁路 → direction=-9（angular.z 先 cap 到 0.3）
        assert int(parts[0]) == -9
    finally:
        node.destroy_node()


def test_tick_lpf_converges_on_steady_input(rclpy_init):
    """连续发相同 local_plan，LPF 应收敛到固定值（30°）。"""
    node, mock = _make_node()
    try:
        _arm_nav(node, linear_x=0.6, omega=0.0)
        pa = _pose_array([_pose_at(0.0, 0.0), _pose_at_bearing(0.5, 30.0)])
        for _ in range(20):
            node._local_plan_cb(pa)
            node._tick()
        last_frame = mock.writes[-1].decode('ascii')
        parts = last_frame.strip().split(',')
        # 收敛后 direction 应接近 30
        assert abs(int(parts[0]) - 30) <= 1
    finally:
        node.destroy_node()


# ===== /nav_speed_mode 订阅 + 档位运行时切换（任务 5）=====


def test_speed_mode_cb_changes_mode(rclpy_init):
    """/nav_speed_mode 回调切换 self._current_speed_mode。"""
    node, _ = _make_node()
    try:
        assert node._current_speed_mode == 2  # 默认中
        node._speed_mode_cb(Int8(data=1))
        assert node._current_speed_mode == 1
        node._speed_mode_cb(Int8(data=3))
        assert node._current_speed_mode == 3
    finally:
        node.destroy_node()


def test_speed_mode_cb_ignores_invalid(rclpy_init):
    """无效值（4）→ 忽略，保持原档位。"""
    node, _ = _make_node()
    try:
        node._speed_mode_cb(Int8(data=2))
        node._speed_mode_cb(Int8(data=4))  # 无效
        assert node._current_speed_mode == 2  # 不变
    finally:
        node.destroy_node()


def test_tick_forward_speed_uses_fast_mode(rclpy_init):
    """档位=3（快）+ TEB 全速 → forward_speed=1000。"""
    node, mock = _make_node()
    try:
        _arm_nav(node, linear_x=0.6, omega=0.0)
        node._speed_mode_cb(Int8(data=3))
        # 不发 local_plan，让它走 fallback
        node._tick()

        last_frame = mock.writes[-1].decode('ascii')
        parts = last_frame.strip().split(',')
        # forward_speed = (0.6 / 0.6) × 1000 = 1000
        assert int(parts[2]) == 1000
    finally:
        node.destroy_node()


def test_tick_speed_mode_switch_takes_effect_next_tick(rclpy_init):
    """档位切换后下一帧立即生效。"""
    node, mock = _make_node()
    try:
        _arm_nav(node, linear_x=0.6, omega=0.0)
        node._tick()
        # 中速 → 700
        assert int(mock.writes[-1].decode('ascii').strip().split(',')[2]) == 700

        node._speed_mode_cb(Int8(data=1))  # 切慢速
        node._tick()
        # 慢速 → 400
        assert int(mock.writes[-1].decode('ascii').strip().split(',')[2]) == 400
    finally:
        node.destroy_node()


# ===== floor 兜底逻辑测试（spec § 3.2 min_forward_speed 保留）=====


def test_tick_min_forward_speed_floor_applied(rclpy_init):
    """min_forward_speed=180 + linear.x=0.06 → 档位算下来 70 < 180 → 抬到 180。"""
    node, mock = _make_node(overrides={
        'nav_min_forward_speed': 180,
        'nav_floor_linear_threshold_mps': 0.05,
    })
    try:
        _arm_nav(node, linear_x=0.06, omega=0.0)  # TEB 输出 0.06 m/s（略大于 floor_threshold）
        # mode=medium(700), teb_max_vel_x=0.6 → normalized=0.1 → 70 < 180 → 抬到 180
        node._tick()

        last_frame = mock.writes[-1].decode('ascii')
        parts = last_frame.strip().split(',')
        assert int(parts[2]) == 180
    finally:
        node.destroy_node()
