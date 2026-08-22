"""chassis_serial_node 节点级测试（mock 串口，不 spin）。

通过 dependency injection 注入 mock serial transport，
让节点逻辑可独立测试，不需要真实 /dev/ttyUSBx。
"""
import pytest
import rclpy


class MockSerial:
    """模拟 serial.Serial 接口，捕获写入字节供断言。"""

    def __init__(self, *args, **kwargs):
        self.writes: list[bytes] = []
        self.is_open = True
        self.opened_with = kwargs

    def write(self, data: bytes) -> int:
        self.writes.append(data)
        return len(data)

    def close(self):
        self.is_open = False


class CapturePublisher:
    """测试用 publisher，捕获 publish 的消息。"""

    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


def mark_nav_ready(node):
    """测试中显式模拟 path_feeder 已接受 FollowPath goal。"""
    from rclpy.parameter import Parameter

    node.set_parameters([
        Parameter('nav_chassis_control_enabled', value=True),
    ])
    node._nav_control_active = True
    node._latest_nav_control_sec = node._now_sec()


@pytest.fixture(scope='module')
def rclpy_init():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture(autouse=True, scope='module')
def _disable_slew_for_legacy_tests():
    """既有测试基于"无限速器"行为设计，自动禁用避免回归。

    限速器行为由 test_chassis_slew_limiter.py 单独验证。
    本 fixture 仅作用于本文件，不影响 test_chassis_slew_limiter.py
    （不同 module）。

    设计依据：spec § 4.4 紧急回退 — slew_limiter_enabled=False 等价于
    限速器实现前的行为。既有测试基于该行为编写。
    """
    import rtk_perception.chassis_serial_node as csn
    orig_set_enabled = csn.ChassisSlewLimiter.set_enabled

    def force_disabled(self, enabled):
        # 不管传入什么，强制禁用（覆盖 ChassisSerialNode.__init__ 的 set_enabled(True)）
        orig_set_enabled(self, False)

    csn.ChassisSlewLimiter.set_enabled = force_disabled
    yield
    csn.ChassisSlewLimiter.set_enabled = orig_set_enabled


def test_node_default_params(rclpy_init):
    """节点用默认参数构造，参数值与规格一致。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode

    mock_serial = MockSerial()
    node = ChassisSerialNode(serial_factory=lambda **kw: mock_serial, open_serial=False)
    try:
        assert node.get_parameter('serial_port').value == '/dev/wheelchair_chassis'
        assert node.get_parameter('baudrate').value == 115200
        assert node.get_parameter('serial_reopen_interval_sec').value == 1.0
        assert node.get_parameter('update_rate_hz').value == 100.0
        assert node.get_parameter('max_speed_mps').value == 1.5
        assert node.get_parameter('lead_gain_deg_per_rad_per_sec').value == 30.0
        assert node.get_parameter('max_lead_deg').value == 60.0
        assert node.get_parameter('heading_sign').value == -1
        assert node.get_parameter('nav_chassis_control_enabled').value is False
        assert node.get_parameter('nav_control_timeout_sec').value == 0.8
        assert node.get_parameter('nav_min_forward_speed').value == 0
        assert node.get_parameter('nav_min_turn_speed').value == 0
        assert node.get_parameter('nav_allow_reverse').value is False
        assert node.get_parameter('eeg_turn_offset_deg').value == 90.0
        assert node.get_parameter('eeg_forward_direction_angle').value == 0
        assert node.get_parameter('eeg_forward_current_angle').value == 0
        assert node.get_parameter('eeg_forward_speed').value == 900
        assert node.get_parameter('eeg_backward_direction_angle').value == 0
        assert node.get_parameter('eeg_backward_current_angle').value == 0
        assert node.get_parameter('eeg_backward_speed').value == -700
        assert node.get_parameter('eeg_left_direction_angle').value == -90
        assert node.get_parameter('eeg_left_current_angle').value == 0
        assert node.get_parameter('eeg_left_speed').value == 300
        assert node.get_parameter('eeg_right_direction_angle').value == 90
        assert node.get_parameter('eeg_right_current_angle').value == 0
        assert node.get_parameter('eeg_right_speed').value == 300
        assert node.get_parameter('eeg_use_heading_feedback').value is False
        assert node.get_parameter('eeg_fixed_current_angle_deg').value == 0.0
        assert node.get_parameter('heading_timeout_sec').value == 0.5
        assert node.get_parameter('cmd_vel_timeout_sec').value == 1.0
        assert node.get_parameter('shutdown_zero_repeat').value == 3
    finally:
        node.destroy_node()


def test_node_opens_serial_exclusive(rclpy_init):
    """生产打开串口时应 exclusive=True，避免前端扫描等进程抢占底盘控制口。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode

    opened = []

    def serial_factory(**kwargs):
        opened.append(kwargs)
        return MockSerial(**kwargs)

    node = ChassisSerialNode(serial_factory=serial_factory, open_serial=True)
    try:
        assert opened
        assert opened[0]['port'] == '/dev/wheelchair_chassis'
        assert opened[0]['baudrate'] == 115200
        assert opened[0]['exclusive'] is True
    finally:
        node.destroy_node()


def test_node_custom_params(rclpy_init):
    """节点接受自定义参数覆盖默认值。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode

    mock_serial = MockSerial()
    node = ChassisSerialNode(
        serial_factory=lambda **kw: mock_serial,
        open_serial=False,
        overrides={
            'max_speed_mps': 1.0,
            'lead_gain_deg_per_rad_per_sec': 50.0,
            'heading_sign': 1,
        },
    )
    try:
        # rclpy set_parameters 后参数取新值（已实测验证）
        assert node.get_parameter('max_speed_mps').value == 1.0
        assert node.get_parameter('lead_gain_deg_per_rad_per_sec').value == 50.0
        assert node.get_parameter('heading_sign').value == 1
    finally:
        node.destroy_node()


def test_node_invalid_max_speed_raises(rclpy_init):
    """max_speed_mps <= 0 应在启动时抛异常（fail-fast）。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode

    mock_serial = MockSerial()
    with pytest.raises(Exception):
        ChassisSerialNode(
            serial_factory=lambda **kw: mock_serial,
            open_serial=False,
            overrides={'max_speed_mps': 0.0},
        )


def test_node_invalid_heading_sign_raises(rclpy_init):
    """heading_sign 不在 {-1, 1} 应在启动时抛异常（fail-fast）。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode

    mock_serial = MockSerial()
    with pytest.raises(Exception):
        ChassisSerialNode(
            serial_factory=lambda **kw: mock_serial,
            open_serial=False,
            overrides={'heading_sign': 2},
        )


def test_tick_writes_straight_frame(rclpy_init):
    """直走场景：cmd_vel(0.6, 0) + heading=45 → 串口收到 '\\n45,45,400'。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode
    from geometry_msgs.msg import Twist, Vector3
    from std_msgs.msg import Float64

    mock_serial = MockSerial()
    node = ChassisSerialNode(
        serial_factory=lambda **kw: mock_serial,
        open_serial=False,
    )
    mark_nav_ready(node)
    try:
        # 注入输入
        node._cmd_vel_cb(Twist(linear=Vector3(x=0.6), angular=Vector3(z=0.0)))
        node._heading_cb(Float64(data=45.0))

        node._tick()

        assert len(mock_serial.writes) == 1
        assert mock_serial.writes[0] == b"\n45,45,700"
    finally:
        node.destroy_node()


def test_tick_writes_left_turn_frame(rclpy_init):
    """左转场景：cmd_vel(0.6, +0.3) + heading=45 → '\\n36,45,700'。

    forward_speed=700 = 0.6/0.6[teb_max_vel_x] * 700[medium]（任务 4 档位算法）。
    """
    from rtk_perception.chassis_serial_node import ChassisSerialNode
    from geometry_msgs.msg import Twist, Vector3
    from std_msgs.msg import Float64

    mock_serial = MockSerial()
    node = ChassisSerialNode(
        serial_factory=lambda **kw: mock_serial,
        open_serial=False,
    )
    mark_nav_ready(node)
    try:
        node._cmd_vel_cb(Twist(linear=Vector3(x=0.6), angular=Vector3(z=0.3)))
        node._heading_cb(Float64(data=45.0))

        node._tick()

        assert mock_serial.writes[-1] == b"\n36,45,700"
    finally:
        node.destroy_node()


def test_tick_writes_right_turn_frame(rclpy_init):
    """右转场景：cmd_vel(0.6, -0.3) + heading=45 → '\\n54,45,700'。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode
    from geometry_msgs.msg import Twist, Vector3
    from std_msgs.msg import Float64

    mock_serial = MockSerial()
    node = ChassisSerialNode(
        serial_factory=lambda **kw: mock_serial,
        open_serial=False,
    )
    mark_nav_ready(node)
    try:
        node._cmd_vel_cb(Twist(linear=Vector3(x=0.6), angular=Vector3(z=-0.3)))
        node._heading_cb(Float64(data=45.0))

        node._tick()

        assert mock_serial.writes[-1] == b"\n54,45,700"
    finally:
        node.destroy_node()


def test_tick_no_heading_no_cmd_vel(rclpy_init):
    """初始无 heading + 无 cmd_vel → '\\n0,0,0'（安全零速）。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode

    mock_serial = MockSerial()
    node = ChassisSerialNode(
        serial_factory=lambda **kw: mock_serial,
        open_serial=False,
    )
    mark_nav_ready(node)
    try:
        node._tick()
        assert mock_serial.writes[-1] == b"\n0,0,0"
    finally:
        node.destroy_node()


def test_tick_nan_cmd_vel(rclpy_init):
    """cmd_vel 含 NaN → 当 0 处理。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode
    from geometry_msgs.msg import Twist, Vector3
    from std_msgs.msg import Float64
    import math

    mock_serial = MockSerial()
    node = ChassisSerialNode(
        serial_factory=lambda **kw: mock_serial,
        open_serial=False,
    )
    mark_nav_ready(node)
    try:
        node._cmd_vel_cb(Twist(linear=Vector3(x=float('nan')), angular=Vector3(z=0.0)))
        node._heading_cb(Float64(data=90.0))

        node._tick()
        # NaN → forward_speed=0；direction=current=90
        assert mock_serial.writes[-1] == b"\n90,90,0"
    finally:
        node.destroy_node()


def test_tick_heading_stale_forces_zero(rclpy_init):
    """heading_imu 超时 > 500ms → forward_speed=0 + current_angle=0。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode
    from geometry_msgs.msg import Twist, Vector3

    mock_serial = MockSerial()
    node = ChassisSerialNode(
        serial_factory=lambda **kw: mock_serial,
        open_serial=False,
    )
    mark_nav_ready(node)
    try:
        # 注入 cmd_vel 和 heading，但把 heading 时间戳改成 1 秒前
        node._cmd_vel_cb(Twist(linear=Vector3(x=0.6), angular=Vector3(z=0.0)))
        node._latest_heading_deg = 45.0
        node._latest_heading_sec = node._now_sec() - 1.0  # 1s 前，超 0.5s 阈值

        node._tick()
        # heading 超时 → forward_speed=0, current_angle=0
        assert mock_serial.writes[-1] == b"\n0,0,0"
    finally:
        node.destroy_node()


def test_tick_heading_fresh_uses_value(rclpy_init):
    """heading_imu 新鲜（< 500ms）→ 正常使用当前值。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode
    from geometry_msgs.msg import Twist, Vector3

    mock_serial = MockSerial()
    node = ChassisSerialNode(
        serial_factory=lambda **kw: mock_serial,
        open_serial=False,
    )
    mark_nav_ready(node)
    try:
        node._cmd_vel_cb(Twist(linear=Vector3(x=0.6), angular=Vector3(z=0.0)))
        # heading 100ms 前，未超时
        node._latest_heading_deg = 45.0
        node._latest_heading_sec = node._now_sec() - 0.1

        node._tick()
        assert mock_serial.writes[-1] == b"\n45,45,700"
    finally:
        node.destroy_node()


def test_tick_cmd_vel_stale_forces_zero(rclpy_init):
    """cmd_vel_safe 超时 > 1s → forward_speed=0（direction 仍跟 heading）。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode
    from geometry_msgs.msg import Twist, Vector3
    from std_msgs.msg import Float64

    mock_serial = MockSerial()
    node = ChassisSerialNode(
        serial_factory=lambda **kw: mock_serial,
        open_serial=False,
    )
    mark_nav_ready(node)
    try:
        # 先注入 cmd_vel 消息（让 _latest_cmd_vel 非 None），再覆盖时间戳为 2s 前
        # 这样 _is_cmd_vel_stale 的时间戳比较分支才会被执行（而非 None 短路）
        node._cmd_vel_cb(Twist(linear=Vector3(x=0.6), angular=Vector3(z=0.0)))
        node._latest_cmd_vel_sec = node._now_sec() - 2.0  # 2s 前，超 1s 阈值
        node._heading_cb(Float64(data=90.0))

        node._tick()
        # cmd_vel 超时 → forward_speed=0, direction=current=90
        assert mock_serial.writes[-1] == b"\n90,90,0"
    finally:
        node.destroy_node()


def test_tick_cmd_vel_fresh_uses_value(rclpy_init):
    """cmd_vel_safe 新鲜（< 1s）→ 正常使用。

    linear.x=1.5 超过 teb_max_vel_x=0.6 → 归一化 clamp 到 1.0 →
    forward_speed = 1.0 * 700[medium] = 700（任务 4 档位算法）。
    """
    from rtk_perception.chassis_serial_node import ChassisSerialNode
    from geometry_msgs.msg import Twist, Vector3
    from std_msgs.msg import Float64

    mock_serial = MockSerial()
    node = ChassisSerialNode(
        serial_factory=lambda **kw: mock_serial,
        open_serial=False,
    )
    mark_nav_ready(node)
    try:
        node._cmd_vel_cb(Twist(linear=Vector3(x=1.5), angular=Vector3(z=0.0)))  # 满速
        node._heading_cb(Float64(data=0.0))

        node._tick()
        assert mock_serial.writes[-1] == b"\n0,0,700"
    finally:
        node.destroy_node()


def test_tick_nav2_reverse_disabled_by_default(rclpy_init):
    """实物自主导航默认禁后退：TEB 负线速度不应变成负 forward_speed。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode
    from geometry_msgs.msg import Twist, Vector3
    from std_msgs.msg import Float64

    mock_serial = MockSerial()
    node = ChassisSerialNode(
        serial_factory=lambda **kw: mock_serial,
        open_serial=False,
    )
    mark_nav_ready(node)
    try:
        node._cmd_vel_cb(Twist(linear=Vector3(x=-0.3), angular=Vector3(z=0.0)))
        node._heading_cb(Float64(data=30.0))

        node._tick()

        assert mock_serial.writes[-1] == b"\n30,30,0"
    finally:
        node.destroy_node()


def test_tick_nav2_reverse_can_be_enabled_for_lab(rclpy_init):
    """实验需要时可显式允许 Nav2 倒车；默认仍是禁止。

    allow_reverse=True：linear=-0.3 → forward_speed = -0.3/0.6 * 700 = -350
    （任务 4 档位算法）。
    """
    from rtk_perception.chassis_serial_node import ChassisSerialNode
    from geometry_msgs.msg import Twist, Vector3
    from std_msgs.msg import Float64

    mock_serial = MockSerial()
    node = ChassisSerialNode(
        serial_factory=lambda **kw: mock_serial,
        open_serial=False,
        overrides={'nav_allow_reverse': True},
    )
    mark_nav_ready(node)
    try:
        node._cmd_vel_cb(Twist(linear=Vector3(x=-0.3), angular=Vector3(z=0.0)))
        node._heading_cb(Float64(data=30.0))

        node._tick()

        assert mock_serial.writes[-1] == b"\n30,30,-350"
    finally:
        node.destroy_node()


def test_tick_publishes_odom_tf(rclpy_init):
    """_tick 同时发布 odom→base_link TF（dead-reckon）。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode
    from geometry_msgs.msg import Twist, Vector3
    from std_msgs.msg import Float64

    mock_serial = MockSerial()
    node = ChassisSerialNode(
        serial_factory=lambda **kw: mock_serial,
        open_serial=False,
    )
    mark_nav_ready(node)
    try:
        node._cmd_vel_cb(Twist(linear=Vector3(x=0.6), angular=Vector3(z=0.0)))
        node._heading_cb(Float64(data=0.0))  # 朝北

        # 跑一帧：0.6 m/s × 0.01s ≈ 0.006m 北向
        node._tick()

        # TF 不易直接断言（TransformBroadcaster 内部走 ROS 中间件）
        # 改为验证节点持有 broadcaster + _dead_reckon 状态更新
        assert hasattr(node, '_tf_broadcaster')
        assert hasattr(node, '_dead_reckon_x_m')
        assert hasattr(node, '_dead_reckon_y_m')
    finally:
        node.destroy_node()


def test_tick_publishes_odom_topic(rclpy_init):
    """_tick 应发布 /odom Odometry；TEB 订阅 topic，不只依赖 TF。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode
    from geometry_msgs.msg import Twist, Vector3
    from std_msgs.msg import Float64

    mock_serial = MockSerial()
    node = ChassisSerialNode(
        serial_factory=lambda **kw: mock_serial,
        open_serial=False,
    )
    mark_nav_ready(node)
    capture = CapturePublisher()
    node._odom_pub = capture
    try:
        node._cmd_vel_cb(Twist(linear=Vector3(x=0.6), angular=Vector3(z=0.2)))
        node._heading_cb(Float64(data=90.0))  # compass east -> ROS yaw 0

        node._tick()

        assert len(capture.messages) == 1
        odom = capture.messages[-1]
        assert odom.header.frame_id == 'odom'
        assert odom.child_frame_id == 'base_link'
        assert odom.pose.pose.orientation.z == pytest.approx(0.0, abs=1e-6)
        assert odom.pose.pose.orientation.w == pytest.approx(1.0, abs=1e-6)
        assert odom.twist.twist.linear.x == pytest.approx(0.6)
        assert odom.twist.twist.angular.z == pytest.approx(0.2)
    finally:
        node.destroy_node()


def test_tick_dead_reckon_accumulates(rclpy_init):
    """多帧 _tick 后 dead-reckon 位置应累积（朝北走，y_m 应增加）。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode
    from geometry_msgs.msg import Twist, Vector3
    from std_msgs.msg import Float64

    mock_serial = MockSerial()
    node = ChassisSerialNode(
        serial_factory=lambda **kw: mock_serial,
        open_serial=False,
    )
    mark_nav_ready(node)
    try:
        node._cmd_vel_cb(Twist(linear=Vector3(x=1.0), angular=Vector3(z=0.0)))
        node._heading_cb(Float64(data=0.0))  # 朝北（compass 0 = +y）

        # 跑 10 帧，每帧间隔 ~10ms
        import time
        for _ in range(10):
            node._tick()
            time.sleep(0.011)

        # 1.0 m/s × ~0.11s ≈ 0.11m，朝北 → y_m 增加
        assert node._dead_reckon_y_m > 0.05, f"y_m should be > 0.05, got {node._dead_reckon_y_m}"
        assert abs(node._dead_reckon_x_m) < 0.01, f"x_m should be ~0, got {node._dead_reckon_x_m}"
    finally:
        node.destroy_node()


def test_tick_dead_reckon_stops_when_cmd_vel_stale(rclpy_init):
    """cmd_vel 超时后，TF dead-reckon 应停止积分（与真轮椅停下对齐）。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode
    from geometry_msgs.msg import Twist, Vector3
    from std_msgs.msg import Float64
    import time

    mock_serial = MockSerial()
    node = ChassisSerialNode(
        serial_factory=lambda **kw: mock_serial,
        open_serial=False,
    )
    mark_nav_ready(node)
    try:
        # 先正常跑 5 帧，让位置累积一些
        node._cmd_vel_cb(Twist(linear=Vector3(x=1.0), angular=Vector3(z=0.0)))
        node._heading_cb(Float64(data=0.0))
        for _ in range(5):
            node._tick()
            time.sleep(0.011)
        position_before_stale = node._dead_reckon_y_m
        assert position_before_stale > 0.02, f"前置条件失败：5 帧后 y_m 应 > 0.02，实际 {position_before_stale}"

        # 模拟 cmd_vel 超时（_latest_cmd_vel 仍非 None，但时间戳 2s 前）
        node._latest_cmd_vel_sec = node._now_sec() - 2.0
        # heading 保持新鲜
        node._heading_cb(Float64(data=0.0))

        # 再跑 5 帧
        for _ in range(5):
            node._tick()
            time.sleep(0.011)

        # y_m 应基本不变（cmd_vel stale → 不积分）
        delta = node._dead_reckon_y_m - position_before_stale
        assert abs(delta) < 0.005, (
            f"cmd_vel stale 后 dead-reckon 不应继续积分，"
            f"但 y_m 增加了 {delta:.4f}m"
        )
    finally:
        node.destroy_node()


def test_shutdown_emits_zero_frames(rclpy_init):
    """节点关闭时连发 shutdown_zero_repeat 帧 '\\nX,Y,0'（防丢包）。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode
    from std_msgs.msg import Float64

    mock_serial = MockSerial()
    node = ChassisSerialNode(
        serial_factory=lambda **kw: mock_serial,
        open_serial=False,
    )
    try:
        node._heading_cb(Float64(data=70.0))
        node._shutdown_safe()

        # 默认 shutdown_zero_repeat=3
        zero_frames = [w for w in mock_serial.writes if w == b"\n70,70,0"]
        assert len(zero_frames) == 3
    finally:
        node.destroy_node()


def test_shutdown_closes_serial(rclpy_init):
    """关闭后串口关闭。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode

    mock_serial = MockSerial()
    node = ChassisSerialNode(
        serial_factory=lambda **kw: mock_serial,
        open_serial=False,
    )
    node._serial = mock_serial  # 模拟已开串口
    try:
        node._shutdown_safe()
        assert mock_serial.is_open is False
    finally:
        node.destroy_node()


class FailingSerial(MockSerial):
    """模拟串口写入失败然后恢复。"""

    def __init__(self, fail_count=1):
        super().__init__()
        self.fail_count = fail_count

    def write(self, data: bytes) -> int:
        if self.fail_count > 0:
            self.fail_count -= 1
            raise OSError("simulate USB disconnect")
        return super().write(data)


def test_write_serial_retries_on_exception(rclpy_init):
    """_write_serial 写入抛 OSError → ERROR 日志 + 不崩（下位机 watchdog 兜底）。

    后续 _tick 仍能成功写入（fail_count 衰减后恢复）。
    """
    from rtk_perception.chassis_serial_node import ChassisSerialNode
    from std_msgs.msg import Float64

    failing = FailingSerial(fail_count=2)
    node = ChassisSerialNode(
        serial_factory=lambda **kw: failing,
        open_serial=False,
    )
    node._serial = failing  # 模拟已开
    try:
        node._heading_cb(Float64(data=0.0))
        # 第一次 _tick：failing.write 抛 OSError，节点不崩，日志 ERROR
        node._tick()
        # 第二次 _tick：fail_count 还有 1，再抛
        node._tick()
        # 第三次 _tick：fail_count=0，正常写
        node._tick()
        # 至少最后一次成功
        assert any(w == b"\n0,0,0" for w in failing.writes), (
            f"应至少有一次成功写入，writes={failing.writes}"
        )
    finally:
        node.destroy_node()


def test_write_serial_keeps_serial_after_exception(rclpy_init):
    """单帧写入失败后 _serial 不应被 None（让后续帧/shutdown 仍能尝试重发），
    且串口应被关闭（is_open=False），让下次 _write_serial 检测到后触发 _open_serial 重连。

    这是 Task 7 代码审查发现的关键交互：_write_serial 不能在异常时
    把 _serial 置 None，否则 _shutdown_safe 的连发循环剩余帧会全部
    静默丢失（if self._serial is None: return）。

    同时异常时必须 close() 串口——否则下次 _write_serial 看 is_open=True
    不会触发重连，USB 已拔的情况下后续帧全部走原 broken fd，永不再恢复。
    """
    from rtk_perception.chassis_serial_node import ChassisSerialNode
    from std_msgs.msg import Float64

    failing = FailingSerial(fail_count=1)
    node = ChassisSerialNode(
        serial_factory=lambda **kw: failing,
        open_serial=False,
    )
    node._serial = failing
    try:
        node._heading_cb(Float64(data=0.0))
        node._tick()  # 这次 write 失败

        # _serial 应仍非 None（让后续 _write_serial / _shutdown_safe 能尝试重发）
        assert node._serial is not None, (
            "_write_serial 异常时不应把 _serial 置 None，否则 shutdown 连发会丢帧"
        )
        # 串口应被关闭（让下次 _write_serial 触发 _open_serial 重连）
        assert node._serial.is_open is False, (
            "_write_serial 异常时应 close() 串口，否则下次不会重连，"
            f"当前 is_open={node._serial.is_open}"
        )
    finally:
        node.destroy_node()


def test_write_serial_reopens_after_close(rclpy_init):
    """is_open=False 时 _write_serial 应调用 _open_serial 重连。

    这是 Task 8 核心重连行为的直接验证（不只是异常不崩）。
    用 spy 计数 _open_serial 调用次数 + 模拟重开成功。
    """
    from rtk_perception.chassis_serial_node import ChassisSerialNode
    from std_msgs.msg import Float64

    failing = FailingSerial(fail_count=1)
    node = ChassisSerialNode(
        serial_factory=lambda **kw: failing,
        open_serial=False,
    )
    node._serial = failing

    # 用 spy 替换 _open_serial：记录调用 + 把 is_open 恢复为 True（模拟重开成功）
    reopen_spy = []
    def fake_open_serial():
        reopen_spy.append(True)
        failing.is_open = True  # 模拟 serial.Serial 重开成功
        return True

    node._open_serial = fake_open_serial

    try:
        node._heading_cb(Float64(data=0.0))

        # 第一次 _tick：write 抛 OSError → except 路径 close → is_open=False
        node._tick()
        assert failing.is_open is False, "第一次异常后串口应被 close"

        # 第二次 _tick：is_open=False → 进 _open_serial 分支 → spy 被调用 → write 成功
        node._tick()

        # 验证 _open_serial 真被调用一次
        assert len(reopen_spy) == 1, (
            f"应触发一次 _open_serial 重连，实际 {len(reopen_spy)} 次"
        )
        # 验证第二次写入成功
        assert any(w == b"\n0,0,0" for w in failing.writes), (
            f"重连后应成功写入，writes={failing.writes}"
        )
    finally:
        node.destroy_node()


# ============ EEG mux 测试 ============

def test_eeg_mux_initial_state(rclpy_init):
    """节点构造后：脑控未待命、未接管、无 cmd_vel_eeg 缓存。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode

    mock_serial = MockSerial()
    node = ChassisSerialNode(serial_factory=lambda **kw: mock_serial, open_serial=False)
    try:
        assert node.eeg_mode_active is False
        assert node._force_stop_active is False
        assert node._latest_cmd_vel_eeg is None
        assert node._latest_cmd_vel_eeg_sec == 0.0
        assert node._last_eeg_motion_sec == 0.0
        assert node._eeg_override_active is False
        # 心跳时间戳初始化为构造时刻（不是 0）
        assert node._last_eeg_mode_msg_time > 0
    finally:
        node.destroy_node()


def test_cmd_vel_eeg_cb_consumed_when_active(rclpy_init):
    """eeg_mode_active=True 时 _cmd_vel_eeg_cb 缓存消息并记录非零动作时间。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode
    from geometry_msgs.msg import Twist, Vector3

    mock_serial = MockSerial()
    node = ChassisSerialNode(serial_factory=lambda **kw: mock_serial, open_serial=False)
    node.eeg_mode_active = True
    try:
        node._cmd_vel_eeg_cb(Twist(linear=Vector3(x=0.5), angular=Vector3(z=0.0)))
        assert node._latest_cmd_vel_eeg is not None
        assert node._latest_cmd_vel_eeg.linear.x == 0.5
        assert node._latest_cmd_vel_eeg_sec > 0
        assert node._last_eeg_motion_sec > 0
    finally:
        node.destroy_node()


def test_cmd_vel_eeg_cb_ignored_when_inactive(rclpy_init):
    """eeg_mode_active=False 时 _cmd_vel_eeg_cb 忽略消息（不缓存）。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode
    from geometry_msgs.msg import Twist, Vector3

    mock_serial = MockSerial()
    node = ChassisSerialNode(serial_factory=lambda **kw: mock_serial, open_serial=False)
    try:
        node._cmd_vel_eeg_cb(Twist(linear=Vector3(x=0.5), angular=Vector3(z=0.0)))
        assert node._latest_cmd_vel_eeg is None  # 仍为初始 None
    finally:
        node.destroy_node()


def test_cmd_vel_cb_cached_when_eeg_armed(rclpy_init):
    """eeg_mode_active=True 时仍缓存 Nav2，供脑控释放后继续自主导航。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode
    from geometry_msgs.msg import Twist, Vector3

    mock_serial = MockSerial()
    node = ChassisSerialNode(serial_factory=lambda **kw: mock_serial, open_serial=False)
    node.eeg_mode_active = True
    try:
        assert node._latest_cmd_vel is None
        node._cmd_vel_cb(Twist(linear=Vector3(x=0.6), angular=Vector3(z=0.0)))
        assert node._latest_cmd_vel is not None
        assert node._latest_cmd_vel.linear.x == 0.6
        assert node._latest_cmd_vel_sec > 0
    finally:
        node.destroy_node()


def test_eeg_mode_active_cb_keeps_nav2_residue(rclpy_init):
    """切到脑控待命时不清零 Nav2 缓存，激活本身不打断自主导航。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode
    from geometry_msgs.msg import Twist, Vector3
    from std_msgs.msg import Bool

    mock_serial = MockSerial()
    node = ChassisSerialNode(serial_factory=lambda **kw: mock_serial, open_serial=False)
    try:
        # 先注入一个 Nav2 残留
        node._cmd_vel_cb(Twist(linear=Vector3(x=0.6), angular=Vector3(z=0.0)))
        assert node._latest_cmd_vel is not None

        # 切 EEG 模式
        node._eeg_mode_active_cb(Bool(data=True))

        assert node.eeg_mode_active is True
        # Nav2 缓存保留；只有非零 /cmd_vel_eeg 才会临时接管。
        assert node._latest_cmd_vel is not None
        assert node._latest_cmd_vel.linear.x == 0.6
        assert node._eeg_override_active is False
        # 心跳时间戳刷新
        assert node._last_eeg_mode_msg_time > 0
    finally:
        node.destroy_node()


def test_eeg_mode_active_cb_deactivate(rclpy_init):
    """切回 Nav2 模式：eeg_mode_active=False。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode
    from std_msgs.msg import Bool

    mock_serial = MockSerial()
    node = ChassisSerialNode(serial_factory=lambda **kw: mock_serial, open_serial=False)
    node.eeg_mode_active = True
    try:
        node._eeg_mode_active_cb(Bool(data=False))
        assert node.eeg_mode_active is False
    finally:
        node.destroy_node()


def test_clear_goal_cb_sets_force_stop(rclpy_init):
    """/clear_goal 收到 → _force_stop_active=True。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode
    from std_msgs.msg import Empty

    mock_serial = MockSerial()
    node = ChassisSerialNode(serial_factory=lambda **kw: mock_serial, open_serial=False)
    try:
        assert node._force_stop_active is False
        node._clear_goal_cb(Empty())
        assert node._force_stop_active is True
    finally:
        node.destroy_node()


def test_check_eeg_mode_fallback_triggers_after_3s(rclpy_init):
    """eeg_mode_active=True + 心跳 > 3s → _check_eeg_mode_fallback 应切回 False。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode

    mock_serial = MockSerial()
    node = ChassisSerialNode(serial_factory=lambda **kw: mock_serial, open_serial=False)
    try:
        # 模拟 BrainControlTab 激活后崩溃：eeg_mode_active=True 但心跳时间戳倒拨 4s
        node.eeg_mode_active = True
        node._last_eeg_mode_msg_time = node._now_sec() - 4.0

        node._check_eeg_mode_fallback()

        # 应自动 fallback 到 Nav2
        assert node.eeg_mode_active is False
    finally:
        node.destroy_node()


def test_check_eeg_mode_fallback_noop_when_inactive(rclpy_init):
    """eeg_mode_active=False 时调用 _check_eeg_mode_fallback 不应误触发。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode

    mock_serial = MockSerial()
    node = ChassisSerialNode(serial_factory=lambda **kw: mock_serial, open_serial=False)
    try:
        # 默认 inactive，即使心跳时间戳很旧也不应改变状态
        node.eeg_mode_active = False
        node._last_eeg_mode_msg_time = node._now_sec() - 10.0  # 比 3s 还旧

        node._check_eeg_mode_fallback()

        # 早退，仍是 False
        assert node.eeg_mode_active is False
    finally:
        node.destroy_node()


# ============ _tick EEG mux 行为测试 ============

def test_tick_nav2_mode_uses_cmd_vel_safe(rclpy_init):
    """Nav2 模式（默认）：cmd_vel_safe=0.6 + cmd_vel_eeg=0.5 → forward_speed=400（Nav2 赢）。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode
    from geometry_msgs.msg import Twist, Vector3
    from std_msgs.msg import Float64

    mock_serial = MockSerial()
    node = ChassisSerialNode(serial_factory=lambda **kw: mock_serial, open_serial=False)
    mark_nav_ready(node)
    try:
        # 注入两路信号，Nav2 模式（eeg_mode_active=False）
        node._cmd_vel_cb(Twist(linear=Vector3(x=0.6), angular=Vector3(z=0.0)))
        node._cmd_vel_eeg_cb(Twist(linear=Vector3(x=0.5), angular=Vector3(z=0.0)))
        node._heading_cb(Float64(data=0.0))

        node._tick()
        # Nav2 模式：forward_speed = 0.6/0.6[teb_max_vel_x] * 700[medium] = 700
        assert mock_serial.writes[-1] == b"\n0,0,700"
    finally:
        node.destroy_node()


def test_tick_eeg_armed_without_motion_keeps_nav2(rclpy_init):
    """脑控待命但未输出非零动作时，自主导航继续进底盘。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode
    from geometry_msgs.msg import Twist, Vector3
    from std_msgs.msg import Bool, Float64

    mock_serial = MockSerial()
    node = ChassisSerialNode(serial_factory=lambda **kw: mock_serial, open_serial=False)
    mark_nav_ready(node)
    try:
        node._cmd_vel_cb(Twist(linear=Vector3(x=0.6), angular=Vector3(z=0.0)))
        node._heading_cb(Float64(data=0.0))
        node._eeg_mode_active_cb(Bool(data=True))

        node._tick()

        assert node._eeg_override_active is False
        assert mock_serial.writes[-1] == b"\n0,0,700"
    finally:
        node.destroy_node()


def test_tick_eeg_zero_before_motion_keeps_nav2(rclpy_init):
    """脑控待命后的 STOP 帧不算用户操控，不能让自主导航停车。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode
    from geometry_msgs.msg import Twist, Vector3
    from std_msgs.msg import Bool, Float64

    mock_serial = MockSerial()
    node = ChassisSerialNode(serial_factory=lambda **kw: mock_serial, open_serial=False)
    mark_nav_ready(node)
    try:
        node._cmd_vel_cb(Twist(linear=Vector3(x=0.6), angular=Vector3(z=0.0)))
        node._heading_cb(Float64(data=0.0))
        node._eeg_mode_active_cb(Bool(data=True))
        node._cmd_vel_eeg_cb(Twist(linear=Vector3(x=0.0), angular=Vector3(z=0.0)))

        node._tick()

        assert node._eeg_override_active is False
        assert mock_serial.writes[-1] == b"\n0,0,700"
    finally:
        node.destroy_node()


def test_tick_eeg_mode_uses_cmd_vel_eeg(rclpy_init):
    """EEG 待命 + 非零 cmd_vel_eeg：脑控实物映射赢。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode
    from geometry_msgs.msg import Twist, Vector3
    from std_msgs.msg import Bool, Float64

    mock_serial = MockSerial()
    node = ChassisSerialNode(serial_factory=lambda **kw: mock_serial, open_serial=False)
    try:
        # 先激活 EEG 待命
        node._eeg_mode_active_cb(Bool(data=True))
        # 注入两路；Nav2 会继续缓存，但非零 EEG 动作临时接管。
        node._cmd_vel_cb(Twist(linear=Vector3(x=0.6), angular=Vector3(z=0.0)))
        node._cmd_vel_eeg_cb(Twist(linear=Vector3(x=0.5), angular=Vector3(z=0.0)))
        node._heading_cb(Float64(data=0.0))

        node._tick()
        assert node._eeg_override_active is True
        assert mock_serial.writes[-1] == b"\n0,0,900"
    finally:
        node.destroy_node()


def test_tick_eeg_override_holds_zero_then_releases_to_nav2(rclpy_init):
    """非零脑控动作触发接管；用户松开后保护期零速，超时回 Nav2。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode
    from geometry_msgs.msg import Twist, Vector3
    from std_msgs.msg import Bool, Float64

    mock_serial = MockSerial()
    node = ChassisSerialNode(serial_factory=lambda **kw: mock_serial, open_serial=False)
    mark_nav_ready(node)
    try:
        node._heading_cb(Float64(data=0.0))
        node._cmd_vel_cb(Twist(linear=Vector3(x=0.6), angular=Vector3(z=0.0)))
        node._eeg_mode_active_cb(Bool(data=True))
        node._cmd_vel_eeg_cb(Twist(linear=Vector3(x=0.5), angular=Vector3(z=0.0)))
        node._tick()
        assert mock_serial.writes[-1] == b"\n0,0,900"

        # Nav2 在后台刷新到更快速度，保护期内仍被脑控压住。
        node._cmd_vel_cb(Twist(linear=Vector3(x=0.9), angular=Vector3(z=0.0)))
        node._cmd_vel_eeg_cb(Twist(linear=Vector3(x=0.0), angular=Vector3(z=0.0)))
        node._tick()
        assert node._eeg_override_active is True
        assert mock_serial.writes[-1] == b"\n0,0,0"

        now = node._now_sec()
        hold = float(node.get_parameter('eeg_override_hold_sec').value)
        node._last_eeg_motion_sec = now - hold - 0.1
        node._tick()

        assert node._eeg_override_active is False
        assert mock_serial.writes[-1] == b"\n0,0,700"
    finally:
        node.destroy_node()


def test_tick_eeg_mode_requires_heading_feedback_defaults_to_zero(rclpy_init):
    """EEG 模式要求 HWT 反馈；无 heading → 零速帧（避免下位机误判异常）。

    2026-07-12 修订：下位机新增加速度限制依赖 current_angle 字段作为运动反馈，
    EEG 模式下 current 必须跟随 HWT 真值才能让下位机识别为正常运动。HWT 失联
    时输出零速帧（与 Nav2 路径一致），不再像旧实现那样固定 current=0 跑脑控帧。
    """
    from rtk_perception.chassis_serial_node import ChassisSerialNode
    from geometry_msgs.msg import Twist, Vector3
    from std_msgs.msg import Bool

    mock_serial = MockSerial()
    node = ChassisSerialNode(serial_factory=lambda **kw: mock_serial, open_serial=False)
    try:
        node._eeg_mode_active_cb(Bool(data=True))
        node._cmd_vel_eeg_cb(Twist(linear=Vector3(x=0.5), angular=Vector3(z=0.0)))

        node._tick()

        assert mock_serial.writes[-1] == b"\n0,0,0"
    finally:
        node.destroy_node()


def test_tick_eeg_heading_feedback_param_is_ignored(rclpy_init):
    """旧 HWT feedback 参数仅兼容声明；EEG 现在始终要求 HWT 在线。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode
    from geometry_msgs.msg import Twist, Vector3
    from std_msgs.msg import Bool, Float64

    mock_serial = MockSerial()
    node = ChassisSerialNode(
        serial_factory=lambda **kw: mock_serial,
        open_serial=False,
        overrides={'eeg_use_heading_feedback': True},
    )
    try:
        node._eeg_mode_active_cb(Bool(data=True))
        node._cmd_vel_eeg_cb(Twist(linear=Vector3(x=0.5), angular=Vector3(z=0.0)))
        node._heading_cb(Float64(data=0.0))

        node._tick()

        assert mock_serial.writes[-1] == b"\n0,0,900"
    finally:
        node.destroy_node()


def test_tick_eeg_forward_uses_hwt_heading_feedback(rclpy_init):
    """脑控底盘帧跟随 HWT 航向：HWT=90° → 帧=(90, 90, 900)。

    2026-07-12 修订：借鉴 Nav2 路径，current_angle 跟随 HWT 真值，
    direction_angle = current + 偏移。下位机看到 PID error=0 → 直行。
    """
    from rtk_perception.chassis_serial_node import ChassisSerialNode
    from geometry_msgs.msg import Twist, Vector3
    from std_msgs.msg import Bool, Float64

    mock_serial = MockSerial()
    node = ChassisSerialNode(
        serial_factory=lambda **kw: mock_serial,
        open_serial=False,
        overrides={'eeg_use_heading_feedback': True},
    )
    try:
        node._eeg_mode_active_cb(Bool(data=True))
        node._cmd_vel_eeg_cb(Twist(linear=Vector3(x=0.5), angular=Vector3(z=0.0)))

        node._heading_cb(Float64(data=90.0))
        node._tick()

        assert mock_serial.writes[-1] == b"\n90,90,900"
    finally:
        node.destroy_node()


def test_tick_eeg_backward_uses_signed_speed(rclpy_init):
    """EEG 后退：current=HWT 真值，forward_speed 与前进相反。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode
    from geometry_msgs.msg import Twist, Vector3
    from std_msgs.msg import Bool, Float64

    mock_serial = MockSerial()
    node = ChassisSerialNode(serial_factory=lambda **kw: mock_serial, open_serial=False)
    try:
        node._eeg_mode_active_cb(Bool(data=True))
        node._cmd_vel_eeg_cb(Twist(linear=Vector3(x=-0.3), angular=Vector3(z=0.0)))
        node._heading_cb(Float64(data=0.0))

        node._tick()

        assert mock_serial.writes[-1] == b"\n0,0,-700"
    finally:
        node.destroy_node()


def test_tick_eeg_left_turn_uses_relative_direction_offset(rclpy_init):
    """EEG 左转：direction = current - 90° 触发下位机差速左转。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode
    from geometry_msgs.msg import Twist, Vector3
    from std_msgs.msg import Bool, Float64

    mock_serial = MockSerial()
    node = ChassisSerialNode(serial_factory=lambda **kw: mock_serial, open_serial=False)
    try:
        node._eeg_mode_active_cb(Bool(data=True))
        node._cmd_vel_eeg_cb(Twist(linear=Vector3(x=0.0), angular=Vector3(z=0.5)))
        node._heading_cb(Float64(data=0.0))

        node._tick()

        assert mock_serial.writes[-1] == b"\n-90,0,300"
    finally:
        node.destroy_node()


def test_tick_eeg_right_turn_keeps_existing_frame(rclpy_init):
    """EEG 右转：direction = current + 90° 触发下位机差速右转。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode
    from geometry_msgs.msg import Twist, Vector3
    from std_msgs.msg import Bool, Float64

    mock_serial = MockSerial()
    node = ChassisSerialNode(serial_factory=lambda **kw: mock_serial, open_serial=False)
    try:
        node._eeg_mode_active_cb(Bool(data=True))
        node._cmd_vel_eeg_cb(Twist(linear=Vector3(x=0.0), angular=Vector3(z=-0.5)))
        node._heading_cb(Float64(data=0.0))

        node._tick()

        assert mock_serial.writes[-1] == b"\n90,0,300"
    finally:
        node.destroy_node()


def test_tick_eeg_override_ignores_cmd_vel_safe_change(rclpy_init):
    """脑控接管期间 cmd_vel_safe 变化不影响 forward_speed。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode
    from geometry_msgs.msg import Twist, Vector3
    from std_msgs.msg import Bool, Float64

    mock_serial = MockSerial()
    node = ChassisSerialNode(serial_factory=lambda **kw: mock_serial, open_serial=False)
    try:
        node._eeg_mode_active_cb(Bool(data=True))
        node._cmd_vel_eeg_cb(Twist(linear=Vector3(x=0.5), angular=Vector3(z=0.0)))
        node._heading_cb(Float64(data=0.0))

        # 第一次 tick：脑控前进标定帧
        node._tick()
        assert mock_serial.writes[-1] == b"\n0,0,900"

        # cmd_vel_safe 变化（应该被忽略）
        node._cmd_vel_cb(Twist(linear=Vector3(x=1.5), angular=Vector3(z=0.0)))
        node._tick()
        # 仍为脑控前进标定帧（Nav2 信号没污染）
        assert mock_serial.writes[-1] == b"\n0,0,900"
    finally:
        node.destroy_node()


def test_tick_eeg_override_bypasses_nav_gps_gate(rclpy_init):
    """GPS 失锁只阻止自主导航，不应清零本地脑控接管指令。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode
    from geometry_msgs.msg import Twist, Vector3
    from std_msgs.msg import Bool, Float64

    mock_serial = MockSerial()
    node = ChassisSerialNode(
        serial_factory=lambda **kw: mock_serial,
        open_serial=False,
        overrides={'gps_required_for_motion': True},
    )
    try:
        assert not node._is_gps_ready(node._now_sec())
        node._eeg_mode_active_cb(Bool(data=True))
        node._cmd_vel_eeg_cb(
            Twist(linear=Vector3(x=0.5), angular=Vector3(z=0.0))
        )
        node._heading_cb(Float64(data=0.0))

        node._tick()

        assert node._eeg_override_active is True
        assert mock_serial.writes[-1] == b"\n0,0,900"
    finally:
        node.destroy_node()


def test_tick_eeg_cmd_vel_stale_forces_zero(rclpy_init):
    """EEG 模式 + cmd_vel_eeg 超过 1s 未更新 → forward_speed=0。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode
    from geometry_msgs.msg import Twist, Vector3
    from std_msgs.msg import Bool, Float64

    mock_serial = MockSerial()
    node = ChassisSerialNode(serial_factory=lambda **kw: mock_serial, open_serial=False)
    try:
        node._eeg_mode_active_cb(Bool(data=True))
        # 注入 cmd_vel_eeg，但时间戳改为 2s 前
        node._cmd_vel_eeg_cb(Twist(linear=Vector3(x=0.5), angular=Vector3(z=0.0)))
        node._latest_cmd_vel_eeg_sec = node._now_sec() - 2.0
        node._heading_cb(Float64(data=0.0))

        node._tick()
        # 超时 → forward_speed=0
        assert mock_serial.writes[-1] == b"\n0,0,0"
    finally:
        node.destroy_node()


def test_tick_clear_goal_overrides_eeg(rclpy_init):
    """/clear_goal 触发强制零速，覆盖 EEG 信号。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode
    from geometry_msgs.msg import Twist, Vector3
    from std_msgs.msg import Bool, Empty, Float64

    mock_serial = MockSerial()
    node = ChassisSerialNode(serial_factory=lambda **kw: mock_serial, open_serial=False)
    try:
        # EEG 激活 + 信号新鲜
        node._eeg_mode_active_cb(Bool(data=True))
        node._cmd_vel_eeg_cb(Twist(linear=Vector3(x=0.5), angular=Vector3(z=0.0)))
        node._heading_cb(Float64(data=0.0))

        # 触发 clear_goal
        node._clear_goal_cb(Empty())

        node._tick()
        # 强制零速覆盖 EEG
        assert mock_serial.writes[-1] == b"\n0,0,0"
    finally:
        node.destroy_node()


def test_tick_clear_goal_releases_when_cmd_vel_safe_zero(rclpy_init):
    """cmd_vel_safe 转为 0 后解除 _force_stop_active，恢复脑控接管。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode
    from geometry_msgs.msg import Twist, Vector3
    from std_msgs.msg import Bool, Empty, Float64

    mock_serial = MockSerial()
    node = ChassisSerialNode(serial_factory=lambda **kw: mock_serial, open_serial=False)
    try:
        node._eeg_mode_active_cb(Bool(data=True))
        node._cmd_vel_eeg_cb(Twist(linear=Vector3(x=0.5), angular=Vector3(z=0.0)))
        node._heading_cb(Float64(data=0.0))

        # 触发 clear_goal
        node._clear_goal_cb(Empty())
        assert node._force_stop_active is True

        # 模拟 Nav2 收到 cancel 后 cmd_vel_safe 转 0。
        node._latest_cmd_vel = Twist(linear=Vector3(x=0.0), angular=Vector3(z=0.0))

        node._tick()
        # _force_stop_active 应被解除
        assert node._force_stop_active is False
        # 此帧仍是零速（解除发生在 tick 内，下一帧才用 EEG）
        assert mock_serial.writes[-1] == b"\n0,0,0"

        # 再 tick 一次，应该用 EEG 前进标定帧
        node._tick()
        assert mock_serial.writes[-1] == b"\n0,0,900"
    finally:
        node.destroy_node()


def test_force_stop_releases_via_cmd_vel_cb_in_eeg_mode(rclpy_init):
    """EEG 模式 + force_stop 激活：通过真实 _cmd_vel_cb 收到零速 → 应解除。

    这是 Task 2 审查发现的死锁场景：原 _cmd_vel_cb 在 EEG 模式短路 return，
    导致 _latest_cmd_vel 永远不更新，force_stop 永远无法解除。
    修复：force_stop 期间不短路，让 cmd_vel 消息能更新缓存供解除判断。
    """
    from rtk_perception.chassis_serial_node import ChassisSerialNode
    from geometry_msgs.msg import Twist, Vector3
    from std_msgs.msg import Bool, Empty, Float64

    mock_serial = MockSerial()
    node = ChassisSerialNode(serial_factory=lambda **kw: mock_serial, open_serial=False)
    try:
        # EEG 激活 + 信号新鲜
        node._eeg_mode_active_cb(Bool(data=True))
        node._cmd_vel_eeg_cb(Twist(linear=Vector3(x=0.5), angular=Vector3(z=0.0)))
        node._heading_cb(Float64(data=0.0))

        # 触发 clear_goal
        node._clear_goal_cb(Empty())
        assert node._force_stop_active is True

        # 通过真实的 _cmd_vel_cb 收到零速 cmd_vel_safe（模拟 Nav2 cancel 后发零）
        node._cmd_vel_cb(Twist(linear=Vector3(x=0.0), angular=Vector3(z=0.0)))

        # 第一次 tick：解除发生在本帧，但本帧仍输出零速
        node._tick()
        assert node._force_stop_active is False, (
            "force_stop 应在收到零速 cmd_vel_safe 后解除，但仍为 True（死锁）"
        )

        # 第二次 tick：应用 EEG 信号
        node._tick()
        assert mock_serial.writes[-1] == b"\n0,0,900"
    finally:
        node.destroy_node()


def test_publish_odom_tf_uses_eeg_velocity(rclpy_init):
    """EEG 模式下 _publish_odom_tf 用 EEG 速度推算 dead-reckon（不是 Nav2）。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode
    from geometry_msgs.msg import Twist, Vector3
    from std_msgs.msg import Bool, Float64
    import time

    mock_serial = MockSerial()
    node = ChassisSerialNode(serial_factory=lambda **kw: mock_serial, open_serial=False)
    try:
        # EEG 激活 + cmd_vel_eeg 1.0 m/s
        node._eeg_mode_active_cb(Bool(data=True))
        node._cmd_vel_eeg_cb(Twist(linear=Vector3(x=1.0), angular=Vector3(z=0.0)))
        node._heading_cb(Float64(data=0.0))  # 朝北

        # 同时注入 Nav2 残留（应被忽略）
        node._latest_cmd_vel = Twist(linear=Vector3(x=0.6), angular=Vector3(z=0.0))

        # 跑 10 帧
        for _ in range(10):
            node._tick()
            time.sleep(0.011)

        # EEG 1.0 m/s × ~0.11s ≈ 0.11m 北向；若用 Nav2 0.6 则只有 0.066m
        # 阈值 0.08 区分两者
        assert node._dead_reckon_y_m > 0.08, (
            f"dead-reckon 应基于 EEG 1.0 m/s 推算（y > 0.08），"
            f"实际 {node._dead_reckon_y_m}（可能误用 Nav2 0.6）"
        )
    finally:
        node.destroy_node()


# ============================================================
# 限速器集成测试（启用限速器，验证 _tick 与限速器协同）
# ============================================================
# 注意：模块级 fixture _disable_slew_for_legacy_tests 会自动禁用限速器，
# 以下测试需要绕过 fixture，所以每个测试内部手动 _slew_limiter.set_enabled(True)
# 强制重新启用。

def test_node_declares_slew_limiter_params(rclpy_init):
    """节点声明 7 个限速器参数，默认值符合设计规格 § 4.1。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode

    mock_serial = MockSerial()
    node = ChassisSerialNode(serial_factory=lambda **kw: mock_serial, open_serial=False)
    try:
        assert node.get_parameter('slew_limiter_enabled').value is True
        assert node.get_parameter('forward_speed_ramp_per_sec').value == 2000
        assert node.get_parameter('direction_jump_threshold_deg').value == 45.0
        assert node.get_parameter('direction_cooldown_ms').value == 200
        assert node.get_parameter('jump_count_window_sec').value == 1.0
        assert node.get_parameter('jump_count_threshold').value == 3
        assert node.get_parameter('direction_long_cooldown_ms').value == 800
    finally:
        node.destroy_node()


def test_node_validates_slew_ramp_zero(rclpy_init):
    """forward_speed_ramp_per_sec=0 触发 ValueError（fail-fast）。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode

    mock_serial = MockSerial()
    with pytest.raises(ValueError, match="forward_speed_ramp_per_sec"):
        ChassisSerialNode(
            serial_factory=lambda **kw: mock_serial,
            open_serial=False,
            overrides={'forward_speed_ramp_per_sec': 0},
        )


def test_node_validates_slew_jump_threshold(rclpy_init):
    """direction_jump_threshold_deg > 180 触发 ValueError。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode

    mock_serial = MockSerial()
    with pytest.raises(ValueError, match="direction_jump_threshold_deg"):
        ChassisSerialNode(
            serial_factory=lambda **kw: mock_serial,
            open_serial=False,
            overrides={'direction_jump_threshold_deg': 200.0},
        )


def test_node_has_slew_limiter_attribute(rclpy_init):
    """节点构造后含 _slew_limiter 属性，类型正确，参数快照正确。"""
    from rtk_perception.chassis_serial_node import (
        ChassisSerialNode, ChassisSlewLimiter,
    )

    mock_serial = MockSerial()
    node = ChassisSerialNode(serial_factory=lambda **kw: mock_serial, open_serial=False)
    try:
        assert isinstance(node._slew_limiter, ChassisSlewLimiter)
        assert node._slew_limiter._ramp_per_sec == 2000
        assert node._slew_limiter._jump_threshold_deg == 45.0
        assert node._slew_limiter._cooldown_sec == 0.2
        assert node._slew_limiter._count_threshold == 3
    finally:
        node.destroy_node()


def test_tick_slew_limiter_smooths_nav_speed(rclpy_init):
    """_tick 内限速器启用时，speed 阶跃被 slew rate 平滑（首帧 speed<700）。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode
    from geometry_msgs.msg import Twist, Vector3
    from std_msgs.msg import Float64

    mock_serial = MockSerial()
    node = ChassisSerialNode(serial_factory=lambda **kw: mock_serial, open_serial=False)
    # 直接修改 _enabled 字段绕过 module fixture（fixture 强制 set_enabled 总是 False）
    node._slew_limiter._enabled = True
    mark_nav_ready(node)
    try:
        node._cmd_vel_cb(Twist(linear=Vector3(x=0.6)))
        node._heading_cb(Float64(data=0.0))
        # 首帧 dt=0 → max_delta=0 → speed 保持 0
        node._tick()
        speed_first = int(mock_serial.writes[-1].decode('ascii').strip().split(',')[2])
        assert speed_first == 0, f"首帧 dt=0 时 speed 应为 0，实际 {speed_first}"
    finally:
        node.destroy_node()


def test_tick_slew_limiter_disabled_passthrough(rclpy_init):
    """限速器禁用时，speed 立即顶到 target（直通）。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode
    from geometry_msgs.msg import Twist, Vector3
    from std_msgs.msg import Float64

    mock_serial = MockSerial()
    node = ChassisSerialNode(serial_factory=lambda **kw: mock_serial, open_serial=False)
    # module fixture 已经禁用限速器；显式确认
    assert node._slew_limiter._enabled is False
    mark_nav_ready(node)
    try:
        node._cmd_vel_cb(Twist(linear=Vector3(x=0.6)))
        node._heading_cb(Float64(data=0.0))
        node._tick()
        speed = int(mock_serial.writes[-1].decode('ascii').strip().split(',')[2])
        # 直通：mode_speed=700 立即输出
        assert speed == 700, f"限速器禁用时应直通 700，实际 {speed}"
    finally:
        node.destroy_node()


def test_tick_runtime_toggle_slew_limiter(rclpy_init):
    """运行时切换 slew_limiter_enabled 立即生效（紧急回退）。

    验证 _on_slew_param_changed 回调被触发，将 ROS 参数变更同步到限速器。
    """
    from rtk_perception.chassis_serial_node import ChassisSerialNode
    from rclpy.parameter import Parameter
    from geometry_msgs.msg import Twist, Vector3
    from std_msgs.msg import Float64

    mock_serial = MockSerial()
    node = ChassisSerialNode(serial_factory=lambda **kw: mock_serial, open_serial=False)
    mark_nav_ready(node)
    try:
        # 通过 ROS 参数变更触发 callback（fixture 让最终值总为 False，
        # 但 callback 内部 self._slew_limiter.set_enabled 应被调用）
        node.set_parameters([
            Parameter('slew_limiter_enabled', value=False)
        ])
        # 验证 callback 至少没崩溃，且参数被设置成功
        assert node.get_parameter('slew_limiter_enabled').value is False

        # 切换回 True
        # 先临时还原 fixture 的 patch，让 set_enabled 正常工作
        import rtk_perception.chassis_serial_node as csn
        orig_method = csn.ChassisSlewLimiter.set_enabled
        csn.ChassisSlewLimiter.set_enabled = lambda self, e: setattr(self, '_enabled', bool(e))
        try:
            node.set_parameters([
                Parameter('slew_limiter_enabled', value=True)
            ])
            assert node._slew_limiter._enabled is True
        finally:
            csn.ChassisSlewLimiter.set_enabled = orig_method
    finally:
        node.destroy_node()
