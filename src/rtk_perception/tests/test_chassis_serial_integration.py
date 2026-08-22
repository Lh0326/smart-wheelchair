"""chassis_serial_node Layer 2 集成测试。

用 os.openpty() 创建伪终端对（master/slave），节点开 slave 端，
测试从 master 端读字节验证：
1. 完整字节流：cmd_vel + heading → 串口收到正确 ASCII 帧
2. heading 变化在帧中跟随
3. 100Hz 帧率（连续 tick N 次应产出 ~N 帧）
4. shutdown 连发 N 帧零速
"""
import os
import time

import pytest
import rclpy
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float64


def _drain_pty(master_fd, timeout=0.2):
    """读取 master 端所有可用字节。"""
    import select
    end = time.time() + timeout
    buf = b""
    while time.time() < end:
        r, _, _ = select.select([master_fd], [], [], 0.05)
        if r:
            try:
                chunk = os.read(master_fd, 4096)
                if chunk:
                    buf += chunk
            except OSError:
                break
        elif buf:
            break  # 没新数据但有旧数据，结束
    return buf


def _parse_frames(raw: bytes):
    """从字节流解析出完整 frame 列表（以 \\n 分隔）。"""
    text = raw.decode("ascii", errors="ignore")
    return [line + "\n" for line in text.split("\n") if line]


@pytest.fixture(scope='module')
def rclpy_init():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture(autouse=True, scope='module')
def _disable_slew_for_legacy_pty_tests():
    """既有 pty 测试基于"无限速器"行为设计，自动禁用避免回归。

    限速器行为由 test_chassis_slew_limiter.py 单独验证。本 fixture 仅
    作用于本文件，且新追加的限速器集成测试通过直接修改 _enabled 字段
    绕过本 fixture。
    """
    import rtk_perception.chassis_serial_node as csn
    orig_set_enabled = csn.ChassisSlewLimiter.set_enabled

    def force_disabled(self, enabled):
        orig_set_enabled(self, False)

    csn.ChassisSlewLimiter.set_enabled = force_disabled
    yield
    csn.ChassisSlewLimiter.set_enabled = orig_set_enabled


@pytest.fixture
def pty_pair():
    master_fd, slave_fd = os.openpty()
    slave_name = os.ttyname(slave_fd)
    yield master_fd, slave_name
    try:
        os.close(master_fd)
    except OSError:
        pass
    try:
        os.close(slave_fd)
    except OSError:
        pass


def _open_real_serial(port: str):
    """生产用 serial.Serial 工厂（测试中通过 pty 名打开）。"""
    import serial
    return serial.Serial(port=port, baudrate=115200, timeout=0)


def _mark_nav_ready(node):
    """测试中显式模拟 path_feeder 已接受 FollowPath goal。"""
    from rclpy.parameter import Parameter

    node.set_parameters([
        Parameter('nav_chassis_control_enabled', value=True),
    ])
    node._nav_control_active = True
    node._latest_nav_control_sec = node._now_sec()


def test_integration_frame_format(pty_pair, rclpy_init):
    """完整字节流：cmd_vel + heading → 串口收到正确 ASCII 帧。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode

    master_fd, slave_name = pty_pair
    node = ChassisSerialNode(
        serial_factory=lambda **kw: _open_real_serial(slave_name),
        open_serial=True,
    )
    try:
        # 给节点时间开串口
        time.sleep(0.1)
        _mark_nav_ready(node)
        # 注入输入
        node._cmd_vel_cb(Twist(linear=Vector3(x=0.6), angular=Vector3(z=0.0)))
        node._heading_cb(Float64(data=45.0))
        # 跑几帧
        for _ in range(5):
            node._tick()
            time.sleep(0.011)
        # 读 master 端
        raw = _drain_pty(master_fd, timeout=0.3)
        frames = _parse_frames(raw)
        assert len(frames) >= 3, f"帧数不足: {frames}"
        # 至少有一帧是预期的（forward_speed=700 = 0.6/0.6*700，任务 4 档位算法）
        assert "45,45,700\n" in frames, f"未找到预期帧: {frames}"
    finally:
        node._shutdown_safe()
        node.destroy_node()


def test_integration_heading_change_propagates(pty_pair, rclpy_init):
    """heading 变化 → 帧 current_angle 字段跟随变化。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode

    master_fd, slave_name = pty_pair
    node = ChassisSerialNode(
        serial_factory=lambda **kw: _open_real_serial(slave_name),
        open_serial=True,
    )
    try:
        time.sleep(0.1)
        _mark_nav_ready(node)
        node._cmd_vel_cb(Twist(linear=Vector3(x=0.0), angular=Vector3(z=0.0)))

        # 第一段：heading=10
        node._heading_cb(Float64(data=10.0))
        for _ in range(3):
            node._tick()
            time.sleep(0.011)

        # 第二段：heading=100
        node._heading_cb(Float64(data=100.0))
        for _ in range(3):
            node._tick()
            time.sleep(0.011)

        raw = _drain_pty(master_fd, timeout=0.3)
        frames = _parse_frames(raw)
        # 应同时看到 10,10,0 和 100,100,0
        assert any(f.startswith("10,") for f in frames), frames
        assert any(f.startswith("100,") for f in frames), frames
    finally:
        node._shutdown_safe()
        node.destroy_node()


def test_integration_rate_approx_100hz(pty_pair, rclpy_init):
    """连续 tick 50 次，应产出 ~50 帧（误差 ±5%，即 >=47 帧）。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode

    master_fd, slave_name = pty_pair
    node = ChassisSerialNode(
        serial_factory=lambda **kw: _open_real_serial(slave_name),
        open_serial=True,
    )
    try:
        time.sleep(0.1)
        _mark_nav_ready(node)
        node._cmd_vel_cb(Twist(linear=Vector3(x=0.6), angular=Vector3(z=0.0)))
        node._heading_cb(Float64(data=0.0))

        for _ in range(50):
            node._tick()
            time.sleep(0.01)  # 模拟 100Hz

        raw = _drain_pty(master_fd, timeout=0.5)
        frames = _parse_frames(raw)
        # 至少 47 帧（50 - 5% 误差）
        assert len(frames) >= 47, f"帧率过低: {len(frames)} 帧 / 50 次 tick"
    finally:
        node._shutdown_safe()
        node.destroy_node()


def test_integration_shutdown_zero_burst(pty_pair, rclpy_init):
    """shutdown 时连发 3 帧 zero（在 master 端可读）。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode

    master_fd, slave_name = pty_pair
    node = ChassisSerialNode(
        serial_factory=lambda **kw: _open_real_serial(slave_name),
        open_serial=True,
    )
    try:
        time.sleep(0.1)
        node._heading_cb(Float64(data=80.0))
        # 清空缓冲
        _drain_pty(master_fd, timeout=0.2)
        # shutdown
        node._shutdown_safe()
        raw = _drain_pty(master_fd, timeout=0.3)
        frames = _parse_frames(raw)
        zero_frames = [f for f in frames if f == "80,80,0\n"]
        assert len(zero_frames) >= 3, f"零速帧不足 3: {frames}"
    finally:
        node.destroy_node()


def test_integration_eeg_mode_byte_stream(pty_pair, rclpy_init):
    """EEG 激活 + cmd_vel_eeg → 串口实际收到正确字节流。"""
    from rtk_perception.chassis_serial_node import ChassisSerialNode
    from geometry_msgs.msg import Twist, Vector3
    from std_msgs.msg import Bool, Float64

    master_fd, slave_name = pty_pair
    node = ChassisSerialNode(
        serial_factory=lambda **kw: _open_real_serial(slave_name),
        open_serial=True,
    )
    try:
        time.sleep(0.1)
        # 激活 EEG 模式
        node._eeg_mode_active_cb(Bool(data=True))
        # 注入 EEG 信号 0.5 m/s
        node._cmd_vel_eeg_cb(Twist(linear=Vector3(x=0.5), angular=Vector3(z=0.0)))
        node._heading_cb(Float64(data=0.0))

        for _ in range(5):
            node._tick()
            time.sleep(0.011)

        raw = _drain_pty(master_fd, timeout=0.3)
        frames = _parse_frames(raw)
        assert len(frames) >= 3, f"帧数不足: {frames}"
        assert any(f == "0,0,700\n" for f in frames), (
            f"未找到预期 EEG 帧 '0,0,700\\n': {frames}"
        )
    finally:
        node._shutdown_safe()
        node.destroy_node()


def test_integration_eeg_to_nav2_switch(pty_pair, rclpy_init):
    """EEG → Nav2 切换：串口帧从脑控标定帧变为 Nav2 帧。

    新档位算法下 EEG 标定帧（700）和 Nav2 档位帧（mode=2 medium=700）数值相同。
    通过两次切换 _eeg_mode_active 状态 + 帧总数验证两段都跑了。
    """
    from rtk_perception.chassis_serial_node import ChassisSerialNode
    from geometry_msgs.msg import Twist, Vector3
    from std_msgs.msg import Bool, Float64

    master_fd, slave_name = pty_pair
    node = ChassisSerialNode(
        serial_factory=lambda **kw: _open_real_serial(slave_name),
        open_serial=True,
    )
    try:
        time.sleep(0.1)
        _mark_nav_ready(node)
        node._heading_cb(Float64(data=0.0))

        # 第一段：EEG 模式，前进标定帧
        node._eeg_mode_active_cb(Bool(data=True))
        node._cmd_vel_eeg_cb(Twist(linear=Vector3(x=0.5), angular=Vector3(z=0.0)))
        for _ in range(3):
            node._tick()
            time.sleep(0.011)
        assert node._eeg_override_active is True

        # 第二段：切回 Nav2
        node._eeg_mode_active_cb(Bool(data=False))
        node._cmd_vel_cb(Twist(linear=Vector3(x=0.6), angular=Vector3(z=0.0)))
        for _ in range(3):
            node._tick()
            time.sleep(0.011)
        assert node._eeg_override_active is False

        raw = _drain_pty(master_fd, timeout=0.3)
        frames = _parse_frames(raw)
        # 两段帧都应为 '0,0,700\\n'（EEG 标定 / Nav2 档位算法均为此值）。
        # 共 6 tick，过滤掉 shutdown 零速帧后应至少 5 帧
        assert frames.count("0,0,700\n") >= 5, (
            f"两段切换后帧数不足: {frames}"
        )
    finally:
        node._shutdown_safe()
        node.destroy_node()


# ============================================================
# 限速器 pty 集成测试（启用限速器，验证真实串口字节流）
# ============================================================

def test_integration_eeg_left_right_cooldown_holds_direction(pty_pair, rclpy_init):
    """EEG LEFT→RIGHT 快速切换 → 限速器 COOLDOWN 期间 direction=current_at_trigger。

    spec § 3.2 修订：COOLDOWN 期间 direction=current_at_trigger（EEG 模式
    current=0），让下位机 PID error=0，避免 speed=0 时仍驱动两轮反向差速。

    验证：pty 字节流的 direction 字段在 COOLDOWN 期间保持 current_at_trigger=0
    （而不是 -90 旧方向，也不是 +90 新方向）。
    """
    from rtk_perception.chassis_serial_node import ChassisSerialNode

    master_fd, slave_name = pty_pair

    mock = _open_real_serial(slave_name)
    node = ChassisSerialNode(
        serial_factory=lambda **kw: mock,
        open_serial=True,
        overrides={
            'eeg_left_direction_angle': -90,
            'eeg_left_speed': 300,
            'eeg_right_direction_angle': 90,
            'eeg_right_speed': 300,
        },
    )
    # 直接修改 _enabled 字段绕过 module fixture（fixture 强制 False）
    node._slew_limiter._enabled = True
    node.eeg_mode_active = True
    node._last_eeg_mode_msg_time = node.get_clock().now().nanoseconds * 1e-9

    try:
        # LEFT 帧：持续 500ms 让限速器 ramp 到 LEFT 中速 + last_direction=-90
        left_twist = Twist(linear=Vector3(x=0.0), angular=Vector3(z=0.5))
        node._latest_cmd_vel_eeg = left_twist
        node._latest_cmd_vel_eeg_sec = node.get_clock().now().nanoseconds * 1e-9
        node._last_eeg_motion_sec = node.get_clock().now().nanoseconds * 1e-9
        for _ in range(50):  # 500ms
            node._tick()
            time.sleep(0.01)

        # 清空 pty 缓冲
        try:
            os.read(master_fd, 65536)
        except OSError:
            pass

        # RIGHT 帧：触发 COOLDOWN，跑 10 帧（100ms 在 COOLDOWN 内）
        right_twist = Twist(linear=Vector3(x=0.0), angular=Vector3(z=-0.5))
        node._latest_cmd_vel_eeg = right_twist
        node._latest_cmd_vel_eeg_sec = node.get_clock().now().nanoseconds * 1e-9
        node._last_eeg_motion_sec = node.get_clock().now().nanoseconds * 1e-9
        for _ in range(10):
            node._tick()
            time.sleep(0.01)

        # 读字节流
        buf = _drain_pty(master_fd, timeout=0.3)

        # 解析最后 5 帧
        text = buf.decode('ascii', errors='ignore')
        parts = [p for p in text.split('\n') if p and ',' in p]
        assert len(parts) >= 3, f"应有 ≥3 帧，实际 {len(parts)}: {parts[-5:] if parts else 'EMPTY'}"

        for frame in parts[-5:]:
            nums = frame.split(',')
            assert len(nums) == 3, f"帧格式错误: {frame}"
            direction = int(nums[0])
            # COOLDOWN 期间 direction=current_at_trigger=0（让 PID error=0，
            # 不是 -90 旧方向也不是 +90 新方向）
            assert direction == 0, (
                f"COOLDOWN 期间 direction 应为 current_at_trigger=0（PID error=0），实际 {direction}"
            )
    finally:
        node._shutdown_safe()
        node.destroy_node()
