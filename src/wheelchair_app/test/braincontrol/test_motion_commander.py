"""MotionCommander 单元测试（改造版，测 /cmd_vel_eeg 发布）。"""
import threading

import pytest
import rclpy
from geometry_msgs.msg import Twist

from wheelchair_app.braincontrol.control_types import MotionCommand
from wheelchair_app.braincontrol.motion_commander import MotionCommander


@pytest.fixture(scope='module')
def rclpy_init():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def node_and_commander(rclpy_init):
    """创建测试 node + commander + 监听 node。"""
    node = rclpy.create_node('test_motion_commander')
    spin_thread = threading.Thread(
        target=lambda: [rclpy.spin_once(node, timeout_sec=0.005) for _ in range(50)],
        daemon=True
    )
    spin_thread.start()

    commander = MotionCommander(ros_node=node)

    # 监听 /cmd_vel_eeg
    received = []
    node.create_subscription(Twist, '/cmd_vel_eeg', lambda m: received.append(m), 10)

    yield node, commander, received
    node.destroy_node()
    spin_thread.join(timeout=1.0)


def _drain(node, received, expected_count, timeout_sec=1.0):
    """轮询直到收到 expected_count 条消息或超时。"""
    import time
    deadline = time.time() + timeout_sec
    while time.time() < deadline and len(received) < expected_count:
        time.sleep(0.02)
    return received[:expected_count]


def test_forward_publishes_positive_linear_x(node_and_commander):
    node, commander, received = node_and_commander
    commander.update(MotionCommand.FORWARD)
    msgs = _drain(node, received, 1)
    assert len(msgs) == 1
    assert msgs[0].linear.x == pytest.approx(0.5)
    assert msgs[0].angular.z == pytest.approx(0.0)


def test_backward_publishes_negative_linear_x(node_and_commander):
    node, commander, received = node_and_commander
    commander.update(MotionCommand.BACKWARD)
    msgs = _drain(node, received, 1)
    assert msgs[0].linear.x == pytest.approx(-0.3)


def test_left_publishes_positive_angular_z(node_and_commander):
    node, commander, received = node_and_commander
    commander.update(MotionCommand.LEFT)
    msgs = _drain(node, received, 1)
    assert msgs[0].angular.z == pytest.approx(0.5)


def test_right_publishes_negative_angular_z(node_and_commander):
    node, commander, received = node_and_commander
    commander.update(MotionCommand.RIGHT)
    msgs = _drain(node, received, 1)
    assert msgs[0].angular.z == pytest.approx(-0.5)


def test_stop_publishes_zero_twist(node_and_commander):
    node, commander, received = node_and_commander
    commander.update(MotionCommand.STOP)
    msgs = _drain(node, received, 1)
    assert msgs[0].linear.x == 0.0
    assert msgs[0].angular.z == 0.0
