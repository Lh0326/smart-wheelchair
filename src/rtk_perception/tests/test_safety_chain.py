"""safety_chain 纯算法单测。"""
import math

import numpy as np
import pytest

from rtk_perception.safety_chain import (
    SafetyConfig,
    apply_safety_chain,
    LaserScanData,
    Twist2D,
)


def make_scan(ranges, angle_min=-math.pi, angle_max=math.pi):
    angles = np.linspace(angle_min, angle_max, len(ranges))
    return LaserScanData(ranges=np.array(ranges, dtype=float), angles=angles)


def test_emergency_stop_close_obstacle():
    """前方 0.2m 障碍 → 全停。"""
    scan = make_scan([0.2] * 360)
    twist = Twist2D(linear_x=0.5, angular_z=0.0)
    out = apply_safety_chain(twist, scan, SafetyConfig())
    assert out.linear_x == 0.0
    assert out.angular_z == 0.0


def test_slowdown_medium_obstacle():
    """前方 0.5m 障碍 → 在急停线与减速线之间连续减速。"""
    scan = make_scan([0.5] * 360)
    twist = Twist2D(linear_x=0.5, angular_z=0.0)
    out = apply_safety_chain(twist, scan, SafetyConfig())
    expected_scale = 0.3 + 0.7 * ((0.5 - 0.3) / (1.0 - 0.3))
    assert out.linear_x == pytest.approx(0.5 * expected_scale, abs=0.01)


def test_passthrough_far_obstacle():
    """前方 1.5m 障碍 → 透传。"""
    scan = make_scan([1.5] * 360)
    twist = Twist2D(linear_x=0.5, angular_z=0.0)
    out = apply_safety_chain(twist, scan, SafetyConfig())
    assert out.linear_x == pytest.approx(0.5, abs=0.01)


def test_no_trigger_side_obstacle():
    """侧方（±60°外）0.2m 障碍 → 不触发。"""
    ranges = [float("inf")] * 360
    ranges[90] = 0.2
    ranges[270] = 0.2
    scan = make_scan(ranges)
    twist = Twist2D(linear_x=0.5, angular_z=0.0)
    out = apply_safety_chain(twist, scan, SafetyConfig())
    assert out.linear_x == pytest.approx(0.5, abs=0.01)


def test_nan_cmd_vel_output_zero():
    """cmd_vel 输入 NaN → 输出 0。"""
    scan = make_scan([float("inf")] * 360)
    twist = Twist2D(linear_x=float("nan"), angular_z=float("nan"))
    out = apply_safety_chain(twist, scan, SafetyConfig())
    assert out.linear_x == 0.0
    assert out.angular_z == 0.0


def test_multiple_obstacles_take_min():
    """前方多个障碍取最近距离。"""
    scan = make_scan([float("inf")] * 360)
    scan.ranges[180] = 0.4
    scan.ranges[170] = 0.6
    twist = Twist2D(linear_x=0.5, angular_z=0.0)
    out = apply_safety_chain(twist, scan, SafetyConfig())
    expected_scale = 0.3 + 0.7 * ((0.4 - 0.3) / (1.0 - 0.3))
    assert out.linear_x == pytest.approx(0.5 * expected_scale, abs=0.01)


def test_slowdown_is_monotonic_and_continuous():
    """障碍越近速度越低，且减速边界附近不发生固定倍率阶跃。"""
    cfg = SafetyConfig()
    twist = Twist2D(linear_x=0.5, angular_z=0.2)

    far = apply_safety_chain(twist, make_scan([0.99] * 360), cfg)
    middle = apply_safety_chain(twist, make_scan([0.65] * 360), cfg)
    near = apply_safety_chain(twist, make_scan([0.31] * 360), cfg)

    assert 0.49 < far.linear_x < 0.5
    assert far.linear_x > middle.linear_x > near.linear_x > 0.0
    assert far.angular_z == middle.angular_z == near.angular_z == 0.2
