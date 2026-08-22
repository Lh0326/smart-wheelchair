"""teb_debug_node 纯函数单测（不依赖 rclpy）。"""
import math
import pytest

from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker


def test_build_cmd_vel_arrow_marker_forward_motion():
    """vx > 0.05 → 绿色箭头，长度 = vx × 2。"""
    from rtk_perception.teb_debug_node import build_cmd_vel_arrow_marker
    m = build_cmd_vel_arrow_marker(
        vx=0.5, wz=0.0, stamp_ns=12345, arrow_scale=2.0,
    )
    assert m.type == Marker.ARROW
    assert len(m.points) == 2
    assert m.points[0].x == pytest.approx(0.0)
    assert m.points[1].x == pytest.approx(1.0)  # 0.5 × 2.0
    assert m.color.g == pytest.approx(1.0)  # 绿色
    assert m.color.r == pytest.approx(0.0)
    assert m.color.a == pytest.approx(1.0)


def test_build_cmd_vel_arrow_marker_stopped():
    """|vx| < 0.05 → 红色（停止）。"""
    from rtk_perception.teb_debug_node import build_cmd_vel_arrow_marker
    m = build_cmd_vel_arrow_marker(
        vx=0.0, wz=0.3, stamp_ns=0, arrow_scale=2.0,
    )
    assert m.color.r == pytest.approx(1.0)
    assert m.color.g == pytest.approx(0.0)
    assert m.points[1].x == pytest.approx(0.0)


def test_build_cmd_vel_arrow_marker_negative_velocity():
    """vx < -0.05 → 黄色（理论不会出现，但需要能处理）。"""
    from rtk_perception.teb_debug_node import build_cmd_vel_arrow_marker
    m = build_cmd_vel_arrow_marker(
        vx=-0.3, wz=0.0, stamp_ns=0, arrow_scale=2.0,
    )
    assert m.color.r == pytest.approx(1.0)
    assert m.color.g == pytest.approx(1.0)
    assert m.points[1].x == pytest.approx(-0.6)


def test_build_cmd_vel_arrow_marker_namespace_fixed():
    """ns/id/type 固定，每次调用覆盖（不累积）。"""
    from rtk_perception.teb_debug_node import build_cmd_vel_arrow_marker
    m = build_cmd_vel_arrow_marker(vx=0.3, wz=0.0, stamp_ns=0, arrow_scale=2.0)
    assert m.ns == "cmd_vel_arrow"
    assert m.id == 0
    assert m.action == Marker.ADD
    assert m.header.frame_id == "base_link"


# ============ trail Marker ============

def test_build_trail_marker_empty_points():
    """空 trail → None（不发布）。"""
    from rtk_perception.teb_debug_node import build_trail_marker
    m = build_trail_marker(trail_points=[], stamp_ns=0)
    assert m is None


def test_build_trail_marker_single_point_skipped():
    """单点 trail → None（LINE_STRIP 需至少 2 点）。"""
    from rtk_perception.teb_debug_node import build_trail_marker
    m = build_trail_marker(trail_points=[(0.0, 0.0, 0.0)], stamp_ns=0)
    assert m is None


def test_build_trail_marker_multi_points():
    """多点 trail → LINE_STRIP + 渐变色（远透明近明亮）。"""
    from rtk_perception.teb_debug_node import build_trail_marker
    pts = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.1), (2.0, 0.0, 0.2)]
    m = build_trail_marker(trail_points=pts, stamp_ns=0)
    assert m is not None
    assert m.type == Marker.LINE_STRIP
    assert len(m.points) == 3
    assert len(m.colors) == 3
    # 渐变：第一点 alpha=0.1，最后一点 alpha=1.0
    assert m.colors[0].a == pytest.approx(0.1, abs=0.05)
    assert m.colors[-1].a == pytest.approx(1.0)
    assert m.header.frame_id == "odom"


# ============ deviation_line Marker ============

def test_build_deviation_line_marker_close():
    """偏离 < 0.3m → 绿色。"""
    from rtk_perception.teb_debug_node import build_deviation_line_marker
    m = build_deviation_line_marker(
        base_x=0.0, base_y=0.0, nearest_x=0.1, nearest_y=0.1, stamp_ns=0,
    )
    assert m is not None
    assert m.type == Marker.LINE_LIST
    assert len(m.points) == 2
    assert m.color.g == pytest.approx(1.0)
    assert m.color.r == pytest.approx(0.0)


def test_build_deviation_line_marker_medium():
    """偏离 0.3-1.0m → 橙色。"""
    from rtk_perception.teb_debug_node import build_deviation_line_marker
    m = build_deviation_line_marker(
        base_x=0.0, base_y=0.0, nearest_x=0.5, nearest_y=0.0, stamp_ns=0,
    )
    assert m is not None
    assert m.color.r == pytest.approx(1.0)
    assert m.color.g == pytest.approx(0.5)


def test_build_deviation_line_marker_far():
    """偏离 > 1.0m → 红色。"""
    from rtk_perception.teb_debug_node import build_deviation_line_marker
    m = build_deviation_line_marker(
        base_x=0.0, base_y=0.0, nearest_x=1.5, nearest_y=0.0, stamp_ns=0,
    )
    assert m is not None
    assert m.color.r == pytest.approx(1.0)
    assert m.color.g == pytest.approx(0.0)


def test_compute_nearest_in_path_simple():
    """path 上找最近点（笛卡尔坐标）。"""
    from rtk_perception.teb_debug_node import compute_nearest_in_path
    points = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    # 查询点在 (0.6, 0.2) → 最近点应在 (0.6, 0.0)（path 上）
    nx, ny, dist = compute_nearest_in_path(points, 0.6, 0.2)
    assert nx == pytest.approx(0.6, abs=0.01)
    assert ny == pytest.approx(0.0, abs=0.01)
    assert dist == pytest.approx(0.2, abs=0.01)


def test_compute_nearest_in_path_empty():
    from rtk_perception.teb_debug_node import compute_nearest_in_path
    result = compute_nearest_in_path([], 0.0, 0.0)
    assert result is None


# ============ mode_text Marker ============

def test_build_mode_text_marker_content():
    """文字内容包含 mode + vx + wz + deviation。"""
    from rtk_perception.teb_debug_node import build_mode_text_marker
    m = build_mode_text_marker(
        mode="ON_PATH", vx=0.5, wz=0.3, deviation=0.4, stamp_ns=0,
    )
    assert m.type == Marker.TEXT_VIEW_FACING
    assert "ON_PATH" in m.text
    assert "0.50" in m.text or "+0.50" in m.text
    assert m.pose.position.z == pytest.approx(2.0)
    assert m.color.r == pytest.approx(1.0)
    assert m.color.g == pytest.approx(1.0)
