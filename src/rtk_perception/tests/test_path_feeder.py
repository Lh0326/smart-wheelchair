"""path_feeder_node.path_signature 单元测试。

验证:
  - 空路径返回空 bytes
  - 短路径(<=sample_count)全部采样
  - 长路径均匀采样(含首尾)
  - mm 级抖动(<1mm)被过滤(签名相同)
  - 实质不同路径签名不同
"""
import threading
from types import SimpleNamespace

import pytest
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from nav_msgs.msg import Path

from rtk_perception.path_feeder_node import (
    PathFeederNode,
    destination_signature,
    path_signature,
    signature_retry_blocked,
)


def _make_pose(x: float, y: float) -> PoseStamped:
    ps = PoseStamped()
    ps.pose = Pose(position=Point(x=x, y=y), orientation=Quaternion(w=1.0))
    return ps


def _make_path(points) -> Path:
    p = Path()
    p.poses = [_make_pose(x, y) for x, y in points]
    return p


def test_path_signature_empty_returns_empty_bytes():
    p = Path()  # poses = []
    assert path_signature(p) == b""


def test_path_signature_short_path_all_sampled():
    """路径只有 3 点,应该全部采样(签名含全部 3 点)。"""
    p = _make_path([(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)])
    sig = path_signature(p)
    assert b"0,0" in sig
    assert b"1000,0" in sig
    assert b"2000,0" in sig


def test_path_signature_long_path_uniform_sampling():
    """长路径(20 点)采样 8 个关键点(含首尾)。"""
    points = [(float(i), float(i * 2)) for i in range(20)]
    p = _make_path(points)
    sig = path_signature(p, sample_count=8).decode()
    parts = sig.split("|")
    assert len(parts) == 8
    # 首点必采
    assert parts[0] == "0,0"
    # 尾点必采
    assert parts[-1] == "19000,38000"


def test_path_signature_mm_jitter_filtered():
    """<1mm 浮点抖动应被视为同一路径(签名相同)。"""
    p1 = _make_path([(1.0001, 2.0001), (3.0002, 4.0002)])
    p2 = _make_path([(1.0004, 2.0003), (3.0005, 4.0004)])
    # 全部 round 到 mm 都是 (1000/2000/3000/4000) mm
    assert path_signature(p1) == path_signature(p2)


def test_path_signature_different_paths_differ():
    p1 = _make_path([(0.0, 0.0), (5.0, 0.0)])   # 直线
    p2 = _make_path([(0.0, 0.0), (5.0, 3.0)])   # 斜线
    assert path_signature(p1) != path_signature(p2)


def test_path_signature_sample_count_one_handles_short():
    """sample_count=1 边界:不应除以零。"""
    p = _make_path([(1.0, 1.0)])
    sig = path_signature(p, sample_count=1)
    assert b"1000,1000" in sig


def test_path_signature_single_pose():
    """只有一个 pose 的路径不应崩溃(均匀采样除零保护)。"""
    p = _make_path([(3.5, 7.2)])
    sig = path_signature(p)
    assert b"3500,7200" in sig


def test_path_signature_can_ignore_live_robot_start_pose():
    p1 = _make_path([(0.0, 0.0), (5.0, 1.0), (10.0, 2.0)])
    p2 = _make_path([(0.8, -0.4), (5.0, 1.0), (10.0, 2.0)])
    assert path_signature(p1, ignore_start_pose=True) == path_signature(
        p2, ignore_start_pose=True
    )


def test_destination_signature_ignores_trimmed_route_prefix():
    original = _make_path([
        (0.0, 0.0), (2.0, 0.0), (5.0, 1.0), (10.0, 2.0)
    ])
    after_first_waypoint = _make_path([
        (2.3, 0.1), (5.0, 1.0), (10.0, 2.0)
    ])

    assert destination_signature(original) == destination_signature(
        after_first_waypoint
    )


def test_destination_signature_changes_for_new_goal():
    first_goal = _make_path([(0.0, 0.0), (10.0, 2.0)])
    second_goal = _make_path([(0.0, 0.0), (12.0, 2.0)])

    assert destination_signature(first_goal) != destination_signature(second_goal)


def test_destination_signature_filters_small_goal_jitter():
    first = _make_path([(0.0, 0.0), (10.04, 2.04)])
    jittered = _make_path([(1.0, 0.0), (10.08, 2.08)])

    assert destination_signature(first, resolution_m=0.25) == destination_signature(
        jittered, resolution_m=0.25
    )


def test_abort_backoff_expires_for_same_path():
    signature = b"route-a"
    assert signature_retry_blocked(signature, 15.0, signature, 14.9)
    assert not signature_retry_blocked(signature, 15.0, signature, 15.0)
    assert not signature_retry_blocked(signature, 15.0, b"route-b", 14.9)


class _Logger:
    def info(self, *args, **kwargs):
        pass

    def warn(self, *args, **kwargs):
        pass


class _Future:
    def __init__(self, value):
        self._value = value

    def result(self):
        return self._value


def _bare_feeder(now_sec=100.0):
    node = PathFeederNode.__new__(PathFeederNode)
    node._lock = threading.Lock()
    node._current_goal_handle = None
    node._current_goal_signature = None
    node._last_sent_signature = None
    node._blocked_signature = None
    node._blocked_until_sec = 0.0
    node._abort_retry_backoff_sec = 5.0
    node._publish_nav_control_active = lambda *args, **kwargs: None
    node.get_logger = lambda: _Logger()
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=int(now_sec * 1e9))
    )
    return node


def test_late_old_goal_result_does_not_clear_new_goal():
    node = _bare_feeder()
    old_handle = object()
    new_handle = object()
    node._current_goal_handle = new_handle
    node._current_goal_signature = b"new-route"

    node._result_cb(
        _Future(SimpleNamespace(status=6)),
        goal_handle=old_handle,
        goal_sig=b"old-route",
    )

    assert node._current_goal_handle is new_handle
    assert node._current_goal_signature == b"new-route"
    assert node._blocked_signature is None


def test_aborted_goal_clears_dedupe_and_starts_retry_backoff():
    node = _bare_feeder(now_sec=100.0)
    handle = object()
    signature = b"route-a"
    node._current_goal_handle = handle
    node._current_goal_signature = signature
    node._last_sent_signature = signature

    node._result_cb(
        _Future(SimpleNamespace(status=6)),
        goal_handle=handle,
        goal_sig=signature,
    )

    assert node._current_goal_handle is None
    assert node._last_sent_signature is None
    assert node._blocked_signature == signature
    assert node._blocked_until_sec == pytest.approx(105.0)
