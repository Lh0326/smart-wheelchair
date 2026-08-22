"""path_to_baselink 纯算法单测（两阶段状态机版）。"""
import math
import pytest

from rtk_perception.path_to_baselink_node import (
    haversine_m,
    compute_bearing_rad,
    normalize_angle_rad,
    project_to_segment,
    find_nearest_on_path,
    find_lookahead_from_segment,
)


# ============ 基础几何 ============

def test_haversine_short_distance():
    d = haversine_m(24.0, 102.0, 24.001, 102.0)
    assert d == pytest.approx(111, abs=2)


def test_haversine_zero():
    assert haversine_m(24.0, 102.0, 24.0, 102.0) == 0.0


def test_bearing_north():
    assert compute_bearing_rad(24.0, 102.0, 24.001, 102.0) == pytest.approx(0.0, abs=0.001)


def test_bearing_east():
    assert compute_bearing_rad(24.0, 102.0, 24.0, 102.001) == pytest.approx(math.pi / 2, abs=0.01)


def test_bearing_south():
    assert abs(compute_bearing_rad(24.0, 102.0, 23.999, 102.0)) == pytest.approx(math.pi, abs=0.01)


def test_bearing_west():
    assert compute_bearing_rad(24.0, 102.0, 24.0, 101.999) == pytest.approx(-math.pi / 2, abs=0.01)


def test_normalize_angle_overflow():
    assert normalize_angle_rad(3 * math.pi) == pytest.approx(math.pi, abs=0.001)
    assert normalize_angle_rad(-3 * math.pi) == pytest.approx(-math.pi, abs=0.001)


# ============ project_to_segment ============

def test_project_to_segment_middle():
    """点投影到段的中间。"""
    p1 = (24.0, 102.0)
    p2 = (24.0 + 0.001, 102.0)  # 向北 111m
    # 用户在段中间正东 50m
    cur = (24.0 + 0.0005, 102.0 + 0.0005 / math.cos(math.radians(24)))
    t, proj = project_to_segment(cur[0], cur[1], p1, p2)
    assert t == pytest.approx(0.5, abs=0.05)
    assert proj[0] == pytest.approx(24.0 + 0.0005, abs=0.00001)


def test_project_to_segment_before_start():
    """点在段起点之前，应 clamp 到 t=0。"""
    p1 = (24.0, 102.0)
    p2 = (24.001, 102.0)
    cur = (23.999, 102.0)  # 在 p1 南边
    t, proj = project_to_segment(cur[0], cur[1], p1, p2)
    assert t == 0.0
    assert proj == p1


def test_project_to_segment_after_end():
    """点在段终点之后，应 clamp 到 t=1。"""
    p1 = (24.0, 102.0)
    p2 = (24.001, 102.0)
    cur = (24.002, 102.0)
    t, proj = project_to_segment(cur[0], cur[1], p1, p2)
    assert t == 1.0
    assert proj == p2


def test_project_degenerate_segment():
    """零长度段（p1==p2）应返回 t=0, p1。"""
    p1 = (24.0, 102.0)
    t, proj = project_to_segment(24.001, 102.001, p1, p1)
    assert t == 0.0
    assert proj == p1


# ============ find_nearest_on_path ============

def test_find_nearest_empty_path():
    idx, dist, point, t = find_nearest_on_path([], 24.0, 102.0)
    assert idx == -1
    assert math.isinf(dist)


def test_find_nearest_single_point():
    idx, dist, point, t = find_nearest_on_path([(24.001, 102.0)], 24.0, 102.0)
    assert idx == 0
    assert dist == pytest.approx(111, abs=2)
    assert point == (24.001, 102.0)


def test_find_nearest_on_path_mid():
    """用户在路径中段旁边，最近点应该返回正确距离。"""
    path = [(24.0 + i * 0.0001, 102.0) for i in range(10)]  # 北向延伸 100m
    # 用户在第 5 段旁边 10m（lat 在节点上，所以最近点 t=1.0 of seg 4 或 t=0.0 of seg 5）
    cur = (24.0 + 5 * 0.0001, 102.0 + 0.0001)
    idx, dist, point, t = find_nearest_on_path(path, cur[0], cur[1])
    assert idx == 4 or idx == 5
    assert dist == pytest.approx(10, abs=2)


def test_find_nearest_on_path_segment_middle():
    """用户在某段中点旁边（不在节点上），最近点 t 应≈0.5。"""
    path = [(24.0 + i * 0.0001, 102.0) for i in range(10)]
    # 用户在第 4 段中间 lat=24.00045 旁边 10m
    cur = (24.0 + 4.5 * 0.0001, 102.0 + 0.0001)
    idx, dist, point, t = find_nearest_on_path(path, cur[0], cur[1])
    assert idx == 4
    assert t == pytest.approx(0.5, abs=0.1)
    assert dist == pytest.approx(10, abs=2)


def test_find_nearest_far_from_path():
    """用户离路径很远（建筑物内），应正确返回最近段。"""
    # 路径向东延伸
    path = [(24.0, 102.0 + i * 0.001) for i in range(10)]
    # 用户在路径北边 30m
    cur = (24.0 + 0.0003, 102.005)
    idx, dist, point, t = find_nearest_on_path(path, cur[0], cur[1])
    assert dist == pytest.approx(33, abs=3)
    # 最近点应该在路径中间段
    assert 4 <= idx <= 6


# ============ find_lookahead_from_segment ============

def test_lookahead_from_start():
    """从路径起点 lookahead 2m。"""
    path = [(24.0 + i * 0.000009, 102.0) for i in range(10)]
    target = find_lookahead_from_segment(path, 0, 0.0, 2.0)
    assert target[0] == pytest.approx(24.0 + 2 * 0.000009, abs=0.000005)


def test_lookahead_from_mid_segment():
    """从段中段 lookahead，应正确累积距离。"""
    path = [(24.0 + i * 0.000009, 102.0) for i in range(10)]  # 10 点，9 段
    # 从第 3 段中间开始 lookahead 2m
    target = find_lookahead_from_segment(path, 3, 0.5, 2.0)
    # 第 3 段 t=0.5 在 24+3.5*0.000009, lookahead 2m → 24+5.5*0.000009
    assert target[0] == pytest.approx(24.0 + 5.5 * 0.000009, abs=0.00002)


def test_lookahead_beyond_path_end():
    """lookahead 超过路径终点，返回终点。"""
    path = [(24.0 + i * 0.000009, 102.0) for i in range(3)]
    target = find_lookahead_from_segment(path, 0, 0.0, 100.0)
    assert target == path[-1]


# ============ find_corners 测试 ============

from rtk_perception.path_to_baselink_node import find_corners


def test_find_corners_straight_line():
    """直线段，无拐角。"""
    path = [(24.0 + i * 0.0001, 102.0) for i in range(5)]
    corners = find_corners(path, threshold_deg=30.0)
    assert len(corners) == 2  # 只有 start + goal
    assert corners[0][2] == "start"
    assert corners[-1][2] == "goal"


def test_find_corners_l_shape():
    """L 形路径，1 个拐角。"""
    path = []
    # 先向北走 5 段
    for i in range(5):
        path.append((24.0 + i * 0.0001, 102.0))
    # 再向东走 5 段（90° 拐弯）
    for j in range(1, 5):
        path.append((24.0 + 4 * 0.0001, 102.0 + j * 0.0001))
    corners = find_corners(path, threshold_deg=30.0)
    assert len(corners) == 3  # start + 1 corner + goal
    assert corners[1][2] == "corner"


def test_find_corners_u_shape():
    """U 形路径，2 个拐角。"""
    path = []
    # 向北
    for i in range(3):
        path.append((24.0 + i * 0.0001, 102.0))
    # 向东
    for j in range(1, 4):
        path.append((24.0 + 2 * 0.0001, 102.0 + j * 0.0001))
    # 向南（180° 拐回）
    for k in range(1, 3):
        path.append((24.0 + 2 * 0.0001 - k * 0.0001, 102.0 + 3 * 0.0001))
    corners = find_corners(path, threshold_deg=30.0)
    corner_types = [c[2] for c in corners]
    assert corner_types.count("corner") == 2


def test_find_corners_small_curve_ignored():
    """缓弯（< 30°）不识别为拐角。"""
    path = []
    # 累积转 20°，每段 5°
    for i in range(10):
        angle = math.radians(i * 5)
        lat = 24.0 + i * 0.0001 * math.cos(angle)
        lon = 102.0 + i * 0.0001 * math.sin(angle)
        path.append((lat, lon))
    corners = find_corners(path, threshold_deg=30.0)
    # 缓弯不识别，只有 start + goal
    assert all(c[2] != "corner" for c in corners)


def test_find_corners_short_path():
    """路径 < 3 点，只有 start + goal。"""
    corners = find_corners([(24.0, 102.0), (24.0001, 102.0)], threshold_deg=30.0)
    assert len(corners) == 2
    assert corners[0][2] == "start"
    assert corners[-1][2] == "goal"


def test_find_corners_can_omit_start_target():
    path = [
        (24.0, 102.0),
        (24.0001, 102.0),
        (24.0001, 102.0001),
    ]
    waypoints = find_corners(path, threshold_deg=30.0, include_start=False)
    assert all(waypoint[2] != "start" for waypoint in waypoints)
    assert waypoints[0][2] in ("corner", "goal")


def test_determine_mode_can_assume_robot_is_already_on_route():
    path = [(24.0, 102.0), (24.001, 102.0)]
    mode, target, *_ = determine_mode(
        path,
        cur_lat=24.0,
        cur_lon=102.001,
        allow_approaching=False,
    )
    assert mode == "ON_PATH"
    assert target == path[-1]


# ============ path_distance_to_waypoint 测试 ============

from rtk_perception.path_to_baselink_node import (
    path_distance_to_waypoint,
    find_next_waypoint,
)


def test_path_distance_same_segment():
    """同段内距离 = haversine。"""
    path = [(24.0, 102.0), (24.001, 102.0), (24.002, 102.0)]
    # 用户在段 0 中间，目标在段 0 末端
    dist = path_distance_to_waypoint(path, from_seg_idx=0, from_seg_t=0.5, target=(24.001, 102.0))
    assert dist == pytest.approx(55, abs=5)  # 半段 ≈ 55m


def test_path_distance_cross_segments():
    """跨段累加。"""
    path = [(24.0, 102.0), (24.001, 102.0), (24.002, 102.0)]
    # 用户在段 0 起点，目标在段 1 末端（24.002, 102.0）
    dist = path_distance_to_waypoint(path, from_seg_idx=0, from_seg_t=0.0, target=(24.002, 102.0))
    assert dist == pytest.approx(222, abs=10)  # 两段 ≈ 222m


def test_path_distance_from_start_to_end():
    """从起点到终点的路径距离 = 完整路径长度。"""
    path = [(24.0, 102.0), (24.001, 102.0), (24.002, 102.0)]
    dist = path_distance_to_waypoint(path, from_seg_idx=0, from_seg_t=0.0, target=(24.002, 102.0))
    expected = haversine_m(24.0, 102.0, 24.001, 102.0) + haversine_m(24.001, 102.0, 24.002, 102.0)
    assert dist == pytest.approx(expected, abs=1)


# ============ find_next_waypoint 测试 ============

def test_find_next_waypoint_at_start():
    """用户在路径起点，下一拐角是第 1 个。"""
    path = []
    for i in range(3):
        path.append((24.0 + i * 0.0001, 102.0))
    for j in range(1, 3):
        path.append((24.0 + 2 * 0.0001, 102.0 + j * 0.0001))
    waypoints = find_corners(path)
    # 用户在路径起点（段 0，t=0）
    next_wp = find_next_waypoint(waypoints, path, from_seg_idx=0, from_seg_t=0.0)
    assert next_wp is not None
    # 应该返回第 1 个拐角（不是 start）
    assert next_wp[3] == "corner"


def test_find_next_waypoint_in_middle():
    """用户在路径中段，下一拐角是当前位置前方最近的一个。"""
    path = []
    for i in range(3):
        path.append((24.0 + i * 0.0001, 102.0))
    for j in range(1, 3):
        path.append((24.0 + 2 * 0.0001, 102.0 + j * 0.0001))
    waypoints = find_corners(path)
    # 用户在段 1 中间（已经过了第 1 个拐角，下一拐角应该是 goal）
    next_wp = find_next_waypoint(waypoints, path, from_seg_idx=2, from_seg_t=0.5)
    assert next_wp is not None
    assert next_wp[3] == "goal"


def test_find_next_waypoint_just_passed_corner():
    """用户刚走过拐角 1，下一拐角是 goal（不是刚走过的拐角 1）。"""
    path = []
    for i in range(3):
        path.append((24.0 + i * 0.0001, 102.0))
    for j in range(1, 3):
        path.append((24.0 + 2 * 0.0001, 102.0 + j * 0.0001))
    waypoints = find_corners(path)
    # 拐角在 path[2] = (24.0002, 102.0)，用户在 path[2] 之后段 2 中间
    next_wp = find_next_waypoint(waypoints, path, from_seg_idx=2, from_seg_t=0.5)
    # 应返回 goal，不是拐角
    assert next_wp[3] == "goal"


def test_find_next_waypoint_path_distance_not_straight():
    """路径距离排序 vs 直线距离。"""
    # 构造路径：先远离再回来
    path = [
        (24.0, 102.0),     # start
        (24.001, 102.0),   # 向北
        (24.001, 102.001), # 向东（拐角 1）
        (24.0, 102.001),   # 向南（拐角 2，靠近 start 直线距离）
        (24.0, 102.002),   # goal
    ]
    waypoints = find_corners(path)
    # 用户在 path[1] 附近，下一拐角应该是 path[2]（路径距离近）
    # 即使 path[3] 直线距离更近（因为 path[3] 离 path[1] 直线近）
    next_wp = find_next_waypoint(waypoints, path, from_seg_idx=0, from_seg_t=1.0)
    assert next_wp is not None
    # 应该是第 1 个拐角 path[2]
    assert next_wp[0] == pytest.approx(24.001, abs=0.00001)
    assert next_wp[1] == pytest.approx(102.001, abs=0.00001)


# ============ determine_mode 三态测试 ============

from rtk_perception.path_to_baselink_node import determine_mode


def test_determine_mode_approaching():
    """用户离路径 > 5m → APPROACHING。"""
    path = [(24.0, 102.0 + i * 0.001) for i in range(10)]  # 东向延伸
    cur_lat = 24.0 + 0.0003  # 北偏 30m
    cur_lon = 102.005
    mode, target, dist_to_path, dist_to_goal, seg_idx, seg_t = determine_mode(
        path, cur_lat, cur_lon,
        approach_threshold_m=5.0, completed_distance_m=3.0
    )
    assert mode == "APPROACHING"


def test_determine_mode_on_path():
    """用户在路径上 + 离终点远 → ON_PATH。"""
    path = [(24.0 + i * 0.0001, 102.0) for i in range(50)]  # 北向 500m
    cur_lat = 24.0 + 5 * 0.0001  # 在路径第 5 点附近
    cur_lon = 102.0
    mode, target, dist_to_path, dist_to_goal, seg_idx, seg_t = determine_mode(
        path, cur_lat, cur_lon,
        approach_threshold_m=5.0, completed_distance_m=3.0
    )
    assert mode == "ON_PATH"
    assert target is not None


def test_determine_mode_completed():
    """用户离终点（路径距离） < 3m → COMPLETED。"""
    path = [(24.0, 102.0), (24.0 + 0.00001, 102.0), (24.0 + 0.00002, 102.0)]
    cur_lat = 24.0 + 0.00002  # 在终点附近
    cur_lon = 102.0
    mode, target, dist_to_path, dist_to_goal, seg_idx, seg_t = determine_mode(
        path, cur_lat, cur_lon,
        approach_threshold_m=5.0, completed_distance_m=3.0
    )
    assert mode == "COMPLETED"
    assert target is None


# ============ WGS84→笛卡尔转换（TEB 任务 1） ============

from rtk_perception.path_to_baselink_node import path_to_cartesian_xy


def test_path_to_cartesian_north_1m():
    """北移 1m → y=1.0, x=0.0"""
    pts = path_to_cartesian_xy(
        origin_lat=24.0, origin_lon=102.0,
        path=[(24.0 + 1.0/111320.0, 102.0)],
    )
    assert pts[0][0] == pytest.approx(0.0, abs=0.01)
    assert pts[0][1] == pytest.approx(1.0, abs=0.01)


def test_path_to_cartesian_east_1m():
    """东移 1m → x=1.0, y=0.0"""
    pts = path_to_cartesian_xy(
        origin_lat=24.0, origin_lon=102.0,
        path=[(24.0, 102.0 + 1.0/(111320.0 * math.cos(math.radians(24.0))))],
    )
    assert pts[0][0] == pytest.approx(1.0, abs=0.01)
    assert pts[0][1] == pytest.approx(0.0, abs=0.01)


def test_path_to_cartesian_multi_points():
    """多点转换保持顺序和距离"""
    pts = path_to_cartesian_xy(
        origin_lat=24.0, origin_lon=102.0,
        path=[
            (24.0, 102.0),
            (24.0 + 10.0/111320.0, 102.0),  # 北 10m
            (24.0 + 10.0/111320.0, 102.0 + 10.0/(111320.0 * math.cos(math.radians(24.0)))),  # 北 10m 后东 10m
        ],
    )
    assert len(pts) == 3
    assert pts[0] == (0.0, 0.0)
    assert pts[1][1] == pytest.approx(10.0, abs=0.1)
    assert pts[2][0] == pytest.approx(10.0, abs=0.1)


def test_path_to_cartesian_empty_input():
    """空路径返回空列表"""
    assert path_to_cartesian_xy(24.0, 102.0, []) == []


def test_path_to_cartesian_skip_passed_points():
    """已走过的点（用户后方的段索引 < 当前）不在结果里"""
    pts = path_to_cartesian_xy(
        origin_lat=24.0, origin_lon=102.0,
        path=[
            (24.0, 102.0),
            (24.0 + 10.0/111320.0, 102.0),
            (24.0 + 10.0/111320.0, 102.0 + 10.0/(111320.0 * math.cos(math.radians(24.0)))),
        ],
        skip_before_seg_idx=0,
        skip_before_seg_t=0.5,
        cur_lat=24.0 + 5.0/111320.0,
        cur_lon=102.0,
    )
    # pts[0] = 当前位置（北 5m）
    # pts[1] = p1（北 10m）—— 必须保留，否则 L 形路径会丢失拐角
    # pts[2] = p2（北 10m + 东 10m）
    assert len(pts) == 3
    assert pts[0][1] == pytest.approx(5.0, abs=0.5)
    assert pts[1][1] == pytest.approx(10.0, abs=0.1)
    assert pts[1][0] == pytest.approx(0.0, abs=0.01)
    assert pts[2][0] == pytest.approx(10.0, abs=0.1)
    assert pts[2][1] == pytest.approx(10.0, abs=0.1)


# ============ nav_path 发布（TEB 任务 2） ============

from rtk_perception.path_to_baselink_node import build_nav_path_msg


def test_build_nav_path_basic():
    """构造 nav_msgs/Path 的 poses 列表（不依赖 rclpy）"""
    cartesian_pts = [(0.0, 0.0), (5.0, 0.0), (5.0, 5.0)]
    poses = build_nav_path_msg(cartesian_pts, frame_id="odom")
    assert len(poses) == 3
    assert poses[0]["x"] == 0.0 and poses[0]["y"] == 0.0
    assert poses[1]["x"] == 5.0 and poses[1]["y"] == 0.0
    assert poses[2]["x"] == 5.0 and poses[2]["y"] == 5.0


def test_build_nav_path_empty_returns_empty():
    poses = build_nav_path_msg([], frame_id="odom")
    assert poses == []


def test_build_nav_path_frame_id_propagates():
    """每个 pose 的 frame_id 应一致"""
    cartesian_pts = [(1.0, 2.0)]
    poses = build_nav_path_msg(cartesian_pts, frame_id="map")
    assert poses[0]["frame_id"] == "map"


# ============ 路径插值（TEB 修复） ============

from rtk_perception.path_to_baselink_node import interpolate_path


def test_interpolate_path_empty():
    assert interpolate_path([]) == []


def test_interpolate_path_single_point():
    assert interpolate_path([(1.0, 2.0)]) == [(1.0, 2.0)]


def test_interpolate_path_short_segment_unchanged():
    """段长 < 0.5m 不插值"""
    pts = [(0.0, 0.0), (0.3, 0.0)]
    result = interpolate_path(pts, max_segment_length_m=0.5)
    assert result == [(0.0, 0.0), (0.3, 0.0)]


def test_interpolate_path_long_segment():
    """段长 1m，间距 0.5m，应插值出 1 个中间点"""
    pts = [(0.0, 0.0), (1.0, 0.0)]
    result = interpolate_path(pts, max_segment_length_m=0.5)
    assert len(result) == 3
    assert result[0] == (0.0, 0.0)
    assert result[-1] == (1.0, 0.0)
    # 中间点应在 (0.5, 0)
    assert result[1][0] == pytest.approx(0.5, abs=0.01)
    assert result[1][1] == pytest.approx(0.0, abs=0.01)


def test_interpolate_path_91m_segment():
    """模拟实测 91m 跨度的 OSM 节点：插值后应得到 ~182 个点"""
    pts = [(0.0, 0.0), (-80.15, -44.32)]
    result = interpolate_path(pts, max_segment_length_m=0.5)
    seg_len = math.sqrt(80.15 ** 2 + 44.32 ** 2)
    expected_count = int(math.ceil(seg_len / 0.5)) + 1  # 起点也算
    assert len(result) == expected_count
    # 相邻点间距不超过 0.5m
    for i in range(len(result) - 1):
        dx = result[i + 1][0] - result[i][0]
        dy = result[i + 1][1] - result[i][1]
        d = math.sqrt(dx * dx + dy * dy)
        assert d <= 0.5 + 1e-6, f"点 {i} 到 {i+1} 间距 {d:.3f}m > 0.5m"


def test_interpolate_path_preserves_corners():
    """多段路径的拐角点应保留"""
    pts = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]
    result = interpolate_path(pts, max_segment_length_m=0.5)
    # 拐角 (1.0, 0.0) 必须在结果里
    assert (1.0, 0.0) in result
    assert (0.0, 0.0) in result
    assert (1.0, 1.0) in result


# ============ determine_mode hysteresis ============

def test_determine_mode_hysteresis_avoids_oscillation():
    """5m 边界附近不震荡：进入 APPROACHING 后要回到 4m 内才切回 ON_PATH。

    场景：直线北向路径，origin 在 (24.0, 102.0)。
    轮椅从路径上向北走，然后向东偏移到 4.5m 处（在 4-5m 之间）。

    无 hysteresis：5m 边界附近 ON_PATH↔APPROACHING 反复切换
    有 hysteresis：进入 APPROACHING 后保持，直到 < 4m 才回 ON_PATH
    """
    from rtk_perception.path_to_baselink_node import determine_mode
    p1 = (24.0, 102.0)
    p2 = (24.01, 102.0)  # 向北 ~1.1km，足够长的路径
    path = [p1, p2]

    # Case 1：mode=ON_PATH，偏 4.5m → 仍在 ON_PATH（< 5m 触发线）
    mode, _, _, _, _, _ = determine_mode(
        path,
        cur_lat=24.005,
        cur_lon=102.0 + 4.5 / (111320.0 * math.cos(math.radians(24.005))),  # 偏东 4.5m
        approach_threshold_m=5.0,
        approach_hysteresis_m=4.0,
        current_mode="ON_PATH",
        completed_distance_m=3.0,
        corner_threshold_deg=30.0,
    )
    assert mode == "ON_PATH", f"ON_PATH 状态下偏 4.5m 应保持 ON_PATH，实际 {mode}"

    # Case 2：mode=APPROACHING，偏 4.5m → 仍保持 APPROACHING（要 < 4m 才回）
    mode, _, _, _, _, _ = determine_mode(
        path,
        cur_lat=24.005,
        cur_lon=102.0 + 4.5 / (111320.0 * math.cos(math.radians(24.005))),
        approach_threshold_m=5.0,
        approach_hysteresis_m=4.0,
        current_mode="APPROACHING",
        completed_distance_m=3.0,
        corner_threshold_deg=30.0,
    )
    assert mode == "APPROACHING", f"APPROACHING 状态下偏 4.5m 应保持 APPROACHING，实际 {mode}"

    # Case 3：mode=APPROACHING，偏 3m → 回 ON_PATH（< 4m hysteresis）
    mode, _, _, _, _, _ = determine_mode(
        path,
        cur_lat=24.005,
        cur_lon=102.0 + 3.0 / (111320.0 * math.cos(math.radians(24.005))),
        approach_threshold_m=5.0,
        approach_hysteresis_m=4.0,
        current_mode="APPROACHING",
        completed_distance_m=3.0,
        corner_threshold_deg=30.0,
    )
    assert mode == "ON_PATH", f"APPROACHING 状态下偏 3m 应回 ON_PATH，实际 {mode}"


def test_determine_mode_hysteresis_default_backward_compatible():
    """旧调用（不传 current_mode / approach_hysteresis_m）应保持原行为。"""
    from rtk_perception.path_to_baselink_node import determine_mode
    p1 = (24.0, 102.0)
    p2 = (24.01, 102.0)
    path = [p1, p2]

    # 偏 6m，APPROACHING
    mode, _, _, _, _, _ = determine_mode(
        path,
        cur_lat=24.005,
        cur_lon=102.0 + 6.0 / (111320.0 * math.cos(math.radians(24.005))),
        approach_threshold_m=5.0,
        completed_distance_m=3.0,
        corner_threshold_deg=30.0,
    )
    assert mode == "APPROACHING"

    # 偏 2m，ON_PATH
    mode, _, _, _, _, _ = determine_mode(
        path,
        cur_lat=24.005,
        cur_lon=102.0 + 2.0 / (111320.0 * math.cos(math.radians(24.005))),
        approach_threshold_m=5.0,
        completed_distance_m=3.0,
        corner_threshold_deg=30.0,
    )
    assert mode == "ON_PATH"


# ============ seg_idx 坚持（绕障中段索引锁定） ============

def test_should_lock_seg_idx_when_deviation_grows():
    """绕障中（dist_to_path 增长）应保持上一帧 seg_idx。"""
    from rtk_perception.path_to_baselink_node import should_lock_seg_idx
    # ON_PATH 且偏 1.5m，上一帧偏 0.5m → 增长 1m → 锁定
    assert should_lock_seg_idx(
        mode="ON_PATH",
        last_seg_idx=2,
        current_dist_to_path=1.5,
        last_dist_to_path=0.5,
    ) is True


def test_should_not_lock_seg_idx_when_deviation_shrinking():
    """回归路径中（dist_to_path 减小）不锁定，用当前 seg_idx。"""
    from rtk_perception.path_to_baselink_node import should_lock_seg_idx
    assert should_lock_seg_idx(
        mode="ON_PATH",
        last_seg_idx=2,
        current_dist_to_path=0.3,
        last_dist_to_path=1.0,
    ) is False


def test_should_not_lock_seg_idx_when_no_history():
    """首次运行（无 last_seg_idx）不锁定。"""
    from rtk_perception.path_to_baselink_node import should_lock_seg_idx
    assert should_lock_seg_idx(
        mode="ON_PATH",
        last_seg_idx=None,
        current_dist_to_path=2.0,
        last_dist_to_path=0.0,
    ) is False


def test_should_not_lock_seg_idx_in_approaching():
    """APPROACHING 模式不锁定（用 nearest_point 引导回路径）。"""
    from rtk_perception.path_to_baselink_node import should_lock_seg_idx
    assert should_lock_seg_idx(
        mode="APPROACHING",
        last_seg_idx=2,
        current_dist_to_path=6.0,
        last_dist_to_path=5.5,
    ) is False


def test_should_not_lock_seg_idx_small_growth():
    """偏离增长 < 0.1m 视为噪声，不锁定。"""
    from rtk_perception.path_to_baselink_node import should_lock_seg_idx
    assert should_lock_seg_idx(
        mode="ON_PATH",
        last_seg_idx=2,
        current_dist_to_path=0.55,
        last_dist_to_path=0.5,
    ) is False
