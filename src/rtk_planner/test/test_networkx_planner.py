"""networkx_planner_node 单元测试
import os as _os
def _find_ws_root():
    r = _os.environ.get("WS_ROOT")
    if r: return r
    d = _os.path.dirname(_os.path.abspath(__file__))
    for _ in range(6):
        if _os.path.exists(_os.path.join(d, "env.sh")): return d
        d = _os.path.dirname(d)
    return d
_WS_ROOT = _find_ws_root()
_MODELS_ROOT = _os.environ.get("MODELS_ROOT", _os.path.join(_WS_ROOT, "models"))


用真实 region.osm 数据做集成测试（不 mock，因为完全离线无副作用）。
"""
import os

import pytest
import rclpy
from sensor_msgs.msg import NavSatFix
from rtk_msgs.msg import GoalGPS, GlobalPlan
from rtk_planner.networkx_planner_node import haversine_m, NetworkxPlannerNode


# 测试用 OSM 数据路径
OSM_PATH = _WS_ROOT + '/data/region.osm'

# 测试区域中心（昆工呈贡校区）
CENTER_LAT = 24.8551
CENTER_LON = 102.8553


def test_haversine_zero_distance():
    """同一点距离为 0"""
    assert haversine_m(CENTER_LAT, CENTER_LON, CENTER_LAT, CENTER_LON) == 0.0


def test_haversine_known_distance():
    """向北 500m 距离 ≈ 500m"""
    d = haversine_m(CENTER_LAT, CENTER_LON, CENTER_LAT + 0.00450, CENTER_LON)
    assert 490 < d < 510, f"期望 ~500m，实际 {d}"


def test_haversine_symmetry():
    """A→B 等于 B→A"""
    d1 = haversine_m(CENTER_LAT, CENTER_LON, 24.8600, 102.8600)
    d2 = haversine_m(24.8600, 102.8600, CENTER_LAT, CENTER_LON)
    assert abs(d1 - d2) < 0.001


@pytest.fixture(scope='module')
def rclpy_init():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture(scope='module')
def planner_node(rclpy_init):
    """模块级共享节点，避免每个测试都加载 5 秒 OSM 数据"""
    if not os.path.exists(OSM_PATH):
        pytest.skip(f'测试数据不存在: {OSM_PATH}')
    node = NetworkxPlannerNode()
    node.last_fix = None
    yield node
    node.destroy_node()


def _make_fix(lat: float, lon: float) -> NavSatFix:
    fix = NavSatFix()
    fix.latitude = lat
    fix.longitude = lon
    return fix


def _make_goal(lat: float, lon: float, source: str = 'map_click') -> GoalGPS:
    goal = GoalGPS()
    goal.latitude = lat
    goal.longitude = lon
    goal.source = source
    return goal


def test_no_fix_returns_all_failed(planner_node):
    """last_fix 为空时返回 ALL_FAILED"""
    planner_node.last_fix = None
    plan = planner_node.plan(_make_goal(CENTER_LAT, CENTER_LON))

    assert plan.status == 'ALL_FAILED'
    assert plan.error_message == 'no_fix_yet'
    assert len(plan.path_wgs84) == 0


def test_too_close_returns_noop(planner_node):
    """距离 < min_goal_distance_m 时返回单点 noop"""
    planner_node.last_fix = _make_fix(CENTER_LAT, CENTER_LON)
    goal = _make_goal(CENTER_LAT + 0.00002, CENTER_LON)  # ~2m

    plan = planner_node.plan(goal)

    assert plan.status == 'OK'
    assert plan.source == 'noop'
    assert plan.distance_meters == 0.0
    assert len(plan.path_wgs84) == 1
    assert plan.path_wgs84[0].y == pytest.approx(CENTER_LAT + 0.00002, abs=1e-6)


def test_normal_route_within_campus(planner_node):
    """校区内典型路径（500m 左右）应该返回 OK + local_networkx"""
    planner_node.last_fix = _make_fix(CENTER_LAT, CENTER_LON)
    # 终点向东约 500m（0.0049 度 × cos(24.86°)）
    goal = _make_goal(CENTER_LAT, CENTER_LON + 0.0049)

    plan = planner_node.plan(goal)

    assert plan.status == 'OK', f'期望 OK，实际 {plan.status} ({plan.error_message})'
    assert plan.source == 'local_networkx'
    assert plan.distance_meters > 100, f'距离应该 >100m，实际 {plan.distance_meters}'
    assert plan.duration_seconds > 0
    assert len(plan.path_wgs84) >= 2

    # 验证坐标范围合理（在测试区域内）
    for p in plan.path_wgs84:
        assert 24.83 < p.y < 24.88
        assert 102.83 < p.x < 102.88


def test_far_route_within_region(planner_node):
    """区域内的远距离路径（1km+）"""
    planner_node.last_fix = _make_fix(CENTER_LAT, CENTER_LON)
    # 终点在 1.5km 外（东北方向）
    goal = _make_goal(CENTER_LAT + 0.010, CENTER_LON + 0.010)

    plan = planner_node.plan(goal)

    # 注意：终点可能落在 OSM 图的孤立分量里，如果失败也接受
    if plan.status == 'OK':
        assert plan.source == 'local_networkx'
        assert plan.distance_meters > 500
        assert len(plan.path_wgs84) >= 3
    else:
        # 区域内但属于不可达分量是可接受的（虽然 osmnx 1 个分量说明应该可达）
        assert plan.status in ('NO_ROUTE', 'ALL_FAILED'), f'意外的状态: {plan.status}'


def test_path_start_end_consistency(planner_node):
    """路径的起点应该接近 fix 位置，终点接近 goal 位置"""
    planner_node.last_fix = _make_fix(CENTER_LAT, CENTER_LON)
    goal = _make_goal(CENTER_LAT + 0.003, CENTER_LON + 0.003)

    plan = planner_node.plan(goal)

    if plan.status == 'OK' and plan.source == 'local_networkx':
        start_p = plan.path_wgs84[0]
        end_p = plan.path_wgs84[-1]
        # 起点和 fix 应该在 ~50m 内（最近节点可能稍远）
        assert haversine_m(start_p.y, start_p.x, CENTER_LAT, CENTER_LON) < 100, \
            f'起点偏离 fix 太远: {haversine_m(start_p.y, start_p.x, CENTER_LAT, CENTER_LON)}m'
        # 终点和 goal 应该在 ~100m 内
        assert haversine_m(end_p.y, end_p.x, goal.latitude, goal.longitude) < 200, \
            f'终点偏离 goal 太远: {haversine_m(end_p.y, end_p.x, goal.latitude, goal.longitude)}m'


def test_goal_source_propagated(planner_node):
    """goal.source 字段应该被复制到 plan.goal_source"""
    planner_node.last_fix = _make_fix(CENTER_LAT, CENTER_LON)
    goal = _make_goal(CENTER_LAT + 0.005, CENTER_LON, source='poi')
    goal.poi_name = 'test_poi'

    plan = planner_node.plan(goal)

    assert plan.goal_source == 'poi'


def test_start_lat_lon_filled(planner_node):
    """plan.start_lat/lon 应该填入 fix 位置"""
    planner_node.last_fix = _make_fix(24.8500, 102.8500)
    goal = _make_goal(24.8600, 102.8600)

    plan = planner_node.plan(goal)

    assert plan.start_lat == pytest.approx(24.8500)
    assert plan.start_lon == pytest.approx(102.8500)
