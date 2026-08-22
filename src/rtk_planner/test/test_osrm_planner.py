"""osrm_planner_node 单元测试"""
import math
from unittest.mock import patch, MagicMock
import requests

import pytest
import rclpy
from sensor_msgs.msg import NavSatFix
from rtk_msgs.msg import GoalGPS, GlobalPlan
from rtk_planner.osrm_planner_node import haversine_m, OsrmPlannerNode


def test_haversine_zero_distance():
    """同一点距离为 0"""
    assert haversine_m(24.8551, 102.8553, 24.8551, 102.8553) == 0.0


def test_haversine_known_distance():
    """昆工呈贡校区中心到 500 米外北点 ≈ 500m"""
    # 中心点
    lat1, lon1 = 24.8551, 102.8553
    # 向北 500m（500/111000 ≈ 0.00450 度）
    lat2, lon2 = 24.8551 + 0.00450, 102.8553
    d = haversine_m(lat1, lon1, lat2, lon2)
    assert 490 < d < 510, f"期望 ~500m，实际 {d}"


def test_haversine_symmetry():
    """A→B 距离等于 B→A"""
    d1 = haversine_m(24.8551, 102.8553, 24.8600, 102.8600)
    d2 = haversine_m(24.8600, 102.8600, 24.8551, 102.8553)
    assert abs(d1 - d2) < 0.001


@pytest.fixture
def planner_node(rclpy_init):
    """构造一个测试用 planner 节点（last_fix 为空）"""
    node = OsrmPlannerNode()
    node.last_fix = None  # 显式置空，模拟未收到 /fix
    yield node
    node.destroy_node()


def _make_fix(lat: float, lon: float) -> NavSatFix:
    fix = NavSatFix()
    fix.latitude = lat
    fix.longitude = lon
    return fix


def test_no_fix_returns_all_failed(planner_node):
    """last_fix 为空时，status=ALL_FAILED, error=no_fix_yet"""
    goal = GoalGPS()
    goal.latitude = 24.8551
    goal.longitude = 102.8553
    goal.source = 'map_click'

    plan = planner_node.plan(goal)

    assert plan.status == 'ALL_FAILED'
    assert plan.error_message == 'no_fix_yet'
    assert len(plan.path_wgs84) == 0


def test_too_close_returns_single_point(planner_node):
    """距离 < min_goal_distance_m 时返回单点 path，source=noop"""
    planner_node.last_fix = _make_fix(24.8551, 102.8553)  # 起点
    goal = GoalGPS()
    # 距起点 ~2.2m，小于默认阈值 5m
    goal.latitude = 24.85512
    goal.longitude = 102.8553
    goal.source = 'map_click'

    plan = planner_node.plan(goal)

    assert plan.status == 'OK'
    assert plan.source == 'noop'
    assert plan.distance_meters == 0.0
    assert len(plan.path_wgs84) == 1
    assert plan.path_wgs84[0].y == pytest.approx(24.85512, abs=1e-6)
    assert plan.path_wgs84[0].x == pytest.approx(102.8553, abs=1e-6)


MOCK_OSRM_OK = {
    "code": "Ok",
    "routes": [{
        "distance": 350.5,
        "duration": 280.4,
        "geometry": {
            "type": "LineString",
            "coordinates": [
                [102.8553, 24.8551],
                [102.8560, 24.8555],
                [102.8570, 24.8560]
            ]
        }
    }]
}


def _mock_requests_ok(url, **kwargs):
    resp = MagicMock()
    resp.status_code = 200
    resp.json.return_value = MOCK_OSRM_OK
    return resp


def test_local_osrm_ok(planner_node):
    """本地 OSRM 返回 Ok，source=local_osrm"""
    planner_node.last_fix = _make_fix(24.8551, 102.8553)
    goal = GoalGPS()
    goal.latitude = 24.8600  # 距起点 ~550m，大于阈值
    goal.longitude = 102.8600
    goal.source = 'map_click'

    with patch('rtk_planner.osrm_planner_node.requests.get', side_effect=_mock_requests_ok) as mock_get:
        plan = planner_node.plan(goal)

    # 期望先调本地 OSRM（被 mock 拦截）
    assert mock_get.call_count == 1
    called_url = mock_get.call_args[0][0]
    assert 'localhost:5000' in called_url

    assert plan.status == 'OK'
    assert plan.source == 'local_osrm'
    assert plan.distance_meters == pytest.approx(350.5)
    assert plan.duration_seconds == pytest.approx(280.4)
    assert len(plan.path_wgs84) == 3
    assert plan.path_wgs84[0].x == pytest.approx(102.8553)
    assert plan.path_wgs84[0].y == pytest.approx(24.8551)
    assert plan.path_wgs84[2].x == pytest.approx(102.8570)
    assert plan.path_wgs84[2].y == pytest.approx(24.8560)


def test_local_unreachable_fallback_public_ok(planner_node):
    """本地 OSRM ConnectionError → fallback 公共成功"""
    planner_node.last_fix = _make_fix(24.8551, 102.8553)
    goal = GoalGPS()
    goal.latitude = 24.8600
    goal.longitude = 102.8600

    def side_effect(url, **kwargs):
        if 'localhost:5000' in url:
            raise requests.ConnectionError("local down")
        resp = MagicMock()
        resp.status_code = 200
        resp.json.return_value = MOCK_OSRM_OK
        return resp

    with patch('rtk_planner.osrm_planner_node.requests.get', side_effect=side_effect) as mock_get:
        plan = planner_node.plan(goal)

    assert mock_get.call_count == 2
    assert plan.status == 'OK'
    assert plan.source == 'public_osrm'


def test_local_no_route_fallback_public_ok(planner_node):
    """本地 OSRM code=NoRoute → fallback 公共成功"""
    planner_node.last_fix = _make_fix(24.8551, 102.8553)
    goal = GoalGPS()
    goal.latitude = 24.8600
    goal.longitude = 102.8600

    no_route = {"code": "NoRoute", "message": "Impossible route"}

    def side_effect(url, **kwargs):
        resp = MagicMock()
        resp.status_code = 200
        resp.json.return_value = no_route if 'localhost:5000' in url else MOCK_OSRM_OK
        return resp

    with patch('rtk_planner.osrm_planner_node.requests.get', side_effect=side_effect):
        plan = planner_node.plan(goal)

    assert plan.status == 'OK'
    assert plan.source == 'public_osrm'


def test_both_unreachable_returns_all_failed(planner_node):
    """本地 ConnectionError + 公共 ConnectionError → ALL_FAILED"""
    planner_node.last_fix = _make_fix(24.8551, 102.8553)
    goal = GoalGPS()
    goal.latitude = 24.8600
    goal.longitude = 102.8600

    def side_effect(url, **kwargs):
        raise requests.ConnectionError("all down")

    with patch('rtk_planner.osrm_planner_node.requests.get', side_effect=side_effect):
        plan = planner_node.plan(goal)

    assert plan.status == 'ALL_FAILED'
    assert plan.error_message == 'public_unreachable'
    assert len(plan.path_wgs84) == 0


def test_both_no_route_returns_no_route(planner_node):
    """本地+公共都返回 code != Ok → NO_ROUTE"""
    planner_node.last_fix = _make_fix(24.8551, 102.8553)
    goal = GoalGPS()
    goal.latitude = 24.8600
    goal.longitude = 102.8600

    no_route = {"code": "NoRoute", "message": "Impossible route"}

    def side_effect(url, **kwargs):
        resp = MagicMock()
        resp.status_code = 200
        resp.json.return_value = no_route
        return resp

    with patch('rtk_planner.osrm_planner_node.requests.get', side_effect=side_effect):
        plan = planner_node.plan(goal)

    assert plan.status == 'NO_ROUTE'
    assert 'Impossible' in plan.error_message or plan.error_message == 'no_route'


def test_disable_public_fallback_skips_public(planner_node):
    """enable_public_fallback=false 且本地失败时，公共不被调用"""
    planner_node.last_fix = _make_fix(24.8551, 102.8553)
    planner_node.enable_public_fallback = False

    goal = GoalGPS()
    goal.latitude = 24.8600
    goal.longitude = 102.8600

    with patch('rtk_planner.osrm_planner_node.requests.get', side_effect=requests.ConnectionError("down")) as mock_get:
        plan = planner_node.plan(goal)

    # 只调一次（本地），公共未被调用
    assert mock_get.call_count == 1
    called_url = mock_get.call_args[0][0]
    assert 'localhost:5000' in called_url
    assert 'router.project-osrm.org' not in called_url

    assert plan.status == 'ALL_FAILED'
    assert plan.error_message == 'local_failed'
