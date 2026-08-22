"""networkx_planner_node 边吸附（edge snapping）单元测试
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


验证路径起终点终止于实际点击点（投影到所在路段），不是路口节点。
测试用例先取图中真实边的某点作为 goal，确保点击点本身在路上。
"""
import os

import pytest
import rclpy
from sensor_msgs.msg import NavSatFix
from rtk_msgs.msg import GoalGPS
from rtk_planner.networkx_planner_node import (
    haversine_m, NetworkxPlannerNode, project_onto_edge, _edge_line,
)
import osmnx as ox
from shapely.geometry import Point as ShapelyPoint


GRAPHML_PATH = _WS_ROOT + '/data/region.graphml'
CENTER_LAT = 24.8551
CENTER_LON = 102.8553


@pytest.fixture(scope='module')
def rclpy_init():
    if not rclpy.ok():
        rclpy.init()
    yield
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture(scope='module')
def planner_node(rclpy_init):
    if not os.path.exists(GRAPHML_PATH):
        pytest.skip(f'测试数据不存在: {GRAPHML_PATH}')
    node = NetworkxPlannerNode()
    yield node
    node.destroy_node()


def _make_fix(lat: float, lon: float) -> NavSatFix:
    fix = NavSatFix()
    fix.latitude = lat
    fix.longitude = lon
    return fix


def _make_goal(lat: float, lon: float) -> GoalGPS:
    g = GoalGPS()
    g.latitude = lat
    g.longitude = lon
    g.source = 'map_click'
    return g


def _point_on_edge(G, edge, frac):
    """取边上 frac 位置的实际经纬度点（确保测试点在路上）"""
    line = _edge_line(G, edge)
    p = line.interpolate(frac, normalized=True)
    return p.y, p.x  # lat, lon


def test_project_onto_edge_frac_in_range(planner_node):
    """project_onto_edge 返回 frac ∈ [0, 1]"""
    edge = ox.distance.nearest_edges(planner_node.G, X=[CENTER_LON], Y=[CENTER_LAT])[0]
    proj_lat, proj_lon, frac = project_onto_edge(planner_node.G, edge, CENTER_LAT, CENTER_LON)
    assert 0.0 <= frac <= 1.0
    assert 24.83 < proj_lat < 24.88
    assert 102.83 < proj_lon < 102.88


def test_project_round_trip_on_edge_point(planner_node):
    """边上某点再投影应该接近自身"""
    G = planner_node.G
    edge = ox.distance.nearest_edges(G, X=[CENTER_LON], Y=[CENTER_LAT])[0]
    # 取边上 frac=0.3 的点
    lat_on, lon_on = _point_on_edge(G, edge, 0.3)
    # 重新投影
    proj_lat, proj_lon, frac = project_onto_edge(G, edge, lat_on, lon_on)
    err = haversine_m(proj_lat, proj_lon, lat_on, lon_on)
    assert err < 1.0, f'重投影偏差 {err:.2f}m，应 <1m'
    assert abs(frac - 0.3) < 0.01, f'frac 偏差 {abs(frac - 0.3):.3f}'


def test_path_end_matches_goal_on_road(planner_node):
    """终点取自实际路边某点，规划路径终点应该 ≈ 该点（<5m）"""
    G = planner_node.G
    # 找一条长边作为测试
    start_edge = ox.distance.nearest_edges(G, X=[CENTER_LON], Y=[CENTER_LAT])[0]
    # 取 start_edge frac=0.1 的点作为 fix（确保 fix 在路上）
    s_lat, s_lon = _point_on_edge(G, start_edge, 0.1)
    planner_node.last_fix = _make_fix(s_lat, s_lon)

    # 找另一条边作为 goal
    goal_lat_offset = CENTER_LAT + 0.003
    goal_lon_offset = CENTER_LON + 0.003
    goal_edge = ox.distance.nearest_edges(G, X=[goal_lon_offset], Y=[goal_lat_offset])[0]
    # 取 goal_edge frac=0.5 的点作为 goal
    g_lat, g_lon = _point_on_edge(G, goal_edge, 0.5)
    goal = _make_goal(g_lat, g_lon)

    plan = planner_node.plan(goal)

    assert plan.status == 'OK', f'期望 OK，实际 {plan.status} ({plan.error_message})'
    assert plan.source == 'local_networkx'

    end_p = plan.path_wgs84[-1]
    err = haversine_m(end_p.y, end_p.x, g_lat, g_lon)
    assert err < 5.0, f'终点偏离 goal {err:.2f}m，期望 <5m（边吸附未生效？）'


def test_path_start_matches_fix_on_road(planner_node):
    """起点取自实际路边某点，规划路径起点应该 ≈ 该点（<5m）"""
    G = planner_node.G
    start_edge = ox.distance.nearest_edges(G, X=[CENTER_LON], Y=[CENTER_LAT])[0]
    s_lat, s_lon = _point_on_edge(G, start_edge, 0.2)  # 边上 20% 位置
    planner_node.last_fix = _make_fix(s_lat, s_lon)

    goal_edge = ox.distance.nearest_edges(G, X=[CENTER_LON + 0.004], Y=[CENTER_LAT])[0]
    g_lat, g_lon = _point_on_edge(G, goal_edge, 0.5)
    goal = _make_goal(g_lat, g_lon)

    plan = planner_node.plan(goal)

    if plan.status == 'OK' and plan.source == 'local_networkx':
        start_p = plan.path_wgs84[0]
        err = haversine_m(start_p.y, start_p.x, s_lat, s_lon)
        assert err < 5.0, f'起点偏离 fix {err:.2f}m，期望 <5m'


def test_goal_at_edge_midpoint_not_overshooting(planner_node):
    """点击在边中点（frac=0.5），路径终点应该在中点附近，不应跨到下个路口"""
    G = planner_node.G
    start_edge = ox.distance.nearest_edges(G, X=[CENTER_LON], Y=[CENTER_LAT])[0]
    s_lat, s_lon = _point_on_edge(G, start_edge, 0.0)  # 起点=边的 u 端
    planner_node.last_fix = _make_fix(s_lat, s_lon)

    # 找一条长度 >50m 的边作为 goal
    goal_edge = None
    for e in G.edges(keys=True):
        if G.edges[e].get('length', 0) > 50:
            # 取距离 CENTER 约 500m 的边
            u_pos = G.nodes[e[0]]
            d = haversine_m(u_pos['y'], u_pos['x'], CENTER_LAT, CENTER_LON)
            if 200 < d < 800:
                goal_edge = e
                break
    if goal_edge is None:
        pytest.skip('没找到合适的测试边')

    g_lat, g_lon = _point_on_edge(G, goal_edge, 0.5)  # 边中点
    goal = _make_goal(g_lat, g_lon)

    plan = planner_node.plan(goal)

    if plan.status == 'OK' and plan.source == 'local_networkx':
        end_p = plan.path_wgs84[-1]
        err = haversine_m(end_p.y, end_p.x, g_lat, g_lon)
        edge_len = G.edges[goal_edge].get('length', 0)
        # 终点应该 ≈ 中点（<5m），不应是边的端点（否则 err ≈ edge_len/2）
        assert err < 5.0, \
            f'终点偏离 {err:.1f}m，边长 {edge_len:.1f}m，期望 <5m（看起来终点跑到了路口）'
