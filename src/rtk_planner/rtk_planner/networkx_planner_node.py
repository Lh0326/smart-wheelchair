"""NetworkX 离线路径规划 ROS2 节点
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


订阅 /fix（当前位置）和 /goal_gps（前端点击终点），
用 osmnx + NetworkX 在本地 OSM 图上做最短路径规划，发布 /global_plan。

完全离线、零外部依赖。不调任何 HTTP/网络服务。

终点（和起点）吸附到最近**边**（路段），不是最近**节点**（路口）。
点击位置投影到所在路段上，路径终止于实际点击点而非下个路口。
"""
import math
import os
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header, Empty
from geometry_msgs.msg import Point

import osmnx as ox
import networkx as nx
from shapely.geometry import Point as ShapelyPoint, LineString
from shapely.wkt import loads as wkt_loads

from rtk_msgs.msg import GoalGPS, GlobalPlan


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """计算两个 WGS84 经纬度点之间的球面距离（米）"""
    R = 6371000.0
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2
    return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def _edge_line(G, edge: Tuple) -> LineString:
    """取边的几何线：若有 geometry 属性则用之（曲线），否则用 u→v 直线"""
    u, v, key = edge
    edge_data = G.edges[edge]
    geom = edge_data.get('geometry')
    if geom:
        return wkt_loads(geom)
    u_node = G.nodes[u]
    v_node = G.nodes[v]
    return LineString([(u_node['x'], u_node['y']), (v_node['x'], v_node['y'])])


def project_onto_edge(G, edge: Tuple, lat: float, lon: float) -> Tuple[float, float, float]:
    """将点击点投影到路段上。

    返回 (proj_lat, proj_lon, frac_from_u)：
      - proj_lat/lon: 投影点的 WGS84 坐标（实际终止/起点位置）
      - frac_from_u: 投影点在 (u→v) 方向上的相对位置，0=在 u, 1=在 v
    """
    line = _edge_line(G, edge)
    point = ShapelyPoint(lon, lat)
    dist_along = line.project(point)  # 在 lon/lat 度空间下
    total = line.length
    frac = dist_along / total if total > 0 else 0.0
    proj = line.interpolate(dist_along)
    return proj.y, proj.x, frac


class NetworkxPlannerNode(Node):
    def __init__(self):
        super().__init__('networkx_planner')

        self.declare_parameter('osm_path', _WS_ROOT + '/data/region.graphml')
        self.declare_parameter('walking_speed_mps', 1.4)
        self.declare_parameter('min_goal_distance_m', 5.0)

        self.osm_path = self.get_parameter('osm_path').value
        self.walking_speed_mps = self.get_parameter('walking_speed_mps').value
        self.min_goal_distance_m = self.get_parameter('min_goal_distance_m').value

        if not os.path.exists(self.osm_path):
            self.get_logger().fatal(f'图文件不存在: {self.osm_path}')
            raise FileNotFoundError(self.osm_path)

        self.get_logger().info(f'加载图: {self.osm_path}（3-5 秒）...')
        self.G = ox.load_graphml(self.osm_path)
        self.get_logger().info(
            f'图加载完成: {len(self.G.nodes)} 节点, {len(self.G.edges)} 边'
        )

        self.last_fix: Optional[NavSatFix] = None

        # 改为 volatile QoS：每 3 秒发新数据，前端/path_to_baselink_node 收到更新
        volatile_qos = QoSProfile(depth=10, durability=QoSDurabilityPolicy.VOLATILE)

        self.create_subscription(NavSatFix, '/fix', self._on_fix, 10)
        self.create_subscription(GoalGPS, '/goal_gps', self._on_goal, 10)
        self.create_subscription(Empty, '/clear_goal', self._on_clear_goal, 10)
        self.plan_pub = self.create_publisher(GlobalPlan, '/global_plan', volatile_qos)

        # 路径实时刷新定时器：每 3 秒用最新 /fix 重算（如果有 last_goal）
        self.declare_parameter('refresh_interval_sec', 3.0)
        refresh_sec = self.get_parameter('refresh_interval_sec').value
        self._last_goal: Optional[GoalGPS] = None
        self.create_timer(refresh_sec, self._refresh_plan)
        self.get_logger().info(f'NetworkX planner: refresh every {refresh_sec}s')

        self.get_logger().info(
            f'Networkx planner 就绪: walking_speed={self.walking_speed_mps} m/s, '
            f'min_goal_distance={self.min_goal_distance_m} m'
        )

    def _on_fix(self, msg: NavSatFix):
        self.last_fix = msg

    def _on_clear_goal(self, msg: Empty):
        """前端清除终点时调用，停止 3 秒定时刷新。"""
        self._last_goal = None
        self.get_logger().info('终点已清除（_last_goal = None），停止定时刷新')

    def _on_goal(self, msg: GoalGPS):
        self._last_goal = msg  # 缓存目标，供定时器复用
        plan = self.plan(msg)
        self.plan_pub.publish(plan)
        self.get_logger().info(
            f'规划结果 status={plan.status} source={plan.source} '
            f'distance={plan.distance_meters:.1f}m points={len(plan.path_wgs84)}'
        )

    def _refresh_plan(self):
        """定时刷新路径：如果有 last_goal，用最新 /fix 重算。"""
        if self._last_goal is None:
            return  # 用户还没点击终点，不刷新
        if self.last_fix is None:
            return  # 还没收到 /fix，不刷新
        # 复用 _on_goal 的规划逻辑
        self._on_goal(self._last_goal)

    def plan(self, goal: GoalGPS) -> GlobalPlan:
        """对给定 goal 规划路径，返回 GlobalPlan 消息（独立方法便于单元测试）"""
        result = GlobalPlan()
        result.goal_lat = goal.latitude
        result.goal_lon = goal.longitude
        result.goal_source = goal.source
        result.header = Header()
        result.header.stamp = self.get_clock().now().to_msg()

        # 检查 1：last_fix 是否有效
        if self.last_fix is None:
            result.status = 'ALL_FAILED'
            result.error_message = 'no_fix_yet'
            return result

        start_lat = self.last_fix.latitude
        start_lon = self.last_fix.longitude
        result.start_lat = start_lat
        result.start_lon = start_lon

        # 检查 2：距离过近，返回单点
        dist = haversine_m(start_lat, start_lon, goal.latitude, goal.longitude)
        if dist < self.min_goal_distance_m:
            p = Point()
            p.x = goal.longitude
            p.y = goal.latitude
            p.z = 0.0
            result.path_wgs84 = [p]
            result.distance_meters = 0.0
            result.duration_seconds = 0.0
            result.source = 'noop'
            result.status = 'OK'
            return result

        # 找起终点最近边（X=lon, Y=lat），不是节点
        try:
            start_edge = ox.distance.nearest_edges(self.G, X=[start_lon], Y=[start_lat])[0]
            goal_edge = ox.distance.nearest_edges(self.G, X=[goal.longitude], Y=[goal.latitude])[0]
        except Exception as e:
            result.status = 'ALL_FAILED'
            result.error_message = f'nearest_edges_failed: {e}'
            return result

        # 把点击位置投影到所在边上
        try:
            s_proj_lat, s_proj_lon, s_frac = project_onto_edge(self.G, start_edge, start_lat, start_lon)
            g_proj_lat, g_proj_lon, g_frac = project_onto_edge(self.G, goal_edge, goal.latitude, goal.longitude)
        except Exception as e:
            result.status = 'ALL_FAILED'
            result.error_message = f'project_failed: {e}'
            return result

        s_u, s_v, _ = start_edge
        g_u, g_v, _ = goal_edge
        s_edge_len = float(self.G.edges[start_edge].get('length', 0.0))
        g_edge_len = float(self.G.edges[goal_edge].get('length', 0.0))

        # 4 候选锚点（start_u/start_v × goal_u/goal_v），选总长最短
        best_total = float('inf')
        best_route = None
        best_s_anchor = None
        best_g_anchor = None

        for s_anchor in (s_u, s_v):
            s_partial = s_edge_len * s_frac if s_anchor == s_u else s_edge_len * (1.0 - s_frac)
            for g_anchor in (g_u, g_v):
                g_partial = g_edge_len * g_frac if g_anchor == g_u else g_edge_len * (1.0 - g_frac)
                try:
                    route = nx.shortest_path(self.G, s_anchor, g_anchor, weight='length')
                except (nx.NetworkXNoPath, nx.NodeNotFound):
                    continue

                # 路径长度（节点间边的 length 之和）
                route_len = 0.0
                for u, v in zip(route[:-1], route[1:]):
                    ed = self.G.get_edge_data(u, v)
                    if ed:
                        first_key = next(iter(ed))
                        route_len += float(ed[first_key].get('length', 0.0))

                # 同边的特殊处理：4 候选里 (u,u) 和 (v,v) 的 route_len=0
                # 但实际几何距离需要考虑两个投影点之间的相对位置
                if start_edge == goal_edge and len(route) == 1:
                    # 同节点：投影点之间的真实距离
                    if s_anchor == s_u:  # g_anchor 也是 u
                        if s_frac >= g_frac:
                            total = (s_frac - g_frac) * s_edge_len  # 直接相连
                        else:
                            total = (s_frac + g_frac) * s_edge_len  # 经 u 绕回
                    else:  # 同 v
                        if s_frac <= g_frac:
                            total = (g_frac - s_frac) * s_edge_len  # 直接相连
                        else:
                            total = (2 - s_frac - g_frac) * s_edge_len  # 经 v 绕回
                else:
                    total = s_partial + route_len + g_partial

                if total < best_total:
                    best_total = total
                    best_route = route
                    best_s_anchor = s_anchor
                    best_g_anchor = g_anchor

        if best_route is None:
            result.status = 'NO_ROUTE'
            result.error_message = 'disconnected'
            return result

        # 构建路径坐标：起点投影 → 路径节点 → 终点投影
        result.path_wgs84 = []

        def _add_point(lon, lat):
            p = Point()
            p.x = lon
            p.y = lat
            p.z = 0.0
            result.path_wgs84.append(p)

        # 起点投影（如果投影恰好在节点上，与路径首个节点重合，可视化上无害）
        _add_point(s_proj_lon, s_proj_lat)

        # 路径节点：跳过与起点投影重合的"反向"节点（同边且 anchor=远离投影端时）
        # 例：start_edge=(u,v)，s_frac=0.3，s_anchor=u：投影已接近 u，路径[u,...]无需重复 u
        # 这种情况下首节点 u 与投影点在同一位置但路径自然展开，不算冗余；保留所有节点即可
        for node_id in best_route:
            node = self.G.nodes[node_id]
            _add_point(node['x'], node['y'])

        # 终点投影
        _add_point(g_proj_lon, g_proj_lat)

        result.distance_meters = best_total
        result.duration_seconds = best_total / self.walking_speed_mps if self.walking_speed_mps > 0 else 0.0
        result.source = 'local_networkx'
        result.status = 'OK'
        return result


def main(args=None):
    rclpy.init(args=args)
    node = NetworkxPlannerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
