"""OSRM 路径规划 ROS2 节点

订阅 /fix（EC20 位置）和 /goal_gps（前端点击终点），
调用 OSRM HTTP API 规划路径，发布 /global_plan。

策略：本地 OSRM 优先 + 公共 OSRM 兜底。
"""
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header
from geometry_msgs.msg import Point
import requests

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


class OsrmPlannerNode(Node):
    def __init__(self):
        super().__init__('osrm_planner')

        # 参数声明
        self.declare_parameter('local_osrm_url', 'http://localhost:5000')
        self.declare_parameter('public_osrm_url', 'https://router.project-osrm.org')
        self.declare_parameter('osrm_profile', 'walking')
        self.declare_parameter('request_timeout', 3.0)
        self.declare_parameter('min_goal_distance_m', 5.0)
        self.declare_parameter('enable_public_fallback', True)

        self.local_osrm_url = self.get_parameter('local_osrm_url').value
        self.public_osrm_url = self.get_parameter('public_osrm_url').value
        self.osrm_profile = self.get_parameter('osrm_profile').value
        self.request_timeout = self.get_parameter('request_timeout').value
        self.min_goal_distance_m = self.get_parameter('min_goal_distance_m').value
        self.enable_public_fallback = self.get_parameter('enable_public_fallback').value

        # 状态
        self.last_fix: Optional[NavSatFix] = None

        # transient_local QoS（latched）：新订阅者立即收到最后一次结果
        latched_qos = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        # 订阅
        self.create_subscription(NavSatFix, '/fix', self._on_fix, 10)
        self.create_subscription(GoalGPS, '/goal_gps', self._on_goal, 10)

        # 发布
        self.plan_pub = self.create_publisher(GlobalPlan, '/global_plan', latched_qos)

        self.get_logger().info(
            f'OSRM planner 启动：local={self.local_osrm_url}, '
            f'public={self.public_osrm_url}, profile={self.osrm_profile}, '
            f'fallback={self.enable_public_fallback}'
        )

    def _on_fix(self, msg: NavSatFix):
        self.last_fix = msg

    def _on_goal(self, msg: GoalGPS):
        plan = self.plan(msg)
        self.plan_pub.publish(plan)
        self.get_logger().info(
            f'规划结果 status={plan.status} source={plan.source} '
            f'distance={plan.distance_meters:.1f}m points={len(plan.path_wgs84)}'
        )

    def plan(self, goal: GoalGPS) -> GlobalPlan:
        """对给定 goal 规划路径，返回 GlobalPlan 消息。

        拆分为独立方法便于单元测试（不依赖 ROS 通信）。
        """
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

        # 检查 2：距离过近（小于阈值），返回单点
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

        # 检查 3：调用本地 OSRM
        try:
            data = self._call_osrm(self.local_osrm_url, start_lat, start_lon,
                                   goal.latitude, goal.longitude)
            if data.get('code') == 'Ok' and data.get('routes'):
                self._parse_route_to_plan(data, result, 'local_osrm')
                return result
            self.get_logger().warn(
                f"本地 OSRM code={data.get('code')} msg={data.get('message')}"
            )
        except requests.RequestException as e:
            self.get_logger().warn(f"本地 OSRM 不可达: {e}")

        # 检查 4：fallback 公共 OSRM
        if self.enable_public_fallback:
            try:
                data = self._call_osrm(self.public_osrm_url, start_lat, start_lon,
                                       goal.latitude, goal.longitude)
                if data.get('code') == 'Ok' and data.get('routes'):
                    self._parse_route_to_plan(data, result, 'public_osrm')
                    return result
                result.status = 'NO_ROUTE'
                result.error_message = data.get('message', 'no_route')
                self.get_logger().warn(
                    f"公共 OSRM code={data.get('code')} msg={data.get('message')}"
                )
                return result
            except requests.RequestException as e:
                result.status = 'ALL_FAILED'
                result.error_message = 'public_unreachable'
                self.get_logger().warn(f"公共 OSRM 不可达: {e}")
                return result

        # fallback 被禁用
        result.status = 'ALL_FAILED'
        result.error_message = 'local_failed'
        return result

    def _build_osrm_url(self, base_url: str, start_lat: float, start_lon: float,
                        goal_lat: float, goal_lon: float) -> str:
        """构造 OSRM route URL（注意 OSRM 是 lon,lat 顺序）"""
        return (
            f"{base_url}/route/v1/{self.osrm_profile}/"
            f"{start_lon},{start_lat};{goal_lon},{goal_lat}"
            f"?overview=full&geometries=geojson"
        )

    def _call_osrm(self, base_url: str, start_lat: float, start_lon: float,
                   goal_lat: float, goal_lon: float) -> dict:
        """调 OSRM HTTP API，返回 JSON dict 或抛 requests 异常"""
        url = self._build_osrm_url(base_url, start_lat, start_lon, goal_lat, goal_lon)
        resp = requests.get(url, timeout=self.request_timeout)
        resp.raise_for_status()
        return resp.json()

    @staticmethod
    def _parse_route_to_plan(data: dict, result: GlobalPlan, source: str):
        """从 OSRM JSON 抽取路径点填入 GlobalPlan"""
        route = data['routes'][0]
        coords = route['geometry']['coordinates']  # [[lon, lat], ...]
        result.path_wgs84 = []
        for lon, lat in coords:
            p = Point()
            p.x = lon
            p.y = lat
            p.z = 0.0
            result.path_wgs84.append(p)
        result.distance_meters = float(route.get('distance', 0.0))
        result.duration_seconds = float(route.get('duration', 0.0))
        result.source = source
        result.status = 'OK'


def main(args=None):
    rclpy.init(args=args)
    node = OsrmPlannerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        # 防御性：节点已被外部 shutdown 时也要安全 destroy
        if rclpy.ok():
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
