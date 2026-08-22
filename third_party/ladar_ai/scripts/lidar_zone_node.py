"""雷达 8 方位分区节点。"""
import math
import signal
from typing import Dict, List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from ladar_ai.utils import angle_to_zone, ZONE_NAMES


def smooth_zone_distances(previous: Dict[str, float], current: Dict[str, float],
                          alpha: float = 0.45) -> Dict[str, float]:
    """距离变近时立即响应，距离变远时平滑恢复，减少前端闪烁。"""
    smoothed = {}
    for zone in ZONE_NAMES:
        new_value = current.get(zone, previous.get(zone, 25.0))
        old_value = previous.get(zone, new_value)
        if new_value <= old_value:
            smoothed[zone] = new_value
        else:
            smoothed[zone] = old_value + (new_value - old_value) * alpha
    return smoothed


def zone_from_scan(scan, range_max: float, min_cluster_points: int = 3,
                   cluster_tolerance: float = 0.25,
                   self_filter_min_range: float = 0.15) -> Dict[str, float]:
    """从 LaserScan 计算 8 方位稳健距离。

    使用聚类算法找最近障碍物簇，取最近有效簇的中位数作为该方位距离。
    """
    grouped = {name: [] for name in ZONE_NAMES}
    effective_max = min(range_max, scan.range_max)
    effective_min = max(scan.range_min, self_filter_min_range)
    for i, r in enumerate(scan.ranges):
        if not math.isfinite(r) or r < effective_min or r >= effective_max:
            continue
        angle = scan.angle_min + i * scan.angle_increment
        zone = angle_to_zone(angle)
        grouped[zone].append(float(r))

    result = {}
    for zone in ZONE_NAMES:
        values = sorted(grouped[zone])
        if not values:
            result[zone] = range_max
            continue

        # 聚类：找到所有连续簇
        clusters = []
        cluster = [values[0]]
        for v in values[1:]:
            if v - cluster[-1] <= cluster_tolerance:
                cluster.append(v)
            else:
                clusters.append(cluster)
                cluster = [v]
        clusters.append(cluster)

        # 取最近有效簇的中位数距离，用于障碍物预警
        valid_clusters = [c for c in clusters if len(c) >= min_cluster_points]
        if valid_clusters:
            best_cluster = min(valid_clusters, key=lambda c: c[len(c) // 2])
            mid = len(best_cluster) // 2
            result[zone] = best_cluster[mid]
        else:
            result[zone] = max(values) if values else range_max

    return result


class LidarZoneNode(Node):
    def __init__(self):
        super().__init__("lidar_zone_node")
        self.declare_parameter("range_max", 25.0)
        self.declare_parameter("min_cluster_points", 3)
        self.declare_parameter("cluster_tolerance_m", 0.25)
        self.declare_parameter("self_filter_min_range_m", 0.15)
        self.declare_parameter("smoothing_alpha", 0.45)
        range_max = self.get_parameter("range_max").get_parameter_value().double_value
        self._zones = {name: range_max for name in ZONE_NAMES}
        self._sub = self.create_subscription(LaserScan, "/scan", self._scan_callback, 10)

        # 延迟导入，消息在构建后才可用
        self._pub = None
        self.get_logger().info("LidarZoneNode started, subscribing /scan")

    def _ensure_pub(self):
        if self._pub is None:
            from ladar_ai.msg import ZoneDistances
            self._pub = self.create_publisher(ZoneDistances, "/lidar_zones", 10)

    def _scan_callback(self, msg: LaserScan):
        self._ensure_pub()
        from ladar_ai.msg import ZoneDistances

        range_max = self.get_parameter("range_max").get_parameter_value().double_value
        min_cluster_points = self.get_parameter("min_cluster_points").value
        cluster_tolerance = self.get_parameter("cluster_tolerance_m").value
        self_filter_min_range = self.get_parameter("self_filter_min_range_m").value
        smoothing_alpha = self.get_parameter("smoothing_alpha").value
        zones = zone_from_scan(msg, range_max, min_cluster_points,
                               cluster_tolerance, self_filter_min_range)
        self._zones = smooth_zone_distances(self._zones, zones, smoothing_alpha)

        out = ZoneDistances()
        out.header = msg.header
        out.front_left = self._zones["front_left"]
        out.front = self._zones["front"]
        out.front_right = self._zones["front_right"]
        out.right = self._zones["right"]
        out.rear_right = self._zones["rear_right"]
        out.rear = self._zones["rear"]
        out.rear_left = self._zones["rear_left"]
        out.left = self._zones["left"]
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = LidarZoneNode()
    signal.signal(signal.SIGINT, lambda *_: rclpy.shutdown())
    signal.signal(signal.SIGTERM, lambda *_: rclpy.shutdown())
    try:
        rclpy.spin(node)
    except Exception:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
