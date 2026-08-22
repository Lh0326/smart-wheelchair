"""LD14P 路沿检测 ROS2 节点。

订阅 /scan_ld14p，发布：
  /curb_left_marker, /curb_right_marker (visualization_msgs/Marker)
  /curb_polygon (geometry_msgs/PolygonStamped)
"""
from __future__ import annotations

import signal
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Point32, Polygon, PolygonStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker

from rtk_perception.curb_detector import CurbConfig, CurbLine, detect_curbs


class CurbDetectorNode(Node):
    def __init__(self):
        super().__init__("curb_detector_node")
        self.declare_parameter("delta_r_threshold", 0.15)
        self.declare_parameter("max_curb_range", 5.0)
        self.declare_parameter("min_line_length", 1.0)
        self.declare_parameter("bin_size_deg", 1.0)
        self.declare_parameter("dbscan_eps", 0.3)
        self.declare_parameter("dbscan_min_samples", 3)

        cfg = CurbConfig(
            delta_r_threshold=self.get_parameter("delta_r_threshold").value,
            max_curb_range=self.get_parameter("max_curb_range").value,
            min_line_length=self.get_parameter("min_line_length").value,
            bin_size_deg=self.get_parameter("bin_size_deg").value,
            dbscan_eps=self.get_parameter("dbscan_eps").value,
            dbscan_min_samples=self.get_parameter("dbscan_min_samples").value,
        )
        self._cfg = cfg
        self._no_curb_frame_count = 0

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._sub = self.create_subscription(
            LaserScan, "/scan_ld14p", self._scan_cb, qos
        )
        self._left_pub = self.create_publisher(Marker, "/curb_left_marker", 10)
        self._right_pub = self.create_publisher(Marker, "/curb_right_marker", 10)
        self._polygon_pub = self.create_publisher(PolygonStamped, "/curb_polygon", 10)
        self.get_logger().info(
            "CurbDetectorNode started: /scan_ld14p → "
            "/curb_left_marker, /curb_right_marker, /curb_polygon"
        )

    def _scan_cb(self, msg: LaserScan):
        # 构造 (r, theta) 极坐标数组
        angles = np.arange(
            msg.angle_min,
            msg.angle_max + msg.angle_increment * 0.5,
            msg.angle_increment,
        )
        ranges = np.asarray(msg.ranges, dtype=float)
        n = min(len(angles), len(ranges))
        polar = np.column_stack([ranges[:n], angles[:n]])

        curbs = detect_curbs(polar, self._cfg)

        if not curbs:
            self._no_curb_frame_count += 1
        else:
            self._no_curb_frame_count = 0

        # 降级模式：连续 10 帧无路沿 → 不发布（VFH+ 自动降级）
        if self._no_curb_frame_count >= 10:
            return

        lefts = [c for c in curbs if c.is_left]
        rights = [c for c in curbs if not c.is_left]
        # 每个 side 取最长的（避免多段干扰）
        left: Optional[CurbLine] = max(lefts, key=lambda c: c.length) if lefts else None
        right: Optional[CurbLine] = (
            max(rights, key=lambda c: c.length) if rights else None
        )

        stamp = msg.header.stamp
        if left is not None:
            self._left_pub.publish(
                self._to_marker(left, stamp, "curb_left", (1.0, 1.0, 0.0))
            )
        if right is not None:
            self._right_pub.publish(
                self._to_marker(right, stamp, "curb_right", (1.0, 1.0, 0.0))
            )
        if left is not None or right is not None:
            self._polygon_pub.publish(self._to_polygon(left, right, stamp))

    def _to_marker(
        self, curb: CurbLine, stamp, ns: str, rgb: Tuple[float, float, float]
    ) -> Marker:
        m = Marker()
        m.header = Header()
        m.header.stamp = stamp
        m.header.frame_id = "base_link"
        m.ns = ns
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.lifetime.sec = 0
        m.lifetime.nanosec = 100_000_000  # 100ms
        p1 = Point()
        p1.x = curb.start_x
        p1.y = curb.start_y
        p1.z = 0.15
        p2 = Point()
        p2.x = curb.end_x
        p2.y = curb.end_y
        p2.z = 0.15
        m.points = [p1, p2]
        m.scale.x = 0.05  # line width
        c = ColorRGBA()
        c.r, c.g, c.b, c.a = rgb[0], rgb[1], rgb[2], 1.0
        m.colors = [c, c]
        return m

    def _to_polygon(
        self,
        left: Optional[CurbLine],
        right: Optional[CurbLine],
        stamp,
    ) -> PolygonStamped:
        """构造"道路可通行区域"多边形（4 点：left.start, right.start, right.end, left.end）。"""
        poly = PolygonStamped()
        poly.header = Header()
        poly.header.stamp = stamp
        poly.header.frame_id = "base_link"
        points: List[Tuple[float, float]] = []
        if left is not None:
            points.append((left.start_x, left.start_y))
            points.append((left.end_x, left.end_y))
        if right is not None:
            points.append((right.end_x, right.end_y))
            points.append((right.start_x, right.start_y))
        for x, y in points:
            p = Point32()
            p.x = float(x)
            p.y = float(y)
            p.z = 0.15
            poly.polygon.points.append(p)
        return poly


def main(args=None):
    rclpy.init(args=args)
    node = CurbDetectorNode()
    signal.signal(signal.SIGINT, lambda *_: rclpy.shutdown())
    signal.signal(signal.SIGTERM, lambda *_: rclpy.shutdown())
    try:
        rclpy.spin(node)
    except Exception as e:
        import traceback
        print(f"[curb_detector_node] FATAL: {e}")
        traceback.print_exc()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
