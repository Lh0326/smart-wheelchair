"""激光雷达 + 深度相机虚拟扫描融合节点。

订阅两个 LaserScan（/scan 和 /scan_depth），输出融合后的 /scan_fused。
融合策略：同角度取最小距离（最近障碍优先）。
"""
from __future__ import annotations

import math
import signal
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


@dataclass
class MergeConfig:
    merge_strategy: str = "min_range"


def merge_two_scans(scan_a: dict, scan_b: dict, cfg: MergeConfig) -> dict:
    """融合两个 LaserScan（dict 表示）。同索引取最小。

    要求两个 scan 的角度参数一致（实际由上游 depthimage_to_laserscan 配置保证）。
    长度不一致时取较短的。
    """
    ranges_a = scan_a["ranges"]
    ranges_b = scan_b["ranges"]
    n = min(len(ranges_a), len(ranges_b))
    merged_ranges = []
    for i in range(n):
        ra = ranges_a[i]
        rb = ranges_b[i]
        if math.isfinite(ra) and math.isfinite(rb):
            merged_ranges.append(min(ra, rb))
        elif math.isfinite(ra):
            merged_ranges.append(ra)
        elif math.isfinite(rb):
            merged_ranges.append(rb)
        else:
            merged_ranges.append(float("inf"))
    return {
        "ranges": merged_ranges,
        "angle_min": scan_a.get("angle_min", -math.pi),
        "angle_max": scan_a.get("angle_max", math.pi),
        "angle_increment": scan_a.get("angle_increment", 2 * math.pi / max(n - 1, 1)),
    }


class LaserMergerNode(Node):
    def __init__(self):
        super().__init__("laser_merger_node")
        self.declare_parameter("input_scan_a", "/scan")
        self.declare_parameter("input_scan_b", "/scan_depth")
        self.declare_parameter("output_scan", "/scan_fused")
        self.declare_parameter("merge_strategy", "min_range")

        topic_a = self.get_parameter("input_scan_a").value
        topic_b = self.get_parameter("input_scan_b").value
        topic_out = self.get_parameter("output_scan").value

        self._cfg = MergeConfig(
            merge_strategy=self.get_parameter("merge_strategy").value
        )
        self._scan_a: Optional[LaserScan] = None
        self._scan_b: Optional[LaserScan] = None
        self._last_a_stamp = 0.0
        self._last_b_stamp = 0.0

        self._sub_a = self.create_subscription(LaserScan, topic_a, self._cb_a, 10)
        self._sub_b = self.create_subscription(LaserScan, topic_b, self._cb_b, 10)
        self._pub = self.create_publisher(LaserScan, topic_out, 10)
        self._timer = self.create_timer(0.05, self._tick)
        self.get_logger().info(
            f"LaserMergerNode: {topic_a} + {topic_b} → {topic_out}"
        )

    def _cb_a(self, msg: LaserScan):
        self._scan_a = msg
        self._last_a_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _cb_b(self, msg: LaserScan):
        self._scan_b = msg
        self._last_b_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _tick(self):
        if self._scan_a is None:
            return
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        b_active = (
            self._scan_b is not None
            and (now_sec - self._last_b_stamp) < 0.5
        )
        if not b_active:
            self._pub.publish(self._scan_a)
            return

        merged_dict = merge_two_scans(
            {
                "ranges": list(self._scan_a.ranges),
                "angle_min": self._scan_a.angle_min,
                "angle_max": self._scan_a.angle_max,
                "angle_increment": self._scan_a.angle_increment,
            },
            {
                "ranges": list(self._scan_b.ranges),
                "angle_min": self._scan_b.angle_min,
                "angle_max": self._scan_b.angle_max,
                "angle_increment": self._scan_b.angle_increment,
            },
            self._cfg,
        )
        out = LaserScan()
        out.header = self._scan_a.header
        out.angle_min = merged_dict["angle_min"]
        out.angle_max = merged_dict["angle_max"]
        out.angle_increment = merged_dict["angle_increment"]
        out.range_min = self._scan_a.range_min
        out.range_max = self._scan_a.range_max
        out.ranges = merged_dict["ranges"]
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = LaserMergerNode()
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
