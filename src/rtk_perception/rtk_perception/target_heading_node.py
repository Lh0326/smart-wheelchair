"""目标航向源节点（A 阶段极简版）。

订阅 RViz "2D Goal Pose" 按钮发出的 /initialpose，提取 yaw，
发布 /target_heading (Float64, rad)，作为 VFH+ 的目标方向中心。

C 阶段升级版会订阅 /global_plan + /fix + /heading_imu，做 GPS 路径对齐。
"""
from __future__ import annotations

import math
import signal

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from std_msgs.msg import Float64


class TargetHeadingNode(Node):
    def __init__(self):
        super().__init__("target_heading_node")
        self._sub = self.create_subscription(
            PoseWithCovarianceStamped,
            "/initialpose",
            self._pose_cb,
            10,
        )
        self._pub = self.create_publisher(Float64, "/target_heading", 10)
        self.get_logger().info(
            "TargetHeadingNode started. Use RViz '2D Goal Pose' to set direction."
        )

    def _pose_cb(self, msg: PoseWithCovarianceStamped):
        q = msg.pose.pose.orientation
        # quaternion → yaw
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        out = Float64()
        out.data = yaw
        self._pub.publish(out)
        self.get_logger().info(f"target_heading = {math.degrees(yaw):.1f}°")


def main(args=None):
    rclpy.init(args=args)
    node = TargetHeadingNode()
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
