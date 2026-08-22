"""转发 /imu/data.orientation → /imu_pose (PoseStamped)，供 RViz Pose 显示。

RViz Humble 的 rviz_default_plugins 没有 Imu 显示插件，
用 Pose + PoseStamped 显示 IMU 朝向（一个朝向箭头）。
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header


class ImuToPoseNode(Node):
    def __init__(self):
        super().__init__('imu_to_pose')
        self._pub = self.create_publisher(PoseStamped, '/imu_pose', 10)
        self.create_subscription(
            Imu, '/imu/data', self._cb, qos_profile_sensor_data
        )
        self.get_logger().info('imu_to_pose 启动：/imu/data → /imu_pose')

    def _cb(self, msg: Imu):
        out = PoseStamped()
        out.header = msg.header
        out.header.frame_id = 'imu_link'
        out.pose.orientation = msg.orientation
        self._pub.publish(out)


def main():
    rclpy.init()
    node = ImuToPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
