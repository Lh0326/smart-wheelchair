"""DetectionArray → PointCloud2,接入 obstacle_layer 第 4 个 source。

每个检测中心转 1 个 PointCloud2 点(在 base_link 系)。
差异化代价通过 intensity 字段编码:
  - 行人/自行车/汽车: intensity=255(动态)
  - 椅子/桌子:        intensity=128(静态)
"""
import struct
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

from rtk_msgs.msg import DetectionArray


# 动态类(person/bicycle/car/motorcycle/bus/truck)走 intensity=255
DYNAMIC_CLASSES = {0, 1, 2, 3, 5, 7}

# 距离阈值(m),超出丢弃
MAX_RANGE_M = 8.0


class DetectionToCloudNode(Node):
    def __init__(self):
        super().__init__('detection_to_cloud')
        self._sub = self.create_subscription(
            DetectionArray, '/detections', self._on_det, 10
        )
        self._pub = self.create_publisher(PointCloud2, '/detection_cloud', 10)
        self.get_logger().info('detection_to_cloud ready')

    def _on_det(self, msg):
        if not msg.detections:
            return

        points = []
        for d in msg.detections:
            planar_dist = math.hypot(d.center_3d.x, d.center_3d.y)
            if d.distance_m < 0 or planar_dist > MAX_RANGE_M:
                continue
            intensity = 255.0 if d.class_id in DYNAMIC_CLASSES else 128.0
            # center_3d 在 base_link 系
            points.append([
                d.center_3d.x, d.center_3d.y, d.center_3d.z, intensity
            ])

        if not points:
            return

        cloud = PointCloud2()
        cloud.header = Header()
        cloud.header.frame_id = msg.header.frame_id or 'base_link'
        cloud.header.stamp = msg.header.stamp
        cloud.height = 1
        cloud.width = len(points)
        cloud.fields = [
            PointField(name='x',         offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',         offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',         offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 16   # 4 字节 × 4 字段
        cloud.row_step = 16 * len(points)
        cloud.is_dense = True
        cloud.data = b''.join(struct.pack('ffff', *p) for p in points)
        self._pub.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = DetectionToCloudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
