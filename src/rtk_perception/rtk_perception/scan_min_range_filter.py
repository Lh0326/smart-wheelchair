"""LaserScan 屏蔽过滤节点(支持圆形 + 矩形)。

订阅输入 scan,过滤机身反射的点(设为 inf),重新发布。

两种屏蔽模式:
1. 圆形(默认): distance < min_range 的点屏蔽
2. 矩形(rect_*参数 > 0 优先): 在传感器本地 LaserScan 坐标系中
   - x 前方 rect_x_front 米、x 后方 rect_x_back 米
   - y 左侧 rect_y_left 米、y 右侧 rect_y_right 米
   - rect_y 为旧参数,会同时设置左右两侧

用法(LD14P 矩形屏蔽):
  ros2 run rtk_perception scan_min_range_filter \
    --ros-args \
    -p input_topic:=/scan_ld14p_raw \
    -p output_topic:=/scan_ld14p \
    -p rect_x_back:=0.45 \
    -p rect_y_left:=0.26 \
    -p rect_y_right:=0.26
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanMinRangeFilterNode(Node):
    def __init__(self):
        super().__init__('scan_min_range_filter')

        self.declare_parameter('input_topic', '/scan_ld14p_raw')
        self.declare_parameter('output_topic', '/scan_ld14p')
        self.declare_parameter('min_range', 0.0)  # 圆形屏蔽半径,0=不启用
        # 矩形屏蔽(优先于圆形),0=不启用
        self.declare_parameter('rect_x_front', 0.0)
        self.declare_parameter('rect_x_back', 0.0)
        self.declare_parameter('rect_y', 0.0)  # legacy: 左右对称
        self.declare_parameter('rect_y_left', 0.0)
        self.declare_parameter('rect_y_right', 0.0)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self._min_range = float(self.get_parameter('min_range').value)
        self._rect_x_front = float(self.get_parameter('rect_x_front').value)
        self._rect_x_back = float(self.get_parameter('rect_x_back').value)
        rect_y = float(self.get_parameter('rect_y').value)
        self._rect_y_left = float(self.get_parameter('rect_y_left').value)
        self._rect_y_right = float(self.get_parameter('rect_y_right').value)
        if rect_y > 0:
            if self._rect_y_left <= 0:
                self._rect_y_left = rect_y
            if self._rect_y_right <= 0:
                self._rect_y_right = rect_y

        self._pub = self.create_publisher(LaserScan, output_topic, 10)
        self._sub = self.create_subscription(
            LaserScan, input_topic, self._on_scan, 10
        )

        if self._use_rect_mask():
            self.get_logger().info(
                f'scan filter: {input_topic} → {output_topic} '
                f'(矩形屏蔽 前{self._rect_x_front}m 后{self._rect_x_back}m '
                f'左{self._rect_y_left}m 右{self._rect_y_right}m)'
            )
        else:
            self.get_logger().info(
                f'scan filter: {input_topic} → {output_topic} '
                f'(圆形屏蔽 r<{self._min_range}m)'
            )

    def _use_rect_mask(self) -> bool:
        return (
            (self._rect_x_front > 0 or self._rect_x_back > 0)
            and (self._rect_y_left > 0 or self._rect_y_right > 0)
        )

    def _is_in_rect_mask(self, x: float, y: float) -> bool:
        """判断 (x, y) 是否在矩形屏蔽区内。

        矩形: 前方 rect_x_front, 后方 rect_x_back,
        左侧 rect_y_left, 右侧 rect_y_right。
        """
        x_min = -self._rect_x_back
        x_max = self._rect_x_front
        y_min = -self._rect_y_right
        y_max = self._rect_y_left
        return x_min <= x <= x_max and y_min <= y <= y_max

    def _on_scan(self, msg: LaserScan):
        use_rect = self._use_rect_mask()

        filtered = list(msg.ranges)
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment

        for i, r in enumerate(filtered):
            if not math.isfinite(r) or r <= 0:
                continue

            # 圆形屏蔽(快速路径)
            if not use_rect and r < self._min_range:
                filtered[i] = float('inf')
                continue

            # 矩形屏蔽:转笛卡尔坐标判断
            if use_rect:
                theta = angle_min + i * angle_inc
                x = r * math.cos(theta)
                y = r * math.sin(theta)
                if self._is_in_rect_mask(x, y):
                    filtered[i] = float('inf')

        msg.ranges = filtered
        if not use_rect:
            msg.range_min = max(msg.range_min, self._min_range)
        # 刷新 stamp 为当前时间：costmap obstacle_layer 用 stamp 查 TF，
        # 若 filter 处理耗时导致原 stamp 滞后，消息可能被 observation_persistence=0 丢弃，
        # 该 ray 不参与下一帧 raytrace 清除 → 障碍 cell 残留 "卡住不消失"。
        msg.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanMinRangeFilterNode()
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
