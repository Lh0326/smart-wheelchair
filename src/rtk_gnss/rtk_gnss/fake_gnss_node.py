#!/usr/bin/env python3
"""
fake_gnss_node - 模拟 GNSS 定位节点（演示用）

发布：
  /fix          (sensor_msgs/NavSatFix)  - 模拟位置，1Hz
  /heading_cog  (std_msgs/Float64)       - 模拟航向（度），1Hz

默认行为：以昆工呈贡校区中心 (24.8551, 102.8553) 为起点，
         沿矩形轨迹缓慢移动（每秒前进约 0.5m），航向沿运动方向。

参数：
  center_lat / center_lon - 轨迹中心
  speed_mps               - 移动速度（米/秒）
  rect_width / rect_height - 矩形轨迹尺寸（米）
  publish_hz              - 发布频率

切换真实 RTK 时：禁用本节点，启动 nmea_navsat_driver 接 RTK 模块即可，
                  前端订阅 /fix 和 /heading_cog 接口不变。
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64


# 米/纬度（用于把米制偏移转换为经纬度增量）
M_PER_DEG_LAT = 111320.0  # 纬度 1 度约 111.32 km
# 经度 1 度的米数依赖纬度
def m_per_deg_lng(lat: float) -> float:
    return M_PER_DEG_LAT * math.cos(math.radians(lat))


class FakeGnssNode(Node):
    def __init__(self):
        super().__init__('fake_gnss')

        self.declare_parameter('center_lat', 24.8551)
        self.declare_parameter('center_lon', 102.8553)
        self.declare_parameter('speed_mps', 0.5)
        self.declare_parameter('rect_width', 40.0)    # 矩形宽 40m
        self.declare_parameter('rect_height', 30.0)   # 矩形高 30m
        self.declare_parameter('publish_hz', 1.0)

        self.center_lat = self.get_parameter('center_lat').value
        self.center_lon = self.get_parameter('center_lon').value
        self.speed = self.get_parameter('speed_mps').value
        self.rect_w = self.get_parameter('rect_width').value
        self.rect_h = self.get_parameter('rect_height').value
        hz = self.get_parameter('publish_hz').value

        self.fix_pub = self.create_publisher(NavSatFix, '/fix', 10)
        self.heading_pub = self.create_publisher(Float64, '/heading_cog', 10)

        # 矩形周长 = 2*(w+h)，走完一圈用时 = 周长 / 速度
        perimeter = 2 * (self.rect_w + self.rect_h)
        self.period_sec = perimeter / self.speed if self.speed > 0 else 1e9
        self.t0 = self.get_clock().now().nanoseconds * 1e-9

        self.timer = self.create_timer(1.0 / hz, self._tick)
        self.get_logger().info(
            f"fake_gnss 启动：中心 ({self.center_lat}, {self.center_lon}), "
            f"矩形 {self.rect_w}x{self.rect_h}m, 速度 {self.speed} m/s, "
            f"周期 {self.period_sec:.1f}s"
        )

    def _tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        t = (now - self.t0) % self.period_sec
        # 沿矩形轨迹（顺时针）
        # 4 段：东→南→西→北
        w, h = self.rect_w, self.rect_h
        seg1 = w / self.speed  # 向东
        seg2 = h / self.speed  # 向南
        seg3 = w / self.speed  # 向西
        seg4 = h / self.speed  # 向北

        if t < seg1:
            # 向东
            dx = t * self.speed - w / 2
            dy = -h / 2
            heading = 90.0  # 东
        elif t < seg1 + seg2:
            # 向南
            dx = w / 2
            dy = -h / 2 + (t - seg1) * self.speed
            heading = 180.0  # 南
        elif t < seg1 + seg2 + seg3:
            # 向西
            dx = w / 2 - (t - seg1 - seg2) * self.speed
            dy = h / 2
            heading = 270.0  # 西
        else:
            # 向北
            dx = -w / 2
            dy = h / 2 - (t - seg1 - seg2 - seg3) * self.speed
            heading = 0.0  # 北

        # 米制偏移转经纬度
        lat = self.center_lat + dy / M_PER_DEG_LAT
        lon = self.center_lon + dx / m_per_deg_lng(self.center_lat)

        # 发布 NavSatFix
        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = 'wgs84'
        fix.latitude = lat
        fix.longitude = lon
        fix.altitude = 1950.0  # 昆明海拔约 1950m
        fix.status.status = 0  # STATUS_FIX
        fix.status.service = 1  # SERVICE_GPS
        self.fix_pub.publish(fix)

        # 发布航向
        heading_msg = Float64()
        heading_msg.data = heading
        self.heading_pub.publish(heading_msg)


def main():
    rclpy.init()
    node = FakeGnssNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
