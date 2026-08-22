#!/usr/bin/env python3
"""
DX-GP10-A 流动站 GNSS 节点 - 发布 /gps/rover_raw 和 /heading_cog

DX-GP10-A 特性（与 EC20 不同）：
  - 默认波特率 9600（EC20 是 115200）
  - 开机自动输出 NMEA，不需要 AT 命令启动
  - 标准 NMEA 0183 协议，多系统 $GN talker

本节点继承 EC20GnssNode 的 NMEA 解析逻辑（_parse_nmea/_parse_coord/_parse_gga/_parse_rmc），
只重写 __init__：去掉 AT 启动调用，改默认参数，发布到 /gps/rover_raw 而不是 /fix。

参数：
  nmea_port  - NMEA 数据串口（默认 /dev/ttyUSB0，DX-GP10-A 通过 CH340 USB 转串口接入）
  baudrate   - 串口波特率（默认 9600，DX-GP10-A 出厂值）
"""
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64

from rtk_gnss.ec20_gnss_node import EC20GnssNode


class Dxgp10GnssNode(EC20GnssNode):
    def __init__(self):
        # 不调 super().__init__()：避免触发父类的 _start_gnss() AT 命令逻辑
        # 直接初始化 rclpy Node 基类，节点名改为 dxgp10_gnss
        Node.__init__(self, 'dxgp10_gnss')

        # DX-GP10-A 默认参数（与 EC20 不同）
        self.declare_parameter('nmea_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 9600)

        self.at_port = None  # DX-GP10-A 无 AT 端口
        self.nmea_port = self.get_parameter('nmea_port').value
        self.baudrate = int(self.get_parameter('baudrate').value)

        # 流动站发布到 /gps/rover_raw（区别于 EC20 的 /fix）
        self.fix_pub = self.create_publisher(NavSatFix, '/gps/rover_raw', 10)
        self.heading_pub = self.create_publisher(Float64, '/heading_cog', 10)

        self._satellites = 0
        self._has_fix = False
        self._last_fix_log = 0.0

        # DX-GP10-A 开机自动输出 NMEA，不需要 _start_gnss()
        self.get_logger().info(
            f'DX-GP10-A 流动站节点启动，串口 {self.nmea_port}@{self.baudrate}bps，'
            f'发布 /gps/rover_raw + /heading_cog'
        )

        # 启动 NMEA 读取线程（_nmea_loop 继承自父类）
        self._running = True
        self._thread = threading.Thread(target=self._nmea_loop, daemon=True)
        self._thread.start()

        # 周期性状态日志（_status_log 继承自父类）
        self.create_timer(5.0, self._status_log)


def main():
    rclpy.init()
    node = Dxgp10GnssNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._running = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
