#!/usr/bin/env python3
"""GPS 加权融合 + 时间滑窗节点

订阅 EC20 基站和 DX-GP10 流动站的 NavSatFix，做两步处理：
  1. 单帧 HDOP 加权融合（HDOP 越小权重越大）
  2. 时间滑窗平均（默认 10 帧），平滑随机噪声

发布 sensor_msgs/NavSatFix 到 /fix（融合后），发布诊断到 /gps/fusion_diag。

为什么这个比位置域 DGPS 好？
- DGPS: /fix = rover - base + BASE_KNOWN  → 两噪声相加，σ = sqrt(σ_base² + σ_rover²)
- 加权融合: σ_fused = sqrt(σ_base²·σ_rover² / (σ_base² + σ_rover²))  → 比任一单机都小
- 滑窗 N 帧: σ_smoothed = σ_fused / sqrt(N)  → 静止时精度可达亚米级

NavSatFix.position_covariance 透传约定（同 dgps_node）：
  [0] = satellites, [4] = hdop
"""
import collections
import math
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header

from rtk_msgs.msg import FusionStatus


def _read_quality(msg: NavSatFix):
    """从 NavSatFix 提取 satellites / hdop（透传约定）"""
    sats = int(msg.position_covariance[0]) if len(msg.position_covariance) > 0 else 0
    hdop = float(msg.position_covariance[4]) if len(msg.position_covariance) > 4 else 0.0
    return sats, hdop


def fuse_weighted(base, rover, base_hdop, rover_hdop):
    """HDOP²倒数加权融合两个 GPS 位置

    权重 w = 1 / hdop²（HDOP 越小权重越大）
    归一化后加权平均

    返回 (lat, lon, w_base, w_rover)
    """
    # 防止 HDOP=0 导致除零
    hb = max(base_hdop, 0.1)
    hr = max(rover_hdop, 0.1)
    wb = 1.0 / (hb * hb)
    wr = 1.0 / (hr * hr)
    s = wb + wr
    wb_n = wb / s
    wr_n = wr / s
    lat = wb_n * base[0] + wr_n * rover[0]
    lon = wb_n * base[1] + wr_n * rover[1]
    return lat, lon, wb_n, wr_n


class GpsFusionNode(Node):
    def __init__(self):
        super().__init__('gps_fusion_node')

        self.declare_parameter('base_topic', '/gps/base_raw')
        self.declare_parameter('rover_topic', '/gps/rover_raw')
        self.declare_parameter('output_topic', '/fix')
        self.declare_parameter('window_size', 10)
        self.declare_parameter('max_age_sec', 3.0)
        self.declare_parameter('publish_hz', 5.0)
        self.declare_parameter('outlier_threshold_meters', 50.0)  # 两机距离>此值（米）认为 rover 异常

        base_topic = self.get_parameter('base_topic').value
        rover_topic = self.get_parameter('rover_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.window_size = int(self.get_parameter('window_size').value)
        self.max_age_sec = float(self.get_parameter('max_age_sec').value)
        publish_hz = float(self.get_parameter('publish_hz').value)
        self.outlier_threshold = float(self.get_parameter('outlier_threshold_meters').value)

        self.fix_pub = self.create_publisher(NavSatFix, output_topic, 10)
        self.diag_pub = self.create_publisher(FusionStatus, '/gps/fusion_diag', 10)

        self._lock = threading.Lock()
        self._base: Optional[NavSatFix] = None
        self._base_t: float = 0.0
        self._rover: Optional[NavSatFix] = None
        self._rover_t: float = 0.0
        self._window_lat = collections.deque(maxlen=self.window_size)
        self._window_lon = collections.deque(maxlen=self.window_size)

        self.create_subscription(NavSatFix, base_topic, self._on_base, 10)
        self.create_subscription(NavSatFix, rover_topic, self._on_rover, 10)

        period = 1.0 / publish_hz if publish_hz > 0 else 0.2
        self.create_timer(period, self._tick)

        self.get_logger().info(
            f'GPS 融合节点启动：base={base_topic}, rover={rover_topic}, '
            f'output={output_topic}, window={self.window_size}, '
            f'max_age={self.max_age_sec}s, publish={publish_hz}Hz'
        )

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_base(self, msg: NavSatFix):
        with self._lock:
            self._base = msg
            self._base_t = self._now_sec()

    def _on_rover(self, msg: NavSatFix):
        with self._lock:
            self._rover = msg
            self._rover_t = self._now_sec()

    def _publish_single(self, msg: NavSatFix, diag, mode: str):
        """单机 fallback：直接用 msg 位置 + 滑窗，无融合"""
        sats, hdop = _read_quality(msg)
        # 滑窗（仍然平滑）
        self._window_lat.append(msg.latitude)
        self._window_lon.append(msg.longitude)
        smooth_lat = sum(self._window_lat) / len(self._window_lat)
        smooth_lon = sum(self._window_lon) / len(self._window_lon)
        diag.fused_lat = smooth_lat
        diag.fused_lon = smooth_lon
        diag.fused_lat_instant = msg.latitude
        diag.fused_lon_instant = msg.longitude
        diag.window_samples = len(self._window_lat)
        # σ 估计：HDOP × 3m，滑窗后 /√N
        sigma_single = max(hdop, 0.5) * 3.0
        n = max(len(self._window_lat), 1)
        sigma_smooth = sigma_single / math.sqrt(n)
        diag.fused_sigma_meters = sigma_smooth
        # 权重全是单机
        if mode == 'base':
            diag.base_weight = 1.0
            diag.rover_weight = 0.0
        else:
            diag.base_weight = 0.0
            diag.rover_weight = 1.0
        diag.base_rover_meters = 0.0

        # 发布 NavSatFix
        fix = NavSatFix()
        fix.header = Header()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = 'wgs84'
        fix.latitude = smooth_lat
        fix.longitude = smooth_lon
        fix.altitude = msg.altitude
        fix.status.status = 0
        fix.status.service = 1
        cov = sigma_smooth * sigma_smooth
        fix.position_covariance[0] = cov
        fix.position_covariance[4] = cov
        fix.position_covariance[8] = cov * 4.0
        fix.position_covariance_type = 2
        self.fix_pub.publish(fix)

    def _tick(self):
        now = self._now_sec()
        with self._lock:
            base = self._base
            base_age = now - self._base_t if base else float('inf')
            rover = self._rover
            rover_age = now - self._rover_t if rover else float('inf')

        diag = FusionStatus()
        diag.header = Header()
        diag.header.stamp = self.get_clock().now().to_msg()
        diag.window_size = self.window_size
        diag.base_age_sec = base_age
        diag.rover_age_sec = rover_age

        if base:
            diag.base_lat = base.latitude
            diag.base_lon = base.longitude
            diag.base_satellites, diag.base_hdop = _read_quality(base)
            diag.base_fix_valid = base.status.status >= 0
        if rover:
            diag.rover_lat = rover.latitude
            diag.rover_lon = rover.longitude
            diag.rover_satellites, diag.rover_hdop = _read_quality(rover)
            diag.rover_fix_valid = rover.status.status >= 0

        # 状态码：
        #   0 = OK          融合输出（两机都有）
        #   1 = STALE       数据过期
        #   2 = NO_BASE     无 base
        #   3 = NO_ROVER    无 rover
        #   4 = NO_FIX      base 或 rover 未定位
        #   5 = FALLBACK_BASE    仅 base 可用，使用 base 单机位置（rover 缺失或失效）
        #   6 = FALLBACK_ROVER   仅 rover 可用，使用 rover 单机位置
        if base is None and rover is None:
            diag.status = 2
            diag.status_message = 'base 和 rover 都未收到'
        elif base is None:
            # 只有 rover
            if rover_age > self.max_age_sec:
                diag.status = 1
                diag.status_message = f'rover 数据过期 {rover_age:.1f}s'
            elif rover.status.status < 0:
                diag.status = 4
                diag.status_message = 'rover 未定位'
            else:
                self._publish_single(rover, diag, mode='rover')
                diag.status = 6
                diag.status_message = f'仅 rover（base 缺失）σ≈{diag.fused_sigma_meters:.2f}m'
        elif rover is None:
            # 只有 base
            if base_age > self.max_age_sec:
                diag.status = 1
                diag.status_message = f'base 数据过期 {base_age:.1f}s'
            elif base.status.status < 0:
                diag.status = 4
                diag.status_message = 'base 未定位'
            else:
                self._publish_single(base, diag, mode='base')
                diag.status = 5
                diag.status_message = f'仅 base（rover 缺失）σ≈{diag.fused_sigma_meters:.2f}m'
        elif base_age > self.max_age_sec or rover_age > self.max_age_sec:
            diag.status = 1
            diag.status_message = f'数据过期 base_age={base_age:.1f}s rover_age={rover_age:.1f}s'
        elif base.status.status < 0 or rover.status.status < 0:
            diag.status = 4
            diag.status_message = 'base 或 rover 未定位'
        else:
            # 两机距离检查
            dist_m = _haversine(
                base.latitude, base.longitude,
                rover.latitude, rover.longitude,
            )
            diag.base_rover_meters = dist_m

            # Outlier rejection: 两机距离 > 阈值时认为 rover 多径锁定
            if dist_m > self.outlier_threshold:
                self._publish_single(base, diag, mode='base')
                diag.status = 5
                diag.base_rover_meters = dist_m
                diag.status_message = (
                    f'rover 异常（多径？距离 {dist_m:.0f}m > {self.outlier_threshold}m）'
                    f'，fallback base 单机 σ≈{diag.fused_sigma_meters:.2f}m'
                )
            else:
                # 单帧加权融合
                lat_f, lon_f, wb, wr = fuse_weighted(
                    (base.latitude, base.longitude),
                    (rover.latitude, rover.longitude),
                    diag.base_hdop, diag.rover_hdop,
                )
                diag.base_weight = wb
                diag.rover_weight = wr
                diag.fused_lat_instant = lat_f
                diag.fused_lon_instant = lon_f

                # 滑窗
                self._window_lat.append(lat_f)
                self._window_lon.append(lon_f)
                smooth_lat = sum(self._window_lat) / len(self._window_lat)
                smooth_lon = sum(self._window_lon) / len(self._window_lon)
                diag.fused_lat = smooth_lat
                diag.fused_lon = smooth_lon
                diag.window_samples = len(self._window_lat)

                # 输出 NavSatFix（滑窗平滑后）
                fix = NavSatFix()
                fix.header = Header()
                fix.header.stamp = self.get_clock().now().to_msg()
                fix.header.frame_id = 'wgs84'
                fix.latitude = smooth_lat
                fix.longitude = smooth_lon
                b_alt = base.altitude
                r_alt = rover.altitude
                fix.altitude = wb * b_alt + wr * r_alt
                fix.status.status = 0
                fix.status.service = 1
                sigma_b = max(diag.base_hdop, 0.5) * 3.0
                sigma_r = max(diag.rover_hdop, 0.5) * 3.0
                sigma_fused = math.sqrt(sigma_b**2 * sigma_r**2 / (sigma_b**2 + sigma_r**2))
                n = max(len(self._window_lat), 1)
                sigma_smooth = sigma_fused / math.sqrt(n)
                cov = sigma_smooth * sigma_smooth
                fix.position_covariance[0] = cov
                fix.position_covariance[4] = cov
                fix.position_covariance[8] = cov * 4.0
                fix.position_covariance_type = 2
                self.fix_pub.publish(fix)

                diag.fused_sigma_meters = sigma_smooth
                diag.status = 0
                diag.status_message = (
                    f'融合 OK win={len(self._window_lat)}/{self.window_size} '
                    f'σ={sigma_smooth:.2f}m Δ={dist_m:.2f}m'
                )

        self.diag_pub.publish(diag)


def _haversine(lat1, lon1, lat2, lon2):
    R = 6371000.0
    dLat = math.radians(lat2 - lat1)
    dLon = math.radians(lon2 - lon1)
    a = (math.sin(dLat/2) ** 2
         + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2))
         * math.sin(dLon/2) ** 2)
    return 2 * R * math.asin(math.sqrt(a))


def main(args=None):
    rclpy.init(args=args)
    node = GpsFusionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
