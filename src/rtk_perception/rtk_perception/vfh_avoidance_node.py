"""VFH+ 避障主节点（基于 ladar-ai obstacle_avoidance_node 改造）。

订阅：
  /scan_fused         双雷达融合 LaserScan
  /target_heading     目标方向 (Float64, rad)
  /curb_polygon       路沿约束多边形（可选）
发布：
  /cmd_vel            最终 Twist 指令
  /vfh_histogram      极坐标直方图 (Marker)
  /vfh_candidate      推荐转向箭头 (Marker)
"""
from __future__ import annotations

import math
import signal
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PolygonStamped, Point
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA, Float64, Header
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist

from rtk_perception.vfh_plus import (
    LaserScanData,
    Twist2D,
    VFHConfig,
    VFHPlus,
    angle_to_sector,
    build_histogram,
)


class VfhAvoidanceNode(Node):
    def __init__(self):
        super().__init__("vfh_avoidance_node")

        # 声明参数
        self.declare_parameter("safety_distance_m", 0.5)
        self.declare_parameter("danger_distance_m", 1.5)
        self.declare_parameter("sector_deg", 5.0)
        self.declare_parameter("threshold_low", 0.2)
        self.declare_parameter("threshold_high", 0.9)
        self.declare_parameter("max_turn_rate_rad_s", 0.8)
        self.declare_parameter("max_speed_m_s", 0.6)
        self.declare_parameter("cost_weight_target", 1.0)
        self.declare_parameter("cost_weight_heading", 0.3)
        self.declare_parameter("cost_weight_prev_dir", 0.6)
        self.declare_parameter("cost_weight_curbs", 2.0)
        self.declare_parameter("front_fov_half_deg", 60.0)
        self.declare_parameter("hysteresis_dead_band_rad", 0.15)
        self.declare_parameter("angular_smoothing_alpha", 0.4)
        self.declare_parameter("cruise_speed_m_s", 0.3)
        self.declare_parameter("control_loop_hz", 20.0)
        # LD14P 机身遮挡屏蔽矩形（轮椅自身遮挡区域不纳入 VFH+ histogram）
        # 以 LD14P 为几何中心，ROS 坐标系 x=前 y=左
        self.declare_parameter("ld14p_shield_front_m", 0.0)    # 正前方不屏蔽
        self.declare_parameter("ld14p_shield_back_m", 0.45)    # 正后方 45cm
        self.declare_parameter("ld14p_shield_side_m", 0.26)    # 左右各 26cm

        cfg = VFHConfig(
            safety_distance_m=self.get_parameter("safety_distance_m").value,
            danger_distance_m=self.get_parameter("danger_distance_m").value,
            sector_deg=self.get_parameter("sector_deg").value,
            threshold_low=self.get_parameter("threshold_low").value,
            threshold_high=self.get_parameter("threshold_high").value,
            max_turn_rate_rad_s=self.get_parameter("max_turn_rate_rad_s").value,
            max_speed_m_s=self.get_parameter("max_speed_m_s").value,
            cost_weight_target=self.get_parameter("cost_weight_target").value,
            cost_weight_heading=self.get_parameter("cost_weight_heading").value,
            cost_weight_prev_dir=self.get_parameter("cost_weight_prev_dir").value,
            cost_weight_curbs=self.get_parameter("cost_weight_curbs").value,
            front_fov_half_deg=self.get_parameter("front_fov_half_deg").value,
            hysteresis_dead_band_rad=self.get_parameter("hysteresis_dead_band_rad").value,
            angular_smoothing_alpha=self.get_parameter("angular_smoothing_alpha").value,
        )
        self._cfg = cfg
        self._vfh = VFHPlus(cfg)
        self._latest_scan: Optional[LaserScanData] = None
        self._latest_scan_ld14p: Optional[LaserScanData] = None
        self._target_heading: Optional[float] = None
        self._curb_mask: Optional[np.ndarray] = None
        self._cruise_speed = self.get_parameter("cruise_speed_m_s").value
        self._ld14p_shield = {
            "front": self.get_parameter("ld14p_shield_front_m").value,
            "back": self.get_parameter("ld14p_shield_back_m").value,
            "side": self.get_parameter("ld14p_shield_side_m").value,
        }

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        # 同时订阅 N10P 和 LD14P，内部合并后传给 VFH+
        # RViz 分别显示两个雷达（红/青），不发布 /scan_fused（无白色点云）
        self._scan_sub = self.create_subscription(
            LaserScan, "/scan", self._scan_cb, qos
        )
        self._scan_ld14p_sub = self.create_subscription(
            LaserScan, "/scan_ld14p", self._scan_ld14p_cb, qos
        )
        self._heading_sub = self.create_subscription(
            Float64, "/target_heading", self._heading_cb, 10
        )
        self._curb_sub = self.create_subscription(
            PolygonStamped, "/curb_polygon", self._curb_cb, 10
        )

        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._histogram_pub = self.create_publisher(Marker, "/vfh_histogram", 10)
        self._candidate_pub = self.create_publisher(Marker, "/vfh_candidate", 10)

        hz = self.get_parameter("control_loop_hz").value
        self._timer = self.create_timer(1.0 / hz, self._control_tick)
        self.get_logger().info("VfhAvoidanceNode started")

    def _scan_cb(self, msg: LaserScan):
        self._latest_scan = self._laser_scan_to_data(msg)

    def _scan_ld14p_cb(self, msg: LaserScan):
        self._latest_scan_ld14p = self._laser_scan_to_data(msg)

    @staticmethod
    def _laser_scan_to_data(msg: LaserScan) -> LaserScanData:
        """把 ROS LaserScan 消息转为 LaserScanData。"""
        angles = np.arange(
            msg.angle_min,
            msg.angle_max + msg.angle_increment * 0.5,
            msg.angle_increment,
        )
        n = min(len(angles), len(msg.ranges))
        return LaserScanData(
            ranges=np.asarray(msg.ranges[:n], dtype=float),
            angles=np.asarray(angles[:n], dtype=float),
        )

    def _merge_scans(self, n10p: Optional[LaserScanData],
                     ld14p: Optional[LaserScanData]) -> Optional[LaserScanData]:
        """合并 N10P 和 LD14P 到统一的 360 bins（1°/bin，[-π, π]）。

        两个雷达在不同高度（N10P z=1.47m / LD14P z=0.13m），
        扫描的是不同水平面的障碍。
        VFH+ histogram 同时需要覆盖高处和低处的障碍。

        LD14P 数据会过滤机身遮挡区域（屏蔽矩形）：
          - 前方 ld14p_shield_front_m（默认 0cm）
          - 后方 ld14p_shield_back_m（默认 45cm）
          - 左右各 ld14p_shield_side_m（默认 26cm）
        """
        NUM_BINS = 360
        bins = np.full(NUM_BINS, float("inf"))

        def _in_shield(r: float, theta_norm: float) -> bool:
            """检查点是否在 LD14P 机身屏蔽矩形内。

            屏蔽矩形（以 LD14P 为中心，ROS 坐标 x=前 y=左）：
              x ∈ [-shield_back, shield_front]
              |y| ≤ shield_side
            """
            x = r * math.cos(theta_norm)
            y = r * math.sin(theta_norm)
            shield = self._ld14p_shield
            x_in = -shield["back"] <= x <= shield["front"]
            y_in = abs(y) <= shield["side"]
            return x_in and y_in

        def resample_n10p(scan: LaserScanData):
            """N10P 直接重采样（不屏蔽）。"""
            nonlocal bins
            if scan is None or scan.ranges.size == 0:
                return
            for r, theta in zip(scan.ranges, scan.angles):
                if not np.isfinite(r) or r < 0.1:
                    continue
                norm = ((float(theta) + math.pi) % (2 * math.pi)) - math.pi
                idx = int((norm + math.pi) / (2 * math.pi) * NUM_BINS) % NUM_BINS
                if bins[idx] > r:
                    bins[idx] = r

        def resample_ld14p(scan: LaserScanData):
            """LD14P 重采样（过滤机身遮挡区域）。"""
            nonlocal bins
            if scan is None or scan.ranges.size == 0:
                return
            for r, theta in zip(scan.ranges, scan.angles):
                if not np.isfinite(r) or r < 0.1:
                    continue
                norm = ((float(theta) + math.pi) % (2 * math.pi)) - math.pi
                # 机身屏蔽：在屏蔽矩形内的点跳过（轮椅自身遮挡，不是障碍）
                if _in_shield(float(r), norm):
                    continue
                idx = int((norm + math.pi) / (2 * math.pi) * NUM_BINS) % NUM_BINS
                if bins[idx] > r:
                    bins[idx] = r

        resample_n10p(n10p)
        resample_ld14p(ld14p)

        angles = np.linspace(-math.pi, math.pi - 2 * math.pi / NUM_BINS, NUM_BINS)
        return LaserScanData(ranges=bins, angles=angles)

    def _heading_cb(self, msg: Float64):
        self._target_heading = float(msg.data)

    def _curb_cb(self, msg: PolygonStamped):
        """把 curb_polygon 外部的扇区标记为禁止。"""
        if not msg.polygon.points:
            self._curb_mask = None
            return
        xs = [p.x for p in msg.polygon.points]
        ys = [p.y for p in msg.polygon.points]
        thetas = [math.atan2(y, x) for x, y in zip(xs, ys) if x * x + y * y > 0.01]
        if not thetas:
            self._curb_mask = None
            return
        theta_min = min(thetas)
        theta_max = max(thetas)
        if theta_max - theta_min > math.pi:
            theta_min, theta_max = theta_max, theta_min + 2 * math.pi
        mask = np.ones(self._cfg.num_sectors, dtype=int)
        for i in range(self._cfg.num_sectors):
            sector_angle = math.radians(i * self._cfg.sector_deg)
            while sector_angle < theta_min:
                sector_angle += 2 * math.pi
            if theta_min <= sector_angle <= theta_max:
                mask[i] = 0
        self._curb_mask = mask

    def _control_tick(self):
        if self._latest_scan is None and self._latest_scan_ld14p is None:
            return

        # 合并 N10P + LD14P 数据（360 bins，覆盖不同高度的障碍）
        merged_scan = self._merge_scans(self._latest_scan, self._latest_scan_ld14p)
        if merged_scan is None:
            return

        intent = Twist2D(linear_x=self._cruise_speed, angular_z=0.0)

        # C1 改造：直接传精确 target_heading 给 VFH+（不再用 ±0.5 二值化 hack）
        # intent.angular_z 仍保留作为"是否要前进"的信号（cruise_speed > 0 即前进）
        target_dir = self._target_heading  # 可能为 None（还没收到 /target_heading）

        # 调试日志（已关闭）
        # if target_dir is not None:
        #     print(f"[DBG1] target_dir={math.degrees(target_dir):+.1f}°", flush=True)

        twist2d = self._vfh.compute(
            merged_scan, intent, Twist2D(),
            target_dir=target_dir,  # 精确目标方向（弧度）
        )

        # 调试日志（已关闭，验证 VFH+ 跟踪 target 成功）
        # print(f"[DBG2] target_dir={math.degrees(target_dir) if target_dir else 0:+.1f}° → VFH out: vx={twist2d.linear_x:.2f} wz={twist2d.angular_z:+.3f}", flush=True)

        # 路沿约束（curb_constraint）已移除：原实现把"转向"误判为"穿越路沿"导致所有转向被禁止
        # 路沿检测的 marker 仍正常显示（黄色虚拟墙），但不阻塞 VFH+ 决策
        # 如果实测发现轮椅冲路沿，再加回更智能的约束（如：只在 best_dir 指向路沿外时减速）
        # if self._curb_mask is not None:
        #     twist2d = self._apply_curb_constraint(twist2d)

        if abs(twist2d.linear_x) < 0.01:
            twist2d.angular_z = 0.0

        self._publish_cmd(twist2d)
        self._publish_debug_markers(twist2d)

    def _apply_curb_constraint(self, twist2d: Twist2D) -> Twist2D:
        """如果当前 angular_z 指向禁止扇区，强制归零。"""
        if twist2d.angular_z > 0.1:
            sector = angle_to_sector(math.pi / 2, self._cfg)
        elif twist2d.angular_z < -0.1:
            sector = angle_to_sector(-math.pi / 2, self._cfg)
        else:
            sector = angle_to_sector(0.0, self._cfg)
        if self._curb_mask[sector] == 1:
            twist2d.angular_z = 0.0
        return twist2d

    def _publish_cmd(self, twist2d: Twist2D):
        msg = Twist()
        msg.linear.x = float(twist2d.linear_x)
        msg.angular.z = float(twist2d.angular_z)
        self._cmd_pub.publish(msg)

    def _publish_debug_markers(self, twist2d: Twist2D):
        # 用合并后的 scan 画 histogram（覆盖 N10P + LD14P）
        merged_scan = self._merge_scans(self._latest_scan, self._latest_scan_ld14p)
        if merged_scan is None:
            return
        stamp = self.get_clock().now().to_msg()

        # Histogram marker
        histogram = build_histogram(merged_scan, self._cfg)
        hist_marker = Marker()
        hist_marker.header = Header()
        hist_marker.header.stamp = stamp
        hist_marker.header.frame_id = "base_link"
        hist_marker.ns = "vfh_histogram"
        hist_marker.id = 0
        hist_marker.type = Marker.LINE_LIST
        hist_marker.action = Marker.ADD
        hist_marker.lifetime.sec = 0
        hist_marker.lifetime.nanosec = 100_000_000
        hist_marker.scale.x = 0.02
        for i in range(self._cfg.num_sectors):
            angle = math.radians(i * self._cfg.sector_deg)
            r_inner = 0.5
            r_outer = 0.5 + histogram[i] * 1.0
            p1 = Point()
            p1.x = r_inner * math.cos(angle)
            p1.y = r_inner * math.sin(angle)
            p1.z = 0.1
            p2 = Point()
            p2.x = r_outer * math.cos(angle)
            p2.y = r_outer * math.sin(angle)
            p2.z = 0.1
            hist_marker.points.extend([p1, p2])
            c = ColorRGBA()
            c.r = float(histogram[i])
            c.g = 0.0
            c.b = 0.0
            c.a = 0.8
            hist_marker.colors.extend([c, c])
        self._histogram_pub.publish(hist_marker)

        # Candidate arrow
        arrow = Marker()
        arrow.header = Header()
        arrow.header.stamp = stamp
        arrow.header.frame_id = "base_link"
        arrow.ns = "vfh_candidate"
        arrow.id = 0
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.lifetime.sec = 0
        arrow.lifetime.nanosec = 100_000_000
        # 箭头方向：优先用 target_heading（来自 GPS 路径），否则用 twist.wz 推断
        direction = 0.0
        if self._target_heading is not None:
            direction = self._target_heading
        elif abs(twist2d.linear_x) > 0.01:
            direction = twist2d.angular_z * 0.5
        # 箭头长度固定 1.0m（让用户能清楚看到方向，不依赖速度大小）
        arrow_length = 1.0
        start = Point()
        start.x = 0.0
        start.y = 0.0
        start.z = 0.3  # 抬高到轮椅上方，避免被点云遮挡
        end = Point()
        end.x = arrow_length * math.cos(direction)
        end.y = arrow_length * math.sin(direction)
        end.z = 0.3
        arrow.points = [start, end]
        # RViz2 ARROW 用 points 模式时 scale 含义：
        #   scale.x = shaft diameter（杆直径）
        #   scale.y = head diameter（头部直径）
        #   scale.z = head length（头部长度，必须 > 0，否则头部不渲染）
        arrow.scale.x = 0.08
        arrow.scale.y = 0.15
        arrow.scale.z = 0.20
        c = ColorRGBA()
        c.r = 0.0
        c.g = 1.0
        c.b = 0.0
        c.a = 1.0
        arrow.color = c
        self._candidate_pub.publish(arrow)


def main(args=None):
    rclpy.init(args=args)
    node = VfhAvoidanceNode()
    signal.signal(signal.SIGINT, lambda *_: rclpy.shutdown())
    signal.signal(signal.SIGTERM, lambda *_: rclpy.shutdown())
    try:
        rclpy.spin(node)
    except Exception as e:
        import traceback
        print(f"[vfh_avoidance_node] FATAL: {e}")
        traceback.print_exc()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
