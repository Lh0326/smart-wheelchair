"""多路 LaserScan 融合节点：N10P + LD14P + Gemini → /scan_fused。

复用 ladar-ai laser_merger_node 思路：同角度取 min(r)（最近障碍优先）。
任意一路真实扫描在线即可输出，避免单个外设缺席时阻塞安全链。
"""
from __future__ import annotations

import math
import signal
from dataclasses import dataclass
from typing import Optional

import rclpy
import tf2_ros
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan


@dataclass
class MergeConfig:
    merge_strategy: str = "min_range"


def _normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def _rotate_point(q, x: float, y: float, z: float) -> tuple[float, float, float]:
    """Rotate a point by a geometry_msgs Quaternion."""
    qx, qy, qz, qw = q.x, q.y, q.z, q.w
    # Rotation matrix expanded from quaternion.
    xx = qx * qx
    yy = qy * qy
    zz = qz * qz
    xy = qx * qy
    xz = qx * qz
    yz = qy * qz
    wx = qw * qx
    wy = qw * qy
    wz = qw * qz

    rx = (1 - 2 * (yy + zz)) * x + 2 * (xy - wz) * y + 2 * (xz + wy) * z
    ry = 2 * (xy + wz) * x + (1 - 2 * (xx + zz)) * y + 2 * (yz - wx) * z
    rz = 2 * (xz - wy) * x + 2 * (yz + wx) * y + (1 - 2 * (xx + yy)) * z
    return rx, ry, rz


def _transform_point(tf_msg, x: float, y: float, z: float) -> tuple[float, float, float]:
    rot = tf_msg.transform.rotation
    trans = tf_msg.transform.translation
    rx, ry, rz = _rotate_point(rot, x, y, z)
    return rx + trans.x, ry + trans.y, rz + trans.z


def _resample_to_360bins(scan: dict, num_bins: int = 360) -> list:
    """把任意角度参数的 LaserScan 重采样到固定 num_bins 个 bin（统一 [-π, π]）。

    解决 N10P [-π, π] 5400 点 和 LD14P [0, 2π] 667 点 角度约定不同 + 点数不同的问题。
    每个 bin 取该角度范围内的最小距离。
    """
    bins = [float("inf")] * num_bins
    angle_min = scan.get("angle_min", -math.pi)
    angle_inc = scan.get("angle_increment", 2 * math.pi / num_bins)
    ranges = scan["ranges"]

    for i, r in enumerate(ranges):
        if not math.isfinite(r) or r < 0.1:
            continue
        angle = angle_min + i * angle_inc
        # 归一化到 [-π, π]
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        # 转 bin index（[-π, π] → [0, num_bins)）
        bin_idx = int((angle + math.pi) / (2 * math.pi) * num_bins) % num_bins
        if bins[bin_idx] > r:
            bins[bin_idx] = r
    return bins


def merge_two_scans(scan_a: dict, scan_b: dict, cfg: MergeConfig) -> dict:
    """融合两个 LaserScan（dict 表示）。

    改造（修复关键 bug）：N10P [-π,π] 5400 点 和 LD14P [0,2π] 667 点 角度约定不同 + 点数不同。
    原版直接按索引配对（min(len_a, len_b)），导致 N10P 数据基本被丢弃。
    现在统一重采样到 360 bins（1°/bin，[-π, π]），然后逐 bin 取 min(r)。
    """
    NUM_BINS = 360
    bins_a = _resample_to_360bins(scan_a, NUM_BINS)
    bins_b = _resample_to_360bins(scan_b, NUM_BINS)

    merged_ranges = []
    for i in range(NUM_BINS):
        ra = bins_a[i]
        rb = bins_b[i]
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
        "angle_min": -math.pi,
        "angle_max": math.pi - 2 * math.pi / NUM_BINS,
        "angle_increment": 2 * math.pi / NUM_BINS,
    }


def merge_n_scans(scans: list, cfg: MergeConfig) -> dict:
    """融合 N 个 LaserScan（dict 表示），按 1°/bin 取 min(r)。

    泛化版 merge_two_scans，支持任意数量的 scan（包括 1 个、3 个、N 个）。
    inf 自动跳过；全部 inf 时输出 inf。
    """
    NUM_BINS = 360
    if not scans:
        return {
            "ranges": [float("inf")] * NUM_BINS,
            "angle_min": -math.pi,
            "angle_max": math.pi - 2 * math.pi / NUM_BINS,
            "angle_increment": 2 * math.pi / NUM_BINS,
        }

    all_bins = [_resample_to_360bins(s, NUM_BINS) for s in scans]

    merged_ranges = []
    for i in range(NUM_BINS):
        candidates = [
            all_bins[s][i]
            for s in range(len(scans))
            if math.isfinite(all_bins[s][i])
        ]
        if candidates:
            merged_ranges.append(min(candidates))
        else:
            merged_ranges.append(float("inf"))

    return {
        "ranges": merged_ranges,
        "angle_min": -math.pi,
        "angle_max": math.pi - 2 * math.pi / NUM_BINS,
        "angle_increment": 2 * math.pi / NUM_BINS,
    }


class FusionScanNode(Node):
    def __init__(self):
        super().__init__("fusion_scan_node")
        self.declare_parameter("input_scan_a", "/scan")
        self.declare_parameter("input_scan_b", "/scan_ld14p")
        self.declare_parameter("input_scan_c", "/scan_gemini")
        self.declare_parameter("output_scan", "/scan_fused")
        self.declare_parameter("target_frame", "base_link")

        topic_a = self.get_parameter("input_scan_a").value
        topic_b = self.get_parameter("input_scan_b").value
        topic_c = self.get_parameter("input_scan_c").value
        topic_out = self.get_parameter("output_scan").value
        self._target_frame = str(self.get_parameter("target_frame").value)

        self._cfg = MergeConfig()
        self._scan_a: Optional[LaserScan] = None
        self._scan_b: Optional[LaserScan] = None
        self._scan_c: Optional[LaserScan] = None
        self._last_a_stamp = 0.0
        self._last_b_stamp = 0.0
        self._last_c_stamp = 0.0
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._last_tf_warn: dict[str, float] = {}

        from rclpy.qos import QoSProfile, ReliabilityPolicy
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._sub_a = self.create_subscription(LaserScan, topic_a, self._cb_a, qos)
        self._sub_b = self.create_subscription(LaserScan, topic_b, self._cb_b, qos)
        self._sub_c = self.create_subscription(LaserScan, topic_c, self._cb_c, qos)
        self._pub = self.create_publisher(LaserScan, topic_out, 10)
        self._timer = self.create_timer(0.05, self._tick)
        self.get_logger().info(
            f"FusionScanNode: {topic_a} + {topic_b} + {topic_c} → {topic_out}"
        )

    def _cb_a(self, msg: LaserScan):
        self._scan_a = msg
        # 用节点接收时刻而非消息时间戳判断新鲜度（雷达驱动有时不填 stamp）
        self._last_a_stamp = self.get_clock().now().nanoseconds * 1e-9

    def _cb_b(self, msg: LaserScan):
        self._scan_b = msg
        self._last_b_stamp = self.get_clock().now().nanoseconds * 1e-9

    def _cb_c(self, msg: LaserScan):
        self._scan_c = msg
        self._last_c_stamp = self.get_clock().now().nanoseconds * 1e-9

    def _lookup_transform(self, source_frame: str):
        source_frame = source_frame.lstrip("/")
        target_frame = self._target_frame.lstrip("/")
        if not source_frame or source_frame == target_frame:
            return None
        try:
            return self._tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=0.02),
            )
        except Exception as e:
            now_sec = self.get_clock().now().nanoseconds * 1e-9
            last = self._last_tf_warn.get(source_frame, 0.0)
            if now_sec - last > 2.0:
                self.get_logger().warn(
                    f"等待 TF {target_frame} <- {source_frame}: {e}"
                )
                self._last_tf_warn[source_frame] = now_sec
            return False

    def _resample_scan_in_target(self, scan: LaserScan, num_bins: int) -> Optional[list]:
        """Transform one LaserScan into target_frame and resample by base angle."""
        tf_msg = self._lookup_transform(scan.header.frame_id)
        if tf_msg is False:
            return None

        bins = [float("inf")] * num_bins
        min_r = max(float(scan.range_min), 0.05)
        max_r = float(scan.range_max) if scan.range_max > 0 else float("inf")

        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r) or r < min_r or r > max_r:
                continue

            theta = scan.angle_min + i * scan.angle_increment
            local_x = float(r) * math.cos(theta)
            local_y = float(r) * math.sin(theta)
            local_z = 0.0

            if tf_msg is None:
                base_x, base_y, _ = local_x, local_y, local_z
            else:
                base_x, base_y, _ = _transform_point(
                    tf_msg, local_x, local_y, local_z
                )

            base_range = math.hypot(base_x, base_y)
            if base_range < 0.05:
                continue
            base_angle = _normalize_angle(math.atan2(base_y, base_x))
            idx = int((base_angle + math.pi) / (2 * math.pi) * num_bins) % num_bins
            if bins[idx] > base_range:
                bins[idx] = base_range

        return bins

    def _tick(self):
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        active_scans = []
        for scan, stamp in (
            (self._scan_a, self._last_a_stamp),
            (self._scan_b, self._last_b_stamp),
            (self._scan_c, self._last_c_stamp),
        ):
            if scan is not None and (now_sec - stamp) < 0.5:
                active_scans.append(scan)

        if not active_scans:
            return

        num_bins = 360
        all_bins = []
        for scan in active_scans:
            bins = self._resample_scan_in_target(scan, num_bins)
            if bins is not None:
                all_bins.append(bins)
        if not all_bins:
            return

        merged_ranges = []
        for i in range(num_bins):
            candidates = [bins[i] for bins in all_bins if math.isfinite(bins[i])]
            merged_ranges.append(min(candidates) if candidates else float("inf"))

        out = LaserScan()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._target_frame
        out.angle_min = -math.pi
        out.angle_max = math.pi - 2 * math.pi / num_bins
        out.angle_increment = 2 * math.pi / num_bins
        out.time_increment = 0.0
        out.scan_time = 0.05
        out.range_min = 0.05
        out.range_max = max(scan.range_max for scan in active_scans) + 1.0
        out.ranges = merged_ranges
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = FusionScanNode()
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
