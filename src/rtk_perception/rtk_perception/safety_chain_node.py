"""Safety chain ROS2 节点。

订阅 /cmd_vel（来自 TEB/controller_server）+ 配置的 LaserScan 输入，
按前方扇形最近障碍施加急停/减速约束，发布 /cmd_vel_safe 给电机层。
"""
from __future__ import annotations

import signal
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from rtk_perception.safety_chain import SafetyConfig, apply_safety_chain
from rtk_perception.vfh_plus import LaserScanData, Twist2D


class SafetyChainNode(Node):
    def __init__(self):
        super().__init__("safety_chain_node")
        self.declare_parameter("emergency_stop_distance", 0.3)
        self.declare_parameter("slowdown_distance", 1.0)
        self.declare_parameter("slowdown_factor", 0.3)
        self.declare_parameter("front_fov_half_deg", 30.0)
        self.declare_parameter("input_scan_topic", "/scan")
        self.declare_parameter("angular_jump_threshold", 0.5)  # rad/s
        self.declare_parameter("scan_timeout_sec", 0.5)

        self._cfg = SafetyConfig(
            emergency_stop_distance=float(
                self.get_parameter("emergency_stop_distance").value),
            slowdown_distance=float(
                self.get_parameter("slowdown_distance").value),
            slowdown_factor=float(
                self.get_parameter("slowdown_factor").value),
            front_fov_half_deg=float(
                self.get_parameter("front_fov_half_deg").value),
        )
        self._latest_scan: Optional[LaserScanData] = None
        self._last_scan_stamp_sec: float = 0.0
        self._scan_timeout_sec = float(self.get_parameter("scan_timeout_sec").value)
        self._latest_cmd: Optional[Twist2D] = None
        # cmd_vel 超时检测:controller_server cancel / 停发时,_latest_cmd 仍保留
        # 最后一次非零 Twist,导致 cmd_vel_safe 持续输出非零 → 轮椅继续走。
        # 加 stamp + 超时阈值(0.5s = 10 个 cycle 没收到,肯定 controller 停了)。
        self._last_cmd_stamp_sec: float = 0.0
        self._cmd_timeout_sec: float = 0.5
        self._angular_jump_threshold = float(
            self.get_parameter("angular_jump_threshold").value)
        self._prev_cmd_vel_angular_z: float = 0.0

        scan_topic = str(self.get_parameter("input_scan_topic").value)

        # 修复:用默认 QoS(depth=10, RELIABLE),兼容所有 publisher
        # 之前 BEST_EFFORT 订阅在 RELIABLE publisher 下可能丢消息导致 _latest_scan=None
        self._scan_sub = self.create_subscription(
            LaserScan, scan_topic, self._scan_cb, 10
        )
        # /cmd_vel 来自 vfh_avoidance_node，RELIABLE
        self._cmd_sub = self.create_subscription(
            Twist, "/cmd_vel", self._cmd_cb, 10
        )
        self._pub = self.create_publisher(Twist, "/cmd_vel_safe", 10)
        # 20 Hz：高频兜底，单次循环延迟 < 50ms
        self._timer = self.create_timer(0.05, self._tick)
        self.get_logger().info(
            "SafetyChainNode started: scan_topic=%s cfg=%s" % (scan_topic, self._cfg)
        )

    def _scan_cb(self, msg: LaserScan):
        # 严格按 angle_increment 重建角度向量，与驱动节点对齐
        n_expected = int(round(
            (msg.angle_max - msg.angle_min) / msg.angle_increment)) + 1
        n = min(n_expected, len(msg.ranges))
        if n <= 0:
            return
        angles = msg.angle_min + np.arange(n) * msg.angle_increment
        self._latest_scan = LaserScanData(
            ranges=np.asarray(msg.ranges[:n], dtype=float),
            angles=angles,
        )
        self._last_scan_stamp_sec = self.get_clock().now().nanoseconds * 1e-9

    def _cmd_cb(self, msg: Twist):
        self._latest_cmd = Twist2D(
            linear_x=msg.linear.x, angular_z=msg.angular.z
        )
        self._last_cmd_stamp_sec = self.get_clock().now().nanoseconds * 1e-9

    def _tick(self):
        if self._latest_scan is None or self._latest_cmd is None:
            # 诊断:每 2 秒打一次状态(避免日志爆炸)
            now = self.get_clock().now().nanoseconds
            if not hasattr(self, "_last_diag_ns") or now - self._last_diag_ns > 2e9:
                self.get_logger().warn(
                    f"safety_chain 等待数据: scan={'有' if self._latest_scan else '无'}, "
                    f"cmd={'有' if self._latest_cmd else '无'}"
                )
                self._last_diag_ns = now
            return

        # cmd_vel 超时:controller cancel/停发时,_latest_cmd 是缓存旧值。
        # 强制 zero 防止"按下清除终点后仍沿路径走"问题。
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        scan_age = now_sec - self._last_scan_stamp_sec
        if scan_age > self._scan_timeout_sec:
            self._pub.publish(Twist())
            self.get_logger().error(
                f"融合雷达超时({scan_age:.2f}s)，安全链强制停车",
                throttle_duration_sec=2.0,
            )
            return
        cmd_age = now_sec - self._last_cmd_stamp_sec
        if cmd_age > self._cmd_timeout_sec:
            msg = Twist()  # 全零 Twist,轮椅立即停
            self._pub.publish(msg)
            # 诊断(节流)
            if not hasattr(self, "_last_stale_diag_ns") or now_sec * 1e9 - getattr(self, "_last_stale_diag_ns", 0) > 2e9:
                self.get_logger().warn(
                    f"cmd_vel 超时({cmd_age:.2f}s 未更新),强制 zero (controller 已停发)",
                    throttle_duration_sec=2.0,
                )
                self._last_stale_diag_ns = now_sec * 1e9
            return

        out = apply_safety_chain(self._latest_cmd, self._latest_scan, self._cfg)

        # === 方向跳变前置监控（spec § 3.4）===
        # 相邻帧 angular.z 跳变超阈值时强制 linear.x=0，让下游 Nav2Protector
        # 有时间响应方向跳变，避免"在第一帧就被冲击"。
        try:
            delta_omega = abs(out.angular_z - self._prev_cmd_vel_angular_z)
            if delta_omega > self._angular_jump_threshold and abs(out.linear_x) > 0.01:
                self.get_logger().warn(
                    f"direction_jump_detected: delta_omega={delta_omega:.2f} rad/s, "
                    f"zeroing linear.x for this frame",
                    throttle_duration_sec=0.5,
                )
                out = type(out)(linear_x=0.0, angular_z=out.angular_z)
        except Exception as e:
            self.get_logger().warn(
                f"safety_chain 方向监控异常，透传 cmd_vel: {e}",
                throttle_duration_sec=1.0,
            )
        finally:
            self._prev_cmd_vel_angular_z = out.angular_z

        msg = Twist()
        msg.linear.x = float(out.linear_x)
        msg.angular.z = float(out.angular_z)
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyChainNode()
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
