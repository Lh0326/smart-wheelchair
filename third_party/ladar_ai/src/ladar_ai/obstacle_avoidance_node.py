"""避障主节点：把 VFH+ 算法接入 ROS2 话题。

订阅：
    /scan_fused           融合后的 LaserScan
    /bci_cmd_vel          BCI 意图（Twist）
    /teleop_cmd_vel       teleop 意图（Twist）
    /voice_motion_intent  语音意图（Twist）
    /odom                 里程计（用于当前速度反馈，可选）
发布：
    /cmd_vel              最终安全 Twist → Gazebo/底盘
    /avoidance/status     JSON 状态
"""
from __future__ import annotations

import json
import signal
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from ladar_ai.intent_arbiter import ArbiterConfig, IntentArbiter
from ladar_ai.vfh_plus import (
    LaserScanData,
    Twist2D,
    VFHConfig,
    VFHPlus,
)


class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__("obstacle_avoidance_node")

        self.declare_parameter("safety_distance_m", 0.5)
        self.declare_parameter("danger_distance_m", 1.5)
        self.declare_parameter("sector_deg", 5.0)
        self.declare_parameter("threshold_low", 0.2)
        self.declare_parameter("threshold_high", 0.9)
        self.declare_parameter("max_turn_rate_rad_s", 0.8)
        self.declare_parameter("max_speed_m_s", 0.6)
        self.declare_parameter("min_turn_radius_m", 0.5)
        self.declare_parameter("cost_weight_target", 1.0)
        self.declare_parameter("cost_weight_heading", 0.3)
        self.declare_parameter("angular_kp", 1.2)
        self.declare_parameter("front_fov_half_deg", 60.0)
        self.declare_parameter("cost_weight_prev_dir", 0.6)
        self.declare_parameter("angular_smoothing_alpha", 0.4)
        self.declare_parameter("hysteresis_dead_band_rad", 0.15)
        self.declare_parameter("intent_timeout_teleop_sec", 2.0)
        self.declare_parameter("intent_timeout_bci_sec", 0.5)
        self.declare_parameter("intent_timeout_voice_sec", 1.0)
        self.declare_parameter("cruise_speed_m_s", 0.0)
        self.declare_parameter("bci_max_speed_m_s", 0.4)
        self.declare_parameter("control_loop_hz", 20.0)

        vfh_cfg = VFHConfig(
            safety_distance_m=self.get_parameter("safety_distance_m").value,
            danger_distance_m=self.get_parameter("danger_distance_m").value,
            sector_deg=self.get_parameter("sector_deg").value,
            threshold_low=self.get_parameter("threshold_low").value,
            threshold_high=self.get_parameter("threshold_high").value,
            max_turn_rate_rad_s=self.get_parameter("max_turn_rate_rad_s").value,
            max_speed_m_s=self.get_parameter("max_speed_m_s").value,
            min_turn_radius_m=self.get_parameter("min_turn_radius_m").value,
            cost_weight_target=self.get_parameter("cost_weight_target").value,
            cost_weight_heading=self.get_parameter("cost_weight_heading").value,
            angular_kp=self.get_parameter("angular_kp").value,
            front_fov_half_deg=self.get_parameter("front_fov_half_deg").value,
            cost_weight_prev_dir=self.get_parameter("cost_weight_prev_dir").value,
            angular_smoothing_alpha=self.get_parameter("angular_smoothing_alpha").value,
            hysteresis_dead_band_rad=self.get_parameter("hysteresis_dead_band_rad").value,
        )
        arb_cfg = ArbiterConfig(
            intent_timeout_teleop_sec=self.get_parameter("intent_timeout_teleop_sec").value,
            intent_timeout_bci_sec=self.get_parameter("intent_timeout_bci_sec").value,
            intent_timeout_voice_sec=self.get_parameter("intent_timeout_voice_sec").value,
            cruise_speed_m_s=self.get_parameter("cruise_speed_m_s").value,
            bci_max_speed_m_s=self.get_parameter("bci_max_speed_m_s").value,
        )

        self._vfh = VFHPlus(vfh_cfg)
        self._arbiter = IntentArbiter(arb_cfg)
        self._latest_scan: Optional[LaserScanData] = None
        self._current_twist = Twist2D()

        self._scan_sub = self.create_subscription(
            LaserScan, "/scan_fused", self._scan_cb, 10
        )
        self._bci_sub = self.create_subscription(
            Twist, "/bci_cmd_vel", lambda m: self._intent_cb("bci", m), 10
        )
        self._teleop_sub = self.create_subscription(
            Twist, "/teleop_cmd_vel", lambda m: self._intent_cb("teleop", m), 10
        )
        self._voice_sub = self.create_subscription(
            Twist, "/voice_motion_intent",
            lambda m: self._intent_cb("voice", m), 10
        )
        self._odom_sub = self.create_subscription(
            Odometry, "/odom", self._odom_cb, 10
        )

        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._status_pub = self.create_publisher(String, "/avoidance/status", 10)

        hz = self.get_parameter("control_loop_hz").value
        self._timer = self.create_timer(1.0 / hz, self._control_tick)
        self.get_logger().info("ObstacleAvoidanceNode started")

    def _scan_cb(self, msg: LaserScan):
        angles = np.arange(
            msg.angle_min,
            msg.angle_max + msg.angle_increment * 0.5,
            msg.angle_increment,
        )
        n = min(len(angles), len(msg.ranges))
        self._latest_scan = LaserScanData(
            ranges=np.asarray(msg.ranges[:n], dtype=float),
            angles=np.asarray(angles[:n], dtype=float),
        )

    def _intent_cb(self, source: str, msg: Twist):
        now = self.get_clock().now().nanoseconds * 1e-9
        self._arbiter.update(
            source,
            Twist2D(linear_x=msg.linear.x, angular_z=msg.angular.z),
            now,
        )

    def _odom_cb(self, msg: Odometry):
        self._current_twist = Twist2D(
            linear_x=msg.twist.twist.linear.x,
            angular_z=msg.twist.twist.angular.z,
        )

    def _control_tick(self):
        if self._latest_scan is None:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        intent = self._arbiter.get_active_intent(now)

        if intent.source == "emergency_stop":
            self._publish(Twist2D(0.0, 0.0), "emergency_stop")
            return

        twist2d = self._vfh.compute(
            self._latest_scan, intent.twist, self._current_twist
        )
        self._publish(twist2d, intent.source)

    def _publish(self, twist2d: Twist2D, source: str):
        # 关键修复：v=0 时强制 w=0。
        # 否则 Gazebo 差速底盘会原地旋转（因为收到 (0, w≠0)），
        # 机器人在 VFH 算法每帧 w 变化的影响下原地左右转——这就是用户看到的"摇晃"。
        if abs(twist2d.linear_x) < 0.01:
            twist2d.angular_z = 0.0

        msg = Twist()
        msg.linear.x = float(twist2d.linear_x)
        msg.angular.z = float(twist2d.angular_z)
        self._cmd_pub.publish(msg)

        status = {
            "source": source,
            "linear_x": round(float(twist2d.linear_x), 3),
            "angular_z": round(float(twist2d.angular_z), 3),
            "brake": bool(twist2d.linear_x == 0.0),
        }
        s = String()
        s.data = json.dumps(status, ensure_ascii=False)
        self._status_pub.publish(s)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    signal.signal(signal.SIGINT, lambda *_: rclpy.shutdown())
    signal.signal(signal.SIGTERM, lambda *_: rclpy.shutdown())
    try:
        rclpy.spin(node)
    except Exception as e:
        import traceback
        print(f"[obstacle_avoidance_node] FATAL: {e}")
        traceback.print_exc()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
