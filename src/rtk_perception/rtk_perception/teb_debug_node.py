"""TEB 调试可视化节点。

订阅 /cmd_vel + /nav_path + odom→base_link TF，发布 4 个 RViz Marker：
  - /debug/cmd_vel_arrow : 速度向量箭头（base_link 前方）
  - /debug/trail         : 历史轨迹尾迹（odom 系，30 秒）
  - /debug/deviation_line: 当前位置到 nav_path 的偏离连线
  - /debug/mode_text     : HUD 文字（mode + vx + wz + deviation）

纯函数（build_*_marker）已分离，便于 pytest 单测。
"""
from __future__ import annotations

import math
import signal
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Path
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, Float64
from tf2_ros import TransformException, Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


# ============ 纯函数（不依赖 rclpy，便于单测） ============

def build_cmd_vel_arrow_marker(
    vx: float,
    wz: float,
    stamp_ns: int,
    arrow_scale: float = 2.0,
) -> Marker:
    """构造 base_link 前方速度向量箭头 Marker。

    长度 = |vx| × arrow_scale（让 0.6 m/s 显示为 1.2 m 长箭头）
    颜色：
      vx > 0.05   → 绿色（前进）
      vx < -0.05  → 黄色（后退，理论不会出现）
      |vx| < 0.05 → 红色（停止）
    """
    m = Marker()
    m.header.frame_id = "base_link"
    m.header.stamp.sec = int(stamp_ns // 1e9)
    m.header.stamp.nanosec = int(stamp_ns % 1e9)
    m.ns = "cmd_vel_arrow"
    m.id = 0
    m.type = Marker.ARROW
    m.action = Marker.ADD
    m.lifetime.sec = 0
    m.lifetime.nanosec = 200_000_000  # 200ms

    start = Point(); start.x = 0.0; start.y = 0.0; start.z = 0.5
    end = Point()
    end.x = float(vx * arrow_scale)
    end.y = 0.0
    end.z = 0.5
    m.points = [start, end]

    m.scale.x = 0.08  # 箭头杆直径
    m.scale.y = 0.15  # 箭头头直径

    if vx > 0.05:
        m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0
    elif vx < -0.05:
        m.color.r = 1.0; m.color.g = 1.0; m.color.b = 0.0
    else:
        m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0
    m.color.a = 1.0

    return m


def build_trail_marker(
    trail_points: List[Tuple[float, float, float]],
    stamp_ns: int,
) -> Optional[Marker]:
    """构造历史轨迹尾迹 Marker（odom 系，LINE_STRIP，渐变色）。

    trail_points: [(x, y, t_sec), ...] in odom frame
    返回 None 表示点数不足（< 2）或为空，调用方应跳过发布。
    """
    if len(trail_points) < 2:
        return None

    m = Marker()
    m.header.frame_id = "odom"
    m.header.stamp.sec = int(stamp_ns // 1e9)
    m.header.stamp.nanosec = int(stamp_ns % 1e9)
    m.ns = "trail"
    m.id = 0
    m.type = Marker.LINE_STRIP
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    m.scale.x = 0.05  # 线宽

    n = len(trail_points)
    for i, (x, y, _) in enumerate(trail_points):
        p = Point(); p.x = float(x); p.y = float(y); p.z = 0.05
        m.points.append(p)
        ratio = i / max(n - 1, 1)  # 0=最旧, 1=最新
        c = ColorRGBA()
        c.b = 1.0
        c.r = 0.5 * ratio
        c.a = 0.1 + 0.9 * ratio
        m.colors.append(c)

    return m


def compute_nearest_in_path(
    path_points: List[Tuple[float, float]],
    query_x: float,
    query_y: float,
) -> Optional[Tuple[float, float, float]]:
    """在 (x, y) 点列表中找离 query 最近的点（段插值）。

    对相邻两点构成的每条线段，求 query 在该段上的最近点（投影 + clamp），
    再比较所有段取全局最近。返回 (nearest_x, nearest_y, distance) 或 None。
    """
    if not path_points:
        return None

    best_x, best_y, best_sq = 0.0, 0.0, math.inf

    # 单点退化情形
    if len(path_points) == 1:
        px, py = path_points[0]
        return (px, py, math.hypot(px - query_x, py - query_y))

    for i in range(len(path_points) - 1):
        ax, ay = path_points[i]
        bx, by = path_points[i + 1]
        dx, dy = bx - ax, by - ay
        seg_len2 = dx * dx + dy * dy
        if seg_len2 < 1e-12:
            # 退化段：当作点 a
            cx, cy = ax, ay
        else:
            # 投影比例 t，clamp 到 [0, 1]
            t = ((query_x - ax) * dx + (query_y - ay) * dy) / seg_len2
            t = max(0.0, min(1.0, t))
            cx = ax + t * dx
            cy = ay + t * dy
        ddx, ddy = cx - query_x, cy - query_y
        sq = ddx * ddx + ddy * ddy
        if sq < best_sq:
            best_sq = sq
            best_x, best_y = cx, cy

    return (best_x, best_y, math.sqrt(best_sq))


def build_deviation_line_marker(
    base_x: float,
    base_y: float,
    nearest_x: float,
    nearest_y: float,
    stamp_ns: int,
) -> Marker:
    """构造 base_link 当前位置到 nav_path 最近点的连线 Marker。

    颜色按偏离距离分级：
      < 0.3m  → 绿色（在路径上）
      0.3-1m  → 橙色（轻微偏）
      > 1m    → 红色（远偏，避障中）
    """
    dist = math.hypot(base_x - nearest_x, base_y - nearest_y)

    m = Marker()
    m.header.frame_id = "odom"
    m.header.stamp.sec = int(stamp_ns // 1e9)
    m.header.stamp.nanosec = int(stamp_ns % 1e9)
    m.ns = "deviation_line"
    m.id = 0
    m.type = Marker.LINE_LIST
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    m.scale.x = 0.04  # 线宽

    start = Point(); start.x = float(base_x); start.y = float(base_y); start.z = 0.05
    end = Point(); end.x = float(nearest_x); end.y = float(nearest_y); end.z = 0.05
    m.points = [start, end]

    if dist < 0.3:
        m.color.r = 0.0; m.color.g = 1.0; m.color.b = 0.0
    elif dist < 1.0:
        m.color.r = 1.0; m.color.g = 0.5; m.color.b = 0.0
    else:
        m.color.r = 1.0; m.color.g = 0.0; m.color.b = 0.0
    m.color.a = 1.0

    return m


def build_mode_text_marker(
    mode: str,
    vx: float,
    wz: float,
    deviation: float,
    stamp_ns: int,
) -> Marker:
    """构造 HUD 文字 Marker（base_link 上方 2m，TEXT_VIEW_FACING）。"""
    m = Marker()
    m.header.frame_id = "base_link"
    m.header.stamp.sec = int(stamp_ns // 1e9)
    m.header.stamp.nanosec = int(stamp_ns % 1e9)
    m.ns = "mode_text"
    m.id = 0
    m.type = Marker.TEXT_VIEW_FACING
    m.action = Marker.ADD
    m.pose.position.x = 0.0
    m.pose.position.y = 0.0
    m.pose.position.z = 2.0
    m.pose.orientation.w = 1.0
    m.scale.z = 0.3  # 文字高度
    m.text = f"[{mode}] vx={vx:+.2f} wz={wz:+.2f} dev={deviation:.2f}m"
    m.color.r = 1.0; m.color.g = 1.0; m.color.b = 1.0; m.color.a = 1.0
    return m


# ============ ROS2 节点 ============

class TebDebugNode(Node):
    def __init__(self):
        super().__init__("teb_debug_node")

        self.declare_parameter("update_rate_hz", 20.0)
        self.declare_parameter("trail_duration_sec", 30.0)
        self.declare_parameter("cmd_vel_arrow_scale", 2.0)

        rate = float(self.get_parameter("update_rate_hz").value)
        self._trail_duration = float(self.get_parameter("trail_duration_sec").value)
        self._arrow_scale = float(self.get_parameter("cmd_vel_arrow_scale").value)

        self._latest_vx: float = 0.0
        self._latest_wz: float = 0.0
        self._latest_nav_path: Optional[Path] = None
        self._trail_points: List[Tuple[float, float, float]] = []  # (x, y, t_sec) in odom

        self.create_subscription(Twist, "/cmd_vel", self._cmd_cb, 10)
        self.create_subscription(Path, "/nav_path", self._nav_path_cb, 10)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._cmd_vel_arrow_pub = self.create_publisher(Marker, "/debug/cmd_vel_arrow", 10)
        self._trail_pub = self.create_publisher(Marker, "/debug/trail", 10)
        self._deviation_line_pub = self.create_publisher(Marker, "/debug/deviation_line", 10)
        self._mode_text_pub = self.create_publisher(Marker, "/debug/mode_text", 10)
        self._deviation_pub = self.create_publisher(Float64, "/debug/deviation", 10)
        # trail / deviation / mode_text 在任务 6 中加

        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"TebDebugNode started: rate={rate}Hz, trail={self._trail_duration}s"
        )

    def _cmd_cb(self, msg: Twist):
        self._latest_vx = msg.linear.x
        self._latest_wz = msg.angular.z

    def _nav_path_cb(self, msg: Path):
        self._latest_nav_path = msg

    def _tick(self):
        stamp_ns = self.get_clock().now().nanoseconds

        # 1. cmd_vel_arrow（base_link 系，不需要 TF）
        arrow = build_cmd_vel_arrow_marker(
            vx=self._latest_vx,
            wz=self._latest_wz,
            stamp_ns=stamp_ns,
            arrow_scale=self._arrow_scale,
        )
        self._cmd_vel_arrow_pub.publish(arrow)

        # 2. 查 odom→base_link TF（trail + deviation 都需要）
        base_x_odom, base_y_odom = 0.0, 0.0
        tf_ok = False
        try:
            tf = self._tf_buffer.lookup_transform(
                "odom", "base_link", rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
            base_x_odom = tf.transform.translation.x
            base_y_odom = tf.transform.translation.y
            tf_ok = True
        except TransformException:
            pass

        # 3. trail（累积轨迹 + 渲染）
        if tf_ok:
            now_sec = stamp_ns * 1e-9
            self._trail_points.append((base_x_odom, base_y_odom, now_sec))
            # 裁剪：超过 trail_duration_sec 或上限 1500 点
            self._trail_points = [
                (x, y, t) for x, y, t in self._trail_points
                if now_sec - t < self._trail_duration
            ][-1500:]
            trail = build_trail_marker(self._trail_points, stamp_ns)
            if trail is not None:
                self._trail_pub.publish(trail)

        # 4. deviation_line + mode_text
        deviation = 0.0
        if tf_ok and self._latest_nav_path and self._latest_nav_path.poses:
            path_pts = [
                (p.pose.position.x, p.pose.position.y)
                for p in self._latest_nav_path.poses
            ]
            nearest = compute_nearest_in_path(path_pts, base_x_odom, base_y_odom)
            if nearest is not None:
                nx, ny, deviation = nearest
                dev_line = build_deviation_line_marker(
                    base_x=base_x_odom, base_y=base_y_odom,
                    nearest_x=nx, nearest_y=ny, stamp_ns=stamp_ns,
                )
                self._deviation_line_pub.publish(dev_line)

                # 同时发 std_msgs/Float64 供 rqt_plot 用
                dev_msg = Float64()
                dev_msg.data = deviation
                self._deviation_pub.publish(dev_msg)

        # 5. mode_text（mode 暂用 ACTIVE，后续可订阅 path_to_baselink 状态）
        mode_msg = build_mode_text_marker(
            mode="ACTIVE",
            vx=self._latest_vx,
            wz=self._latest_wz,
            deviation=deviation,
            stamp_ns=stamp_ns,
        )
        self._mode_text_pub.publish(mode_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TebDebugNode()
    signal.signal(signal.SIGINT, lambda *_: rclpy.shutdown())
    signal.signal(signal.SIGTERM, lambda *_: rclpy.shutdown())
    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"[teb_debug_node] FATAL: {e}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
