"""GPS 路径对齐节点（C1，两阶段状态机版）。

把 NetworkX 全局路径转换为机器人本体坐标系下的目标航向角。

**两阶段算法（解决"用户不在路径上"+ "L 形路径窜路"问题）**：

  阶段 1 APPROACHING（用户离路径 > approach_threshold_m）：
    - 目标点 = 路径上离用户最近的点（动态，跟着用户走，不写死起点）
    - 引导用户从建筑物内走到道路上的路径接入点
    - 例：用户在 A 楼 → 接入点是 A 楼最近的路边
          用户换到 B 楼 → 接入点自动变为 B 楼最近的路边

  阶段 2 ON_PATH（用户在路径上 < approach_threshold_m）：
    - 目标点 = 路径上当前位置向前 lookahead_distance_m 的点
    - 分段跟踪，L 形路径会先朝下再朝右（不会直接指向终点窜路）

订阅：
  /global_plan    (rtk_msgs/GlobalPlan)
  /fix            (sensor_msgs/NavSatFix)
  /heading_imu    (std_msgs/Float64)

发布：
  /target_heading          (std_msgs/Float64)  目标偏角（弧度，相对当前朝向）
  /target_point_marker     (visualization_msgs/Marker)  当前目标点（在 base_link 系）
"""
from __future__ import annotations

import math
import signal
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Point, Pose, PoseArray
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import ColorRGBA, Empty, Float64, Header
from visualization_msgs.msg import Marker

from rtk_msgs.msg import GlobalPlan


# ============ 纯算法（不依赖 rclpy，便于单测） ============

def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    R = 6371000.0
    lat1_r = math.radians(lat1)
    lat2_r = math.radians(lat2)
    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1_r) * math.cos(lat2_r) * math.sin(dlon / 2) ** 2
    return 2 * R * math.asin(min(1.0, math.sqrt(a)))


def compute_bearing_rad(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    lat1_r = math.radians(lat1)
    lat2_r = math.radians(lat2)
    dlon = math.radians(lon2 - lon1)
    y = math.sin(dlon) * math.cos(lat2_r)
    x = math.cos(lat1_r) * math.sin(lat2_r) - math.sin(lat1_r) * math.cos(lat2_r) * math.cos(dlon)
    return math.atan2(y, x)


def normalize_angle_rad(angle: float) -> float:
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


def project_to_segment(
    cur_lat: float, cur_lon: float,
    p1: Tuple[float, float], p2: Tuple[float, float],
) -> Tuple[float, Tuple[float, float]]:
    """将 (cur_lat, cur_lon) 投影到线段 p1→p2 上。

    返回 (参数 t in [0,1], 投影点 (lat, lon))。
    t=0 表示投影在 p1，t=1 表示在 p2。
    用局部平面近似（小距离 < 100m 误差可忽略）。
    """
    ref_lat = p1[0]
    ref_lon = p1[1]
    lat_per_m = 1.0 / 111320.0
    lon_per_m = 1.0 / (111320.0 * max(math.cos(math.radians(ref_lat)), 0.01))

    cx = (cur_lon - ref_lon) / lon_per_m
    cy = (cur_lat - ref_lat) / lat_per_m
    ax = (p2[1] - p1[1]) / lon_per_m
    ay = (p2[0] - p1[0]) / lat_per_m
    seg_len_sq = ax * ax + ay * ay

    if seg_len_sq < 1e-9:
        return 0.0, p1

    t = (cx * ax + cy * ay) / seg_len_sq
    t = max(0.0, min(1.0, t))

    proj_lat = p1[0] + t * (p2[0] - p1[0])
    proj_lon = p1[1] + t * (p2[1] - p1[1])
    return t, (proj_lat, proj_lon)


def find_nearest_on_path(
    path: List[Tuple[float, float]],
    cur_lat: float,
    cur_lon: float,
) -> Tuple[int, float, Tuple[float, float], float]:
    """找路径上离用户最近的点（包括段上插值）。

    返回 (segment_idx, distance_m, (lat, lon), segment_t)。
    segment_idx = 路径段索引（路径段 i 是 path[i] → path[i+1]）
    segment_t = 在该段上的参数 [0, 1]
    """
    if not path:
        return -1, float("inf"), (cur_lat, cur_lon), 0.0

    if len(path) == 1:
        d = haversine_m(cur_lat, cur_lon, path[0][0], path[0][1])
        return 0, d, path[0], 0.0

    best_dist = float("inf")
    best_idx = 0
    best_point = path[0]
    best_t = 0.0

    for i in range(len(path) - 1):
        p1 = path[i]
        p2 = path[i + 1]
        t, proj = project_to_segment(cur_lat, cur_lon, p1, p2)
        d = haversine_m(cur_lat, cur_lon, proj[0], proj[1])
        if d < best_dist:
            best_dist = d
            best_idx = i
            best_point = proj
            best_t = t

    return best_idx, best_dist, best_point, best_t


def find_lookahead_from_segment(
    path: List[Tuple[float, float]],
    start_seg_idx: int,
    segment_t: float,
    lookahead_m: float,
) -> Tuple[float, float]:
    """从路径段 start_seg_idx 的参数 segment_t 处开始，向前累积 lookahead_m 距离。

    返回 (lat, lon) 目标点。
    """
    if not path:
        return (0.0, 0.0)
    if start_seg_idx >= len(path) - 1:
        return path[-1]

    # 段中间的起点
    p1_lat = path[start_seg_idx][0] + segment_t * (path[start_seg_idx + 1][0] - path[start_seg_idx][0])
    p1_lon = path[start_seg_idx][1] + segment_t * (path[start_seg_idx + 1][1] - path[start_seg_idx][1])
    p1 = (p1_lat, p1_lon)

    accum = 0.0
    for i in range(start_seg_idx, len(path) - 1):
        p2 = path[i + 1]
        seg_len = haversine_m(p1[0], p1[1], p2[0], p2[1])
        if accum + seg_len >= lookahead_m:
            ratio = (lookahead_m - accum) / max(seg_len, 1e-6)
            return (
                p1[0] + ratio * (p2[0] - p1[0]),
                p1[1] + ratio * (p2[1] - p1[1]),
            )
        accum += seg_len
        p1 = p2

    return path[-1]


def path_to_cartesian_xy(
    origin_lat: float,
    origin_lon: float,
    path: List[Tuple[float, float]],
    skip_before_seg_idx: int = -1,
    skip_before_seg_t: float = 0.0,
    cur_lat: Optional[float] = None,
    cur_lon: Optional[float] = None,
) -> List[Tuple[float, float]]:
    """把 WGS84 路径转换为以 (origin_lat, origin_lon) 为原点的局部笛卡尔坐标 (x=东, y=北)。

    skip_before_seg_idx/seg_t：用户当前位置在路径上的段索引 + 参数 t，
    这些点之前的路径会被插值到当前点开始（不输出用户后方的点）。

    返回 [(x_meters, y_meters), ...]。
    """
    if not path:
        return []

    lat_per_m = 1.0 / 111320.0
    lon_per_m = 1.0 / (111320.0 * max(math.cos(math.radians(origin_lat)), 0.01))

    def to_xy(lat: float, lon: float) -> Tuple[float, float]:
        x = (lon - origin_lon) / lon_per_m
        y = (lat - origin_lat) / lat_per_m
        return (x, y)

    if skip_before_seg_idx < 0 or cur_lat is None or cur_lon is None:
        return [to_xy(lat, lon) for lat, lon in path]

    result: List[Tuple[float, float]] = []
    result.append(to_xy(cur_lat, cur_lon))
    for i in range(skip_before_seg_idx + 1, len(path)):
        result.append(to_xy(path[i][0], path[i][1]))
    return result


def build_nav_path_msg(
    cartesian_pts: List[Tuple[float, float]],
    frame_id: str = "odom",
) -> List[dict]:
    """把笛卡尔坐标列表转换为 PoseStamped 字典列表（不依赖 ROS，便于单测）。

    ROS 节点的 _publish_nav_path 方法会用这些字典构造 nav_msgs/Path。
    用 dict 而非 ROS 消息类型是为了纯 Python 单测（不需要 rclpy）。
    """
    result = []
    for x, y in cartesian_pts:
        result.append({
            "x": float(x),
            "y": float(y),
            "z": 0.0,
            "frame_id": frame_id,
        })
    return result


def interpolate_path(
    cartesian_pts: List[Tuple[float, float]],
    max_segment_length_m: float = 0.5,
) -> List[Tuple[float, float]]:
    """在路径节点之间线性插值，保证相邻点间距 ≤ max_segment_length_m。

    TEB 期望密集路径点（典型 < 1m 间距）才能规划局部轨迹。
    OSM 路径节点间距可能 50-100m，直接喂给 TEB 会 "trajectory is not feasible"。

    起点保留原坐标（不插值到 0），终点也保留。
    """
    if not cartesian_pts:
        return []
    if len(cartesian_pts) == 1:
        return list(cartesian_pts)

    result: List[Tuple[float, float]] = []
    for i in range(len(cartesian_pts) - 1):
        x1, y1 = cartesian_pts[i]
        x2, y2 = cartesian_pts[i + 1]
        dx = x2 - x1
        dy = y2 - y1
        seg_len = math.sqrt(dx * dx + dy * dy)
        result.append((x1, y1))
        if seg_len <= max_segment_length_m:
            continue
        n_segments = int(math.ceil(seg_len / max_segment_length_m))
        for k in range(1, n_segments):
            t = k / n_segments
            result.append((x1 + t * dx, y1 + t * dy))
    result.append(cartesian_pts[-1])
    return result


def find_corners(
    path: List[Tuple[float, float]],
    threshold_deg: float = 30.0,
    include_start: bool = True,
) -> List[Tuple[float, float, str]]:
    """识别路径上的拐角点。

    返回 [(lat, lon, type), ...]，type ∈ {"start", "corner", "goal"}。
    type="corner" 的判定：相邻两段的方向差（bearing） > threshold_deg。
    """
    if not path:
        return []
    if len(path) < 3:
        if len(path) == 1:
            return [(path[0][0], path[0][1], "goal")]
        if include_start:
            return [(path[0][0], path[0][1], "start"),
                    (path[-1][0], path[-1][1], "goal")]
        return [(path[-1][0], path[-1][1], "goal")]

    waypoints: List[Tuple[float, float, str]] = []
    if include_start:
        waypoints.append((path[0][0], path[0][1], "start"))

    for i in range(1, len(path) - 1):
        in_dir = compute_bearing_rad(path[i-1][0], path[i-1][1], path[i][0], path[i][1])
        out_dir = compute_bearing_rad(path[i][0], path[i][1], path[i+1][0], path[i+1][1])
        diff = abs(normalize_angle_rad(out_dir - in_dir))
        if math.degrees(diff) > threshold_deg:
            waypoints.append((path[i][0], path[i][1], "corner"))

    waypoints.append((path[-1][0], path[-1][1], "goal"))
    return waypoints


def find_position_on_path(
    path: List[Tuple[float, float]],
    target_lat: float,
    target_lon: float,
) -> Tuple[int, float]:
    """找 target 在路径上的位置（段索引 + 参数 t）。

    假设 target 是路径节点之一（不处理段中间任意点）。
    返回 (seg_idx, seg_t)。seg_idx=-1 表示未找到。
    """
    for i, p in enumerate(path):
        if abs(p[0] - target_lat) < 1e-9 and abs(p[1] - target_lon) < 1e-9:
            if i == len(path) - 1:
                return i - 1, 1.0  # 最后一个点，属于上一段末端
            return i, 0.0  # 起点位置
    return -1, 0.0


def path_distance_to_waypoint(
    path: List[Tuple[float, float]],
    from_seg_idx: int,
    from_seg_t: float,
    target: Tuple[float, float],
) -> float:
    """从 (seg_idx, seg_t) 沿路径累加距离到 target 点。

    target 必须是路径上的节点。
    """
    target_seg_idx, target_seg_t = find_position_on_path(path, target[0], target[1])
    if target_seg_idx < 0:
        return float("inf")

    # 如果 target 在 from 之前（路径回退），返回 inf
    if target_seg_idx < from_seg_idx or \
       (target_seg_idx == from_seg_idx and target_seg_t < from_seg_t):
        return float("inf")

    # 起点位置
    if from_seg_idx >= len(path) - 1:
        p_cur = path[-1]
    else:
        p1 = path[from_seg_idx]
        p2 = path[from_seg_idx + 1]
        p_cur = (p1[0] + from_seg_t * (p2[0] - p1[0]),
                 p1[1] + from_seg_t * (p2[1] - p1[1]))

    total = 0.0
    for i in range(from_seg_idx, target_seg_idx + 1):
        if i >= len(path) - 1:
            break
        if i < target_seg_idx:
            p_next = path[i + 1]
        else:
            # 最后一段：到 target_seg_t
            p1 = path[i]
            p2 = path[i + 1]
            p_next = (p1[0] + target_seg_t * (p2[0] - p1[0]),
                      p1[1] + target_seg_t * (p2[1] - p1[1]))
        total += haversine_m(p_cur[0], p_cur[1], p_next[0], p_next[1])
        p_cur = p_next

    return total


def find_next_waypoint(
    waypoints: List[Tuple[float, float, str]],
    path: List[Tuple[float, float]],
    from_seg_idx: int,
    from_seg_t: float,
    min_path_dist_m: float = 0.5,
) -> Optional[Tuple[float, float, float, str]]:
    """从用户位置沿路径找下一个规划点（路径距离排序）。

    返回 (lat, lon, path_distance_m, type) 或 None。
    路径距离 < min_path_dist_m 的点视为"刚走过"，跳过。
    """
    best = None
    best_path_dist = float("inf")

    for wp_lat, wp_lon, wp_type in waypoints:
        path_dist = path_distance_to_waypoint(path, from_seg_idx, from_seg_t, (wp_lat, wp_lon))
        if path_dist == float("inf"):
            continue  # 在用户后方，跳过
        if path_dist < min_path_dist_m:
            continue  # 刚走过，跳过
        if path_dist < best_path_dist:
            best_path_dist = path_dist
            best = (wp_lat, wp_lon, path_dist, wp_type)

    return best


def should_lock_seg_idx(
    mode: str,
    last_seg_idx: Optional[int],
    current_dist_to_path: float,
    last_dist_to_path: float,
    growth_threshold: float = 0.1,
) -> bool:
    """判断是否应该锁定上一帧的 seg_idx（绕障中段索引坚持）。

    判定：mode == "ON_PATH" 且有历史 seg_idx 且当前偏离比上一帧大 growth_threshold 以上。
    用于 _publish_nav_path：避免轮椅绕障时 find_nearest_on_path 跳到错误段。

    返回 True → 用 last_seg_idx；False → 用当前帧 seg_idx。
    """
    if mode != "ON_PATH":
        return False
    if last_seg_idx is None:
        return False
    return current_dist_to_path > last_dist_to_path + growth_threshold


def determine_mode(
    path: List[Tuple[float, float]],
    cur_lat: float,
    cur_lon: float,
    approach_threshold_m: float = 5.0,
    approach_hysteresis_m: float = 4.0,
    current_mode: str = "INIT",
    completed_distance_m: float = 3.0,
    corner_threshold_deg: float = 30.0,
    allow_approaching: bool = True,
) -> Tuple[str, Optional[Tuple[float, float]], float, float, int, float]:
    """三态状态机判断（带 hysteresis 防 5m 边界震荡）。

    返回 (mode, target_point, dist_to_path_m, dist_to_goal_along_path_m, seg_idx, seg_t)
    mode ∈ {"NO_PATH", "APPROACHING", "ON_PATH", "COMPLETED"}

    hysteresis 逻辑：
      ON_PATH → APPROACHING 触发：dist_to_path > approach_threshold_m (5m)
      APPROACHING → ON_PATH 回切：dist_to_path < approach_hysteresis_m (4m)
    中间区（4-5m）保持当前 mode，避免边界震荡。
    """
    if not path:
        return "NO_PATH", None, float("inf"), float("inf"), -1, 0.0

    seg_idx, dist_to_path, nearest_point, seg_t = find_nearest_on_path(path, cur_lat, cur_lon)

    dist_to_goal = path_distance_to_waypoint(path, seg_idx, seg_t, path[-1])

    if dist_to_goal < completed_distance_m:
        return "COMPLETED", None, dist_to_path, dist_to_goal, seg_idx, seg_t

    if allow_approaching:
        if current_mode == "APPROACHING":
            on_path_threshold = approach_hysteresis_m
        else:
            on_path_threshold = approach_threshold_m
        if dist_to_path > approach_threshold_m:
            return "APPROACHING", nearest_point, dist_to_path, dist_to_goal, seg_idx, seg_t
        if dist_to_path > on_path_threshold and current_mode == "APPROACHING":
            return "APPROACHING", nearest_point, dist_to_path, dist_to_goal, seg_idx, seg_t

    # ON_PATH：跟踪下一个拐角
    waypoints = find_corners(path, corner_threshold_deg, include_start=False)
    next_wp = find_next_waypoint(waypoints, path, seg_idx, seg_t)
    if next_wp is None:
        return "ON_PATH", path[-1], dist_to_path, dist_to_goal, seg_idx, seg_t
    return "ON_PATH", (next_wp[0], next_wp[1]), dist_to_path, dist_to_goal, seg_idx, seg_t


# ============ ROS2 节点 ============

class PathToBaseLinkNode(Node):
    def __init__(self):
        super().__init__("path_to_baselink_node")
        self.declare_parameter("lookahead_distance_m", 2.0)
        self.declare_parameter("approach_threshold_m", 5.0)  # 离路径 > 5m 触发 APPROACHING
        self.declare_parameter("approach_hysteresis_m", 4.0)  # ON_PATH 回切阈值
        self.declare_parameter("assume_start_on_route", True)
        self.declare_parameter("update_rate_hz", 10.0)
        self.declare_parameter("fix_timeout_sec", 2.0)
        # IMU heading 滤波参数（在 _heading_cb 中应用，73Hz）
        # 关键：从源头滤掉 IMU 抖动，target 自然稳定，不需要 target 二次滤波
        self.declare_parameter("heading_deadband_deg", 0.5)  # 相邻帧差<0.5° 视为噪声不响应
        self.declare_parameter("heading_alpha", 0.2)         # 低通滤波系数（小=稳，大=快）
        self.declare_parameter("heading_fast_threshold_deg", 15.0)  # >15° 视为真实转向直接采用
        # D4 三态状态机参数
        self.declare_parameter("corner_threshold_deg", 30.0)
        self.declare_parameter("completed_distance_m", 3.0)
        self.declare_parameter("completed_confirm_sec", 1.0)

        self._lookahead = self.get_parameter("lookahead_distance_m").value
        self._approach_threshold = self.get_parameter("approach_threshold_m").value
        self._approach_hysteresis_m = self.get_parameter("approach_hysteresis_m").value
        self._assume_start_on_route = bool(
            self.get_parameter("assume_start_on_route").value
        )
        rate = self.get_parameter("update_rate_hz").value
        self._fix_timeout = self.get_parameter("fix_timeout_sec").value
        self._heading_deadband_rad = math.radians(self.get_parameter("heading_deadband_deg").value)
        self._heading_alpha = self.get_parameter("heading_alpha").value
        self._heading_fast_threshold_rad = math.radians(self.get_parameter("heading_fast_threshold_deg").value)
        self._corner_threshold_deg = self.get_parameter("corner_threshold_deg").value
        self._completed_distance_m = self.get_parameter("completed_distance_m").value
        self._completed_confirm_sec = self.get_parameter("completed_confirm_sec").value
        self._completed_since_sec: Optional[float] = None

        # 缓存当前状态（供 D5 发布 topic 用）
        self._current_mode: str = "INIT"
        self._current_target_point: Optional[Tuple[float, float]] = None
        self._current_dist_to_path: float = 0.0
        self._current_dist_to_goal: float = 0.0

        self._latest_plan: Optional[GlobalPlan] = None
        self._latest_plan_time: float = 0.0
        self._latest_fix: Optional[NavSatFix] = None
        self._latest_heading: float = 0.0  # 已经滤波过的 heading（用于 target 计算）
        self._latest_fix_time: float = 0.0
        self._latest_heading_time: float = 0.0
        self._latest_mode: str = "INIT"
        self._smoothed_heading: Optional[float] = None  # heading 滤波状态

        # seg_idx 坚持：绕障中锁定上一帧 seg_idx
        self._last_seg_idx: Optional[int] = None
        self._last_dist_to_path: float = 0.0

        # TEB 修复：odom 系原点 = 第一次收到 /fix 的位置
        # sim_chassis 启动时 initial_lat/lon 也是这个值（它的 odom 原点）
        # 所以 path_to_cartesian_xy 必须用这个固定原点，frame_id 才与 odom 系一致
        self._odom_origin_lat: Optional[float] = None
        self._odom_origin_lon: Optional[float] = None
        self._last_diag_sec = 0.0

        # QoS 与 NetworkX publisher 对齐（VOLATILE），否则 DDS 不兼容导致消息丢失
        plan_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(GlobalPlan, "/global_plan", self._plan_cb, plan_qos)
        self.create_subscription(NavSatFix, "/fix", self._fix_cb, 10)
        self.create_subscription(Float64, "/heading_imu", self._heading_cb, 10)
        self.create_subscription(Empty, "/clear_goal", self._on_clear_goal, 10)

        self._target_pub = self.create_publisher(Float64, "/target_heading", 10)
        self._marker_pub = self.create_publisher(Marker, "/target_point_marker", 10)
        self._next_wp_pub = self.create_publisher(NavSatFix, '/next_waypoint_wgs84', 10)
        self._all_wp_pub = self.create_publisher(PoseArray, '/all_waypoints_wgs84', 10)

        # TEB: 发布 nav_msgs/Path 供 controller_server 跟踪
        from nav_msgs.msg import Path as NavPath
        self._NavPath = NavPath  # 缓存类，避免反复 import
        self._nav_path_pub = self.create_publisher(NavPath, "/nav_path", 10)

        self._timer = self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f"PathToBaseLinkNode started. lookahead={self._lookahead}m, "
            f"approach_threshold={self._approach_threshold}m"
        )

    def _plan_cb(self, msg: GlobalPlan):
        self._latest_plan = msg
        self._latest_plan_time = self.get_clock().now().nanoseconds * 1e-9
        self._completed_since_sec = None
        self._latest_mode = "INIT"
        self._last_seg_idx = None
        self._last_dist_to_path = 0.0

    def _on_clear_goal(self, msg):
        """前端清除终点时调用，清空缓存的路径（停止发 target_heading 和 waypoints）。"""
        self._latest_plan = None
        self._completed_since_sec = None
        self._latest_mode = "INIT"
        self._smoothed_heading = None
        # 注意：odom 原点不重置（保持里程计连续，避免 TF 跳变）
        self.get_logger().info("终点已清除，停止输出 target_heading 和 waypoints")

    def _fix_cb(self, msg: NavSatFix):
        # 拒绝 EC20/GNSS 未定位时发的占位 /fix：
        #   - status.status < STATUS_FIX(0)：NO_FIX 等
        #   - (lat,lon) ≈ (0,0)：EC20 冷启动 fix_quality=0 时发的赤道+本初子午线占位
        # 这两种情况都把消息丢弃，避免把 (0,0) 锁定为 odom 原点（之前实物模式的根因：
        # path_to_baselink 在 EC20 冷启动阶段锁了 (0,0)，后续真实 fix 来了也不再更新，
        # 导致 /nav_path 转出来的笛卡尔坐标偏离机器人几千公里 → TEB trajectory infeasible）
        if msg.status.status < 0:
            return
        if abs(msg.latitude) < 1.0 and abs(msg.longitude) < 1.0:
            return
        if not (math.isfinite(msg.latitude) and math.isfinite(msg.longitude)):
            return

        # odom 原点只在首次有效 fix 时锁定；之前若被错误锁定为 (0,0) 也用真实 fix 覆盖
        bad_origin = (
            self._odom_origin_lat is None
            or (abs(self._odom_origin_lat) < 1.0 and abs(self._odom_origin_lon) < 1.0)
        )
        if bad_origin:
            self._odom_origin_lat = msg.latitude
            self._odom_origin_lon = msg.longitude
            self.get_logger().info(
                f"odom 原点锁定: ({msg.latitude:.6f}, {msg.longitude:.6f})"
            )
        self._latest_fix = msg
        self._latest_fix_time = self.get_clock().now().nanoseconds * 1e-9

    def _heading_cb(self, msg: Float64):
        """订阅 IMU heading（100Hz），从源头滤波。

        ⚠️ /heading_imu 发布的是**度数**（0-360°，指南针角度），不是弧度！
        内部所有计算用弧度，所以收到后先 math.radians() 转换。

        三档自适应滤波：
        - 相邻帧差 < 0.5°：视为 IMU 噪声，完全不响应
        - 0.5° ~ 15°：慢速低通（α=0.2），抗抖动
        - > 15°：轮椅真的转向了，直接采用
        """
        if not math.isfinite(msg.data):
            return
        # /heading_imu 是度数（0-360°），转弧度
        raw = math.radians(float(msg.data))

        if self._smoothed_heading is None:
            self._smoothed_heading = raw
        else:
            diff = normalize_angle_rad(raw - self._smoothed_heading)
            abs_diff = abs(diff)
            if abs_diff < self._heading_deadband_rad:
                # 死区：不更新（保留旧值）
                pass
            elif abs_diff > self._heading_fast_threshold_rad:
                # 大转向：直接采用
                self._smoothed_heading = raw
            else:
                # 中等变化：低通滤波
                self._smoothed_heading += self._heading_alpha * diff

        self._latest_heading = self._smoothed_heading

    def _tick(self):
        now_sec = self.get_clock().now().nanoseconds * 1e-9

        if self._latest_plan is None:
            self._log_state_waiting(now_sec, "plan")
            return
        if self._latest_fix is None:
            self._log_state_waiting(now_sec, "fix")
            self.get_logger().warn("等待 /fix（GPS 未定位）", throttle_duration_sec=5.0)
            return
        if now_sec - self._latest_fix_time > self._fix_timeout:
            self._log_state_waiting(now_sec, "fix_stale")
            self.get_logger().warn("/fix 数据过期（GPS 信号丢失？）", throttle_duration_sec=5.0)
            return
        if self._odom_origin_lat is None or self._odom_origin_lon is None:
            self._log_state_waiting(now_sec, "odom_origin")
            return

        plan = self._latest_plan
        if plan.status != "OK" or not plan.path_wgs84:
            self._log_state_waiting(now_sec, f"plan_status={plan.status}")
            return

        path = [(p.y, p.x) for p in plan.path_wgs84]

        mode, target_point, dist_to_path, dist_to_goal, cur_seg_idx, cur_seg_t = determine_mode(
            path,
            self._latest_fix.latitude,
            self._latest_fix.longitude,
            approach_threshold_m=self._approach_threshold,
            approach_hysteresis_m=self._approach_hysteresis_m,
            current_mode=self._latest_mode,
            completed_distance_m=self._completed_distance_m,
            corner_threshold_deg=self._corner_threshold_deg,
            allow_approaching=not self._assume_start_on_route,
        )

        # COMPLETED 1 秒确认逻辑
        if mode == "COMPLETED":
            if self._completed_since_sec is None:
                self._completed_since_sec = now_sec
                return  # 第一帧不输出，等确认
            if now_sec - self._completed_since_sec < self._completed_confirm_sec:
                return  # 确认时间不够，不输出
            # 确认通过，正式进入 COMPLETED
            self._latest_mode = "COMPLETED"
            self.get_logger().info("✅ 已到达终点（COMPLETED）", throttle_duration_sec=5.0)
            return  # COMPLETED 不输出 target_heading
        else:
            # 离开 COMPLETED 状态，重置确认计时
            self._completed_since_sec = None

        if target_point is None:
            return

        target_lat, target_lon = target_point
        cur_lat = self._latest_fix.latitude
        cur_lon = self._latest_fix.longitude

        dist_to_target = haversine_m(cur_lat, cur_lon, target_lat, target_lon)
        if dist_to_target < 0.5:
            return

        bearing = compute_bearing_rad(cur_lat, cur_lon, target_lat, target_lon)
        # heading 已在 _heading_cb 滤波，target 直接用（不再二次滤波）
        # 注意方向：heading 和 bearing 都是指南针角度（顺时针为正：0=北,90=东,180=南,270=西）
        # 但 ROS 角度逆时针为正（+90°=左转, -90°=右转）
        # 所以 target = heading - bearing（不是 bearing - heading，否则左右反了）
        target_heading = normalize_angle_rad(self._latest_heading - bearing)

        # 发布 target_heading
        out = Float64()
        out.data = target_heading
        self._target_pub.publish(out)

        # 缓存当前 mode 和 target（供 D5 发布 topic 用）
        self._current_mode = mode
        self._current_target_point = target_point
        self._current_dist_to_path = dist_to_path
        self._current_dist_to_goal = dist_to_goal

        # 发布目标点 Marker（在 base_link 系下，用 smoothed 方向 + 距离）
        self._publish_target_marker(target_heading, dist_to_target, mode)

        # 发布 WGS84 规划点（供前端订阅显示 markers）
        self._publish_waypoints(path, target_point, mode)

        # TEB: 发布 nav_msgs/Path（odom 系，供 controller_server 跟踪）
        # cur_seg_idx/cur_seg_t 复用 determine_mode 返回值，避免重复调用 find_nearest_on_path
        self._publish_nav_path(
            path=path,
            cur_lat=self._latest_fix.latitude,
            cur_lon=self._latest_fix.longitude,
            seg_idx=cur_seg_idx,
            seg_t=cur_seg_t,
            mode=mode,
        )

        self._latest_mode = mode
        self.get_logger().info(
            f"[{mode}] target={math.degrees(target_heading):+.1f}° "
            f"bearing={math.degrees(bearing):.1f}° heading={math.degrees(self._latest_heading) % 360:.1f}° "
            f"dist_to_path={dist_to_path:.1f}m dist_to_goal={dist_to_goal:.1f}m",
            throttle_duration_sec=1.0,
        )

    def _log_state_waiting(self, now_sec: float, reason: str):
        """低频输出路径转换等待原因，便于实物链路调试。"""
        if now_sec - self._last_diag_sec < 3.0:
            return
        self._last_diag_sec = now_sec
        self.get_logger().info(
            "path_to_baselink waiting: reason=%s plan=%s fix=%s heading=%s "
            "odom_origin=%s latest_mode=%s"
            % (
                reason,
                self._latest_plan is not None,
                self._latest_fix is not None,
                self._smoothed_heading is not None,
                self._odom_origin_lat is not None and self._odom_origin_lon is not None,
                self._latest_mode,
            )
        )

    def _publish_target_marker(self, heading_rad: float, distance_m: float, mode: str):
        """发布目标点 Marker，在 base_link 系下用 (heading, distance) 表示位置。"""
        m = Marker()
        m.header = Header()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "base_link"
        m.ns = "target_point"
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.lifetime.sec = 0
        m.lifetime.nanosec = 200_000_000  # 200ms

        # 在 base_link 系下的位置
        clamped_dist = min(distance_m, 8.0)  # RViz 显示距离封顶 8m，避免太远看不见
        pos = Point()
        pos.x = clamped_dist * math.cos(heading_rad)
        pos.y = clamped_dist * math.sin(heading_rad)
        pos.z = 0.3
        m.pose.position = pos

        # 球大小
        m.scale.x = 0.20
        m.scale.y = 0.20
        m.scale.z = 0.20

        # 颜色按模式区分
        c = ColorRGBA()
        c.a = 1.0
        if mode == "APPROACHING":
            c.r = 1.0  # 红色 = 正在走向路径
            c.g = 0.0
            c.b = 0.0
        else:
            c.r = 0.0  # 蓝色 = 在路径上跟踪
            c.g = 0.5
            c.b = 1.0
        m.color = c

        self._marker_pub.publish(m)

    def _publish_waypoints(self, path: List[Tuple[float, float]],
                            current_target: Tuple[float, float], mode: str):
        """发布所有规划点（PoseArray）和当前目标（NavSatFix）。

        PoseArray 约定（与 GlobalPlan.path_wgs84 一致）：
          position.x = longitude, position.y = latitude
          waypoints[0] = start, waypoints[-1] = goal, 中间 = corner
        frame_id = "wgs84"（前端按约定解析）
        orientation.z 编码 type：0.0=start, 0.5=corner, 1.0=goal
        """
        if not path:
            return

        waypoints = find_corners(
            path, self._corner_threshold_deg, include_start=False
        )

        # 1. 发布所有规划点 (PoseArray)
        pose_array = PoseArray()
        pose_array.header = Header()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = "wgs84"

        for wp_lat, wp_lon, wp_type in waypoints:
            pose = Pose()
            pose.position.x = wp_lon  # lon
            pose.position.y = wp_lat  # lat
            pose.position.z = 0.0
            # 用 orientation.z 编码 type（前端解析）：
            #   0.0 = start, 0.5 = corner, 1.0 = goal
            if wp_type == "start":
                pose.orientation.z = 0.0
            elif wp_type == "corner":
                pose.orientation.z = 0.5
            else:  # goal
                pose.orientation.z = 1.0
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)

        self._all_wp_pub.publish(pose_array)

        # 2. 发布当前目标 (NavSatFix) — 只在非 COMPLETED 模式
        if mode != "COMPLETED" and current_target is not None:
            navsat = NavSatFix()
            navsat.header = Header()
            navsat.header.stamp = self.get_clock().now().to_msg()
            navsat.header.frame_id = "wgs84"
            navsat.latitude = current_target[0]
            navsat.longitude = current_target[1]
            navsat.status.status = 0  # GNSS status
            self._next_wp_pub.publish(navsat)

    def _publish_nav_path(
        self,
        path: List[Tuple[float, float]],
        cur_lat: float,
        cur_lon: float,
        seg_idx: int,
        seg_t: float,
        mode: str,
    ):
        """发布 nav_msgs/Path（odom 系，供 controller_server 跟踪）。

        COMPLETED 模式不发布（让 TEB 停下）。
        """
        if mode == "COMPLETED":
            return

        # seg_idx 坚持：绕障中（dist_to_path 增长）锁定上一帧 seg_idx
        # 避免轮椅偏离路径时 find_nearest_on_path 跳到错误段
        if should_lock_seg_idx(
            mode=mode,
            last_seg_idx=self._last_seg_idx,
            current_dist_to_path=self._current_dist_to_path,
            last_dist_to_path=self._last_dist_to_path,
        ):
            seg_idx = self._last_seg_idx
            # 在固定段上重新算 seg_t（位置仍准确）
            if seg_idx < len(path) - 1:
                seg_t, _ = project_to_segment(
                    cur_lat, cur_lon, path[seg_idx], path[seg_idx + 1]
                )

        # 记录本帧状态供下帧判断
        self._last_seg_idx = seg_idx
        self._last_dist_to_path = self._current_dist_to_path

        cartesian_pts = path_to_cartesian_xy(
            origin_lat=self._odom_origin_lat,
            origin_lon=self._odom_origin_lon,
            path=path,
            skip_before_seg_idx=seg_idx,
            skip_before_seg_t=seg_t,
            cur_lat=cur_lat,
            cur_lon=cur_lon,
        )

        # APPROACHING 模式（用户离路径 > approach_threshold）：
        # 在 cur_pos 后插入路径最近点，让 TEB 先引导轮椅走到路径上，再沿路径走
        # 否则 TEB 直接从用户位置连到 path[seg_idx+1]（远端 OSM 节点），
        # 可能导致轮椅朝错误方向缓慢移动
        if mode == "APPROACHING" and seg_idx < len(path) - 1:
            p1_lat, p1_lon = path[seg_idx]
            p2_lat, p2_lon = path[seg_idx + 1]
            nearest_lat = p1_lat + seg_t * (p2_lat - p1_lat)
            nearest_lon = p1_lon + seg_t * (p2_lon - p1_lon)
            lat_per_m = 1.0 / 111320.0
            lon_per_m = 1.0 / (
                111320.0 * max(math.cos(math.radians(self._odom_origin_lat)), 0.01)
            )
            nearest_x = (nearest_lon - self._odom_origin_lon) / lon_per_m
            nearest_y = (nearest_lat - self._odom_origin_lat) / lat_per_m
            # 只在 nearest 与 cur_pos 距离 > 1m 时插入（避免冗余点）
            cur_x, cur_y = cartesian_pts[0]
            if (nearest_x - cur_x) ** 2 + (nearest_y - cur_y) ** 2 > 1.0:
                cartesian_pts.insert(1, (nearest_x, nearest_y))

        # 关键：OSM 路径节点间距 50-100m，TEB 期望 < 1m 间距才能规划局部轨迹
        # 必须在节点之间插值，否则 TEB 报 "trajectory is not feasible"
        cartesian_pts = interpolate_path(cartesian_pts, max_segment_length_m=0.5)
        pose_dicts = build_nav_path_msg(cartesian_pts, frame_id="odom")

        msg = self._NavPath()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        from geometry_msgs.msg import PoseStamped
        for pd in pose_dicts:
            ps = PoseStamped()
            ps.header = Header()
            ps.header.stamp = msg.header.stamp
            ps.header.frame_id = "odom"
            ps.pose.position.x = pd["x"]
            ps.pose.position.y = pd["y"]
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self._nav_path_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PathToBaseLinkNode()
    signal.signal(signal.SIGINT, lambda *_: rclpy.shutdown())
    signal.signal(signal.SIGTERM, lambda *_: rclpy.shutdown())
    try:
        rclpy.spin(node)
    except Exception as e:
        import traceback
        print(f"[path_to_baselink_node] FATAL: {e}")
        traceback.print_exc()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
