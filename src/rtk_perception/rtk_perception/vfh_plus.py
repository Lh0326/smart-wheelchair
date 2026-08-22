"""VFH+ (Vector Field Histogram Plus) 纯 Python 实现。

不依赖 rclpy，可独立用于 pytest 单测和算法验证。
所有 ROS2 适配在 obstacle_avoidance_node.py 中完成。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np


@dataclass
class VFHConfig:
    """VFH+ 算法参数。所有参数可在 ROS2 param 中覆盖。"""
    safety_distance_m: float = 0.5
    danger_distance_m: float = 1.5
    sector_deg: float = 5.0
    threshold_low: float = 0.2
    threshold_high: float = 0.9
    max_turn_rate_rad_s: float = 0.8
    max_speed_m_s: float = 0.6
    min_turn_radius_m: float = 0.5
    cost_weight_target: float = 1.0
    cost_weight_heading: float = 0.3
    angular_kp: float = 1.2
    front_fov_half_deg: float = 60.0   # 扩大前方 FOV（覆盖斜前方）
    cost_weight_prev_dir: float = 0.6  # 转向惯性权重（防抖动）
    angular_smoothing_alpha: float = 0.4  # 角速度低通滤波系数（越小越平滑）
    hysteresis_dead_band_rad: float = 0.15  # 方向死区（小于此角差不切换）
    cost_weight_curbs: float = 2.0  # 路沿惩罚权重

    @property
    def num_sectors(self) -> int:
        return int(round(360.0 / self.sector_deg))


@dataclass
class LaserScanData:
    """解耦后的 LaserScan 数据。"""
    ranges: np.ndarray
    angles: np.ndarray


@dataclass
class Twist2D:
    """解耦后的 2D 速度指令。"""
    linear_x: float = 0.0
    angular_z: float = 0.0


@dataclass
class Intent:
    """一次意图事件。"""
    twist: Twist2D
    source: str
    timestamp: float


def angle_to_sector(angle_rad: float, cfg: VFHConfig) -> int:
    """弧度 → 扇区索引（0 到 num_sectors-1）。"""
    deg = math.degrees(angle_rad) % 360.0
    return int(deg / cfg.sector_deg) % cfg.num_sectors


def build_histogram(scan: LaserScanData, cfg: VFHConfig) -> np.ndarray:
    """Step 1+2: 构建极坐标危险度直方图。

    权重公式：w = a * (b - d)^2
      a = 1 / danger_distance^2  （归一化系数）
      b = danger_distance + safety_distance  （最大影响距离）
      d = 实际距离
    """
    histogram = np.zeros(cfg.num_sectors, dtype=float)
    b_const = cfg.danger_distance_m + cfg.safety_distance_m
    a_const = 1.0 / (cfg.danger_distance_m ** 2)

    if scan.ranges.size == 0:
        return histogram

    for r, theta in zip(scan.ranges, scan.angles):
        if not np.isfinite(r):
            continue
        if r < 0.1 or r >= b_const:
            continue
        sector = angle_to_sector(float(theta), cfg)
        weight = a_const * (b_const - r) ** 2
        histogram[sector] = min(1.0, histogram[sector] + weight)
    return histogram


def binarize_with_hysteresis(
    histogram: np.ndarray, prev_binary: np.ndarray, cfg: VFHConfig
) -> np.ndarray:
    """Step 3: 双阈值二值化，带迟滞防抖。返回 int 数组，1=blocked, 0=free。"""
    binary = np.zeros(cfg.num_sectors, dtype=int)
    for i in range(cfg.num_sectors):
        h = histogram[i]
        if h >= cfg.threshold_high:
            binary[i] = 1
        elif h <= cfg.threshold_low:
            binary[i] = 0
        else:
            binary[i] = int(prev_binary[i])
    return binary


def min_front_distance(scan: LaserScanData, cfg: VFHConfig) -> float:
    """计算机器人正前方 ±front_fov_half_deg 范围内的最近距离。"""
    min_dist = float("inf")
    half_fov_rad = math.radians(cfg.front_fov_half_deg)
    if scan.ranges.size == 0:
        return 25.0
    for r, theta in zip(scan.ranges, scan.angles):
        if not np.isfinite(r) or r < 0.1:
            continue
        norm = ((float(theta) + math.pi) % (2 * math.pi)) - math.pi
        if abs(norm) <= half_fov_rad:
            if r < min_dist:
                min_dist = r
    return min_dist if min_dist != float("inf") else 25.0


def find_free_sectors(masked: np.ndarray, cfg: VFHConfig) -> List[Tuple[int, int]]:
    """Step 5a: 在二值化直方图中找连续的 free 扇区组（处理环形边界）。

    返回 [(start, end_exclusive), ...]，所有索引在 [0, n) 内。
    跨 0° 边界的连续 free 会被拆成两段 (start, n) 和 (0, end)。

    改造（C1）：按 event 时间顺序配对 starts 和 ends（修复原版 zip 错配 bug）。
    """
    n = cfg.num_sectors
    candidates: List[Tuple[int, int]] = []
    if len(masked) == 0:
        return candidates

    arr = np.asarray(masked)
    free_count = int((arr == 0).sum())
    if free_count == 0:
        return []
    if free_count >= n:
        return [(0, n)]

    # 按 event 时间顺序记录（start/end 交替出现）
    # 每个 event: (position, type) where type='s' (start) or 'e' (end exclusive)
    events: List[Tuple[int, str]] = []
    for i in range(n):
        prev = (i - 1) % n
        if arr[i] == 0 and arr[prev] != 0:
            events.append((i, 's'))
        elif arr[i] != 0 and arr[prev] == 0:
            events.append((i, 'e'))

    if not events:
        return []

    # 检查第一个 event 是否是 'e'（说明 free 区跨 0°，从 sector 0 开始）
    if events[0][1] == 'e':
        # 第一个 free 区从 sector 0 开始，到 events[0] 结束
        # 把最后一个 's' 移到开头（它实际是上一个 free 区的起点，跨 0°）
        last_start = None
        for j in range(len(events) - 1, -1, -1):
            if events[j][1] == 's':
                last_start = events.pop(j)
                break
        if last_start:
            events.insert(0, last_start)

    # 现在 events 应该是 s, e, s, e, ... 顺序
    # 按 (start, end) 配对
    i = 0
    while i + 1 < len(events):
        s_pos, s_type = events[i]
        e_pos, e_type = events[i + 1]
        if s_type == 's' and e_type == 'e':
            if s_pos < e_pos:
                # 正常 candidate（不跨 0°）
                candidates.append((s_pos, e_pos))
            else:
                # 跨 0°（s > e，不应该发生在排序后，但保险处理）
                candidates.append((s_pos, n))
                if e_pos > 0:
                    candidates.append((0, e_pos))
            i += 2
        else:
            i += 1

    return candidates


def _normalize_angle(angle_rad: float) -> float:
    """归一化到 [-pi, pi]。"""
    while angle_rad > math.pi:
        angle_rad -= 2 * math.pi
    while angle_rad < -math.pi:
        angle_rad += 2 * math.pi
    return angle_rad


def _sector_center_angle(start: int, end: int, cfg: VFHConfig) -> float:
    """计算扇区组的中心角度（弧度），范围 [-pi, pi]。"""
    n = cfg.num_sectors
    if start == 0 and end >= n:
        # 全圆 free，中心为正前方
        return 0.0
    mid_sector = (start + end) // 2
    angle = math.radians(mid_sector * cfg.sector_deg)
    return _normalize_angle(angle)


def select_best_direction(
    candidates: List[Tuple[int, int]],
    target_rad: Optional[float],
    cfg: VFHConfig,
    current_heading_rad: float = 0.0,
    prev_best_dir: float = 0.0,
) -> float:
    """Step 5b: 从候选扇区中选代价最小的方向。

    改造（C1）：在每个 candidate 角度范围内找最接近 target 的角度作为代表。
    - 如果 target 在 candidate 范围内 → 直接用 target（理想）
    - 否则 → 用最近的 candidate 边界（尽量靠近 target）

    这样无论全圆 free 还是部分 free，VFH+ 都会尽量朝 target 方向走。
    """
    if not candidates:
        return 0.0
    best_angle = 0.0
    best_cost = float("inf")

    for start, end in candidates:
        if target_rad is not None:
            # 用 sector index 判断 target 是否在 candidate 范围内（避免角度跨 0° 的歧义）
            target_sector = angle_to_sector(target_rad, cfg)
            if start <= target_sector < end:
                # target 在 candidate 范围内 → 直接用 target（理想）
                cand_angle = target_rad
            else:
                # target 不在范围 → 用 candidate center（避障优先）
                cand_angle = _sector_center_angle(start, end, cfg)
        else:
            cand_angle = _sector_center_angle(start, end, cfg)

        target_cost = abs(_normalize_angle(cand_angle - target_rad)) if target_rad is not None else 0.0
        heading_cost = abs(_normalize_angle(cand_angle - current_heading_rad))
        prev_dir_cost = abs(_normalize_angle(cand_angle - prev_best_dir))
        cost = (
            cfg.cost_weight_target * target_cost
            + cfg.cost_weight_heading * heading_cost
            + cfg.cost_weight_prev_dir * prev_dir_cost
        )
        if cost < best_cost:
            best_cost = cost
            best_angle = cand_angle
    return best_angle


def safety_fence(twist: Twist2D, scan: LaserScanData, cfg: VFHConfig) -> Twist2D:
    """硬约束：前方安全距离内禁止前进；速度硬上限。"""
    min_front = min_front_distance(scan, cfg)
    if min_front < cfg.safety_distance_m and twist.linear_x > 0:
        twist.linear_x = 0.0
    twist.linear_x = max(-0.3, min(cfg.max_speed_m_s, twist.linear_x))
    twist.angular_z = max(-cfg.max_turn_rate_rad_s, min(cfg.max_turn_rate_rad_s, twist.angular_z))
    return twist


class VFHPlus:
    """VFH+ 算法封装：订阅扫描 + 意图，输出安全 Twist。"""

    def __init__(self, config: VFHConfig):
        self.cfg = config
        self._prev_binary = np.zeros(config.num_sectors, dtype=int)
        self._prev_best_dir: float = 0.0   # 上一帧选定的方向（转向惯性 + 防抖）
        self._prev_angular_z: float = 0.0  # 上一帧输出角速度（低通滤波）
        self._dir_lock_frames: int = 0     # 方向锁定计数器（防震荡）
        self._dir_lock_threshold: int = 60 # 锁定 60 帧（3 秒 @20Hz，防掉头震荡）

    def compute(
        self,
        scan: LaserScanData,
        intent_twist: Twist2D,
        current_twist: Optional[Twist2D] = None,
        target_dir: Optional[float] = None,
    ) -> Twist2D:
        """主入口：6 步流水线 + 安全栅栏。

        target_dir: 外部传入的精确目标方向（弧度，相对当前朝向）。
                    如果为 None，则从 intent_twist 推导（向后兼容，二值化）。
        """
        if current_twist is None:
            current_twist = Twist2D()

        histogram = build_histogram(scan, self.cfg)
        binary = binarize_with_hysteresis(histogram, self._prev_binary, self.cfg)
        self._prev_binary = binary
        masked = self._mask_by_kinematics(binary, current_twist)
        candidates = find_free_sectors(masked, self.cfg)
        if not candidates:
            self._prev_angular_z = 0.0
            return Twist2D(0.0, 0.0)

        # 优先用外部传入的精确 target_dir（C1 改造）；
        # 否则从 intent_twist 二值化推导（向后兼容）
        if target_dir is None:
            target_dir = self._derive_target_direction(intent_twist)
        raw_best_dir = select_best_direction(
            candidates, target_dir, self.cfg,
            current_heading_rad=0.0,
            prev_best_dir=self._prev_best_dir,
        )
        # 方向锁定 + 死区：防止在 ±180°（掉头）场景左右来回切换
        if self._dir_lock_frames > 0:
            # 锁定期内强制保持上一帧方向
            self._dir_lock_frames -= 1
            best_dir = self._prev_best_dir
        elif abs(_normalize_angle(raw_best_dir - self._prev_best_dir)) < self.cfg.hysteresis_dead_band_rad:
            # 死区内保持
            best_dir = self._prev_best_dir
        else:
            # 方向切换 → 触发锁定
            best_dir = raw_best_dir
            self._dir_lock_frames = self._dir_lock_threshold
        self._prev_best_dir = best_dir

        twist = self._compute_velocity(scan, best_dir, intent_twist)
        # 角速度低通滤波（防抖动）
        alpha = self.cfg.angular_smoothing_alpha
        smoothed_w = (1.0 - alpha) * self._prev_angular_z + alpha * twist.angular_z
        twist.angular_z = max(-self.cfg.max_turn_rate_rad_s,
                              min(self.cfg.max_turn_rate_rad_s, smoothed_w))
        self._prev_angular_z = twist.angular_z

        return safety_fence(twist, scan, self.cfg)

    def _mask_by_kinematics(
        self, binary: np.ndarray, current_twist: Twist2D
    ) -> np.ndarray:
        """Step 4: 运动学掩蔽（简化版：当前直接复用 binary）。"""
        return binary.copy()

    @staticmethod
    def _derive_target_direction(intent_twist: Twist2D) -> Optional[float]:
        """从意图 Twist 推导目标方向（弧度）。"""
        if abs(intent_twist.linear_x) < 0.05 and abs(intent_twist.angular_z) < 0.1:
            return None
        if intent_twist.angular_z > 0.1:
            return math.pi / 2
        if intent_twist.angular_z < -0.1:
            return -math.pi / 2
        return 0.0

    def _compute_velocity(
        self,
        scan: LaserScanData,
        best_dir_rad: float,
        intent_twist: Twist2D,
    ) -> Twist2D:
        """Step 6: 根据前方最近距离和 best_dir 计算 Twist。

        关键：当意图源要求 v=0（如 cruise_speed=0 或 stop 意图），机器人必须停止。
        VFH 不能擅自前进。
        """
        intent_v = intent_twist.linear_x if intent_twist.linear_x > 0 else 0.0
        if intent_v < 0.01:
            # 意图明确要求停止 → VFH 不前进
            v = 0.0
        else:
            min_front = min_front_distance(scan, self.cfg)
            if min_front <= self.cfg.safety_distance_m:
                v = 0.0
            else:
                ratio = min(
                    (min_front - self.cfg.safety_distance_m) / self.cfg.danger_distance_m,
                    1.0,
                )
                v = ratio * min(intent_v, self.cfg.max_speed_m_s)
        w = self.cfg.angular_kp * best_dir_rad
        w = max(-self.cfg.max_turn_rate_rad_s,
                min(self.cfg.max_turn_rate_rad_s, w))
        return Twist2D(linear_x=v, angular_z=w)
