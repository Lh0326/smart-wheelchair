"""Safety chain 纯算法：前方扇形最近障碍 → 急停/减速。

不依赖 rclpy，可独立用于 pytest 单测和算法验证。
所有 ROS2 适配在 safety_chain_node.py 中完成。
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from rtk_perception.vfh_plus import LaserScanData, Twist2D

# 重新导出，便于上层和测试集中导入
__all__ = [
    "LaserScanData",
    "Twist2D",
    "SafetyConfig",
    "min_front_distance_in_fov",
    "apply_safety_chain",
]


@dataclass
class SafetyConfig:
    """安全链参数。所有参数可在 ROS2 param 中覆盖。"""
    emergency_stop_distance: float = 0.3   # < 0.3m 全停
    slowdown_distance: float = 1.0         # < 1.0m 减速 0.3x
    slowdown_factor: float = 0.3
    front_fov_half_deg: float = 30.0       # 前方 ±30° 扇形


def _normalize_to_pi_vec(angles: np.ndarray) -> np.ndarray:
    """将任意角度向量规约到 [-pi, pi]（保持 float64 dtype）。"""
    a = np.asarray(angles, dtype=float)
    return np.arctan2(np.sin(a), np.cos(a))


def min_front_distance_in_fov(scan: LaserScanData, cfg: SafetyConfig) -> float:
    """计算前方 ±front_fov_half_deg 扇形内的最近距离。

    过滤规则：
      - NaN / Inf 视为无效读数（雷达盲区）
      - < 0.1m 视为串扰（实际硬件不可能这么近）

    若无有效读数，返回 +inf（视作无障碍）。
    """
    if scan.ranges.size == 0:
        return float("inf")

    half_fov_rad = math.radians(cfg.front_fov_half_deg)
    angles_in_fov = np.abs(_normalize_to_pi_vec(scan.angles)) <= half_fov_rad
    if not np.any(angles_in_fov):
        return float("inf")

    ranges_fov = scan.ranges[angles_in_fov]
    valid = np.isfinite(ranges_fov) & (ranges_fov >= 0.1)
    if not np.any(valid):
        return float("inf")

    return float(np.min(ranges_fov[valid]))


def apply_safety_chain(twist: Twist2D, scan: LaserScanData, cfg: SafetyConfig) -> Twist2D:
    """对 cmd_vel 应用最后安全约束。

    规则（按优先级）：
      1. 输入含 NaN/Inf → 输出全零（防 NaN 串到电机层）
      2. 前方 FOV 最近障碍 < emergency_stop_distance → 全停
      3. 前方 FOV 最近障碍进入减速区 → 按距离连续缩放线速度
      4. 否则透传
    """
    if not (math.isfinite(twist.linear_x) and math.isfinite(twist.angular_z)):
        return Twist2D(0.0, 0.0)

    min_front = min_front_distance_in_fov(scan, cfg)

    if min_front < cfg.emergency_stop_distance:
        return Twist2D(0.0, 0.0)

    if min_front < cfg.slowdown_distance:
        slowdown_span = cfg.slowdown_distance - cfg.emergency_stop_distance
        if slowdown_span <= 1e-6:
            scale = cfg.slowdown_factor
        else:
            progress = (
                (min_front - cfg.emergency_stop_distance) / slowdown_span
            )
            progress = max(0.0, min(1.0, progress))
            scale = cfg.slowdown_factor + (
                1.0 - cfg.slowdown_factor
            ) * progress
        return Twist2D(
            linear_x=twist.linear_x * scale,
            angular_z=twist.angular_z,
        )

    return Twist2D(linear_x=twist.linear_x, angular_z=twist.angular_z)
