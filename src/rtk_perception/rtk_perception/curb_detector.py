"""LD14P 路沿检测纯算法（无 rclpy，便于 pytest 单测）。

5 步流水线：
  1. 极坐标 (r, θ) 数据
  2. 按角度分 bin
  3. 相邻 bin 距离突变 → 候选路沿点
  4. DBSCAN 聚类去噪
  5. RANSAC 直线拟合 → 路沿线段
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np


@dataclass
class CurbConfig:
    """路沿检测参数。"""
    delta_r_threshold: float = 0.15       # 相邻 bin 距离突变阈值
    max_curb_range: float = 5.0           # 候选点最大距离
    min_line_length: float = 1.0          # 路沿线段最短长度
    bin_size_deg: float = 1.0             # 角度 bin 大小
    dbscan_eps: float = 0.3               # DBSCAN 聚类半径
    dbscan_min_samples: int = 3           # DBSCAN 最小簇大小

    @property
    def num_bins(self) -> int:
        return int(round(360.0 / self.bin_size_deg))


@dataclass
class CurbLine:
    """一条识别到的路沿。"""
    start_x: float
    start_y: float
    end_x: float
    end_y: float
    centroid_x: float
    centroid_y: float
    length: float

    @property
    def is_left(self) -> bool:
        return self.centroid_x < 0


def bucket_by_angle(polar: np.ndarray, cfg: CurbConfig) -> List[Optional[float]]:
    """Step 2：极坐标 → 按角度分 bin，每 bin 取代表距离（最小值）。

    polar: shape (N, 2)，每行 (r, theta_rad)
    返回长度 num_bins 的 list，每个元素是该 bin 的最小有效距离或 None。
    """
    bins: List[Optional[float]] = [None] * cfg.num_bins
    if polar is None or polar.size == 0:
        return bins
    if polar.ndim == 1:
        polar = polar.reshape(1, -1)
    for row in polar:
        r = float(row[0])
        theta = float(row[1])
        if not np.isfinite(r) or not np.isfinite(theta):
            continue
        if r <= 0.1:  # 忽略过近的点（噪声）
            continue
        if r > cfg.max_curb_range * 2.0:  # 远超 max_curb_range 的点直接忽略
            continue
        deg = math.degrees(theta) % 360.0
        idx = int(deg / cfg.bin_size_deg) % cfg.num_bins
        if bins[idx] is None or r < bins[idx]:
            bins[idx] = r
    return bins


def _find_jump_candidates(
    bins: List[Optional[float]], cfg: CurbConfig
) -> List[Tuple[float, float]]:
    """原始 focal-range 启发：相邻 bin 距离突变 → 候选点。"""
    candidates: List[Tuple[float, float]] = []
    n = cfg.num_bins
    for i in range(n):
        r_cur = bins[i]
        r_next = bins[(i + 1) % n]
        if r_cur is None or r_next is None:
            continue
        if r_cur > cfg.max_curb_range or r_next > cfg.max_curb_range:
            continue
        delta_r = abs(r_next - r_cur)
        if delta_r > cfg.delta_r_threshold:
            r_mid = (r_cur + r_next) / 2.0
            theta_mid = math.radians((i + 0.5) * cfg.bin_size_deg)
            x = r_mid * math.cos(theta_mid)
            y = r_mid * math.sin(theta_mid)
            candidates.append((x, y))
    return candidates


def _find_contiguous_candidates(
    bins: List[Optional[float]], cfg: CurbConfig
) -> List[Tuple[float, float]]:
    """候选路沿点（笛卡尔坐标）。

    策略：把所有有效 bin（距离 ≤ max_curb_range）的笛卡尔点都纳入候选。
    真正的"路沿/墙"会形成一段连续角度范围内的扫描，PCA 拟合后变成一条直线；
    随机噪声虽然也能产生候选，但不会形成共线簇，会在 fit_line 阶段被 colinearity
    阈值过滤掉。
    """
    candidates: List[Tuple[float, float]] = []
    for i, r in enumerate(bins):
        if r is None:
            continue
        if r > cfg.max_curb_range:
            continue
        theta = math.radians(i * cfg.bin_size_deg)
        x = r * math.cos(theta)
        y = r * math.sin(theta)
        candidates.append((x, y))
    return candidates


def _find_candidates(
    bins: List[Optional[float]], cfg: CurbConfig
) -> List[Tuple[float, float]]:
    """Step 3：候选路沿点。

    合并两种启发：
      - focal-range 跳变（路沿突然出现/消失）
      - 连续扫描段（墙/栏杆等延伸障碍）
    去重由后续 DBSCAN 自然完成。
    """
    jump_pts = _find_jump_candidates(bins, cfg)
    contig_pts = _find_contiguous_candidates(bins, cfg)
    # 简单合并；DBSCAN 会处理冗余
    return jump_pts + contig_pts


def _dbscan(
    points: np.ndarray, eps: float, min_samples: int
) -> List[List[int]]:
    """简化 DBSCAN 聚类，返回簇索引列表。"""
    if len(points) == 0:
        return []
    n = len(points)
    visited = np.zeros(n, dtype=bool)
    clusters: List[List[int]] = []
    for i in range(n):
        if visited[i]:
            continue
        dists = np.linalg.norm(points - points[i], axis=1)
        neighbors = np.where(dists <= eps)[0].tolist()
        if len(neighbors) < min_samples:
            visited[i] = True
            continue
        # 扩展簇
        cluster: List[int] = []
        queue = list(neighbors)
        while queue:
            j = queue.pop(0)
            if visited[j]:
                continue
            visited[j] = True
            cluster.append(j)
            d = np.linalg.norm(points - points[j], axis=1)
            nb = np.where(d <= eps)[0].tolist()
            if len(nb) >= min_samples:
                queue.extend(nb)
        if cluster:
            clusters.append(cluster)
    return clusters


def fit_line(points: np.ndarray) -> Optional[CurbLine]:
    """Step 5：PCA / SVD 直线拟合，返回 CurbLine 或 None。

    要求点集足够共线：主奇异值至少是次奇异值的 3 倍。
    否则视为非线状（如纯噪声云），返回 None。

    points: shape (N, 2)
    """
    if points is None or len(points) < 2:
        return None
    pts = np.asarray(points, dtype=float)
    if pts.ndim != 2 or pts.shape[1] != 2:
        return None
    centroid = pts.mean(axis=0)
    centered = pts - centroid
    try:
        u, s, vh = np.linalg.svd(centered, full_matrices=False)
    except np.linalg.LinAlgError:
        return None
    if vh.shape[0] < 1 or len(s) < 2:
        return None
    # 共线性门槛：主方差远大于次方差（线段 vs 云团）
    if s[1] > 1e-6 and s[0] / max(s[1], 1e-9) < 3.0:
        return None
    direction = vh[0]
    projections = centered @ direction
    p_min = float(projections.min())
    p_max = float(projections.max())
    length = p_max - p_min
    if length < 0.1:
        return None
    start = centroid + direction * p_min
    end = centroid + direction * p_max
    return CurbLine(
        start_x=float(start[0]),
        start_y=float(start[1]),
        end_x=float(end[0]),
        end_y=float(end[1]),
        centroid_x=float(centroid[0]),
        centroid_y=float(centroid[1]),
        length=float(length),
    )


def detect_curbs(polar: np.ndarray, cfg: CurbConfig) -> List[CurbLine]:
    """完整 5 步流水线：极坐标 → 路沿线段列表。"""
    if polar is None or polar.size == 0:
        return []
    # 过滤 NaN/inf
    if polar.ndim != 2 or polar.shape[1] < 2:
        return []
    finite_mask = np.isfinite(polar[:, 0]) & np.isfinite(polar[:, 1])
    if not finite_mask.any():
        return []
    polar = polar[finite_mask]

    bins = bucket_by_angle(polar, cfg)
    candidates = _find_candidates(bins, cfg)
    if len(candidates) < cfg.dbscan_min_samples:
        return []
    cand_array = np.array(candidates, dtype=float)
    clusters = _dbscan(cand_array, cfg.dbscan_eps, cfg.dbscan_min_samples)
    curbs: List[CurbLine] = []
    for cluster_idx in clusters:
        cluster_pts = cand_array[cluster_idx]
        line = fit_line(cluster_pts)
        if line is not None and line.length >= cfg.min_line_length:
            curbs.append(line)
    return curbs
