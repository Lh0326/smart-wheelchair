"""LaserScan → [{a, r}] 极坐标点列表的纯函数。

从 web_node._scan_cb 提取，便于单测和复用。
"""
from __future__ import annotations

import math
from typing import Dict, List


def scan_to_points(msg, max_points: int = 720,
                   max_range_cap: float = 12.0) -> List[Dict[str, float]]:
    """将 sensor_msgs/LaserScan 转为前端可绘制的极坐标点列表。

    处理顺序（关键）：先过滤无效点，再降采样。这样无论原始数据中
    inf/nan 占比多高，所有有效点都会被纳入降采样候选池。

    - 过滤：inf / nan / 小于 range_min / 大于 effective_max 的点全部丢弃
    - 距离上限：effective_max = min(msg.range_max, max_range_cap)，
      避免远点压缩主雷达可读性
    - 降采样：有效点数 > max_points 时，等间距取 max_points 个；
      否则全部保留
    """
    ranges = msg.ranges
    effective_max = min(float(msg.range_max), max_range_cap)
    effective_min = float(msg.range_min)

    # 第一步：遍历全部 ranges，收集所有有效点（不预先抽样）
    valid: List[Dict[str, float]] = []
    for index, dist in enumerate(ranges):
        if not math.isfinite(dist) or dist < effective_min or dist > effective_max:
            continue
        angle = msg.angle_min + index * msg.angle_increment
        valid.append({"a": float(angle), "r": float(dist)})

    if len(valid) <= max_points:
        return valid

    # 第二步：仅当有效点过多时，对 valid 做等间距降采样
    step = (len(valid) - 1) / (max_points - 1)
    sampled: List[Dict[str, float]] = []
    i = 0
    while i < max_points:
        sampled.append(valid[int(round(i * step))])
        i += 1
    return sampled
