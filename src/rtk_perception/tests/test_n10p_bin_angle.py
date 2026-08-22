"""N10P 驱动角度 bin 映射纯函数单测（不依赖 rclpy）。

测试目标：硬件报的 ang_deg 是 CW 方向（0=前, 90=右, 180=后, 270=左），
驱动要把它映射到 ROS CCW 约定的 bin 索引（bin 0 = angle=-π = -X 后方,
bin n/2 = angle=0 = +X 前方, bin 3n/4 = angle=π/2 = +Y 左方）。

参考设计：点云融合修复设计(2026-07-06) §3.1
"""
import math

import pytest

# 通过 importlib 直接从脚本文件加载（脚本不在 rtk_perception 包内）
import importlib.util
import os

_SPEC = importlib.util.spec_from_file_location(
    "n10p_python_driver",
    os.path.join(os.path.dirname(__file__), "..", "..", "..", "scripts", "n10p_python_driver.py"),
)
_n10p = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_n10p)
bin_angle_deg = _n10p.bin_angle_deg


N = 540  # 与驱动 POINTS_PER_FRAME 一致


def test_flip_true_front_stays_front():
    """flip=true：硬件 0°（前）应映射到 ROS angle=0（+X 前）的 bin。"""
    idx = bin_angle_deg(0.0, flip_horizontal=True, num_bins=N)
    # angle=0 对应 bin n/2
    assert idx == N // 2


def test_flip_true_90deg_cw_maps_to_minus_y():
    """flip=true：硬件 90°（CW 物理右）应映射到 ROS angle=-π/2（-Y 右）的 bin。"""
    idx = bin_angle_deg(90.0, flip_horizontal=True, num_bins=N)
    # angle=-π/2 对应 bin n/4
    assert idx == N // 4


def test_flip_true_270deg_cw_maps_to_plus_y():
    """flip=true：硬件 270°（CW 物理左）应映射到 ROS angle=π/2（+Y 左）的 bin。"""
    idx = bin_angle_deg(270.0, flip_horizontal=True, num_bins=N)
    # angle=π/2 对应 bin 3n/4
    assert idx == 3 * N // 4


def test_flip_true_180deg_unchanged():
    """flip=true：硬件 180°（后）应映射到 ROS angle=±π（-X 后）的 bin。"""
    idx = bin_angle_deg(180.0, flip_horizontal=True, num_bins=N)
    # angle=±π 对应 bin 0 或 n（取模后是 0）
    assert idx == 0


def test_flip_false_keeps_original_mapping():
    """flip=false：保留原始 [0, 2π] 映射（硬件 90° → bin n/4，用于回滚对比）。"""
    idx_90 = bin_angle_deg(90.0, flip_horizontal=False, num_bins=N)
    # 原始映射：ang_deg 直接除 360 乘 n
    assert idx_90 == int(90.0 / 360.0 * N) % N


def test_index_always_in_range():
    """任何 ang_deg（包括 360°、负数、超大值）都应返回 [0, n) 内的索引。"""
    for ang in [-720.0, -0.5, 0.0, 90.0, 180.0, 359.99, 360.0, 720.0, 1e6]:
        idx = bin_angle_deg(ang, flip_horizontal=True, num_bins=N)
        assert 0 <= idx < N
        idx = bin_angle_deg(ang, flip_horizontal=False, num_bins=N)
        assert 0 <= idx < N
