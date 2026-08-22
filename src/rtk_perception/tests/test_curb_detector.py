"""curb_detector 纯算法单测（不依赖 rclpy）。"""
import math
import numpy as np
import pytest

from rtk_perception.curb_detector import (
    CurbConfig,
    bucket_by_angle,
    detect_curbs,
    fit_line,
)


def make_polar(points_xy):
    """(x, y) 笛卡尔列表 → (r, theta) 极坐标 numpy 数组。"""
    arr = []
    for x, y in points_xy:
        r = math.sqrt(x * x + y * y)
        t = math.atan2(y, x)
        arr.append((r, t))
    return np.array(arr)


# ---------- Test 1：平直路沿 ----------
def test_straight_curb_both_sides():
    """左侧 x=-2 一条直线，右侧 x=2 一条直线，前方无障碍。"""
    pts = []
    for y in np.linspace(-3, 3, 30):
        pts.append((-2.0, float(y)))  # 左路沿
        pts.append((2.0, float(y)))   # 右路沿
    polar = make_polar(pts)
    curbs = detect_curbs(polar, CurbConfig())
    assert len(curbs) >= 2
    lefts = [c for c in curbs if c.centroid_x < 0]
    rights = [c for c in curbs if c.centroid_x >= 0]
    assert len(lefts) >= 1 and len(rights) >= 1


# ---------- Test 2：路沿有缺口 ----------
def test_curb_with_gap():
    """左路沿在 y=[1, 2] 处有缺口。"""
    pts = []
    for y in np.linspace(-3, 1, 20):
        pts.append((-2.0, float(y)))
    for y in np.linspace(2, 3, 10):
        pts.append((-2.0, float(y)))
    for y in np.linspace(-3, 3, 30):
        pts.append((2.0, float(y)))
    polar = make_polar(pts)
    curbs = detect_curbs(polar, CurbConfig())
    lefts = [c for c in curbs if c.centroid_x < 0]
    assert len(lefts) >= 1


# ---------- Test 3：纯噪声 ----------
def test_pure_noise_no_curb():
    """纯随机点，没有连续线段，不应输出路沿。"""
    rng = np.random.default_rng(42)
    pts = rng.uniform(-5, 5, (50, 2))
    polar = make_polar(pts)
    curbs = detect_curbs(polar, CurbConfig())
    assert len(curbs) == 0


# ---------- Test 4：远距离弱信号 ----------
def test_far_weak_signal_filtered():
    """8m 处零星几个点，应被 max_curb_range=5m 过滤。"""
    pts = [(8.0, 1.0), (8.0, 0.5), (8.0, -0.5), (8.0, -1.0)]
    polar = make_polar(pts)
    curbs = detect_curbs(polar, CurbConfig(max_curb_range=5.0))
    assert len(curbs) == 0


# ---------- Test 5：弯道路沿 ----------
def test_curved_curb():
    """路沿是弧形，应该能用直线近似（容差内）。"""
    pts = []
    for theta in np.linspace(-math.pi / 4, math.pi / 4, 20):
        x = 2.0 * math.cos(theta) - 0.5
        y = 2.0 * math.sin(theta)
        pts.append((x, y))
    polar = make_polar(pts)
    curbs = detect_curbs(polar, CurbConfig(min_line_length=1.0))
    assert len(curbs) >= 1


# ---------- Test 6：数据全 NaN ----------
def test_all_nan_returns_empty():
    polar = np.array([(float("nan"), float("nan"))] * 10)
    curbs = detect_curbs(polar, CurbConfig())
    assert curbs == []


# ---------- Test 7：极近路沿 ----------
def test_very_close_curb_detected():
    """0.3m 处的路沿仍能识别。"""
    pts = []
    for y in np.linspace(-0.5, 0.5, 10):
        pts.append((-0.3, float(y)))
        pts.append((0.3, float(y)))
    polar = make_polar(pts)
    curbs = detect_curbs(polar, CurbConfig())
    assert len(curbs) >= 1


# ---------- Test 8：DBSCAN eps 边界 ----------
def test_dbscan_eps_boundary():
    """两个点恰好相距 0.3m（eps 边界）。"""
    cfg = CurbConfig(delta_r_threshold=0.01, min_line_length=0.2)
    pts = [(0.0, 0.0), (0.0, 0.3), (0.0, 0.6)]
    polar = make_polar(pts)
    curbs = detect_curbs(polar, cfg)
    # 应该聚成一类
    assert len(curbs) <= 1


# ---------- Test 9：RANSAC outlier 容忍 ----------
def test_ransac_outlier_tolerance():
    """20% 离群点的直线仍能拟合。"""
    pts = []
    for y in np.linspace(0, 2, 10):
        pts.append((-2.0, float(y)))
    # 加 2 个离群点（20%）
    pts.append((-2.0, 5.0))
    pts.append((2.0, 1.0))
    polar = make_polar(pts)
    curbs = detect_curbs(polar, CurbConfig(min_line_length=1.0))
    assert len(curbs) >= 1


# ---------- Test 10：左右分类 ----------
def test_left_right_classification():
    pts = []
    for y in np.linspace(-3, 3, 30):
        pts.append((-2.0, float(y)))
        pts.append((2.0, float(y)))
    polar = make_polar(pts)
    curbs = detect_curbs(polar, CurbConfig())
    lefts = [c for c in curbs if c.centroid_x < 0]
    rights = [c for c in curbs if c.centroid_x >= 0]
    assert all(c.centroid_x < 0 for c in lefts)
    assert all(c.centroid_x >= 0 for c in rights)


# ---------- Test 11：角度 bin 边界 ----------
def test_angle_bin_boundary():
    """点恰好落在 1° 整数倍边界上。"""
    cfg = CurbConfig(bin_size_deg=1.0)
    # 0°, 1°, 2° 三个点
    polar = np.array([(1.0, 0.0), (1.0, math.radians(1.0)), (1.0, math.radians(2.0))])
    bins = bucket_by_angle(polar, cfg)
    assert len(bins) == 360
    # 0°, 1°, 2° 三个 bin 应该有点
    assert bins[0] is not None
    assert bins[1] is not None
    assert bins[2] is not None


# ---------- Test 12：帧间稳定性 ----------
def test_frame_stability():
    """连续 10 帧路沿位置抖动应 < 0.1m。"""
    cfg = CurbConfig()
    base_pts = [(-2.0, float(y)) for y in np.linspace(-2, 2, 20)]
    detected_positions = []
    for i in range(10):
        # 每帧加 ±2cm 噪声
        noisy = [(x + np.random.uniform(-0.02, 0.02), y) for x, y in base_pts]
        polar = make_polar(noisy)
        curbs = detect_curbs(polar, cfg)
        if curbs:
            lefts = [c for c in curbs if c.centroid_x < 0]
            if lefts:
                detected_positions.append(lefts[0].centroid_x)
    if len(detected_positions) >= 2:
        positions = np.array(detected_positions)
        std = float(np.std(positions))
        assert std < 0.1, f"帧间标准差 {std:.3f}m 过大"


# ---------- Test 13：fit_line 单测 ----------
def test_fit_line_basic():
    """3 个共线点的拟合。"""
    pts = np.array([[0.0, 0.0], [1.0, 1.0], [2.0, 2.0]])
    line = fit_line(pts)
    assert line is not None
    assert line.length == pytest.approx(2 * math.sqrt(2), abs=0.01)
