"""VFH+ 算法单测。"""
import math
import numpy as np
import pytest

from rtk_perception.vfh_plus import VFHConfig, LaserScanData, Twist2D, Intent


class TestVFHConfig:
    def test_default_values(self):
        cfg = VFHConfig()
        assert cfg.safety_distance_m == 0.5
        assert cfg.max_speed_m_s == 0.6
        assert cfg.sector_deg == 5.0
        assert cfg.front_fov_half_deg == 60.0
        assert cfg.cost_weight_prev_dir == 0.6
        assert cfg.angular_smoothing_alpha == 0.4

    def test_num_sectors_computed(self):
        cfg = VFHConfig()
        assert cfg.num_sectors == 72  # 360 / 5

    def test_custom_values(self):
        cfg = VFHConfig(sector_deg=10.0)
        assert cfg.num_sectors == 36


class TestDataclasses:
    def test_twist_default(self):
        t = Twist2D()
        assert t.linear_x == 0.0
        assert t.angular_z == 0.0

    def test_laser_scan_from_ranges(self):
        ranges = np.array([1.0, 2.0, 3.0])
        angles = np.array([0.0, 0.1, 0.2])
        scan = LaserScanData(ranges=ranges, angles=angles)
        assert len(scan.ranges) == 3


class TestAngleToSector:
    def test_front_zero(self):
        from rtk_perception.vfh_plus import angle_to_sector
        cfg = VFHConfig()
        assert angle_to_sector(0.0, cfg) == 0

    def test_90_degrees(self):
        from rtk_perception.vfh_plus import angle_to_sector
        cfg = VFHConfig()
        assert angle_to_sector(math.radians(90), cfg) == 18

    def test_negative_angle_wraps(self):
        from rtk_perception.vfh_plus import angle_to_sector
        cfg = VFHConfig()
        assert angle_to_sector(math.radians(-5), cfg) == 71


class TestBuildHistogram:
    def test_empty_scan(self):
        from rtk_perception.vfh_plus import build_histogram
        cfg = VFHConfig()
        scan = LaserScanData(ranges=np.array([]), angles=np.array([]))
        h = build_histogram(scan, cfg)
        assert len(h) == 72
        assert np.all(h == 0.0)

    def test_close_obstacle_high_weight(self):
        from rtk_perception.vfh_plus import build_histogram
        cfg = VFHConfig()
        scan = LaserScanData(
            ranges=np.array([0.5]),
            angles=np.array([0.0]),
        )
        h = build_histogram(scan, cfg)
        assert h[0] > 0.5

    def test_far_obstacle_low_weight(self):
        from rtk_perception.vfh_plus import build_histogram
        cfg = VFHConfig()
        scan = LaserScanData(
            ranges=np.array([5.0]),
            angles=np.array([0.0]),
        )
        h = build_histogram(scan, cfg)
        assert h[0] == 0.0

    def test_nan_ranges_ignored(self):
        from rtk_perception.vfh_plus import build_histogram
        cfg = VFHConfig()
        scan = LaserScanData(
            ranges=np.array([np.nan, 0.5, np.inf]),
            angles=np.array([0.0, 0.1, 0.2]),
        )
        h = build_histogram(scan, cfg)
        assert h[0] >= 0.0


class TestBinarize:
    def test_high_value_blocked(self):
        from rtk_perception.vfh_plus import binarize_with_hysteresis
        cfg = VFHConfig()
        histogram = np.zeros(72)
        histogram[0] = 2.0
        prev = np.zeros(72, dtype=int)
        binary = binarize_with_hysteresis(histogram, prev, cfg)
        assert binary[0] == 1

    def test_low_value_free(self):
        from rtk_perception.vfh_plus import binarize_with_hysteresis
        cfg = VFHConfig()
        histogram = np.zeros(72)
        histogram[0] = 0.1
        prev = np.zeros(72, dtype=int)
        binary = binarize_with_hysteresis(histogram, prev, cfg)
        assert binary[0] == 0

    def test_hysteresis_keeps_previous_state(self):
        from rtk_perception.vfh_plus import binarize_with_hysteresis
        cfg = VFHConfig()
        histogram = np.zeros(72)
        histogram[0] = 1.0
        prev = np.zeros(72, dtype=int)
        prev[0] = 1
        binary = binarize_with_hysteresis(histogram, prev, cfg)
        assert binary[0] == 1


class TestFindFreeSectors:
    def test_all_free_returns_full_circle(self):
        from rtk_perception.vfh_plus import find_free_sectors
        cfg = VFHConfig()
        masked = np.zeros(72, dtype=int)
        candidates = find_free_sectors(masked, cfg)
        # 全 free 时应覆盖整个圆
        total = 0
        for s, e in candidates:
            total += e - s if e > s else 72 - s + e
        assert total >= 72

    def test_all_blocked_returns_empty(self):
        from rtk_perception.vfh_plus import find_free_sectors
        cfg = VFHConfig()
        masked = np.ones(72, dtype=int)
        candidates = find_free_sectors(masked, cfg)
        assert candidates == []

    def test_single_blocked_sector_splits_into_two(self):
        from rtk_perception.vfh_plus import find_free_sectors
        cfg = VFHConfig()
        masked = np.zeros(72, dtype=int)
        masked[36] = 1
        candidates = find_free_sectors(masked, cfg)
        assert len(candidates) == 2


class TestSelectBest:
    def test_target_front_prefers_front_candidate(self):
        from rtk_perception.vfh_plus import select_best_direction
        cfg = VFHConfig()
        # (0, 36) 中心扇区 18 = 90°；但归一到 [-pi, pi] 后 = 90°
        # target = 0，应该选更接近 0° 的候选
        candidates = [(0, 18), (40, 60)]
        best = select_best_direction(candidates, target_rad=0.0, cfg=cfg)
        # 应该选 (0, 18) 中心 = 9*5 = 45°，比 (40,60) 中心 250° (= -110°) 接近 0°
        assert abs(best) < math.radians(120)

    def test_no_target_returns_small_angle(self):
        from rtk_perception.vfh_plus import select_best_direction
        cfg = VFHConfig()
        candidates = [(0, 10)]
        best = select_best_direction(candidates, target_rad=None, cfg=cfg)
        # 无目标时偏好当前航向（0°）
        assert abs(best) < math.radians(60)


class TestMinFrontDistance:
    def test_clear_front(self):
        from rtk_perception.vfh_plus import min_front_distance
        cfg = VFHConfig()
        scan = LaserScanData(
            ranges=np.array([5.0, 5.0, 5.0]),
            angles=np.array([-0.1, 0.0, 0.1]),
        )
        assert min_front_distance(scan, cfg) == pytest.approx(5.0, abs=0.01)

    def test_close_obstacle(self):
        from rtk_perception.vfh_plus import min_front_distance
        cfg = VFHConfig()
        scan = LaserScanData(
            ranges=np.array([0.5, 5.0]),
            angles=np.array([0.0, math.radians(90)]),
        )
        assert min_front_distance(scan, cfg) == pytest.approx(0.5, abs=0.01)

    def test_no_valid_returns_large(self):
        from rtk_perception.vfh_plus import min_front_distance
        cfg = VFHConfig()
        scan = LaserScanData(
            ranges=np.array([np.nan, np.nan]),
            angles=np.array([0.0, 0.1]),
        )
        d = min_front_distance(scan, cfg)
        assert d >= 25.0


class TestSafetyFence:
    def test_blocks_forward_when_too_close(self):
        from rtk_perception.vfh_plus import safety_fence
        cfg = VFHConfig()
        scan = LaserScanData(
            ranges=np.array([0.3, 0.3]),
            angles=np.array([0.0, 0.05]),
        )
        twist = Twist2D(linear_x=0.5)
        result = safety_fence(twist, scan, cfg)
        assert result.linear_x == 0.0

    def test_allows_forward_when_safe(self):
        from rtk_perception.vfh_plus import safety_fence
        cfg = VFHConfig()
        scan = LaserScanData(
            ranges=np.array([2.0, 2.0]),
            angles=np.array([0.0, 0.05]),
        )
        twist = Twist2D(linear_x=0.5)
        result = safety_fence(twist, scan, cfg)
        assert result.linear_x == 0.5

    def test_clamps_max_speed(self):
        from rtk_perception.vfh_plus import safety_fence
        cfg = VFHConfig()
        scan = LaserScanData(
            ranges=np.array([5.0]),
            angles=np.array([0.0]),
        )
        twist = Twist2D(linear_x=2.0)
        result = safety_fence(twist, scan, cfg)
        assert result.linear_x == cfg.max_speed_m_s

    def test_clamps_max_reverse(self):
        from rtk_perception.vfh_plus import safety_fence
        cfg = VFHConfig()
        scan = LaserScanData(
            ranges=np.array([5.0]),
            angles=np.array([0.0]),
        )
        twist = Twist2D(linear_x=-1.0)
        result = safety_fence(twist, scan, cfg)
        assert result.linear_x == -0.3


class TestVFHPlusIntegration:
    """VFHPlus 主类端到端集成测试。"""

    def _make_scan_with_obstacle(self, obstacle_dist, obstacle_angle_deg=0.0):
        """生成一个含单一障碍物的扫描，前方其余空旷。"""
        angles = np.linspace(-math.pi, math.pi, 360, endpoint=False)
        ranges = np.full(360, 10.0)
        target_angle = math.radians(obstacle_angle_deg)
        idx = int(round((target_angle + math.pi) / (2 * math.pi / 360))) % 360
        for offset in range(-3, 4):
            ranges[(idx + offset) % 360] = obstacle_dist
        return LaserScanData(ranges=ranges, angles=angles)

    def test_clear_path_goes_straight(self):
        from rtk_perception.vfh_plus import VFHPlus
        vfh = VFHPlus(VFHConfig())
        scan = LaserScanData(
            ranges=np.full(360, 10.0),
            angles=np.linspace(-math.pi, math.pi, 360, endpoint=False),
        )
        twist = vfh.compute(scan, Twist2D(0.5, 0.0), Twist2D(0.0, 0.0))
        assert twist.linear_x > 0.05
        assert abs(twist.angular_z) < 0.5

    def test_obstacle_ahead_causes_turn(self):
        from rtk_perception.vfh_plus import VFHPlus
        vfh = VFHPlus(VFHConfig())
        scan = self._make_scan_with_obstacle(obstacle_dist=1.0, obstacle_angle_deg=0.0)
        twist = vfh.compute(scan, Twist2D(0.5, 0.0), Twist2D(0.0, 0.0))
        assert abs(twist.angular_z) > 0.05

    def test_obstacle_too_close_brakes(self):
        from rtk_perception.vfh_plus import VFHPlus
        vfh = VFHPlus(VFHConfig())
        scan = self._make_scan_with_obstacle(obstacle_dist=0.3, obstacle_angle_deg=0.0)
        twist = vfh.compute(scan, Twist2D(0.5, 0.0), Twist2D(0.0, 0.0))
        assert twist.linear_x == 0.0

    def test_surrounded_obstacles_brakes(self):
        from rtk_perception.vfh_plus import VFHPlus
        vfh = VFHPlus(VFHConfig())
        scan = LaserScanData(
            ranges=np.full(360, 0.5),
            angles=np.linspace(-math.pi, math.pi, 360, endpoint=False),
        )
        twist = vfh.compute(scan, Twist2D(0.5, 0.0), Twist2D(0.0, 0.0))
        assert twist.linear_x == 0.0
