"""fusion_scan 纯算法单测（不依赖 rclpy）。

改造后（360 bins 重采样）：merge_two_scans 统一输出 360 bins，
测试改为验证重采样 + 融合的正确性。
"""
import math
import pytest

from rtk_perception.fusion_scan_node import merge_two_scans, MergeConfig, _resample_to_360bins

NUM_BINS = 360


def _make_scan(ranges, angle_min=-math.pi, angle_max=math.pi):
    """构造 scan dict。"""
    n = len(ranges)
    inc = (angle_max - angle_min) / max(n - 1, 1)
    return {
        "ranges": ranges,
        "angle_min": angle_min,
        "angle_max": angle_max,
        "angle_increment": inc,
    }


def test_both_finite_take_min():
    """两个有效距离取最小（同角度）。"""
    # 两个 scan 都在 angle=0（前方）放障碍
    bin_0 = NUM_BINS // 2  # angle=0 对应 bin 180
    a = [10.0] * 10
    b = [10.0] * 10
    # 在 angle≈0 处放障碍
    a_scan = _make_scan_with_obstacle(a_angle=0.0, a_dist=1.0)
    b_scan = _make_scan_with_obstacle(a_angle=0.0, a_dist=0.5)
    out = merge_two_scans(a_scan, b_scan, MergeConfig())
    # bin 180（angle=0）应该取 min(1.0, 0.5) = 0.5
    assert out["ranges"][bin_0] == pytest.approx(0.5, abs=0.01)


def _make_scan_with_obstacle(a_angle, a_dist, num_points=360, default_dist=10.0):
    """在指定角度放一个障碍点。"""
    angle_min = -math.pi
    angle_max = math.pi
    inc = (angle_max - angle_min) / num_points
    ranges = [default_dist] * num_points
    # 找到最接近 a_angle 的点
    target_idx = int(round((a_angle - angle_min) / inc)) % num_points
    ranges[target_idx] = a_dist
    return {
        "ranges": ranges,
        "angle_min": angle_min,
        "angle_max": angle_max,
        "angle_increment": inc,
    }


def test_one_inf_take_other():
    """一个 scan 无数据（inf），另一个有 → 取有的。"""
    a_scan = _make_scan_with_obstacle(a_angle=0.0, a_dist=2.0)
    b_scan = _make_scan([float("inf")] * 360)
    out = merge_two_scans(a_scan, b_scan, MergeConfig())
    bin_0 = NUM_BINS // 2
    assert out["ranges"][bin_0] == pytest.approx(2.0, abs=0.01)


def test_both_inf_stay_inf():
    """两个 scan 都 inf → 输出 inf。"""
    a = _make_scan([float("inf")] * 10)
    b = _make_scan([float("inf")] * 10)
    out = merge_two_scans(a, b, MergeConfig())
    assert math.isinf(out["ranges"][0])


def test_different_angle_conventions():
    """N10P [-π,π] 和 LD14P [0,2π] 同一物理方向应正确融合。

    关键测试：LD14P angle=0（前方，在 [0,2π] 约定下）应映射到 bin 180（[-π,π] 的 0°）。
    """
    # N10P: [-π, π]，前方 0° 放障碍 1.0m
    n10p = _make_scan_with_obstacle(a_angle=0.0, a_dist=1.0, num_points=5400)
    # LD14P: [0, 2π]，前方 0° 放障碍 0.5m
    ld14p_ranges = [10.0] * 667
    ld14p_inc = 2 * math.pi / 667
    ld14p_target_idx = 0  # angle=0 对应 index 0
    ld14p_ranges[0] = 0.5
    ld14p = {
        "ranges": ld14p_ranges,
        "angle_min": 0.0,
        "angle_max": 2 * math.pi,
        "angle_increment": ld14p_inc,
    }
    out = merge_two_scans(n10p, ld14p, MergeConfig())
    # bin 180（[-π,π] 的 0° = 前方）应该取 min(1.0, 0.5) = 0.5
    bin_front = NUM_BINS // 2
    assert out["ranges"][bin_front] == pytest.approx(0.5, abs=0.01), \
        f"前方 bin 应为 0.5，实际 {out['ranges'][bin_front]}"


def test_output_always_360_bins():
    """输出固定 360 bins（无论输入多少点）。"""
    a = _make_scan([1.0] * 100)
    b = _make_scan([2.0] * 500)
    out = merge_two_scans(a, b, MergeConfig())
    assert len(out["ranges"]) == NUM_BINS
    assert out["angle_min"] == pytest.approx(-math.pi)
    assert out["angle_increment"] == pytest.approx(2 * math.pi / NUM_BINS)


def test_resample_to_360bins_angle_mapping():
    """重采样：angle=0（前方）应映射到 bin 180。"""
    # 在 angle=0 放障碍
    scan = _make_scan_with_obstacle(a_angle=0.0, a_dist=1.0, num_points=360)
    bins = _resample_to_360bins(scan, NUM_BINS)
    bin_front = NUM_BINS // 2
    assert bins[bin_front] == pytest.approx(1.0, abs=0.01)


def test_resample_handles_0_to_2pi():
    """LD14P [0, 2π] 约定：angle=0 应映射到 bin 180（[-π,π] 的 0°）。"""
    ld14p_ranges = [10.0] * 667
    ld14p_ranges[0] = 1.0  # angle=0 放障碍
    ld14p = {
        "ranges": ld14p_ranges,
        "angle_min": 0.0,
        "angle_max": 2 * math.pi,
        "angle_increment": 2 * math.pi / 667,
    }
    bins = _resample_to_360bins(ld14p, NUM_BINS)
    bin_front = NUM_BINS // 2  # angle=0 对应 bin 180
    assert bins[bin_front] == pytest.approx(1.0, abs=0.01), \
        f"angle=0 应映射到 bin 180，实际值 {bins[bin_front]}"


def test_three_way_merge_all_finite_take_min():
    """三路 scan 同角度都有效 → 取 min(r)。"""
    from rtk_perception.fusion_scan_node import merge_n_scans
    a = _make_scan_with_obstacle(a_angle=0.0, a_dist=3.0)
    b = _make_scan_with_obstacle(a_angle=0.0, a_dist=2.0)
    c = _make_scan_with_obstacle(a_angle=0.0, a_dist=1.0)
    out = merge_n_scans([a, b, c], MergeConfig())
    bin_0 = NUM_BINS // 2
    assert out["ranges"][bin_0] == pytest.approx(1.0, abs=0.01)


def test_three_way_merge_one_inf_ignored():
    """三路中一路 inf → 自动跳过，取其他两路 min。"""
    from rtk_perception.fusion_scan_node import merge_n_scans
    a = _make_scan_with_obstacle(a_angle=0.0, a_dist=2.0)
    b = _make_scan_with_obstacle(a_angle=0.0, a_dist=1.5)
    c = _make_scan([float("inf")] * 360)
    out = merge_n_scans([a, b, c], MergeConfig())
    bin_0 = NUM_BINS // 2
    assert out["ranges"][bin_0] == pytest.approx(1.5, abs=0.01)


def test_three_way_merge_two_inf_still_works():
    """三路中两路 inf → 取唯一有效那路。"""
    from rtk_perception.fusion_scan_node import merge_n_scans
    a = _make_scan_with_obstacle(a_angle=0.0, a_dist=2.0)
    b = _make_scan([float("inf")] * 360)
    c = _make_scan([float("inf")] * 360)
    out = merge_n_scans([a, b, c], MergeConfig())
    bin_0 = NUM_BINS // 2
    assert out["ranges"][bin_0] == pytest.approx(2.0, abs=0.01)


def test_three_way_merge_camera_only_in_fov():
    """相机 scan FOV 外为 inf（±32° 之外），不污染其他角度。"""
    from rtk_perception.fusion_scan_node import merge_n_scans
    # N10P 全 360° 有数据 5.0m
    a = _make_scan_with_obstacle(a_angle=0.0, a_dist=5.0, default_dist=5.0)
    # 相机只在 ±32° 内有数据 1.0m，其他全 inf
    c_ranges = [float("inf")] * 360
    # bin 180 = angle 0°，bin 180±32 = 相机 FOV 边界
    for offset in range(-32, 33):
        c_ranges[180 + offset] = 1.0
    c = {
        "ranges": c_ranges,
        "angle_min": -math.pi,
        "angle_max": math.pi,
        "angle_increment": 2 * math.pi / 360,
    }
    out = merge_n_scans([a, c], MergeConfig())
    # FOV 内（bin 180）取 min(5.0, 1.0) = 1.0
    assert out["ranges"][180] == pytest.approx(1.0, abs=0.01)
    # FOV 外（bin 90 = angle -90°）只有 N10P 数据 5.0
    assert out["ranges"][90] == pytest.approx(5.0, abs=0.01)


def test_merge_two_scans_still_works_back_compat():
    """merge_two_scans 仍可调用（向后兼容，不破坏现有测试）。"""
    a = _make_scan_with_obstacle(a_angle=0.0, a_dist=1.0)
    b = _make_scan_with_obstacle(a_angle=0.0, a_dist=0.5)
    out = merge_two_scans(a, b, MergeConfig())
    bin_0 = NUM_BINS // 2
    assert out["ranges"][bin_0] == pytest.approx(0.5, abs=0.01)
