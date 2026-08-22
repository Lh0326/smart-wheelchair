"""check_oscillation.py 纯逻辑单测（不依赖 rosbag）。"""
import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

from check_oscillation import find_turning_windows, count_zero_crossings
import numpy as np


def test_find_turning_windows_basic():
    """持续 1s 的拐弯窗口被识别"""
    # 10Hz 采样 2s，0-1s 是 wz=0.5（拐弯），1-2s 是 wz=0
    times = np.array([i * 0.1 for i in range(20)])
    wzs = np.array([0.5] * 10 + [0.0] * 10)
    windows = find_turning_windows(times, wzs)
    assert len(windows) == 1
    s, e = windows[0]
    assert s == 0 and e == 9


def test_find_turning_windows_short_blip_ignored():
    """短脉冲（< 0.5s）不算拐弯窗口"""
    times = np.array([i * 0.1 for i in range(20)])
    wzs = np.array([0.5] * 3 + [0.0] * 17)  # 只有 0.3s
    windows = find_turning_windows(times, wzs)
    assert len(windows) == 0


def test_count_zero_crossings_no_crossing():
    """单调符号无过零"""
    wzs = np.array([0.5, 0.6, 0.7, 0.5])
    assert count_zero_crossings(wzs) == 0


def test_count_zero_crossings_one_flip():
    """一次符号翻转"""
    wzs = np.array([0.5, -0.5, -0.6])
    assert count_zero_crossings(wzs) == 1


def test_count_zero_crossings_oscillation():
    """多次翻转（震荡）"""
    wzs = np.array([0.5, -0.5, 0.5, -0.5, 0.5])
    assert count_zero_crossings(wzs) == 4


def test_count_zero_crossings_zero_band_filter():
    """|wz| < 0.05 视为零，不算翻转"""
    wzs = np.array([0.5, 0.03, 0.5])  # 中间 0.03 被忽略
    assert count_zero_crossings(wzs) == 0


def test_count_zero_crossings_teb_smooth_curve():
    """模拟 TEB 平滑拐弯：持续正向无翻转"""
    import math
    # 100 个采样点，wz 从 0.3 平滑升到 0.8 再降回 0.3
    wzs = np.array([0.3 + 0.5 * math.sin(i / 100 * math.pi) for i in range(100)])
    assert count_zero_crossings(wzs) == 0  # 全正，0 过零


if __name__ == "__main__":
    test_find_turning_windows_basic()
    test_find_turning_windows_short_blip_ignored()
    test_count_zero_crossings_no_crossing()
    test_count_zero_crossings_one_flip()
    test_count_zero_crossings_oscillation()
    test_count_zero_crossings_zero_band_filter()
    test_count_zero_crossings_teb_smooth_curve()
    print("All 7 tests passed")
