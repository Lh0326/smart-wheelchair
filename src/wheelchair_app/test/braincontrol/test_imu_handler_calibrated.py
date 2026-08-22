"""ImuHandler.calibrated_roll 公开方法测试（spec § 7.1）。"""
from wheelchair_app.braincontrol.imu_handler import ImuHandler


def test_calibrated_roll_uncalibrated_returns_zero():
    """未校准时 calibrated_roll 总返回 0.0。"""
    h = ImuHandler()
    assert h.calibrated_roll(10.0) == 0.0


def test_calibrated_roll_subtracts_baseline():
    """已校准，roll=10, baseline=2 → 返回 8。"""
    h = ImuHandler()
    h.feed_calibration(0.0, 2.0)
    h.feed_calibration(0.0, 2.0)
    h.finish_calibration()
    assert h.calibrated_roll(10.0) == 8.0


def test_calibrated_roll_negative_baseline():
    """已校准，roll=-5, baseline=3 → 返回 -8。"""
    h = ImuHandler()
    h.feed_calibration(0.0, 3.0)
    h.feed_calibration(0.0, 3.0)
    h.finish_calibration()
    assert h.calibrated_roll(-5.0) == -8.0


def test_calibrated_roll_at_baseline_returns_zero():
    """roll 恰好等于 baseline → 返回 0.0。"""
    h = ImuHandler()
    h.feed_calibration(0.0, 5.0)
    h.feed_calibration(0.0, 5.0)
    h.finish_calibration()
    assert h.calibrated_roll(5.0) == 0.0
