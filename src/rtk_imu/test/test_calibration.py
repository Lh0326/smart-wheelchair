"""BiasCalibrator 静态 bias 校准单测"""
import pytest

from rtk_imu.calibration import BiasCalibrator
from rtk_imu.packet_parser import PacketResult
from rtk_imu.jy901_protocol import REG_ACC, REG_GYRO


def _gyro_result(wx, wy, wz):
    return PacketResult(reg=REG_GYRO, gyro=[wx, wy, wz])


def _acc_result(ax, ay, az):
    return PacketResult(reg=REG_ACC, acc=[ax, ay, az])


# ========== 初始状态 ==========

def test_initially_calibrating():
    cal = BiasCalibrator(duration_sec=3.0)
    assert cal.is_calibrating() is True


def test_initial_bias_zero():
    cal = BiasCalibrator(duration_sec=3.0)
    assert cal.get_bias() == [0.0, 0.0, 0.0]


# ========== 校准过程 ==========

def test_calibration_completes_after_duration():
    """喂入足够时间的数据后，校准完成"""
    cal = BiasCalibrator(duration_sec=1.0)
    # 模拟 1.5 秒数据（100Hz × 1.5s = 150 包）
    import time
    start = time.monotonic()
    for _ in range(150):
        cal.feed(_gyro_result(0.001, -0.002, 0.003))
        if not cal.is_calibrating():
            break
    assert cal.is_calibrating() is False


def test_bias_is_average_of_samples():
    """校准完成后，bias 应为采样的平均值"""
    cal = BiasCalibrator(duration_sec=0.5)  # 短时长便于测试
    import time
    deadline = time.monotonic() + 0.6
    while time.monotonic() < deadline:
        cal.feed(_gyro_result(0.005, -0.003, 0.001))
    # 等校准完成
    assert cal.is_calibrating() is False
    bias = cal.get_bias()
    assert bias[0] == pytest.approx(0.005, abs=1e-3)
    assert bias[1] == pytest.approx(-0.003, abs=1e-3)
    assert bias[2] == pytest.approx(0.001, abs=1e-3)


def test_bias_subtraction_in_apply():
    """apply_to_gyro 应减去 bias"""
    cal = BiasCalibrator(duration_sec=0.5)
    import time
    deadline = time.monotonic() + 0.6
    while time.monotonic() < deadline:
        cal.feed(_gyro_result(0.01, 0.0, 0.0))
    assert cal.is_calibrating() is False
    # apply 后 gx 应近 0
    corrected = cal.apply_to_gyro([0.01, 0.002, 0.003])
    assert corrected[0] == pytest.approx(0.0, abs=1e-3)
    assert corrected[1] == pytest.approx(0.002, abs=1e-3)


def test_acc_packet_records_gravity():
    """加速度包在校准期被记录，用于诊断重力向量"""
    cal = BiasCalibrator(duration_sec=0.5)
    import time
    deadline = time.monotonic() + 0.6
    while time.monotonic() < deadline:
        cal.feed(_acc_result(0.0, 0.0, 9.8))
    gravity = cal.get_gravity()
    assert gravity[2] == pytest.approx(9.8, abs=0.5)


# ========== 非校准期 feed ==========

def test_post_calibration_feed_ignored_for_bias():
    """校准完成后，feed 不再累积 bias"""
    cal = BiasCalibrator(duration_sec=0.5)
    import time
    deadline = time.monotonic() + 0.6
    while time.monotonic() < deadline:
        cal.feed(_gyro_result(0.005, 0.0, 0.0))
    assert cal.is_calibrating() is False
    bias_before = cal.get_bias()
    # 继续喂入不同值
    for _ in range(100):
        cal.feed(_gyro_result(0.5, 0.5, 0.5))
    bias_after = cal.get_bias()
    assert bias_after == bias_before


def test_minimum_samples_required():
    """少于 10 个样本不算完成（避免单包完成）"""
    cal = BiasCalibrator(duration_sec=0.01)  # 极短
    cal.feed(_gyro_result(0.0, 0.0, 0.0))
    cal.feed(_gyro_result(0.0, 0.0, 0.0))
    # 时间够但样本太少，仍在校准
    assert cal.is_calibrating() is True
