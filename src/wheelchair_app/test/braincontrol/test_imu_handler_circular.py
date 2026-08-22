"""ImuHandler 圆形 magnitude 滞回判定测试（spec § 3 Layer 1）。

覆盖：
- 中心点 / 第二圆圈内 → NONE
- 走出第二圆圈 → 对应方向（主导轴判定）
- 滞回区（10°-20°）保持当前方向
- 退出条件（< 10°）→ NONE
- bug 3 场景：FORWARD→LEFT 不卡死
- 主导轴切换 + 反向切换
"""
from wheelchair_app.braincontrol.imu_handler import ImuHandler
from wheelchair_app.braincontrol.control_types import TiltDirection


def _calibrated_handler() -> ImuHandler:
    """返回已校准的 ImuHandler（基线 pitch=0, roll=0）。"""
    h = ImuHandler()
    h.feed_calibration(0.0, 0.0)
    h.feed_calibration(0.0, 0.0)
    h.finish_calibration()
    return h


def test_center_is_none():
    """头部静止（mag < 10°）→ NONE。"""
    h = _calibrated_handler()
    assert h.update(3.0, 3.0) == TiltDirection.NONE  # mag=4.2°


def test_within_first_circle_is_none():
    """第一圆圈内（mag < 10°）→ NONE。"""
    h = _calibrated_handler()
    assert h.update(6.0, 6.0) == TiltDirection.NONE  # mag=8.5°


def test_within_second_circle_is_none():
    """第二圆圈内（10° <= mag < 20°）→ NONE。"""
    h = _calibrated_handler()
    assert h.update(10.0, 10.0) == TiltDirection.NONE  # mag=14.1°


def test_within_second_circle_high_mag_is_none():
    """仍在第二圆圈内（mag=18°）→ NONE。"""
    h = _calibrated_handler()
    assert h.update(15.0, 10.0) == TiltDirection.NONE  # mag=18.0°


def test_just_outside_second_circle_pitch():
    """刚好走出第二圆圈（mag=21°，pitch 主导）→ FORWARD。"""
    h = _calibrated_handler()
    assert h.update(21.0, 0.0) == TiltDirection.FORWARD


def test_just_outside_second_circle_roll():
    """刚好走出第二圆圈（mag=21°，roll 主导）→ LEFT。"""
    h = _calibrated_handler()
    assert h.update(0.0, 21.0) == TiltDirection.LEFT


def test_diagonal_pitch_dominant():
    """斜向走出第二圆圈（mag > 20°），pitch 主导 → FORWARD。"""
    h = _calibrated_handler()
    assert h.update(16.0, 14.0) == TiltDirection.FORWARD  # mag=21.3°


def test_diagonal_roll_dominant():
    """斜向走出第二圆圈，roll 主导 → LEFT。"""
    h = _calibrated_handler()
    assert h.update(10.0, 18.0) == TiltDirection.LEFT  # mag=20.6°


def test_hysteresis_hold_in_band():
    """已在 LEFT，输入 mag 在 10-20 之间 → 保持 LEFT（滞回）。"""
    h = _calibrated_handler()
    h.update(0.0, 25.0)  # 进入 LEFT
    assert h._current_dir == TiltDirection.LEFT
    # 滞回区内（mag=15.6°）
    result = h.update(10.0, 12.0)
    assert result == TiltDirection.LEFT


def test_exit_when_below_first_circle():
    """已在 LEFT，输入 mag < 10° → 退出回 NONE。"""
    h = _calibrated_handler()
    h.update(0.0, 25.0)  # 进入 LEFT
    result = h.update(5.0, 5.0)  # mag=7.07°
    assert result == TiltDirection.NONE


def test_bug3_forward_to_left_no_stuck():
    """bug 3 修复：从 FORWARD 直接移到 LEFT，不卡在 FORWARD。"""
    h = _calibrated_handler()
    h.update(20.0, 0.0)  # 进入 FORWARD
    assert h._current_dir == TiltDirection.FORWARD
    # 平滑过渡到 LEFT（magnitude 始终 > EXIT_DEG）
    result = h.update(5.0, 20.0)  # mag=20.6°，roll 主导
    assert result == TiltDirection.LEFT, f"应该切到 LEFT，实际 {result}（卡死 bug）"


def test_dominant_axis_switch_to_left_in_band():
    """已在 FORWARD，滞回区内 roll 变主导 → 切到 LEFT。"""
    h = _calibrated_handler()
    h.update(20.0, 0.0)  # FORWARD
    # 在滞回区（mag=20.6°），roll 主导
    result = h.update(10.0, 18.0)
    assert result == TiltDirection.LEFT


def test_reverse_to_backward():
    """已在 FORWARD，pitch 反向超 -20° → BACKWARD。"""
    h = _calibrated_handler()
    h.update(20.0, 0.0)  # FORWARD
    result = h.update(-20.0, 0.0)
    assert result == TiltDirection.BACKWARD


def test_uncalibrated_returns_none():
    """未校准时任何输入都返回 NONE。"""
    h = ImuHandler()  # 不校准
    assert h.update(30.0, 30.0) == TiltDirection.NONE
