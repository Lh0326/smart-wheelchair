"""ChassisSlewLimiter + 辅助纯函数 Layer 1 单元测试。

不初始化 rclpy，纯函数/纯类 in/out 验证。秒级跑完。
"""
import math

import pytest


def test_wrap_deg_float_basic():
    """正常值原样返回（不取整）。"""
    from rtk_perception.chassis_serial_node import wrap_deg_float
    assert wrap_deg_float(45.5) == 45.5
    assert wrap_deg_float(-45.5) == -45.5


def test_wrap_deg_float_overflow():
    """超范围值归一化到 [-180, 180)。"""
    from rtk_perception.chassis_serial_node import wrap_deg_float
    assert wrap_deg_float(360.0) == 0.0
    assert wrap_deg_float(540.0) == -180.0
    assert wrap_deg_float(-270.0) == 90.0


def test_wrap_deg_float_boundary_difference():
    """175 与 -175 的差是 -10（不是 +350），用于跳变检测的 wrap_deg_float(target - prev)。"""
    from rtk_perception.chassis_serial_node import wrap_deg_float
    # 175 - (-175) = 350，wrap 后 = -10
    assert wrap_deg_float(175.0 - (-175.0)) == -10.0
    # -175 - 175 = -350，wrap 后 = 10
    assert wrap_deg_float(-175.0 - 175.0) == 10.0


def test_slew_speed_accel_up():
    """target > current + max_delta → 返回 current + max_delta。"""
    from rtk_perception.chassis_serial_node import slew_speed
    # ramp=2000/sec, dt=0.01 → max_delta=20
    # current=0, target=700 → 0+20=20
    assert slew_speed(target=700, current=0, ramp_per_sec=2000, dt=0.01) == 20


def test_slew_speed_accel_down():
    """target < current - max_delta → 返回 current - max_delta。"""
    from rtk_perception.chassis_serial_node import slew_speed
    # current=700, target=0 → 700-20=680
    assert slew_speed(target=0, current=700, ramp_per_sec=2000, dt=0.01) == 680


def test_slew_speed_within_rate():
    """|target - current| ≤ max_delta → 返回 target。"""
    from rtk_perception.chassis_serial_node import slew_speed
    # current=0, target=15, max_delta=20 → 15
    assert slew_speed(target=15, current=0, ramp_per_sec=2000, dt=0.01) == 15
    # current=0, target=-15 → -15
    assert slew_speed(target=-15, current=0, ramp_per_sec=2000, dt=0.01) == -15


def test_slew_speed_negative_target():
    """target 为负值（反向速度）也正确 slew。"""
    from rtk_perception.chassis_serial_node import slew_speed
    # current=0, target=-1000, max_delta=20 → -20
    assert slew_speed(target=-1000, current=0, ramp_per_sec=2000, dt=0.01) == -20


def test_slew_speed_dt_clamp():
    """dt 异常大（>0.2）时 clamp 到 0.2，防单帧暴冲。"""
    from rtk_perception.chassis_serial_node import slew_speed
    # dt=10.0 应被 clamp 到 0.2，max_delta=2000*0.2=400
    # current=0, target=1000 → 400
    assert slew_speed(target=1000, current=0, ramp_per_sec=2000, dt=10.0) == 400


def test_slew_speed_nan_target():
    """NaN/Inf target 视为 0（与既有 compute_forward_speed 风格一致）。"""
    from rtk_perception.chassis_serial_node import slew_speed
    # NaN target → 0，current=100 → 0 ramp（100-20=80）
    assert slew_speed(target=float('nan'), current=100, ramp_per_sec=2000, dt=0.01) == 80
    # Inf target → 0（同 NaN 处理，保守）
    assert slew_speed(target=float('inf'), current=0, ramp_per_sec=2000, dt=0.01) == 0
    assert slew_speed(target=float('-inf'), current=0, ramp_per_sec=2000, dt=0.01) == 0


# ============================================================
# ChassisSlewLimiter 类测试
# ============================================================

def _make_limiter(**overrides):
    """构造默认参数的限速器，允许覆盖单个参数。"""
    from rtk_perception.chassis_serial_node import ChassisSlewLimiter
    defaults = dict(
        ramp_per_sec=2000,
        jump_threshold_deg=45.0,
        cooldown_sec=0.2,
        window_sec=1.0,
        count_threshold=3,
        long_cooldown_sec=0.8,
    )
    defaults.update(overrides)
    return ChassisSlewLimiter(**defaults)


# === 构造 + 校验 ===

def test_limiter_init_defaults():
    """构造后状态字段都是默认初值。"""
    lim = _make_limiter()
    assert lim._state == 'NORMAL'
    assert lim._current_speed == 0
    assert lim._last_direction is None
    assert lim._jump_timestamps == []
    assert lim._enabled is True


def test_limiter_init_validation_ramp_zero():
    """ramp_per_sec=0 触发 ValueError。"""
    with pytest.raises(ValueError, match="ramp_per_sec"):
        _make_limiter(ramp_per_sec=0)


def test_limiter_init_validation_ramp_too_large():
    """ramp_per_sec > 10000 触发 ValueError。"""
    with pytest.raises(ValueError, match="ramp_per_sec"):
        _make_limiter(ramp_per_sec=20000)


def test_limiter_init_validation_threshold_zero():
    """threshold=0 触发 ValueError。"""
    with pytest.raises(ValueError, match="jump_threshold_deg"):
        _make_limiter(jump_threshold_deg=0.0)


def test_limiter_init_validation_threshold_too_large():
    """threshold > 180 触发 ValueError。"""
    with pytest.raises(ValueError, match="jump_threshold_deg"):
        _make_limiter(jump_threshold_deg=200.0)


def test_limiter_init_validation_window():
    """window_sec <= 0 触发 ValueError。"""
    with pytest.raises(ValueError, match="window_sec"):
        _make_limiter(window_sec=0.0)


def test_limiter_init_validation_count_threshold():
    """count_threshold < 1 触发 ValueError。"""
    with pytest.raises(ValueError, match="count_threshold"):
        _make_limiter(count_threshold=0)


# === NORMAL 状态 ===

def test_limiter_normal_first_frame_from_zero():
    """首帧 last_direction=None → 不触发 jump，speed 从 0 ramp。"""
    lim = _make_limiter()
    direction, current, speed = lim.apply(
        direction_target=0.0, current_target=0, speed_target=700,
        dt=0.01, now=0.0,
    )
    assert direction == 0
    assert current == 0
    assert speed == 20  # 0 + 2000*0.01
    assert lim._state == 'NORMAL'
    assert lim._last_direction == 0.0
    assert lim._current_speed == 20


def test_limiter_normal_small_jump_no_cooldown():
    """小跳变（< threshold）→ 跟踪 target，不触发 COOLDOWN。"""
    lim = _make_limiter()
    lim.apply(direction_target=0.0, current_target=0, speed_target=300, dt=0.01, now=0.0)
    direction, _, _ = lim.apply(
        direction_target=30.0, current_target=0, speed_target=300,
        dt=0.01, now=0.01,
    )
    assert direction == 30
    assert lim._state == 'NORMAL'
    assert lim._last_direction == 30.0


def test_limiter_normal_continuous_speed_ramp():
    """连续帧 speed 按 slew rate 累积 ramp，最终达到 target。"""
    lim = _make_limiter()
    speed = 0
    for i in range(40):
        _, _, speed = lim.apply(
            direction_target=0.0, current_target=0, speed_target=700,
            dt=0.01, now=i * 0.01,
        )
    # 35 帧达到 700（2000*0.01=20/帧，700/20=35），之后保持
    assert speed == 700


# === COOLDOWN 状态机 ===

def test_limiter_jump_above_threshold_triggers_cooldown():
    """方向跳变 ≥ threshold → 进入 COOLDOWN。

    spec § 3.2 修订：COOLDOWN 期间输出 direction=current_at_trigger，
    让 PID error=0（不是 direction_at_trigger）。
    """
    lim = _make_limiter()
    lim.apply(direction_target=0.0, current_target=0, speed_target=300, dt=0.01, now=0.0)
    direction, current_out, _ = lim.apply(
        direction_target=90.0, current_target=0, speed_target=300, dt=0.01, now=0.01,
    )
    assert lim._state == 'COOLDOWN'
    assert lim._direction_at_trigger == 0.0
    assert lim._current_at_trigger == 0
    assert lim._cooldown_until == pytest.approx(0.01 + 0.2)
    # COOLDOWN 期间 direction=current_at_trigger=0（让 PID error=0）
    assert direction == 0
    assert current_out == 0


def test_limiter_cooldown_speed_target_zero():
    """COOLDOWN 期间 forward_speed target=0（自然 ramp 到 0）。"""
    lim = _make_limiter()
    # 先 ramp 到中速 300（15 帧）
    for i in range(15):
        _, _, speed = lim.apply(
            direction_target=0.0, current_target=0, speed_target=300,
            dt=0.01, now=i * 0.01,
        )
    assert speed == 300
    # 触发 COOLDOWN（本帧 speed target=0，ramp 300→280）
    lim.apply(direction_target=90.0, current_target=0, speed_target=300, dt=0.01, now=0.15)
    assert lim._state == 'COOLDOWN'
    # COOLDOWN 第二帧：speed 从 280 继续 ramp 到 260
    _, _, speed = lim.apply(
        direction_target=90.0, current_target=0, speed_target=300, dt=0.01, now=0.16,
    )
    assert speed == 260  # 280 - 20


def test_limiter_cooldown_expires_returns_to_normal():
    """cooldown_sec 后回到 NORMAL。"""
    lim = _make_limiter()
    lim.apply(direction_target=0.0, current_target=0, speed_target=300, dt=0.01, now=0.0)
    # 触发 COOLDOWN（cooldown_until = 0.01 + 0.2 = 0.21）
    lim.apply(direction_target=90.0, current_target=0, speed_target=300, dt=0.01, now=0.01)
    assert lim._state == 'COOLDOWN'
    # 在 cooldown 期间（now=0.1 < 0.21）
    lim.apply(direction_target=90.0, current_target=0, speed_target=300, dt=0.01, now=0.1)
    assert lim._state == 'COOLDOWN'
    # cooldown 结束（now=0.21）
    direction, _, _ = lim.apply(
        direction_target=90.0, current_target=0, speed_target=300, dt=0.01, now=0.21,
    )
    assert lim._state == 'NORMAL'
    assert direction == 90


def test_limiter_cooldown_direction_uses_current_at_trigger():
    """COOLDOWN 期间 direction_out = current_at_trigger（spec § 3.2 修订）。

    修订原因：让下位机 PID 看到 error = direction - current = 0，
    避免 speed=0 时 PID 仍驱动两轮做反向差速（实测电源冲击根因）。

    旧设计是 direction = direction_at_trigger（保持旧方向），但下位机
    PID 看到 error=-90 仍驱动电机，限速无效。
    """
    lim = _make_limiter()
    # current_target=5（模拟 HWT 报告航向，与 direction_target 不同）
    lim.apply(direction_target=-90.0, current_target=5, speed_target=300, dt=0.01, now=0.0)
    # LEFT(-90) → RIGHT(+90)：jump=180
    direction, current_out, _ = lim.apply(
        direction_target=90.0, current_target=5, speed_target=300, dt=0.01, now=0.01,
    )
    # COOLDOWN 期间 direction=current_at_trigger=5（不是 -90 也不是 +90）
    assert direction == 5
    assert current_out == 5
    assert lim._current_at_trigger == 5
    # COOLDOWN 中第二帧（current 可能变化但限速器输出锁定的还是触发时快照）
    direction, current_out, _ = lim.apply(
        direction_target=90.0, current_target=8, speed_target=300, dt=0.01, now=0.05,
    )
    assert direction == 5  # 仍是触发时的 current_at_trigger
    assert current_out == 5


def test_limiter_wrap_boundary_no_false_trigger():
    """175° → -175° 的差是 -10°（不是 +350°），不触发 COOLDOWN。"""
    lim = _make_limiter()
    lim.apply(direction_target=175.0, current_target=0, speed_target=300, dt=0.01, now=0.0)
    direction, _, _ = lim.apply(
        direction_target=-175.0, current_target=0, speed_target=300, dt=0.01, now=0.01,
    )
    assert lim._state == 'NORMAL'
    assert direction == -175


def test_limiter_forward_to_right_triggers_cooldown():
    """用户反馈关键场景：FORWARD(0) → RIGHT(90) jump=90 触发 COOLDOWN。

    spec § 3.2 修订：COOLDOWN 期间 direction=current_at_trigger=0
    （不是 direction_at_trigger=0）。值相同因为 current_target=0。
    """
    lim = _make_limiter()
    lim.apply(direction_target=0.0, current_target=0, speed_target=700, dt=0.01, now=0.0)
    direction, current_out, speed = lim.apply(
        direction_target=90.0, current_target=0, speed_target=300, dt=0.01, now=0.01,
    )
    assert lim._state == 'COOLDOWN'
    assert direction == 0  # current_at_trigger=0
    assert current_out == 0
    # speed target=0，从 20 ramp
    assert speed == 0  # 20-20=0


def test_limiter_speed_reverse_no_jump():
    """FORWARD(speed=+700) → BACKWARD(speed=-700) 方向不变，由 slew rate 平滑。"""
    lim = _make_limiter()
    for i in range(35):
        _, _, speed = lim.apply(
            direction_target=0.0, current_target=0, speed_target=700,
            dt=0.01, now=i * 0.01,
        )
    assert speed == 700
    direction, _, speed = lim.apply(
        direction_target=0.0, current_target=0, speed_target=-700,
        dt=0.01, now=0.35,
    )
    assert lim._state == 'NORMAL'
    assert direction == 0
    assert speed == 680


def test_limiter_cooldown_no_reset_during_cooldown():
    """COOLDOWN 期间检测到新 jump 不重置 cooldown_until（防永久冷却）。"""
    lim = _make_limiter()
    lim.apply(direction_target=0.0, current_target=0, speed_target=300, dt=0.01, now=0.0)
    lim.apply(direction_target=90.0, current_target=0, speed_target=300, dt=0.01, now=0.01)
    first_cooldown_until = lim._cooldown_until
    # 在 COOLDOWN 中再次发 jump（target 反向）— 但 COOLDOWN 中不检测 jump
    lim.apply(direction_target=-90.0, current_target=0, speed_target=300, dt=0.01, now=0.1)
    assert lim._cooldown_until == first_cooldown_until


# === LONG_COOLDOWN 高频切换兜底 ===

def test_limiter_long_cooldown_below_threshold_no_trigger():
    """窗口内 < count_threshold 次跳变 → 不触发 LONG_COOLDOWN。"""
    lim = _make_limiter()
    lim.apply(direction_target=-90.0, current_target=0, speed_target=300, dt=0.01, now=0.0)
    # 2 次跳变（< 3）
    lim.apply(direction_target=90.0, current_target=0, speed_target=300, dt=0.01, now=0.2)
    assert lim._state == 'COOLDOWN'  # 不是 LONG_COOLDOWN
    # COOLDOWN 结束（now=0.4 > 0.4 边界）
    lim.apply(direction_target=90.0, current_target=0, speed_target=300, dt=0.01, now=0.41)
    assert lim._state == 'NORMAL'
    # 第 2 次跳变
    lim.apply(direction_target=-90.0, current_target=0, speed_target=300, dt=0.01, now=0.6)
    assert lim._state == 'COOLDOWN'  # 仍不是 LONG_COOLDOWN
    assert len(lim._jump_timestamps) == 2


def test_limiter_long_cooldown_triggers_at_threshold():
    """窗口内 ≥ count_threshold 次跳变 → 触发 LONG_COOLDOWN。"""
    lim = _make_limiter()
    lim.apply(direction_target=-90.0, current_target=0, speed_target=300, dt=0.01, now=0.0)
    # 3 次跳变，每次间隔足够（含 COOLDOWN）
    # 第 1 次（t=0.2）
    lim.apply(direction_target=90.0, current_target=0, speed_target=300, dt=0.01, now=0.2)
    assert lim._state == 'COOLDOWN'
    # 第 1 次 COOLDOWN 结束 + 第 2 次跳变（t=0.41）
    lim.apply(direction_target=90.0, current_target=0, speed_target=300, dt=0.01, now=0.41)
    assert lim._state == 'NORMAL'
    lim.apply(direction_target=-90.0, current_target=0, speed_target=300, dt=0.01, now=0.41)
    assert lim._state == 'COOLDOWN'
    assert len(lim._jump_timestamps) == 2
    # 第 2 次 COOLDOWN 结束 + 第 3 次跳变（t=0.62）
    lim.apply(direction_target=-90.0, current_target=0, speed_target=300, dt=0.01, now=0.62)
    assert lim._state == 'NORMAL'
    lim.apply(direction_target=90.0, current_target=0, speed_target=300, dt=0.01, now=0.62)
    # 第 3 次累计 → LONG_COOLDOWN
    assert lim._state == 'LONG_COOLDOWN'


def test_limiter_long_cooldown_expires_clears_timestamps():
    """LONG_COOLDOWN 结束 → 清空 _jump_timestamps + 回 NORMAL。"""
    lim = _make_limiter()
    # 直接构造 LONG_COOLDOWN 状态
    lim._state = 'LONG_COOLDOWN'
    lim._cooldown_until = 1.0
    lim._jump_timestamps = [0.5, 0.6, 0.7]
    lim._direction_at_trigger = -90.0
    # now=1.0 时 LONG_COOLDOWN 结束
    lim.apply(direction_target=90.0, current_target=0, speed_target=300, dt=0.01, now=1.0)
    assert lim._state == 'NORMAL'
    assert lim._jump_timestamps == []


def test_limiter_sliding_window_drops_old_timestamps():
    """滑动窗口外（> window_sec）的旧跳变不计入。"""
    lim = _make_limiter()
    # 直接调用 _record_jump_and_check_long（私有方法但可访问）
    lim._record_jump_and_check_long(0.0)
    lim._record_jump_and_check_long(0.1)
    assert len(lim._jump_timestamps) == 2
    # now=1.5 时窗口 [0.5, 1.5]，旧记录 0.0/0.1 被清理
    lim._record_jump_and_check_long(1.5)
    assert lim._jump_timestamps == [1.5]


# === 总开关 + 协同 ===

def test_limiter_disabled_passthrough():
    """禁用时直通 target（不 slew），但更新内部状态。"""
    lim = _make_limiter()
    lim.set_enabled(False)
    direction, current, speed = lim.apply(
        direction_target=90.0, current_target=0, speed_target=700,
        dt=0.01, now=0.0,
    )
    assert direction == 90
    assert speed == 700
    assert lim._current_speed == 700
    assert lim._last_direction == 90.0


def test_limiter_reenable_preserves_current_speed():
    """禁用 → 重新启用时，从 _current_speed 继续，不重置为 0。"""
    lim = _make_limiter()
    lim.set_enabled(False)
    lim.apply(direction_target=0.0, current_target=0, speed_target=500, dt=0.01, now=0.0)
    assert lim._current_speed == 500
    lim.set_enabled(True)
    _, _, speed = lim.apply(
        direction_target=0.0, current_target=0, speed_target=500,
        dt=0.01, now=0.01,
    )
    assert speed == 500


def test_limiter_force_stop_target_zero_ramps_down():
    """mux 输出 target=0（force_stop/失联）→ speed 自然 ramp 到 0。"""
    lim = _make_limiter()
    for i in range(35):
        _, _, speed = lim.apply(
            direction_target=0.0, current_target=0, speed_target=700,
            dt=0.01, now=i * 0.01,
        )
    assert speed == 700
    _, _, speed = lim.apply(
        direction_target=0.0, current_target=0, speed_target=0, dt=0.01, now=0.35,
    )
    assert speed == 680  # 自然 ramp


def test_limiter_continuous_eeg_shake_long_cooldown():
    """EEG 连续摇头 LEFT/RIGHT/LEFT/RIGHT（4 次反转）→ 触发 LONG_COOLDOWN。"""
    lim = _make_limiter()
    # t=0 LEFT=-90 首帧
    lim.apply(direction_target=-90.0, current_target=0, speed_target=300, dt=0.01, now=0.0)
    # 4 次反转，间隔足够（每次包含 COOLDOWN）
    # 第 1 次：t=0.21 LEFT→RIGHT
    lim.apply(direction_target=90.0, current_target=0, speed_target=300, dt=0.01, now=0.21)
    assert lim._state == 'COOLDOWN'
    # COOLDOWN 结束 t=0.42，立即触发第 2 次：RIGHT→LEFT
    lim.apply(direction_target=90.0, current_target=0, speed_target=300, dt=0.01, now=0.42)
    lim.apply(direction_target=-90.0, current_target=0, speed_target=300, dt=0.01, now=0.42)
    assert lim._state == 'COOLDOWN'
    # COOLDOWN 结束 t=0.63，立即触发第 3 次：LEFT→RIGHT → LONG_COOLDOWN
    lim.apply(direction_target=-90.0, current_target=0, speed_target=300, dt=0.01, now=0.63)
    lim.apply(direction_target=90.0, current_target=0, speed_target=300, dt=0.01, now=0.63)
    assert lim._state == 'LONG_COOLDOWN'
