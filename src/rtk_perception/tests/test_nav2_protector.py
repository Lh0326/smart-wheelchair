"""Nav2AutonomyProtector Layer 1 单元测试。

不初始化 rclpy，纯类 in/out 验证。秒级跑完。
"""
import pytest


def _make_protector(t_holder=None, params=None, logs=None):
    """构造 Protector + 注入可控时钟和日志收集器。"""
    from rtk_perception.nav2_protector import (
        Nav2AutonomyProtector,
        Nav2ProtectorParams,
    )

    if t_holder is None:
        t_holder = [0.0]

    def clock():
        return t_holder[0]

    def log(level, msg):
        if logs is not None:
            logs.append((level, msg))

    p = Nav2AutonomyProtector(
        params=params or Nav2ProtectorParams(),
        clock_fn=clock,
        log_fn=log,
    )
    return p, t_holder


def test_normal_path_no_trigger():
    """无触发时原样返回三字段。"""
    from rtk_perception.nav2_protector import FrameContext

    p, t = _make_protector()
    ctx = FrameContext(prev_direction_deg=0.0)
    d, c, s, status = p.apply(
        direction_deg=10.0, current_int=100, speed_float=500.0, frame_ctx=ctx
    )
    assert status == 'NORMAL'
    assert d == 10.0
    assert c == 100
    assert s == 500.0


def test_t1_lookahead_jump_triggers_ramp_cooldown():
    """direction 跳变 > 30° 触发 RAMP_COOLDOWN_LOOKAHEAD。"""
    from rtk_perception.nav2_protector import FrameContext

    p, t = _make_protector()
    # 第 0 帧：建立 baseline
    ctx0 = FrameContext(prev_direction_deg=0.0)
    p.apply(direction_deg=0.0, current_int=100, speed_float=500.0, frame_ctx=ctx0)

    # 第 1 帧：跳变 35°
    t[0] = 0.01
    ctx1 = FrameContext(prev_direction_deg=0.0)
    d, c, s, status = p.apply(
        direction_deg=35.0, current_int=100, speed_float=500.0, frame_ctx=ctx1
    )
    assert status == 'RAMP_COOLDOWN_LOOKAHEAD'
    # 触发瞬间锁定 direction=current_at_trigger=100
    assert d == 100.0
    assert c == 100


def test_ramp_cooldown_locks_current_to_at_trigger():
    """RAMP_COOLDOWN 期间 current_out 必须锁定到 current_at_trigger。

    这是自主导航摇摆+掉电 bug 的回归测试：
    - direction 锁定到 current_at_trigger（让 PID error=0）
    - current 也必须锁定到 current_at_trigger
    - 否则下位机看到 direction - current != 0 持续做反向差速 → 摇摆 + USB 过流
    """
    from rtk_perception.nav2_protector import FrameContext

    p, t = _make_protector()
    # 触发 RAMP_COOLDOWN
    ctx0 = FrameContext(prev_direction_deg=0.0)
    p.apply(direction_deg=0.0, current_int=100, speed_float=500.0, frame_ctx=ctx0)
    t[0] = 0.01
    ctx1 = FrameContext(prev_direction_deg=0.0)
    p.apply(direction_deg=35.0, current_int=100, speed_float=500.0, frame_ctx=ctx1)

    # 第 2 帧：IMU 转到 105°（轮椅在惯性运动）
    t[0] = 0.02
    ctx2 = FrameContext(prev_direction_deg=35.0)
    d, c, s, status = p.apply(
        direction_deg=40.0, current_int=105, speed_float=500.0, frame_ctx=ctx2
    )
    assert status == 'RAMP_COOLDOWN_LOOKAHEAD'
    # 关键断言：current 必须仍是 100，不能跟随 IMU 实时值变 105
    assert d == 100.0, f'direction 应锁定 100，实际 {d}'
    assert c == 100, f'current 必须锁定 100（PID error=0），实际 {c}'


def test_hard_cooldown_locks_current_to_at_trigger():
    """HARD_COOLDOWN 期间 current_out 必须锁定到 current_at_trigger。"""
    from rtk_perception.nav2_protector import FrameContext, Nav2ProtectorParams

    # escalation_count=2 → 第 2 次触发升级 HARD_COOLDOWN
    params = Nav2ProtectorParams(cooldown_escalation_count=2)
    p, t = _make_protector(params=params)

    # 触发 1：进 RAMP
    ctx0 = FrameContext(prev_direction_deg=0.0)
    p.apply(direction_deg=0.0, current_int=100, speed_float=500.0, frame_ctx=ctx0)
    t[0] = 0.01
    p.apply(
        direction_deg=35.0,
        current_int=100,
        speed_float=500.0,
        frame_ctx=FrameContext(prev_direction_deg=0.0),
    )

    # RAMP 过期，回 NORMAL
    t[0] = 0.30  # > ramp_cooldown_ms 200ms
    p.apply(
        direction_deg=35.0,
        current_int=100,
        speed_float=500.0,
        frame_ctx=FrameContext(prev_direction_deg=35.0),
    )

    # 触发 2：升级 HARD_COOLDOWN
    t[0] = 0.31
    p.apply(
        direction_deg=80.0,
        current_int=100,
        speed_float=500.0,
        frame_ctx=FrameContext(prev_direction_deg=35.0),
    )

    # HARD_COOLDOWN 期间：IMU 转，current 必须锁定
    t[0] = 0.50
    d, c, s, status = p.apply(
        direction_deg=85.0,
        current_int=130,  # IMU 已转 30°
        speed_float=500.0,
        frame_ctx=FrameContext(prev_direction_deg=80.0),
    )
    assert status.startswith('HARD_COOLDOWN'), f'状态错: {status}'
    assert d == 100.0, f'direction 应锁定 100，实际 {d}'
    assert c == 100, f'current 必须锁定 100（PID error=0），实际 {c}'
    assert s == 0.0, f'HARD_COOLDOWN speed 应为 0，实际 {s}'


def test_t5_clear_goal_immediate_hard_cooldown_locks_current():
    """T5 CLEAR_GOAL 触发的 HARD_COOLDOWN 也必须锁定 current。"""
    from rtk_perception.nav2_protector import FrameContext

    p, t = _make_protector()
    # 触发 T5
    ctx = FrameContext(prev_direction_deg=0.0, clear_goal_age_sec=0.1)
    d, c, s, status = p.apply(
        direction_deg=10.0, current_int=200, speed_float=500.0, frame_ctx=ctx
    )
    assert status == 'HARD_COOLDOWN_CLEAR_GOAL'
    assert d == 200.0
    assert c == 200, f'触发瞬间 current_at_trigger=200，current_out 必须 200，实际 {c}'
    assert s == 0.0

    # 下一帧 IMU 转，current 必须仍锁
    t[0] = 0.01
    ctx2 = FrameContext(prev_direction_deg=10.0, clear_goal_age_sec=0.11)
    d, c, s, status = p.apply(
        direction_deg=15.0, current_int=210, speed_float=500.0, frame_ctx=ctx2
    )
    assert status == 'HARD_COOLDOWN_CLEAR_GOAL'
    assert d == 200.0
    assert c == 200, f'HARD_COOLDOWN 期间 current 必须锁 200，实际 {c}'


def test_escalation_path_locks_current():
    """escalation 触发的 HARD_COOLDOWN 也必须锁定 current。"""
    from rtk_perception.nav2_protector import FrameContext, Nav2ProtectorParams

    params = Nav2ProtectorParams(cooldown_escalation_count=2)
    p, t = _make_protector(params=params)

    # 第 0 帧：NORMAL baseline
    p.apply(
        direction_deg=0.0,
        current_int=50,
        speed_float=300.0,
        frame_ctx=FrameContext(prev_direction_deg=0.0),
    )
    # 触发 1：RAMP_COOLDOWN
    t[0] = 0.01
    p.apply(
        direction_deg=40.0,
        current_int=50,
        speed_float=300.0,
        frame_ctx=FrameContext(prev_direction_deg=0.0),
    )

    # RAMP 到期，回 NORMAL
    t[0] = 0.22
    p.apply(
        direction_deg=40.0,
        current_int=50,
        speed_float=300.0,
        frame_ctx=FrameContext(prev_direction_deg=40.0),
    )

    # 触发 2 → escalation HARD_COOLDOWN（trigger_history 在 1s 窗口内有 2 次）
    t[0] = 0.23
    d, c, s, status = p.apply(
        direction_deg=80.0,
        current_int=50,
        speed_float=300.0,
        frame_ctx=FrameContext(prev_direction_deg=40.0),
    )
    assert status == 'HARD_COOLDOWN_ESCALATION'
    assert d == 50.0
    assert c == 50, f'escalation 触发瞬间 current 必须 50，实际 {c}'
    assert s == 0.0

    # 后续帧：IMU 转，current 必须仍锁
    t[0] = 0.30
    d, c, s, status = p.apply(
        direction_deg=85.0,
        current_int=70,  # IMU 转 20°
        speed_float=300.0,
        frame_ctx=FrameContext(prev_direction_deg=80.0),
    )
    # 后续帧走第 2 步 HARD_COOLDOWN return，status 用 last_trigger_name 后缀
    assert status == 'HARD_COOLDOWN_LOOKAHEAD'
    assert d == 50.0
    assert c == 50, f'HARD_COOLDOWN 期间 current 必须 50，实际 {c}'


def test_ramp_cooldown_speed_decreases_over_time():
    """RAMP_COOLDOWN 期间 speed 按 ramp_cooldown_ms 线性减到 0。"""
    from rtk_perception.nav2_protector import FrameContext, Nav2ProtectorParams

    params = Nav2ProtectorParams(ramp_cooldown_ms=200)
    p, t = _make_protector(params=params)

    # 触发 RAMP
    p.apply(
        direction_deg=0.0,
        current_int=0,
        speed_float=400.0,
        frame_ctx=FrameContext(prev_direction_deg=0.0),
    )
    t[0] = 0.01
    p.apply(
        direction_deg=40.0,
        current_int=0,
        speed_float=400.0,
        frame_ctx=FrameContext(prev_direction_deg=0.0),
    )

    # 50ms 后（200ms ramp 的 1/4）→ speed 约 300
    t[0] = 0.06
    _, _, s_mid, _ = p.apply(
        direction_deg=40.0,
        current_int=0,
        speed_float=400.0,
        frame_ctx=FrameContext(prev_direction_deg=40.0),
    )
    assert 250 < s_mid < 350, f'ramp 中段速度应 ~300，实际 {s_mid}'

    # 200ms 后 → speed=0 且状态回 NORMAL
    t[0] = 0.22
    d, c, s, status = p.apply(
        direction_deg=40.0,
        current_int=0,
        speed_float=400.0,
        frame_ctx=FrameContext(prev_direction_deg=40.0),
    )
    assert status == 'NORMAL'
    assert s == 400.0  # NORMAL 状态原样返回


def test_t2_cmd_vel_step_triggers_cooldown():
    """cmd_vel 阶跃 > 0.3 m/s 或 > 0.5 rad/s 触发 RAMP_COOLDOWN_CMD_VEL_STEP。"""
    from rtk_perception.nav2_protector import FrameContext

    p, t = _make_protector()
    p.apply(
        direction_deg=0.0,
        current_int=0,
        speed_float=400.0,
        frame_ctx=FrameContext(prev_direction_deg=0.0, cmd_vel_linear_x=0.0),
    )
    t[0] = 0.01
    d, c, s, status = p.apply(
        direction_deg=0.0,
        current_int=0,
        speed_float=400.0,
        frame_ctx=FrameContext(
            prev_direction_deg=0.0,
            cmd_vel_linear_x=0.5,  # Δ=0.5 > 0.3
            prev_cmd_vel_linear_x=0.0,
        ),
    )
    assert status == 'RAMP_COOLDOWN_CMD_VEL_STEP'


def test_t4_gear_shift_triggers_cooldown():
    """档位切换触发 RAMP_COOLDOWN_GEAR_SHIFT。"""
    from rtk_perception.nav2_protector import FrameContext

    p, t = _make_protector()
    p.apply(
        direction_deg=0.0,
        current_int=0,
        speed_float=400.0,
        frame_ctx=FrameContext(prev_direction_deg=0.0, nav_speed_mode=1, prev_nav_speed_mode=1),
    )
    t[0] = 0.01
    d, c, s, status = p.apply(
        direction_deg=0.0,
        current_int=0,
        speed_float=400.0,
        frame_ctx=FrameContext(
            prev_direction_deg=0.0,
            nav_speed_mode=2,  # 档位切换
            prev_nav_speed_mode=1,
        ),
    )
    assert status == 'RAMP_COOLDOWN_GEAR_SHIFT'


def test_t1_threshold_uses_circular_delta():
    """T1 用圆周差：355 → 5 跳变应判 10° 而非 350°。"""
    from rtk_perception.nav2_protector import FrameContext

    p, t = _make_protector()
    p.apply(
        direction_deg=355.0,
        current_int=0,
        speed_float=400.0,
        frame_ctx=FrameContext(prev_direction_deg=0.0),
    )
    t[0] = 0.01
    # 355 → 5 圆周差 10°，不触发
    d, c, s, status = p.apply(
        direction_deg=5.0,
        current_int=0,
        speed_float=400.0,
        frame_ctx=FrameContext(prev_direction_deg=355.0),
    )
    assert status == 'NORMAL'
