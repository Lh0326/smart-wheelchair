"""ControlStateMachine 3 状态机测试。

覆盖 spec § 5：
    - 初始 LOCKED
    - 瞌睡门控（neutral/relaxed → DISABLED）
    - 持续清醒解锁（DISABLED + focused 持续 2s → ACTIVE）
    - frown toggle（LOCKED ↔ ACTIVE）
    - 防误触：冷却 + grace period（输出超时已移除，spec § 2.2）
"""
from wheelchair_app.braincontrol.control_state_machine import ControlStateMachine
from wheelchair_app.braincontrol.control_types import ControlState, MotionCommand, TiltDirection


def test_initial_state_is_locked():
    """启动默认 LOCKED（spec § 5.3）。"""
    sm = ControlStateMachine()
    assert sm.state == ControlState.LOCKED


def test_drowsy_drops_to_disabled():
    """LOCKED + focus=='relaxed' → DISABLED。"""
    sm = ControlStateMachine()
    cmd = sm.update(focus_state='relaxed', toggle_event=False,
                    tilt=TiltDirection.NONE, dt_ms=100)
    assert sm.state == ControlState.DISABLED
    assert cmd == MotionCommand.STOP


def test_neutral_treated_as_drowsy():
    """neutral 保守视为瞌睡，从 LOCKED 切 DISABLED。"""
    sm = ControlStateMachine()
    sm.update(focus_state='neutral', toggle_event=False,
              tilt=TiltDirection.NONE, dt_ms=100)
    assert sm.state == ControlState.DISABLED


def test_sustained_focus_unlocks_to_active():
    """DISABLED 状态持续 2s focused → 直接 ACTIVE。"""
    sm = ControlStateMachine()
    # 先进 DISABLED
    sm.update(focus_state='relaxed', toggle_event=False,
              tilt=TiltDirection.NONE, dt_ms=100)
    assert sm.state == ControlState.DISABLED
    # < 2s 不解锁
    sm.update(focus_state='focused', toggle_event=False,
              tilt=TiltDirection.NONE, dt_ms=1900)
    assert sm.state == ControlState.DISABLED
    # >= 2s 解锁到 ACTIVE
    sm.update(focus_state='focused', toggle_event=False,
              tilt=TiltDirection.NONE, dt_ms=200)
    assert sm.state == ControlState.ACTIVE


def test_locked_state_outputs_stop_even_with_tilt():
    """LOCKED 状态下头部动作不输出。"""
    sm = ControlStateMachine()
    assert sm.state == ControlState.LOCKED
    cmd = sm.update(focus_state='focused', toggle_event=False,
                    tilt=TiltDirection.FORWARD, dt_ms=100)
    assert cmd == MotionCommand.STOP
    assert sm.state == ControlState.LOCKED


def test_frown_unlocks_to_active():
    """LOCKED + frown → ACTIVE。"""
    sm = ControlStateMachine()
    cmd = sm.update(focus_state='focused', toggle_event=True,
                    tilt=TiltDirection.FORWARD, dt_ms=100)
    assert sm.state == ControlState.ACTIVE
    assert cmd == MotionCommand.FORWARD


def test_frown_locks_back_from_active():
    """ACTIVE + frown（冷却后）→ LOCKED。"""
    sm = ControlStateMachine()
    sm.update(focus_state='focused', toggle_event=True,
              tilt=TiltDirection.NONE, dt_ms=100)
    assert sm.state == ControlState.ACTIVE
    # 冷却期过去
    for _ in range(20):
        sm.update(focus_state='focused', toggle_event=False,
                  tilt=TiltDirection.NONE, dt_ms=100)
    # 再次 frown → LOCKED
    sm.update(focus_state='focused', toggle_event=True,
              tilt=TiltDirection.NONE, dt_ms=100)
    assert sm.state == ControlState.LOCKED


def test_frown_cooldown_blocks_rapid_toggle():
    """切换后 1.5s 内的 frown 不响应。"""
    sm = ControlStateMachine()
    # LOCKED → ACTIVE
    sm.update(focus_state='focused', toggle_event=True,
              tilt=TiltDirection.NONE, dt_ms=100)
    assert sm.state == ControlState.ACTIVE
    # 1s 内再次 frown（冷却中）→ 不切换
    for _ in range(10):
        sm.update(focus_state='focused', toggle_event=True,
                  tilt=TiltDirection.NONE, dt_ms=100)
    assert sm.state == ControlState.ACTIVE


def test_focus_freeze_prevents_emg_pollution_lockout():
    """frown 触发后 1.5s 内 EMG 让 focus=neutral 也不切 DISABLED。

    抬头纹动作会污染 EEG，SVM 可能输出 neutral；grace period 防止误判。
    """
    sm = ControlStateMachine()
    sm.update(focus_state='focused', toggle_event=True,
              tilt=TiltDirection.NONE, dt_ms=100)
    assert sm.state == ControlState.ACTIVE
    # 立刻 EMG 污染让 SVM 输出 neutral
    for _ in range(10):  # 1000ms < 1500ms grace
        sm.update(focus_state='neutral', toggle_event=False,
                  tilt=TiltDirection.NONE, dt_ms=100)
    assert sm.state == ControlState.ACTIVE  # 仍在 ACTIVE


def test_no_auto_lock_after_long_output():
    """spec § 2.2：删除 MAX_CONTINUOUS_OUTPUT_MS 后，60s 持续输出仍 ACTIVE。

    之前 30s 会自动切 LOCKED，现在不再自动锁定，仅咬牙 toggle 切换。
    """
    sm = ControlStateMachine()
    sm.update(focus_state='focused', toggle_event=True,
              tilt=TiltDirection.NONE, dt_ms=100)  # → ACTIVE
    # 持续 FORWARD 60s（6000 帧 × 10ms）
    for i in range(6000):
        cmd = sm.update(focus_state='focused', toggle_event=False,
                        tilt=TiltDirection.FORWARD, dt_ms=100)
        assert cmd == MotionCommand.FORWARD
        assert sm.state == ControlState.ACTIVE, (
            f"60s 内不应自动锁定，但第 {i} 帧后状态变 {sm.state}"
        )


def test_active_to_locked_only_via_clench():
    """spec § 2.2：ACTIVE → LOCKED 只能通过咬牙 toggle。"""
    sm = ControlStateMachine()
    sm.update(focus_state='focused', toggle_event=True,
              tilt=TiltDirection.NONE, dt_ms=100)  # → ACTIVE
    assert sm.state == ControlState.ACTIVE
    # 咬牙 toggle 切到 LOCKED
    sm.update(focus_state='focused', toggle_event=True,
              tilt=TiltDirection.NONE, dt_ms=2000)  # 等冷却过
    assert sm.state == ControlState.LOCKED


def test_drowsy_safety_preserved():
    """spec § 2.2：瞌睡安全机制保留（删自动锁定不影响）。"""
    sm = ControlStateMachine()
    sm.update(focus_state='focused', toggle_event=True,
              tilt=TiltDirection.NONE, dt_ms=100)  # → ACTIVE
    assert sm.state == ControlState.ACTIVE
    # 等过 FOCUS_FREEZE_MS（咬牙 toggle 后 1.5s EEG 冻结）
    for _ in range(20):
        sm.update(focus_state='focused', toggle_event=False,
                  tilt=TiltDirection.NONE, dt_ms=100)
    # 瞌睡信号 → DISABLED
    cmd = sm.update(focus_state='relaxed', toggle_event=False,
                    tilt=TiltDirection.FORWARD, dt_ms=100)
    assert sm.state == ControlState.DISABLED
    assert cmd == MotionCommand.STOP
