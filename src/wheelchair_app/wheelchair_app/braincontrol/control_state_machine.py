"""脑控轮椅 3 状态机：DISABLED / LOCKED / ACTIVE。

状态转换（spec § 5.2）：
    初始：LOCKED
    任意 + focus in {neutral, relaxed} → DISABLED（安全第一）
    DISABLED + focused 持续 2s → ACTIVE（直接跳过 LOCKED）
    LOCKED + toggle（咬牙 rising edge）→ ACTIVE
    ACTIVE + toggle（咬牙 rising edge）→ LOCKED
"""
from .control_types import ControlState, MotionCommand, TiltDirection

# 防误触参数（spec § 5.3）
FOCUS_HOLD_MS = 2000         # DISABLED → ACTIVE 持续清醒
FROWN_COOLDOWN_MS = 1500     # toggle 冷却（原 frown 冷却，保留命名兼容）
FOCUS_FREEZE_MS = 1500       # toggle 后 EEG 冻结（咬牙动作会污染 EEG）

# focus_state → 门控语义（spec § 4.2）
DROWSY_LIKE = {'neutral', 'relaxed'}  # 保守视为瞌睡


class ControlStateMachine:
    """3 状态机：DISABLED / LOCKED / ACTIVE。

    状态转换规则：
        任意 + drowsy → DISABLED（安全第一）
        DISABLED + focused 持续 2s → ACTIVE（spec § 5.2 直接跳过 LOCKED）
        LOCKED + toggle（咬牙 rising edge）→ ACTIVE
        ACTIVE + toggle（咬牙 rising edge）→ LOCKED

    防误触机制：
        - FROWN_COOLDOWN_MS：toggle（咬牙）切换后冷却，防抖
          （命名保留 FROWN_COOLDOWN_MS 以兼容现有调用方，语义即 toggle 冷却）
        - FOCUS_FREEZE_MS：toggle 后冻结 EEG 更新（咬牙动作会污染 EEG）

    注：MAX_CONTINUOUS_OUTPUT_MS（30s 自动锁定）已于 2026-06-30 删除（spec § 2.2），
        仅保留咬牙 toggle 切换 LOCKED↔ACTIVE。
    """

    def __init__(self):
        self._state = ControlState.LOCKED  # spec § 5.3：启动默认 LOCKED
        self._focus_hold_ms = 0
        self._frown_cooldown_ms = 0  # toggle 冷却计时器（保留命名兼容）
        self._focus_freeze_ms = 0
        self._last_effective_focus = 'focused'

    @property
    def state(self) -> ControlState:
        return self._state

    def update(self, focus_state: str, toggle_event: bool,
               tilt: TiltDirection, dt_ms: int) -> MotionCommand:
        """每帧调用，返回当前应输出的 MotionCommand。

        Args:
            focus_state: 'focused' / 'neutral' / 'relaxed'
            toggle_event: 咬牙 ClenchDetector 的 rising edge event
                          （LOCKED ↔ ACTIVE toggle 触发源）
            tilt: ImuHandler 输出的 TiltDirection
            dt_ms: 距上一帧的时间间隔（毫秒）
        """
        # 1. EEG grace period：toggle 后 1.5s 冻结 focus 更新
        if self._focus_freeze_ms > 0:
            self._focus_freeze_ms -= dt_ms
            effective_focus = self._last_effective_focus
        else:
            effective_focus = focus_state
            self._last_effective_focus = focus_state

        # 2. toggle 冷却倒计时
        if self._frown_cooldown_ms > 0:
            self._frown_cooldown_ms -= dt_ms

        # 3. 瞌睡 → DISABLED（安全第一）
        if effective_focus in DROWSY_LIKE:
            self._transition(ControlState.DISABLED)
            self._focus_hold_ms = 0
            return MotionCommand.STOP

        # 4. 持续清醒：DISABLED → ACTIVE（spec § 5.2 直接跳到 ACTIVE）
        if self._state == ControlState.DISABLED:
            self._focus_hold_ms += dt_ms
            if self._focus_hold_ms >= FOCUS_HOLD_MS:
                self._transition(ControlState.ACTIVE)
            else:
                return MotionCommand.STOP

        # 5. toggle（咬牙 rising edge）切换（带冷却）
        if toggle_event and self._frown_cooldown_ms <= 0:
            if self._state == ControlState.LOCKED:
                self._transition(ControlState.ACTIVE)
                self._frown_cooldown_ms = FROWN_COOLDOWN_MS
                self._focus_freeze_ms = FOCUS_FREEZE_MS
            elif self._state == ControlState.ACTIVE:
                self._transition(ControlState.LOCKED)
                self._frown_cooldown_ms = FROWN_COOLDOWN_MS
                self._focus_freeze_ms = FOCUS_FREEZE_MS

        # 6. 输出（spec § 2.2：删除 MAX_CONTINUOUS_OUTPUT_MS 自动锁定）
        if self._state == ControlState.ACTIVE:
            return self._tilt_to_command(tilt)
        return MotionCommand.STOP

    def _transition(self, new_state: ControlState) -> None:
        if self._state != new_state:
            self._state = new_state

    def _tilt_to_command(self, tilt: TiltDirection) -> MotionCommand:
        return {
            TiltDirection.NONE: MotionCommand.STOP,
            TiltDirection.FORWARD: MotionCommand.FORWARD,
            TiltDirection.BACKWARD: MotionCommand.BACKWARD,
            TiltDirection.LEFT: MotionCommand.LEFT,
            TiltDirection.RIGHT: MotionCommand.RIGHT,
        }[tilt]
