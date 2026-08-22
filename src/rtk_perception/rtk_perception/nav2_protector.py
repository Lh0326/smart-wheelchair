"""自主导航专属 PID=0 保护层。

针对自主导航场景下 ChassisSlewLimiter 触发判据未覆盖的 5 类
direction_angle / cmd_vel 跳变，提供独立的 COOLDOWN 状态机，
让下位机 PID error = 0（direction = current）从而避免 USB hub 过流掉电。

设计见自主导航功耗保护设计文档(2026-07-09)。
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, List, Optional, Tuple


@dataclass
class Nav2ProtectorParams:
    """Protector 的可调参数（通过 ROS 参数注入）。"""
    lookahead_jump_threshold_deg: float = 30.0
    cmd_vel_step_linear: float = 0.3
    cmd_vel_step_angular: float = 0.5
    ramp_cooldown_ms: int = 200
    hard_cooldown_ms: int = 1000
    cooldown_escalation_count: int = 3
    cooldown_escalation_window_sec: float = 1.0
    clear_goal_window_sec: float = 0.5


@dataclass
class FrameContext:
    """单帧上下文，由 chassis_serial_node 收集后传入。

    所有 prev_* 字段在 chassis_serial_node 维护，不在 Protector 内部缓存
    （Protector 无状态依赖外部，便于 REPL 验证）。
    """
    cmd_vel_linear_x: float = 0.0
    cmd_vel_angular_z: float = 0.0
    prev_cmd_vel_linear_x: float = 0.0
    prev_cmd_vel_angular_z: float = 0.0
    nav_speed_mode: int = 1
    prev_nav_speed_mode: int = 1
    follow_path_active: bool = False
    follow_path_just_started: bool = False
    follow_path_just_ended: bool = False
    clear_goal_age_sec: float = float('inf')
    prev_direction_deg: float = 0.0


class Nav2AutonomyProtector:
    """自主导航 PID=0 保护层状态机。

    状态：NORMAL → RAMP_COOLDOWN(200ms) → HARD_COOLDOWN(1s)
    升级：1s 滑窗累计 ≥ cooldown_escalation_count 次触发 → HARD_COOLDOWN
    """

    STATE_NORMAL = 'NORMAL'
    STATE_RAMP_COOLDOWN = 'RAMP_COOLDOWN'
    STATE_HARD_COOLDOWN = 'HARD_COOLDOWN'

    def __init__(
        self,
        params: Nav2ProtectorParams,
        clock_fn: Optional[Callable[[], float]] = None,
        log_fn: Optional[Callable[[str, str], None]] = None,
    ):
        self._params = params
        self._state = self.STATE_NORMAL
        self._state_enter_time: float = 0.0
        self._current_at_trigger: int = 0
        self._trigger_history: List[float] = []
        self._last_trigger_name: str = ''
        self._clock_fn = clock_fn or (lambda: 0.0)
        self._log_fn = log_fn or (lambda level, msg: None)

    def apply(
        self,
        direction_deg: float,
        current_int: int,
        speed_float: float,
        frame_ctx: FrameContext,
    ) -> Tuple[float, int, float, str]:
        """对一帧三字段做保护处理。

        返回 (direction_out, current_out, speed_out, status_str)。
        status_str 取值见规格 § 5（NORMAL / RAMP_COOLDOWN_* / HARD_COOLDOWN_* / ...）。
        """
        now = self._clock_fn()

        # 1. 状态机：检查当前 cooldown 是否到期
        if self._state == self.STATE_RAMP_COOLDOWN:
            if now - self._state_enter_time >= self._params.ramp_cooldown_ms / 1000.0:
                self._enter_state(self.STATE_NORMAL, now)
        elif self._state == self.STATE_HARD_COOLDOWN:
            if now - self._state_enter_time >= self._params.hard_cooldown_ms / 1000.0:
                self._enter_state(self.STATE_NORMAL, now)
                self._trigger_history.clear()

        # 2. 若在 cooldown，输出保护值
        # 关键：direction 和 current 都锁定到 current_at_trigger，
        # 让下位机 PID error = direction - current = 0，避免 USB hub 过流掉电。
        # 复用 ChassisSlewLimiter § 3.2 修订时已验证的设计。
        if self._state == self.STATE_RAMP_COOLDOWN:
            return (
                float(self._current_at_trigger),
                self._current_at_trigger,
                self._ramp_down_speed(speed_float, now),
                f'RAMP_COOLDOWN_{self._last_trigger_name}',
            )
        if self._state == self.STATE_HARD_COOLDOWN:
            return (
                float(self._current_at_trigger),
                self._current_at_trigger,
                0.0,
                f'HARD_COOLDOWN_{self._last_trigger_name}',
            )

        # 3. NORMAL：检查触发器
        trigger = self._check_triggers(direction_deg, frame_ctx, now)
        if trigger is None:
            return (direction_deg, current_int, speed_float, 'NORMAL')

        # 命中触发器，进入 COOLDOWN
        self._current_at_trigger = current_int
        self._last_trigger_name = trigger
        self._trigger_history.append(now)
        window = self._params.cooldown_escalation_window_sec
        self._trigger_history = [
            t for t in self._trigger_history if now - t < window
        ]

        # T5 clear_goal 直接进 HARD_COOLDOWN
        if trigger == 'CLEAR_GOAL':
            self._enter_state(self.STATE_HARD_COOLDOWN, now)
            self._log_fn('WARN', f'Protector T5 CLEAR_GOAL → HARD_COOLDOWN')
            return (
                float(self._current_at_trigger),
                self._current_at_trigger,
                0.0,
                'HARD_COOLDOWN_CLEAR_GOAL',
            )

        # 1s 滑窗累计 ≥ escalation_count → 升级 HARD_COOLDOWN
        if len(self._trigger_history) >= self._params.cooldown_escalation_count:
            self._enter_state(self.STATE_HARD_COOLDOWN, now)
            self._trigger_history.clear()
            self._log_fn(
                'WARN',
                f'Protector 升级 HARD_COOLDOWN（{trigger} 触发第 '
                f'{self._params.cooldown_escalation_count} 次）',
            )
            return (
                float(self._current_at_trigger),
                self._current_at_trigger,
                0.0,
                'HARD_COOLDOWN_ESCALATION',
            )

        # 普通 RAMP_COOLDOWN
        self._enter_state(self.STATE_RAMP_COOLDOWN, now)
        self._log_fn('INFO', f'Protector T={trigger} → RAMP_COOLDOWN')
        return (
            float(self._current_at_trigger),
            self._current_at_trigger,
            self._ramp_down_speed(speed_float, now),
            f'RAMP_COOLDOWN_{trigger}',
        )

    def _enter_state(self, new_state: str, now: float) -> None:
        self._state = new_state
        self._state_enter_time = now

    def _ramp_down_speed(self, target_speed: float, now: float) -> float:
        """线性斜率减速：target → 0 跨越 ramp_cooldown_ms。"""
        elapsed = now - self._state_enter_time
        ramp_sec = self._params.ramp_cooldown_ms / 1000.0
        if ramp_sec <= 0:
            return 0.0
        factor = max(0.0, 1.0 - elapsed / ramp_sec)
        return target_speed * factor

    @staticmethod
    def _circular_delta(a: float, b: float) -> float:
        """计算 360° 圆周上的最短角度差，避免 ±180° 翻转误判。"""
        d = abs(a - b) % 360.0
        return min(d, 360.0 - d)

    def _check_triggers(
        self,
        direction_deg: float,
        frame_ctx: FrameContext,
        now: float,
    ) -> Optional[str]:
        """检查所有触发器，返回命中的触发器名（T1-T5），未命中返回 None。

        优先级：T5 (clear_goal) > T1 (lookahead) > T2 (cmd_vel step)
                > T3 (FollowPath event) > T4 (gear shift)
        T5 直接进 HARD_COOLDOWN（最高危时序窗口）。
        """
        # T5: clear_goal 时序窗口（最高危，先判）
        if frame_ctx.clear_goal_age_sec < self._params.clear_goal_window_sec:
            return 'CLEAR_GOAL'
        # T1: lookahead 方向跳变
        if self._detect_lookahead_jump(direction_deg, frame_ctx):
            return 'LOOKAHEAD'
        # T2: cmd_vel 阶跃
        if self._detect_cmd_vel_step(frame_ctx):
            return 'CMD_VEL_STEP'
        # T3: FollowPath 始末
        if frame_ctx.follow_path_just_started or frame_ctx.follow_path_just_ended:
            return 'FOLLOW_PATH'
        # T4: 档位切换
        if frame_ctx.nav_speed_mode != frame_ctx.prev_nav_speed_mode:
            return 'GEAR_SHIFT'
        return None

    def _detect_lookahead_jump(
        self, direction_deg: float, frame_ctx: FrameContext
    ) -> bool:
        delta = self._circular_delta(direction_deg, frame_ctx.prev_direction_deg)
        return delta > self._params.lookahead_jump_threshold_deg

    def _detect_cmd_vel_step(self, frame_ctx: FrameContext) -> bool:
        d_linear = abs(frame_ctx.cmd_vel_linear_x - frame_ctx.prev_cmd_vel_linear_x)
        d_angular = abs(frame_ctx.cmd_vel_angular_z - frame_ctx.prev_cmd_vel_angular_z)
        return (
            d_linear > self._params.cmd_vel_step_linear
            or d_angular > self._params.cmd_vel_step_angular
        )
