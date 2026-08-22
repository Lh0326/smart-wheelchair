"""实物底盘串口节点。

订阅 /cmd_vel_safe + /cmd_vel_eeg + /heading_imu，按 100Hz 通过串口发送三字段协议给下位机：
    \\ndirection_angle,current_angle,forward_speed

HWT906P 的 /heading_imu 只作为 current_angle 反馈；脑控模式的动作意图
来自 /cmd_vel_eeg。下位机需要 current_angle 才能正确做差速/方向闭环。
下位机内部 PID 比较前两个字段驱动电机，失联时自带 watchdog 停车。

仿真模式（use_real_chassis=false）下不启动此节点，由 sim_chassis_node 替代。
"""
from __future__ import annotations

import math
import signal

import rclpy
from geometry_msgs.msg import Pose, PoseArray, TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, Float64, Int8, String
from tf2_ros import TransformBroadcaster

from action_msgs.msg import GoalStatusArray
from rtk_perception.nav2_protector import (
    FrameContext,
    Nav2AutonomyProtector,
    Nav2ProtectorParams,
)


def wrap_deg(deg: float) -> int:
    """归一化浮点角度到 [-180, 180] 整数（四舍五入）。

    注意：±180 输入会归一化为 -180（同一方向，对底盘 PID 等价）。
    """
    deg = ((deg + 180.0) % 360.0) - 180.0
    return int(round(deg))


def wrap_deg_float(deg: float) -> float:
    """归一化浮点角度到 [-180, 180)（不取整，用于方向跳变检测）。

    与既有 wrap_deg（int 版本）配套：wrap_deg 用于协议字段最终编码，
    wrap_deg_float 用于跳变检测的中间计算，避免取整误差导致
    175° → -175° 被误判为 350° 跳变。
    """
    return ((deg + 180.0) % 360.0) - 180.0


def slew_speed(target: float, current: int, ramp_per_sec: int, dt: float) -> int:
    """限制 forward_speed 每周期最大变化量。

    NaN/Inf target 视为 0（与 compute_forward_speed 一致）。
    dt clamp 到 [0, 0.2] 防单帧暴冲（与 _publish_odom_tf 既有风格一致）。
    """
    if math.isnan(target) or math.isinf(target):
        target = 0.0
    if dt > 0.2:
        dt = 0.2
    elif dt < 0.0:
        dt = 0.0
    delta = target - current
    max_delta = ramp_per_sec * dt
    if delta > max_delta:
        return int(current + max_delta)
    if delta < -max_delta:
        return int(current - max_delta)
    return int(target)


def compute_forward_speed(linear_x_mps: float, max_speed_mps: float) -> int:
    """线速度 m/s → 协议量纲 [-1000, 1000]。

    NaN 输入返回 0（防 TEB 偶发 NaN）。
    """
    if math.isnan(linear_x_mps) or math.isinf(linear_x_mps):
        return 0
    raw = round(linear_x_mps / max_speed_mps * 1000.0)
    return max(-1000, min(1000, int(raw)))


def compute_nav_forward_speed(
    linear_x_mps: float,
    omega_rad_s: float,
    max_speed_mps: float,
    min_forward_speed: int,
    min_turn_speed: int,
    linear_deadband_mps: float,
    angular_deadband_rad_s: float,
    floor_linear_threshold_mps: float,
    allow_reverse: bool = False,
) -> int:
    """自主导航 Twist → 协议 forward_speed。

    真实底盘存在电机死区，TEB 的小线速度可能在虚拟底盘有效、到实物却不动。
    同时 TEB 可能输出低线速/原地转向，三字段协议通常需要一个很小的
    forward_speed 才能让下位机按 direction/current 角差做差速修正。
    """
    if math.isnan(linear_x_mps) or math.isinf(linear_x_mps):
        linear_x_mps = 0.0
    if math.isnan(omega_rad_s) or math.isinf(omega_rad_s):
        omega_rad_s = 0.0
    if not allow_reverse and linear_x_mps < 0.0:
        linear_x_mps = 0.0

    min_forward_speed = max(0, min(1000, int(min_forward_speed)))
    min_turn_speed = max(0, min(1000, int(min_turn_speed)))
    linear_deadband_mps = max(0.0, float(linear_deadband_mps))
    angular_deadband_rad_s = max(0.0, float(angular_deadband_rad_s))
    floor_linear_threshold_mps = max(linear_deadband_mps, float(floor_linear_threshold_mps))

    if abs(linear_x_mps) < linear_deadband_mps:
        if abs(omega_rad_s) >= angular_deadband_rad_s:
            return min_turn_speed
        return 0

    speed = compute_forward_speed(linear_x_mps, max_speed_mps)
    if (
        min_forward_speed > 0
        and abs(linear_x_mps) >= floor_linear_threshold_mps
        and 0 < abs(speed) < min_forward_speed
    ):
        return min_forward_speed if speed > 0 else -min_forward_speed
    return speed


def compute_direction_angle(
    current_angle: int,
    omega_rad_s: float,
    gain_deg_per_rad_per_sec: float,
    max_lead_deg: float,
    heading_sign: int,
) -> int:
    """当前航向 + 角速度前馈 → 目标航向（绝对，度）。

    heading_sign: +1 或 -1，用于消除 ROS Twist angular.z (REP-103 逆时针正)
    与 /heading_imu (罗盘顺时针正) 的符号差异。默认 -1。
    """
    if math.isnan(omega_rad_s) or math.isinf(omega_rad_s):
        omega_rad_s = 0.0
    omega_compass = omega_rad_s * heading_sign
    lead = omega_compass * gain_deg_per_rad_per_sec
    lead = max(-max_lead_deg, min(max_lead_deg, lead))
    return wrap_deg(current_angle + lead)


class NavHeadingHold:
    """自主导航直行绝对航向保持与机械偏置补偿。"""

    def __init__(self, enabled=True, steering_trim_deg=0.0,
                 enter_error_deg=5.0, exit_error_deg=10.0,
                 enter_omega_rad_s=0.05, exit_omega_rad_s=0.12):
        self.enabled = bool(enabled)
        self.steering_trim_deg = float(steering_trim_deg)
        self.enter_error_deg = max(0.0, float(enter_error_deg))
        self.exit_error_deg = max(self.enter_error_deg, float(exit_error_deg))
        self.enter_omega_rad_s = max(0.0, float(enter_omega_rad_s))
        self.exit_omega_rad_s = max(self.enter_omega_rad_s, float(exit_omega_rad_s))
        self._held_direction_deg = None

    def reset(self):
        self._held_direction_deg = None

    def apply(self, planned_direction_deg, current_heading_deg, linear_x_mps,
              omega_rad_s, moving_deadband_mps):
        if not self.enabled or abs(linear_x_mps) < moving_deadband_mps:
            self.reset()
            return float(planned_direction_deg)
        planned_error = abs(wrap_deg_float(planned_direction_deg - current_heading_deg))
        omega_abs = abs(float(omega_rad_s))
        if self._held_direction_deg is not None:
            if planned_error >= self.exit_error_deg or omega_abs >= self.exit_omega_rad_s:
                self.reset()
            else:
                return wrap_deg_float(self._held_direction_deg)
        trimmed = wrap_deg_float(planned_direction_deg + self.steering_trim_deg)
        if planned_error <= self.enter_error_deg and omega_abs <= self.enter_omega_rad_s:
            self._held_direction_deg = trimmed
        return trimmed


def compute_roll_direction_offset_deg(
    roll_deg: float,
    enabled: bool,
    gain: float,
    saturation: float,
    polarity: int,
) -> int:
    """头部 roll 度数 → direction_angle 偏移量（整数度，带饱和）。

    EEG FORWARD/BACKWARD 帧期间，把 ESP32+IMU 头部 roll 信号转成
    下位机 direction_angle 字段的偏移量。LEFT/RIGHT 帧不调用本函数
    （roll 已在判定阶段用过）。

    NaN/Inf 输入返回 0（防 IMU 数据异常，与 compute_forward_speed 风格一致）。
    saturation <= 0 时返回 0（兜底，正常配置下由参数校验拦截）。
    """
    if not enabled:
        return 0
    if math.isnan(roll_deg) or math.isinf(roll_deg):
        return 0
    sat = max(0.0, float(saturation))
    raw = roll_deg * gain * polarity
    return int(max(-sat, min(sat, int(round(raw)))))


def compute_eeg_directional_fields(
    linear_x_mps: float,
    omega_rad_s: float,
    current_angle: int,
    forward_frame: tuple[int, int, int],
    backward_frame: tuple[int, int, int],
    left_frame: tuple[int, int, int],
    right_frame: tuple[int, int, int],
    roll_offset_deg: float = 0.0,
) -> tuple[int, int, int]:
    """脑控 Twist → 下位机三字段。

    语义（2026-07-12 修订）：标定帧的第 0 字段（direction_angle）解释为
    **相对 current_angle 的偏移**，不再是绝对方向。函数内执行
    `direction = wrap_deg(current_angle + frame_offset [+ roll_offset])`。

    修订原因：下位机新增加速度限制后，旧实现（current 固定 0）让下位机
    误判为"无运动反馈"，触发保护拒绝执行 forward_speed。借鉴 Nav2 路径
    （current 跟随 HWT 真值、direction = current + 偏移、下位机 PID error
    仍代表动作意图）后，三字段结构与 Nav2 模式同构，加速度限制通过。

    调用方必须传入 HWT 真值作为 current_angle（不再传固定 0）。

    roll_offset_deg：FORWARD/BACKWARD 帧的方向微调偏移（度），
    由 compute_roll_direction_offset_deg 计算；LEFT/RIGHT 帧不消费。
    """
    if math.isnan(linear_x_mps) or math.isinf(linear_x_mps):
        linear_x_mps = 0.0
    if math.isnan(omega_rad_s) or math.isinf(omega_rad_s):
        omega_rad_s = 0.0

    if abs(linear_x_mps) >= 0.001:
        offset, _current_in_frame, speed = (
            forward_frame if linear_x_mps > 0 else backward_frame
        )
        direction = wrap_deg(int(current_angle) + offset + roll_offset_deg)
        return (direction, int(current_angle), speed)

    if abs(omega_rad_s) >= 0.001:
        offset, _current_in_frame, speed = (
            left_frame if omega_rad_s > 0 else right_frame
        )
        direction = wrap_deg(int(current_angle) + offset)
        return (direction, int(current_angle), speed)

    return int(current_angle), int(current_angle), 0


class ChassisSlewLimiter:
    """底盘指令限速器（防主机死机）。

    三机制：
      1. forward_speed 斜率限制（slew_speed）
      2. 方向显著跳变保护（COOLDOWN 状态）
      3. 高频切换兜底（LONG_COOLDOWN 状态）

    状态机：
        NORMAL → 检测 jump ≥ threshold → COOLDOWN
        COOLDOWN → 时间到 → NORMAL
        COOLDOWN 触发时若窗口内 jump 计数 ≥ count_threshold → LONG_COOLDOWN
        LONG_COOLDOWN → 时间到 → NORMAL（清空计数）

    纯 Python 类，无 rclpy 依赖（便于单元测试）。
    通过 _tick 100Hz 调用 .apply() 喂入目标三字段，返回限速后的实际发送三字段。
    """

    def __init__(
        self,
        ramp_per_sec: int,
        jump_threshold_deg: float,
        cooldown_sec: float,
        window_sec: float,
        count_threshold: int,
        long_cooldown_sec: float,
    ):
        # 参数校验（fail-fast）
        if not isinstance(ramp_per_sec, int) or ramp_per_sec <= 0 or ramp_per_sec > 10000:
            raise ValueError(
                f"ramp_per_sec 必须是 int 且 ∈ (0, 10000]，当前 {ramp_per_sec}"
            )
        if jump_threshold_deg <= 0 or jump_threshold_deg > 180:
            raise ValueError(
                f"jump_threshold_deg 必须 ∈ (0, 180]，当前 {jump_threshold_deg}"
            )
        if cooldown_sec < 0:
            raise ValueError(f"cooldown_sec 必须 ≥ 0，当前 {cooldown_sec}")
        if window_sec <= 0:
            raise ValueError(f"window_sec 必须 > 0，当前 {window_sec}")
        if not isinstance(count_threshold, int) or count_threshold < 1:
            raise ValueError(
                f"count_threshold 必须是 int 且 ≥ 1，当前 {count_threshold}"
            )
        if long_cooldown_sec < 0:
            raise ValueError(
                f"long_cooldown_sec 必须 ≥ 0，当前 {long_cooldown_sec}"
            )

        # 参数快照
        self._ramp_per_sec = ramp_per_sec
        self._jump_threshold_deg = jump_threshold_deg
        self._cooldown_sec = cooldown_sec
        self._window_sec = window_sec
        self._count_threshold = count_threshold
        self._long_cooldown_sec = long_cooldown_sec

        # 内部状态
        self._state = 'NORMAL'  # 'NORMAL' / 'COOLDOWN' / 'LONG_COOLDOWN'
        self._current_speed = 0  # 上次实际输出的 forward_speed（int）
        self._last_direction: float | None = None
        self._cooldown_until = 0.0
        self._direction_at_trigger: float | None = None
        # 触发冷却时的 current_angle 快照（spec § 3.2 修订）：
        # COOLDOWN 期间输出 direction=current_at_trigger，让下位机 PID
        # 看到 error=0，避免 speed=0 时 PID 仍驱动两轮做反向差速（实测
        # 限速器首版未消除电源冲击的根因）。
        self._current_at_trigger: int = 0
        self._jump_timestamps: list[float] = []
        self._enabled = True

    def set_enabled(self, enabled: bool) -> None:
        """总开关。False 时 apply() 直通 target，但保留内部状态。"""
        self._enabled = bool(enabled)

    def apply(
        self,
        direction_target: float,
        current_target: int,
        speed_target: int,
        dt: float,
        now: float,
    ) -> tuple[int, int, int]:
        """限速主入口。返回 (direction_out, current_out, speed_out)。

        direction_target: 度（float）
        current_target: 度（int）
        speed_target: 协议单位 [-1000, 1000]（int）
        dt: 距上一帧的秒数（自动 clamp [0, 0.2]）
        now: 当前时间戳（秒，用于冷却计时与滑动窗口）
        """
        if not self._enabled:
            # 直通：仍然更新 _current_speed / _last_direction 状态，
            # 避免重新启用时大跳
            self._current_speed = int(speed_target)
            self._last_direction = float(direction_target)
            return (
                int(round(direction_target)),
                int(current_target),
                int(speed_target),
            )

        # 状态机推进：检查是否该退出冷却
        # (浮点 epsilon 防精度问题：0.01 + 0.2 = 0.21000000000000002 > 0.21)
        if self._state in ('COOLDOWN', 'LONG_COOLDOWN'):
            if now >= self._cooldown_until - 1e-9:
                if self._state == 'LONG_COOLDOWN':
                    # LONG_COOLDOWN 结束清空跳变历史（避免立刻再次触发）
                    self._jump_timestamps = []
                self._state = 'NORMAL'
                self._direction_at_trigger = None
                # COOLDOWN 结束的瞬间接受用户最新意图（避免立即重新触发 COOLDOWN
                # 导致死循环：旧 last_direction（=-90）vs 新 target（=90）总差 180°）
                self._last_direction = float(direction_target)

        # NORMAL 状态：检测方向跳变
        if self._state == 'NORMAL':
            should_jump = False
            if self._last_direction is not None:
                diff = wrap_deg_float(direction_target - self._last_direction)
                if abs(diff) >= self._jump_threshold_deg:
                    should_jump = True

            if should_jump:
                self._direction_at_trigger = self._last_direction
                # 记录触发时的 current_angle（COOLDOWN 输出用，spec § 3.2 修订）
                self._current_at_trigger = int(current_target)
                if self._record_jump_and_check_long(now):
                    # 升级为 LONG_COOLDOWN
                    self._state = 'LONG_COOLDOWN'
                    self._cooldown_until = now + self._long_cooldown_sec
                else:
                    self._state = 'COOLDOWN'
                    self._cooldown_until = now + self._cooldown_sec

        # 计算输出
        if self._state in ('COOLDOWN', 'LONG_COOLDOWN'):
            # 关键：输出 direction = current_at_trigger，让下位机 PID
            # error = direction - current = 0，避免 speed=0 时 PID 仍驱动
            # 两轮做反向差速（实测电源冲击的根因）。
            # 代价：COOLDOWN 期间轮椅不转向，仅按当前 speed 直走减速。
            direction_out = float(self._current_at_trigger)
            current_out = int(self._current_at_trigger)
            target_for_slew = 0
        else:  # NORMAL
            direction_out = direction_target
            current_out = int(current_target)
            target_for_slew = speed_target
            self._last_direction = float(direction_target)

        speed_out = slew_speed(
            target=target_for_slew,
            current=self._current_speed,
            ramp_per_sec=self._ramp_per_sec,
            dt=dt,
        )
        self._current_speed = speed_out

        return (
            int(round(direction_out)),
            int(current_out),
            int(speed_out),
        )

    def _record_jump_and_check_long(self, now: float) -> bool:
        """记录跳变时间戳，返回是否应进入 LONG_COOLDOWN。"""
        self._jump_timestamps.append(now)
        self._jump_timestamps = [
            t for t in self._jump_timestamps if t > now - self._window_sec
        ]
        return len(self._jump_timestamps) >= self._count_threshold


def _pose_yaw_rad(pose: Pose) -> float | None:
    """从 Pose.orientation 四元数提取 yaw（弧度）。

    四元数全 0（无效）→ 返回 None。
    """
    qz = float(pose.orientation.z)
    qw = float(pose.orientation.w)
    qx = float(pose.orientation.x)
    qy = float(pose.orientation.y)
    norm_sq = qx * qx + qy * qy + qz * qz + qw * qw
    if norm_sq < 1e-9:
        return None
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )


def extract_lookahead_yaw(
    local_plan: PoseArray,
    lookahead_distance: float,
) -> float | None:
    """从 TEB /local_plan (base_link 系 PoseArray) 提取 lookahead 点的朝向。

    在路径上沿累积前进距离找到 lookahead_distance 处的位姿，返回其 yaw（弧度）。
    - 路径总长 < lookahead → 取末点
    - 路径为空 / 四元数无效 / lookahead<0 → 返回 None
    - 路径相邻点之间用线性插值

    spec § 2.1 / § 5.1。
    """
    if local_plan is None or lookahead_distance < 0:
        return None
    poses = list(local_plan.poses)
    if not poses:
        return None

    if len(poses) == 1:
        return _pose_yaw_rad(poses[0])

    accumulated = 0.0
    prev = poses[0]
    for i in range(1, len(poses)):
        cur = poses[i]
        dx = cur.position.x - prev.position.x
        dy = cur.position.y - prev.position.y
        dz = cur.position.z - prev.position.z
        seg_len = math.sqrt(dx * dx + dy * dy + dz * dz)
        if seg_len <= 1e-9:
            prev = cur
            continue

        if accumulated + seg_len >= lookahead_distance:
            remaining = lookahead_distance - accumulated
            t = remaining / seg_len
            yaw_prev = _pose_yaw_rad(prev)
            yaw_cur = _pose_yaw_rad(cur)
            if yaw_prev is None or yaw_cur is None:
                return None
            dyaw = (yaw_cur - yaw_prev + math.pi) % (2 * math.pi) - math.pi
            return yaw_prev + dyaw * t

        accumulated += seg_len
        prev = cur

    return _pose_yaw_rad(poses[-1])


def extract_lookahead_bearing(
    local_plan: PoseArray,
    lookahead_distance: float,
) -> float | None:
    """提取base_link到前视点的几何方位，避免TEB姿态角噪声驱动摇摆。"""
    if local_plan is None or lookahead_distance <= 0.0 or not local_plan.poses:
        return None
    poses = list(local_plan.poses)
    previous = poses[0]
    accumulated = 0.0
    target_x = float(previous.position.x)
    target_y = float(previous.position.y)
    for current in poses[1:]:
        dx = float(current.position.x - previous.position.x)
        dy = float(current.position.y - previous.position.y)
        segment_length = math.hypot(dx, dy)
        if segment_length > 1e-9:
            if accumulated + segment_length >= lookahead_distance:
                ratio = (lookahead_distance - accumulated) / segment_length
                target_x = float(previous.position.x) + ratio * dx
                target_y = float(previous.position.y) + ratio * dy
                break
            accumulated += segment_length
            target_x = float(current.position.x)
            target_y = float(current.position.y)
        previous = current
    if math.hypot(target_x, target_y) < 0.05:
        return None
    return math.atan2(target_y, target_x)


def compute_nav_speed_with_mode(
    teb_linear_x: float,
    teb_max_vel_x: float,
    mode_speed: int,
) -> int:
    """TEB linear.x → 协议 forward_speed（按档位比例缩放，spec § 3.2）。

    档位定义"TEB 全速时的协议速度"。TEB 输出比例（带符号）完整保留 →
    避障减速、起步加速等曲线形状不变。

    归一化：normalized = teb_linear_x / teb_max_vel_x，clamp 到 [-1, 1]
    防 TEB 偶发超量。结果 clamp 到 [-1000, 1000]。

    NaN/Inf 输入返回 0。
    """
    if math.isnan(teb_linear_x) or math.isinf(teb_linear_x):
        return 0
    if teb_max_vel_x <= 1e-6:
        return 0
    normalized = teb_linear_x / teb_max_vel_x
    normalized = max(-1.0, min(1.0, normalized))
    raw = int(round(normalized * mode_speed))
    return max(-1000, min(1000, raw))


def twist_has_motion(
    twist: Twist,
    linear_deadband_mps: float = 0.001,
    angular_deadband_rad_s: float = 0.001,
) -> bool:
    """True 表示 Twist 是用户正在操控底盘的非零动作。"""
    linear_x = float(twist.linear.x)
    omega = float(twist.angular.z)
    return (
        abs(linear_x) >= max(0.0, float(linear_deadband_mps))
        or abs(omega) >= max(0.0, float(angular_deadband_rad_s))
    )


def encode_frame(direction_angle: int, current_angle: int, forward_speed: int) -> str:
    """三字段 → ASCII 字符串（下位机协议以 \\n 作为帧头）。"""
    return f"\n{int(direction_angle)},{int(current_angle)},{int(forward_speed)}"


def encode_frame_bytes(direction_angle: int, current_angle: int, forward_speed: int) -> bytes:
    """三字段 → UTF-8 ASCII 字节（供 serial.write 直接使用）。"""
    return encode_frame(direction_angle, current_angle, forward_speed).encode("ascii")


class ChassisSerialNode(Node):
    """实物底盘串口节点。

    通过 serial_factory 注入串口对象（测试用 MockSerial，生产用 serial.Serial）。
    open_serial=False 时跳过真实串口打开（测试用）。
    overrides 字典允许测试覆盖任意 ROS 参数默认值。
    """

    def __init__(
        self,
        serial_factory=None,
        open_serial: bool = True,
        overrides: dict | None = None,
    ):
        super().__init__('chassis_serial_node')
        overrides = overrides or {}

        # === 声明参数 ===
        self.declare_parameter('serial_port', '/dev/wheelchair_chassis')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('serial_reopen_interval_sec', 1.0)
        self.declare_parameter('update_rate_hz', 100.0)
        self.declare_parameter('max_speed_mps', 1.5)
        self.declare_parameter('lead_gain_deg_per_rad_per_sec', 30.0)
        self.declare_parameter('max_lead_deg', 60.0)
        self.declare_parameter('heading_sign', -1)
        self.declare_parameter('nav_chassis_control_enabled', False)
        self.declare_parameter('nav_control_timeout_sec', 0.8)
        self.declare_parameter('nav_min_forward_speed', 0)
        self.declare_parameter('nav_min_turn_speed', 0)
        self.declare_parameter('nav_linear_deadband_mps', 0.01)
        self.declare_parameter('nav_angular_deadband_rad_s', 0.05)
        self.declare_parameter('nav_floor_linear_threshold_mps', 0.05)
        self.declare_parameter('nav_allow_reverse', False)
        # Nav2 模式下 omega 绝对值上限（rad/s）。
        # 实测 TEB 输出 omega=-0.8 rad/s 让下位机做原地急转弯，两轮反向大功率
        # → USB hub 过流掉电。Cap 到 0.3 rad/s 把电流冲击降到 USB hub 容忍范围。
        # 0 = 不限制（恢复旧行为）。仅作用于 Nav2 路径，不影响 EEG。
        self.declare_parameter('nav_omega_cap_rad_s', 0.3)
        self.declare_parameter('eeg_turn_offset_deg', 90.0)

        self.declare_parameter('eeg_forward_direction_angle', 0)
        self.declare_parameter('eeg_forward_current_angle', 0)
        self.declare_parameter('eeg_forward_speed', 900)

        self.declare_parameter('eeg_backward_direction_angle', 0)
        self.declare_parameter('eeg_backward_current_angle', 0)
        self.declare_parameter('eeg_backward_speed', -700)

        self.declare_parameter('eeg_left_direction_angle', -90)
        self.declare_parameter('eeg_left_current_angle', 0)
        self.declare_parameter('eeg_left_speed', 300)

        self.declare_parameter('eeg_right_direction_angle', 90)
        self.declare_parameter('eeg_right_current_angle', 0)
        self.declare_parameter('eeg_right_speed', 300)
        # === EEG roll 方向微调补偿参数（spec § 3）===
        self.declare_parameter('eeg_roll_compensation_enabled', True)
        self.declare_parameter('eeg_roll_gain_deg_per_deg', 0.5)
        self.declare_parameter('eeg_roll_saturation_deg', 20.0)
        self.declare_parameter('eeg_roll_polarity', -1)
        self.declare_parameter('eeg_head_pose_timeout_sec', 3.0)
        self.declare_parameter('eeg_override_hold_sec', 1.5)
        self.declare_parameter('eeg_motion_linear_deadband_mps', 0.001)
        self.declare_parameter('eeg_motion_angular_deadband_rad_s', 0.001)

        # === Nav2 lookahead + 速度档位参数（spec § 3）===
        self.declare_parameter('nav_use_local_plan_lookahead', True)
        self.declare_parameter('nav_lookahead_distance_m', 0.5)
        self.declare_parameter('nav_lookahead_lpf_alpha', 0.3)
        self.declare_parameter('nav_local_plan_timeout_sec', 0.5)
        self.declare_parameter('nav_steering_deadband_deg', 2.0)
        self.declare_parameter('nav_teb_max_vel_x', 0.6)
        self.declare_parameter('nav_speed_mode_default', 2)
        self.declare_parameter('nav_speed_slow', 400)
        self.declare_parameter('nav_speed_medium', 700)
        self.declare_parameter('nav_speed_fast', 1000)
        self.declare_parameter('nav_heading_hold_enabled', True)
        self.declare_parameter('nav_steering_trim_deg', 0.0)
        self.declare_parameter('nav_heading_hold_enter_error_deg', 5.0)
        self.declare_parameter('nav_heading_hold_exit_error_deg', 10.0)
        self.declare_parameter('nav_heading_hold_enter_omega_rad_s', 0.05)
        self.declare_parameter('nav_heading_hold_exit_omega_rad_s', 0.12)
        self.declare_parameter('gps_required_for_motion', False)
        self.declare_parameter('gps_fix_timeout_sec', 1.5)

        # === Nav2 Protector 参数（spec § 3.3）===
        self.declare_parameter('nav2_protector_enabled', True)
        self.declare_parameter('nav2_lookahead_jump_threshold_deg', 30.0)
        self.declare_parameter('nav2_cmd_vel_step_linear', 0.3)
        self.declare_parameter('nav2_cmd_vel_step_angular', 0.5)
        self.declare_parameter('nav2_ramp_cooldown_ms', 200)
        self.declare_parameter('nav2_hard_cooldown_ms', 1000)
        self.declare_parameter('nav2_cooldown_escalation_count', 3)
        self.declare_parameter('nav2_cooldown_escalation_window_sec', 1.0)
        self.declare_parameter('nav2_clear_goal_window_sec', 0.5)

        # === 限速器参数（spec § 4.1）===
        self.declare_parameter('slew_limiter_enabled', True)
        self.declare_parameter('forward_speed_ramp_per_sec', 2000)
        self.declare_parameter('direction_jump_threshold_deg', 45.0)
        self.declare_parameter('direction_cooldown_ms', 200)
        self.declare_parameter('jump_count_window_sec', 1.0)
        self.declare_parameter('jump_count_threshold', 3)
        self.declare_parameter('direction_long_cooldown_ms', 800)

        # 兼容旧参数名，但运行时不再使用：
        # HWT906P 只属于上位机导航姿态链路，不能参与脑控底盘控制帧生成。
        self.declare_parameter('eeg_use_heading_feedback', False)
        self.declare_parameter('eeg_fixed_current_angle_deg', 0.0)
        self.declare_parameter('heading_timeout_sec', 0.5)
        self.declare_parameter('cmd_vel_timeout_sec', 1.0)
        self.declare_parameter('shutdown_zero_repeat', 3)

        # 测试覆盖（在 declare 之后立即 set）
        for k, v in overrides.items():
            self.set_parameters([rclpy.parameter.Parameter(k, value=v)])

        # === 参数校验（fail-fast）===
        max_speed = float(self.get_parameter('max_speed_mps').value)
        if max_speed <= 0:
            self.get_logger().fatal(f"max_speed_mps 必须 > 0，当前 {max_speed}")
            raise ValueError(f"max_speed_mps must be > 0, got {max_speed}")
        heading_sign = int(self.get_parameter('heading_sign').value)
        if heading_sign not in (-1, 1):
            self.get_logger().fatal(
                f"heading_sign 必须 ∈ {{-1, 1}}，当前 {heading_sign}"
            )
            raise ValueError(f"heading_sign must be -1 or 1, got {heading_sign}")
        roll_gain = float(self.get_parameter('eeg_roll_gain_deg_per_deg').value)
        if roll_gain < 0:
            self.get_logger().fatal(
                f"eeg_roll_gain_deg_per_deg 必须 >= 0，当前 {roll_gain}"
            )
            raise ValueError(
                f"eeg_roll_gain_deg_per_deg must be >= 0, got {roll_gain}"
            )
        roll_sat = float(self.get_parameter('eeg_roll_saturation_deg').value)
        if roll_sat <= 0:
            self.get_logger().fatal(
                f"eeg_roll_saturation_deg 必须 > 0，当前 {roll_sat}"
            )
            raise ValueError(
                f"eeg_roll_saturation_deg must be > 0, got {roll_sat}"
            )
        roll_polarity = int(self.get_parameter('eeg_roll_polarity').value)
        if roll_polarity not in (-1, 1):
            self.get_logger().fatal(
                f"eeg_roll_polarity 必须 ∈ {{-1, 1}}，当前 {roll_polarity}"
            )
            raise ValueError(
                f"eeg_roll_polarity must be -1 or 1, got {roll_polarity}"
            )
        lookahead_dist = float(self.get_parameter('nav_lookahead_distance_m').value)
        if lookahead_dist <= 0:
            self.get_logger().fatal(
                f"nav_lookahead_distance_m 必须 > 0，当前 {lookahead_dist}"
            )
            raise ValueError(
                f"nav_lookahead_distance_m must be > 0, got {lookahead_dist}"
            )
        lpf_alpha = float(self.get_parameter('nav_lookahead_lpf_alpha').value)
        if not 0 < lpf_alpha <= 1:
            self.get_logger().fatal(
                f"nav_lookahead_lpf_alpha 必须 ∈ (0, 1]，当前 {lpf_alpha}"
            )
            raise ValueError(
                f"nav_lookahead_lpf_alpha must be in (0, 1], got {lpf_alpha}"
            )
        plan_timeout = float(self.get_parameter('nav_local_plan_timeout_sec').value)
        if plan_timeout <= 0:
            self.get_logger().fatal(
                f"nav_local_plan_timeout_sec 必须 > 0，当前 {plan_timeout}"
            )
            raise ValueError(
                f"nav_local_plan_timeout_sec must be > 0, got {plan_timeout}"
            )
        teb_max_vel = float(self.get_parameter('nav_teb_max_vel_x').value)
        if teb_max_vel <= 0:
            self.get_logger().fatal(
                f"nav_teb_max_vel_x 必须 > 0，当前 {teb_max_vel}"
            )
            raise ValueError(
                f"nav_teb_max_vel_x must be > 0, got {teb_max_vel}"
            )
        speed_mode = int(self.get_parameter('nav_speed_mode_default').value)
        if speed_mode not in (1, 2, 3):
            self.get_logger().fatal(
                f"nav_speed_mode_default 必须 ∈ {{1, 2, 3}}，当前 {speed_mode}"
            )
            raise ValueError(
                f"nav_speed_mode_default must be 1, 2 or 3, got {speed_mode}"
            )
        slow = int(self.get_parameter('nav_speed_slow').value)
        med = int(self.get_parameter('nav_speed_medium').value)
        fast = int(self.get_parameter('nav_speed_fast').value)
        if not (1 <= slow < med < fast <= 1000):
            self.get_logger().fatal(
                f"速度档位必须满足 1≤slow<medium<fast≤1000，"
                f"当前 slow={slow}, medium={med}, fast={fast}"
            )
            raise ValueError(
                f"nav_speed ordering violated: "
                f"nav_speed_slow={slow}, nav_speed_medium={med}, nav_speed_fast={fast}"
            )
        # === 限速器参数校验（spec § 4.3）===
        if not isinstance(
            self.get_parameter('slew_limiter_enabled').value, bool
        ):
            raise ValueError("slew_limiter_enabled 必须是 bool")
        slew_ramp = int(self.get_parameter('forward_speed_ramp_per_sec').value)
        if not (0 < slew_ramp <= 10000):
            self.get_logger().fatal(
                f"forward_speed_ramp_per_sec 必须 ∈ (0, 10000]，当前 {slew_ramp}"
            )
            raise ValueError(
                f"forward_speed_ramp_per_sec must be in (0, 10000], got {slew_ramp}"
            )
        slew_jump_th = float(self.get_parameter('direction_jump_threshold_deg').value)
        if not (0 < slew_jump_th <= 180):
            self.get_logger().fatal(
                f"direction_jump_threshold_deg 必须 ∈ (0, 180]，当前 {slew_jump_th}"
            )
            raise ValueError(
                f"direction_jump_threshold_deg must be in (0, 180], got {slew_jump_th}"
            )
        slew_cd_ms = int(self.get_parameter('direction_cooldown_ms').value)
        if slew_cd_ms < 0:
            raise ValueError(
                f"direction_cooldown_ms must be >= 0, got {slew_cd_ms}"
            )
        slew_window = float(self.get_parameter('jump_count_window_sec').value)
        if not (slew_window > 0):
            raise ValueError(
                f"jump_count_window_sec must be > 0, got {slew_window}"
            )
        slew_count_th = int(self.get_parameter('jump_count_threshold').value)
        if slew_count_th < 1:
            raise ValueError(
                f"jump_count_threshold must be >= 1, got {slew_count_th}"
            )
        slew_long_cd_ms = int(self.get_parameter('direction_long_cooldown_ms').value)
        if slew_long_cd_ms < 0:
            raise ValueError(
                f"direction_long_cooldown_ms must be >= 0, got {slew_long_cd_ms}"
            )
        # === 串口 ===
        self._serial_factory = serial_factory or self._default_serial_factory
        self._serial = None
        self._last_serial_open_attempt_sec = 0.0
        if open_serial:
            self._open_serial()
        elif serial_factory is not None:
            # 测试注入 serial_factory 时即使 open_serial=False 也持有 mock，
            # 否则 _write_serial 无目标可写（dependency injection 语义）。
            self._serial = self._serial_factory()

        # === 限速器（spec § 2.3）===
        self._slew_limiter = ChassisSlewLimiter(
            ramp_per_sec=int(
                self.get_parameter('forward_speed_ramp_per_sec').value),
            jump_threshold_deg=float(
                self.get_parameter('direction_jump_threshold_deg').value),
            cooldown_sec=float(
                self.get_parameter('direction_cooldown_ms').value) / 1000.0,
            window_sec=float(
                self.get_parameter('jump_count_window_sec').value),
            count_threshold=int(
                self.get_parameter('jump_count_threshold').value),
            long_cooldown_sec=float(
                self.get_parameter('direction_long_cooldown_ms').value) / 1000.0,
        )
        self._slew_limiter.set_enabled(
            bool(self.get_parameter('slew_limiter_enabled').value)
        )

        # === Nav2Protector 实例化（spec § 3.3）===
        self._nav2_protector_params = Nav2ProtectorParams(
            lookahead_jump_threshold_deg=float(
                self.get_parameter('nav2_lookahead_jump_threshold_deg').value),
            cmd_vel_step_linear=float(
                self.get_parameter('nav2_cmd_vel_step_linear').value),
            cmd_vel_step_angular=float(
                self.get_parameter('nav2_cmd_vel_step_angular').value),
            ramp_cooldown_ms=int(
                self.get_parameter('nav2_ramp_cooldown_ms').value),
            hard_cooldown_ms=int(
                self.get_parameter('nav2_hard_cooldown_ms').value),
            cooldown_escalation_count=int(
                self.get_parameter('nav2_cooldown_escalation_count').value),
            cooldown_escalation_window_sec=float(
                self.get_parameter('nav2_cooldown_escalation_window_sec').value),
            clear_goal_window_sec=float(
                self.get_parameter('nav2_clear_goal_window_sec').value),
        )
        self._nav2_protector = Nav2AutonomyProtector(
            params=self._nav2_protector_params,
            clock_fn=self._now_sec,
            log_fn=lambda level, msg: getattr(self.get_logger(), level.lower())(msg),
        )
        self._nav2_protector_enabled = bool(
            self.get_parameter('nav2_protector_enabled').value)

        # 状态发布
        self._protection_status_pub = self.create_publisher(
            String, '/chassis_protection_status', 10)
        self._last_status_publish_sec: float = 0.0
        self._status_publish_period_sec: float = 0.1  # 10Hz
        self._last_frame_status: str = 'NORMAL'

        # FollowPath 状态订阅
        self._follow_path_active: bool = False
        self._follow_path_just_started: bool = False
        self._follow_path_just_ended: bool = False
        self._follow_path_status_sub = self.create_subscription(
            GoalStatusArray, '/follow_path_status',
            self._follow_path_status_cb, 10)

        # 上一帧缓存（用于触发器判定）
        self._prev_direction_deg: float = 0.0
        self._prev_cmd_vel_linear_x: float = 0.0
        self._prev_cmd_vel_angular_z: float = 0.0
        self._prev_nav_speed_mode: int = 1
        # 首帧初始化标志：第一次 _build_frame_ctx 时把 prev 设为本帧值，
        # 让 Δ=0 跳过 T1/T2/T4 触发器，避免启动自主导航第一帧就误触发 RAMP_COOLDOWN。
        # T3/T5 由 follow_path/clear_goal 事件门控，不受影响。
        self._frame_ctx_initialized: bool = False
        self._last_clear_goal_sec: float = -1.0  # -1 表示从未触发
        # dt 计算需要：记录上次 tick 时间戳
        self._last_tick_now_sec = 0.0
        # 监听运行时 slew_limiter_enabled 切换
        self.add_on_set_parameters_callback(self._on_slew_param_changed)

        # === 最新输入缓存 ===
        self._latest_cmd_vel: Twist | None = None
        self._latest_cmd_vel_sec: float = 0.0
        self._latest_heading_deg: float | None = None
        self._latest_heading_sec: float = 0.0
        self._latest_gps_valid: bool = False
        self._latest_gps_sec: float = 0.0
        self._nav_control_active = False
        self._latest_nav_control_sec: float = 0.0

        # === EEG mux 状态（与 sim_chassis_node 对齐）===
        # eeg_mode_active=True 只表示脑控待命；最近出现非零 /cmd_vel_eeg
        # 才临时接管底盘。接管释放前仍执行 EEG 零速帧，避免立刻回到 Nav2。
        # 3s 无 /eeg_mode_active 心跳 → 自动 fallback 到 Nav2。
        self.eeg_mode_active = False
        self._last_eeg_mode_msg_time = self._now_sec()
        self._latest_cmd_vel_eeg: Twist | None = None
        self._latest_cmd_vel_eeg_sec: float = 0.0
        self._last_eeg_motion_sec: float = 0.0
        self._eeg_override_active = False
        self._force_stop_active = False  # /clear_goal 触发的强制零速
        # /clear_goal 安全保障：强制停止必须保持至少 hold_sec，并且期间 cmd_vel_safe
        # 必须**连续** zero（连续 N 帧），才允许解除。
        # 单帧 zero 不够：controller cancel 是异步的（50-100ms 延迟），cancel 期间可能
        # 还在发非零 cmd_vel，单帧 zero 触发解除后下一帧 cmd_vel 又来 → 底盘继续动。
        self.declare_parameter('clear_goal_hold_sec', 0.5)
        self._clear_goal_hold_sec = float(self.get_parameter('clear_goal_hold_sec').value)
        self._force_stop_start_sec: float = 0.0
        self._consecutive_zero_frames: int = 0
        self._clear_goal_zero_threshold: int = 30  # 100Hz × 0.3s = 30 帧连续 zero
        # === cmd_vel soft-start ramp（缓解电机启动电流尖峰） ===
        self.declare_parameter('cmd_vel_ramp_rate', 1.0)  # m/s²，每秒最大速度增量（USB Hub OCP 错峰）
        self._cmd_vel_ramp_rate = float(self.get_parameter('cmd_vel_ramp_rate').value)
        self._ramp_linear_x: float = 0.0
        self._ramp_omega: float = 0.0
        self._last_ramp_sec: float = 0.0
        # === 串口打开静默期（覆盖下位机上电自检 / 使能电机冲击窗口）===
        self.declare_parameter('serial_open_silence_sec', 2.0)
        self._serial_open_sec: float = 0.0
        self._serial_silence_sec: float = float(
            self.get_parameter('serial_open_silence_sec').value
        )
        # === EEG roll 缓存（spec § 4.2 改动 3）===
        # 注意：默认 0.0 不是 None，避免 _tick 内 NoneType 异常；
        # 失联判定靠 _latest_head_pose_sec + timeout。
        self._latest_roll_deg: float = 0.0
        self._latest_head_pose_sec: float = 0.0

        # === Nav2 local_plan + 速度档位缓存（spec § 4.2 / § 3.3）===
        self._latest_local_plan: PoseArray | None = None
        self._latest_local_plan_sec: float = 0.0
        self._lookahead_yaw_filtered: float = 0.0  # LPF 状态
        self._current_speed_mode: int = int(self.get_parameter('nav_speed_mode_default').value)
        self._nav_heading_hold = NavHeadingHold(
            enabled=bool(self.get_parameter('nav_heading_hold_enabled').value),
            steering_trim_deg=float(self.get_parameter('nav_steering_trim_deg').value),
            enter_error_deg=float(self.get_parameter('nav_heading_hold_enter_error_deg').value),
            exit_error_deg=float(self.get_parameter('nav_heading_hold_exit_error_deg').value),
            enter_omega_rad_s=float(self.get_parameter('nav_heading_hold_enter_omega_rad_s').value),
            exit_omega_rad_s=float(self.get_parameter('nav_heading_hold_exit_omega_rad_s').value),
        )

        # === dead-reckon 状态（odom→base_link TF）===
        # 下位机不上报编码器，靠 cmd_vel + heading 推算位置给 RViz/TEB。
        # 这里同时发布 /odom topic；TEB controller_server 订阅的是 Odometry 消息，
        # 只发布 TF 不足以让 TEB 正常闭环。
        self._dead_reckon_x_m = 0.0
        self._dead_reckon_y_m = 0.0
        self._last_tick_sec = 0.0

        # mux 选出的最新速度（供 _publish_odom_tf 复用 + 未来角速度积分/调试预留，避免重复算 mux）
        self._selected_linear_x = 0.0
        self._selected_omega = 0.0
        self._last_serial_frame_log_sec = 0.0

        self._tf_broadcaster = TransformBroadcaster(self)
        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # === 订阅 ===
        self.create_subscription(Twist, '/cmd_vel_safe', self._cmd_vel_cb, 10)
        self.create_subscription(Float64, '/heading_imu', self._heading_cb, 10)
        self.create_subscription(NavSatFix, '/fix', self._gps_fix_cb, 10)
        self.create_subscription(Bool, '/nav_control_active', self._nav_control_active_cb, 10)

        # EEG mux 新增订阅
        self.create_subscription(Twist, '/cmd_vel_eeg', self._cmd_vel_eeg_cb, 1)
        self.create_subscription(Bool, '/eeg_mode_active', self._eeg_mode_active_cb, 10)
        self.create_subscription(Empty, '/clear_goal', self._clear_goal_cb, 10)
        self.create_subscription(
            Float64, '/eeg_head_pose', self._head_pose_cb, 10
        )

        # Nav2 lookahead + 档位订阅（spec § 4.2 / § 3.3）
        self.create_subscription(PoseArray, '/local_plan', self._local_plan_cb, 10)
        self.create_subscription(Int8, '/nav_speed_mode', self._speed_mode_cb, 10)

        # === EEG 心跳 fallback：3s 无 /eeg_mode_active → 自动回 Nav2 ===
        self.create_timer(1.0, self._check_eeg_mode_fallback)

        # === 100Hz 定时器 ===
        rate = float(self.get_parameter('update_rate_hz').value)
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f"ChassisSerialNode started: port={self.get_parameter('serial_port').value}, "
            f"baud={self.get_parameter('baudrate').value}, rate={rate}Hz"
        )

    def _default_serial_factory(self, **kwargs):
        import serial
        return serial.Serial(**kwargs)

    def _open_serial(self) -> bool:
        port = self.get_parameter('serial_port').value
        baud = int(self.get_parameter('baudrate').value)
        self._last_serial_open_attempt_sec = self._now_sec()
        try:
            self._serial = self._serial_factory(
                port=port,
                baudrate=baud,
                timeout=0,
                exclusive=True,
            )
            self.get_logger().info(f"串口已打开: {port}@{baud} exclusive=True")
            # 启动静默期：标记串口刚打开时间，_tick 在静默期内强制输出零速，
            # 覆盖下位机固件"上电自检 / 使能电机"窗口（典型 1-2 秒），
            # 避免下位机自检冲击叠加首帧 cmd_vel 触发可见的"转一下"。
            self._serial_open_sec = self._now_sec()
            self._serial_silence_sec = float(
                self.get_parameter('serial_open_silence_sec').value
            ) if self.has_parameter('serial_open_silence_sec') else 2.0
            return True
        except Exception as e:
            self._serial = None
            self.get_logger().error(
                f"串口打开失败 {port}: {e}；节点保持运行并等待热插拔/重连"
            )
            return False

    def _cmd_vel_cb(self, msg: Twist):
        """Nav2 速度回调。

        即使脑控模式已待命，也持续缓存 Nav2 的 /cmd_vel_safe。
        这样脑控临时接管释放后，自主导航无需重新发 goal 就能继续执行。
        """
        self._latest_cmd_vel = msg
        self._latest_cmd_vel_sec = self._now_sec()

    def _heading_cb(self, msg: Float64):
        self._latest_heading_deg = float(msg.data)
        self._latest_heading_sec = self._now_sec()

    def _nav_control_active_cb(self, msg: Bool):
        prev = self._nav_control_active
        self._nav_control_active = bool(msg.data)
        self._latest_nav_control_sec = self._now_sec()
        if prev != self._nav_control_active:
            self.get_logger().info(
                f"nav_control_active: {prev} -> {self._nav_control_active}"
            )

    def _cmd_vel_eeg_cb(self, msg: Twist):
        """脑控速度回调：仅 eeg_mode_active=True 时消费。"""
        if self.eeg_mode_active:
            now = self._now_sec()
            self._latest_cmd_vel_eeg = msg
            self._latest_cmd_vel_eeg_sec = now
            if self._is_eeg_motion_cmd(msg):
                self._last_eeg_motion_sec = now

    def _head_pose_cb(self, msg: Float64):
        """头部 roll 缓存（BrainControlTab 发布，spec § 4.2 改动 3）。

        只在 EEG override 期间被消费；超时由 _tick 内 timeout 检查处理。
        """
        self._latest_roll_deg = float(msg.data)
        self._latest_head_pose_sec = self._now_sec()

    def _local_plan_cb(self, msg: PoseArray):
        """TEB /local_plan 缓存（spec § 4.2）。

        只在 Nav2 模式下被消费；超时由 _tick 内 timeout 检查处理。
        frame_id 不是 base_link 时 _tick 内会 fallback。
        """
        self._latest_local_plan = msg
        self._latest_local_plan_sec = self._now_sec()

    def _gps_fix_cb(self, msg: NavSatFix):
        self._latest_gps_sec = self._now_sec()
        self._latest_gps_valid = (
            msg.status.status >= 0
            and math.isfinite(msg.latitude)
            and math.isfinite(msg.longitude)
            and not (abs(msg.latitude) < 1.0 and abs(msg.longitude) < 1.0)
        )

    def _is_gps_ready(self, now: float) -> bool:
        if not bool(self.get_parameter('gps_required_for_motion').value):
            return True
        timeout = max(0.1, float(self.get_parameter('gps_fix_timeout_sec').value))
        return self._latest_gps_valid and (now - self._latest_gps_sec) <= timeout

    def _speed_mode_cb(self, msg: Int8):
        """/nav_speed_mode 档位切换（前端按键触发）。"""
        new_mode = int(msg.data)
        if new_mode not in (1, 2, 3):
            self.get_logger().warn(
                f"收到无效 nav_speed_mode={new_mode}，忽略（必须 ∈ {{1, 2, 3}}）"
            )
            return
        if new_mode != self._current_speed_mode:
            self._current_speed_mode = new_mode
            self.get_logger().info(f"nav_speed_mode: {new_mode}")

    def _on_slew_param_changed(self, params):
        """参数变更回调：同步 slew_limiter_enabled 到限速器（紧急回退用）。

        其他限速器参数（ramp/threshold/cooldown 等）改动需重启节点才生效，
        因限速器构造时已快照参数。仅 slew_limiter_enabled 支持运行时切换。

        ROS2 Humble 期望返回单个 SetParametersResult（不是 list）。
        """
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == 'slew_limiter_enabled':
                self._slew_limiter.set_enabled(bool(p.value))
                self.get_logger().info(
                    f"slew_limiter_enabled 运行时切换: {bool(p.value)}"
                )
        return SetParametersResult(successful=True)

    def _eeg_mode_active_cb(self, msg: Bool):
        """切换脑控待命状态。

        - True: 脑控待命，但不立即打断 Nav2
        - False: 清空脑控接管状态，回 Nav2
        每次刷新心跳时间戳（供 _check_eeg_mode_fallback 监测）。
        """
        prev = self.eeg_mode_active
        self.eeg_mode_active = bool(msg.data)
        self._last_eeg_mode_msg_time = self._now_sec()
        if prev != self.eeg_mode_active:
            self.get_logger().info(
                f"eeg_mode_active: {prev} -> {self.eeg_mode_active}"
            )
            self._latest_cmd_vel_eeg = None
            self._latest_cmd_vel_eeg_sec = 0.0
            self._last_eeg_motion_sec = 0.0
            self._eeg_override_active = False

    def _clear_goal_cb(self, msg: Empty):
        """前端'清除终点' → 强制零速，并保持至少 hold_sec + 连续 N 帧 zero 才解除。

        策略（双重保障）：
        - 按按钮 → 设 _force_stop_active=True，记录开始时间
        - 解除条件（必须全部满足）：
          (1) 已经过 clear_goal_hold_sec（默认 0.5s）—— 让 controller cancel 充分生效
          (2) cmd_vel_safe 连续 N 帧（30 帧 @ 100Hz = 0.3s）真为零
        - 任一帧非零 → 计数器清零，重新等待
        """
        now = self._now_sec()
        self._force_stop_active = True
        self._force_stop_start_sec = now
        self._consecutive_zero_frames = 0
        self._last_clear_goal_sec = now  # 供 Nav2Protector T5 时序窗口判定
        self.get_logger().info(
            f"clear_goal 收到，强制 zero 至少保持 {self._clear_goal_hold_sec}s + "
            f"连续 {self._clear_goal_zero_threshold} 帧 zero 才解除"
        )

    def _follow_path_status_cb(self, msg: GoalStatusArray) -> None:
        """从 FollowPath action status 推断 active/just_started/just_ended。

        /follow_path_status 是 action_msgs/GoalStatusArray，status_list 末尾
        反映当前/最近一次 goal。我们只关心是否有 ACCEPTED/EXECUTING 状态。
        """
        if not msg.status_list:
            return
        last_status = msg.status_list[-1].status
        # GoalStatus: 1=ACCEPTED, 2=EXECUTING, 4=SUCCEEDED, 5=CANCELED, 6=ABORTED
        was_active = self._follow_path_active
        is_active = last_status in (1, 2)
        self._follow_path_active = is_active
        self._follow_path_just_started = is_active and not was_active
        self._follow_path_just_ended = was_active and not is_active

    def _build_frame_ctx(
        self,
        direction_deg: float,
        linear_x: float,
        omega: float,
        now: float,
    ) -> FrameContext:
        """收集本帧上下文，供 Nav2Protector 判定。"""
        # 首帧初始化：把 prev 设为本帧值，避免首帧误触发 T1/T2/T4。
        # 启动自主导航时，Protector 第一次 apply 看到 Δ=0 → NORMAL，后续按真实变化判定。
        if not self._frame_ctx_initialized:
            self._prev_direction_deg = float(direction_deg)
            self._prev_cmd_vel_linear_x = float(linear_x)
            self._prev_cmd_vel_angular_z = float(omega)
            self._prev_nav_speed_mode = self._current_speed_mode
            self._frame_ctx_initialized = True

        clear_goal_age = (now - self._last_clear_goal_sec
                          if self._last_clear_goal_sec >= 0 else float('inf'))
        ctx = FrameContext(
            cmd_vel_linear_x=linear_x,
            cmd_vel_angular_z=omega,
            prev_cmd_vel_linear_x=self._prev_cmd_vel_linear_x,
            prev_cmd_vel_angular_z=self._prev_cmd_vel_angular_z,
            nav_speed_mode=self._current_speed_mode,
            prev_nav_speed_mode=self._prev_nav_speed_mode,
            follow_path_active=self._follow_path_active,
            follow_path_just_started=self._follow_path_just_started,
            follow_path_just_ended=self._follow_path_just_ended,
            clear_goal_age_sec=clear_goal_age,
            prev_direction_deg=self._prev_direction_deg,
        )
        # 更新 prev 缓存
        self._prev_cmd_vel_linear_x = linear_x
        self._prev_cmd_vel_angular_z = omega
        self._prev_direction_deg = direction_deg
        self._prev_nav_speed_mode = self._current_speed_mode
        return ctx

    def _publish_protection_status(self, status: str, now: float) -> None:
        """10Hz 节流发布 Protector 状态。"""
        self._last_frame_status = status
        if now - self._last_status_publish_sec < self._status_publish_period_sec:
            return
        msg = String()
        msg.data = status
        self._protection_status_pub.publish(msg)
        self._last_status_publish_sec = now

    def _check_eeg_mode_fallback(self):
        """3 秒无 /eeg_mode_active 心跳 → fallback 到 Nav2 模式。

        BrainControlTab 崩溃 / EEGLogger 掉线 → 心跳停止 → 自动交还控制权。
        """
        if not self.eeg_mode_active:
            return
        now = self._now_sec()
        if now - self._last_eeg_mode_msg_time > 3.0:
            self.get_logger().warn(
                "eeg_mode_active 3s 无更新，fallback 到 Nav2 模式"
            )
            self.eeg_mode_active = False
            self._eeg_override_active = False
            self._last_eeg_motion_sec = 0.0

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _is_eeg_motion_cmd(self, msg: Twist) -> bool:
        return twist_has_motion(
            msg,
            linear_deadband_mps=float(
                self.get_parameter('eeg_motion_linear_deadband_mps').value),
            angular_deadband_rad_s=float(
                self.get_parameter('eeg_motion_angular_deadband_rad_s').value),
        )

    def _should_use_eeg_override(self, now: float) -> bool:
        """脑控是否应临时接管底盘。

        eeg_mode_active 只是待命信号；接管由最近一次非零 /cmd_vel_eeg 触发，
        并在 eeg_override_hold_sec 内保持。保持期内如果 BrainControlTab 持续
        发布 STOP，底盘会执行零速；保持期结束后自动回到 Nav2。
        """
        if not self.eeg_mode_active:
            return False
        if self._last_eeg_motion_sec <= 0.0:
            return False
        hold_sec = max(0.0, float(self.get_parameter('eeg_override_hold_sec').value))
        return (now - self._last_eeg_motion_sec) <= hold_sec

    def _tick(self):
        """100Hz 主循环：mux 选 Twist → 计算三字段 → 编码 → 写串口。

        优先级：串口静默期 > _force_stop_active > EEG 临时接管 > Nav2。
        """
        now = self._now_sec()
        heading_stale = self._is_heading_stale(now)
        nav_control_ready = self._is_nav_control_ready(now)
        gps_ready = self._is_gps_ready(now)
        eeg_override_requested = self._should_use_eeg_override(now)
        self._eeg_override_active = False

        # === 串口静默期：刚打开串口 2 秒内强制零速 ===
        # 下位机固件上电时会做电机自检 / 使能电机，瞬间有冲击电流 → 底盘转一下。
        # 软件无法阻止下位机自检本身，但静默期确保软件层不主动发任何非零速度，
        # 把软件可控的部分压到零。
        if (self._serial_open_sec > 0
                and now - self._serial_open_sec < self._serial_silence_sec):
            linear_x, omega = 0.0, 0.0
        # === mux：选 Twist 来源 ===
        elif not gps_ready and not eeg_override_requested:
            linear_x, omega = 0.0, 0.0
            self.get_logger().error(
                'GPS无有效定位或定位超时，自主导航硬门控停车',
                throttle_duration_sec=2.0,
            )
        elif self._force_stop_active:
            # /clear_goal 强制零速（覆盖 EEG 和 Nav2）
            # 解除条件双重保障：
            #   (1) 已过 hold_sec（让 controller cancel 充分生效，防止单帧 zero 假信号）
            #   (2) cmd_vel_safe 连续 N 帧真为零（confirm Nav2 真的停了）
            now = self._now_sec()
            elapsed = now - self._force_stop_start_sec
            cmd_zero = (self._latest_cmd_vel is not None
                    and abs(self._latest_cmd_vel.linear.x) < 0.001
                    and abs(self._latest_cmd_vel.angular.z) < 0.001)
            if cmd_zero:
                self._consecutive_zero_frames += 1
            else:
                if self._consecutive_zero_frames > 0:
                    self.get_logger().warn(
                        f"强制停止期间 cmd_vel_safe 出现非零（连续 zero 帧计数 "
                        f"{self._consecutive_zero_frames} → 0），重置等待"
                    )
                self._consecutive_zero_frames = 0
            if (elapsed >= self._clear_goal_hold_sec
                    and self._consecutive_zero_frames >= self._clear_goal_zero_threshold):
                self._force_stop_active = False
                self.get_logger().info(
                    f"强制停止解除（保持 {elapsed:.2f}s + 连续 "
                    f"{self._consecutive_zero_frames} 帧 zero）"
                )
            # 强制期间本帧始终输出零速
            linear_x, omega = 0.0, 0.0
        elif eeg_override_requested:
            # EEG 临时接管：用 /cmd_vel_eeg。若命令流断开，保持期内先零速。
            self._eeg_override_active = True
            twist_src = self._latest_cmd_vel_eeg
            twist_src_sec = self._latest_cmd_vel_eeg_sec
            if twist_src is None or (now - twist_src_sec) > 1.0:
                linear_x, omega = 0.0, 0.0
            else:
                linear_x = float(twist_src.linear.x)
                omega = float(twist_src.angular.z)
        else:
            # Nav2 模式：用 /cmd_vel_safe
            if not nav_control_ready:
                linear_x, omega = 0.0, 0.0
            elif self._is_cmd_vel_stale(now) or self._latest_cmd_vel is None:
                linear_x, omega = 0.0, 0.0
            else:
                linear_x = float(self._latest_cmd_vel.linear.x)
                omega = float(self._latest_cmd_vel.angular.z)

        # 缓存供 _publish_odom_tf 使用（避免重复计算 mux 逻辑）
        self._selected_linear_x = linear_x
        self._selected_omega = omega

        # === Nav2 omega cap：限制角速度绝对值，降低原地急转弯电流冲击 ===
        # TEB 默认允许 omega=±0.8 rad/s，差速底盘做这种急转弯时两轮反向大功率
        # → USB hub 过流掉电。Cap 到 0.3 rad/s（默认）让电流冲击降到容忍范围。
        # 仅作用于 Nav2 路径（_eeg_override_active=False），不影响 EEG 模式。
        if not self._eeg_override_active:
            omega_cap = float(self.get_parameter('nav_omega_cap_rad_s').value)
            if omega_cap > 0:
                omega = max(-omega_cap, min(omega_cap, omega))
                self._selected_omega = omega

        # === Soft-start ramp：每帧线性限幅，缓解电机启动电流尖峰 ===
        # 电机从 0 启动到目标速度瞬态电流可达稳态 3-5 倍。
        # 同时多个 USB 设备（IMU/雷达/相机）也在供电，瞬时叠加 → USB 总线过流保护
        # → 主板 USB 端口 disconnect → ROS2 节点崩溃链 → 系统假死。
        # ramp 让每帧速度增量受限（默认 ramp_rate=2.0 m/s² 即 0.5s 达到 1.0 m/s）。
        if self._cmd_vel_ramp_rate > 0:
            dt = 0.01 if self._last_ramp_sec <= 0 else (now - self._last_ramp_sec)
            max_delta = self._cmd_vel_ramp_rate * dt
            dx = linear_x - self._ramp_linear_x
            domega = omega - self._ramp_omega
            self._ramp_linear_x += max(-max_delta, min(max_delta, dx))
            self._ramp_omega += max(-max_delta * 2.0, min(max_delta * 2.0, domega))
            self._last_ramp_sec = now
            linear_x = self._ramp_linear_x
            omega = self._ramp_omega

        # === 字段计算 ===
        if self._eeg_override_active:
            # 脑控接管（2026-07-12 修订）：current_angle 跟随 HWT 真值，
            # direction_angle = current_angle + 动作偏移。借鉴 Nav2 路径，
            # 让下位机看到的三字段结构与自主导航同构，避免新加的加速度限制
            # 因 current_angle 始终为 0 误判为"无运动反馈"而拒绝执行。
            #
            # 旧实现固定 current_angle=0 在下位机加了加速度限制后导致 EEG
            # 完全不动（Nav2 模式正常）。HWT 失联时退化到零速帧（与 Nav2 一致）。
            if heading_stale or self._latest_heading_deg is None:
                current_angle = 0
                forward_speed = 0
                direction_angle = 0
            else:
                current_angle = wrap_deg(self._latest_heading_deg)
                # roll 缓存超时 → 临时变量取 0（不清 _latest_roll_deg，避免 None）
                head_pose_timeout = float(
                    self.get_parameter('eeg_head_pose_timeout_sec').value
                )
                if (now - self._latest_head_pose_sec) > head_pose_timeout:
                    roll_deg = 0.0
                else:
                    roll_deg = self._latest_roll_deg
                roll_offset = compute_roll_direction_offset_deg(
                    roll_deg=roll_deg,
                    enabled=bool(
                        self.get_parameter('eeg_roll_compensation_enabled').value),
                    gain=float(
                        self.get_parameter('eeg_roll_gain_deg_per_deg').value),
                    saturation=float(
                        self.get_parameter('eeg_roll_saturation_deg').value),
                    polarity=int(
                        self.get_parameter('eeg_roll_polarity').value),
                )
                direction_angle, current_angle, forward_speed = compute_eeg_directional_fields(
                    linear_x_mps=linear_x,
                    omega_rad_s=omega,
                    current_angle=current_angle,
                    forward_frame=self._eeg_frame_param('forward'),
                    backward_frame=self._eeg_frame_param('backward'),
                    left_frame=self._eeg_frame_param('left'),
                    right_frame=self._eeg_frame_param('right'),
                    roll_offset_deg=roll_offset,
                )
        elif not nav_control_ready:
            # HWT906P 是导航航向反馈源，不是运动意图源。
            # 只有 path_feeder 确认 TEB 正在跟踪有效路径时，/cmd_vel_safe 才能进入真实底盘。
            current_angle = 0
            forward_speed = 0
            direction_angle = 0
        elif heading_stale or self._latest_heading_deg is None:
            current_angle = 0
            forward_speed = 0
            direction_angle = 0
        else:
            current_angle = wrap_deg(self._latest_heading_deg)

            # === forward_speed：用档位算法（spec § 3.2）===
            mode_speed = {
                1: int(self.get_parameter('nav_speed_slow').value),
                2: int(self.get_parameter('nav_speed_medium').value),
                3: int(self.get_parameter('nav_speed_fast').value),
            }[self._current_speed_mode]
            # allow_reverse 守护：默认禁止后退（spec 既有行为）
            linear_x_for_calc = linear_x
            if not bool(self.get_parameter('nav_allow_reverse').value) and linear_x < 0.0:
                linear_x_for_calc = 0.0
            forward_speed = compute_nav_speed_with_mode(
                teb_linear_x=linear_x_for_calc,
                teb_max_vel_x=float(self.get_parameter('nav_teb_max_vel_x').value),
                mode_speed=mode_speed,
            )
            # floor 兜底（spec § 3.2 保留既有 min_forward_speed 逻辑）：
            # 档位算法算出的小速度可能不足以驱动电机死区，需抬到 min_forward_speed
            min_forward_speed = int(
                self.get_parameter('nav_min_forward_speed').value)
            floor_threshold = float(
                self.get_parameter('nav_floor_linear_threshold_mps').value)
            if (min_forward_speed > 0
                    and abs(linear_x_for_calc) >= floor_threshold
                    and 0 < abs(forward_speed) < min_forward_speed):
                forward_speed = (min_forward_speed if forward_speed > 0
                                 else -min_forward_speed)
            # deadzone 处理：linear.x 太小时归零或转 min_turn_speed
            linear_deadband = float(
                self.get_parameter('nav_linear_deadband_mps').value)
            angular_deadband = float(
                self.get_parameter('nav_angular_deadband_rad_s').value)
            if abs(linear_x) < linear_deadband:
                if abs(omega) >= angular_deadband:
                    forward_speed = int(
                        self.get_parameter('nav_min_turn_speed').value)
                else:
                    forward_speed = 0
            elif (linear_x < 0.0
                    and not bool(self.get_parameter('nav_allow_reverse').value)
                    and abs(omega) >= angular_deadband):
                # TEB 输出 vx<0 但被禁后退拦截（linear_x_for_calc=0 → forward_speed=0）。
                # 如果同时给了 omega（典型掉头场景：轮椅朝向与全局路径相反，路径在
                # 轮椅后方，TEB 想倒退+转向来对齐路径），按 min_turn_speed 输出让轮椅
                # 能原地转弯对齐路径。否则 direction_angle 跟着 omega 变化但 speed=0，
                # 下位机维持电机力矩却不驱动 → 只听到电流声轮椅不动。
                forward_speed = int(
                    self.get_parameter('nav_min_turn_speed').value)

            # === direction_angle：lookahead 优先，fallback 到 angular.z 前馈 ===
            use_lookahead = bool(
                self.get_parameter('nav_use_local_plan_lookahead').value
            )
            local_plan_timeout = float(
                self.get_parameter('nav_local_plan_timeout_sec').value
            )
            local_plan_fresh = (
                self._latest_local_plan is not None
                and (now - self._latest_local_plan_sec) <= local_plan_timeout
                and len(self._latest_local_plan.poses) > 0
                and self._latest_local_plan.header.frame_id == 'base_link'
            )

            if use_lookahead and local_plan_fresh:
                yaw_rel_raw = extract_lookahead_bearing(
                    self._latest_local_plan,
                    float(self.get_parameter('nav_lookahead_distance_m').value),
                )
                if yaw_rel_raw is None:
                    # lookahead 提取失败 → fallback
                    direction_angle = compute_direction_angle(
                        current_angle=current_angle,
                        omega_rad_s=omega,
                        gain_deg_per_rad_per_sec=float(
                            self.get_parameter('lead_gain_deg_per_rad_per_sec').value),
                        max_lead_deg=float(self.get_parameter('max_lead_deg').value),
                        heading_sign=int(self.get_parameter('heading_sign').value),
                    )
                else:
                    # LPF（spec § 3）
                    alpha = float(self.get_parameter('nav_lookahead_lpf_alpha').value)
                    self._lookahead_yaw_filtered = (
                        alpha * yaw_rel_raw
                        + (1.0 - alpha) * self._lookahead_yaw_filtered
                    )
                    steering_deadband = math.radians(float(
                        self.get_parameter('nav_steering_deadband_deg').value
                    ))
                    if abs(self._lookahead_yaw_filtered) < steering_deadband:
                        self._lookahead_yaw_filtered = 0.0
                    direction_angle = wrap_deg(
                        current_angle + math.degrees(self._lookahead_yaw_filtered)
                    )
            else:
                # fallback：angular.z 前馈（既有逻辑）
                if use_lookahead and self._latest_local_plan is not None:
                    if self._latest_local_plan.header.frame_id != 'base_link':
                        self.get_logger().error(
                            f"/local_plan frame_id 不是 base_link: "
                            f"{self._latest_local_plan.header.frame_id}，"
                            f"fallback 到 angular.z 前馈",
                            throttle_duration_sec=2.0,
                        )
                direction_angle = compute_direction_angle(
                    current_angle=current_angle,
                    omega_rad_s=omega,
                    gain_deg_per_rad_per_sec=float(
                        self.get_parameter('lead_gain_deg_per_rad_per_sec').value),
                    max_lead_deg=float(self.get_parameter('max_lead_deg').value),
                    heading_sign=int(self.get_parameter('heading_sign').value),
                )

            # 直行段锁定 HWT 绝对航向；TEB 明确要求转弯时自动释放。
            direction_angle = self._nav_heading_hold.apply(
                planned_direction_deg=float(direction_angle),
                current_heading_deg=float(current_angle),
                linear_x_mps=float(linear_x),
                omega_rad_s=float(omega),
                moving_deadband_mps=float(
                    self.get_parameter('nav_linear_deadband_mps').value),
            )

        # === 限速器：slew rate + 方向跳变保护（spec § 3）===
        dt = now - self._last_tick_now_sec
        if self._last_tick_now_sec == 0.0:
            dt = 0.0  # 首帧 dt=0
        self._last_tick_now_sec = now
        # === 模式分流（spec § 3.3）===
        # Nav2Protector 负责自主导航特有的事件/阶跃保护；它不能替代通用
        # ChassisSlewLimiter。所有最终指令仍必须经过通用限速器，否则 Nav2
        # 的 forward_speed 可从 0 单帧跳到死区补偿值，产生新的电流冲击。
        use_nav2_protector = (
            self._is_nav_control_ready(now)
            and not self._should_use_eeg_override(now)
            and self._nav2_protector_enabled
        )

        if use_nav2_protector:
            try:
                ctx = self._build_frame_ctx(
                    direction_deg=float(direction_angle),
                    linear_x=linear_x,
                    omega=omega,
                    now=now,
                )
                d_out, c_out, s_out, status = self._nav2_protector.apply(
                    direction_deg=float(direction_angle),
                    current_int=int(current_angle),
                    speed_float=float(forward_speed),
                    frame_ctx=ctx,
                )
                direction_angle = float(d_out)
                current_angle = int(c_out)
                forward_speed = float(s_out)
                self._publish_protection_status(status, now)
            except Exception as e:
                self.get_logger().error(
                    f'Nav2Protector 故障，跳过专用保护并保留通用限速: {e}',
                    throttle_duration_sec=1.0,
                )
                self._publish_protection_status('PROTECTOR_FAULT', now)
        else:
            self._publish_protection_status('EEG_PATH', now)

        # 最终统一限速：保证 Nav2、EEG、force-stop 和故障回退路径都不会绕过
        # forward_speed 斜率限制及 direction PID=0 跳变保护。
        direction_angle, current_angle, forward_speed = self._slew_limiter.apply(
            direction_target=float(direction_angle),
            current_target=int(current_angle),
            speed_target=int(forward_speed),
            dt=dt,
            now=now,
        )

        # GPS失锁是自主导航硬安全门控，不能经过软件/下位机的缓降过程。
        # 脑控是本地人工控制链路，不依赖 GNSS；否则室内无定位时即使 EEG
        # 已接管也会在这里被二次清零，表现为模式已激活但电机完全不驱动。
        # 同时清空软件斜坡状态，恢复定位后必须从零重新平滑起步。
        if not gps_ready and not self._eeg_override_active:
            current_angle = (
                wrap_deg(self._latest_heading_deg)
                if self._latest_heading_deg is not None else 0
            )
            direction_angle = current_angle
            forward_speed = 0
            self._ramp_linear_x = 0.0
            self._ramp_omega = 0.0
            self._slew_limiter._current_speed = 0
            self._slew_limiter._last_direction = float(direction_angle)

        # === 编码 + 写串口 ===
        frame = encode_frame_bytes(direction_angle, current_angle, forward_speed)
        self._log_serial_frame_if_needed(
            now, frame, direction_angle, current_angle, forward_speed, linear_x, omega
        )
        self._write_serial(frame)

        # === 发布 /odom + odom→base_link TF（dead-reckon）===
        self._publish_odom_tf()

    def _is_cmd_vel_stale(self, now: float) -> bool:
        """cmd_vel_safe 从未收到 或 超过 cmd_vel_timeout_sec → 视为失联。

        失联时强制 forward_speed=0（轮椅原地等指令），direction_angle 仍跟 heading。
        """
        if self._latest_cmd_vel is None:
            return True
        timeout = float(self.get_parameter('cmd_vel_timeout_sec').value)
        return (now - self._latest_cmd_vel_sec) > timeout

    def _is_heading_stale(self, now: float) -> bool:
        """heading_imu 从未收到 或 超过 heading_timeout_sec → 视为失联。

        失联时强制 forward_speed=0 防轮椅瞎走。
        """
        if self._latest_heading_deg is None:
            return True
        timeout = float(self.get_parameter('heading_timeout_sec').value)
        return (now - self._latest_heading_sec) > timeout

    def _is_nav_control_ready(self, now: float) -> bool:
        """真实自主导航底盘执行门控。

        HWT906P 只提供 current_angle 反馈。底盘运动必须由有效 TEB FollowPath
        goal 对应的 /cmd_vel_safe 触发；path_feeder 通过 /nav_control_active 心跳
        表示该条件成立。心跳失联则立即回零。

        cmd_vel 旁路（2026-07-11）：TEB 持续输出非零 cmd_vel_safe 时强制返回 True。
        原因：progress_checker 死锁——轮椅电机死区导致 30 秒无 progress →
        controller_server abort → path_feeder 发 nav_control_active=False →
        chassis 输出零速 → 轮椅永远不动 → 继续 abort。旁路让 chassis 在 TEB
        主动输出时持续尝试驱动底盘，解开死锁，便于调试电机死区。
        """
        if not bool(self.get_parameter('nav_chassis_control_enabled').value):
            return False
        if (self._latest_cmd_vel is not None
                and not self._is_cmd_vel_stale(now)
                and (abs(self._latest_cmd_vel.linear.x) > 0.01
                     or abs(self._latest_cmd_vel.angular.z) > 0.05)):
            return True
        if not self._nav_control_active:
            return False
        timeout = float(self.get_parameter('nav_control_timeout_sec').value)
        return (now - self._latest_nav_control_sec) <= timeout

    def _eeg_frame_param(self, action: str) -> tuple[int, int, int]:
        """读取脑控动作的实物标定帧。"""
        return (
            int(self.get_parameter(f'eeg_{action}_direction_angle').value),
            int(self.get_parameter(f'eeg_{action}_current_angle').value),
            int(self.get_parameter(f'eeg_{action}_speed').value),
        )

    def _log_serial_frame_if_needed(
        self,
        now: float,
        frame: bytes,
        direction_angle: int,
        current_angle: int,
        forward_speed: int,
        linear_x: float,
        omega: float,
    ):
        """低频打印真实串口帧，方便实物联调确认协议格式。"""
        if not self._eeg_override_active and forward_speed == 0:
            return
        if (now - self._last_serial_frame_log_sec) < 1.0:
            return
        self._last_serial_frame_log_sec = now
        self.get_logger().info(
            "serial_frame=%r eeg=%s fields=(%d,%d,%d) cmd=(%.3f,%.3f)"
            % (
                frame,
                self._eeg_override_active,
                direction_angle,
                current_angle,
                forward_speed,
                linear_x,
                omega,
            )
        )

    def _write_serial(self, frame: bytes):
        """写串口，捕获异常不崩，下位机 watchdog 自动停。

        运行中 USB 拔了 / 内核错误 → 仅记 ERROR 日志，
        下次 _write_serial 仍会尝试（is_open=False 时自动 _open_serial 重开）。

        重要：异常时不把 _serial 置 None，否则 _shutdown_safe 连发循环
        剩余帧会被 'if self._serial is None: return' 静默吞掉，
        违反规格"所有故障路径下最后的串口输出倾向于零速度"原则。
        """
        now = self._now_sec()
        if self._serial is None:
            reopen_interval = float(
                self.get_parameter('serial_reopen_interval_sec').value
            )
            if (now - self._last_serial_open_attempt_sec) >= reopen_interval:
                self._open_serial()
            if self._serial is None:
                return
        try:
            if not self._serial.is_open:
                if not self._open_serial() or self._serial is None:
                    return
            self._serial.write(frame)
        except Exception as e:
            self.get_logger().error(
                f"串口写入失败（下位机 watchdog 会自动停）: {e}"
            )
            # 关闭串口（让下次 _write_serial 触发 _open_serial 重连），
            # 但不 null out _serial，让 shutdown 连发仍能尝试重发
            try:
                if hasattr(self._serial, 'is_open') and self._serial.is_open:
                    self._serial.close()
            except Exception:
                pass

    def _shutdown_safe(self):
        """关闭前连发 shutdown_zero_repeat 帧零速（防丢包）+ 关闭串口。

        在 main() 的 finally 块中调用。

        边缘场景：若 USB 已物理拔出，_write_serial 会因 _open_serial 抛异常
        而让本次连发帧全部丢失。此时软件层无法送达零速，依赖下位机 watchdog
        兜底（报文中断自动停）。这是已知限制，无法在软件层完全规避。
        """
        repeat = int(self.get_parameter('shutdown_zero_repeat').value)
        # 用最后已知 heading 填 current_angle，方向字段也置为 current
        # （轮椅直走方向 → 直走但零速）
        if self._latest_heading_deg is not None:
            current_angle = wrap_deg(self._latest_heading_deg)
        else:
            current_angle = 0

        for _ in range(repeat):
            self._write_serial(encode_frame_bytes(current_angle, current_angle, 0))
        self.get_logger().info(f"关闭前已连发 {repeat} 帧零速")

        if self._serial is not None and self._serial.is_open:
            try:
                self._serial.close()
            except Exception as e:
                self.get_logger().warn(f"关闭串口异常: {e}")

    def _publish_odom_tf(self):
        """dead-reckon 更新位置 + 发布 /odom 和 odom→base_link TF。

        下位机不上报编码器，靠 cmd_vel + heading 推算。
        短距离可用，长距离会漂移（已知限制，规格第 10 节）。
        """
        now = self._now_sec()
        if self._last_tick_sec == 0.0:
            self._last_tick_sec = now
            dt = 0.0
        else:
            dt = min(now - self._last_tick_sec, 0.2)  # 上限防大跳
            self._last_tick_sec = now

        # 用 _tick 选出的速度推算（mux 已处理 force_stop / EEG / Nav2 / 超时零速）
        if not self._is_heading_stale(now) and self._latest_heading_deg is not None:
            vx = self._selected_linear_x
            heading_val = self._latest_heading_deg if self._latest_heading_deg is not None else 0.0
            heading_compass_rad = math.radians(heading_val)
            # compass: 0=N(+y), π/2=E(+x)
            self._dead_reckon_x_m += vx * math.sin(heading_compass_rad) * dt
            self._dead_reckon_y_m += vx * math.cos(heading_compass_rad) * dt

        # compass heading → ROS yaw（REP-103）：yaw_ros = π/2 - heading_compass
        if self._latest_heading_deg is not None:
            heading_compass_rad = math.radians(self._latest_heading_deg)
            yaw_ros = math.pi / 2 - heading_compass_rad
        else:
            yaw_ros = 0.0
        half_yaw = yaw_ros / 2.0
        qz = float(math.sin(half_yaw))
        qw = float(math.cos(half_yaw))

        stamp = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = float(self._dead_reckon_x_m)
        odom.pose.pose.position.y = float(self._dead_reckon_y_m)
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = float(self._selected_linear_x)
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = float(self._selected_omega)
        odom.pose.covariance[0] = 0.25
        odom.pose.covariance[7] = 0.25
        odom.pose.covariance[35] = 0.10
        odom.twist.covariance[0] = 0.10
        odom.twist.covariance[7] = 0.50
        odom.twist.covariance[35] = 0.20
        self._odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = float(self._dead_reckon_x_m)
        tf.transform.translation.y = float(self._dead_reckon_y_m)
        tf.transform.translation.z = 0.0
        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = ChassisSerialNode()
    signal.signal(signal.SIGINT, lambda *_: rclpy.shutdown())
    signal.signal(signal.SIGTERM, lambda *_: rclpy.shutdown())
    try:
        rclpy.spin(node)
    except Exception:
        pass
    finally:
        node._shutdown_safe()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
