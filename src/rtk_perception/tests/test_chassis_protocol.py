"""chassis_serial_node 纯函数 Layer 1 单元测试。

不初始化 rclpy，纯函数 in/out 验证。秒级跑完。
"""
import math

import pytest


def test_wrap_deg_normal():
    """正常值原样返回（取整）。"""
    from rtk_perception.chassis_serial_node import wrap_deg
    assert wrap_deg(45.4) == 45
    assert wrap_deg(45.6) == 46
    assert wrap_deg(-45.6) == -46


def test_wrap_deg_boundaries():
    """±180 边界值（简单公式把 +180 系列归一化为 -180，方向等价）。"""
    from rtk_perception.chassis_serial_node import wrap_deg
    assert wrap_deg(180.0) == -180  # +180 与 -180 同方向
    assert wrap_deg(-180.0) == -180


def test_wrap_deg_overflow():
    """超范围值归一化到 [-180, 180]。"""
    from rtk_perception.chassis_serial_node import wrap_deg
    assert wrap_deg(360.0) == 0
    assert wrap_deg(540.0) == -180  # 540 = 360 + 180，归一化为 -180
    assert wrap_deg(-270.0) == 90
    assert wrap_deg(720.0) == 0
    assert wrap_deg(-540.0) == -180


def test_compute_forward_speed_basic():
    """max_speed_mps=1.5 时 0.6 m/s → 400。"""
    from rtk_perception.chassis_serial_node import compute_forward_speed
    assert compute_forward_speed(0.6, max_speed_mps=1.5) == 400


def test_compute_forward_speed_full_scale():
    """刚好 1.5 m/s → 1000。"""
    from rtk_perception.chassis_serial_node import compute_forward_speed
    assert compute_forward_speed(1.5, max_speed_mps=1.5) == 1000


def test_compute_forward_speed_reverse():
    """负速度（后退）→ 负值。"""
    from rtk_perception.chassis_serial_node import compute_forward_speed
    assert compute_forward_speed(-0.3, max_speed_mps=1.5) == -200


def test_compute_forward_speed_clip_high():
    """超上限 clip 到 1000。"""
    from rtk_perception.chassis_serial_node import compute_forward_speed
    assert compute_forward_speed(3.0, max_speed_mps=1.5) == 1000


def test_compute_forward_speed_clip_low():
    """低于下限 clip 到 -1000。"""
    from rtk_perception.chassis_serial_node import compute_forward_speed
    assert compute_forward_speed(-3.0, max_speed_mps=1.5) == -1000


def test_compute_forward_speed_nan():
    """NaN 输入返回 0（防 TEB 偶发 NaN）。"""
    from rtk_perception.chassis_serial_node import compute_forward_speed
    assert compute_forward_speed(float('nan'), max_speed_mps=1.5) == 0


def test_compute_nav_forward_speed_applies_forward_floor():
    """自主导航小速度超过阈值时应用真实电机死区补偿。"""
    from rtk_perception.chassis_serial_node import compute_nav_forward_speed

    assert compute_nav_forward_speed(
        linear_x_mps=0.06,
        omega_rad_s=0.0,
        max_speed_mps=1.5,
        min_forward_speed=180,
        min_turn_speed=160,
        linear_deadband_mps=0.01,
        angular_deadband_rad_s=0.05,
        floor_linear_threshold_mps=0.05,
    ) == 180


def test_compute_nav_forward_speed_keeps_zero_zero():
    """无线速度且无角速度时必须保持零速。"""
    from rtk_perception.chassis_serial_node import compute_nav_forward_speed

    assert compute_nav_forward_speed(
        linear_x_mps=0.0,
        omega_rad_s=0.0,
        max_speed_mps=1.5,
        min_forward_speed=180,
        min_turn_speed=160,
        linear_deadband_mps=0.01,
        angular_deadband_rad_s=0.05,
        floor_linear_threshold_mps=0.05,
    ) == 0


def test_compute_nav_forward_speed_turn_only_uses_turn_floor():
    """TEB 低线速/原地转向时给实物底盘一个低速差速触发量。"""
    from rtk_perception.chassis_serial_node import compute_nav_forward_speed

    assert compute_nav_forward_speed(
        linear_x_mps=0.0,
        omega_rad_s=0.3,
        max_speed_mps=1.5,
        min_forward_speed=180,
        min_turn_speed=160,
        linear_deadband_mps=0.01,
        angular_deadband_rad_s=0.05,
        floor_linear_threshold_mps=0.05,
    ) == 160


def test_compute_direction_angle_straight():
    """ω=0 时 direction_angle = current_angle。"""
    from rtk_perception.chassis_serial_node import compute_direction_angle
    assert compute_direction_angle(
        current_angle=45, omega_rad_s=0.0,
        gain_deg_per_rad_per_sec=30.0, max_lead_deg=60.0,
        heading_sign=-1,
    ) == 45


def test_compute_direction_angle_left_turn():
    """TEB 输出 ω=+0.3 (左转 REP-103)，heading_sign=-1 → direction_angle = 45 - 9 = 36。"""
    from rtk_perception.chassis_serial_node import compute_direction_angle
    assert compute_direction_angle(
        current_angle=45, omega_rad_s=0.3,
        gain_deg_per_rad_per_sec=30.0, max_lead_deg=60.0,
        heading_sign=-1,
    ) == 36


def test_compute_direction_angle_right_turn():
    """TEB 输出 ω=-0.3 (右转)，heading_sign=-1 → direction_angle = 45 + 9 = 54。"""
    from rtk_perception.chassis_serial_node import compute_direction_angle
    assert compute_direction_angle(
        current_angle=45, omega_rad_s=-0.3,
        gain_deg_per_rad_per_sec=30.0, max_lead_deg=60.0,
        heading_sign=-1,
    ) == 54


def test_compute_direction_angle_clip_max_lead():
    """ω 过大 → lead 被 max_lead_deg clip。"""
    from rtk_perception.chassis_serial_node import compute_direction_angle
    # ω=10 * 30 = 300° → clip 到 60°
    result = compute_direction_angle(
        current_angle=45, omega_rad_s=10.0,
        gain_deg_per_rad_per_sec=30.0, max_lead_deg=60.0,
        heading_sign=-1,
    )
    # heading_sign=-1 → -300 clip 到 -60 → 45 - 60 = -15
    assert result == -15


def test_compute_direction_angle_wrap():
    """结果超 ±180 时 wrap。"""
    from rtk_perception.chassis_serial_node import compute_direction_angle
    # current=170 + lead=60 (右转) = 230 → wrap 到 -130
    result = compute_direction_angle(
        current_angle=170, omega_rad_s=-2.0,
        gain_deg_per_rad_per_sec=30.0, max_lead_deg=60.0,
        heading_sign=-1,
    )
    assert result == -130


def test_compute_eeg_directional_fields_forward():
    """脑控前进：使用独立标定帧。"""
    from rtk_perception.chassis_serial_node import compute_eeg_directional_fields

    assert compute_eeg_directional_fields(
        linear_x_mps=0.5,
        omega_rad_s=0.0,
        current_angle=90,
        forward_frame=(90, 90, 700),
        backward_frame=(180, 0, 700),
        left_frame=(0, 90, 300),
        right_frame=(90, 0, 300),
    ) == (90, 90, 700)


def test_compute_eeg_directional_fields_backward():
    """脑控后退：使用独立标定帧。

    direction=180 经 wrap_deg 归一化为 -180（与 +180 同方向，对底盘
    PID 等价）。任务 2 把 FORWARD/BACKWARD 分支改为 wrap(direction+offset)
    后，期望值需匹配归一化结果。
    """
    from rtk_perception.chassis_serial_node import compute_eeg_directional_fields

    assert compute_eeg_directional_fields(
        linear_x_mps=-0.3,
        omega_rad_s=0.0,
        current_angle=90,
        forward_frame=(90, 90, 700),
        backward_frame=(180, 0, 700),
        left_frame=(0, 90, 300),
        right_frame=(90, 0, 300),
    ) == (-180, 0, 700)


def test_compute_eeg_directional_fields_left_right():
    """脑控左右转：右转保持已验证帧，左转用等价负角差帧。"""
    from rtk_perception.chassis_serial_node import compute_eeg_directional_fields

    left = compute_eeg_directional_fields(
        linear_x_mps=0.0,
        omega_rad_s=0.5,
        current_angle=90,
        forward_frame=(90, 90, 700),
        backward_frame=(180, 0, 700),
        left_frame=(0, 90, 300),
        right_frame=(90, 0, 300),
    )
    right = compute_eeg_directional_fields(
        linear_x_mps=0.0,
        omega_rad_s=-0.5,
        current_angle=90,
        forward_frame=(90, 90, 700),
        backward_frame=(180, 0, 700),
        left_frame=(0, 90, 300),
        right_frame=(90, 0, 300),
    )

    assert left == (0, 90, 300)
    assert right == (90, 0, 300)


def test_encode_frame_basic():
    """三字段 → ASCII 字符串，\\n 作为帧头。"""
    from rtk_perception.chassis_serial_node import encode_frame
    assert encode_frame(45, 45, 400) == "\n45,45,400"


def test_encode_frame_negative():
    """负值正确编码。"""
    from rtk_perception.chassis_serial_node import encode_frame
    assert encode_frame(-30, -30, -200) == "\n-30,-30,-200"


def test_encode_frame_to_bytes():
    """encode_frame_bytes 返回 UTF-8 ASCII 字节。"""
    from rtk_perception.chassis_serial_node import encode_frame_bytes
    assert encode_frame_bytes(0, 0, 0) == b"\n0,0,0"


# ===== compute_roll_direction_offset_deg 测试（spec § 7.1）=====

def test_roll_offset_disabled_returns_zero():
    """enabled=False 时任何 roll 都返回 0。"""
    from rtk_perception.chassis_serial_node import compute_roll_direction_offset_deg
    assert compute_roll_direction_offset_deg(
        roll_deg=30.0, enabled=False, gain=0.5, saturation=20.0, polarity=-1
    ) == 0


def test_roll_offset_normal_negative_polarity():
    """roll=10°, gain=0.5, polarity=-1 → -5（左歪→负 direction→左转）。"""
    from rtk_perception.chassis_serial_node import compute_roll_direction_offset_deg
    assert compute_roll_direction_offset_deg(
        roll_deg=10.0, enabled=True, gain=0.5, saturation=20.0, polarity=-1
    ) == -5


def test_roll_offset_normal_positive_polarity():
    """roll=10°, gain=0.5, polarity=+1 → +5。"""
    from rtk_perception.chassis_serial_node import compute_roll_direction_offset_deg
    assert compute_roll_direction_offset_deg(
        roll_deg=10.0, enabled=True, gain=0.5, saturation=20.0, polarity=1
    ) == 5


def test_roll_offset_saturate_negative():
    """roll=100° 极端值，polarity=-1, sat=20 → 饱和到 -20。"""
    from rtk_perception.chassis_serial_node import compute_roll_direction_offset_deg
    assert compute_roll_direction_offset_deg(
        roll_deg=100.0, enabled=True, gain=0.5, saturation=20.0, polarity=-1
    ) == -20


def test_roll_offset_saturate_positive():
    """roll=100°, polarity=+1, sat=20 → 饱和到 +20。"""
    from rtk_perception.chassis_serial_node import compute_roll_direction_offset_deg
    assert compute_roll_direction_offset_deg(
        roll_deg=100.0, enabled=True, gain=0.5, saturation=20.0, polarity=1
    ) == 20


def test_roll_offset_nan_returns_zero():
    """NaN roll → 返回 0（防 IMU 数据异常）。"""
    from rtk_perception.chassis_serial_node import compute_roll_direction_offset_deg
    assert compute_roll_direction_offset_deg(
        roll_deg=float('nan'), enabled=True, gain=0.5, saturation=20.0, polarity=-1
    ) == 0


def test_roll_offset_inf_returns_zero():
    """Inf roll → 返回 0。"""
    from rtk_perception.chassis_serial_node import compute_roll_direction_offset_deg
    assert compute_roll_direction_offset_deg(
        roll_deg=float('inf'), enabled=True, gain=0.5, saturation=20.0, polarity=-1
    ) == 0


def test_roll_offset_sat_zero_returns_zero():
    """saturation=0 → 总返回 0（边界情况兜底）。"""
    from rtk_perception.chassis_serial_node import compute_roll_direction_offset_deg
    assert compute_roll_direction_offset_deg(
        roll_deg=30.0, enabled=True, gain=0.5, saturation=0.0, polarity=-1
    ) == 0


# ===== compute_eeg_directional_fields + roll_offset_deg 测试（spec § 7.1）=====

_FORWARD_FRAME = (0, 0, 700)
_BACKWARD_FRAME = (0, 0, -700)
_LEFT_FRAME = (-90, 0, 300)
_RIGHT_FRAME = (90, 0, 300)


def test_eeg_directional_fields_forward_with_roll_offset():
    """FORWARD 帧 + roll_offset=-15 → direction=-15, current=0, speed=700。"""
    from rtk_perception.chassis_serial_node import compute_eeg_directional_fields
    result = compute_eeg_directional_fields(
        linear_x_mps=0.5, omega_rad_s=0.0, current_angle=0,
        forward_frame=_FORWARD_FRAME, backward_frame=_BACKWARD_FRAME,
        left_frame=_LEFT_FRAME, right_frame=_RIGHT_FRAME,
        roll_offset_deg=-15.0,
    )
    assert result == (-15, 0, 700)


def test_eeg_directional_fields_backward_with_roll_offset():
    """BACKWARD 帧 + roll_offset=+10 → direction=+10, current=0, speed=-700。"""
    from rtk_perception.chassis_serial_node import compute_eeg_directional_fields
    result = compute_eeg_directional_fields(
        linear_x_mps=-0.3, omega_rad_s=0.0, current_angle=0,
        forward_frame=_FORWARD_FRAME, backward_frame=_BACKWARD_FRAME,
        left_frame=_LEFT_FRAME, right_frame=_RIGHT_FRAME,
        roll_offset_deg=10.0,
    )
    assert result == (10, 0, -700)


def test_eeg_directional_fields_left_ignores_roll_offset():
    """LEFT 帧 + roll_offset=-15 → 不消费 roll → (-90, 0, 300)。"""
    from rtk_perception.chassis_serial_node import compute_eeg_directional_fields
    result = compute_eeg_directional_fields(
        linear_x_mps=0.0, omega_rad_s=0.5, current_angle=0,
        forward_frame=_FORWARD_FRAME, backward_frame=_BACKWARD_FRAME,
        left_frame=_LEFT_FRAME, right_frame=_RIGHT_FRAME,
        roll_offset_deg=-15.0,
    )
    assert result == (-90, 0, 300)


def test_eeg_directional_fields_right_ignores_roll_offset():
    """RIGHT 帧 + roll_offset=+15 → 不消费 roll → (90, 0, 300)。"""
    from rtk_perception.chassis_serial_node import compute_eeg_directional_fields
    result = compute_eeg_directional_fields(
        linear_x_mps=0.0, omega_rad_s=-0.5, current_angle=0,
        forward_frame=_FORWARD_FRAME, backward_frame=_BACKWARD_FRAME,
        left_frame=_LEFT_FRAME, right_frame=_RIGHT_FRAME,
        roll_offset_deg=15.0,
    )
    assert result == (90, 0, 300)


def test_eeg_directional_fields_default_roll_offset_is_zero():
    """不传 roll_offset_deg（默认 0）→ 行为与旧版本完全一致。"""
    from rtk_perception.chassis_serial_node import compute_eeg_directional_fields
    result = compute_eeg_directional_fields(
        linear_x_mps=0.5, omega_rad_s=0.0, current_angle=0,
        forward_frame=_FORWARD_FRAME, backward_frame=_BACKWARD_FRAME,
        left_frame=_LEFT_FRAME, right_frame=_RIGHT_FRAME,
    )
    assert result == (0, 0, 700)


# ===== extract_lookahead_yaw 测试（spec § 5.1）=====

from geometry_msgs.msg import Pose, PoseArray, Point
from std_msgs.msg import Header


def _pose_at(x_m: float, yaw_deg: float) -> Pose:
    """构造 base_link 系下 (x, 0, 0) 位置 + yaw 朝向的 Pose。"""
    import math
    p = Pose()
    p.position = Point(x=x_m, y=0.0, z=0.0)
    half = math.radians(yaw_deg) / 2.0
    p.orientation.z = math.sin(half)
    p.orientation.w = math.cos(half)
    return p


def _pose_array(poses: list) -> PoseArray:
    pa = PoseArray()
    pa.header.frame_id = 'base_link'
    pa.poses = poses
    return pa


def test_extract_lookahead_yaw_empty_returns_none():
    """空 PoseArray → None。"""
    from rtk_perception.chassis_serial_node import extract_lookahead_yaw
    assert extract_lookahead_yaw(_pose_array([]), 0.5) is None


def test_extract_lookahead_yaw_single_point_returns_its_yaw():
    """单点路径 → 取该点朝向。"""
    from rtk_perception.chassis_serial_node import extract_lookahead_yaw
    import math
    pa = _pose_array([_pose_at(0.0, 30.0)])
    result = extract_lookahead_yaw(pa, 0.5)
    assert result is not None
    assert math.isclose(result, math.radians(30.0), abs_tol=0.001)


def test_extract_lookahead_yaw_short_path_returns_end():
    """路径总长 < lookahead → 取末点。"""
    from rtk_perception.chassis_serial_node import extract_lookahead_yaw
    import math
    pa = _pose_array([_pose_at(0.0, 0.0), _pose_at(0.2, 15.0)])
    result = extract_lookahead_yaw(pa, 0.5)
    assert result is not None
    assert math.isclose(result, math.radians(15.0), abs_tol=0.001)


def test_extract_lookahead_yaw_exact_distance_returns_that_point():
    """路径总长 = lookahead → 取末点（边界）。"""
    from rtk_perception.chassis_serial_node import extract_lookahead_yaw
    import math
    pa = _pose_array([_pose_at(0.0, 0.0), _pose_at(0.5, 25.0)])
    result = extract_lookahead_yaw(pa, 0.5)
    assert result is not None
    assert math.isclose(result, math.radians(25.0), abs_tol=0.001)


def test_extract_lookahead_yaw_long_path_returns_midpoint():
    """路径总长 > lookahead → 取累积距离到达处的点。"""
    from rtk_perception.chassis_serial_node import extract_lookahead_yaw
    import math
    pa = _pose_array([
        _pose_at(0.0, 0.0), _pose_at(0.3, 10.0), _pose_at(0.6, 20.0),
        _pose_at(0.9, 30.0), _pose_at(1.2, 40.0),
    ])
    result = extract_lookahead_yaw(pa, 0.5)
    assert result is not None
    assert math.radians(10.0) < result < math.radians(20.0)


def test_extract_lookahead_yaw_zero_quaternion_returns_none():
    """pose.orientation 全 0（无效）→ None。"""
    from rtk_perception.chassis_serial_node import extract_lookahead_yaw
    bad = Pose()
    bad.position = Point(x=0.5, y=0.0, z=0.0)
    bad.orientation.z = 0.0
    bad.orientation.w = 0.0
    pa = _pose_array([bad])
    assert extract_lookahead_yaw(pa, 0.5) is None


def test_extract_lookahead_yaw_zero_distance_returns_first():
    """lookahead=0 → 取首点朝向。"""
    from rtk_perception.chassis_serial_node import extract_lookahead_yaw
    import math
    pa = _pose_array([_pose_at(0.0, 45.0), _pose_at(0.5, 90.0)])
    result = extract_lookahead_yaw(pa, 0.0)
    assert result is not None
    assert math.isclose(result, math.radians(45.0), abs_tol=0.001)


def test_extract_lookahead_yaw_negative_distance_returns_none():
    """lookahead<0 → None（参数校验兜底）。"""
    from rtk_perception.chassis_serial_node import extract_lookahead_yaw
    pa = _pose_array([_pose_at(0.5, 30.0)])
    assert extract_lookahead_yaw(pa, -0.1) is None


# ===== compute_nav_speed_with_mode 测试（spec § 5.1）=====

def test_nav_speed_with_mode_full_speed():
    """TEB 全速 0.6 + 中速 700 → 700。"""
    from rtk_perception.chassis_serial_node import compute_nav_speed_with_mode
    assert compute_nav_speed_with_mode(
        teb_linear_x=0.6, teb_max_vel_x=0.6, mode_speed=700
    ) == 700


def test_nav_speed_with_mode_half_speed():
    """TEB 避障减速到 0.3 + 中速 700 → 350。"""
    from rtk_perception.chassis_serial_node import compute_nav_speed_with_mode
    assert compute_nav_speed_with_mode(
        teb_linear_x=0.3, teb_max_vel_x=0.6, mode_speed=700
    ) == 350


def test_nav_speed_with_mode_zero():
    """TEB linear.x=0 → 0。"""
    from rtk_perception.chassis_serial_node import compute_nav_speed_with_mode
    assert compute_nav_speed_with_mode(
        teb_linear_x=0.0, teb_max_vel_x=0.6, mode_speed=700
    ) == 0


def test_nav_speed_with_mode_negative_reverse():
    """TEB linear.x=-0.1（倒车）+ 中速 700 → -117（按比例，round）。"""
    from rtk_perception.chassis_serial_node import compute_nav_speed_with_mode
    assert compute_nav_speed_with_mode(
        teb_linear_x=-0.1, teb_max_vel_x=0.6, mode_speed=700
    ) == -117


def test_nav_speed_with_mode_saturate():
    """TEB linear.x > teb_max_vel_x（饱和）→ clamp 到 mode_speed。"""
    from rtk_perception.chassis_serial_node import compute_nav_speed_with_mode
    assert compute_nav_speed_with_mode(
        teb_linear_x=1.0, teb_max_vel_x=0.6, mode_speed=700
    ) == 700


def test_nav_speed_with_mode_nan():
    """TEB linear.x=NaN → 0。"""
    from rtk_perception.chassis_serial_node import compute_nav_speed_with_mode
    assert compute_nav_speed_with_mode(
        teb_linear_x=float('nan'), teb_max_vel_x=0.6, mode_speed=700
    ) == 0
