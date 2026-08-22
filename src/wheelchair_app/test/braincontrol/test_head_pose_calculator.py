"""head_pose_calculator 单元测试（极简版）。"""
import math
import json
import tempfile
import os
import pytest
from wheelchair_app.braincontrol.head_pose_calculator import HeadPoseCalculator


def quat_from_euler(pitch_deg, roll_deg, yaw_deg=0):
    """构造四元数 [q0,q1,q2,q3] from 欧拉角（度，ZYX 顺序）。"""
    p = math.radians(pitch_deg)
    r = math.radians(roll_deg)
    y = math.radians(yaw_deg)
    cp, sp = math.cos(p/2), math.sin(p/2)
    cr, sr = math.cos(r/2), math.sin(r/2)
    cy, sy = math.cos(y/2), math.sin(y/2)
    q0 = cr*cp*cy + sr*sp*sy
    q1 = sr*cp*cy - cr*sp*sy
    q2 = cr*sp*cy + sr*cp*sy
    q3 = cr*cp*sy - sr*sp*cy
    return [q0, q1, q2, q3]


# ===== L2 姿态解算（基础） =====

def test_identity_quaternion_gives_zero_tilt():
    calc = HeadPoseCalculator()
    p, r = calc.quaternion_to_tilt(1.0, 0.0, 0.0, 0.0)
    assert abs(p) < 0.01
    assert abs(r) < 0.01


def test_pitch_only_positive():
    """pitch=+30° → pitch 输出 +30°（默认 sign_flip['pitch']=+1）。"""
    calc = HeadPoseCalculator()
    q = quat_from_euler(pitch_deg=30, roll_deg=0)
    p, r = calc.quaternion_to_tilt(*q)
    assert abs(p - 30) < 0.5
    assert abs(r) < 0.5


def test_pitch_only_negative():
    """pitch=-25° → pitch 输出 -25°。"""
    calc = HeadPoseCalculator()
    q = quat_from_euler(pitch_deg=-25, roll_deg=0)
    p, r = calc.quaternion_to_tilt(*q)
    assert abs(p - (-25)) < 0.5
    assert abs(r) < 0.5


def test_roll_only():
    """roll=20° → roll 输出 -20°（默认 sign_flip['roll']=-1，因为维特 X+ 朝右）。"""
    calc = HeadPoseCalculator()
    q = quat_from_euler(pitch_deg=0, roll_deg=20)
    p, r = calc.quaternion_to_tilt(*q)
    assert abs(p) < 0.5
    assert abs(r - (-20)) < 0.5


def test_pitch_and_roll_combined():
    """pitch=15° + roll=-10° → pitch=15, roll=+10（符号反转）。"""
    calc = HeadPoseCalculator()
    q = quat_from_euler(pitch_deg=15, roll_deg=-10)
    p, r = calc.quaternion_to_tilt(*q)
    assert abs(p - 15) < 0.5
    assert abs(r - 10) < 0.5  # -10 反转为 +10


# ===== Yaw 不变性 =====

def test_yaw_does_not_affect_pitch():
    """Yaw 旋转不改变 pitch。"""
    calc = HeadPoseCalculator()
    q0 = quat_from_euler(pitch_deg=20, roll_deg=0, yaw_deg=0)
    p_base, _ = calc.quaternion_to_tilt(*q0)
    for yaw in [30, 60, 90, 135, 180, -45, -90]:
        q = quat_from_euler(pitch_deg=20, roll_deg=0, yaw_deg=yaw)
        p, _ = calc.quaternion_to_tilt(*q)
        assert abs(p - p_base) < 1.5, f"yaw={yaw}: pitch 变化 {p - p_base:.2f}°"


def test_yaw_does_not_affect_roll():
    """Yaw 旋转不改变 roll。"""
    calc = HeadPoseCalculator()
    q0 = quat_from_euler(pitch_deg=0, roll_deg=15, yaw_deg=0)
    _, r_base = calc.quaternion_to_tilt(*q0)
    for yaw in [45, 90, 180, -90]:
        q = quat_from_euler(pitch_deg=0, roll_deg=15, yaw_deg=yaw)
        _, r = calc.quaternion_to_tilt(*q)
        assert abs(r - r_base) < 1.5, f"yaw={yaw}: roll 变化 {r - r_base:.2f}°"


# ===== 调试开关 =====

def test_sign_flip_pitch():
    """sign_flip['pitch']=-1 时 pitch 输出反向。"""
    calc = HeadPoseCalculator(sign_flip={'pitch': -1, 'roll': +1})
    q = quat_from_euler(pitch_deg=30, roll_deg=0)
    p, r = calc.quaternion_to_tilt(*q)
    assert abs(p - (-30)) < 0.5


def test_sign_flip_roll():
    """sign_flip['roll']=+1 时 roll 输出正向（默认是 -1）。"""
    calc = HeadPoseCalculator(sign_flip={'pitch': +1, 'roll': +1})
    q = quat_from_euler(pitch_deg=0, roll_deg=20)
    p, r = calc.quaternion_to_tilt(*q)
    assert abs(r - 20) < 0.5


def test_swap_axes():
    """swap_axes=True 时 pitch 和 roll 互换。"""
    calc = HeadPoseCalculator(swap_axes=True)
    q = quat_from_euler(pitch_deg=30, roll_deg=0)
    p, r = calc.quaternion_to_tilt(*q)
    # 原本 pitch=30, roll=0；互换后 pitch=0, roll=-30
    assert abs(p) < 0.5
    assert abs(r - (-30)) < 0.5


# ===== L4 LPF + 微分 =====

def test_lpf_first_frame_passthrough():
    """LPF 第一帧直通，不混入 0 初始值。"""
    calc = HeadPoseCalculator()
    q = quat_from_euler(pitch_deg=20, roll_deg=0)
    p, r, _, _ = calc.update(*q, t_ms=0)
    assert abs(p - 20) < 1.0  # 第一帧等于原始值


def test_lpf_smooths_step_input():
    """阶跃输入被 LPF 平滑收敛。"""
    calc = HeadPoseCalculator(alpha=0.3)
    q = quat_from_euler(pitch_deg=30, roll_deg=0)
    t_ms = 0
    for _ in range(50):
        t_ms += 10
        p, r, _, _ = calc.update(*q, t_ms)
    assert abs(p - 30) < 2.0


def test_omega_computed_during_motion():
    """动作时角速度非零。"""
    calc = HeadPoseCalculator(alpha=1.0)  # 关闭 LPF
    q1 = quat_from_euler(pitch_deg=0, roll_deg=0)
    calc.update(*q1, t_ms=0)
    q2 = quat_from_euler(pitch_deg=30, roll_deg=0)
    p, r, omega_p, omega_r = calc.update(*q2, t_ms=10)
    assert omega_p > 500  # 30°/0.01s = 3000°/s，被 β=0.5 平滑到 ~1500


def test_reset_clears_state():
    calc = HeadPoseCalculator()
    for i in range(10):
        q = quat_from_euler(pitch_deg=i*2, roll_deg=0)
        calc.update(*q, t_ms=i*10)
    calc.reset()
    q = quat_from_euler(pitch_deg=20, roll_deg=0)
    p, r, omega_p, omega_r = calc.update(*q, t_ms=1000)
    assert abs(p - 20) < 1.0  # LPF 第一帧等于原始值
    assert omega_p == 0.0  # reset 后没有上一帧，omega=0


# ===== 配置持久化 =====

def test_load_save_config_roundtrip():
    """保存后加载，配置一致。"""
    calc = HeadPoseCalculator(
        sign_flip={'pitch': -1, 'roll': +1},
        swap_axes=True,
        alpha=0.4, beta=0.6,
    )
    with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
        path = f.name
    try:
        calc.save_config(path)
        loaded = HeadPoseCalculator.load_config(path)
        assert loaded.sign_flip == {'pitch': -1, 'roll': +1}
        assert loaded.swap_axes is True
        assert loaded.alpha == 0.4
        assert loaded.beta == 0.6
    finally:
        os.unlink(path)


def test_load_config_missing_file_returns_default():
    """文件不存在时返回默认配置。"""
    calc = HeadPoseCalculator.load_config('/nonexistent/path/imu_config.json')
    assert calc.sign_flip == {'pitch': +1, 'roll': -1}
    assert calc.swap_axes is False


def test_load_config_corrupt_file_returns_default():
    """配置文件损坏时回退默认。"""
    with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
        f.write('not a valid json {{{')
        path = f.name
    try:
        calc = HeadPoseCalculator.load_config(path)
        assert calc.sign_flip == {'pitch': +1, 'roll': -1}
    finally:
        os.unlink(path)


def test_set_pca_vectors_compat_updates_sign_flip():
    """set_pca_vectors 兼容旧 API，实际只更新 sign_flip。"""
    calc = HeadPoseCalculator()
    calc.set_pca_vectors([1, 0, 0], [0, 1, 0], -1, +1)
    assert calc.sign_flip['pitch'] == -1
    assert calc.sign_flip['roll'] == +1
