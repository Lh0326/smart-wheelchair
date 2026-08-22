"""头部姿态计算器（极简版）：四元数 → pitch/roll 欧拉角。

实现：标准 ZYX 欧拉角分解公式。
- pitch（绕 Y 轴）= 低头/仰头
- roll（绕 X 轴）= 左歪/右歪

维特 IMU 默认体坐标系：X+ 右、Y+ 前、Z+ 上（ENU 右手系）。
默认 sign_flip 适配维特坐标系（X+ 右 → 用户左歪对应 roll+）。

简化原则：
- 不用 PCA、不用 axis_remap、不用向量拟合
- 只有 2 个调试开关：pitch 翻转、roll 翻转
- 如果 IMU 物理安装让 pitch/roll 互换，调换公式中的 q1/q2 即可（提供 swap_axes 开关）
"""
import math
from typing import Tuple


# 文档 §7.1 默认参数
ALPHA_LPF = 0.3
BETA_DIFF = 0.5


class HeadPoseCalculator:
    """四元数 → pitch/roll 欧拉角（极简版）。

    公式（标准 ZYX 分解）：
        pitch = asin(clamp(2(q0·q2 − q1·q3), -1, 1)) · 180/π
        roll  = atan2(2(q0·q1 + q2·q3), 1 − 2(q1² + q2²)) · 180/π

    调试开关：
    - sign_flip['pitch'] / sign_flip['roll']：±1，翻转符号
    - swap_axes：False/True，交换 pitch 和 roll（IMU 物理旋转 90° 时用）
    """

    def __init__(self, alpha: float = ALPHA_LPF, beta: float = BETA_DIFF,
                 sign_flip: dict = None, swap_axes: bool = False,
                 pitch_offset_deg: float = 0.0, roll_offset_deg: float = 0.0):
        """
        Args:
            alpha: LPF 系数（默认 0.3）
            beta: 滤波微分系数（默认 0.5）
            sign_flip: {'pitch': ±1, 'roll': ±1}，符号反转
            swap_axes: True 表示 IMU 物理旋转 90°，pitch 和 roll 互换
            pitch_offset_deg: pitch 角度偏移（度），用于微调正前方位置
            roll_offset_deg: roll 角度偏移（度），用于微调正前方位置
        """
        self.alpha = alpha
        self.beta = beta
        # 维特 IMU 默认约定：pitch+ 是低头（用户体感），roll+ 是左歪
        # 用户左歪时 IMU 输出 roll 是负的（因为维特 X+ 朝右），所以 roll 默认 -1
        self.sign_flip = sign_flip if sign_flip is not None else {
            'pitch': +1, 'roll': -1,
        }
        self.swap_axes = swap_axes
        self.pitch_offset_deg = pitch_offset_deg
        self.roll_offset_deg = roll_offset_deg

        # 滤波状态
        self._pitch_f = 0.0
        self._roll_f = 0.0
        self._omega_pitch = 0.0
        self._omega_roll = 0.0
        self._prev_pitch_f = None
        self._prev_roll_f = None
        self._prev_t_ms = None
        self._first_frame = True

        # 兼容旧 API（imu_live_view.py 可能引用）
        self.axis_remap = {'pitch_axis': 'gy', 'roll_axis': 'gx'}
        self.pitch_vector = None
        self.roll_vector = None

    def quaternion_to_tilt(self, q0: float, q1: float, q2: float,
                           q3: float) -> Tuple[float, float]:
        """四元数 → yaw-不变的 pitch/roll（度）。

        标准 ZYX 欧拉角分解。pitch/roll 在 |角度| < 90° 时严格 yaw-不变。
        """
        # 标准 ZYX 公式（pitch 绕 Y 轴，roll 绕 X 轴）
        sin_pitch = 2 * (q0*q2 - q1*q3)
        sin_pitch = max(-1.0, min(1.0, sin_pitch))  # clamp 防止 asin 域错误
        pitch = math.degrees(math.asin(sin_pitch))

        roll = math.degrees(math.atan2(
            2 * (q0*q1 + q2*q3),
            1 - 2 * (q1*q1 + q2*q2),
        ))

        # 应用调试开关
        if self.swap_axes:
            pitch, roll = roll, pitch
        pitch = pitch * self.sign_flip['pitch'] + self.pitch_offset_deg
        roll = roll * self.sign_flip['roll'] + self.roll_offset_deg

        return pitch, roll

    def update(self, q0: float, q1: float, q2: float, q3: float,
               t_ms: float) -> Tuple[float, float, float, float]:
        """LPF + 滤波微分。

        Returns:
            (pitch_filtered, roll_filtered, omega_pitch, omega_roll)
            pitch/roll 单位度；omega 单位度/秒
        """
        pitch_raw, roll_raw = self.quaternion_to_tilt(q0, q1, q2, q3)

        # LPF（首帧直通，避免 0 初始值污染）
        if self._first_frame:
            self._pitch_f = pitch_raw
            self._roll_f = roll_raw
            self._first_frame = False
        else:
            self._pitch_f = self.alpha * pitch_raw + (1 - self.alpha) * self._pitch_f
            self._roll_f = self.alpha * roll_raw + (1 - self.alpha) * self._roll_f

        # 滤波微分
        if (self._prev_pitch_f is not None and
                self._prev_roll_f is not None and
                self._prev_t_ms is not None):
            dt_s = (t_ms - self._prev_t_ms) / 1000.0
            if dt_s > 1e-6:
                omega_p_raw = (self._pitch_f - self._prev_pitch_f) / dt_s
                omega_r_raw = (self._roll_f - self._prev_roll_f) / dt_s
                self._omega_pitch = (self.beta * omega_p_raw +
                                     (1 - self.beta) * self._omega_pitch)
                self._omega_roll = (self.beta * omega_r_raw +
                                    (1 - self.beta) * self._omega_roll)

        self._prev_pitch_f = self._pitch_f
        self._prev_roll_f = self._roll_f
        self._prev_t_ms = t_ms

        return self._pitch_f, self._roll_f, self._omega_pitch, self._omega_roll

    def reset(self) -> None:
        """重置滤波状态。"""
        self._pitch_f = 0.0
        self._roll_f = 0.0
        self._omega_pitch = 0.0
        self._omega_roll = 0.0
        self._prev_pitch_f = None
        self._prev_roll_f = None
        self._prev_t_ms = None
        self._first_frame = True

    # ===== 兼容旧 API（被 imu_live_view.py 调用）=====
    def set_pca_vectors(self, pitch_vector, roll_vector, pitch_sign, roll_sign):
        """兼容旧 API：本极简版不支持 PCA，调用时直接更新 sign_flip。"""
        self.sign_flip['pitch'] = int(pitch_sign)
        self.sign_flip['roll'] = int(roll_sign)

    def save_config(self, path):
        """保存配置到 JSON。"""
        import json
        config = {
            'version': '2.0',
            'sign_flip': self.sign_flip,
            'swap_axes': self.swap_axes,
            'pitch_offset_deg': self.pitch_offset_deg,
            'roll_offset_deg': self.roll_offset_deg,
            'alpha': self.alpha,
            'beta': self.beta,
        }
        with open(path, 'w') as f:
            json.dump(config, f, indent=2, ensure_ascii=False)

    @classmethod
    def load_config(cls, path):
        """从 JSON 加载配置。文件不存在时返回默认配置。"""
        import json, os
        if not os.path.exists(path):
            return cls()
        try:
            with open(path) as f:
                config = json.load(f)
            return cls(
                alpha=config.get('alpha', ALPHA_LPF),
                beta=config.get('beta', BETA_DIFF),
                sign_flip=config.get('sign_flip'),
                swap_axes=config.get('swap_axes', False),
                pitch_offset_deg=config.get('pitch_offset_deg', 0.0),
                roll_offset_deg=config.get('roll_offset_deg', 0.0),
            )
        except Exception:
            return cls()  # 配置损坏时回退默认
