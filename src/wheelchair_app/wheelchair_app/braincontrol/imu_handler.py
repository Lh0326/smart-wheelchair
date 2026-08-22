from .control_types import TiltDirection

# 圆形 magnitude 阈值（中等灵敏度档，配合 TiltIndicator MAX_DEG=30/EDGE_GRIP=18）
# ENTER 12°：头部倾斜约 12° 即触发方向命令（原 20°，省力 ~40%）
# EXIT 6°：滞回防抖区间 6°（原 10°），保持锁定感
ENTER_DEG = 12.0
EXIT_DEG = 6.0
ENTER_DEG_SQ = ENTER_DEG * ENTER_DEG   # 平方和比较免 sqrt
EXIT_DEG_SQ = EXIT_DEG * EXIT_DEG


class ImuHandler:
    """IMU 校准 + 圆形 magnitude 滞回 + 方向判定。

    pitch: 低头为正（绕 X 轴），单位度
    roll:  左歪为正（绕 Y 轴），单位度

    判定逻辑（spec § 2.1，灵敏度参数 2026-07-06 调整）：
    - magnitude = sqrt(pitch_rel² + roll_rel²)
    - 进入方向：magnitude >= ENTER_DEG（12°）
    - 退出方向：magnitude < EXIT_DEG（6°）
    - 方向选择：主导轴（|pitch| >= |roll| → 前后，否则 → 左右）
    """

    def __init__(self):
        # 校准状态
        self._pitch_0: float = 0.0
        self._roll_0: float = 0.0
        self._cal_pitch_sum = 0.0
        self._cal_roll_sum = 0.0
        self._cal_n = 0
        self._is_calibrated = False
        # 当前方向
        self._current_dir = TiltDirection.NONE

    @property
    def is_calibrated(self) -> bool:
        return self._is_calibrated

    def feed_calibration(self, pitch: float, roll: float) -> None:
        if self._is_calibrated:
            return  # 已校准，忽略后续 feed（避免污染基线）
        self._cal_pitch_sum += pitch
        self._cal_roll_sum += roll
        self._cal_n += 1

    def finish_calibration(self) -> None:
        if self._cal_n == 0:
            return
        self._pitch_0 = self._cal_pitch_sum / self._cal_n
        self._roll_0 = self._cal_roll_sum / self._cal_n
        self._is_calibrated = True

    def calibrated_roll(self, roll: float) -> float:
        """返回减去校准 baseline 后的 roll；未校准时返回 0.0。

        BrainControlTab 在 _on_imu_data 中调用，发布到 /eeg_head_pose
        供 chassis_serial_node 做 EEG FORWARD/BACKWARD 帧的方向微调。
        未校准时返回 0，避免发布 baseline 未建立的噪声。
        """
        if not self._is_calibrated:
            return 0.0
        return roll - self._roll_0

    def update(self, pitch: float, roll: float) -> TiltDirection:
        if not self._is_calibrated:
            return TiltDirection.NONE
        pitch_rel = pitch - self._pitch_0
        roll_rel = roll - self._roll_0
        return self._decide(pitch_rel, roll_rel)

    def reset(self) -> None:
        """重置所有状态（用户重新戴帽时调用）。"""
        self._pitch_0 = 0.0
        self._roll_0 = 0.0
        self._cal_pitch_sum = 0.0
        self._cal_roll_sum = 0.0
        self._cal_n = 0
        self._is_calibrated = False
        self._current_dir = TiltDirection.NONE

    def _decide(self, pitch_rel: float, roll_rel: float) -> TiltDirection:
        """圆形 magnitude 滞回判定（spec § 2.1）。"""
        mag_sq = pitch_rel * pitch_rel + roll_rel * roll_rel

        if self._current_dir != TiltDirection.NONE:
            # 已在方向态：magnitude < EXIT_DEG 才退出
            if mag_sq < EXIT_DEG_SQ:
                self._current_dir = TiltDirection.NONE
            else:
                # 滞回区内每帧重算主导轴（允许 FORWARD → LEFT 平滑过渡）
                self._current_dir = self._dominant_direction(pitch_rel, roll_rel)
            return self._current_dir

        # NONE 态：magnitude >= ENTER_DEG 才进入
        if mag_sq >= ENTER_DEG_SQ:
            self._current_dir = self._dominant_direction(pitch_rel, roll_rel)
        return self._current_dir

    def _dominant_direction(self, pitch_rel: float, roll_rel: float) -> TiltDirection:
        """主导轴判定方向（与 TiltIndicator 染色逻辑一致）。

        |pitch| >= |roll| → 前后（pitch+ 前 / pitch- 后）
        否则              → 左右（roll+ 左 / roll- 右）
        """
        if abs(pitch_rel) >= abs(roll_rel):
            return TiltDirection.FORWARD if pitch_rel > 0 else TiltDirection.BACKWARD
        return TiltDirection.LEFT if roll_rel > 0 else TiltDirection.RIGHT
