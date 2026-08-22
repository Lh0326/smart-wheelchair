"""BrainControl Tab UI 缺陷修复回归测试。

覆盖规格：脑控界面修复设计(2026-06-29)
- #5 FocusDetector 加载 SVM 模型
- #1 IMU 右栏状态 label 同步更新
- #3 运动指令 label 彩色背景
"""
import os
from unittest.mock import MagicMock

import pytest

pytest.importorskip('PyQt5')
pytest.importorskip('rclpy')
pytest.importorskip('matplotlib')


@pytest.fixture
def qt_app():
    from PyQt5.QtWidgets import QApplication
    return QApplication.instance() or QApplication([])


@pytest.fixture
def tab(qt_app):
    from wheelchair_app.tabs.braincontrol_tab import BrainControlTab
    return BrainControlTab(ros_node=MagicMock())


def test_focus_model_path_exists(tab):
    """#5: BrainControlTab 应引用存在的 focus_svm.joblib 路径。"""
    from wheelchair_app.tabs.braincontrol_tab import _FOCUS_MODEL_PATH
    assert os.path.exists(_FOCUS_MODEL_PATH), (
        f"focus SVM 模型文件不存在：{_FOCUS_MODEL_PATH}"
    )


def test_focus_detector_loaded_model(tab):
    """#5: FocusDetector 实例的 classifier 应非 None（加载了 SVM 模型）。"""
    assert tab._focus_detector is not None
    assert tab._focus_detector.classifier is not None, (
        "FocusDetector.classifier 为 None —— 模型未加载，状态机会死锁"
    )


def test_right_imu_state_label_updates_with_heartbeat(tab):
    """#1: IMU 数据心跳触发后，右栏 _imu_state_label 应显示已连接。"""
    import time
    # 模拟 IMU 数据回调（写入 _last_imu_data_ts 心跳）
    tab._on_imu_data([10.0, 5.0, 180.0])
    # 调用状态更新
    tab._update_status_labels()
    text = tab._imu_state_label.text()
    assert "✓" in text, f"右栏 IMU 状态 label 未更新为已连接：{text}"


def test_right_imu_state_label_shows_disconnected_when_no_heartbeat(tab):
    """#1: 无 IMU 心跳时，右栏 _imu_state_label 应显示未连接。"""
    # 强制心跳为 0（初始化状态）
    tab._last_imu_data_ts = 0.0
    tab._imu_connect_state = 'connected_waiting'  # 模拟已发起连接
    tab._update_status_labels()
    text = tab._imu_state_label.text()
    assert "✗" in text, f"右栏 IMU 状态 label 未显示未连接：{text}"


def test_motion_command_label_color_changes_with_cmd(tab):
    """#3: 运动指令 label 应根据 cmd 显示对应背景色。"""
    from wheelchair_app.braincontrol.control_types import MotionCommand
    # mock 状态机 state 为 ACTIVE（避免 DISABLED 干扰；state 属性无 setter，写私有字段）
    from wheelchair_app.braincontrol.control_types import ControlState
    tab._state_machine._state = ControlState.ACTIVE

    # FORWARD → 绿 #4a8
    tab._on_motion_command(MotionCommand.FORWARD)
    css_fwd = tab._motion_command_label.styleSheet().lower()
    assert "#4a8" in css_fwd, f"FORWARD 应绿色 #4a8，实际：{css_fwd}"

    # LEFT → 黄 #ca0
    tab._on_motion_command(MotionCommand.LEFT)
    css_left = tab._motion_command_label.styleSheet().lower()
    assert "#ca0" in css_left, f"LEFT 应黄色 #ca0，实际：{css_left}"

    # STOP → 灰 #888
    tab._on_motion_command(MotionCommand.STOP)
    css_stop = tab._motion_command_label.styleSheet().lower()
    assert "#888" in css_stop, f"STOP 应灰色 #888，实际：{css_stop}"


def test_imu_calibration_triggers_after_enough_frames(tab):
    """#3 修复：ImuHandler 前 20 帧累积校准，第 20 帧后 is_calibrated 变 True。

    根因：BrainControlTab 此前从不调 feed_calibration/finish_calibration，
    ImuHandler._is_calibrated 永远 False，update() 永远返回 NONE，
    状态机永远输出 STOP。
    """
    from wheelchair_app.braincontrol.control_types import TiltDirection
    # 校准前
    assert not tab._imu_handler.is_calibrated
    # 喂 19 帧（3 元数 [pitch, roll, yaw] 格式，绕过 HeadPoseCalculator）
    for _ in range(19):
        tab._on_imu_data([0.0, 0.0, 0.0])
    assert not tab._imu_handler.is_calibrated, "19 帧不应触发校准"
    # _current_tilt 在校准前一直 NONE
    assert tab._current_tilt == TiltDirection.NONE
    # 第 20 帧触发校准
    tab._on_imu_data([0.0, 0.0, 0.0])
    assert tab._imu_handler.is_calibrated, "20 帧应触发校准"


def test_imu_direction_after_calibration(tab):
    """校准完成后，足够大的 pitch 应触发 FORWARD（不再恒 NONE）。"""
    from wheelchair_app.braincontrol.control_types import TiltDirection
    # 先完成校准（20 帧基线，pitch=roll=0）
    for _ in range(20):
        tab._on_imu_data([0.0, 0.0, 0.0])
    assert tab._imu_handler.is_calibrated

    # 喂 pitch=30°（>action_deg=18°），绕过 HeadPoseCalculator
    # 直接走 3 元数分支，pitch_rel = 30 - 0 = 30° → FORWARD
    tab._on_imu_data([30.0, 0.0, 0.0])
    assert tab._current_tilt == TiltDirection.FORWARD, (
        f"校准后 pitch=30° 应触发 FORWARD，实际 _current_tilt={tab._current_tilt}"
    )

    # 回到中性（< hysteresis_exit_deg=8°）→ 应退出方向
    tab._on_imu_data([0.0, 0.0, 0.0])
    assert tab._current_tilt == TiltDirection.NONE, (
        f"回中后应退出方向变 NONE，实际 _current_tilt={tab._current_tilt}"
    )


def test_quaternion_neutral_does_not_lock_to_right(tab):
    """ESP32+IMU 静止四元数接近 180° roll 时，也不能固定输出 RIGHT。"""
    from wheelchair_app.braincontrol.control_types import TiltDirection

    neutral_quat = [-0.02, 0.95, 0.02, 0.30]
    for _ in range(21):
        tab._on_imu_data(neutral_quat)

    assert tab._imu_handler.is_calibrated
    assert tab._imu_quat_ref is not None
    assert tab._current_tilt == TiltDirection.NONE, (
        f"静止四元数不应触发 RIGHT，实际 _current_tilt={tab._current_tilt}"
    )
    assert "roll=0" in tab._euler_label.text() or "roll=-0" in tab._euler_label.text()


def test_stale_imu_clears_last_motion(tab):
    """IMU 断流后清零最后一次头姿，防止右转指令卡死。"""
    import time
    from wheelchair_app.braincontrol.control_types import TiltDirection

    tab._current_tilt = TiltDirection.RIGHT
    tab._last_imu_data_ts = time.time() - 2.0

    tab._expire_stale_imu_tilt()

    assert tab._current_tilt == TiltDirection.NONE


def test_clench_detector_threshold_lowered(tab):
    """用户要求：咬牙阈值从 0.5 降到 0.2 让检测更敏感。"""
    assert tab._clench_detector is not None, "ClenchDetector 应成功初始化"
    assert tab._clench_detector.threshold == 0.2, (
        f"threshold 应为 0.2，实际 {tab._clench_detector.threshold}"
    )


def test_clench_detector_higher_infer_frequency(tab):
    """用户要求：提高咬牙 P 值更新频率（内部 infer_every_ms 从 500 降到 50）。"""
    assert tab._clench_detector is not None
    assert tab._clench_detector.infer_every_ms == 50, (
        f"infer_every_ms 应为 50（每次 update 都推断），实际 {tab._clench_detector.infer_every_ms}"
    )
    # 外部节流 150ms（不再跟 FocusDetector 共用 500ms）
    assert tab._clench_interval == 0.15, (
        f"_clench_interval 应为 0.15（150ms 外部节流），实际 {tab._clench_interval}"
    )


def test_focus_detector_receives_car_preprocessed_data(tab, monkeypatch):
    """外部 CAR 修复：FocusDetector 收到的 window 跨通道均值应 ≈ 0。

    根因：main_eeg.py:836-838 在调 FocusDetector.update 前做了一次 CAR，
    训练 SVM 用的是 CAR 后的数据。rtk 之前直接喂 raw，特征功率谱偏高，
    SVM 倾向判 focused，专注度分数难下来。
    """
    import numpy as np
    from wheelchair_app.braincontrol.focus_detector import FocusResult

    received = []
    def spy(window):
        received.append(window.copy())
        return FocusResult(score=50.0, state='neutral', confidence=0.5,
                           p_focus=0.5, p_relax=0.5)
    monkeypatch.setattr(tab._focus_detector, 'update', spy)

    # 喂有共同偏置的 EEG 数据：8 通道都加 100 的常量偏置
    # CAR 后这个共同偏置应被消掉（每时刻跨通道均值为 0）
    n_samples = 1000
    common_bias = 100.0
    data = [[common_bias + float(i % 10) for _ in range(n_samples)]
            for i in range(8)]
    tab._on_eeg_data(data)
    tab._last_focus_compute = 0.0  # 强制下次 tick 进入 FocusDetector 分支
    tab._refresh_canvases()

    assert len(received) >= 1, "FocusDetector.update 未被调用"
    window = received[0]
    # 跨通道均值（每时刻 8 通道的平均）应接近 0
    cross_ch_mean = np.mean(window, axis=1)
    max_dev = float(np.max(np.abs(cross_ch_mean)))
    assert max_dev < 1e-6, (
        f"FocusDetector 输入未做 CAR，跨通道均值偏差 {max_dev}（应 ≈ 0）"
    )


def test_motion_command_label_shows_locked_states(tab):
    """锁定状态时，右栏指令 label 应显示"疲劳锁定"/"主动锁定"覆盖 cmd。

    DISABLED（疲劳锁定）> LOCKED（主动锁定）> cmd，与 _update_motion_banner 一致。
    """
    from wheelchair_app.braincontrol.control_types import (
        ControlState, MotionCommand,
    )

    # DISABLED → 疲劳锁定（即使 _last_cmd 是 FORWARD 也应被覆盖）
    tab._state_machine._state = ControlState.DISABLED
    tab._last_cmd = MotionCommand.FORWARD
    tab._update_motion_command_label()
    text = tab._motion_command_label.text()
    css = tab._motion_command_label.styleSheet().lower()
    assert "疲劳锁定" in text, f"DISABLED 应显示疲劳锁定，实际：{text}"
    assert "#a00" in css, f"DISABLED 应红色 #a00，实际：{css}"

    # LOCKED → 主动锁定
    tab._state_machine._state = ControlState.LOCKED
    tab._last_cmd = MotionCommand.LEFT
    tab._update_motion_command_label()
    text = tab._motion_command_label.text()
    css = tab._motion_command_label.styleSheet().lower()
    assert "主动锁定" in text, f"LOCKED 应显示主动锁定，实际：{text}"
    assert "#a80" in css, f"LOCKED 应橙色 #a80，实际：{css}"
