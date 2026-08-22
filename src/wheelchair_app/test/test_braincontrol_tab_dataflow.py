"""BrainControlTab 数据路由 + canvas 刷新测试。"""
from unittest.mock import MagicMock

import numpy as np
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


def test_score_canvas_is_matplotlib_figure(tab):
    """中栏分数 canvas 是 FigureCanvas 而非占位 QLabel。"""
    from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
    assert isinstance(tab._score_canvas, FigureCanvasQTAgg)


def test_waveform_canvas_is_matplotlib_figure(tab):
    """中栏波形 canvas 是 FigureCanvas。"""
    from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
    assert isinstance(tab._waveform_canvas, FigureCanvasQTAgg)


def test_tilt_indicator_is_real_widget(tab):
    """右栏 TiltIndicator 是真实 QWidget 而非占位 QLabel。"""
    from wheelchair_app.braincontrol.tilt_indicator import TiltIndicator
    assert isinstance(tab._tilt_indicator, TiltIndicator)


def test_eeg_data_callback_caches_samples(tab):
    """EEG 数据回调缓存最近样本。"""
    fake_data = np.random.randn(8, 100).tolist()  # 8 通道 × 100 样本（list 格式，匹配 data_updated 信号）
    for _ in range(5):
        tab._on_eeg_data(fake_data)
    assert tab._eeg_buffer is not None
    assert tab._eeg_buffer.shape[1] >= 100  # 至少 100 样本


def test_imu_data_callback_updates_quaternion_label(tab):
    """IMU 数据回调更新四元数 label。"""
    # 注意：根据 ESP32ImuReader 实际 data_format，
    # data 可能是 [w, x, y, z] 或 [pitch, roll, yaw]
    # 这里测试 [pitch=10.0, roll=5.0, yaw=180.0] 格式
    tab._on_imu_data([10.0, 5.0, 180.0])
    text = tab._quaternion_label.text()
    # 至少有一个数值被更新到 label
    assert "--" not in text, f"label 未更新: {text}"


def test_focus_result_publishes_to_focus_state(tab, monkeypatch):
    """FocusResult 出来后发布 /focus_state String。"""
    published = []

    def fake_publish(msg):
        published.append(msg.data)

    tab._focus_state_pub.publish = fake_publish

    # Mock FocusResult（实际属性名根据 focus_detector.py 确认）
    fake_result = MagicMock()
    fake_result.state = 'focused'
    fake_result.p_focus = 0.85
    fake_result.emg_pollution = 0.1
    fake_result.confidence = 0.9
    tab._on_focus_result(fake_result)

    assert any('focused' in s for s in published), \
        f"/focus_state 未发布 focused，发布内容: {published}"


# ===== /eeg_head_pose publisher 测试（spec § 7.3）=====


def test_braincontrol_tab_publishs_head_pose_after_calibration(qt_app):
    """IMU 校准完成后，_on_imu_data 触发 /eeg_head_pose 发布（data=15.0）。"""
    from wheelchair_app.tabs.braincontrol_tab import BrainControlTab
    from std_msgs.msg import Float64

    ros_node = MagicMock()
    pub = MagicMock()
    ros_node.create_publisher.return_value = pub
    ros_node.get_clock().now().to_msg.return_value = MagicMock(nanoseconds=0)

    tab = BrainControlTab(ros_node=ros_node)
    try:
        # 强制进入已校准状态：feed 20 帧 baseline=0
        for _ in range(21):
            tab._imu_handler.feed_calibration(0.0, 0.0)
        tab._imu_handler.finish_calibration()

        # 触发 IMU 数据回调：pitch=10, roll=15
        tab._on_imu_data([10.0, 15.0, 0.0])

        # 应该 publish 一个 Float64，data=15.0（校准 baseline=0，roll=15-0=15）
        assert pub.publish.called
        published_msgs = [c.args[0] for c in pub.publish.call_args_list
                          if isinstance(c.args[0], Float64)]
        assert len(published_msgs) > 0
        assert published_msgs[-1].data == 15.0
    finally:
        del tab


def test_braincontrol_tab_no_head_pose_publish_when_uncalibrated(qt_app):
    """IMU 未校准时 _on_imu_data 应发布 data=0（保持心跳，不污染缓存）。"""
    from wheelchair_app.tabs.braincontrol_tab import BrainControlTab
    from std_msgs.msg import Float64

    ros_node = MagicMock()
    pub = MagicMock()
    ros_node.create_publisher.return_value = pub
    ros_node.get_clock().now().to_msg.return_value = MagicMock(nanoseconds=0)

    tab = BrainControlTab(ros_node=ros_node)
    try:
        # 未校准（_imu_handler.is_calibrated == False）
        tab._on_imu_data([10.0, 15.0, 0.0])

        # 校准期间也应发 Float64（data=0.0）保持心跳
        published_msgs = [c.args[0] for c in pub.publish.call_args_list
                          if isinstance(c.args[0], Float64)]
        assert len(published_msgs) > 0
        # 校准期间发的应该是 0.0（_publish_head_pose(0.0)）
        assert all(m.data == 0.0 for m in published_msgs)
    finally:
        del tab
