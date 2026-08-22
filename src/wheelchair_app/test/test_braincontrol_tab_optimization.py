"""脑电控制 Tab 优化测试：buffer 改造 / blit / 心跳判定 / 字号放大 / 运动横幅。"""
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


def test_eeg_buffer_is_preallocated_ndarray(tab):
    """EEG buffer 是预分配的固定形状 ndarray，不是 None 或动态数组。"""
    assert tab._eeg_buffer is not None
    assert isinstance(tab._eeg_buffer, np.ndarray)
    assert tab._eeg_buffer.shape == (8, 1500)
    assert tab._eeg_buffer.dtype == np.float64


def test_eeg_buffer_no_concatenate_on_data(tab):
    """EEG 数据到达后 buffer 形状不变（无 concat 重建）。"""
    shape_before = tab._eeg_buffer.shape
    id_before = id(tab._eeg_buffer)
    # 模拟一帧 EEG 数据（8 通道 × 50 样本）
    data = [[float(i + j) for j in range(50)] for i in range(8)]
    tab._on_eeg_data(data)
    assert tab._eeg_buffer.shape == shape_before
    # 同一对象（没有重建）
    assert id(tab._eeg_buffer) == id_before


def test_eeg_buffer_preserves_order(tab):
    """连续两帧数据按时间顺序排列（旧在左，新在右）。"""
    # 第一帧：全部为 1.0
    frame1 = [[1.0] * 100 for _ in range(8)]
    tab._on_eeg_data(frame1)
    # 第二帧：全部为 2.0
    frame2 = [[2.0] * 100 for _ in range(8)]
    tab._on_eeg_data(frame2)
    # 最后 100 个样本应该是 2.0，倒数 101-200 个应该是 1.0
    assert np.all(tab._eeg_buffer[:, -100:] == 2.0)
    assert np.all(tab._eeg_buffer[:, -200:-100] == 1.0)


def test_eeg_buffer_oversized_frame(tab):
    """单帧数据 >= 容量时只保留最后 1500 个样本。"""
    big = [[float(j) for j in range(2000)] for _ in range(8)]
    tab._on_eeg_data(big)
    # 只保留最后 1500 个
    assert tab._eeg_buffer.shape == (8, 1500)
    expected_last = float(1999)
    assert tab._eeg_buffer[0, -1] == expected_last


def test_eeg_buffer_samples_tracks_count(tab):
    """_eeg_buffer_samples 跟踪累计有效样本数。"""
    # 初始为 0
    assert tab._eeg_buffer_samples == 0
    # 喂 50 样本
    tab._on_eeg_data([[float(i) for i in range(50)] for _ in range(8)])
    assert tab._eeg_buffer_samples == 50
    # 再喂 100 样本
    tab._on_eeg_data([[float(i) for i in range(100)] for _ in range(8)])
    assert tab._eeg_buffer_samples == 150


def test_eeg_buffer_samples_saturates_at_capacity(tab):
    """_eeg_buffer_samples 饱和后不再增长（最多 capacity=1500）。"""
    # 喂 2000 样本（超过容量）
    tab._on_eeg_data([[float(i) for i in range(2000)] for _ in range(8)])
    assert tab._eeg_buffer_samples == 1500
    # 再喂 100 样本，仍为 1500（饱和）
    tab._on_eeg_data([[float(i) for i in range(100)] for _ in range(8)])
    assert tab._eeg_buffer_samples == 1500


def test_focus_detector_skipped_until_min_samples(tab, monkeypatch):
    """FocusDetector 在未满旧项目 2s 窗口前不被调用。"""
    from types import SimpleNamespace

    call_count = {'n': 0}

    def spy(window):
        call_count['n'] += 1
        return SimpleNamespace(
            score=50.0, state='neutral', p_focus=0.5, p_relax=0.5,
            confidence=0.5, emg_level=0.0, features={},
            top_contributors=[], artifact_rejected=False,
            fallback_reason=None,
        )

    monkeypatch.setattr(tab._focus_detector, 'update', spy)
    # 喂 999 样本（< 1000），FocusDetector 不应被调用
    tab._on_eeg_data([[float(i) for i in range(999)] for _ in range(8)])
    tab._last_focus_compute = 0.0  # 强制下次 tick 进入 FocusDetector 分支
    tab._refresh_canvases()
    assert call_count['n'] == 0
    # 喂到 1000 样本（2s @500Hz），FocusDetector 应被调用
    tab._on_eeg_data([[float(i) for i in range(1)] for _ in range(8)])
    tab._last_focus_compute = 0.0
    tab._refresh_canvases()
    assert call_count['n'] == 1


def test_focus_detector_uses_last_two_seconds_window(tab, monkeypatch):
    """瞌睡检测应使用 muscles-braincontrol 的最后 2s 窗口。"""
    from types import SimpleNamespace

    captured = {}

    def spy(window):
        captured['window'] = window.copy()
        return SimpleNamespace(
            score=50.0, state='neutral', p_focus=0.5, p_relax=0.5,
            confidence=0.5, emg_level=0.0, features={},
            top_contributors=[], artifact_rejected=False,
            fallback_reason=None,
        )

    monkeypatch.setattr(tab._focus_detector, 'update', spy)
    for ch in range(8):
        tab._eeg_buffer[ch, :] = (
            ch * 10000.0 + (ch + 1) * np.arange(1500, dtype=float)
        )
    tab._eeg_buffer_samples = 1500
    tab._last_focus_compute = 0.0

    tab._refresh_canvases()

    window = captured['window']
    assert window.shape == (1000, 8)
    # 外部 CAR 后，每个时刻跨通道均值为 0；数值确认取的是 500..1499。
    assert np.allclose(np.mean(window, axis=1), 0.0)
    assert np.isclose(window[0, 0], -36750.0)
    assert np.isclose(window[-1, 0], -40246.5)


# ========== 任务 2：matplotlib 波形 blit 改造 ==========


def test_wave_lines_are_animated(tab):
    """波形线条设了 animated=True，让 blit 识别为可增量重绘的 artist。"""
    for line in tab._wave_lines:
        assert line.get_animated() is True


def test_wave_ax_ylim_is_fixed(tab):
    """波形 axes 用固定 ylim（不每帧重算）。"""
    ymin, ymax = tab._wave_ax.get_ylim()
    # 固定范围 -200~1800μV：8 通道 offset=ch*200（CH1=0、CH8=1400），±200 padding 让首末通道不贴边
    assert ymin == -200.0
    assert ymax == 1800.0


def test_wave_bg_cache_field_exists(tab):
    """有 _wave_bg 字段用于缓存背景（初始为 None）。"""
    assert hasattr(tab, '_wave_bg')
    assert tab._wave_bg is None


def test_control_timer_independent_from_canvas_timer(tab):
    """IMU 直控有独立控制定时器，不依赖 EEG canvas 刷新。"""
    assert hasattr(tab, '_control_timer')
    assert tab._control_timer.isActive()
    assert tab._control_timer.interval() == 50


def test_clench_detector_uses_last_two_seconds_window(tab, monkeypatch):
    """咬牙检测应使用训练一致的最后 2s 窗口，而不是 3s UI 波形缓冲。"""
    from types import SimpleNamespace
    from wheelchair_app.tabs import braincontrol_tab as brain_tab_mod

    captured = {}

    class FakeClenchDetector:
        def update(self, window, dt_ms):
            captured['window'] = window.copy()
            captured['dt_ms'] = dt_ms
            return SimpleNamespace(event=False, is_clenching=False, proba=0.12)

    tab._clench_detector = FakeClenchDetector()
    for ch in range(8):
        tab._eeg_buffer[ch, :] = np.arange(1500, dtype=float) + ch * 10000.0
    tab._eeg_buffer_samples = 1500
    tab._last_clench_compute = 10.0
    monkeypatch.setattr(brain_tab_mod.time, 'time', lambda: 10.2)

    tab._tick_clench_detector()

    assert captured['window'].shape == (1000, 8)
    assert captured['window'][0, 0] == 500.0
    assert captured['window'][-1, 0] == 1499.0
    assert 190 <= captured['dt_ms'] <= 210


def test_update_wave_display_uses_blit(tab, monkeypatch):
    """_update_wave_display 主要走 blit 路径，draw 只在首帧捕获背景时调用一次。"""
    blit_calls = []
    draw_calls = []

    real_blit = tab._waveform_canvas.blit
    real_draw = tab._waveform_canvas.draw

    def spy_blit(bbox):
        blit_calls.append(bbox)
        # 不实际调用 real_blit，避免无 GUI 环境 blit 失败

    def spy_draw():
        draw_calls.append(1)
        # 实际调用 real_draw，让 copy_from_bbox 能拿到背景
        real_draw()

    monkeypatch.setattr(tab._waveform_canvas, 'blit', spy_blit)
    monkeypatch.setattr(tab._waveform_canvas, 'draw', spy_draw)

    # 预捕获背景（绕过首帧 draw 触发）
    real_draw()
    tab._wave_bg = tab._waveform_canvas.copy_from_bbox(tab._wave_ax.bbox)

    # 喂足够数据
    data = [[float(i + j) for j in range(200)] for i in range(8)]
    tab._on_eeg_data(data)

    draw_before = len(draw_calls)
    tab._update_wave_display()

    # blit 必须被调用
    assert len(blit_calls) >= 1
    # draw 在 _update_wave_display 中不再被调用（首帧已被预捕获绕过）
    assert len(draw_calls) == draw_before


# ========== 任务 3：EEG/IMU 状态以数据心跳为准 ==========

import time as _time


def test_status_heartbeat_fields_exist(tab):
    """有 EEG/IMU 心跳时间戳字段，初始为 0。"""
    assert hasattr(tab, '_last_eeg_data_ts')
    assert hasattr(tab, '_last_imu_data_ts')
    assert tab._last_eeg_data_ts == 0.0
    assert tab._last_imu_data_ts == 0.0


def test_status_connect_state_fields_exist(tab):
    """有 EEG/IMU connect_state 字段，初始为 'idle'。"""
    assert hasattr(tab, '_eeg_connect_state')
    assert hasattr(tab, '_imu_connect_state')
    assert tab._eeg_connect_state == 'idle'
    assert tab._imu_connect_state == 'idle'


def test_status_shows_disconnected_when_no_data(tab):
    """未收到任何数据时（connect_state='idle'），EEG/IMU 状态都是"未连接"。"""
    tab._update_status_labels()
    assert "未连接" in tab._eeg_status_label.text()
    assert "未连接" in tab._imu_status_label.text()


def test_status_shows_connected_when_eeg_data_recent(tab):
    """最近 1s 收到 EEG 数据时显示"已连接"。"""
    tab._eeg_connect_state = 'connected_waiting'
    tab._last_eeg_data_ts = _time.time()
    tab._update_status_labels()
    assert "已连接" in tab._eeg_status_label.text()
    assert "✓" in tab._eeg_status_label.text()


def test_status_shows_disconnected_when_eeg_data_stale(tab):
    """EEG 数据时间戳超过 1s 显示"未连接"。"""
    tab._eeg_connect_state = 'connected_waiting'
    tab._last_eeg_data_ts = _time.time() - 2.0  # 2 秒前
    tab._update_status_labels()
    assert "未连接" in tab._eeg_status_label.text()


def test_status_shows_connected_when_imu_data_recent(tab):
    """最近 1s 收到 IMU 数据时显示"已连接"。"""
    tab._imu_connect_state = 'connected_waiting'
    tab._last_imu_data_ts = _time.time()
    tab._update_status_labels()
    assert "已连接" in tab._imu_status_label.text()
    assert "✓" in tab._imu_status_label.text()


def test_status_preserves_failed_text(tab):
    """connect_state='failed' 时 _update_status_labels 不覆盖失败文本。"""
    tab._eeg_connect_state = 'failed'
    tab._eeg_status_label.setText("EEG: ✗ 端口被占用")
    tab._last_eeg_data_ts = _time.time()  # 即使最近有数据
    tab._update_status_labels()
    assert "端口被占用" in tab._eeg_status_label.text()


def test_on_eeg_data_updates_heartbeat(tab):
    """_on_eeg_data 入口更新 EEG 心跳时间戳。"""
    before = tab._last_eeg_data_ts
    data = [[float(i + j) for j in range(50)] for i in range(8)]
    tab._on_eeg_data(data)
    assert tab._last_eeg_data_ts > before


def test_on_imu_data_updates_heartbeat(tab):
    """_on_imu_data 入口更新 IMU 心跳时间戳。"""
    before = tab._last_imu_data_ts
    # 模拟欧拉角数据（3 float）—— 注意 _on_imu_data 入口要先写心跳，
    # 即使后续解析失败也应已写入
    tab._on_imu_data([1.0, 2.0, 3.0])
    assert tab._last_imu_data_ts > before


def test_status_shows_disconnected_when_connected_but_no_data(tab):
    """连接成功但数据从未到达（connect_state='connected_waiting' + ts=0）显示未连接。

    锁死核心修复场景：串口打开但设备沉默时，UI 不应误导用户为"已连接"。
    """
    tab._eeg_connect_state = 'connected_waiting'
    tab._last_eeg_data_ts = 0.0  # 从未收到数据
    tab._update_status_labels()
    assert "未连接" in tab._eeg_status_label.text()


def test_status_preserves_scanning_text(tab):
    """connect_state='scanning' 时 _update_status_labels 不覆盖扫描中文本。"""
    tab._eeg_connect_state = 'scanning'
    tab._eeg_status_label.setText("EEG: 扫描中...")
    tab._last_eeg_data_ts = 0.0
    tab._update_status_labels()
    assert "扫描中" in tab._eeg_status_label.text()


# ========== 任务 4：专注度 24pt + 二元配色 ==========


def _make_focus_result(score=80.0, state='focused', p_focus=0.8,
                       confidence=0.9, emg_level=0.1):
    """构造 FocusResult-like 对象（用 SimpleNamespace 避免依赖模型文件）。"""
    from types import SimpleNamespace
    return SimpleNamespace(
        score=score, state=state, p_focus=p_focus, p_relax=1 - p_focus,
        confidence=confidence, emg_level=emg_level,
        features={}, top_contributors=[], artifact_rejected=False,
        fallback_reason=None,
    )


def test_focus_score_label_large_font(tab):
    """专注度分数 label 字号 >= 24pt。"""
    font = tab._focus_score_label.font()
    assert font.pointSize() >= 24


def test_focus_score_label_bold(tab):
    """专注度分数 label 加粗。"""
    font = tab._focus_score_label.font()
    assert font.bold() is True


def test_focus_state_label_medium_bold_font(tab):
    """专注度状态 label 字号 >= 14pt 且加粗。"""
    font = tab._focus_state_label.font()
    assert font.pointSize() >= 14
    assert font.bold() is True


def test_focus_state_focused_when_score_above_50(tab):
    """score > 50 显示"专注"且背景绿色。"""
    result = _make_focus_result(score=82.0, state='focused', p_focus=0.82)
    tab._on_focus_result(result)
    assert "专注" in tab._focus_state_label.text()
    style = tab._focus_state_label.styleSheet().lower()
    assert "#4a8" in style


def test_focus_state_relaxed_shows_drowsy(tab):
    """旧项目语义：state=relaxed 显示"瞌睡"且背景红色。"""
    result = _make_focus_result(score=40.0, state='relaxed', p_focus=0.40)
    tab._on_focus_result(result)
    assert "瞌睡" in tab._focus_state_label.text()
    style = tab._focus_state_label.styleSheet().lower()
    assert "#a00" in style


def test_focus_state_neutral_shows_normal(tab):
    """旧项目语义：state=neutral 是"正常"，不是直接显示"瞌睡"。"""
    result = _make_focus_result(score=50.0, state='neutral', p_focus=0.50)
    tab._on_focus_result(result)
    assert "正常" in tab._focus_state_label.text()
    style = tab._focus_state_label.styleSheet().lower()
    assert "#a80" in style


def test_focus_score_label_shows_pure_number(tab):
    """分数 label 显示纯数字（不带"分数:"前缀）。"""
    result = _make_focus_result(score=82.0, state='focused', p_focus=0.82)
    tab._on_focus_result(result)
    text = tab._focus_score_label.text()
    assert "82" in text
    assert "分数" not in text


def test_focus_score_at_exact_threshold_uses_state(tab):
    """显示状态跟随 FocusResult.state，不再用 score=50 二元切分。"""
    result = _make_focus_result(score=50.0, state='neutral', p_focus=0.50)
    tab._on_focus_result(result)
    assert "正常" in tab._focus_state_label.text()


def test_motion_banner_exists(tab):
    """有顶部运动状态横幅 label。"""
    assert hasattr(tab, '_motion_banner_label')
    from PyQt5.QtWidgets import QLabel
    assert isinstance(tab._motion_banner_label, QLabel)


def test_motion_banner_large_font(tab):
    """横幅字号 >= 20pt 且加粗。"""
    font = tab._motion_banner_label.font()
    assert font.pointSize() >= 20
    assert font.bold() is True


def test_motion_banner_initial_text(tab):
    """初始状态横幅显示"静止"（浅灰底）。"""
    assert "静止" in tab._motion_banner_label.text()


def test_motion_banner_disabled_state(tab):
    """ControlState.DISABLED 显示"疲劳锁定"红底。"""
    from wheelchair_app.braincontrol.control_types import ControlState
    tab._state_machine._state = ControlState.DISABLED
    tab._update_motion_banner()
    assert "疲劳锁定" in tab._motion_banner_label.text()
    style = tab._motion_banner_label.styleSheet().lower()
    assert "#a00" in style


def test_motion_banner_locked_state(tab):
    """ControlState.LOCKED 显示"锁定（主动）"橙底。"""
    from wheelchair_app.braincontrol.control_types import ControlState
    tab._state_machine._state = ControlState.LOCKED
    tab._update_motion_banner()
    assert "主动" in tab._motion_banner_label.text()
    style = tab._motion_banner_label.styleSheet().lower()
    assert "#a80" in style


def test_motion_banner_forward(tab):
    """ACTIVE + FORWARD 显示"前进"绿底。"""
    from wheelchair_app.braincontrol.control_types import (
        ControlState, MotionCommand
    )
    tab._state_machine._state = ControlState.ACTIVE
    tab._last_cmd = MotionCommand.FORWARD
    tab._update_motion_banner()
    assert "前进" in tab._motion_banner_label.text()
    style = tab._motion_banner_label.styleSheet().lower()
    assert "#4a8" in style


def test_motion_banner_backward(tab):
    """ACTIVE + BACKWARD 显示"后退"红橙底。"""
    from wheelchair_app.braincontrol.control_types import (
        ControlState, MotionCommand
    )
    tab._state_machine._state = ControlState.ACTIVE
    tab._last_cmd = MotionCommand.BACKWARD
    tab._update_motion_banner()
    assert "后退" in tab._motion_banner_label.text()
    style = tab._motion_banner_label.styleSheet().lower()
    assert "#a84" in style


def test_motion_banner_left_right(tab):
    """ACTIVE + LEFT/RIGHT 显示"左转"/"右转"黄底。"""
    from wheelchair_app.braincontrol.control_types import (
        ControlState, MotionCommand
    )
    tab._state_machine._state = ControlState.ACTIVE

    tab._last_cmd = MotionCommand.LEFT
    tab._update_motion_banner()
    assert "左转" in tab._motion_banner_label.text()
    style = tab._motion_banner_label.styleSheet().lower()
    assert "#ca0" in style

    tab._last_cmd = MotionCommand.RIGHT
    tab._update_motion_banner()
    assert "右转" in tab._motion_banner_label.text()


def test_motion_banner_stop(tab):
    """ACTIVE + STOP 显示"静止"浅灰底。"""
    from wheelchair_app.braincontrol.control_types import (
        ControlState, MotionCommand
    )
    tab._state_machine._state = ControlState.ACTIVE
    tab._last_cmd = MotionCommand.STOP
    tab._update_motion_banner()
    assert "静止" in tab._motion_banner_label.text()
    style = tab._motion_banner_label.styleSheet().lower()
    assert "#f5f5f5" in style


def test_motion_banner_priority_disabled_over_locked(tab):
    """DISABLED 优先于 LOCKED 显示（虽然状态机不会同时为两者，但 _update_motion_banner 判定顺序要对）。"""
    from wheelchair_app.braincontrol.control_types import (
        ControlState, MotionCommand
    )
    # 状态机内部一次只能是一个状态，但 banner 判定顺序应该是 DISABLED > LOCKED > 运动指令 > STOP
    tab._state_machine._state = ControlState.DISABLED
    tab._last_cmd = MotionCommand.FORWARD  # 即使有运动指令
    tab._update_motion_banner()
    # DISABLED 应该胜出
    assert "疲劳锁定" in tab._motion_banner_label.text()


def test_right_column_wider_than_before(tab):
    """右栏 stretch 应该 >= 2（原为 1，现在加倍让 IMU 雷达图变大）。"""
    # 通过SizePolicy或stretch验证——直接检查 TiltIndicator 最小尺寸加大
    min_w = tab._tilt_indicator.minimumSize().width()
    min_h = tab._tilt_indicator.minimumSize().height()
    assert min_w >= 300, f"TiltIndicator min width {min_w} < 300"
    assert min_h >= 300, f"TiltIndicator min height {min_h} < 300"


def test_motion_banner_text_no_emoji(tab):
    """横幅文案不再有 emoji 和括号（'疲劳锁定'而非'🔒 锁定（瞌睡）'）。"""
    from wheelchair_app.braincontrol.control_types import ControlState
    tab._state_machine._state = ControlState.DISABLED
    tab._update_motion_banner()
    text = tab._motion_banner_label.text()
    assert "疲劳锁定" in text
    assert "🔒" not in text
    assert "（" not in text and "(" not in text

    tab._state_machine._state = ControlState.LOCKED
    tab._update_motion_banner()
    assert "主动锁定" in tab._motion_banner_label.text()


def test_motion_banner_has_border(tab):
    """横幅样式表包含 border（粗边框，让"框感"明显）。"""
    tab._update_motion_banner()
    style = tab._motion_banner_label.styleSheet().lower()
    assert "border" in style
    assert "3px" in style or "4px" in style or "5px" in style  # 至少 3px 粗


def test_motion_command_label_chinese(tab):
    """右栏运动指令 label 显示中文（前进/后退/左转/右转/静止）。"""
    from wheelchair_app.braincontrol.control_types import (
        ControlState, MotionCommand,
    )
    tab._state_machine._state = ControlState.ACTIVE
    tab._last_cmd = None  # 重置去重缓存
    tab._on_motion_command(MotionCommand.FORWARD)
    text = tab._motion_command_label.text()
    assert "前进" in text
    assert "FORWARD" not in text

    tab._last_cmd = None
    tab._on_motion_command(MotionCommand.LEFT)
    assert "左转" in tab._motion_command_label.text()


# ========== 任务 7：去掉皱眉，咬牙 rising edge 做 LOCKED ↔ ACTIVE toggle ==========


def test_no_frown_detector_in_tab(tab):
    """FrownDetector 已从 tab 移除。"""
    assert not hasattr(tab, '_frown_detector') or tab._frown_detector is None


def test_no_frown_label_in_tab(tab):
    """左栏"皱眉: --" label 已移除。"""
    assert not hasattr(tab, '_frown_label')


def test_toggle_pending_field_renamed(tab):
    """_frown_pending 改名为 _toggle_pending。"""
    assert hasattr(tab, '_toggle_pending')
    assert not hasattr(tab, '_frown_pending')
    assert tab._toggle_pending is False  # 初始 False


def test_clench_event_sets_toggle_pending(tab):
    """咬牙 rising edge 触发 _toggle_pending = True。"""
    from types import SimpleNamespace
    tab._toggle_pending = False
    # 模拟 ClenchResult 的 rising edge event
    fake_event = SimpleNamespace(event=True, is_clenching=True, proba=0.85)
    tab._on_clench_event(fake_event)
    assert tab._toggle_pending is True


def test_state_machine_uses_toggle_event(tab):
    """状态机 update() 第 2 个参数名是 toggle_event（不是 frown_event）。"""
    import inspect
    from wheelchair_app.braincontrol.control_state_machine import ControlStateMachine
    sig = inspect.signature(ControlStateMachine.update)
    assert 'toggle_event' in sig.parameters
    assert 'frown_event' not in sig.parameters


def test_state_machine_toggle_locks_active(tab):
    """LOCKED + toggle → ACTIVE；ACTIVE + toggle → LOCKED（咬牙 toggle 行为）。"""
    from wheelchair_app.braincontrol.control_types import (
        ControlState, MotionCommand, TiltDirection
    )
    # R9：_tick_state_machine 在 isVisible()=False 时跳过状态机输出，
    # 测试需要 tab 可见才走完整路径
    tab.show()
    # LOCKED + toggle → ACTIVE
    tab._state_machine._state = ControlState.LOCKED
    tab._focus_state = 'focused'
    tab._toggle_pending = True
    tab._current_tilt = TiltDirection.NONE
    tab._last_state_machine_tick = 0  # 用默认 dt
    tab._tick_state_machine()
    assert tab._state_machine.state == ControlState.ACTIVE
    assert tab._toggle_pending is False  # tick 后消费

    # 推进冷却计时（FRONW_COOLDOWN_MS=1500ms），让下一次 toggle 不被冷却挡住。
    # 直接置位状态机内部冷却字段，避免多次 tick 累积。
    tab._state_machine._frown_cooldown_ms = 0

    # ACTIVE + toggle → LOCKED
    tab._toggle_pending = True
    tab._last_state_machine_tick = 0
    tab._tick_state_machine()
    assert tab._state_machine.state == ControlState.LOCKED
