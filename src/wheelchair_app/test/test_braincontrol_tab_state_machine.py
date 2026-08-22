"""BrainControlTab ControlStateMachine 接线测试（任务 6）。

实测架构（与 spec § 6 一致）：
    - ControlStateMachine 是纯 Python 类（无 pyqtSignal），每帧由 tab
      主动调 .update(focus_state, toggle_event, tilt, dt_ms) -> MotionCommand
    - 同步：tab 调 commander.update(cmd) 发布 /cmd_vel_eeg
    - tab 暴露 _state_machine / _motion_commander / _clench_detector /
      _imu_handler 实例
    - tab 暴露 _on_motion_command(cmd) 处理输出（commander + label）
    - tab 把 FocusResult / HeadPose / Clench 事件喂给状态机
"""
from unittest.mock import MagicMock

import pytest

pytest.importorskip('PyQt5')
pytest.importorskip('rclpy')


@pytest.fixture
def qt_app():
    from PyQt5.QtWidgets import QApplication
    return QApplication.instance() or QApplication([])


@pytest.fixture
def tab(qt_app):
    from wheelchair_app.tabs.braincontrol_tab import BrainControlTab
    t = BrainControlTab(ros_node=MagicMock())
    # R9 补丁：_tick_state_machine 在 isVisible()=False 时跳过状态机输出
    # 测试需要 tab 可见才走完整路径
    t.show()
    return t


# ========== 实例存在性 ==========

def test_state_machine_instance_exists(tab):
    """tab 初始化了 ControlStateMachine 实例。"""
    from wheelchair_app.braincontrol.control_state_machine import \
        ControlStateMachine
    assert isinstance(tab._state_machine, ControlStateMachine)


def test_motion_commander_instance_exists(tab):
    """tab 初始化了 MotionCommander 实例。"""
    from wheelchair_app.braincontrol.motion_commander import MotionCommander
    assert isinstance(tab._motion_commander, MotionCommander)


def test_clench_detector_instance_exists(tab):
    """tab 初始化了 ClenchDetector 实例（或 None 因模型缺失）。"""
    from wheelchair_app.braincontrol.clench_detector import ClenchDetector
    # 任务文档允许模型缺失时为 None（lazy fallback），但模型文件存在时应为实例
    assert tab._clench_detector is None or isinstance(
        tab._clench_detector, ClenchDetector
    )


# ========== 输出路由 ==========

def test_state_machine_output_calls_commander_update(tab, monkeypatch):
    """_on_motion_command 把 MotionCommand 推给 MotionCommander.update。"""
    called = {'cmd': None}

    def fake_update(cmd):
        called['cmd'] = cmd

    tab._motion_commander.update = fake_update

    from wheelchair_app.braincontrol.control_types import MotionCommand
    tab._on_motion_command(MotionCommand.FORWARD)
    assert called['cmd'] == MotionCommand.FORWARD


def test_repeated_non_stop_commands_are_republished(tab):
    """持续头姿下相同非 STOP 指令也要持续发布，刷新底盘 watchdog。"""
    calls = []
    tab._motion_commander.update = lambda cmd: calls.append(cmd)

    from wheelchair_app.braincontrol.control_types import MotionCommand
    tab._on_motion_command(MotionCommand.FORWARD)
    tab._on_motion_command(MotionCommand.FORWARD)

    assert calls == [MotionCommand.FORWARD, MotionCommand.FORWARD]


def test_state_machine_output_updates_gui_label(tab):
    """_on_motion_command 更新运动指令 label（中文）。"""
    from wheelchair_app.braincontrol.control_types import (
        ControlState, MotionCommand,
    )
    tab._state_machine._state = ControlState.ACTIVE
    tab._on_motion_command(MotionCommand.LEFT)
    assert "左转" in tab._motion_command_label.text()


def test_state_machine_output_updates_state_label(tab):
    """_on_motion_command 更新状态机 label（显示当前状态）。"""
    # 直接验证：当前状态（LOCKED/ACTIVE/...）的 name 出现在 label
    expected = tab._state_machine.state.name
    from wheelchair_app.braincontrol.control_types import MotionCommand
    tab._on_motion_command(MotionCommand.STOP)
    assert expected in tab._state_machine_label.text()


# ========== 输入路由：FocusResult ==========

def test_focus_result_routed_to_state_machine(tab, monkeypatch):
    """FocusResult 经 _on_focus_result 喂给 state_machine.update。

    ControlStateMachine.update(focus_state, toggle_event, tilt, dt_ms) 是同步
    API，没有独立 on_focus 方法。tab 的 _on_focus_result 负责缓存 focus_state，
    下一次 _refresh_canvases / tick 时统一喂状态机。
    """
    # 监视 state_machine.update
    calls = {'focus_state': None, 'count': 0}
    real_update = tab._state_machine.update

    def spy_update(focus_state, toggle_event, tilt, dt_ms):
        calls['focus_state'] = focus_state
        calls['count'] += 1
        return real_update(focus_state, toggle_event, tilt, dt_ms)

    monkeypatch.setattr(tab._state_machine, 'update', spy_update)

    # 模拟 FocusResult：state=focused，p_focus=0.85
    fake_result = MagicMock()
    fake_result.state = 'focused'
    fake_result.score = 0.85
    fake_result.p_focus = 0.85
    fake_result.confidence = 0.9
    fake_result.emg_level = 0.2
    tab._on_focus_result(fake_result)

    # 触发一次状态机 tick（让缓存喂给 update）
    tab._tick_state_machine()

    assert calls['count'] >= 1, \
        "state_machine.update 未被调用"
    assert calls['focus_state'] == 'focused', \
        f"focus_state 未正确路由：{calls['focus_state']}"


# ========== 输入路由：HeadPose（IMU） ==========

def test_head_pose_routed_to_state_machine(tab, monkeypatch):
    """IMU euler 数据经 _on_imu_data 喂给状态机。

    ControlStateMachine 需要的是 TiltDirection（经 ImuHandler.update(pitch, roll)
    转换）。tab 的 _on_imu_data 负责：解析 euler → HeadPoseCalculator →
    ImuHandler → TiltDirection → 缓存；下次 _tick_state_machine 喂状态机。
    """
    calls = {'tilt': None, 'count': 0}
    real_update = tab._state_machine.update

    def spy_update(focus_state, toggle_event, tilt, dt_ms):
        calls['tilt'] = tilt
        calls['count'] += 1
        return real_update(focus_state, toggle_event, tilt, dt_ms)

    monkeypatch.setattr(tab._state_machine, 'update', spy_update)

    # 触发 IMU 数据：euler 格式 [pitch=10, roll=5, yaw=180]
    # 注意：未校准的 ImuHandler.update 会直接返回 NONE（_decide 的早退分支）
    tab._on_imu_data([10.0, 5.0, 180.0])

    # 触发一次 tick，让状态机消费
    tab._tick_state_machine()

    # 至少 update 被调过一次（tilt 是 TiltDirection 枚举值，包括 NONE）
    from wheelchair_app.braincontrol.control_types import TiltDirection
    assert calls['count'] >= 1, "state_machine.update 未被调用"
    assert calls['tilt'] in TiltDirection.__members__.values(), \
        f"tilt 不是 TiltDirection：{calls['tilt']}"
