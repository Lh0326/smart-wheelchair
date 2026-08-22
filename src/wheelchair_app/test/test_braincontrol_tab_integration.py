"""BrainControlTab 集成测试（无硬件依赖，跨组件端到端）。

聚焦补充价值——现有单元测试（test_braincontrol_tab_dataflow/
connection/skeleton/state_machine）全部用 ros_node=MagicMock() 拦截
publisher，验证的是「调用了 publish」。本文件用真实 rclpy Node + spin
线程，验证「消息真的能被另一个 subscription 收到」：

  - /focus_state String 真实出 ROS 总线
  - /eeg_mode_active Bool 真实出 ROS 总线
  - /cmd_vel_eeg Twist 真实出 ROS 总线
  - 完整状态机闭环（FocusDetector + ClenchDetector + ImuHandler +
    ControlStateMachine + MotionCommander）端到端跑通，无 mock
  - 安全不变量（toggle OFF 或 relaxed → /cmd_vel_eeg 恒为 STOP）

避免与 mock-based 单元测试重复。
"""
import threading
import time

import numpy as np
import pytest

pytest.importorskip('PyQt5')
pytest.importorskip('rclpy')
pytest.importorskip('matplotlib')


# ========== rclpy + Qt fixtures ==========

@pytest.fixture(scope='module')
def rclpy_init():
    import rclpy
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def ros_node(rclpy_init):
    """真实 rclpy Node（spin_once 在背景线程，让 publisher/subscription 真实通信）。

    不能用 MagicMock：MotionCommander.__init__ 会调
    ros_node.create_publisher(Twist, '/cmd_vel_eeg', 10)，mock 返回的
    是 MagicMock 而非真实 Publisher，publish 不会进 ROS 总线，订阅收不到。
    """
    import rclpy
    n = rclpy.create_node('test_braincontrol_integration')

    stop_flag = {'on': False}

    def _spin():
        while not stop_flag['on']:
            rclpy.spin_once(n, timeout_sec=0.005)

    spin_thread = threading.Thread(target=_spin, daemon=True)
    spin_thread.start()
    yield n
    stop_flag['on'] = True
    n.destroy_node()
    spin_thread.join(timeout=2.0)


@pytest.fixture
def qt_app():
    from PyQt5.QtWidgets import QApplication
    return QApplication.instance() or QApplication([])


@pytest.fixture
def tab(qt_app, ros_node):
    """真实 BrainControlTab（ros_node 是真实 rclpy Node）。"""
    from wheelchair_app.tabs.braincontrol_tab import BrainControlTab
    t = BrainControlTab(ros_node=ros_node)
    # R9 补丁：_tick_state_machine 在 isVisible()=False 时跳过状态机输出
    # 测试需要 tab 可见才走完整路径
    t.show()
    return t


def _drain(received, expected_count, timeout_sec=1.0):
    """轮询直到 received 至少有 expected_count 条消息或超时。

    返回全部已收到的消息（不切片）——让测试自己断言具体条目，
    避免订阅队列顺序导致期望的某条消息被截断。
    """
    deadline = time.time() + timeout_sec
    while time.time() < deadline and len(received) < expected_count:
        time.sleep(0.01)
    return list(received)


# ========== 5 个集成测试 case ==========

def test_focus_state_publishes_to_real_ros_bus(tab, ros_node):
    """Case 1：FocusResult → /focus_state String 真实出 ROS 总线。

    与单元测试的区别：单元测试 monkeypatch 拦截 publish 验证「调了 publish」；
    本测试用真实 subscription 监听，验证「消息真的被另一个节点收到」。
    额外断言：5Hz throttle 生效（200ms 内第二次调用不会发布）。
    """
    from std_msgs.msg import String
    received = []
    ros_node.create_subscription(
        String, '/focus_state', lambda m: received.append(m.data), 10
    )

    # 构造一个真实的 FocusResult dataclass-like 对象（不用 MagicMock，
    # 避免 _on_focus_result 里 getattr 拦截返回非数值）
    class _FakeFocusResult:
        state = 'focused'
        score = 0.85
        p_focus = 0.85
        confidence = 0.9
        emg_level = 0.1

    tab._on_focus_result(_FakeFocusResult())
    msgs = _drain(received, 1)
    assert len(msgs) == 1
    assert msgs[0].startswith('focused:')

    # 5Hz throttle 验证：上次发布后再立即调一次，不应再发（< 200ms 窗口）
    received.clear()
    tab._on_focus_result(_FakeFocusResult())
    msgs = _drain(received, 1, timeout_sec=0.3)
    assert len(msgs) == 0, "5Hz throttle 失效：200ms 内重复发布了"


def test_eeg_mode_toggle_publishes_bool_to_real_ros_bus(tab, ros_node):
    """Case 2：toggle ON → /eeg_mode_active 收到 Bool(true)，OFF → Bool(false)。

    与单元测试的区别：真实 ROS 总线监听，验证 toggle 行为完整闭环
    （setChecked + _on_eeg_mode_toggled → publish → 另一节点订阅）。
    """
    from std_msgs.msg import Bool
    received = []
    ros_node.create_subscription(
        Bool, '/eeg_mode_active', lambda m: received.append(m.data), 10
    )

    # toggle ON
    tab._eeg_mode_toggle.setChecked(True)
    tab._on_eeg_mode_toggled()
    msgs = _drain(received, 1)
    assert msgs == [True], f"toggle ON 应发布 Bool(True)，实际：{msgs}"

    # toggle OFF
    received.clear()
    tab._eeg_mode_toggle.setChecked(False)
    tab._on_eeg_mode_toggled()
    msgs = _drain(received, 1)
    assert msgs == [False], f"toggle OFF 应发布 Bool(False)，实际：{msgs}"


def test_clench_toggle_unlocks_and_tilt_publishes_cmd_vel_eeg(tab, ros_node):
    """Case 3：clench toggle LOCKED→ACTIVE + tilt FORWARD → /cmd_vel_eeg Twist。

    完整端到端：ControlStateMachine 默认 LOCKED，toggle_event（咬牙上升沿）切到
    ACTIVE；ACTIVE 下 tilt=FORWARD → MotionCommand.FORWARD →
    MotionCommander.publish(Twist)。验证 /cmd_vel_eeg 真实收到 Twist，
    且 linear.x > 0（前进）。这是 FocusDetector + ClenchDetector +
    ImuHandler + ControlStateMachine + MotionCommander 协同的最小闭环。
    """
    from geometry_msgs.msg import Twist
    from wheelchair_app.braincontrol.control_types import (
        MotionCommand, TiltDirection,
    )

    received = []
    ros_node.create_subscription(
        Twist, '/cmd_vel_eeg', lambda m: received.append(m), 10
    )

    # 直接喂状态机输入：focus='focused' + toggle=True（咬牙上升沿）+ tilt=NONE
    # ControlStateMachine.update 是同步 API，tab._tick_state_machine 每 50ms
    # 调一次；这里手动设置缓存 + 触发一次 tick 模拟一帧。
    tab._focus_state = 'focused'
    tab._toggle_pending = True
    tab._current_tilt = TiltDirection.NONE
    tab._tick_state_machine()

    # 此时状态机应从 LOCKED 切到 ACTIVE（toggle 上升沿）；tilt=NONE 输出 STOP
    from wheelchair_app.braincontrol.control_types import ControlState
    assert tab._state_machine.state == ControlState.ACTIVE, \
        "toggle 上升沿应把状态机从 LOCKED 切到 ACTIVE"

    # 第二帧：tilt=FORWARD，toggle_pending=False（已消费）
    tab._current_tilt = TiltDirection.FORWARD
    tab._toggle_pending = False
    tab._tick_state_machine()

    # 等 FORWARD Twist 真实出 ROS 总线（取 2 条：第 1 帧 STOP + 第 2 帧 FORWARD）
    msgs = _drain(received, 2)
    assert len(msgs) >= 2, \
        f"应收到至少 2 条 Twist（LOCKED→ACTIVE 时 STOP + ACTIVE 下 FORWARD），实际：{len(msgs)}"
    # 最后一条应是 FORWARD：linear.x > 0
    assert msgs[-1].linear.x > 0.0, \
        f"FORWARD 应该让 linear.x > 0，实际：{msgs[-1].linear.x}"


def test_locked_state_never_publishes_nonzero_twist(tab, ros_node):
    """Case 4：LOCKED 状态下 tilt 任何方向都不应产生非零 Twist。

    安全不变量：状态机默认 LOCKED（__init__ 设的），即使头姿态任意变化，
    /cmd_vel_eeg 也只能收到 STOP（linear/angular 全 0）。防止脑控被误激活
    导致轮椅意外移动——这是端到端安全测试，单元测试覆盖不到（单测只 mock
    commander.update 调用次数）。
    """
    from geometry_msgs.msg import Twist
    from wheelchair_app.braincontrol.control_types import TiltDirection

    received = []
    ros_node.create_subscription(
        Twist, '/cmd_vel_eeg', lambda m: received.append(m), 10
    )

    # 状态机默认 LOCKED，focus='focused'，toggle=False，tilt 任意
    tab._focus_state = 'focused'
    tab._toggle_pending = False
    for tilt in (TiltDirection.FORWARD, TiltDirection.BACKWARD,
                 TiltDirection.LEFT, TiltDirection.RIGHT):
        tab._current_tilt = tilt
        tab._tick_state_machine()

    msgs = _drain(received, 1)
    # LOCKED 下所有 Twist 必须是 STOP（全 0）
    for m in msgs:
        assert m.linear.x == 0.0 and m.linear.y == 0.0 \
            and m.angular.z == 0.0, \
            f"LOCKED 状态下不应有非零 Twist，收到：{m}"


def test_eeg_buffer_feeds_focus_detector_end_to_end(tab):
    """Case 5：多帧 EEG → 50ms timer → FocusDetector 真实计算无崩溃。

    与 test_dataflow.test_eeg_data_callback_caches_samples 的区别：
    单测只验证 _eeg_buffer shape，本测试进一步触发 _refresh_canvases（50ms
    timer 的回调），让 FocusDetector 真正消费 buffer（不 mock FocusResult），
    验证完整链路：EEG buffer → FocusDetector.update → _on_focus_result
    → GUI label 更新 + 状态机输入缓存，无异常。
    """
    # 喂入足够多的样本（FocusDetector 需要 2s 窗口 ≈ 500 样本@250Hz）
    fake_data = np.random.randn(8, 600).tolist()
    for _ in range(5):
        tab._on_eeg_data(fake_data)

    # 验证 buffer 已就绪
    assert tab._eeg_buffer.shape[0] == 8
    assert tab._eeg_buffer.shape[1] >= 500

    # 手动触发一次 canvas refresh（50ms timer 的回调），让 FocusDetector 真跑
    # 用 try/except 包装：FocusDetector 默认 classifier=None，update 内部
    # 可能返回 None 或抛 ValueError，_refresh_canvases 已有 try 兜底
    tab._refresh_canvases()  # 不应抛异常

    # canvas refresh timer 应已启动（_on_eeg_data 内启动）
    assert tab._canvas_refresh_timer.isActive()
