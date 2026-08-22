"""BrainControlTab 骨架测试：tab 可加载，三栏布局存在。"""
import pytest

pytest.importorskip('PyQt5')
pytest.importorskip('rclpy')


@pytest.fixture
def qt_app(qapp_args=None):
    from PyQt5.QtWidgets import QApplication
    app = QApplication.instance() or QApplication([])
    return app


@pytest.fixture
def ros_node():
    """用 mock node 避免依赖真实 rclpy init。"""
    from unittest.mock import MagicMock
    return MagicMock()


def test_tab_constructs_without_hardware(qt_app, ros_node):
    """tab 实例化成功，不连硬件也能加载。"""
    from wheelchair_app.tabs.braincontrol_tab import BrainControlTab
    tab = BrainControlTab(ros_node=ros_node)
    assert tab is not None


def test_three_column_layout_exists(qt_app, ros_node):
    """三栏布局控件存在。"""
    from wheelchair_app.tabs.braincontrol_tab import BrainControlTab
    tab = BrainControlTab(ros_node=ros_node)

    # 左栏控件（端口自动识别，无下拉）
    assert hasattr(tab, '_connect_button'), "外设连接按钮缺失"
    assert hasattr(tab, '_eeg_mode_toggle'), "脑控模式 toggle 缺失"
    assert hasattr(tab, '_eeg_status_label'), "EEG 状态 label 缺失"
    assert hasattr(tab, '_imu_status_label'), "IMU 状态 label 缺失"

    # 中栏控件
    assert hasattr(tab, '_score_canvas'), "分数 canvas 缺失"
    assert hasattr(tab, '_waveform_canvas'), "8 通道波形 canvas 缺失"

    # 右栏控件
    assert hasattr(tab, '_imu_state_label'), "IMU 状态 label 缺失"
    assert hasattr(tab, '_tilt_indicator'), "TiltIndicator 罗盘缺失"
    assert hasattr(tab, '_motion_command_label'), "运动指令 label 缺失"


def test_no_hardcoded_sys_path_insert(qt_app, ros_node):
    """验证不再硬编码 sys.path.insert 到 muscles-braincontrol。"""
    import wheelchair_app.tabs.braincontrol_tab as mod
    src = open(mod.__file__).read()
    assert 'muscles-braincontrol' not in src, \
        "braincontrol_tab.py 仍含硬编码外部路径"


def test_no_port_combo_widgets(qt_app, ros_node):
    """端口下拉已移除（自动识别替代）。"""
    from wheelchair_app.tabs.braincontrol_tab import BrainControlTab
    tab = BrainControlTab(ros_node=ros_node)
    assert not hasattr(tab, '_eeg_port_combo'), "EEG 端口下拉应已移除"
    assert not hasattr(tab, '_imu_port_combo'), "IMU 端口下拉应已移除"
    assert not hasattr(tab, '_refresh_ports_button'), "刷新端口按钮应已移除"


def test_initial_status_labels_pending(qt_app, ros_node):
    """启动时状态 label 显示'待扫描'。"""
    from wheelchair_app.tabs.braincontrol_tab import BrainControlTab
    tab = BrainControlTab(ros_node=ros_node)
    assert '待扫描' in tab._eeg_status_label.text()
    assert '待扫描' in tab._imu_status_label.text()


def test_auto_detect_devices_method_exists(qt_app, ros_node):
    """自动识别方法存在。"""
    from wheelchair_app.tabs.braincontrol_tab import BrainControlTab
    tab = BrainControlTab(ros_node=ros_node)
    assert hasattr(tab, '_auto_detect_devices')
    assert hasattr(tab, '_test_imu_port')
    assert hasattr(tab, '_test_eeg_port')


def test_no_navigation_toolbar(qt_app, ros_node):
    """NavigationToolbar 已删除（matplotlib 工具条不贴主题）。"""
    from wheelchair_app.tabs.braincontrol_tab import BrainControlTab
    tab = BrainControlTab(ros_node=ros_node)
    assert not hasattr(tab, '_nav_toolbar'), "NavigationToolbar 应已删除"
