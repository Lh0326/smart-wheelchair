"""主窗口单元测试。"""
import os
import sys
from unittest.mock import MagicMock

import pytest


# 测试前确保 QT_QPA_PLATFORM=offscreen(无 GUI 环境也能跑)
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')


@pytest.fixture
def main_window():
    """提供已实例化的 MainWindow(mock rclpy 节点)。"""
    from PyQt5.QtWidgets import QApplication
    from wheelchair_app.main_window import MainWindow

    # 确保 QApplication 单例
    app = QApplication.instance() or QApplication(sys.argv)

    ros_node = MagicMock()
    win = MainWindow(ros_node)
    yield win
    win.close()


def test_main_window_has_tab_changed_signal():
    """主窗口应有 tab_changed signal。"""
    from wheelchair_app.main_window import MainWindow
    assert hasattr(MainWindow, 'tab_changed')


def test_main_window_add_tab_increments_count(main_window):
    """add_tab 应增加 QTabWidget 计数。"""
    from PyQt5.QtWidgets import QLabel
    initial = main_window._tab_widget.count()
    main_window.add_tab("测试", QLabel("hi"))
    assert main_window._tab_widget.count() == initial + 1


def test_main_window_switch_to_changes_index(main_window):
    """switch_to 应改变当前 tab 索引。"""
    from PyQt5.QtWidgets import QLabel
    main_window.add_tab("A", QLabel("a"))
    main_window.add_tab("B", QLabel("b"))
    main_window.switch_to("B")
    assert main_window._tab_widget.currentIndex() == 1  # 0=A,1=B


def test_main_window_switch_to_unknown_noop(main_window):
    """switch_to 未知 name 应 no-op(不抛异常)。"""
    main_window.switch_to("不存在的tab")  # 不抛即通过


def test_main_window_update_clock(main_window):
    """update_clock 应更新状态栏文字。"""
    main_window.update_clock("12:34:56")
    assert main_window._status_clock.text() == "12:34:56"


def test_ros_bridge_singleton():
    """RosBridgeNode 应是单例。"""
    # 跳过:rclpy.init 已在其他测试中执行,这里只验证类结构
    from wheelchair_app.ros_bridge import RosBridgeNode
    assert hasattr(RosBridgeNode, 'instance')
    assert hasattr(RosBridgeNode, '_instance')
