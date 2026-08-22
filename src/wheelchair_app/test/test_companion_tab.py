"""CompanionTab 单元测试(QWebEngineView 版本)。

验证:
  - 加载正确的 URL(http://localhost:8000/companion/index.html)
  - 不需要 ROS 数据(网页内部自己通过 roslibjs 连)
"""
import os
import sys
from unittest.mock import MagicMock

# 必须在 QApplication 之前 import QtWebEngineWidgets,否则 PyQt5 报错。
# pytest collection 顺序下,test_main_window 可能先创建 QApplication,所以这里
# 在模块顶部强制 import,确保 QtWebEngine 进程子系统在 QApplication 之前注册。
from PyQt5.QtWebEngineWidgets import QWebEngineView  # noqa: F401

import pytest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')


@pytest.fixture
def companion_tab():
    from PyQt5.QtWidgets import QApplication
    from wheelchair_app.tabs.companion_tab import CompanionTab

    app = QApplication.instance() or QApplication(sys.argv)
    tab = CompanionTab(MagicMock())
    yield tab
    tab.close()


def test_companion_tab_loads_companion_url(companion_tab):
    """应加载 http://localhost:8000/companion/index.html。

    load() 是异步的,刚 init 完 url() 可能还是空,所以只验证 URL 常量。
    """
    assert companion_tab.URL == "http://localhost:8000/companion/index.html"


def test_companion_tab_accepts_ros_node(companion_tab):
    """构造时应接受 ros_node 参数(供未来扩展,不直接崩)。"""
    assert companion_tab._ros is not None
