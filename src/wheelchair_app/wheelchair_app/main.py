"""wheelchair_app 主入口:PyQt5 + rclpy。"""
import os
# cv2(OpenCV)自带 Qt 库会覆盖 PyQt5 的 platform plugin 路径,
# 必须在 import cv2 之前显式指定 PyQt5 用系统 Qt5 plugins。
# 文件级影响:任何间接 import cv2 的子模块(companion_tab)都会触发此保护。
os.environ.setdefault(
    'QT_QPA_PLATFORM_PLUGIN_PATH',
    '/usr/lib/x86_64-linux-gnu/qt5/plugins'
)
import sys
import traceback
from datetime import datetime

from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QTimer

from wheelchair_app.main_window import MainWindow
from wheelchair_app.tabs.navigation_tab import NavigationTab
from wheelchair_app.tabs.companion_tab import CompanionTab
from wheelchair_app.ros_bridge import init_rclpy, shutdown_rclpy

SKIP_BRAINCONTROL = os.environ.get('SKIP_BRAINCONTROL', '0') == '1'
if not SKIP_BRAINCONTROL:
    from wheelchair_app.tabs.braincontrol_tab import BrainControlTab


def main():
    """启动顺序:rclpy → Qt → 窗口 → tabs → 时钟 → 显示 → exec → 清理。

    rclpy 必须先 init(否则 RosBridgeNode 构造失败);
    shutdown_rclpy 在 finally 中执行,确保异常时也清理。
    """
    ros_node = init_rclpy()
    exit_code = 0
    try:
        app = QApplication(sys.argv)
        win = MainWindow(ros_node)

        win.add_tab("自主导航", NavigationTab(ros_node))
        win.add_tab("小智陪伴", CompanionTab(ros_node))
        if not SKIP_BRAINCONTROL:
            win.add_tab("脑电控制", BrainControlTab(ros_node))

        clock_timer = QTimer()
        clock_timer.timeout.connect(
            lambda: win.update_clock(datetime.now().strftime("%H:%M:%S"))
        )
        clock_timer.start(1000)

        # 硬件监控由 MainWindow._hw_timer 直接采集 sysfs/psutil（CGN-watching 方案），
        # 不再依赖 /hw_status topic → 无 DDS 延迟、无 subscription GC 问题、数据准确。

        win.show()
        exit_code = app.exec_()
    except Exception:
        traceback.print_exc()
        exit_code = 1
    finally:
        shutdown_rclpy()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()
