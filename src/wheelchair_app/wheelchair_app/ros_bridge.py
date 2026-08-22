"""rclpy Node 单例,通过 QTimer 周期 spin_once,避免阻塞 Qt 事件循环。"""
import rclpy
from rclpy.node import Node
from PyQt5.QtCore import QTimer, QObject, pyqtSignal


class RosBridgeNode(Node, QObject):
    """rclpy + Qt 桥接节点。

    单例模式:整个应用只有一个 RosBridgeNode 实例。
    QTimer 周期调用 rclpy.spin_once(self, 0.005s),避免阻塞 Qt。

    已知 warning:Node + QObject 多重继承会触发 QObject::startTimer 警告
    (Qt 元系统在 rclpy C 扩展对象上装 timer 钩子),不影响功能。
    后续如出现 race condition 可重构为组合模式(RosBridge 持有 Node 而非继承)。
    """

    _instance = None

    @classmethod
    def instance(cls):
        if cls._instance is None:
            cls._instance = cls()
        return cls._instance

    def __init__(self):
        Node.__init__(self, 'wheelchair_app_bridge')
        QObject.__init__(self)

        # QTimer 周期 spin_once(60Hz)
        self._timer = QTimer()
        self._timer.timeout.connect(self._spin_once)
        self._timer.start(16)  # ~60Hz

    def _spin_once(self):
        """处理 ROS2 待处理回调(5ms 超时,留余量处理高频话题)。"""
        rclpy.spin_once(self, timeout_sec=0.005)


def init_rclpy():
    """初始化 rclpy(幂等)。"""
    if not rclpy.ok():
        rclpy.init()
    return RosBridgeNode.instance()


def shutdown_rclpy():
    """关闭 rclpy(顺序:先停 QTimer,再 destroy_node,最后 shutdown)。"""
    if RosBridgeNode._instance is not None:
        RosBridgeNode._instance._timer.stop()  # 先停 QTimer 避免 spin 卡死
        RosBridgeNode._instance.destroy_node()
        RosBridgeNode._instance = None
    if rclpy.ok():
        rclpy.shutdown()
