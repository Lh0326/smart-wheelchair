"""MotionCommander：将 MotionCommand 枚举转换为 Twist 发布到 /cmd_vel_eeg。

搬迁自 muscles-braincontrol，原 update() 是 logger.info 桩。
rtk 端改造：加 rclpy publisher，发布 geometry_msgs/Twist。

速度约定（与 sim_navigation_teb.launch.py TEB max_vel_x=0.6 保守一致）：
    FORWARD:  linear.x = +0.5
    BACKWARD: linear.x = -0.3
    LEFT:     angular.z = +0.5
    RIGHT:    angular.z = -0.5
    STOP:     全 0
"""
import logging
import time
from typing import Dict

from geometry_msgs.msg import Twist, Vector3

from .control_types import MotionCommand

logger = logging.getLogger(__name__)


# Twist 工厂（避免每次构造 Vector3）
def _twist(linear_x: float = 0.0, angular_z: float = 0.0) -> Twist:
    return Twist(
        linear=Vector3(x=linear_x),
        angular=Vector3(z=angular_z),
    )


# MotionCommand → Twist 映射表
_VEL_MAP: Dict[MotionCommand, Twist] = {
    MotionCommand.FORWARD:  _twist(linear_x=+0.5),
    MotionCommand.BACKWARD: _twist(linear_x=-0.3),
    MotionCommand.LEFT:     _twist(angular_z=+0.5),
    MotionCommand.RIGHT:    _twist(angular_z=-0.5),
    MotionCommand.STOP:     _twist(),
}


class MotionCommander:
    """接收 MotionCommand，发布对应 Twist 到 /cmd_vel_eeg。

    使用方式：
        commander = MotionCommander(ros_node=some_rclpy_node)
        commander.update(MotionCommand.FORWARD)
    """

    def __init__(self, ros_node):
        """初始化 commander。

        Args:
            ros_node: rclpy Node 实例（用于 create_publisher 和 get_clock）。
        """
        self._node = ros_node
        # 控制类 topic 只保留最新帧，避免 UI/ROS 短时抖动后补发旧速度。
        self._pub = ros_node.create_publisher(Twist, '/cmd_vel_eeg', 1)
        self._publish_count = 0
        self._last_publish_time = 0.0

    def update(self, cmd: MotionCommand) -> None:
        """发布 cmd 对应的 Twist 到 /cmd_vel_eeg。

        Args:
            cmd: MotionCommand 枚举值。
        """
        twist = _VEL_MAP[cmd]
        self._pub.publish(twist)
        self._publish_count += 1
        self._last_publish_time = time.time()
        logger.debug(f"MotionCommand published: {cmd.name} -> "
                     f"linear.x={twist.linear.x}, angular.z={twist.angular.z} "
                     f"(total={self._publish_count})")

    @property
    def publish_count(self) -> int:
        """累计发布次数（GUI 显示用）。"""
        return self._publish_count

    @property
    def last_publish_time(self) -> float:
        """上次发布时间戳（GUI 显示用，fallback 监测用）。"""
        return self._last_publish_time
