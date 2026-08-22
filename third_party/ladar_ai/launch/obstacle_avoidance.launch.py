"""启动避障栈：laser_merger + obstacle_avoidance。

teleop_twist_keyboard 需要 TTY，不能在 launch 中后台运行。
请在单独的终端手动运行：
    ros2 run teleop_twist_keyboard teleop_twist_keyboard \
        --ros-args -r /cmd_vel:=/teleop_cmd_vel
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ladar_ai_share = get_package_share_directory("ladar_ai")
    vfh_params = os.path.join(ladar_ai_share, "config", "vfh_params.yaml")

    laser_merger = Node(
        package="ladar_ai",
        executable="laser_merger_node",
        name="laser_merger_node",
        parameters=[vfh_params],
        output="screen",
    )

    obstacle_avoidance = Node(
        package="ladar_ai",
        executable="obstacle_avoidance_node",
        name="obstacle_avoidance_node",
        parameters=[vfh_params],
        output="screen",
    )

    return LaunchDescription([laser_merger, obstacle_avoidance])
