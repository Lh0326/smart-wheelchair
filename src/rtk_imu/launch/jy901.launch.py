"""JY901 IMU 单节点启动。

用法：
    ros2 launch rtk_imu jy901.launch.py
    ros2 launch rtk_imu jy901.launch.py port:=/dev/ttyUSB0 baud:=9600
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('rtk_imu')
    default_params = os.path.join(pkg_dir, 'config', 'jy901_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyIMU'),
        DeclareLaunchArgument('baud', default_value='921600'),

        Node(
            package='rtk_imu',
            executable='jy901_driver',
            name='jy901_driver',
            output='screen',
            parameters=[
                default_params,
                {
                    'port': LaunchConfiguration('port'),
                    'baud': PythonExpression(['int("', LaunchConfiguration('baud'), '")']),
                },
            ],
        ),
    ])
