"""启动 fake_gnss 演示节点 + IMU 航向积分节点

跟 ec20_gnss.launch.py 对称——无论用真实 GNSS 还是虚拟 GNSS，
IMU 航向积分（依赖 Gemini 335L，不依赖 GNSS 硬件）都应该启动。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('center_lat', default_value='24.8551'),
        DeclareLaunchArgument('center_lon', default_value='102.8553'),
        DeclareLaunchArgument('speed_mps', default_value='0.5'),
        Node(
            package='rtk_gnss',
            executable='fake_gnss',
            name='fake_gnss',
            parameters=[{
                'center_lat': LaunchConfiguration('center_lat'),
                'center_lon': LaunchConfiguration('center_lon'),
                'speed_mps': LaunchConfiguration('speed_mps'),
                'rect_width': 40.0,
                'rect_height': 30.0,
                'publish_hz': 1.0,
            }],
            output='screen',
        ),
        Node(
            package='rtk_gnss',
            executable='imu_heading',
            name='imu_heading',
            parameters=[{
                'gyro_axis': 'y',
                'calibration_time': 3.0,
                'initial_yaw': 0.0,
                'use_manual_bias': False,
            }],
            output='screen',
        ),
    ])
