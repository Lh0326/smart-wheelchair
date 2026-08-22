"""IMU 数据源三选一启动（jy901/gemini/sim_chassis）。

用法：
    # 默认：JY901 实物 IMU
    ros2 launch rtk_imu imu_selector.launch.py

    # 切回 Gemini 335L 相机 IMU（imu_heading_node）
    ros2 launch rtk_imu imu_selector.launch.py use_jy901:=false use_gemini:=true

    # 仿真模式（sim_chassis 已在 sim_navigation 栈中启动，本 launch 不重复启动）
    ros2 launch rtk_imu imu_selector.launch.py use_jy901:=false use_gemini:=false
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rtk_imu_dir = get_package_share_directory('rtk_imu')
    rtk_gnss_dir = get_package_share_directory('rtk_gnss')

    use_jy901_arg = DeclareLaunchArgument(
        'use_jy901', default_value='true',
        description='启动 JY901 实物 IMU 驱动',
    )
    use_gemini_arg = DeclareLaunchArgument(
        'use_gemini', default_value='false',
        description='启动 Gemini 335L 相机 IMU 驱动（imu_heading_node）',
    )

    use_jy901 = LaunchConfiguration('use_jy901')
    use_gemini = LaunchConfiguration('use_gemini')

    # JY901 实物 IMU
    jy901_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rtk_imu_dir, 'launch', 'jy901.launch.py')
        ),
        condition=IfCondition(use_jy901),
    )

    # Gemini 335L 相机 IMU（imu_heading_node + orbbec_camera）
    gemini_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rtk_gnss_dir, 'launch', 'orbbec_camera.launch.py')
        ),
        condition=IfCondition(use_gemini),
    )
    gemini_imu_node = Node(
        package='rtk_gnss',
        executable='imu_heading',
        name='imu_heading',
        output='screen',
        condition=IfCondition(use_gemini),
    )

    return LaunchDescription([
        use_jy901_arg,
        use_gemini_arg,
        jy901_launch,
        gemini_camera_launch,
        gemini_imu_node,
    ])
