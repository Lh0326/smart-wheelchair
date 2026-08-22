"""启动 robot_localization EKF：GPS 融合 + IMU

数据流：
  gps_fusion → /fix → navsat_transform → /odometry/gps → ekf_filter → /odometry/filtered
  imu_heading → /imu/data ──────────────────────────────→ ekf_filter
  ekf_filter → /odometry/filtered → navsat_transform → /gps/filtered（WGS-84 修正后）

启动前确保：
  - gps_fusion 节点在跑（发布 /fix）
  - imu_heading 节点在跑（发布 /imu/data + /heading_imu）
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rtk_gnss_dir = get_package_share_directory('rtk_gnss')
    navsat_yaml = os.path.join(rtk_gnss_dir, 'config', 'navsat_transform.yaml')
    ekf_yaml = os.path.join(rtk_gnss_dir, 'config', 'ekf.yaml')

    return LaunchDescription([
        # navsat_transform: WGS-84 ↔ map frame
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[
                navsat_yaml,
                {
                    # 显式指定 topic（避免命名混淆）
                    'use_odometry_yaw': False,
                },
            ],
            remappings=[
                ('gps/fix', '/fix'),                         # navsat 默认订阅 gps/fix → 我们的 /fix
                ('imu/data', '/imu/data'),                   # IMU
                ('odometry/filtered', '/odometry/filtered'), # EKF 反向
                # 输出 topic 保持默认：odometry/gps, gps/filtered
            ],
        ),

        # ekf_filter: 融合 /odometry/gps + /imu/data → /odometry/filtered
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                ekf_yaml,
                {
                    'frequency': 30.0,
                    'two_d_mode': True,
                    'publish_tf': False,
                },
            ],
            remappings=[
                ('odometry/filtered', '/odometry/filtered'),
            ],
        ),
    ])
