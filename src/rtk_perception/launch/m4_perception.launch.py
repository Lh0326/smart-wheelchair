"""M4 感知与避障：一键启动 robot_state_publisher + 5 个感知节点 + RViz。

启动项：
  - robot_state_publisher：发布 /robot_description 与静态 TF
  - fusion_scan_node     ：/scan + /scan_ld14p -> /scan_fused
  - curb_detector_node   ：/scan_fused -> /curb_left_marker, /curb_right_marker, /curb_polygon
  - target_heading_node  ：订阅目标点 -> /target_heading
  - vfh_avoidance_node   ：/scan_fused + /target_heading -> /cmd_vel, /vfh_histogram, /vfh_candidate
  - safety_chain_node    ：/scan_fused + /cmd_vel -> /cmd_vel_safe
  - rviz2                ：加载 perception.rviz
"""
import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('rtk_perception')
    urdf = os.path.join(pkg_dir, 'urdf', 'wheelchair.urdf.xacro')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'perception.rviz')
    params_file = os.path.join(pkg_dir, 'config', 'perception_params.yaml')

    with open(urdf, 'r') as f:
        robot_desc = f.read()

    return LaunchDescription([
        # URDF -> robot_state_publisher（发布 /robot_description 与 TF）
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            output='screen',
        ),

        # N10P driver 的 frame_id 是 "laser"（lslidar yaml 默认），加一个静态 TF
        # 把 laser frame 链到 base_link（N10P 在中心后方 0.255m，高 1.47m）
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser',
            arguments=['-0.255', '0', '1.47', '0', '0', '0', 'base_link', 'laser'],
        ),

        # 感知节点（4 个，fusion_scan_node 已取消——两个雷达在不同高度不应融合）
        Node(
            package='rtk_perception',
            executable='curb_detector_node',
            name='curb_detector_node',
            parameters=[params_file],
            output='screen',
        ),
        Node(
            package='rtk_perception',
            executable='path_to_baselink_node',
            name='path_to_baselink_node',
            parameters=[{
                'lookahead_distance_m': 2.0,
                'update_rate_hz': 10.0,
            }],
            output='screen',
        ),
        Node(
            package='rtk_perception',
            executable='vfh_avoidance_node',
            name='vfh_avoidance_node',
            parameters=[params_file],
            output='screen',
        ),
        Node(
            package='rtk_perception',
            executable='safety_chain_node',
            name='safety_chain_node',
            parameters=[params_file],
            output='screen',
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
