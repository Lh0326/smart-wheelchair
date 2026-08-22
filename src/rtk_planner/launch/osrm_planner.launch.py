"""OSRM planner 节点 launch（可独立启动用于调试）"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('local_osrm_url', default_value='http://localhost:5000'),
        DeclareLaunchArgument('public_osrm_url', default_value='https://router.project-osrm.org'),
        DeclareLaunchArgument('osrm_profile', default_value='walking'),
        DeclareLaunchArgument('request_timeout', default_value='3.0'),
        DeclareLaunchArgument('min_goal_distance_m', default_value='5.0'),
        DeclareLaunchArgument('enable_public_fallback', default_value='true'),

        Node(
            package='rtk_planner',
            executable='osrm_planner_node',
            name='osrm_planner',
            parameters=[{
                'local_osrm_url': LaunchConfiguration('local_osrm_url'),
                'public_osrm_url': LaunchConfiguration('public_osrm_url'),
                'osrm_profile': LaunchConfiguration('osrm_profile'),
                'request_timeout': LaunchConfiguration('request_timeout'),
                'min_goal_distance_m': LaunchConfiguration('min_goal_distance_m'),
                'enable_public_fallback': LaunchConfiguration('enable_public_fallback'),
            }],
        ),
    ])
