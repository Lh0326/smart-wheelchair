"""NetworkX planner 节点 launch（可独立启动用于调试）
import os as _os
def _find_ws_root():
    r = _os.environ.get("WS_ROOT")
    if r: return r
    d = _os.path.dirname(_os.path.abspath(__file__))
    for _ in range(6):
        if _os.path.exists(_os.path.join(d, "env.sh")): return d
        d = _os.path.dirname(d)
    return d
_WS_ROOT = _find_ws_root()
_MODELS_ROOT = _os.environ.get("MODELS_ROOT", _os.path.join(_WS_ROOT, "models"))


完全离线路径规划，不需要 OSRM Docker，也不需要联网。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'osm_path',
            default_value=_WS_ROOT + '/data/region.graphml',
            description='OSM GraphML 文件路径（启动时加载到内存）',
        ),
        DeclareLaunchArgument(
            'walking_speed_mps',
            default_value='1.4',
            description='步行速度（m/s），用于算时长。1.4 m/s ≈ 5 km/h',
        ),
        DeclareLaunchArgument(
            'min_goal_distance_m',
            default_value='5.0',
            description='小于此距离不规划，返回单点 noop',
        ),

        Node(
            package='rtk_planner',
            executable='networkx_planner_node',
            name='networkx_planner',
            parameters=[{
                'osm_path': LaunchConfiguration('osm_path'),
                'walking_speed_mps': LaunchConfiguration('walking_speed_mps'),
                'min_goal_distance_m': LaunchConfiguration('min_goal_distance_m'),
            }],
        ),
    ])
