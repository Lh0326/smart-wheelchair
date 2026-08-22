"""启动 mbtiles-server"""
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    default_mbtiles = os.path.expanduser(_WS_ROOT + '/data/region.mbtiles')

    return LaunchDescription([
        DeclareLaunchArgument(
            'mbtiles',
            default_value=default_mbtiles,
            description='mbtiles 文件路径',
        ),
        DeclareLaunchArgument(
            'port',
            default_value='8080',
            description='HTTP 服务端口',
        ),
        Node(
            package='rtk_map',
            executable='mbtiles_server',
            name='mbtiles_server',
            parameters=[{
                'mbtiles': LaunchConfiguration('mbtiles'),
                'port': LaunchConfiguration('port'),
            }],
            output='screen',
        ),
    ])
