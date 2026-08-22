"""M1.5 端到端：mbtiles + 前端 + rosbridge + EC20 GNSS + Gemini 335L"""
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

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rtk_map_dir = get_package_share_directory('rtk_map')
    rtk_frontend_dir = get_package_share_directory('rtk_frontend')
    rtk_gnss_dir = get_package_share_directory('rtk_gnss')

    enable_planner_arg = DeclareLaunchArgument(
        'enable_planner',
        default_value='true',
        description='是否启动 planner 节点',
    )

    planner_engine_arg = DeclareLaunchArgument(
        'planner_engine',
        default_value='networkx',
        description='路径规划引擎: networkx（完全离线，推荐）或 osrm（依赖本地/公共 OSRM）',
        choices=['networkx', 'osrm'],
    )

    # 启用 planner 且 engine=osrm 时启动
    osrm_enabled = PythonExpression([
        "'", LaunchConfiguration('enable_planner'),
        "' == 'true' and '",
        LaunchConfiguration('planner_engine'), "' == 'osrm'"
    ])
    osrm_planner_node = Node(
        package='rtk_planner',
        executable='osrm_planner_node',
        name='osrm_planner',
        parameters=[{
            'local_osrm_url': 'http://localhost:5000',
            'public_osrm_url': 'https://router.project-osrm.org',
            'osrm_profile': 'walking',
            'request_timeout': 3.0,
            'min_goal_distance_m': 5.0,
            'enable_public_fallback': True,
        }],
        condition=IfCondition(osrm_enabled),
    )

    # 启用 planner 且 engine=networkx 时启动
    networkx_enabled = PythonExpression([
        "'", LaunchConfiguration('enable_planner'),
        "' == 'true' and '",
        LaunchConfiguration('planner_engine'), "' == 'networkx'"
    ])
    networkx_planner_node = Node(
        package='rtk_planner',
        executable='networkx_planner_node',
        name='networkx_planner',
        parameters=[{
            'osm_path': _WS_ROOT + '/data/region.graphml',
            'walking_speed_mps': 1.4,
            'min_goal_distance_m': 5.0,
        }],
        condition=IfCondition(networkx_enabled),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'mbtiles',
            default_value=_WS_ROOT + '/data/region.mbtiles',
            description='mbtiles 文件路径',
        ),
        DeclareLaunchArgument(
            'use_fake_gnss',
            default_value='false',
            description='true=用 fake_gnss 演示，false=用真实 EC20F GNSS',
        ),

        # mbtiles-server（瓦片图离线服务，备用——前端默认用高德在线瓦片）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rtk_map_dir, 'launch', 'mbtiles_server.launch.py')
            ),
            launch_arguments={'mbtiles': LaunchConfiguration('mbtiles')}.items(),
        ),

        # 前端 + rosbridge
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rtk_frontend_dir, 'launch', 'frontend.launch.py')
            ),
        ),

        # 真实 EC20F GNSS 节点（use_fake_gnss=false 时启动）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rtk_gnss_dir, 'launch', 'ec20_gnss.launch.py')
            ),
            condition=UnlessCondition(LaunchConfiguration('use_fake_gnss')),
        ),

        # fake_gnss 演示节点（use_fake_gnss=true 时启动，与虚拟底盘开发栈配套）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rtk_gnss_dir, 'launch', 'fake_gnss.launch.py')
            ),
            condition=IfCondition(LaunchConfiguration('use_fake_gnss')),
        ),

        # Orbbec Gemini 335L 深度相机 + IMU
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rtk_gnss_dir, 'launch', 'orbbec_camera.launch.py')
            ),
        ),

        # 全局规划节点（/global_plan latched QoS）
        # 默认用 networkx（完全离线），传 planner_engine:=osrm 可回退到 OSRM 行为
        enable_planner_arg,
        planner_engine_arg,
        osrm_planner_node,
        networkx_planner_node,
    ])
