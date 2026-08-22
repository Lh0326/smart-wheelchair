"""wheelchair_app 一键启动:PyQt5 main + rosbridge + static_server。
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


用法:
  ros2 launch wheelchair_app wheelchair_app.launch.py

启动后:
  - 主窗口加载 http://localhost:8000/{nav,companion}/index.html
  - 状态栏显示时钟
  - 3 个 tab 切换:自主导航 / 小智陪伴 / 脑电控制
"""
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rosbridge_dir = get_package_share_directory('rosbridge_server')

    return LaunchDescription([
        # 1. rosbridge WebSocket(端口 9091,项目特殊端口)
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(rosbridge_dir, 'launch', 'rosbridge_websocket_launch.xml')
            ),
            launch_arguments={'port': '9091'}.items(),
        ),

        # 2. 前端静态文件服务(端口 8000,提供 /nav/ 和 /companion/)
        Node(
            package='rtk_frontend',
            executable='frontend_server',
            name='frontend_static_server',
            parameters=[{'port': 8000, 'host': '0.0.0.0'}],
            output='screen',
        ),

        # 3. PyQt5 主应用
        Node(
            package='wheelchair_app',
            executable='main',
            name='wheelchair_main',
            output='screen',
            # PyQt5 需要 DISPLAY 环境变量
            additional_env={'DISPLAY': os.environ.get('DISPLAY', ':0')},
        ),

        # 4. 硬件监控节点（发布 /hw_status 供前端 hw-strip 显示）
        Node(
            package='wheelchair_app',
            executable='hw_monitor_node',
            name='hw_monitor_node',
            output='screen',
        ),

        # 5. 路径语音播报节点（订阅 /global_plan + /fix + /clear_goal，
        #    发布 /tts_request 由 ladar_ai/tts_node 发声）
        Node(
            package='wheelchair_app',
            executable='voice_announce_node',
            name='voice_announce_node',
            output='screen',
            parameters=[{
                'graphml_path': _WS_ROOT + '/data/region.graphml',
                'turn_ahead_meters': 5.0,
                'arrival_meters': 3.0,
                'offroute_meters': 25.0,
            }],
        ),
    ])
