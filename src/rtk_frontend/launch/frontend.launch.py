"""启动前端：静态文件服务 + rosbridge_suite"""
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rosbridge_dir = get_package_share_directory('rosbridge_server')

    return LaunchDescription([
        # rosbridge_suite（WebSocket 桥，端口 9091，绕过 9090 诡异占用）
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(rosbridge_dir, 'launch', 'rosbridge_websocket_launch.xml')
            ),
            launch_arguments={'port': '9091'}.items(),
        ),

        # 前端静态文件服务
        Node(
            package='rtk_frontend',
            executable='frontend_server',
            name='frontend_static_server',
            parameters=[{'port': 8000, 'host': '0.0.0.0'}],
            output='screen',
        ),
    ])
