"""启动真实 EC20F GNSS 节点

前置条件：
  - EC20 dongle 已插入（lsusb 显示 2c7c:0125）
  - /dev/ttyUSB_AT 是 AT 端口，/dev/ttyUSB_NMEA 是 NMEA 端口
  - dongle 放在窗户边或室外（室内信号弱）

heading 由 rtk_imu 包的 jy901_driver 提供（HWT906P 磁力计绝对方位），
本 launch 不再启动 imu_heading（Gemini 335L 陀螺仪积分），避免 /heading_imu 多发布者冲突。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('at_port', default_value='/dev/ttyUSB_AT'),
        DeclareLaunchArgument('nmea_port', default_value='/dev/ttyUSB_NMEA'),
        DeclareLaunchArgument('output_topic', default_value='/fix',
                              description='NavSatFix 发布话题。DGPS 模式下设为 /gps/base_raw'),
        Node(
            package='rtk_gnss',
            executable='ec20_gnss',
            name='ec20_gnss',
            parameters=[{
                'at_port': LaunchConfiguration('at_port'),
                'nmea_port': LaunchConfiguration('nmea_port'),
                'baudrate': 115200,
                'output_topic': LaunchConfiguration('output_topic'),
            }],
            output='screen',
        ),
    ])
