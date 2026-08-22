"""启动 Gazebo Fortress + bcr_bot + 传感器桥接 + depthimage_to_laserscan。

前置：wheelchair_nav_ws 已构建并 source（提供 bcr_bot 包）。
本机环境：Gazebo Garden 6.16.0（gz_version=6，启动器走 ign gazebo 路径）。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ladar_ai_share = get_package_share_directory("ladar_ai")
    world_file = os.path.join(ladar_ai_share, "worlds", "static_room.sdf")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    # 通过 ros_gz_sim 的 gz_sim.launch.py 启动，避免 gz/ign 命令混淆
    # gz_version=6 → 走 ign gazebo 路径（适配本机 Garden）
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": f"{world_file} -r -v 2",
            "gz_version": "6",
        }.items(),
    )

    bcr_bot_share = get_package_share_directory("bcr_bot")
    bcr_bot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bcr_bot_share, "launch", "bcr_bot_gz_spawn.launch.py")
        ),
        launch_arguments={
            "position_x": "-2.5",
            "position_y": "-1.5",
            "orientation_yaw": "0.5",
            "two_d_lidar_enabled": "true",
            "camera_enabled": "true",
            "odometry_source": "world",
        }.items(),
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        arguments=[
            "/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan",
            "/camera/depth/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image",
            "/camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",
            "/odom@nav_msgs/msg/Odometry@ignition.msgs.Odometry",
            "/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
            "/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock",
        ],
        output="screen",
    )

    depth_to_laser = Node(
        package="depthimage_to_laserscan",
        executable="depthimage_to_laserscan_node",
        name="depthimage_to_laserscan",
        remappings=[
            ("depth_image", "/camera/depth/image_raw"),
            ("depth", "/camera/depth/image_raw"),
            ("image", "/camera/depth/image_raw"),
            ("scan", "/scan_depth"),
        ],
        parameters=[
            {
                "output_frame": "camera_link",
                "range_min": 0.15,
                "range_max": 5.0,
                "scan_height": 10,
            }
        ],
        output="screen",
    )

    # bcr_bot gz_spawn.launch.py 内置 bridge 把话题加了 /bcr_bot/ 前缀，
    # 用 relay 把话题名统一到无前缀（实物迁移时不用改代码）
    from launch.actions import ExecuteProcess
    relay_odom = ExecuteProcess(
        cmd=["ros2", "run", "topic_tools", "relay", "/bcr_bot/odom", "/odom"],
        output="screen",
    )
    relay_scan = ExecuteProcess(
        cmd=["ros2", "run", "topic_tools", "relay", "/bcr_bot/scan", "/scan"],
        output="screen",
    )
    relay_cmd_vel = ExecuteProcess(
        cmd=["ros2", "run", "topic_tools", "relay", "/cmd_vel", "/bcr_bot/cmd_vel"],
        output="screen",
    )

    return LaunchDescription([
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", ladar_ai_share),
        gz_sim,
        bcr_bot_spawn,
        bridge,
        depth_to_laser,
        relay_odom,
        relay_scan,
        relay_cmd_vel,
    ])
