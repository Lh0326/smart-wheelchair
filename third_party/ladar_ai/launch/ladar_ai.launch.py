"""Ladar-AI 系统启动文件。"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="ladar_ai",
            executable="lidar_zone_node",
            name="lidar_zone_node",
            output="screen",
        ),
        Node(
            package="ladar_ai",
            executable="camera_detect_node",
            name="camera_detect_node",
            output="screen",
        ),
        Node(
            package="ladar_ai",
            executable="voice_node",
            name="voice_node",
            output="screen",
        ),
        Node(
            package="ladar_ai",
            executable="fusion_decision_node",
            name="fusion_decision_node",
            output="screen",
        ),
        Node(
            package="ladar_ai",
            executable="tts_node",
            name="tts_node",
            output="screen",
        ),
        Node(
            package="ladar_ai",
            executable="web_node",
            name="web_node",
            output="screen",
        ),
    ])
