#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

'''
Parameter Description:
---
- Set laser scan directon: 
  1. Set counterclockwise, example: {'laser_scan_dir': True}
  2. Set clockwise,        example: {'laser_scan_dir': False}
- Angle crop setting, Mask data within the set angle range:
  1. Enable angle crop fuction:
    1.1. enable angle crop,  example: {'enable_angle_crop_func': True}
    1.2. disable angle crop, example: {'enable_angle_crop_func': False}
  2. Angle cropping interval setting:
  - The distance and intensity data within the set angle range will be set to 0.
  - angle >= 'angle_crop_min' and angle <= 'angle_crop_max' which is [angle_crop_min, angle_crop_max], unit is degress.
    example:
      {'angle_crop_min': 135.0}
      {'angle_crop_max': 225.0}
      which is [135.0, 225.0], angle unit is degress.
'''

def generate_launch_description():
  # LDROBOT LiDAR publisher node
  ldlidar_node = Node(
      package='ldlidar',
      executable='ldlidar',
      name='ldlidar_publisher_ld14p',
      output='screen',
      parameters=[
        {'product_name': 'LDLiDAR_LD14P'},
        {'topic_name': 'scan_ld14p'},
        {'port_name': '/dev/LD14P'},
        {'frame_id': 'ld14p_link'},
        {'laser_scan_dir': True},
        {'enable_angle_crop_func': False},#单角度裁剪开关：值为False时表示不使用多角度裁剪，默认为False
        {'angle_crop_min': 135.0},#单角度裁剪开始值
        {'angle_crop_max': 225.0},#单角度裁剪结束值
        {'truncated_mode_': 0}#值为1表示使用多角度裁剪，同时enable_angle_crop_func设为False，角度值在/main.cpp中修改
      ]
  )

  # base_link to ld14p_link tf node
  base_link_to_laser_tf_node = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_link_to_ld14p_link',
    arguments=['0','0','-0.3','0','0','0','base_link','ld14p_link']
  )

  scan_fre_node = ExecuteProcess(
    cmd=['ros2','run','ldlidar','LD14P_scan_fre.py']
  )


  # Define LaunchDescription variable
  ld = LaunchDescription()

  #ld.add_action(scan_fre_node)  #<!--调节雷达扫描频率，scan_fre扫描频率与雷达串口号请在LD14P_scan_fre.py文件中修改-->
  ld.add_action(ldlidar_node)
  ld.add_action(base_link_to_laser_tf_node)

  return ld