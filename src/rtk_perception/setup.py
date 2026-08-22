import os
from glob import glob
from setuptools import setup

package_name = 'rtk_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf.xacro')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='BrainControl Developer',
    maintainer_email='dev@braincontrol.local',
    description='M4 感知与避障',
    license='MIT',
    entry_points={
        'console_scripts': [
            'fusion_scan_node = rtk_perception.fusion_scan_node:main',
            'curb_detector_node = rtk_perception.curb_detector_node:main',
            'target_heading_node = rtk_perception.target_heading_node:main',
            'vfh_avoidance_node = rtk_perception.vfh_avoidance_node:main',
            'safety_chain_node = rtk_perception.safety_chain_node:main',
            'path_to_baselink_node = rtk_perception.path_to_baselink_node:main',
            'sim_chassis_node = rtk_perception.sim_chassis_node:main',
            'chassis_serial_node = rtk_perception.chassis_serial_node:main',
            'path_feeder_node = rtk_perception.path_feeder_node:main',
            'teb_debug_node = rtk_perception.teb_debug_node:main',
            'camera_detect_node = rtk_perception.camera_detect_node:main',
            'camera_http_streamer = rtk_perception.camera_http_streamer:main',
            'scan_min_range_filter = rtk_perception.scan_min_range_filter:main',
            'detection_to_cloud = rtk_perception.detection_to_cloud:main',
            'costmap_periodic_clear = rtk_perception.costmap_periodic_clear:main',
        ],
    },
)
