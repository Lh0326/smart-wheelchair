import os
from glob import glob

from setuptools import setup

package_name = 'rtk_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='BrainControl Developer',
    maintainer_email='dev@braincontrol.local',
    description='OSRM 路径规划 ROS2 节点',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'osrm_planner_node = rtk_planner.osrm_planner_node:main',
            'networkx_planner_node = rtk_planner.networkx_planner_node:main',
        ],
    },
)
