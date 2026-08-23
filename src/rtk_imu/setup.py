import os
from glob import glob
from setuptools import setup

package_name = 'rtk_imu'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/udev', glob('udev/*.rules')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='BrainControl Developer',
    maintainer_email='dev@braincontrol.local',
    description='HWT906P IMU 驱动（维特标准协议）',
    license='MIT',
    entry_points={
        'console_scripts': [
            'jy901_driver = rtk_imu.jy901_driver_node:main',
            'imu_to_pose = rtk_imu.imu_to_pose_node:main',
        ],
    },
)
