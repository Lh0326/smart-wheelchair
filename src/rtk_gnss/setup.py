import os
from glob import glob
from setuptools import setup

package_name = 'rtk_gnss'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='BrainControl Developer',
    maintainer_email='dev@braincontrol.local',
    description='GNSS 定位节点',
    license='MIT',
    entry_points={
        'console_scripts': [
            'fake_gnss = rtk_gnss.fake_gnss_node:main',
            'ec20_gnss = rtk_gnss.ec20_gnss_node:main',
            'dxgp10_gnss = rtk_gnss.dxgp10_gnss_node:main',
            'imu_heading = rtk_gnss.imu_heading_node:main',
            'gps_fusion = rtk_gnss.gps_fusion_node:main',
        ],
    },
)
