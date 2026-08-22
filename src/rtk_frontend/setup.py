import os
from glob import glob

from setuptools import setup

package_name = 'rtk_frontend'

# 只复制 frontend 下的普通文件（排除 vendor 等子目录，子目录由专门的 data_files 条目处理）
frontend_files = [f for f in glob('frontend/*') if os.path.isfile(f)]

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/frontend', frontend_files),
        ('share/' + package_name + '/frontend/vendor', glob('frontend/vendor/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='BrainControl Developer',
    maintainer_email='dev@braincontrol.local',
    description='Leaflet 前端 + ROS2 桥节点',
    license='MIT',
    entry_points={
        'console_scripts': [
            'bridge_node = rtk_frontend.bridge_node:main',
            'frontend_server = rtk_frontend.static_server:main',
        ],
    },
)
