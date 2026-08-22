import os
from glob import glob
from setuptools import setup

setup(
    name='rtk_bringup',
    version='0.1.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/rtk_bringup']),
        ('share/rtk_bringup', ['package.xml']),
        ('share/rtk_bringup/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='BrainControl Developer',
    maintainer_email='dev@braincontrol.local',
    description='顶层启动包',
    license='MIT',
)
