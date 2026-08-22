from setuptools import setup

package_name = 'rtk_map'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/mbtiles_server.launch.py']),
    ],
    install_requires=['setuptools', 'Flask'],
    zip_safe=True,
    maintainer='BrainControl Developer',
    maintainer_email='dev@braincontrol.local',
    description='离线 mbtiles 瓦片图服务',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mbtiles_server = rtk_map.mbtiles_server:main',
        ],
    },
)
