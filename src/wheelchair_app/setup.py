from setuptools import setup
from glob import glob
import os

PACKAGE_NAME = 'wheelchair_app'

setup(
    name=PACKAGE_NAME,
    version='0.1.0',
    packages=[
        PACKAGE_NAME,
        f'{PACKAGE_NAME}.tabs',
        f'{PACKAGE_NAME}.nodes',
        f'{PACKAGE_NAME}.braincontrol',  # 脑控模块
    ],
    package_data={
        '': ['*.json'],
        f'{PACKAGE_NAME}.braincontrol': [
            'models/*.joblib',  # SVM 模型
            '*.json',           # imu_config.json
        ],
    },
    include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml']),
        # Web 资源(nav 含 vendor/ 子目录,需递归 glob)
        ('share/' + PACKAGE_NAME + '/web/shared',
            glob('web/shared/*')),
        ('share/' + PACKAGE_NAME + '/web/nav',
            [f for f in glob('web/nav/*') if not f.endswith('/vendor')]),
        ('share/' + PACKAGE_NAME + '/web/nav/vendor',
            glob('web/nav/vendor/*')),
        ('share/' + PACKAGE_NAME + '/web/companion',
            glob('web/companion/*')),
        # Qt 资源
        ('share/' + PACKAGE_NAME + '/resources',
            glob('resources/*')),
        # Launch
        ('share/' + PACKAGE_NAME + '/launch',
            glob('launch/*.py')),
        # Braincontrol 模型 + 配置
        ('share/' + PACKAGE_NAME + '/braincontrol/models',
            glob('wheelchair_app/braincontrol/models/*.joblib')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'main = wheelchair_app.main:main',
            'hw_monitor_node = wheelchair_app.nodes.hw_monitor_node:main',
            'voice_node = wheelchair_app.nodes.voice_node:main',
            'voice_announce_node = wheelchair_app.nodes.voice_announce_node:main',
        ],
    },
)
