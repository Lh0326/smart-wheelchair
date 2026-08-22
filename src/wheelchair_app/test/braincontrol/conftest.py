"""braincontrol 测试套件共享的 pytest 配置。

rtk 工程下 wheelchair_app 已作为可安装 ROS2 Python 包，测试通过
``source source_env.sh`` 设置 PYTHONPATH 后直接以
``from wheelchair_app.braincontrol.X import ...`` 形式 import，
无需 muscles-braincontrol 时代的 sys.path 注入。
"""
