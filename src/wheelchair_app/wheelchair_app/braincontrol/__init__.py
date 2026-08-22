"""脑控运行时模块（搬迁自 muscles-braincontrol 项目）。

包含：EEG 采集 + 专注度 pipeline + IMU 头姿 + 咬牙/皱眉检测 +
控制状态机 + 运动指令发布。GUI 由 wheelchair_app.tabs.braincontrol_tab 渲染。

训练/校准工具保留在原项目：
    原 muscles-braincontrol 项目(已并入本仓库)
"""

from .control_types import MotionCommand  # noqa: F401
