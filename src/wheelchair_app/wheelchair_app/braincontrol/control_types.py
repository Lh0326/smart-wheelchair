"""脑控轮椅控制层共享枚举类型。

被 imu_handler / frown_detector / control_state_machine /
motion_commander / main_eeg 共同引用，是控制层的协议契约。
"""
from enum import Enum


class TiltDirection(Enum):
    """头部姿态方向（基于 pitch/roll 相对基线）。

    轴约定：pitch > 0 = 低头（FORWARD），roll > 0 = 左歪（LEFT）。
    """
    NONE = 0
    FORWARD = 1     # 低头（pitch > 0）
    BACKWARD = 2    # 仰头（pitch < 0）
    LEFT = 3        # 左歪（roll > 0）
    RIGHT = 4       # 右歪（roll < 0）


class MotionCommand(Enum):
    """运动指令输出。"""
    STOP = 0
    FORWARD = 1
    BACKWARD = 2
    LEFT = 3
    RIGHT = 4


class ControlState(Enum):
    """控制层状态机。"""
    DISABLED = 0    # 瞌睡或 EEG 异常，所有输出 STOP
    LOCKED = 1      # 清醒但用户主动锁定，头部动作不输出
    ACTIVE = 2      # 清醒且未锁，输出运动指令
