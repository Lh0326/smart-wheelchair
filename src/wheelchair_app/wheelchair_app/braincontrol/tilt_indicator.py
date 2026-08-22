"""圆形姿态指示器（雷达图）—— EEG/IMU GUI 共享组件。

从 imu_live_view.py 提取，避免代码重复。两个 GUI（main_eeg.py 和 scripts/imu_live_view.py）
共用同一个 TiltIndicator 实现。

特性：
- 圆形极坐标裁剪（圆点不超出雷达图圆内）
- 小角度非线性放大（轻微头动也有明显位置反馈）
- 18° 连续饱和到外圈（无突然跳边）
- pitch/roll 数值显示
- 4 个方向中文标签（前/后/左/右）
"""
import math
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPainter, QPen, QColor, QFont
from PyQt5.QtWidgets import QWidget, QSizePolicy

from .imu_handler import ENTER_DEG, EXIT_DEG


# 沉墨 Deep Ink 主题色（与 braincontrol_tab.py 的 _INK_* 保持一致）
_INK_FACE = QColor('#1A1F2A')   # 背景
_INK_GRID = QColor('#3A3A3A')   # 同心圆刻度
_INK_DIM = QColor('#8A7D68')    # 十字轴 / 中心点边
_INK_CENTER_FILL = QColor('#3A3A3A')  # 中心点填充
_INK_TEXT = QColor('#E8E4DC')   # 文字 / 方向标签

# 显示映射只增强雷达图反馈，不改变 ImuHandler 的实际控制阈值。
_DISPLAY_MAX_DEG = 30.0
_DISPLAY_FULL_SCALE_DEG = 18.0
_DISPLAY_RESPONSE_EXPONENT = 0.65


def map_tilt_for_display(pitch_deg: float, roll_deg: float):
    """将真实倾角映射为更敏感的雷达图坐标。

    保持方向不变，用幂函数放大小角度：约 3°/6°/12° 分别显示在
    31%/49%/77% 半径处，18° 连续到达外圈。该函数不参与运动判定。
    """
    magnitude = math.hypot(pitch_deg, roll_deg)
    if magnitude <= 1e-6:
        return 0.0, 0.0

    normalized = min(magnitude / _DISPLAY_FULL_SCALE_DEG, 1.0)
    display_magnitude = (
        _DISPLAY_MAX_DEG * normalized ** _DISPLAY_RESPONSE_EXPONENT
    )
    scale = display_magnitude / magnitude
    return pitch_deg * scale, roll_deg * scale


def _display_radius_for_angle(angle_deg: float, radius: int) -> int:
    pitch_display, _ = map_tilt_for_display(angle_deg, 0.0)
    return int(radius * pitch_display / _DISPLAY_MAX_DEG)


class TiltIndicator(QWidget):
    """圆形姿态指示器。

    通过 set_tilt(pitch, roll) 更新当前姿态，set_baseline(pitch, roll) 设置基线。
    paintEvent 自动绘制圆点位置 + 颜色 + 文字。

    主题：沉墨 Deep Ink 深色，与 braincontrol_tab 一致。
    尺寸：sizePolicy Expanding，撑满父布局可用空间；paintEvent 取 min(w,h) 保持圆形。
    """

    def __init__(self):
        super().__init__()
        # minimumSize 400×320：保证"前"字标签（cx-8, cy-radius-4）始终在
        # 左上角 pitch/roll 文字框右侧（文字框右边界 ≤177px，"前"字 x ≥184px
        # 当 width≥400 时）。窄于 400 会发生重叠（缺陷 #4 修复闭环）。
        self.setMinimumSize(400, 320)
        # Expanding 让雷达图撑满右栏宽度（不再留空白边）
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self._pitch_deg = 0.0
        self._roll_deg = 0.0
        self._pitch_baseline = 0.0
        self._roll_baseline = 0.0
        self._locked = False  # 锁定时圆点强制在中心

    def set_tilt(self, pitch_deg, roll_deg):
        self._pitch_deg = pitch_deg
        self._roll_deg = roll_deg
        self.update()

    def set_baseline(self, pitch_deg, roll_deg):
        self._pitch_baseline = pitch_deg
        self._roll_baseline = roll_deg

    def set_locked(self, locked: bool):
        """锁定状态下圆点强制显示在雷达图中心。"""
        self._locked = locked
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w = self.width()
        h = self.height()
        cx, cy = w // 2, h // 2
        radius = min(w, h) // 2 - 30

        # 沉墨主题背景
        p.fillRect(self.rect(), _INK_FACE)

        # 同心圆按真实头部倾角绘制；6° 回中圈和 12° 触发圈与控制逻辑同步。
        for deg in (3.0, EXIT_DEG, ENTER_DEG, _DISPLAY_FULL_SCALE_DEG):
            if deg == EXIT_DEG:
                p.setPen(QPen(QColor(80, 150, 95), 1))
            elif deg == ENTER_DEG:
                p.setPen(QPen(QColor(210, 165, 70), 2))
            else:
                p.setPen(QPen(_INK_GRID, 1))
            r = _display_radius_for_angle(deg, radius)
            p.drawEllipse(cx - r, cy - r, 2 * r, 2 * r)

        # 十字坐标轴
        p.setPen(QPen(_INK_DIM, 1, Qt.DashLine))
        p.drawLine(cx - radius, cy, cx + radius, cy)
        p.drawLine(cx, cy - radius, cx, cy + radius)

        # 中心点
        p.setPen(QPen(_INK_DIM, 1))
        p.setBrush(_INK_CENTER_FILL)
        p.drawEllipse(cx - 4, cy - 4, 8, 8)

        # 当前姿态点（相对基线）
        pitch_rel = self._pitch_deg - self._pitch_baseline
        roll_rel = self._roll_deg - self._roll_baseline

        # 锁定状态：圆点强制在中心 + 灰色
        if self._locked:
            px = cx
            py = cy
            color = QColor(150, 150, 150)  # 灰色 = 锁定
            pitch_display = 0
            roll_display = 0
        else:
            magnitude = math.sqrt(pitch_rel*pitch_rel + roll_rel*roll_rel)
            pitch_display, roll_display = map_tilt_for_display(
                pitch_rel, roll_rel
            )

            # pitch > 0 = FORWARD → 屏幕上方
            # roll > 0 = LEFT → 屏幕左侧
            px = cx - int(radius * roll_display / _DISPLAY_MAX_DEG)
            py = cy - int(radius * pitch_display / _DISPLAY_MAX_DEG)

            # 染色按真实角度判断，避免显示放大后过早变成方向色。
            if magnitude < EXIT_DEG:
                color = QColor(100, 180, 100)  # 绿色 = STOP
            elif abs(pitch_rel) >= abs(roll_rel):
                color = QColor(70, 130, 220) if pitch_rel > 0 else QColor(220, 130, 70)
            else:
                color = QColor(220, 200, 70)

        # 外圈光晕让当前位置在深色背景中更容易捕捉。
        halo = QColor(color)
        halo.setAlpha(65)
        p.setPen(Qt.NoPen)
        p.setBrush(halo)
        p.drawEllipse(px - 14, py - 14, 28, 28)
        p.setPen(QPen(color, 2))
        p.setBrush(color)
        p.drawEllipse(px - 8, py - 8, 16, 16)

        # 文字：左上角，避开"前"字（前字在 cy - radius - 4 居中位置）
        # 加半透明背景框避免压在网格线上看不清
        p.setPen(QPen(_INK_TEXT))
        font = QFont('Sans', 9)
        p.setFont(font)
        if self._locked:
            status_text = "🔒 锁定"
        elif magnitude >= ENTER_DEG:
            status_text = "触发区"
        elif magnitude < EXIT_DEG:
            status_text = "回中区"
        else:
            status_text = "跟随区"
        text_line = f"pitch={pitch_rel:+.1f}° roll={roll_rel:+.1f}° {status_text}".strip()
        text_rect = self.rect().adjusted(8, 8, 0, 0)
        # 半透明背景（与 _INK_FACE 同色但 alpha=200）
        metrics = p.fontMetrics()
        text_w = metrics.horizontalAdvance(text_line) + 8
        text_h = metrics.height() + 4
        p.fillRect(4, 4, text_w, text_h, QColor(26, 31, 42, 200))
        p.drawText(text_rect, Qt.AlignTop | Qt.AlignLeft, text_line)
        p.drawText(cx + radius + 4, cy + 4, "右")
        p.drawText(cx - radius - 16, cy + 4, "左")
        p.drawText(cx - 8, cy - radius - 4, "前")
        p.drawText(cx - 8, cy + radius + 14, "后")
