"""BrainControlTab：脑电控制 + IMU 头部姿态 → /cmd_vel_eeg 运动指令。

完整搬迁自 muscles-braincontrol 运行时控制部分。三栏布局：
    左栏：连接外设 + 串口设置 + 专注度/频带/质量状态
    中栏：matplotlib 分数+频带 canvas + 8 通道波形 canvas
    右栏：IMU 头姿 + 咬牙事件 + 运动指令输出

数据流：
    EEG (ADS1299Reader QThread) ──┐
    IMU (ESP32ImuReader QThread) ─┤
                                  ├─→ ControlStateMachine ─→ MotionCommander ─→ /cmd_vel_eeg
    咬牙 (ClenchDetector) ──────┘
    专注度 (FocusDetector) ──────┘

ROS topic：
    发布 /focus_state (5Hz throttle, std_msgs/String "<state>:<p_focus>")
    发布 /cmd_vel_eeg (20Hz, geometry_msgs/Twist)
    发布 /eeg_mode_active (事件触发, std_msgs/Bool)
"""
import logging
import math
import os
import threading
import time

import numpy as np
import serial
import matplotlib
import matplotlib.font_manager as fm
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure

# 中文字体配置（参照 main_eeg.py:465-470，避免标题/标签显示方块）
_available_fonts = {f.name for f in fm.fontManager.ttflist}
for _candidate in ['Noto Sans CJK SC', 'WenQuanYi Zen Hei',
                   'WenQuanYi Micro Hei', 'Source Han Sans CN',
                   'Source Han Serif CN', 'Noto Sans CJK JP',
                   'SimHei', 'Droid Sans Fallback']:
    if _candidate in _available_fonts:
        matplotlib.rcParams['font.sans-serif'] = [_candidate]
        break
matplotlib.rcParams['axes.unicode_minus'] = False

# 沉墨 Deep Ink 深色主题（贴合 wheelchair_app sink.qss）
# 背景 #0F1219 / 主文字 #E8E4DC / 次文字 #B8A98E / 强调 #D4A574
_INK_BG = '#0F1219'        # figure + ax 背景
_INK_FACE = '#1A1F2A'      # ax 内部稍浅（区分面板）
_INK_TEXT = '#E8E4DC'      # 主文字
_INK_DIM = '#8A7D68'       # 次文字/spine/tick
_INK_ACCENT = '#D4A574'    # 强调色（金棕）
_INK_GRID = '#3A3A3A'      # 网格
_INK_FOCUS_GREEN = '#6FD68C'  # 专注线（柔和绿）
_INK_FOCUS_RED = '#E06C75'    # 分心线（柔和红）
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QGroupBox, QHBoxLayout, QLabel, QMessageBox, QProgressBar,
    QPushButton, QVBoxLayout, QWidget, QApplication
)

from ..braincontrol.ads1299 import ADS1298Data, ADS1298cmd
from ..braincontrol.ads1299_reader import ADS1299Reader
from ..braincontrol.clench_detector import ClenchDetector
from ..braincontrol.clench_features import CLENCH_WINDOW_SEC
from ..braincontrol.control_state_machine import ControlStateMachine
from ..braincontrol.control_types import (
    ControlState, MotionCommand, TiltDirection,
)
from ..braincontrol.focus_detector import FocusDetector
from ..braincontrol.head_pose_calculator import HeadPoseCalculator
from ..braincontrol.imu_handler import ImuHandler
from ..braincontrol.imu_reader import ESP32ImuReader
from ..braincontrol.motion_commander import MotionCommander
from ..braincontrol.tilt_indicator import TiltIndicator

logger = logging.getLogger(__name__)

# 运动指令英文枚举名 → 中文（右栏 "指令:" label 显示用）。
_CMD_CN = {
    'FORWARD': '前进',
    'BACKWARD': '后退',
    'LEFT': '左转',
    'RIGHT': '右转',
    'STOP': '静止',
}

# ADS1299 EEG 板用 1000000 baud（与 muscles-braincontrol main_eeg.py 一致）
_EEG_BAUDRATE = 1000000
# 维特 ESP32+IMU 模块用 115200 baud
_IMU_BAUDRATE = 115200
# 状态机 tick 间隔（毫秒）：独立控制环使用，不能依赖 EEG 绘图刷新。
_STATE_MACHINE_DT_MS = 50
# EEG 识别采样率：ADS1299Reader 与训练数据均按 500Hz 使用。
_EEG_SAMPLE_RATE_HZ = 500
# muscles-braincontrol/main_eeg.py 使用 _WINDOW_SIZE=1000（2s @500Hz）。
# 专注/瞌睡 SVM 和咬牙 SVM 都按这个窗口分布训练，不能喂 UI 的 3s 波形缓冲。
_FOCUS_WINDOW_SAMPLES = int(2.0 * _EEG_SAMPLE_RATE_HZ)
_CLENCH_WINDOW_SAMPLES = int(CLENCH_WINDOW_SEC * _EEG_SAMPLE_RATE_HZ)
# IMU 数据超时后强制清零头姿，避免 reader 断流后保持最后一次右转/左转。
_IMU_STALE_TIMEOUT_SEC = 0.75
# 咬牙 SVM 模型路径（相对包根 braincontrol/models/）
_CLENCH_MODEL_PATH = os.path.join(
    os.path.dirname(__file__), '..', 'braincontrol',
    'models', 'clench_svm.joblib',
)
# 专注度 SVM 模型路径（与 _CLENCH_MODEL_PATH 同款写法）。
# 历史 bug：BrainControlTab 实例化 FocusDetector() 时漏传 model_path，
# 导致 classifier=None、p_focus 恒为 0.5、focus_state 永远 'neutral'，
# 状态机死锁在 DISABLED，运动指令永远 STOP。
_FOCUS_MODEL_PATH = os.path.join(
    os.path.dirname(__file__), '..', 'braincontrol',
    'models', 'focus_svm.joblib',
)


class BrainControlTab(QWidget):
    """脑电控制 Tab（PyQt5 三栏布局）。"""

    def __init__(self, ros_node, parent=None):
        """初始化 tab。

        Args:
            ros_node: rclpy Node 实例（共享自 RosBridgeNode）。
            parent: Qt 父控件。
        """
        super().__init__(parent)
        self._ros_node = ros_node

        # 状态
        self._eeg_connected = False
        self._imu_connected = False
        self._monitoring = False
        self._eeg_mode_active = False
        self._eeg_reader = None
        self._imu_reader = None
        self._eeg_serial = None
        self._imu_serial = None
        self._eeg_ads_data = None
        self._eeg_data_lock = None
        self._last_focus_publish = 0.0

        # 算法实例 + 数据缓冲（任务 5：数据路由到 FocusDetector/HeadPoseCalculator）
        # 加载训练好的 SVM 模型；失败时回退到 classifier=None（向后兼容）。
        try:
            self._focus_detector = FocusDetector(model_path=_FOCUS_MODEL_PATH)
            logger.info(f"FocusDetector 已加载 SVM 模型: {_FOCUS_MODEL_PATH}")
        except Exception as e:
            logger.warning(
                f"FocusDetector 模型加载失败，回退到 classifier=None: {e}"
            )
            self._focus_detector = FocusDetector()
        self._head_pose_calc = HeadPoseCalculator()
        # EEG buffer：预分配固定 (8, 1500) ndarray，零分配滚动写入。
        # 1500 = 500Hz × 3s，与 _init_wave_plot 的 xlim(0, 3.0) 对齐，
        # 保证 3 秒窗口能完整装下，波形右对齐显示满画面（示波器风格）。
        # 旧方案用 np.concatenate 每帧重建，高频回调下 GC 压力大。
        self._eeg_buffer = np.zeros((8, 1500), dtype=float)
        # 预分配后无法用 .size==0 判空，需独立计数器跟踪有效样本数。
        # 饱和后保持在 capacity（1500）；buffer 本身始终是 (8, 1500)，
        # 有效数据在末尾 self._eeg_buffer_samples 列，前面是零填充。
        self._eeg_buffer_samples = 0
        self._last_focus_compute = 0.0
        # ClenchDetector 独立节流：150ms（提高 P 值更新频率，原跟 FocusDetector
        # 共用 500ms 节流，P 值更新慢）。_clench_interval 单独控制咬牙检测频率。
        self._last_clench_compute = 0.0
        self._clench_interval = 0.15
        # 分数历史曲线缓存（参照 main_eeg.py:_update_score_plot）
        self._time_history = []
        self._score_history = []

        # 任务 6：ControlStateMachine 接线
        # 实测架构（与 control_state_machine.py 一致）：状态机是纯 Python 类，
        # 同步 update(focus_state, toggle_event, tilt, dt_ms) -> MotionCommand；
        # 没有 pyqtSignal，由 tab 每 tick 主动调用。
        self._state_machine = ControlStateMachine()
        self._motion_commander = MotionCommander(ros_node=ros_node)
        # 最新 MotionCommand 缓存：用于 UI 横幅/右栏标签显示。
        # 指令本身不去重，20Hz 持续发布以刷新底盘侧 cmd watchdog。
        self._last_cmd = None

        # ImuHandler：pitch/roll → TiltDirection（带校准/死区/锁定逻辑）
        # 未校准时 update() 直接返回 NONE（_decide 早退），安全 fallback。
        # 校准由 _on_imu_data 累积前 N 帧自动触发；用户点"设为正前方"按钮
        # 调 reset() 重新校准。ESP32+IMU 数据率 4-5Hz，20 帧 ≈ 4-5 秒。
        self._imu_handler = ImuHandler()
        self._imu_cal_frames_needed = 20
        self._imu_cal_frames_collected = 0
        # 四元数先做相对正前方校准，再转 pitch/roll。ESP32+IMU 静止四元数
        # 可能接近 180° roll；直接转全局欧拉角会在 ±180° 附近跳变，
        # 表现为右侧运动指令固定右转。
        self._imu_quat_ref = None
        self._imu_quat_cal_samples = []

        # 咬牙检测器（替代原皱眉检测，做 LOCKED ↔ ACTIVE toggle）
        # threshold=0.2（原默认 0.5）：用户要求降低阈值更敏感
        # infer_every_ms=50（原默认 500）：内部每次 update 都推断，
        # 由外部 _clench_interval=150ms 控制真实频率（≈ 6.7Hz P 值更新）
        try:
            self._clench_detector = ClenchDetector(
                model_path=_CLENCH_MODEL_PATH,
                threshold=0.2,
                infer_every_ms=50,
            )
        except Exception as e:
            logger.warning(f"ClenchDetector init failed: {e}")
            self._clench_detector = None

        # 状态机输入缓存：focus_state 由 _on_focus_result 写入；
        # toggle_pending 由 _on_clench_event（ClenchDetector rising edge）触发置位；
        # tilt 由 _on_imu_data（pitch/roll→ImuHandler）写入。
        # _tick_state_machine() 每 50ms 把缓存喂给状态机。
        self._focus_state = 'focused'
        self._current_tilt = TiltDirection.NONE
        self._toggle_pending = False
        self._last_state_machine_tick = 0.0

        # 50ms 节流 timer（避免 EEG QThread 高频回调阻塞主线程）
        self._canvas_refresh_timer = QTimer(self)
        self._canvas_refresh_timer.timeout.connect(self._refresh_canvases)

        # 独立控制环：IMU 头姿 → 状态机 → /cmd_vel_eeg。
        # 不能绑在 EEG canvas 刷新上，否则没有 EEG 数据或绘图/SVM 变慢时，
        # 底盘命令会延迟、断续，甚至触发底盘侧 1s cmd watchdog 归零。
        self._control_timer = QTimer(self)
        self._control_timer.timeout.connect(self._tick_control_loop)
        self._control_timer.start(_STATE_MACHINE_DT_MS)

        # EEG 模式心跳定时器：1Hz 发布 /eeg_mode_active
        # chassis_serial_node 3s 无心跳会 fallback 到 Nav2，必须持续发
        self._eeg_heartbeat_timer = QTimer(self)
        self._eeg_heartbeat_timer.timeout.connect(self._eeg_heartbeat)
        self._eeg_heartbeat_timer.start(1000)  # 1Hz

        # blit 背景缓存：None 表示尚未捕获（首帧或 resize 后需重新捕获）
        self._wave_bg = None

        # EEG/IMU 数据心跳时间戳（用于状态显示判定，避免串口已开但无数据时误导用户）
        self._last_eeg_data_ts = 0.0
        self._last_imu_data_ts = 0.0
        # 连接尝试状态：'idle' / 'scanning' / 'connected_waiting' / 'failed'
        self._eeg_connect_state = 'idle'
        self._imu_connect_state = 'idle'

        # ROS publishers
        from std_msgs.msg import Bool, Float64, String
        self._focus_state_pub = ros_node.create_publisher(String, '/focus_state', 10)
        self._cmd_vel_eeg_pub = None  # 由 MotionCommander 管理（/cmd_vel_eeg）
        self._eeg_mode_pub = ros_node.create_publisher(Bool, '/eeg_mode_active', 10)
        self._head_pose_pub = ros_node.create_publisher(Float64, '/eeg_head_pose', 10)

        self._build_ui()
        # 启动后立即自动扫描一次（用户切到 Tab 3 即可看到设备状态）
        # 实际连接由 [🔌 自动扫描并连接外设] 按钮触发
        self._eeg_status_label.setText("EEG: 待扫描")
        self._imu_status_label.setText("IMU: 待扫描")

    # ========== UI 构建 ==========

    def _build_ui(self):
        """构建顶部横幅 + 三栏布局。"""
        # 外层垂直：顶部运动状态横幅 + 三栏
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(4)

        # 顶部运动/锁定横幅（参照 muscles-braincontrol main_eeg.py:552-558 风格）
        self._motion_banner_label = QLabel("静止")
        banner_font = self._motion_banner_label.font()
        banner_font.setPointSize(20)
        banner_font.setBold(True)
        self._motion_banner_label.setFont(banner_font)
        self._motion_banner_label.setAlignment(Qt.AlignCenter)
        self._motion_banner_label.setStyleSheet(
            "background-color: #f5f5f5; color: #888;"
            "padding: 10px; border-radius: 6px; margin: 4px;"
            "border: 3px solid #888;"
        )
        outer.addWidget(self._motion_banner_label)

        # 三栏（原布局）
        columns = QHBoxLayout()
        columns.addWidget(self._build_left_column(), stretch=1)
        columns.addWidget(self._build_middle_column(), stretch=2)
        columns.addWidget(self._build_right_column(), stretch=2)
        outer.addLayout(columns, stretch=1)

    def resizeEvent(self, event):
        """窗口大小变化时让 blit 背景缓存失效，下一帧重新捕获。"""
        self._wave_bg = None
        super().resizeEvent(event)

    def _build_left_column(self) -> QWidget:
        col = QWidget()
        v = QVBoxLayout(col)

        # 操作按钮（自动扫描，无需手动选端口）
        self._connect_button = QPushButton("🔌 自动扫描并连接外设")
        self._connect_button.setCheckable(True)
        self._connect_button.clicked.connect(self._on_connect_clicked)
        v.addWidget(self._connect_button)

        self._eeg_mode_toggle = QPushButton("🧠 脑控模式")
        self._eeg_mode_toggle.setCheckable(True)
        self._eeg_mode_toggle.clicked.connect(self._on_eeg_mode_toggled)
        v.addWidget(self._eeg_mode_toggle)

        # 串口状态
        status_gb = QGroupBox("串口状态")
        sv = QVBoxLayout(status_gb)
        self._eeg_status_label = QLabel("EEG: ✗ 未连接")
        self._imu_status_label = QLabel("IMU: ✗ 未连接")
        sv.addWidget(self._eeg_status_label)
        sv.addWidget(self._imu_status_label)
        v.addWidget(status_gb)

        # 专注度 GroupBox
        focus_gb = QGroupBox("专注度")
        fg = QVBoxLayout(focus_gb)
        self._focus_score_label = QLabel("--")
        score_font = self._focus_score_label.font()
        score_font.setPointSize(24)
        score_font.setBold(True)
        self._focus_score_label.setFont(score_font)
        self._focus_score_label.setAlignment(Qt.AlignCenter)

        self._focus_state_label = QLabel("等待数据")
        state_font = self._focus_state_label.font()
        state_font.setPointSize(14)
        state_font.setBold(True)
        self._focus_state_label.setFont(state_font)
        self._focus_state_label.setAlignment(Qt.AlignCenter)
        self._focus_state_label.setStyleSheet(
            "background-color: #888; color: white; padding: 6px; border-radius: 4px;"
        )
        self._focus_progress = QProgressBar()
        self._focus_progress.setRange(0, 100)
        fg.addWidget(self._focus_score_label)
        fg.addWidget(self._focus_state_label)
        fg.addWidget(self._focus_progress)
        v.addWidget(focus_gb)

        # 频带功率 GroupBox（占位，任务 5 接入实时数据）
        band_gb = QGroupBox("频带功率")
        bg = QVBoxLayout(band_gb)
        self._band_label = QLabel("θ: --  α: --  β: --")
        bg.addWidget(self._band_label)
        v.addWidget(band_gb)

        # 实时质量 GroupBox
        qual_gb = QGroupBox("实时质量")
        qg = QVBoxLayout(qual_gb)
        self._quality_label = QLabel("EMG: --  置信度: --")
        qg.addWidget(self._quality_label)
        v.addWidget(qual_gb)

        v.addStretch()
        return col

    def _build_middle_column(self) -> QWidget:
        """中栏：分数+频带 canvas + 8 通道波形 canvas。

        参照 muscles-braincontrol main_eeg.py 风格 + 沉墨 Deep Ink 深色主题：
        - canvas_main: subplots(2,1)，分数历史曲线 + 5 频段柱状图
        - canvas_wave: 8 通道独立颜色 + 偏移叠加
        """
        col = QWidget()
        v = QVBoxLayout(col)

        # 顶部 canvas：分数 + 频带（2 行 1 列子图，比例 2:1.5）
        self._score_fig = Figure(figsize=(8, 3.5), layout='constrained',
                                  facecolor=_INK_BG)
        gs = self._score_fig.add_gridspec(2, 1, height_ratios=[2, 1.5])
        self._score_ax = self._score_fig.add_subplot(gs[0])
        self._bands_ax = self._score_fig.add_subplot(gs[1])
        self._style_ax(self._score_ax)
        self._style_ax(self._bands_ax)
        self._init_score_plot()
        self._init_bands_plot()
        self._score_canvas = FigureCanvasQTAgg(self._score_fig)
        self._score_canvas.setMinimumHeight(220)
        v.addWidget(self._score_canvas, stretch=2)

        # 底部 canvas：8 通道波形（独立 figure 避免连带重绘）
        self._wave_fig = Figure(figsize=(8, 4), layout='constrained',
                                 facecolor=_INK_BG)
        self._wave_ax = self._wave_fig.add_subplot(111)
        self._style_ax(self._wave_ax)
        self._init_wave_plot()
        self._waveform_canvas = FigureCanvasQTAgg(self._wave_fig)
        self._waveform_canvas.setMinimumHeight(280)
        v.addWidget(self._waveform_canvas, stretch=3)

        return col

    def _style_ax(self, ax):
        """应用沉墨 Deep Ink 深色主题到 ax。"""
        ax.set_facecolor(_INK_FACE)
        ax.tick_params(colors=_INK_DIM, labelsize=8)
        for spine in ax.spines.values():
            spine.set_color(_INK_DIM)
            spine.set_linewidth(0.5)
        ax.xaxis.label.set_color(_INK_TEXT)
        ax.yaxis.label.set_color(_INK_TEXT)
        ax.title.set_color(_INK_TEXT)

    def _init_score_plot(self):
        """初始化专注度分数历史曲线（参照 main_eeg.py:577-589）。"""
        # 历史曲线（金棕实线，贴主题）
        self._score_line, = self._score_ax.plot([], [], color=_INK_ACCENT,
                                                  linewidth=2)
        self._score_ax.set_ylim(0, 100)
        self._score_ax.set_ylabel('专注度（高=清醒）')
        self._score_ax.set_title('实时专注度')
        # 阈值线：70 柔绿虚线（专注线）+ 40 柔红虚线（分心线）
        self._score_ax.axhline(y=70, color=_INK_FOCUS_GREEN,
                               linestyle='--', alpha=0.6, label='专注线')
        self._score_ax.axhline(y=40, color=_INK_FOCUS_RED,
                               linestyle='--', alpha=0.6, label='分心线')
        leg = self._score_ax.legend(loc='upper right', fontsize=8,
                                     facecolor=_INK_FACE,
                                     edgecolor=_INK_DIM,
                                     labelcolor=_INK_TEXT)
        self._score_ax.grid(True, color=_INK_GRID, alpha=0.5)

    def _init_bands_plot(self):
        """初始化 5 频段功率柱状图（参照 main_eeg.py:591-596）。

        颜色：δ紫 θ蓝 α绿 β橙 γ红（与 BANDS 顺序对应，深色调）。
        """
        self._band_bars = self._bands_ax.bar(
            ['δ', 'θ', 'α', 'β', 'γ'], [0, 0, 0, 0, 0],
            color=['#9C5FB5', '#5C9CD6', '#6FD68C', '#E0A050', '#E06C75'],
            edgecolor=_INK_DIM, linewidth=0.5
        )
        self._bands_ax.set_ylabel('功率 (μV²)')
        self._bands_ax.set_title('频带功率 (前额均值)')
        self._bands_ax.grid(True, color=_INK_GRID, alpha=0.5, axis='y')

    def _init_wave_plot(self):
        """初始化 8 通道波形（参照 main_eeg.py:598-606）。

        8 个独立颜色 + 每通道偏移 200μV 叠加显示，避免互相遮挡。
        深色主题适配的柔和色板。
        """
        # 深色背景适配的 8 色（避免纯饱和原色刺眼）
        colors = ['#5C9CD6', '#6FD68C', '#E06C75', '#56B4C9',
                  '#9C5FB5', '#E0C050', '#B8A98E', '#E07A50']
        self._wave_lines = []
        for i in range(8):
            line, = self._wave_ax.plot([], [], color=colors[i],
                                        linewidth=0.7, label=f'CH{i+1}')
            self._wave_lines.append(line)
        self._wave_ax.grid(True, color=_INK_GRID, alpha=0.5)
        leg = self._wave_ax.legend(loc='upper right', fontsize=7, ncol=4,
                                    facecolor=_INK_FACE,
                                    edgecolor=_INK_DIM,
                                    labelcolor=_INK_TEXT)
        # rotation=0 让 μV 横排不竖排；labelpad=20 让标签离轴线远一点避免遮挡刻度
        self._wave_ax.set_ylabel('μV', rotation=0, labelpad=20)
        self._wave_ax.set_title('8 通道 EEG 波形 (预处理后)')
        # blit：线条标记为 animated，让 draw() 跳过、只由 draw_artist 增量绘制
        for line in self._wave_lines:
            line.set_animated(True)
        # 固定 xlim/ylim：
        # - xlim 0~3 秒（500Hz × 3s = 1500 样本窗口），用时间秒数让横轴可读
        # - ylim -200~1800μV：8 通道 offset=ch*200（CH1=0、CH8=1400），
        #   留 ±200μV padding 让 CH1 和 CH8 不贴边，画面均衡填满
        # 旧值 xlim(0,1500) 是样本序号无单位；ylim(-2000,2000) 让 8 条线挤在中间 1600μV
        self._wave_ax.set_xlim(0.0, 3.0)
        self._wave_ax.set_ylim(-200.0, 1800.0)
        self._wave_ax.set_xlabel('时间 (s)')

    def _build_right_column(self) -> QWidget:
        col = QWidget()
        v = QVBoxLayout(col)

        # IMU 头姿 GroupBox
        imu_gb = QGroupBox("🧭 IMU 头部姿态")
        ig = QVBoxLayout(imu_gb)
        self._imu_state_label = QLabel("状态: ✗ 未连接")
        ig.addWidget(self._imu_state_label)
        self._quaternion_label = QLabel("四元数: w=-- x=-- y=-- z=--")
        ig.addWidget(self._quaternion_label)
        self._euler_label = QLabel("pitch=-- roll=-- yaw=--")
        ig.addWidget(self._euler_label)
        # TiltIndicator 罗盘（真实圆形姿态控件）
        self._tilt_indicator = TiltIndicator()
        self._tilt_indicator.setMinimumSize(300, 300)
        ig.addWidget(self._tilt_indicator, stretch=1)  # stretch=1 让雷达图占满剩余空间
        self._reset_forward_button = QPushButton("设为正前方")
        # 连接 IMU 校准重置：reset() 已在 ImuHandler 中实现（imu_handler.py:59）。
        # 点击后清零校准基准，下一帧 IMU 数据进入会重新累积校准。
        self._reset_forward_button.clicked.connect(self._on_reset_forward)
        ig.addWidget(self._reset_forward_button)
        v.addWidget(imu_gb)

        # 咬牙事件 GroupBox
        event_gb = QGroupBox("事件检测")
        ev = QVBoxLayout(event_gb)
        self._clench_label = QLabel("咬牙: --")
        ev.addWidget(self._clench_label)
        v.addWidget(event_gb)

        # 运动指令输出 GroupBox
        cmd_gb = QGroupBox("运动指令输出")
        cg = QVBoxLayout(cmd_gb)
        self._motion_command_label = QLabel("指令: STOP")
        self._motion_command_label.setStyleSheet("font-size: 18pt; font-weight: bold;")
        self._state_machine_label = QLabel("状态机: IDLE")
        self._publish_rate_label = QLabel("发布频率: -- Hz")
        cg.addWidget(self._motion_command_label)
        cg.addWidget(self._state_machine_label)
        cg.addWidget(self._publish_rate_label)
        v.addWidget(cmd_gb)

        v.addStretch()
        return col

    # ========== 串口刷新 ==========

    def _refresh_ports(self):
        """占位（已迁移到自动识别 _auto_detect_devices，保留方法名兼容）。"""
        pass

    # ========== 端口自动识别（参照 main_eeg.py:_auto_connect_imu）==========

    # 已知非脑控设备的 serial_number（CH9102 雷达）
    _KNOWN_LIDAR_SERIALS = {'5A6C086938', '5B8E671052'}

    # 设备 VID 标识（拔插法实测）
    _ESP32_NATIVE_VID = 0x303a   # ESP32 native USB（EEG WIFI 模块用，含 ADS1299）
    _CH340_VID = 0x1a86          # CH340（脑控 IMU + 底盘 + HWT906P 共用，按 udev symlink 排除非脑控）
    _EC25_MODEM_VID = 0x2c7c     # EC25 4G modem
    _PREFERRED_EEG_PORT = '/dev/ttyEEG'
    _PREFERRED_BC_IMU_PORT = '/dev/ttyBCIMU'
    _EXCLUDED_SERIAL_LINKS = (
        '/dev/wheelchair_chassis',
        '/dev/ttyIMU',
        '/dev/LD14P',
        '/dev/lidar_n10p',
        '/dev/ttyUSB_DIAG',
        '/dev/ttyUSB_NMEA',
        '/dev/ttyUSB_GNSS',
        '/dev/ttyUSB_AT',
        '/dev/ttyUSB_MODEM',
    )

    def _is_chassis_control_port(self, device: str) -> bool:
        """True 表示该串口不应作为脑控 EEG/IMU 外设扫描。"""
        if not device:
            return False

        excluded = set(self._EXCLUDED_SERIAL_LINKS)
        excluded.add(os.environ.get('CHASSIS_SERIAL_PORT', ''))
        device_real = os.path.realpath(device)
        for port in excluded:
            if not port:
                continue
            if device == port or device_real == os.path.realpath(port):
                return True
        return False

    def _try_preferred_eeg_port(self) -> str | None:
        """优先使用 udev 稳定软链接 /dev/ttyEEG。"""
        port = self._PREFERRED_EEG_PORT
        if os.path.exists(port) and not self._is_chassis_control_port(port):
            if self._test_eeg_port(port):
                return port
            logger.warning(f"首选 EEG 端口存在但识别失败: {port}")
        return None

    def _try_preferred_imu_port(self) -> str | None:
        """优先使用 udev 稳定软链接 /dev/ttyBCIMU。"""
        port = self._PREFERRED_BC_IMU_PORT
        if os.path.exists(port) and not self._is_chassis_control_port(port):
            if self._test_imu_port(port):
                return port
            logger.warning(f"首选脑控 IMU 端口存在但识别失败: {port}")
        return None

    def _auto_detect_devices(self):
        """自动扫描 ESP32+IMU 和 EEG WIFI 模块（ADS1299）。

        实测硬件分布（2026-07-12 二次复核，CSV + udevadm 双确认）：
        - EEG WIFI 模块 = ESP32-C3 native USB（VID=0x303a PID=0x1001），
          1M baud，需发 start_collect_cmd 后才有 0xA5 EEG 帧
        - 脑控 IMU = CH340（VID=0x1a86 PID=0x7523），115200 baud，
          主动输出 4 字段 CSV 四元数（4-5Hz，无需触发）
        - HWT906P 自主导航 IMU = /dev/ttyIMU（CH340，921600）
        - 电控底盘 = /dev/wheelchair_chassis（CH340，115200）

        历史 bug：之前两次注释都把脑控 IMU 写错（先说是 CH340，又说是
        ESP32-S3），实际 2026-07-12 实测 ttyUSB6@3-3.1.3 是 CH340，
        115200 主动流 CSV 四元数。EEG 因代码兜底扫 ESP32 VID 还能连，
        IMU 之前只在 ESP32 候选里兜底，CH340 永远扫不到。

        识别策略：
        - 优先用 udev 软链接 /dev/ttyEEG、/dev/ttyBCIMU（依赖正确 udev 规则）
        - EEG 兜底：扫所有 ESP32 VID=0x303a 设备，发 start_collect_cmd 看响应
        - IMU 兜底：扫所有 CH340 VID=0x1a86 PID=0x7523 设备（排除已被底盘/
          HWT906P/雷达 udev 软链接占用的），115200 读 CSV 四元数
        - 排除：底盘、HWT906P、雷达、EC25 modem 等非脑控串口

        Returns:
            (eeg_port, imu_port) 任一未找到为 None。
        """
        from serial.tools import list_ports
        eeg_port = self._try_preferred_eeg_port()
        imu_port = self._try_preferred_imu_port()

        if eeg_port and imu_port:
            return eeg_port, imu_port

        # 收集候选设备：ESP32（EEG）+ CH340（IMU 兜底）
        esp32_candidates = []
        ch340_candidates = []
        for p in list_ports.comports():
            if self._is_chassis_control_port(p.device):
                logger.info(f"跳过非脑控串口: {p.device}")
                continue
            if p.serial_number in self._KNOWN_LIDAR_SERIALS:
                continue
            if p.vid == self._EC25_MODEM_VID:
                continue
            if p.vid == self._ESP32_NATIVE_VID:
                esp32_candidates.append(p.device)
            elif p.vid == self._CH340_VID and p.pid == 0x7523:
                ch340_candidates.append(p.device)
        logger.info(f"ESP32 候选: {esp32_candidates}, CH340 候选: {ch340_candidates}")

        # EEG 兜底：从 ESP32 候选中找响应 start_collect_cmd 的
        if eeg_port is None:
            for dev in esp32_candidates:
                if self._test_eeg_port(dev):
                    eeg_port = dev
                    logger.info(f"EEG 兜底识别成功: {dev}")
                    break

        # IMU 兜底：先试 CH340 候选（脑控 IMU 是 CH340）；如果都没有，
        # 再试 ESP32 候选（保留对老固件 ESP32 IMU 的兼容）
        if imu_port is None:
            imu_search_order = ch340_candidates + [
                d for d in esp32_candidates if d != eeg_port
            ]
            for dev in imu_search_order:
                if dev == eeg_port:
                    continue
                if self._test_imu_port(dev):
                    imu_port = dev
                    logger.info(f"IMU 兜底识别成功: {dev}")
                    break
            if imu_port is None:
                logger.warning(
                    f"IMU 候选 {imu_search_order} 无 CSV 响应——"
                    f"检查 IMU 是否上电、波特率是否 115200、CSV 格式是否 4 字段浮点"
                )

        return eeg_port, imu_port

    def _test_imu_port(self, port: str) -> bool:
        """测试端口是否 ESP32-S3+IMU（115200 输出 4 字段浮点 CSV）。

        ESP32-S3 首次打开串口会因 DTR/RTS 边沿触发 reset，输出 4-5KB boot log。
        原实现 timeout=0.5 试读 3 次只够读 boot log 前几行，永远遇不到 CSV，
        导致 ttyACM3 ESP32-S3 IMU 误判为"非 IMU"。这里分两阶段：
        阶段 1（≤1.5s）：排空 boot log（in_waiting 空转后退出）
        阶段 2（≤2.5s）：等 CSV 四元数帧（4 字段 float），4-5Hz 应在 0.25s 内出现
        """
        try:
            with serial.Serial(port, 115200, timeout=0.3) as ser:
                # 阶段 1：排空 boot log
                ser.reset_input_buffer()
                t0 = time.time()
                while time.time() - t0 < 1.5:
                    n = ser.in_waiting
                    if n == 0:
                        break
                    ser.read(n)
                    time.sleep(0.05)
                # 阶段 2：等 CSV（最多 2.5s ≈ 10 帧 @4Hz）
                t0 = time.time()
                while time.time() - t0 < 2.5:
                    line = ser.readline().decode('ascii', errors='ignore').strip()
                    if not line:
                        continue
                    parts = line.split(',')
                    if len(parts) != 4:
                        continue
                    try:
                        for p in parts:
                            float(p)
                        return True
                    except ValueError:
                        continue
                return False
        except Exception:
            return False

    def _test_eeg_port(self, port: str) -> bool:
        """测试端口是否 ADS1299 EEG（1M bps + 发 start_collect_cmd 后含 0xA5）。

        ESP32-C3（WIFI 模块）固件不会主动流 EEG 数据，必须先发
        ADS1298cmd.start_collect_cmd()（hex: aa0c801103...9ebb）触发。
        发送后 200ms 内应收到 0xA5 帧头。
        """
        try:
            with serial.Serial(port, _EEG_BAUDRATE, timeout=0.3) as ser:
                # 触发 EEG 数据流（关键：ESP32-C3 上电不会自动流）
                start_cmd = bytes(ADS1298cmd.start_collect_cmd())
                ser.write(start_cmd)
                time.sleep(0.2)
                data = ser.read(200)
                return len(data) > 0 and b'\xa5' in data
        except Exception:
            return False

    # ========== 连接/断开外设 ==========

    def _on_connect_clicked(self):
        """统一连接/断开 EEG + IMU 外设。

        判断依据是当前的连接状态（_eeg_connected/_imu_connected），
        而非按钮 checked 状态——这样无论通过真实 click（先 setChecked
        再回调）还是测试直接调用本方法，行为都一致。
        """
        logger.info(f"_on_connect_clicked 触发, eeg_connected={self._eeg_connected}, imu_connected={self._imu_connected}")
        if self._eeg_connected or self._imu_connected:
            self._disconnect_external_devices()
        else:
            self._connect_external_devices()

    def _connect_external_devices(self):
        """自动扫描并启动 EEG + IMU QThread，单路失败不影响另一路。"""
        logger.info("_connect_external_devices 开始扫描")
        self._eeg_connect_state = 'scanning'
        self._imu_connect_state = 'scanning'
        self._eeg_status_label.setText("EEG: 扫描中...")
        self._eeg_status_label.setStyleSheet("color: #D4A574;")
        self._imu_status_label.setText("IMU: 扫描中...")
        self._imu_status_label.setStyleSheet("color: #D4A574;")
        QApplication.processEvents()  # 让 UI 立即刷新

        eeg_port, imu_port = self._auto_detect_devices()
        logger.info(f"扫描结果: eeg_port={eeg_port}, imu_port={imu_port}")
        any_success = False

        # 启动 EEG
        if eeg_port:
            try:
                self._eeg_serial = serial.Serial(
                    port=eeg_port, baudrate=_EEG_BAUDRATE, timeout=0.1
                )
                # 关键：ESP32-C3 WIFI 模块固件不会主动流 EEG 数据，
                # 必须发 start_collect_cmd 触发（参照 main_eeg.py）
                self._eeg_serial.write(bytes(ADS1298cmd.start_collect_cmd()))
                self._eeg_ads_data = ADS1298Data()
                self._eeg_data_lock = threading.Lock()
                self._eeg_reader = ADS1299Reader(
                    serial_port=self._eeg_serial,
                    ads_data=self._eeg_ads_data,
                    data_lock=self._eeg_data_lock,
                )
                self._eeg_reader.data_updated.connect(self._on_eeg_data)
                self._eeg_reader.start()
                self._eeg_connected = True
                self._eeg_connect_state = 'connected_waiting'
                self._eeg_status_label.setText("EEG: 已连接，等待数据...")
                self._eeg_status_label.setStyleSheet("color: #D4B14A;")  # 黄色
                any_success = True
                logger.info(f"EEG reader started on {eeg_port}")
            except Exception as e:
                if self._eeg_serial is not None:
                    try:
                        self._eeg_serial.close()
                    except Exception:
                        pass
                    self._eeg_serial = None
                self._eeg_reader = None
                self._eeg_connected = False
                self._eeg_connect_state = 'failed'
                self._eeg_status_label.setText(f"EEG: ✗ {e}")
                self._eeg_status_label.setStyleSheet("color: #E06C75;")
                logger.error(f"EEG connect failed: {e}")
        else:
            self._eeg_connected = False
            self._eeg_connect_state = 'failed'
            self._eeg_status_label.setText("EEG: ✗ 未检测到（请插入 ADS1299 WIFI 模块）")
            self._eeg_status_label.setStyleSheet("color: #E06C75;")

        # 启动 IMU
        if imu_port:
            try:
                self._imu_serial = serial.Serial(
                    port=imu_port, baudrate=_IMU_BAUDRATE, timeout=0.1
                )
                self._imu_reader = ESP32ImuReader(serial_port=self._imu_serial)
                self._imu_reader.data_updated.connect(self._on_imu_data)
                self._imu_reader.error.connect(self._on_imu_error)
                self._imu_reader.start()
                self._imu_connected = True
                self._imu_connect_state = 'connected_waiting'
                self._imu_status_label.setText("IMU: 已连接，等待数据...")
                self._imu_status_label.setStyleSheet("color: #D4B14A;")  # 黄色
                any_success = True
                logger.info(f"IMU reader started on {imu_port}")
            except Exception as e:
                if self._imu_serial is not None:
                    try:
                        self._imu_serial.close()
                    except Exception:
                        pass
                    self._imu_serial = None
                self._imu_reader = None
                self._imu_connected = False
                self._imu_connect_state = 'failed'
                self._imu_status_label.setText(f"IMU: ✗ {e}")
                self._imu_status_label.setStyleSheet("color: #E06C75;")
                logger.error(f"IMU connect failed: {e}")
        else:
            self._imu_connected = False
            self._imu_connect_state = 'failed'
            self._imu_status_label.setText("IMU: ✗ 未检测到（请插入 ESP32+IMU）")
            self._imu_status_label.setStyleSheet("color: #E06C75;")

        if not any_success:
            self._connect_button.setChecked(False)
            QMessageBox.critical(self, "连接失败", "EEG 和 IMU 都启动失败，请检查端口")
        else:
            self._connect_button.setChecked(True)
            self._connect_button.setText("🔌 断开外设")

    def _disconnect_external_devices(self):
        """停止 EEG 和 IMU QThread。

        ADS1299Reader/ESP32ImuReader 的 run() 是 while self._running 自旋循环
        + msleep()，不进 Qt event loop，所以 QThread.quit()（事件循环退出）
        对它们无效（no-op）。直接调它们的 stop()——内部置 _running=False +
        wait(2000)，让线程正常退出而不是被 terminate() 强杀。
        """
        # EEG
        if self._eeg_reader is not None:
            try:
                self._eeg_reader.stop()
            except Exception as e:
                logger.warning(f"disconnect EEG exception: {e}")
            self._eeg_reader = None
        if self._eeg_serial is not None:
            # 发 stop_collect_cmd 让 ESP32-C3 停止数据流（避免下次连接残留）
            try:
                self._eeg_serial.write(bytes(ADS1298cmd.stop_collect_cmd()))
                time.sleep(0.05)
            except Exception:
                pass
            try:
                self._eeg_serial.close()
            except Exception:
                pass
            self._eeg_serial = None
        self._eeg_status_label.setText("EEG: ✗ 未连接")
        self._eeg_status_label.setStyleSheet("color: gray;")
        self._eeg_connected = False
        # 断开后回到 idle：让 _update_status_labels 不会用失败/已连接语义覆盖此文本
        self._eeg_connect_state = 'idle'
        self._last_eeg_data_ts = 0.0

        # IMU
        if self._imu_reader is not None:
            try:
                self._imu_reader.stop()
            except Exception as e:
                logger.warning(f"disconnect IMU exception: {e}")
            self._imu_reader = None
        if self._imu_serial is not None:
            try:
                self._imu_serial.close()
            except Exception:
                pass
            self._imu_serial = None
        self._imu_status_label.setText("IMU: ✗ 未连接")
        self._imu_status_label.setStyleSheet("color: gray;")
        self._imu_connected = False
        # 断开后回到 idle：让 _update_status_labels 不会用失败/已连接语义覆盖此文本
        self._imu_connect_state = 'idle'
        self._last_imu_data_ts = 0.0

        self._connect_button.setChecked(False)
        self._connect_button.setText("🔌 脑控外设连接")

    def _on_eeg_data(self, data):
        """EEG 数据回调：缓存最近 1500 样本 + 启动 50ms 节流 canvas 刷新。

        使用预分配 ndarray + 切片赋值实现零分配滚动窗口：
        - 单帧 n >= 1000：直接覆盖整个 buffer
        - 单帧 n < 1000：左移 n 位（丢弃最旧的 n 个），尾部追加新数据
        buffer 始终保持按时间顺序（旧在左，新在右），其他读取处无需改。
        """
        if data is None or len(data) == 0:
            return
        self._last_eeg_data_ts = time.time()  # 数据有效，记录心跳
        try:
            arr = np.asarray(data, dtype=float)
        except Exception as e:
            logger.warning(f"EEG data parse failed: {e}")
            return
        if arr.ndim == 1:
            arr = arr.reshape(1, -1)

        capacity = self._eeg_buffer.shape[1]  # 1500
        n = arr.shape[1]
        n_ch = min(self._eeg_buffer.shape[0], arr.shape[0])

        if n >= capacity:
            # 单帧覆盖：只保留最后 capacity 个样本
            self._eeg_buffer[:n_ch, :] = arr[:n_ch, -capacity:]
            self._eeg_buffer_samples = capacity
        else:
            # 左移 n 位：把 [n:] 复制到 [:-n]，腾出尾部 n 位
            self._eeg_buffer[:n_ch, :-n] = self._eeg_buffer[:n_ch, n:]
            self._eeg_buffer[:n_ch, -n:] = arr[:n_ch, -n:]
            self._eeg_buffer_samples = min(
                capacity, self._eeg_buffer_samples + n
            )

        # 50ms 节流：避免每帧都触发 matplotlib 重绘
        if not self._canvas_refresh_timer.isActive():
            self._canvas_refresh_timer.start(50)

    def _on_imu_data(self, data):
        """IMU 数据回调：更新 GroupBox label + 推给 TiltIndicator + 路由到 ImuHandler。

        ESP32ImuReader data_format='auto'，3 float → euler [pitch, roll, yaw]（度），
        4 float → quaternion [q0,q1,q2,q3]。euler 直送 TiltIndicator；
        quaternion 经 HeadPoseCalculator 转 pitch/roll（带 LPF）。
        任务 6：pitch/roll → ImuHandler.update() 得到 TiltDirection，
        缓存到 _current_tilt，下次 _tick_state_machine() 喂状态机。
        """
        if data is None:
            return
        try:
            n = len(data)
        except TypeError:
            return
        self._last_imu_data_ts = time.time()  # data 有效，记录心跳
        try:
            if n >= 4:
                q = self._normalize_quaternion(data[0], data[1], data[2], data[3])
                if q is None:
                    return
                if self._imu_quat_ref is None:
                    self._collect_imu_quat_reference(q)
                    # 相对正前方尚未建立时，显式喂 0/0，避免全局欧拉角
                    # 在 ±180° 附近把校准期也显示成右转。
                    q0, q1, q2, q3 = 1.0, 0.0, 0.0, 0.0
                else:
                    q0, q1, q2, q3 = self._relative_quaternion(
                        self._imu_quat_ref, q
                    )
                t_ms = time.time() * 1000.0
                pitch, roll, _, _ = self._head_pose_calc.update(
                    q0, q1, q2, q3, t_ms
                )
                self._quaternion_label.setText(
                    f"四元数: w={q[0]:.2f} x={q[1]:.2f} y={q[2]:.2f} z={q[3]:.2f}"
                )
                self._euler_label.setText(
                    f"pitch={pitch:.1f}° roll={roll:.1f}°"
                )
            elif n >= 3:
                pitch, roll, yaw = data[0], data[1], data[2]
                self._quaternion_label.setText(
                    f"pitch={pitch:.1f}° roll={roll:.1f}° yaw={yaw:.1f}°"
                )
                self._euler_label.setText(
                    f"pitch={pitch:.1f}° roll={roll:.1f}° yaw={yaw:.1f}°"
                )
            else:
                return
            self._tilt_indicator.set_tilt(float(pitch), float(roll))

            # 任务 6：pitch/roll → TiltDirection（带校准/死区/锁定）
            # 校准：未校准时累积前 _imu_cal_frames_needed 帧作为基线，
            # 满后 finish_calibration()。ImuHandler.update 在未校准时返回 NONE，
            # 所以 _current_tilt 在校准完成前一直是 NONE（运动指令保持 STOP）。
            # 用户点"设为正前方"按钮 → _on_reset_forward → reset() 重新校准。
            if not self._imu_handler.is_calibrated:
                self._imu_handler.feed_calibration(float(pitch), float(roll))
                self._imu_cal_frames_collected += 1
                # 校准期间也发 0，让 chassis_serial_node 时间戳不超时
                self._publish_head_pose(0.0)
                if self._imu_cal_frames_collected >= self._imu_cal_frames_needed:
                    self._imu_handler.finish_calibration()
                    logger.info(
                        f"IMU 校准完成（{self._imu_cal_frames_collected} 帧）："
                        f"pitch_0={self._imu_handler._pitch_0:+.2f}° "
                        f"roll_0={self._imu_handler._roll_0:+.2f}°"
                    )
                self._current_tilt = TiltDirection.NONE
            else:
                self._current_tilt = self._imu_handler.update(
                    float(pitch), float(roll)
                )
                # 发布校准后 roll 给 chassis_serial_node 做 EEG FORWARD/BACKWARD
                # 帧的方向微调（spec § 4.1）。未校准时 calibrated_roll 返回 0，
                # 不会污染 chassis_serial_node 的 roll 缓存。
                self._publish_head_pose(
                    self._imu_handler.calibrated_roll(float(roll))
                )
        except Exception as e:
            logger.warning(f"IMU data parse failed: {e}")

    @staticmethod
    def _normalize_quaternion(q0, q1, q2, q3):
        try:
            vals = [float(q0), float(q1), float(q2), float(q3)]
        except (TypeError, ValueError):
            return None
        norm = math.sqrt(sum(v * v for v in vals))
        if norm < 1e-6:
            return None
        return tuple(v / norm for v in vals)

    @staticmethod
    def _quat_conjugate(q):
        return (q[0], -q[1], -q[2], -q[3])

    @staticmethod
    def _quat_multiply(a, b):
        aw, ax, ay, az = a
        bw, bx, by, bz = b
        return (
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        )

    def _collect_imu_quat_reference(self, q):
        if self._imu_quat_cal_samples:
            first = self._imu_quat_cal_samples[0]
            if sum(a * b for a, b in zip(first, q)) < 0.0:
                q = tuple(-v for v in q)
        self._imu_quat_cal_samples.append(q)
        if len(self._imu_quat_cal_samples) < self._imu_cal_frames_needed:
            return

        sums = [sum(sample[i] for sample in self._imu_quat_cal_samples)
                for i in range(4)]
        self._imu_quat_ref = self._normalize_quaternion(*sums)
        self._imu_quat_cal_samples.clear()
        self._head_pose_calc.reset()
        logger.info(f"脑控 IMU 四元数正前方参考已建立: {self._imu_quat_ref}")

    def _relative_quaternion(self, q_ref, q_current):
        return self._normalize_quaternion(
            *self._quat_multiply(self._quat_conjugate(q_ref), q_current)
        ) or (1.0, 0.0, 0.0, 0.0)

    def _on_imu_error(self, message):
        """IMU 串口/解析错误：立即清零头姿，避免保持最后一次运动指令。"""
        logger.error(f"IMU reader error: {message}")
        self._current_tilt = TiltDirection.NONE
        self._last_imu_data_ts = 0.0
        self._imu_connect_state = 'failed'
        self._imu_connected = False
        self._on_motion_command(MotionCommand.STOP)

    def _expire_stale_imu_tilt(self):
        if self._last_imu_data_ts <= 0:
            return
        if time.time() - self._last_imu_data_ts <= _IMU_STALE_TIMEOUT_SEC:
            return
        if self._current_tilt != TiltDirection.NONE:
            logger.warning("IMU 数据超时，清零头姿运动指令")
        self._current_tilt = TiltDirection.NONE

    def _update_status_labels(self):
        """根据 EEG/IMU 数据心跳时间戳更新左侧栏状态 label。

        判定规则（避免串口已开但无数据时误导用户）：
        - connect_state == 'failed'：保留 _connect_external_devices 设置的失败文本
        - connect_state == 'scanning'：保留 _connect_external_devices 设置的"扫描中..."文本
        - 最近 1s 有数据：显示 "✓ 已连接"（绿）
        - 其他（无数据或数据过期）：显示 "✗ 未连接"（红）
        """
        now = time.time()
        # EEG
        if self._eeg_connect_state not in ('failed', 'scanning'):
            if now - self._last_eeg_data_ts < 1.0 and self._last_eeg_data_ts > 0:
                self._eeg_status_label.setText("EEG: ✓ 已连接")
                self._eeg_status_label.setStyleSheet("color: #6FD68C;")
            else:
                self._eeg_status_label.setText("EEG: ✗ 未连接")
                self._eeg_status_label.setStyleSheet("color: #E06C75;")
        # IMU 左栏（串口状态）+ 右栏（头部姿态 GroupBox）共用同一心跳判定
        if self._imu_connect_state not in ('failed', 'scanning'):
            if now - self._last_imu_data_ts < 1.0 and self._last_imu_data_ts > 0:
                self._imu_status_label.setText("IMU: ✓ 已连接")
                self._imu_status_label.setStyleSheet("color: #6FD68C;")
                # 右栏 _imu_state_label：与左栏同步，避免左右栏状态矛盾
                self._imu_state_label.setText("状态: ✓ 已连接")
                self._imu_state_label.setStyleSheet("color: #6FD68C; font-weight: bold;")
            else:
                self._imu_status_label.setText("IMU: ✗ 未连接")
                self._imu_status_label.setStyleSheet("color: #E06C75;")
                self._imu_state_label.setText("状态: ✗ 未连接")
                self._imu_state_label.setStyleSheet("color: #E06C75; font-weight: bold;")

    def _update_motion_banner(self):
        """根据 ControlState + MotionCommand 更新顶部横幅文字和染色。

        优先级：DISABLED > LOCKED > 运动指令 > STOP。
        参照 muscles-braincontrol main_eeg.py:1775-1820 风格。
        """
        state = self._state_machine.state
        cmd = self._last_cmd

        # 默认（STOP / None / ACTIVE 但无指令）
        text = "静止"
        bg = "#f5f5f5"
        fg = "#888"

        if state == ControlState.DISABLED:
            text = "疲劳锁定"
            bg = "#a00"
            fg = "white"
        elif state == ControlState.LOCKED:
            text = "主动锁定"
            bg = "#a80"
            fg = "white"
        elif cmd == MotionCommand.FORWARD:
            text = "前进"
            bg = "#4a8"
            fg = "white"
        elif cmd == MotionCommand.BACKWARD:
            text = "后退"
            bg = "#a84"
            fg = "white"
        elif cmd == MotionCommand.LEFT:
            text = "左转"
            bg = "#ca0"
            fg = "white"
        elif cmd == MotionCommand.RIGHT:
            text = "右转"
            bg = "#ca0"
            fg = "white"
        # STOP / None 走默认"静止"

        self._motion_banner_label.setText(text)
        # 边框颜色：浅灰底用深灰 #888（否则看不见），其他饱和背景色用同色（形成实心厚框）。
        border_color = "#888" if bg == "#f5f5f5" else bg
        self._motion_banner_label.setStyleSheet(
            f"background-color: {bg}; color: {fg};"
            f"padding: 10px; border-radius: 6px; margin: 4px;"
            f"border: 3px solid {border_color};"
        )

    def _refresh_canvases(self):
        """50ms 节流回调：重绘 8 通道波形 + 500ms 周期跑 FocusDetector。

        运动控制由 _control_timer 独立驱动，避免 matplotlib/SVM 影响
        IMU 直控实时性。
        """
        if self._eeg_buffer_samples == 0:
            return

        # 8 通道波形（参照 main_eeg.py:_update_display 风格）
        # 用 set_data 替代 clear+plot，高效；显示窗口最近 1500 样本
        self._update_wave_display()

        # FocusDetector 每 500ms 一次（与 muscles-braincontrol/main_eeg.py
        # _focus_tick 对齐）；只喂最近 2s 数据，不能喂 3s UI 波形缓冲。
        now = time.time()
        if now - self._last_focus_compute > 0.5:
            if self._eeg_buffer_samples >= _FOCUS_WINDOW_SAMPLES:
                self._last_focus_compute = now
                try:
                    # 外部 CAR（Common Average Reference）：8 通道逐时刻减均值，
                    # 与 main_eeg.py:834-842 完全对齐。FocusDetector 内部还会
                    # 再做一次 CAR，这是旧训练/推理链路的一部分。
                    window = self._eeg_buffer[:, -_FOCUS_WINDOW_SAMPLES:].T
                    car_mean = np.mean(window, axis=1, keepdims=True)
                    window_car = window - car_mean
                    result = self._focus_detector.update(window_car)
                    self._on_focus_result(result)
                except Exception as e:
                    logger.warning(f"FocusDetector update failed: {e}")

    # ========== matplotlib 子图刷新（参照 main_eeg.py:_update_*）==========

    def _update_wave_display(self):
        """刷新 8 通道波形（blit 增量重绘）。

        blit 流程：
        1. 首帧或背景丢失时，draw 一次 + copy_from_bbox 捕获背景
        2. 每帧 restore_region 恢复背景（清空旧线条）
        3. set_data 更新 8 条线
        4. draw_artist 增量绘制每条线
        5. blit 把 axes 区域增量推到屏幕

        ylim/xlim 固定（_init_wave_plot 设定，覆盖 8 通道 × 200μV 偏移 + 3 秒窗口），不每帧重算。
        """
        if self._eeg_buffer_samples < 100:
            return
        canvas = self._waveform_canvas
        n_ch = min(8, self._eeg_buffer.shape[0])
        disp_n = min(self._eeg_buffer_samples, 1500)
        # x 轴用秒数（500Hz 采样率），与 _init_wave_plot 的 xlim(0, 3.0) 对齐。
        # 右对齐：最新数据在 x=3.0（右边缘），老数据向左延伸。
        # 启动期 disp_n < 1500 时数据靠右，左半部分随数据累积逐渐填满；
        # 满 3 秒后整画面滚动（新数据从右边进，老数据从左边出）。
        x = np.arange(disp_n) / 500.0 + (3.0 - disp_n / 500.0)

        # 1. 背景未捕获时先捕获（首帧 / resize 后）
        if self._wave_bg is None:
            try:
                canvas.draw()  # 一次性全量画
            except Exception as e:
                # offscreen/极小窗口下 matplotlib constrained_layout 偶发
                # Singular matrix；波形可降级，控制环不能被绘图异常打断。
                logger.warning(
                    f"wave canvas initial draw failed, retry without "
                    f"constrained_layout: {e}"
                )
                try:
                    self._wave_fig.set_constrained_layout(False)
                    canvas.draw()
                except Exception as retry_e:
                    logger.warning(f"wave canvas draw skipped: {retry_e}")
                    return
            self._wave_bg = canvas.copy_from_bbox(self._wave_ax.bbox)

        # 2. 恢复背景（清除上一帧的线条残留）
        canvas.restore_region(self._wave_bg)

        # 3 + 4. 更新 + 增量绘制每条线
        # CAR（跨 8 通道减均值，去除共同噪声），与 main_eeg.py:_fill_wave_buffer
        # 对齐。raw_data 经 CAR 后波形更干净（眨眼/心电/市电残余共同成分被消）。
        window = self._eeg_buffer[:n_ch, -disp_n:]  # (n_ch, disp_n)
        car_mean = np.mean(window, axis=0, keepdims=True)  # 跨通道逐时刻均值
        window_car = window - car_mean
        for ch in range(n_ch):
            seg = window_car[ch]
            offset = ch * 200
            y = seg + offset
            self._wave_lines[ch].set_data(x, y)
            self._wave_ax.draw_artist(self._wave_lines[ch])

        # 5. blit 推到屏幕
        canvas.blit(self._wave_ax.bbox)
        canvas.flush_events()

    def _update_score_plot(self):
        """刷新专注度分数历史曲线（参照 main_eeg.py:_update_score_plot）。"""
        if len(self._time_history) < 2:
            return
        times = np.array(self._time_history)
        times = times - times[0]  # 相对时间（从 0 开始）
        scores = np.array(self._score_history)
        self._score_line.set_data(times, scores)
        self._score_ax.set_xlim(times[0], max(times[-1], times[0] + 10))
        self._score_ax.set_ylim(0, 100)
        self._score_canvas.draw_idle()

    def _update_band_plot(self, result):
        """刷新 5 频段功率柱状图（参照 main_eeg.py:_update_band_plot）。

        从 FocusResult.features 聚合 5 频段均值（跨 8 通道）。
        features 必须是 dict（MagicMock 测试场景下可能不是，跳过）。
        """
        from ..braincontrol.eeg_bands import BANDS
        feats = getattr(result, 'features', None)
        if not isinstance(feats, dict):
            return  # MagicMock 或 None，跳过避免 NaN
        means = []
        for band_name in BANDS:
            vals = [feats.get(f'power_{band_name}_ch{ch}', 0.0)
                    for ch in range(8)]
            try:
                arr = np.array(vals, dtype=float)
                means.append(float(arr.mean()) if len(arr) else 0.0)
            except (TypeError, ValueError):
                means.append(0.0)
        for bar, val in zip(self._band_bars, means):
            bar.set_height(val)
        max_val = max(max(means) if means else [0.0], 1e-6) * 1.3
        self._bands_ax.set_ylim(0, max_val)
        self._score_canvas.draw_idle()


    def _on_focus_result(self, result):
        """FocusResult 处理：更新 GUI + 发布 /focus_state（5Hz throttle）+ 喂状态机。"""
        if result is None:
            return
        # GUI 更新（getattr 兜底 + float() 强转，避免 MagicMock 属性
        # 拦截返回非数值；FocusResult 实际字段为 score/state/p_focus/
        # confidence/emg_level，emg_pollution 不存在）
        state = getattr(result, 'state', 'unknown') or 'unknown'
        try:
            score = float(getattr(result, 'score', 0.0) or 0.0)
        except (TypeError, ValueError):
            score = 0.0
        try:
            p_focus = float(getattr(result, 'p_focus', 0.0) or 0.0)
        except (TypeError, ValueError):
            p_focus = 0.0
        try:
            confidence = float(getattr(result, 'confidence', 0.0) or 0.0)
        except (TypeError, ValueError):
            confidence = 0.0
        emg_raw = getattr(result, 'emg_level', None)
        if emg_raw is None:
            emg_raw = getattr(result, 'emg_pollution', 0.0)
        try:
            emg_level = float(emg_raw or 0.0)
        except (TypeError, ValueError):
            emg_level = 0.0

        # 分数：纯数字 24pt 显示（字号已在 _build_left_column 设置）
        self._focus_score_label.setText(f"{score:.0f}")

        # 状态显示与 muscles-braincontrol/main_eeg.py 对齐：
        # focused=清醒/专注，neutral=正常，relaxed=瞌睡。
        if state == 'focused':
            self._focus_state_label.setText("专注")
            self._focus_state_label.setStyleSheet(
                "background-color: #4a8; color: white; padding: 6px; border-radius: 4px;"
            )
        elif state == 'relaxed':
            self._focus_state_label.setText("瞌睡")
            self._focus_state_label.setStyleSheet(
                "background-color: #a00; color: white; padding: 6px; border-radius: 4px;"
            )
        elif state == 'neutral':
            self._focus_state_label.setText("正常")
            self._focus_state_label.setStyleSheet(
                "background-color: #a80; color: white; padding: 6px; border-radius: 4px;"
            )
        else:
            self._focus_state_label.setText(state)
            self._focus_state_label.setStyleSheet(
                "background-color: #888; color: white; padding: 6px; border-radius: 4px;"
            )
        self._focus_progress.setValue(int(max(0.0, min(1.0, p_focus)) * 100))
        self._quality_label.setText(
            f"EMG: {emg_level:.2f}  置信度: {confidence:.2f}"
        )

        # 分数历史曲线 + 频带柱状图（参照 main_eeg.py:_update_score_plot/_update_band_plot）
        # 旧主程序画的是 FocusResult.score（平滑后的 0-100），不是原始 p_focus。
        score_pct = max(0.0, min(100.0, score))
        self._time_history.append(time.time())
        self._score_history.append(score_pct)
        # 限制历史长度（保留最近 60 秒 @500ms = 120 点）
        if len(self._time_history) > 120:
            self._time_history = self._time_history[-120:]
            self._score_history = self._score_history[-120:]
        self._update_score_plot()
        self._update_band_plot(result)

        # 任务 6：缓存 focus_state 供状态机消费（_tick_state_machine）
        if state in ('focused', 'neutral', 'relaxed'):
            self._focus_state = state

        # 发布 /focus_state（5Hz throttle：≥200ms 间隔）
        now = time.time()
        if now - self._last_focus_publish >= 0.2:
            self._last_focus_publish = now
            from std_msgs.msg import String
            msg = String()
            msg.data = f"{state}:{p_focus:.3f}"
            self._focus_state_pub.publish(msg)

    # ========== 脑控模式 toggle ==========

    def _on_eeg_mode_toggled(self):
        """切换脑控模式，发布 /eeg_mode_active Bool。"""
        from std_msgs.msg import Bool
        self._eeg_mode_active = self._eeg_mode_toggle.isChecked()
        msg = Bool()
        msg.data = self._eeg_mode_active
        self._eeg_mode_pub.publish(msg)
        self._eeg_mode_toggle.setText(
            f"🧠 脑控模式：{'激活' if self._eeg_mode_active else '待机'}"
        )
        logger.info(f"eeg_mode_active={self._eeg_mode_active}")

    def _eeg_heartbeat(self):
        """1Hz 心跳：持续发布 /eeg_mode_active（防 chassis_serial_node 3s fallback）。

        BrainControlTab toggle 是事件触发（点击时发一次），chassis_serial_node 需要
        持续心跳确认 BrainControlTab 还活着。这个定时器在 toggle 激活时每秒发一次。
        """
        if self._eeg_mode_active:
            from std_msgs.msg import Bool
            msg = Bool()
            msg.data = True
            self._eeg_mode_pub.publish(msg)

    # ========== ControlStateMachine 接线（任务 6）==========

    def _tick_control_loop(self):
        """固定 50ms 控制循环：状态显示 + 状态机 + 连续速度发布。

        这是 IMU 直控底盘的实时路径。它独立于 EEG 波形刷新，保证持续头姿
        输入会持续刷新 /cmd_vel_eeg，底盘侧不会因为 1s 未更新而停车。
        """
        self._update_status_labels()
        self._expire_stale_imu_tilt()
        self._tick_clench_detector()
        self._tick_state_machine()
        self._update_motion_banner()
        self._update_motion_command_label()

    def _tick_clench_detector(self):
        """固定控制环内更新咬牙检测，避免依赖 EEG 波形绘图刷新。

        ClenchDetector 的模型按 2s EEG 窗口训练；UI 波形缓冲是 3s，
        因此这里只取最后 2s 有效数据，避免短促咬牙被 3s 窗口稀释。
        """
        if self._clench_detector is None:
            return
        if self._eeg_buffer_samples < _CLENCH_WINDOW_SAMPLES:
            return

        now = time.time()
        elapsed = now - self._last_clench_compute
        if elapsed <= self._clench_interval:
            return

        if self._last_clench_compute > 0:
            dt_ms = int(elapsed * 1000)
            dt_ms = max(1, min(dt_ms, 1000))
        else:
            dt_ms = int(self._clench_interval * 1000)
        self._last_clench_compute = now

        try:
            window = self._eeg_buffer[:, -_CLENCH_WINDOW_SAMPLES:].T
            clench_result = self._clench_detector.update(window, dt_ms=dt_ms)
            if clench_result.event:
                self._on_clench_event(clench_result)
            elif clench_result.is_clenching:
                self._clench_label.setText(
                    f"咬牙: 持续 (P={clench_result.proba:.2f})"
                )
            else:
                self._clench_label.setText(
                    f"咬牙: -- (P={clench_result.proba:.2f})"
                )
        except Exception as e:
            logger.warning(f"ClenchDetector update failed: {e}")

    def _tick_state_machine(self):
        """每 50ms 调用：把缓存里的 focus_state/toggle/tilt 喂给状态机。

        实测 ControlStateMachine.update(focus_state, toggle_event, tilt, dt_ms)
        是同步 API，没有 pyqtSignal；输出 MotionCommand 由本方法直接推给
        _on_motion_command。

        dt_ms 推算：用 time.time() 算距上次 tick 的实际间隔，避免 QTimer
        抖动让状态机计时漂移；首帧用模块默认 _STATE_MACHINE_DT_MS。

        可见性暂停（规格 § 4.4 / § 7.4 / R9）：tab 不可见时跳过状态机
        输出（保留 EEG/IMU 监测，但停止发布 /cmd_vel_eeg），避免用户切到
        Tab 1/2 时脑控仍在发指令造成混乱。
        """
        # R9：tab 不可见时暂停状态机输出（避免误操作）
        if not self.isVisible():
            return

        now = time.time()
        if self._last_state_machine_tick > 0:
            dt_ms = int((now - self._last_state_machine_tick) * 1000)
            # 防御：QTimer 抖动可能给到 0 或超大值，clamp 到合理区间
            dt_ms = max(1, min(dt_ms, 1000))
        else:
            dt_ms = _STATE_MACHINE_DT_MS
        self._last_state_machine_tick = now

        try:
            cmd = self._state_machine.update(
                self._focus_state,
                self._toggle_pending,
                self._current_tilt,
                dt_ms,
            )
            # 消费 toggle pending（rising edge 只触发一次）
            self._toggle_pending = False
            self._on_motion_command(cmd)
        except Exception as e:
            logger.error(f"ControlStateMachine update failed: {e}")

    def _publish_head_pose(self, roll_deg: float) -> None:
        """发布校准后的头部 roll 到 /eeg_head_pose（chassis_serial_node 消费）。

        chassis_serial_node 在 EEG override 期间把 roll 算成 direction_angle
        偏移加到下位机串口帧；非 EEG 模式下 chassis_serial_node 不消费此 topic。
        """
        if self._head_pose_pub is None:
            return
        try:
            from std_msgs.msg import Float64
            msg = Float64()
            msg.data = float(roll_deg)
            self._head_pose_pub.publish(msg)
        except Exception as e:
            logger.warning(f"publish /eeg_head_pose failed: {e}")

    def _on_motion_command(self, cmd):
        """ControlStateMachine 输出 MotionCommand 时调用。

        - 调 MotionCommander 发布 /cmd_vel_eeg
        - 更新运动指令 label 和状态机 label

        注意：非 STOP 也必须持续发布。底盘 mux 对 /cmd_vel_eeg 有 1s
        新鲜度 watchdog；如果持续低头只发第一帧，轮椅会在 1s 后归零，
        表现为"能激活但几乎不听指令"。
        """
        self._last_cmd = cmd

        # 发布 /cmd_vel_eeg
        try:
            self._motion_commander.update(cmd)
        except Exception as e:
            logger.error(f"MotionCommander.update failed: {e}")

        # 更新状态机 label（state 是 ControlState 枚举）
        try:
            state = self._state_machine.state
            state_name = state.name if hasattr(state, 'name') else str(state)
        except Exception:
            state_name = "?"
        self._state_machine_label.setText(f"状态机: {state_name}")
        # 同步更新运动指令 label 的彩色背景（镜像顶部横幅配色）
        self._update_motion_command_label()

    # 运动指令 label 配色（镜像 _update_motion_banner 的颜色映射）
    _MOTION_CMD_STYLE = {
        # cmd_name → (背景色, 边框色)
        'FORWARD':  ('#4a8', '#4a8'),
        'BACKWARD': ('#a84', '#a84'),
        'LEFT':     ('#ca0', '#ca0'),
        'RIGHT':    ('#ca0', '#ca0'),
        'STOP':     ('#888', '#888'),
    }

    def _update_motion_command_label(self):
        """根据 state + _last_cmd 更新右栏运动指令 label 的文字+彩色背景。

        优先级与 _update_motion_banner 一致：DISABLED > LOCKED > cmd > STOP。
        DISABLED/LOCKED 时显示"疲劳锁定"/"主动锁定"，覆盖普通运动指令。
        """
        state = self._state_machine.state

        if state == ControlState.DISABLED:
            text = "疲劳锁定"
            bg = "#a00"
        elif state == ControlState.LOCKED:
            text = "主动锁定"
            bg = "#a80"
        else:
            # ACTIVE / 其他：跟随 _last_cmd
            try:
                cmd_name = self._last_cmd.name if hasattr(self._last_cmd, 'name') else 'STOP'
            except Exception:
                cmd_name = 'STOP'
            text = _CMD_CN.get(cmd_name, cmd_name)
            bg_tuple = self._MOTION_CMD_STYLE.get(cmd_name, ('#888', '#888'))
            bg = bg_tuple[0]

        self._motion_command_label.setText(f"指令：{text}")
        self._motion_command_label.setStyleSheet(
            f"font-size: 18pt; font-weight: bold; color: white;"
            f"background-color: {bg}; padding: 8px; border-radius: 6px;"
            f"border: 3px solid {bg};"
        )

    def _on_reset_forward(self):
        """"设为正前方"按钮回调：重置 IMU 校准基准。

        ImuHandler.reset() 会清零累积校准和当前方向，下一帧 IMU 数据进入后
        重新累积校准；用户戴帽对正前方时点此按钮，tilt 即可正常输出方向。
        """
        if self._imu_handler is None:
            return
        try:
            self._imu_handler.reset()
            self._imu_cal_frames_collected = 0
            self._imu_quat_ref = None
            self._imu_quat_cal_samples.clear()
            self._head_pose_calc.reset()
            self._current_tilt = TiltDirection.NONE
            self._on_motion_command(MotionCommand.STOP)
            logger.info("IMU 校准已重置，等待重新累积 20 帧基线")
        except Exception as e:
            logger.warning(f"ImuHandler.reset failed: {e}")

    def _on_clench_event(self, event):
        """咬牙事件：触发 LOCKED ↔ ACTIVE toggle + 更新 GUI。

        ClenchDetector 输出 ClenchResult（含 .event/.is_clenching/.proba），
        本方法只在 .event=True 时被调用（rising edge）。

        咬牙 rising edge 置位 _toggle_pending，下次 _tick_state_machine 把它
        喂给状态机，触发 LOCKED ↔ ACTIVE 切换（spec § 5.2 toggle）。
        """
        # 咬牙 rising edge 触发 toggle（替代原 frown_pending）
        self._toggle_pending = True

        try:
            proba = float(getattr(event, 'proba', 0.0) or 0.0)
        except (TypeError, ValueError):
            proba = 0.0
        self._clench_label.setText(f"咬牙: 触发 (P={proba:.2f})")
        logger.info(f"Clench rising edge: proba={proba:.3f}")
