"""PyQt5 主窗口:顶部水平 tab 栏 + 全局硬件监控条 + 状态栏。

硬件监控条直接采集系统数据（CGN-watching 方案），不依赖 /hw_status topic：
  - CPU: psutil.cpu_percent()
  - RAM: psutil.virtual_memory()
  - GPU: /sys/class/drm/i915 card gt_act_freq_mhz / gt_RP0_freq_mhz（Intel Arc 真实频率）
  - NPU: /sys/class/accel/accel0 runtime_active_time 占空比（Intel AI Boost 真实占空比）

不走 ROS2 → 无 DDS 延迟、无 subscription GC 问题、数据准确。
"""
import os
import time

from ament_index_python.packages import get_package_share_directory
from PyQt5.QtCore import Qt, pyqtSignal, QTimer
from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QTabWidget, QVBoxLayout, QHBoxLayout,
    QStatusBar, QLabel, QProgressBar
)

try:
    import psutil
    PSUTIL_AVAILABLE = True
except ImportError:
    PSUTIL_AVAILABLE = False


# ============ 硬件采集（移植自 CGN-watching，纯 sysfs/psutil，无 ROS2 依赖）============

# NPU runtime PM 状态缓存（占空比需基于上次调用差值计算）
_npu_prev = {"active": 0, "suspended": 0, "time": 0}


def _read_sysfs_int(path: str) -> int:
    try:
        return int(open(path).read().strip())
    except Exception:
        return 0


def collect_cpu_percent() -> float:
    """CPU 总占用 %（首次调用返回 0，psutil 需要两次调用建立基线）。"""
    if not PSUTIL_AVAILABLE:
        return 0.0
    return psutil.cpu_percent(interval=None)


def collect_ram() -> tuple:
    """返回 (已用 GB, 总量 GB, 占用 %)。"""
    if not PSUTIL_AVAILABLE:
        return 0.0, 0.0, 0.0
    m = psutil.virtual_memory()
    return m.used / (1024**3), m.total / (1024**3), m.percent


def collect_gpu_percent() -> int:
    """Intel GPU 利用率 %（i915 sysfs：act_freq / RP0_freq）。

    DRM DVFS 会让频率在 450-1700MHz 跳动，正常现象。
    """
    drm = "/sys/class/drm"
    try:
        for d in sorted(os.listdir(drm)):
            if not d.startswith("card"):
                continue
            base = os.path.join(drm, d)
            driver_link = os.path.join(base, "device", "driver")
            if not (os.path.islink(driver_link) and "i915" in os.readlink(driver_link)):
                continue
            if not os.path.exists(os.path.join(base, "gt_act_freq_mhz")):
                continue
            act = _read_sysfs_int(os.path.join(base, "gt_act_freq_mhz"))
            max_f = _read_sysfs_int(os.path.join(base, "gt_RP0_freq_mhz"))
            if max_f > 0 and act > 0:
                return min(100, round(act / max_f * 100))
            return 0
    except Exception:
        pass
    return 0


def collect_npu_percent() -> int:
    """Intel NPU 占空比 %（accel0 runtime_active_time 差值）。

    优先 devfreq cur_freq/max_freq；fallback runtime PM active/suspended 时间差。
    """
    accel = "/sys/class/accel/accel0"
    if not os.path.exists(accel):
        return 0

    # 1. devfreq 真实频率
    devfreq = os.path.join(accel, "device", "devfreq")
    try:
        if os.path.isdir(devfreq):
            dfs = os.listdir(devfreq)
            if dfs:
                dfp = os.path.join(devfreq, dfs[0])
                cur = _read_sysfs_int(os.path.join(dfp, "cur_freq")) // 1000
                mx = _read_sysfs_int(os.path.join(dfp, "max_freq")) // 1000
                if mx > 0 and cur > 0:
                    return min(100, round(cur / mx * 100))
    except Exception:
        pass

    # 2. runtime PM 占空比
    power_path = os.path.join(accel, "device", "power")
    try:
        active = _read_sysfs_int(os.path.join(power_path, "runtime_active_time"))
        suspended = _read_sysfs_int(os.path.join(power_path, "runtime_suspended_time"))
    except Exception:
        return 0

    now = time.monotonic_ns()
    util = 0
    if _npu_prev["time"] > 0:
        dt_active = active - _npu_prev["active"]
        dt_suspended = suspended - _npu_prev["suspended"]
        dt_total = dt_active + dt_suspended
        if dt_total > 0:
            util = min(100, round(dt_active / dt_total * 100))
    _npu_prev["active"] = active
    _npu_prev["suspended"] = suspended
    _npu_prev["time"] = now
    return util


class MainWindow(QMainWindow):
    """智慧轮椅主窗口(沉墨 Deep Ink 视觉)。"""

    tab_changed = pyqtSignal(int)  # tab 切换 signal

    def __init__(self, ros_node):
        super().__init__()
        self._ros = ros_node
        # tab name → widget 映射(QTabWidget 不支持按 name 查找,dict 提供 O(1) 切换)
        self._tabs = {}

        self.setWindowTitle("智慧轮椅 — BrainControl")
        self.resize(1440, 900)
        self.setMinimumSize(1200, 800)

        self._setup_ui()
        self._apply_qss()

    def _setup_ui(self):
        """中央 widget:QTabWidget (上) + HwStrip (下,含时钟)。

        QStatusBar 已完全移除——之前残留的 _hw_label 即使清空也占用一行，
        被 HwStrip 重复显示。HwStrip 现在唯一底部元素，时钟合并到 HwStrip 右侧。
        """
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        # 上:tab 栏(占大部分空间)
        self._tab_widget = QTabWidget()
        layout.addWidget(self._tab_widget, stretch=1)

        # 下:全局硬件监控条(三个 tab 共享,PyQt5 原生,不依赖 web 渲染)
        # 内含 CPU/GPU/NPU/RAM 4 个 cell + 右侧时钟
        self._build_hw_strip()
        layout.addWidget(self._hw_strip_container)

        # tab 切换 signal
        self._tab_widget.currentChanged.connect(self.tab_changed.emit)

    def _build_hw_strip(self):
        """硬件监控条:CPU / GPU / NPU / RAM 4 个 cell + 右侧时钟。

        每个 cell = 标签 + 进度条 + 数值,数据由 QTimer 1Hz 直接采集（不走 /hw_status topic）。
        RAM 显示已用/总量 GB(2x 8G DDR5 = 15G),其它显示 %。
        """
        self._hw_strip_container = QWidget()
        self._hw_strip_container.setStyleSheet("""
            QWidget { background: #050608; border-top: 1px solid rgba(212,165,116,0.12); }
            QLabel { color: #8A7D68; font-size: 11px; }
            QProgressBar {
                background: rgba(255,255,255,0.05);
                border: none;
                border-radius: 2px;
                min-height: 4px;
                max-height: 6px;
            }
            QProgressBar::chunk { background: #D4A574; border-radius: 2px; }
        """)

        strip_layout = QHBoxLayout(self._hw_strip_container)
        strip_layout.setContentsMargins(12, 4, 12, 4)
        strip_layout.setSpacing(12)

        # 4 个 cell:CPU / GPU / NPU / RAM
        self._hw_cells = {}
        for key, label, color in [
            ('cpu', 'CPU', '#D4A574'),
            ('gpu', 'GPU', '#6A9B9B'),
            ('npu', 'NPU', '#8BA571'),
            ('ram', 'RAM', '#B8A98E'),
        ]:
            cell = self._build_hw_cell(label, color)
            self._hw_cells[key] = cell
            strip_layout.addWidget(cell.container)

        strip_layout.addStretch()  # 中间弹性空间

        # 右侧时钟（替代 QStatusBar）
        self._hw_clock = QLabel("--:--:--")
        self._hw_clock.setStyleSheet(
            "color: #B8A98E; font-family: 'Geist Mono', monospace; font-size: 12px; "
            "padding-left: 12px;"
        )
        strip_layout.addWidget(self._hw_clock)

        # 启动 QTimer：1Hz 直接采集（CGN-watching 方案，不走 ROS2）
        if PSUTIL_AVAILABLE:
            psutil.cpu_percent(percpu=True)  # 预热基线
            psutil.cpu_percent(percpu=True)
            self._hw_timer = QTimer(self)
            self._hw_timer.timeout.connect(self._tick_hw_collect)
            self._hw_timer.start(1000)
            # 首次立即采集一次
            QTimer.singleShot(100, self._tick_hw_collect)

    def _tick_hw_collect(self):
        """1Hz 直接采集 CPU/GPU/NPU/RAM 并更新 UI。"""
        try:
            cpu = collect_cpu_percent()
            gpu = collect_gpu_percent()
            npu = collect_npu_percent()
            ram_used, ram_total, ram_pct = collect_ram()

            self._hw_cells['cpu'].bar.setValue(int(cpu))
            self._hw_cells['cpu'].value.setText(f"{cpu:.0f}%")
            self._hw_cells['gpu'].bar.setValue(int(gpu))
            self._hw_cells['gpu'].value.setText(f"{gpu:.0f}%")
            self._hw_cells['npu'].bar.setValue(int(npu))
            self._hw_cells['npu'].value.setText(f"{npu:.0f}%")
            self._hw_cells['ram'].bar.setValue(int(ram_pct))
            self._hw_cells['ram'].value.setText(f"{ram_used:.1f}/{ram_total:.0f}G")
        except Exception as e:
            # 任何采集异常不应阻塞 UI
            pass

    def _build_hw_cell(self, label: str, color: str):
        """单个 hw cell:label + progress + value。返回带 .container / .bar / .value 的对象。"""
        container = QWidget()
        h = QHBoxLayout(container)
        h.setContentsMargins(0, 0, 0, 0)
        h.setSpacing(6)

        label_widget = QLabel(label)
        label_widget.setMinimumWidth(28)

        bar = QProgressBar()
        bar.setRange(0, 100)
        bar.setValue(0)
        bar.setTextVisible(False)
        # 每个进度条独立 chunk 颜色
        bar.setStyleSheet(f"""
            QProgressBar::chunk {{ background: {color}; border-radius: 2px; }}
        """)

        value = QLabel("--")
        value.setMinimumWidth(70)
        value.setStyleSheet("color: #B8A98E; font-family: 'Geist Mono', monospace; font-size: 11px;")

        h.addWidget(label_widget)
        h.addWidget(bar, stretch=1)
        h.addWidget(value)

        # 包装一个 namespace 对象
        class _Cell:
            pass
        cell = _Cell()
        cell.container = container
        cell.bar = bar
        cell.value = value
        return cell

    def add_tab(self, name: str, widget: QWidget):
        """添加一个 tab,返回索引。"""
        idx = self._tab_widget.addTab(widget, name)
        self._tabs[name] = widget
        return idx

    def switch_to(self, name: str):
        """切换到指定 tab name;未找到时 no-op。"""
        widget = self._tabs.get(name)
        if widget:
            self._tab_widget.setCurrentWidget(widget)

    def update_clock(self, time_str: str):
        """HwStrip 右侧时钟更新。"""
        self._hw_clock.setText(time_str)

    def update_hw_status(self, cpu, mem, gpu, npu,
                          mem_used_gb=None, mem_total_gb=None):
        """更新硬件监控条。

        Parameters
        ----------
        cpu, mem, gpu, npu : float
            占用百分比(0-100)。
        mem_used_gb, mem_total_gb : float, optional
            RAM 已用/总量 GB(显示在 RAM cell,2x 8G DDR5 = ~15G)。
        """
        self._hw_cells['cpu'].bar.setValue(int(cpu))
        self._hw_cells['cpu'].value.setText(f"{cpu:.0f}%")
        self._hw_cells['gpu'].bar.setValue(int(gpu))
        self._hw_cells['gpu'].value.setText(f"{gpu:.0f}%")
        self._hw_cells['npu'].bar.setValue(int(npu))
        self._hw_cells['npu'].value.setText(f"{npu:.0f}%")

        # RAM:显示已用/总量 GB(若调用方提供),否则用百分比
        if mem_used_gb is not None and mem_total_gb is not None:
            ram_pct = (mem_used_gb / mem_total_gb * 100) if mem_total_gb > 0 else 0
            self._hw_cells['ram'].bar.setValue(int(ram_pct))
            self._hw_cells['ram'].value.setText(f"{mem_used_gb:.1f}/{mem_total_gb:.0f}G")
        else:
            self._hw_cells['ram'].bar.setValue(int(mem))
            self._hw_cells['ram'].value.setText(f"{mem:.0f}%")

    def _apply_qss(self):
        """加载沉墨 QSS(文件不存在时静默跳过,不影响功能)。"""
        qss_path = os.path.join(
            get_package_share_directory('wheelchair_app'),
            'resources', 'sinki.qss'
        )
        if os.path.exists(qss_path):
            with open(qss_path, 'r') as f:
                self.setStyleSheet(f.read())
