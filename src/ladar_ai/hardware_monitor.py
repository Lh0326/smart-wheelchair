"""硬件监控采集函数。

抽取自内部监控工具CGN-watching，去掉 PyQt5 依赖，只保留采集逻辑。
所有函数纯读，无副作用（除 get_npu_info 的模块级 _npu_prev 状态用于占空比计算）。

依赖：psutil（CPU/MEM）、i915 sysfs（GPU）、accel runtime PM sysfs（NPU）。
读不到 sysfs 时返回 0，不抛错。
"""
import os
import time

try:
    import psutil
except ImportError:
    psutil = None


def get_cpu_info():
    """返回 (各核利用率 list, 总利用率 %, 频率 MHz, 温度 °C)。

    温度读不到时返回 None。
    """
    if psutil is None:
        return [], 0, 0, None
    per_core = psutil.cpu_percent(percpu=True)
    total = psutil.cpu_percent(interval=None)
    freq = psutil.cpu_freq()
    freq_mhz = freq.current if freq else 0
    temp = None
    if hasattr(psutil, "sensors_temperatures"):
        temps = psutil.sensors_temperatures()
        for name in ("coretemp", "acpitz", "x86_pkg_temp"):
            if name in temps and temps[name]:
                temp = temps[name][0].current
                break
    return per_core, total, freq_mhz, temp


def get_mem_info():
    """返回 (已用 GB, 总量 GB, 百分比)。"""
    if psutil is None:
        return 0.0, 0.0, 0.0
    m = psutil.virtual_memory()
    return m.used / (1024**3), m.total / (1024**3), m.percent


def get_gpu_info():
    """通过 i915 sysfs 读 Intel GPU 真实瞬时状态。

    返回 (实际频率 MHz, 最大频率 MHz, 利用率 %, (VRAM 已用 GB, VRAM 总量 GB))。
    无 i915 卡时返回 (0, 0, 0, (0, 0))。
    """
    drm = "/sys/class/drm"
    card = None
    try:
        for d in sorted(os.listdir(drm)):
            if not d.startswith("card"):
                continue
            p = os.path.join(drm, d, "device", "driver")
            if os.path.islink(p) and "i915" in os.readlink(p):
                if os.path.exists(os.path.join(drm, d, "gt_act_freq_mhz")):
                    card = d
                    break
    except Exception:
        return 0, 0, 0, (0, 0)

    if not card:
        return 0, 0, 0, (0, 0)

    base = os.path.join(drm, card)
    try:
        act_freq = int(open(f"{base}/gt_act_freq_mhz").read().strip())
    except Exception:
        act_freq = 0
    try:
        max_freq = int(open(f"{base}/gt_RP0_freq_mhz").read().strip())
    except Exception:
        max_freq = 0

    if max_freq > 0 and act_freq > 0:
        util_est = min(100, round(act_freq / max_freq * 100))
    else:
        util_est = 0

    vram_used_gb = vram_total_gb = 0.0
    try:
        dev = os.path.join(base, "device")
        vram_total = int(open(f"{dev}/mem_info_total").read().strip()) / (1024**2)
        avail = int(open(f"{dev}/mem_info_avail").read().strip()) / (1024**2)
        vram_total_gb = vram_total / 1024
        vram_used_gb = (vram_total - avail) / 1024
    except Exception:
        pass

    return act_freq, max_freq, util_est, (vram_used_gb, vram_total_gb)


def _read_sysfs_int(path):
    """读 sysfs 整数，失败返回 0。"""
    try:
        return int(open(path).read().strip())
    except Exception:
        return 0


# NPU runtime PM 占空比计算所需模块级状态
_npu_prev = {"active": 0, "suspended": 0, "time": 0}


def get_npu_info():
    """读 Intel NPU 真实状态。

    返回 (利用率 %, 频率 MHz, 最大频率 MHz)。

    策略（按优先级）：
      1. devfreq 读真实频率（kernel >= 6.11）
      2. runtime PM active/suspended 时间差算占空比
    """
    accel = "/sys/class/accel/accel0"
    if not os.path.exists(accel):
        return 0, 0, 0

    npu_freq = 0
    npu_max_freq = 0

    # 1. 尝试 devfreq
    devfreq = os.path.join(accel, "device", "devfreq")
    if os.path.isdir(devfreq):
        try:
            dfs = os.listdir(devfreq)
        except Exception:
            dfs = []
        if dfs:
            dfp = os.path.join(devfreq, dfs[0])
            npu_freq = _read_sysfs_int(f"{dfp}/cur_freq") // 1000
            npu_max_freq = _read_sysfs_int(f"{dfp}/max_freq") // 1000

    if npu_max_freq > 0 and npu_freq > 0:
        return min(100, round(npu_freq / npu_max_freq * 100)), npu_freq, npu_max_freq

    # 2. runtime PM 占空比
    power_path = os.path.join(accel, "device", "power")
    try:
        active = int(open(os.path.join(power_path, "runtime_active_time")).read().strip())
        suspended = int(open(os.path.join(power_path, "runtime_suspended_time")).read().strip())
    except Exception:
        return 0, 0, 0

    now = time.monotonic_ns()
    util = 0
    global _npu_prev
    if _npu_prev["time"] > 0:
        dt_active = active - _npu_prev["active"]
        dt_suspended = suspended - _npu_prev["suspended"]
        dt_total = dt_active + dt_suspended
        if dt_total > 0:
            util = min(100, round(dt_active / dt_total * 100))

    _npu_prev["active"] = active
    _npu_prev["suspended"] = suspended
    _npu_prev["time"] = now

    return util, 0, 0
