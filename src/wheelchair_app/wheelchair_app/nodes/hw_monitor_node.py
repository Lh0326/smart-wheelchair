"""硬件监控节点：发布 CPU/内存/GPU/NPU 占用到 /hw_status(std_msgs/String JSON)。

发布频率：1Hz
数据格式(JSON):
{
  "cpu_percent": 45.2,            # 总 CPU 占用 %
  "cpu_per_core": [85, 30, ...],  # 每核占用 %（14 核）
  "mem_percent": 43.5,            # 内存占用 %
  "mem_used_gb": 6.5,             # 已用内存 GB
  "mem_total_gb": 15.0,           # 总内存 GB
  "gpu_percent": 12,              # Intel Arc GPU %(读 sysfs,intel_gpu_top 太重不用)
  "npu_percent": 8,               # Intel NPU %(暂用 YOLO 节点 CPU 近似)
  "load_1m": 13.24,               # load average 1分钟
  "top_processes": [              # TOP 3 CPU 进程
    {"name": "controller_server", "cpu": 18.5},
    ...
  ]
}
"""
import json
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    import psutil
    PSUTIL_AVAILABLE = True
except ImportError:
    PSUTIL_AVAILABLE = False


class HwMonitorNode(Node):
    def __init__(self):
        super().__init__('hw_monitor_node')

        if not PSUTIL_AVAILABLE:
            self.get_logger().error('psutil 不可用,无法采集硬件数据')
            return

        # CPU 百分比需要预热(首次调用返回 0,二次调用让 psutil 建立基线)
        psutil.cpu_percent(percpu=True)
        psutil.cpu_percent(percpu=True)  # 二次预热

        self._pub = self.create_publisher(String, '/hw_status', 10)
        # 1Hz 发布(1s 间隔,psutil cpu_percent 基于上次调用间隔算 %,间隔过短会误报 100%)
        self.create_timer(1.0, self._tick)

        self.get_logger().info('hw_monitor_node 启动 @ 1Hz')

    def _tick(self):
        try:
            data = self._collect()
            msg = String()
            msg.data = json.dumps(data, ensure_ascii=False)
            self._pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f'采集失败: {e}')

    def _collect(self):
        # CPU
        cpu_per_core = psutil.cpu_percent(percpu=True)
        cpu_total = sum(cpu_per_core) / max(len(cpu_per_core), 1)

        # 内存
        mem = psutil.virtual_memory()

        # load average
        load1, _, _ = os.getloadavg()

        # GPU(Intel Arc,读 sysfs,可能无权限,fallback 0)
        gpu = self._read_gpu()

        # NPU(暂用 YOLO 节点 CPU 近似,或 N/A)
        npu = self._estimate_npu()

        # TOP 3 进程
        top = self._top_processes(3)

        return {
            'cpu_percent': round(cpu_total, 1),
            'cpu_per_core': [round(c, 0) for c in cpu_per_core],
            'mem_percent': round(mem.percent, 1),
            'mem_used_gb': round(mem.used / 1024**3, 1),
            'mem_total_gb': round(mem.total / 1024**3, 1),
            'gpu_percent': gpu,
            'npu_percent': npu,
            'load_1m': round(load1, 2),
            'top_processes': top,
        }

    def _read_gpu(self):
        """读 Intel GPU freq(粗略估算占用)。"""
        try:
            # Intel Arc 在 /sys/class/drm/card*/gt_cur_freq_mhz
            for card in ['card0', 'card1']:
                cur_path = f'/sys/class/drm/{card}/device/gt_cur_freq_mhz'
                max_path = f'/sys/class/drm/{card}/device/gt_max_freq_mhz'
                if os.path.exists(cur_path):
                    with open(cur_path) as f:
                        cur = int(f.read().strip())
                    with open(max_path) as f:
                        mx = int(f.read().strip())
                    return round(cur / mx * 100) if mx > 0 else 0
        except Exception:
            pass
        return 0  # 无法读取

    def _estimate_npu(self):
        """NPU 估算：用 camera_detect_node(YOLO NPU)的 CPU 近似占用。"""
        try:
            for p in psutil.process_iter(['name', 'cpu_percent']):
                if 'camera_detect' in (p.info.get('name') or ''):
                    # YOLO 主要在 NPU,CPU 占用 30% 大概对应 NPU 60%+
                    cpu = p.info.get('cpu_percent') or 0
                    return round(min(100, cpu * 2))
        except Exception:
            pass
        return 0

    def _top_processes(self, n=3):
        """TOP N CPU 进程(只看 ROS 相关)。"""
        try:
            procs = []
            for p in psutil.process_iter(['name', 'cmdline', 'cpu_percent']):
                cmdline = ' '.join(p.info.get('cmdline') or [])
                # 只看 ROS / wheelchair / ladar 相关
                if any(kw in cmdline for kw in [
                    'ros2', 'controller_server', 'wheelchair', 'camera_detect',
                    'orbbec', 'voice', 'tts', 'sim_chassis', 'path_',
                    'safety', 'teb', 'networkx', 'rosbridge', 'rviz',
                ]):
                    cpu = p.info.get('cpu_percent') or 0
                    short_name = self._short_name(cmdline)
                    procs.append({'name': short_name, 'cpu': round(cpu, 1)})
            procs.sort(key=lambda x: x['cpu'], reverse=True)
            return procs[:n]
        except Exception:
            return []

    @staticmethod
    def _short_name(cmdline):
        """从 cmdline 提取短名。"""
        for kw in [
            'controller_server', 'camera_detect', 'orbbec', 'voice_node',
            'tts_node', 'sim_chassis', 'path_feeder', 'path_to_baselink',
            'safety_chain', 'teb_debug', 'networkx', 'rosbridge', 'rviz2',
            'wheelchair_app',
        ]:
            if kw in cmdline:
                return kw
        return cmdline[:30]


def main(args=None):
    rclpy.init(args=args)
    node = HwMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
