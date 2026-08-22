#!/usr/bin/env python3
"""自主导航摆动 + 掉电 实时监控脚本。

监控目标（验证 Nav2Protector current 锁定修复是否有效）：
  1. /chassis_protection_status —— Protector 状态机切换（NORMAL/RAMP/HARD_COOLDOWN）
  2. /heading_imu —— HWT906P IMU 当前航向（看是否跳变 > 30°）
  3. /cmd_vel + /cmd_vel_safe —— TEB 原始 vs safety_chain 输出
  4. USB disconnect —— journalctl 实时（ch341/usb 3-3 掉电事件）

输出分四块，彩色高亮：
  [PROTECTOR] 状态切换 + 触发器名（T1=LOOKAHEAD / T2=CMD_VEL_STEP / ...）
  [IMU]       heading 值 + 跳变告警（> 30°/帧 标红）
  [CMD_VEL]   linear.x / angular.z 原始 vs safe 对比
  [USB]       ch341 disconnect 实时事件

启动方式：
  source WS_ROOT_PLACEHOLDER/install/setup.bash
  python3 WS_ROOT_PLACEHOLDER/scripts/watch_autonav_debug.py

停止：Ctrl+C，最后打印累计统计。
"""
from __future__ import annotations

import math
import os
import signal
import subprocess
import sys
import threading
import time
from collections import Counter, deque
from datetime import datetime


# ============== 配置 ==============
IMU_JUMP_THRESHOLD_DEG = 30.0   # T1 触发阈值，跟 Protector 一致
CMD_VEL_JUMP_OMEGA = 0.5        # safety_chain 方向监控阈值
HISTORY_LEN = 60                # 累计事件保留窗口

# ============== 颜色 ==============
RED = '\033[91m'
GREEN = '\033[92m'
YELLOW = '\033[93m'
BLUE = '\033[94m'
MAGENTA = '\033[95m'
CYAN = '\033[96m'
BOLD = '\033[1m'
RESET = '\033[0m'

COLOR_PROTECTOR = MAGENTA
COLOR_IMU = CYAN
COLOR_CMD = BLUE
COLOR_USB = RED


def log(tag: str, color: str, msg: str) -> None:
    ts = datetime.now().strftime('%H:%M:%S.%f')[:-3]
    print(f'{color}[{ts}] [{tag}]{RESET} {msg}', flush=True)


# ============== 全局状态 ==============
class Stats:
    """累计统计。"""

    def __init__(self) -> None:
        self.protector_status_counts: Counter = Counter()
        self.protector_trigger_counts: Counter = Counter()  # RAMP/HARD_COOLDOWN_* 的触发器
        self.imu_jumps: int = 0
        self.imu_max_jump_deg: float = 0.0
        self.cmd_vel_jumps: int = 0
        self.usb_disconnects: int = 0
        self.start_time: float = time.time()
        self.last_status: str = ''
        self.last_heading: float | None = None
        self.recent_events: deque = deque(maxlen=HISTORY_LEN)


STATS = Stats()


def print_summary() -> None:
    elapsed = time.time() - STATS.start_time
    print(f'\n{BOLD}========== 累计统计（{elapsed:.1f}s）========={RESET}')
    print(f'{BOLD}Protector 状态分布：{RESET}')
    for status, count in STATS.protector_status_counts.most_common():
        marker = '⚠️' if 'COOLDOWN' in status else '✓'
        print(f'  {marker} {status:40s} {count:6d} 次')
    print(f'{BOLD}Protector 触发器命中（COOLDOWN 期间）：{RESET}')
    if STATS.protector_trigger_counts:
        for trigger, count in STATS.protector_trigger_counts.most_common():
            print(f'  🔥 {trigger:30s} {count:6d} 次')
    else:
        print(f'  {GREEN}（无触发）{RESET}')
    print(f'{BOLD}IMU 跳变（> {IMU_JUMP_THRESHOLD_DEG}°/帧）：{RESET}'
          f'{STATS.imu_jumps} 次，最大 {STATS.imu_max_jump_deg:.1f}°')
    print(f'{BOLD}cmd_vel_safe 方向跳变（> {CMD_VEL_JUMP_OMEGA} rad/s）：{RESET}'
          f'{STATS.cmd_vel_jumps} 次')
    print(f'{BOLD}USB 掉电事件：{RESET}{RED}{STATS.usb_disconnects} 次{RESET}')

    # 诊断结论
    print(f'\n{BOLD}========== 诊断结论 =========={RESET}')
    total_cooldown = sum(c for s, c in STATS.protector_status_counts.items()
                        if 'COOLDOWN' in s)
    if total_cooldown == 0:
        print(f'{GREEN}✓ Protector 未触发 COOLDOWN — 修复有效，底盘应平稳{RESET}')
    elif STATS.usb_disconnects == 0 and total_cooldown < 20:
        print(f'{YELLOW}⚠ Protector 偶尔触发（{total_cooldown} 次），'
              f'但未掉电 — 可接受{RESET}')
    else:
        print(f'{RED}✗ Protector 频繁触发（{total_cooldown} 次）'
              f'或 USB 掉电（{STATS.usb_disconnects} 次）— 仍有问题{RESET}')
        if STATS.imu_jumps > 5:
            print(f'{RED}  → IMU 频繁跳变（{STATS.imu_jumps} 次），'
                  f'根因可能在 HWT906P 硬件/磁力计干扰{RESET}')
        elif total_cooldown > 50:
            print(f'{RED}  → Protector 触发太频繁，'
                  f'可能 T1 阈值 30° 太敏感，需调高{RESET}')


# ============== USB journalctl 监控线程 ==============
def watch_usb_disconnects() -> None:
    """journalctl -k -f 实时过滤 ch341/usb disconnect 事件。"""
    cmd = ['journalctl', '-k', '-f', '-o', 'cat']
    try:
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                stderr=subprocess.DEVNULL, text=True)
        for line in proc.stdout:
            line = line.strip()
            if not line:
                continue
            if any(kw in line for kw in ('ch341', 'usb 3-3', 'USB disconnect',
                                          'device disconnected', 'over-current')):
                STATS.usb_disconnects += 1
                log('USB', COLOR_USB, f'{RED}{BOLD}掉电事件 #{STATS.usb_disconnects}: '
                    f'{line}{RESET}')
    except Exception as e:
        log('USB', COLOR_USB, f'journalctl 监控失败: {e}')


# ============== ROS2 订阅线程 ==============
def watch_ros_topics() -> None:
    """订阅 Protector / IMU / cmd_vel 三个 topic。"""
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String, Float64
    from geometry_msgs.msg import Twist

    rclpy.init()
    node = Node('watch_autonav_debug')

    def on_protection_status(msg: String) -> None:
        status = msg.data
        STATS.protector_status_counts[status] += 1
        # 提取触发器名（RAMP_COOLDOWN_T1_LOOKAHEAD → LOOKAHEAD）
        trigger = ''
        if 'COOLDOWN' in status:
            parts = status.split('_', 2)
            if len(parts) >= 3:
                trigger = parts[2]
                STATS.protector_trigger_counts[trigger] += 1
        # 状态切换时打印
        if status != STATS.last_status:
            color = GREEN if status == 'NORMAL' else (
                YELLOW if 'RAMP' in status else RED)
            arrow = '→'
            log('PROTECTOR', COLOR_PROTECTOR,
                f'{arrow} {color}{BOLD}{status}{RESET}'
                f'{" (T=" + trigger + ")" if trigger else ""}')
            STATS.last_status = status

    def on_heading(msg: Float64) -> None:
        heading = float(msg.data)
        if STATS.last_heading is not None:
            delta = abs(heading - STATS.last_heading) % 360.0
            delta = min(delta, 360.0 - delta)
            if delta > IMU_JUMP_THRESHOLD_DEG:
                STATS.imu_jumps += 1
                if delta > STATS.imu_max_jump_deg:
                    STATS.imu_max_jump_deg = delta
                log('IMU', COLOR_IMU,
                    f'{RED}{BOLD}跳变 {delta:.1f}°{RESET} '
                    f'({STATS.last_heading:.1f}° → {heading:.1f}°) '
                    f'#{STATS.imu_jumps}')
        STATS.last_heading = heading

    last_cmd_safe_omega = [0.0]

    def on_cmd_vel(msg: Twist) -> None:
        # TEB 原始输出（不经 safety_chain）
        if abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.05:
            log('CMD_VEL', COLOR_CMD,
                f'TEB    : linear.x={msg.linear.x:+.3f}  '
                f'angular.z={msg.angular.z:+.3f}')

    def on_cmd_vel_safe(msg: Twist) -> None:
        omega = msg.angular.z
        delta = abs(omega - last_cmd_safe_omega[0])
        if delta > CMD_VEL_JUMP_OMEGA and abs(msg.linear.x) > 0.01:
            STATS.cmd_vel_jumps += 1
            log('CMD_VEL', COLOR_CMD,
                f'{YELLOW}方向跳变 Δω={delta:.2f} rad/s，'
                f'safety_chain 应将 linear.x 归零 #{STATS.cmd_vel_jumps}{RESET}')
        last_cmd_safe_omega[0] = omega

    node.create_subscription(String, '/chassis_protection_status',
                             on_protection_status, 10)
    node.create_subscription(Float64, '/heading_imu', on_heading, 10)
    node.create_subscription(Twist, '/cmd_vel', on_cmd_vel, 10)
    node.create_subscription(Twist, '/cmd_vel_safe', on_cmd_vel_safe, 10)

    log('MAIN', GREEN, f'{BOLD}监控已启动{RESET}，等待数据...')
    log('MAIN', GREEN,
        f'  Protector 阈值：T1 跳变 > {IMU_JUMP_THRESHOLD_DEG}°')
    log('MAIN', GREEN,
        f'  safety_chain：方向跳变 > {CMD_VEL_JUMP_OMEGA} rad/s')
    log('MAIN', GREEN, '  Ctrl+C 停止并输出累计统计')
    print()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


# ============== 入口 ==============
def main() -> None:
    # 检查 ROS 环境
    if not os.environ.get('AMENT_PREFIX_PATH'):
        print(f'{RED}错误：未 source ROS2 环境{RESET}')
        print(f'请先运行：{YELLOW}source WS_ROOT_PLACEHOLDER/install/setup.bash{RESET}')
        sys.exit(1)

    # 启动 USB 监控线程
    usb_thread = threading.Thread(target=watch_usb_disconnects, daemon=True)
    usb_thread.start()

    # 信号处理
    def sigint_handler(sig, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, sigint_handler)

    # 主线程跑 ROS 订阅
    try:
        watch_ros_topics()
    finally:
        print_summary()


if __name__ == '__main__':
    main()
