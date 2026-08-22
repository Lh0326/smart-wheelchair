#!/usr/bin/env python3
"""N10P 雷达 Python ROS2 驱动（绕过 C++ lslidar_driver v5.1.1 的 CRC 失败 bug）。

直接读 /dev/lidar_n10p 串口（用 termios + os.read，避开 pyserial 在 USB 抖动时的脆弱异常），按 LSLIDAR N10Plus 协议解析，发布 /scan。

为什么不用官方 C++ 驱动：
- lslidar_driver v5.1.1 在本机上持续 CRC 失败（calculated vs received 不匹配）
- 直接 Python 读串口验证：5 个包中 4 个 CRC 通过
- C++ 驱动 buffer 处理有 bug（strace 时正常，不 strace 时持续失败）

为什么不用 pyserial：
- pyserial 在 USB CDC-ACM 抖动时（select 报可读但 read 返回 0）抛 SerialException
- 雷达 USB 每 5-30 秒就会抖一次，pyserial 反复断开重连，丢包严重
- 直接 os.read 处理 EAGAIN 更稳定

N10Plus 协议（108 字节/包）：
  bytes[0:2]   = magic 0xA5 0x5A
  bytes[2]     = Length
  bytes[3:5]   = Speed (big-endian)
  bytes[5:7]   = start_angle (big-endian, 0.01° 单位, mod 36000)
  bytes[7:103] = 16 group * (2 echo * 3 byte) = 96 byte 距离/强度
  bytes[105:107] = end_angle (big-endian, 0.01° 单位)
  bytes[107]   = additive checksum (sum bytes[0:107] mod 256)
"""
import errno
import fcntl
import os
import select
import sys
import termios
import threading
import time
from math import pi

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

PORT = "/dev/lidar_n10p"
BAUD = 460800
PACKET_LEN = 108
POINTS_PER_PACKET = 16  # 16 angle groups
DISTANCE_RES_M = 0.001  # 1 mm
FRAME_HZ = 10
POINTS_PER_FRAME = 540


def bin_angle_deg(ang_deg: float, flip_horizontal: bool, num_bins: int) -> int:
    """把硬件报的原始角度（0.01° 单位累积后的浮点 deg）映射到 ROS LaserScan bin 索引。

    背景：N10P 雷达硬件扫描方向是 CW（顺时针），即 ang_deg=90 是物理 -Y（右）。
    ROS LaserScan 约定 CCW 且发布为 [-π, π]：bin 0 = angle=-π（后），bin n/2 = angle=0（前）。

    flip_horizontal=true：完整修复 CW→CCW 翻转，前后左右全部正确（推荐默认）。
    flip_horizontal=false：保留原始 bin 公式 int(ang_deg/360*n)，配合新 [-π, π]
        发布约定下左右正确但前后反（用于现场对比、回滚诊断）。

    输出 bin 约定（ROS 标准 [-π, π]）：
      bin 0     → angle=-π      = -X（后）
      bin n/4   → angle=-π/2    = -Y（右）
      bin n/2   → angle=0       = +X（前）
      bin 3n/4  → angle=π/2     = +Y（左）
    """
    if flip_horizontal:
        ros_ang_deg = (360.0 - ang_deg) % 360.0
        # bin 0 = angle=-π，所以加 0.5 的偏移
        return int((ros_ang_deg / 360.0 + 0.5) * num_bins) % num_bins
    else:
        return int(ang_deg / 360.0 * num_bins) % num_bins


def open_serial(port: str, baud: int) -> int:
    """打开串口，返回 fd。用 termios 配置 raw 8N1 阻塞模式（VMIN=1 VTIME=0）。"""
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    # 取消 O_NONBLOCK
    flags = fcntl.fcntl(fd, fcntl.F_GETFL)
    fcntl.fcntl(fd, fcntl.F_SETFL, flags & ~os.O_NONBLOCK)

    attrs = termios.tcgetattr(fd)
    # raw mode
    attrs[0] = 0  # iflag
    attrs[1] = 0  # oflag
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL  # cflag
    attrs[3] = 0  # lflag (raw)
    # cc: VMIN=1, VTIME=2 (200ms timeout per read)
    attrs[6][termios.VMIN] = 1
    attrs[6][termios.VTIME] = 2

    # set speed (Python termios 没暴露 cfsetispeed/cfsetospeed，直接写 attrs[4]/[5])
    speed_const = {
        230400: termios.B230400,
        460800: termios.B460800,
        500000: termios.B500000,
        921600: termios.B921600,
    }[baud]
    attrs[4] = speed_const  # ispeed
    attrs[5] = speed_const  # ospeed
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


def robust_read(fd: int, n: int, timeout_s: float = 0.5) -> bytes:
    """读 n 字节，USB 抖动时重试，不抛异常。"""
    buf = bytearray()
    deadline = time.monotonic() + timeout_s
    while len(buf) < n:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        # poll 等 POLLIN
        r, _, _ = select.select([fd], [], [], min(remaining, 0.1))
        if not r:
            continue
        try:
            chunk = os.read(fd, n - len(buf))
        except OSError as e:
            if e.errno in (errno.EAGAIN, errno.EINTR):
                continue
            # 其他错误：短暂休眠后继续
            time.sleep(0.01)
            continue
        if not chunk:
            # USB CDC-ACM 偶尔 select 报可读但 read 返回空，忽略
            continue
        buf.extend(chunk)
    return bytes(buf)


class N10PDriver(Node):
    def __init__(self):
        super().__init__("n10p_python_driver")
        self.declare_parameter("frame_id", "laser")
        self.declare_parameter("output_topic", "/scan")
        self.declare_parameter("port", PORT)
        self.declare_parameter("baud", BAUD)
        self.declare_parameter("range_min", 0.15)
        self.declare_parameter("range_max", 25.0)
        self.declare_parameter("flip_horizontal", True)

        self.frame_id = self.get_parameter("frame_id").value
        self.output_topic = str(self.get_parameter("output_topic").value)
        self._port = str(self.get_parameter("port").value)
        self._baud = int(self.get_parameter("baud").value)
        self.range_min = float(self.get_parameter("range_min").value)
        self.range_max = float(self.get_parameter("range_max").value)
        self._flip_horizontal = bool(self.get_parameter("flip_horizontal").value)

        self.scan_pub = self.create_publisher(LaserScan, self.output_topic, 10)

        self.frame_points = []
        self.last_start_angle = None

        self._fd = None
        self._running = True
        self._last_open_error_log = 0.0
        self._pkts_ok = 0
        self._pkts_bad = 0
        self._frames = 0
        self._nonzero_pts = 0  # 有效距离的点数（诊断用）
        self.create_timer(5.0, self._log_stats)

        self._t = threading.Thread(target=self._read_loop, daemon=True)
        self._t.start()

    def _open_serial_if_needed(self) -> bool:
        if self._fd is not None:
            return True

        try:
            self._fd = open_serial(self._port, self._baud)
        except (FileNotFoundError, PermissionError, OSError) as e:
            now = time.monotonic()
            if now - self._last_open_error_log >= 2.0:
                self.get_logger().warn(f"等待串口 {self._port}: {e}")
                self._last_open_error_log = now
            time.sleep(0.5)
            return False

        self.get_logger().info(
            f"Opened {self._port} @ {self._baud} (fd={self._fd}), "
            f"frame_id={self.frame_id}, output={self.output_topic}, "
            f"flip_horizontal={self._flip_horizontal}"
        )
        # 发送启动电机命令（N10Plus 协议：A5 5A 55 + ... + 0x01 0x01 + ... + FA FB，188 字节）
        self._send_motor_command(1)
        return True

    def _close_serial(self):
        if self._fd is None:
            return
        try:
            os.close(self._fd)
        except Exception:
            pass
        self._fd = None

    def _read_loop(self):
        while self._running and rclpy.ok():
            if not self._open_serial_if_needed():
                continue
            try:
                self._read_one_packet()
            except Exception as e:
                self.get_logger().warn(f"读取异常: {e}")
                self._close_serial()
                time.sleep(0.5)

    def _send_motor_command(self, motor_command: int):
        """发送电机控制命令。motor_command: 1=旋转/启动, 0=停止。"""
        if motor_command not in (0, 1):
            return
        data = bytearray(188)
        data[0] = 0xA5
        data[1] = 0x5A
        data[2] = 0x55
        data[186] = 0xFA
        data[187] = 0xFB
        if motor_command == 1:
            data[184] = 0x01
            data[185] = 0x01
        else:
            data[184] = 0x03
            data[185] = 0x00
        try:
            os.write(self._fd, bytes(data))
            self.get_logger().info(f"已发送 motor_control={motor_command} 命令 (188 字节)")
        except OSError as e:
            self.get_logger().warn(f"发送 motor_control 命令失败: {e}")

    def _read_one_packet(self):
        # Sync 0xA5 0x5A
        while self._running and rclpy.ok():
            b = robust_read(self._fd, 1, timeout_s=0.5)
            if not b:
                continue
            if b[0] == 0xA5:
                b2 = robust_read(self._fd, 1, timeout_s=0.2)
                if b2 and b2[0] == 0x5A:
                    break
                # 如果不是 0x5A 但可能是 0xA5（连续 magic），回到外层

        body = robust_read(self._fd, PACKET_LEN - 2, timeout_s=0.5)
        if len(body) != PACKET_LEN - 2:
            return

        pkt = bytearray([0xA5, 0x5A]) + bytearray(body)
        cs = sum(pkt[:-1]) & 0xFF
        if cs != pkt[-1]:
            self._pkts_bad += 1
            return
        self._pkts_ok += 1
        self._parse_packet(pkt)

    def _parse_packet(self, pkt):
        start_angle = ((pkt[5] << 8) | pkt[6]) % 36000
        end_angle = ((pkt[105] << 8) | pkt[106]) % 36000

        if end_angle <= start_angle:
            angle_span = end_angle + 36000 - start_angle
        else:
            angle_span = end_angle - start_angle
        angle_inc = angle_span / (POINTS_PER_PACKET - 1) if POINTS_PER_PACKET > 1 else 0

        # 跨越 0° → 一帧结束
        if self.last_start_angle is not None:
            if self.last_start_angle > 27000 and start_angle < 9000:
                self._publish_frame()

        for i in range(POINTS_PER_PACKET):
            ang_raw = (start_angle + int(angle_inc * i)) % 36000
            ang_deg = ang_raw / 100.0
            off = 7 + i * 6
            # 取 echo 0（echo 1 是同点二次回波，盲区近距离才用）
            dist_mm = (pkt[off] << 8) | pkt[off + 1]
            intensity = pkt[off + 2]
            if dist_mm == 0 or dist_mm == 0xFFFF:
                range_m = float("inf")
            else:
                range_m = dist_mm * DISTANCE_RES_M
                if range_m < self.range_min or range_m > self.range_max:
                    range_m = float("inf")
                else:
                    self._nonzero_pts += 1
            self.frame_points.append((ang_deg, range_m, float(intensity)))

        self.last_start_angle = start_angle

    def _publish_frame(self):
        if not self.frame_points:
            return

        n = POINTS_PER_FRAME
        ranges = [float("inf")] * n
        intensities = [0.0] * n
        for ang_deg, r, inten in self.frame_points:
            idx = bin_angle_deg(ang_deg, self._flip_horizontal, n)
            if r < ranges[idx]:
                ranges[idx] = r
                intensities[idx] = inten

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        # ROS 标准 [-π, π] 约定，与 LD14P / Gemini 一致
        msg.angle_min = -pi
        msg.angle_max = pi - 2.0 * pi / n
        msg.angle_increment = 2.0 * pi / n
        msg.time_increment = 0.0
        msg.scan_time = 1.0 / FRAME_HZ
        msg.range_min = self.range_min
        msg.range_max = self.range_max
        msg.ranges = ranges
        msg.intensities = intensities
        self.scan_pub.publish(msg)

        self._frames += 1
        self.frame_points = []

    def _log_stats(self):
        total = self._pkts_ok + self._pkts_bad
        if total > 0:
            ok_rate = self._pkts_ok / total * 100
            self.get_logger().info(
                f"stats: ok={self._pkts_ok} bad={self._pkts_bad} "
                f"({ok_rate:.1f}% ok) frames={self._frames} valid_pts={self._nonzero_pts}"
            )

    def destroy_node(self):
        self._running = False
        if self._t.is_alive():
            self._t.join(timeout=1.0)
        self._close_serial()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = N10PDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
