#!/usr/bin/env python3
"""
EC20F GNSS 节点 - 发布 /fix 和 /heading_cog

工作流程：
  1. AT 端口（默认 /dev/ttyUSB_AT）发送 AT+QGPS=1 启动 GNSS
  2. NMEA 端口（默认 /dev/ttyUSB_NMEA）持续读取 GGA/RMC 报文
  3. 解析后发布 sensor_msgs/NavSatFix 到 /fix（来自 GGA）
  4. 发布 std_msgs/Float64 到 /heading_cog（来自 RMC course，仅移动时有效）

室内信号弱，需要把 EC20 dongle 放到窗户边或室外才能定位。
首次冷启动需要 30-60 秒搜索卫星。

参数：
  at_port    - AT 指令串口（默认 /dev/ttyUSB_AT）
  nmea_port  - NMEA 数据串口（默认 /dev/ttyUSB_NMEA）
  baudrate   - 串口波特率（默认 115200）
"""
import math
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64

import serial


# 修复 pyserial 在 EC20 上的 DTR/RTS ioctl 失败问题
def _patch_serial():
    _orig_setDTR = serial.Serial.setDTR
    _orig_setRTS = serial.Serial.setRTS

    def _safe_setDTR(self, level):
        try:
            _orig_setDTR(self, level)
        except (BrokenPipeError, OSError):
            pass

    def _safe_setRTS(self, level):
        try:
            _orig_setRTS(self, level)
        except (BrokenPipeError, OSError):
            pass

    serial.Serial.setDTR = _safe_setDTR
    serial.Serial.setRTS = _safe_setRTS


_patch_serial()


class EC20GnssNode(Node):
    def __init__(self):
        super().__init__('ec20_gnss')

        self.declare_parameter('at_port', '/dev/ttyUSB_AT')
        self.declare_parameter('nmea_port', '/dev/ttyUSB_NMEA')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('output_topic', '/fix')

        self.at_port = self.get_parameter('at_port').value
        self.nmea_port = self.get_parameter('nmea_port').value
        self.baudrate = int(self.get_parameter('baudrate').value)
        output_topic = self.get_parameter('output_topic').value

        self.fix_pub = self.create_publisher(NavSatFix, output_topic, 10)
        self.heading_pub = self.create_publisher(Float64, '/heading_cog', 10)

        self._satellites = 0
        self._tracked = 0
        self._gsv_tracked = 0
        self._has_fix = False
        self._last_fix_log = 0.0

        # 启动 GNSS
        self._start_gnss()

        # 启动 NMEA 读取线程
        self._running = True
        self._thread = threading.Thread(target=self._nmea_loop, daemon=True)
        self._thread.start()

        # 周期性状态日志
        self.create_timer(5.0, self._status_log)

    def _start_gnss(self):
        try:
            at = serial.Serial(self.at_port, self.baudrate, timeout=1)
            at.write(b'AT\r\n')
            time.sleep(0.5)
            resp = at.read(256).decode('ascii', errors='ignore')
            if 'OK' not in resp:
                self.get_logger().error(f'AT 端口 {self.at_port} 无响应')
                at.close()
                return

            at.write(b'AT+QGPS?\r\n')
            time.sleep(0.5)
            status = at.read(256).decode('ascii', errors='ignore')
            if '+QGPS: 1' in status:
                self.get_logger().info('✅ GNSS 已在运行')
            else:
                self.get_logger().info('启动 GNSS (AT+QGPS=1)...')
                at.write(b'AT+QGPS=1\r\n')
                time.sleep(1.5)
                resp = at.read(256).decode('ascii', errors='ignore')
                if 'OK' in resp:
                    self.get_logger().info('✅ GNSS 启动成功')
                else:
                    self.get_logger().error(f'❌ GNSS 启动失败: {resp}')
            at.close()
        except Exception as e:
            self.get_logger().error(f'启动 GNSS 异常: {e}')

    def _nmea_loop(self):
        try:
            nmea = serial.Serial(self.nmea_port, self.baudrate, timeout=1)
        except Exception as e:
            self.get_logger().error(f'无法打开 NMEA 端口 {self.nmea_port}: {e}')
            return

        self.get_logger().info(f'开始读取 NMEA: {self.nmea_port}')
        buf = ''
        while self._running and rclpy.ok():
            chunk = nmea.read(512).decode('ascii', errors='ignore')
            if chunk:
                buf += chunk
                while '\n' in buf:
                    line, buf = buf.split('\n', 1)
                    line = line.strip()
                    if line.startswith('$'):
                        self._parse_nmea(line)

        nmea.close()

    def _parse_nmea(self, sentence):
        if '*' not in sentence:
            return
        body, cksum = sentence.rsplit('*', 1)
        body = body.lstrip('$')
        calc = 0
        for c in body:
            calc ^= ord(c)
        if f'{calc:02X}' != cksum.upper():
            return

        fields = sentence.split(',')
        talker_type = fields[0][3:]

        if talker_type == 'GGA':
            self._parse_gga(fields)
        elif talker_type == 'RMC':
            self._parse_rmc(fields)
        elif talker_type == 'GSV':
            self._parse_gsv(fields)

    def _parse_coord(self, val, direction):
        if not val or not direction:
            return 0.0
        if direction in ('N', 'S'):
            deg = float(val[:2])
            minutes = float(val[2:])
        else:
            deg = float(val[:3])
            minutes = float(val[3:])
        result = deg + minutes / 60.0
        if direction in ('S', 'W'):
            result = -result
        return result

    def _parse_gga(self, f):
        if len(f) < 10:
            return
        try:
            fix_quality = int(f[6]) if f[6] else 0
            # fix_quality=0 时也发布 /fix（status=NO_FIX, lat/lon=0），
            # 让前端能区分"冷启动"（5 秒收不到 /fix）和"信号弱"（收到 /fix 但 status=-1）。
            if fix_quality == 0:
                lat = 0.0
                lon = 0.0
                altitude = 0.0
            else:
                lat = self._parse_coord(f[2], f[3])
                lon = self._parse_coord(f[4], f[5])
                altitude = float(f[9]) if f[9] else 0.0
            satellites = int(f[7]) if f[7] else 0
            hdop = float(f[8]) if f[8] else 0.0

            # _satellites (inview) maintained by GSV only; do NOT overwrite with GGA f[7]
            # (used-in-fix count), which is empty=0 when no fix and would hide real inview.
            if fix_quality > 0:
                self._has_fix = True

            fix = NavSatFix()
            fix.header.stamp = self.get_clock().now().to_msg()
            fix.header.frame_id = 'wgs84'
            fix.latitude = lat
            fix.longitude = lon
            fix.altitude = altitude
            # ROS2 NavSatStatus: STATUS_NO_FIX=-1, STATUS_FIX=0
            # fix_quality: 0=无, 1=GPS, 2=DGPS → 都映射到 STATUS_FIX（ROS2 无 DGPS 常量）
            fix.status.status = -1 if fix_quality == 0 else 0
            fix.status.service = 1
            # 通过 position_covariance 透传 satellites / hdop（dgps_node 专用诊断字段），
            # position_covariance_type=0 (UNKNOWN) 表示不当作真实协方差消费
            fix.position_covariance[0] = float(satellites)
            fix.position_covariance[4] = float(hdop)
            fix.position_covariance_type = 0
            self.fix_pub.publish(fix)

            # 首次定位或每 30 秒记录一次
            now = time.time()
            if fix_quality > 0 and now - self._last_fix_log > 30.0:
                self.get_logger().info(
                    f'定位: {lat:.6f}, {lon:.6f}, 海拔={altitude:.1f}m, '
                    f'卫星={satellites}, HDOP={hdop:.1f}'
                )
                self._last_fix_log = now
        except (ValueError, IndexError):
            pass

    def _parse_rmc(self, f):
        if len(f) < 10:
            return
        try:
            fix_mode = f[2] if len(f) > 2 else 'V'
            if fix_mode != 'A':
                return
            course = float(f[8]) if f[8] else 0.0
            heading = Float64()
            heading.data = course
            self.heading_pub.publish(heading)
        except (ValueError, IndexError):
            pass

    def _parse_gsv(self, f):
        # GSV 第 4 字段是"可见卫星总数"。GGA 在 fix_quality=0 时直接 return，
        # 不会更新 _satellites，所以从 GSV 这里更新，让日志能反映 NMEA 实际流入状态。
        if len(f) < 4:
            return
        try:
            sentence_no = int(f[2]) if f[2] else 0
            if sentence_no == 1:  # first sentence of a GSV cycle: refresh inview + reset tracked
                total = int(f[3]) if f[3] else 0
                if total > 0:
                    self._satellites = total
                self._gsv_tracked = 0
            # count realTracked: per-sat [PRN,el,az,snr] every 4 fields; snr non-empty and != 34
            i = 4
            while i + 3 < len(f):
                el = f[i + 1].strip()
                snr = f[i + 3].split("*")[0].strip()  # last sat snr carries *checksum
                if el and el != "00" and snr and snr != "34":
                    self._gsv_tracked += 1
                i += 4
            self._tracked = self._gsv_tracked
        except (ValueError, IndexError):
            pass

    def _status_log(self):
        if not self._has_fix:
            if self._tracked == 0:
                tip = 'RF链路不通: 检查SMA接头/U.FL跳线, 天线挪到窗外开阔处'
            else:
                tip = '链路通, 收敛中, 再等1~2分钟'
            self.get_logger().warn(
                f'尚未定位（可见={self._satellites}, 跟踪={self._tracked}）→ {tip}'
            )


def main():
    rclpy.init()
    node = EC20GnssNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._running = False
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
