"""EC20 _parse_gga NavSatFix.status 映射测试

验证 fix_quality 到 NavSatFix.status.status 的映射：
  fix_quality=0 (无定位) → status=-1 (STATUS_NO_FIX)，仍发布 /fix 让前端能区分"信号弱"
  fix_quality=1 (GPS 定位) → status=0 (STATUS_FIX)
  fix_quality=2 (DGPS) → status=0 (STATUS_FIX，ROS2 无 DGPS 常量)

背景：前端 GPS 三态显示需要区分"冷启动"和"信号弱"。
冷启动 = 5 秒收不到 /fix；信号弱 = 收到 /fix 但 status=-1。
所以节点必须在 fix_quality=0 时也发布 /fix（status=-1, lat/lon=0）。
"""
from unittest.mock import patch
import pytest

from rtk_gnss.ec20_gnss_node import EC20GnssNode
from sensor_msgs.msg import NavSatStatus


def _nmea_checksum(body: str) -> str:
    cs = 0
    for c in body:
        cs ^= ord(c)
    return f'{cs:02X}'


def _make_sentence(body: str) -> str:
    return f'${body}*{_nmea_checksum(body)}'


@pytest.fixture
def parser_node(rclpy_init):
    with patch('serial.Serial'):
        node = EC20GnssNode()
        node._running = False
    yield node
    node.destroy_node()


def _parse_gga_capture(node, quality, sats=5, hdop=1.7):
    """构造 GGA 句子并解析，返回发布的 NavSatFix 列表"""
    # GGA: time, lat, NS, lon, EW, quality, sats, hdop, alt, M, ...
    # 昆明 24.8551N, 102.8553E
    body = f'GPGGA,092750.000,2451.3060,N,10251.3180,E,{quality},{sats},{hdop:.2f},1950.0,M,55.2,M,,'
    sentence = _make_sentence(body)
    published = []
    with patch.object(node.fix_pub, 'publish', side_effect=lambda m: published.append(m)):
        node._parse_nmea(sentence)
    return published


# ========== status 映射测试 ==========

def test_parse_gga_no_fix_publishes_with_status_no_fix(parser_node):
    """fix_quality=0 时也发布 /fix，status=STATUS_NO_FIX(-1)，lat/lon=0

    这是前端区分"冷启动"和"信号弱"的关键：节点在跑但没锁定卫星时，
    仍然发布 /fix(status=-1)，前端据此显示"信号弱"而非"冷启动"。
    """
    published = _parse_gga_capture(parser_node, quality=0)

    assert len(published) == 1, "fix_quality=0 时必须发布 /fix（让前端能感知信号弱状态）"
    fix = published[0]
    assert fix.status.status == NavSatStatus.STATUS_NO_FIX, "fix_quality=0 → status=-1"
    assert fix.latitude == 0.0, "fix_quality=0 → lat=0（不暴露无效坐标）"
    assert fix.longitude == 0.0, "fix_quality=0 → lon=0"
    assert fix.status.service == NavSatStatus.SERVICE_GPS


def test_parse_gga_gps_fix_publishes_with_status_fix(parser_node):
    """fix_quality=1 时发布 /fix，status=STATUS_FIX(0)，lat/lon 为真实坐标"""
    published = _parse_gga_capture(parser_node, quality=1)

    assert len(published) == 1
    fix = published[0]
    assert fix.status.status == NavSatStatus.STATUS_FIX, "fix_quality=1 → status=0 (STATUS_FIX)"
    assert fix.latitude == pytest.approx(24.8551, abs=1e-4)
    assert fix.longitude == pytest.approx(102.8553, abs=1e-4)
    assert fix.altitude == pytest.approx(1950.0, abs=0.1)


def test_parse_gga_dgps_fix_publishes_with_status_fix(parser_node):
    """fix_quality=2 (DGPS) 时发布 /fix，status=STATUS_FIX(0)

    ROS2 NavSatStatus 没有 STATUS_DGPS_FIX 常量，DGPS 也映射到 STATUS_FIX。
    """
    published = _parse_gga_capture(parser_node, quality=2)

    assert len(published) == 1
    fix = published[0]
    assert fix.status.status == NavSatStatus.STATUS_FIX


def test_parse_gga_no_fix_still_carries_satellites_and_hdop(parser_node):
    """fix_quality=0 时 position_covariance 仍透传 satellites/hdop（诊断字段）

    前端在信号弱状态下可以显示"看到 N 颗卫星但未锁定"。
    """
    published = _parse_gga_capture(parser_node, quality=0, sats=17, hdop=99.9)

    assert len(published) == 1
    fix = published[0]
    assert fix.position_covariance[0] == 17.0, "satellites 透传"
    assert fix.position_covariance[4] == pytest.approx(99.9), "hdop 透传"
    assert fix.position_covariance_type == 0


def test_parse_gga_short_sentence_does_not_publish(parser_node):
    """GGA 字段不足 10 个时不发布（防御性）"""
    body = 'GPGGA,092750.000,2451.3060,N'  # 只有 4 个字段
    sentence = _make_sentence(body)
    published = []
    with patch.object(parser_node.fix_pub, 'publish', side_effect=lambda m: published.append(m)):
        parser_node._parse_nmea(sentence)
    assert len(published) == 0
