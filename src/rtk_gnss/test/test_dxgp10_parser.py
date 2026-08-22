"""DX-GP10-A NMEA 解析单元测试

测试继承自 EC20GnssNode 的 _parse_coord / _parse_gga / _parse_rmc / _parse_nmea。
DX-GP10-A 用 $GN talker（多系统 GPS+BDS+GLONASS），数据格式与 EC20 一致（NMEA 0183 标准）。
"""
from unittest.mock import patch

import pytest

from rtk_gnss.dxgp10_gnss_node import Dxgp10GnssNode


def _nmea_checksum(body: str) -> str:
    """计算 NMEA 句子 body 部分的 XOR checksum（$ 和 * 之间）"""
    cs = 0
    for c in body:
        cs ^= ord(c)
    return f'{cs:02X}'


def _make_sentence(body: str) -> str:
    """构造完整 NMEA 句子 $body*CS"""
    return f'${body}*{_nmea_checksum(body)}'


@pytest.fixture
def parser_node(rclpy_init):
    """构造测试节点，mock 串口避免真实硬件访问"""
    with patch('serial.Serial'):
        node = Dxgp10GnssNode()
        node._running = False  # 阻止后台线程
    yield node
    node.destroy_node()


# ========== _parse_coord 测试 ==========

def test_parse_coord_north(parser_node):
    """北纬正确解析：度分 → 十进制度"""
    # 24°51.306' N → 24.8551°
    assert parser_node._parse_coord('2451.3060', 'N') == pytest.approx(24.8551, abs=1e-6)


def test_parse_coord_south(parser_node):
    """南纬返回负值"""
    assert parser_node._parse_coord('3456.7890', 'S') == pytest.approx(-(34 + 56.7890 / 60), abs=1e-6)


def test_parse_coord_east(parser_node):
    """东经：3 位度数"""
    # 102°51.318' E → 102.8553°
    assert parser_node._parse_coord('10251.3180', 'E') == pytest.approx(102.8553, abs=1e-6)


def test_parse_coord_west(parser_node):
    """西经返回负值"""
    assert parser_node._parse_coord('12345.6789', 'W') == pytest.approx(-(123 + 45.6789 / 60), abs=1e-6)


def test_parse_coord_empty_returns_zero(parser_node):
    """空值或方向缺失返回 0"""
    assert parser_node._parse_coord('', 'N') == 0.0
    assert parser_node._parse_coord('1234.5', '') == 0.0


# ========== _parse_gga 测试 ==========

def test_parse_gga_valid_publishes_fix(parser_node, capsys):
    """完整 GGA 句子解析后发布 NavSatFix 到 /gps/rover_raw"""
    # 构造 $GNGGA 句子：昆明 24.8551N, 102.8553E
    # GGA 字段: time, lat, NS, lon, EW, quality, sats, hdop, alt, M, ...
    body = 'GNGGA,092750.000,2451.3060,N,10251.3180,E,1,8,1.03,1950.0,M,55.2,M,,'
    sentence = _make_sentence(body)

    published = []
    with patch.object(parser_node.fix_pub, 'publish', side_effect=lambda m: published.append(m)):
        parser_node._parse_nmea(sentence)

    assert len(published) == 1
    fix = published[0]
    assert fix.latitude == pytest.approx(24.8551, abs=1e-6)
    assert fix.longitude == pytest.approx(102.8553, abs=1e-6)
    assert fix.altitude == pytest.approx(1950.0, abs=0.1)
    assert fix.header.frame_id == 'wgs84'


def test_parse_gga_no_fix_skipped(parser_node):
    """quality=0（未定位）的 GGA 不发布"""
    body = 'GNGGA,092750.000,,,N,,,E,0,0,,,M,,M,,'
    sentence = _make_sentence(body)

    published = []
    with patch.object(parser_node.fix_pub, 'publish', side_effect=lambda m: published.append(m)):
        parser_node._parse_nmea(sentence)

    assert len(published) == 0
    assert parser_node._satellites == 0
    assert parser_node._has_fix is False


def test_parse_gga_updates_satellite_count(parser_node):
    """GGA 解析后 _satellites 字段更新"""
    body = 'GNGGA,092750.000,2451.3060,N,10251.3180,E,1,12,1.03,1950.0,M,55.2,M,,'
    sentence = _make_sentence(body)

    with patch.object(parser_node.fix_pub, 'publish'):
        parser_node._parse_nmea(sentence)

    assert parser_node._satellites == 12
    assert parser_node._has_fix is True


# ========== _parse_rmc 测试 ==========

def test_parse_rmc_valid_publishes_heading(parser_node):
    """RMC 句子解析后发布 course 到 /heading_cog"""
    # RMC 字段: time, status(A/V), lat, NS, lon, EW, speed, course, date, mag_var, mag_var_dir
    body = 'GNRMC,092750.000,A,2451.3060,N,10251.3180,E,0.5,123.4,010123,,,A'
    sentence = _make_sentence(body)

    published = []
    with patch.object(parser_node.heading_pub, 'publish', side_effect=lambda m: published.append(m)):
        parser_node._parse_nmea(sentence)

    assert len(published) == 1
    assert published[0].data == pytest.approx(123.4)


def test_parse_rmc_invalid_skipped(parser_node):
    """status=V（无效）的 RMC 不发布"""
    body = 'GNRMC,092750.000,V,2451.3060,N,10251.3180,E,0.5,123.4,010123,,,A'
    sentence = _make_sentence(body)

    published = []
    with patch.object(parser_node.heading_pub, 'publish', side_effect=lambda m: published.append(m)):
        parser_node._parse_nmea(sentence)

    assert len(published) == 0


# ========== _parse_nmea 校验和测试 ==========

def test_parse_nmea_bad_checksum_rejected(parser_node):
    """checksum 错误的句子被拒绝"""
    body = 'GNGGA,092750.000,2451.3060,N,10251.3180,E,1,8,1.03,1950.0,M,55.2,M,,'
    bad_sentence = f'${body}*00'  # 故意错的 checksum

    published = []
    with patch.object(parser_node.fix_pub, 'publish', side_effect=lambda m: published.append(m)):
        parser_node._parse_nmea(bad_sentence)

    assert len(published) == 0


def test_parse_nmea_no_checksum_rejected(parser_node):
    """无 * 分隔符的句子被拒绝"""
    published = []
    with patch.object(parser_node.fix_pub, 'publish', side_effect=lambda m: published.append(m)):
        parser_node._parse_nmea('$GNGGA,1234567')  # 无 *

    assert len(published) == 0


def test_parse_nmea_unknown_talker_ignored(parser_node):
    """未识别的句子类型（如 GLL）不引发异常"""
    body = 'GNGLL,2451.3060,N,10251.3180,E,092750.000,A,A'
    sentence = _make_sentence(body)

    # 不应该崩溃，也不应该发布 fix（GLL 不被处理）
    published = []
    with patch.object(parser_node.fix_pub, 'publish', side_effect=lambda m: published.append(m)):
        parser_node._parse_nmea(sentence)

    assert len(published) == 0


# ========== 集成：完整流程 ==========

def test_full_gn_talker_flow(parser_node):
    """完整模拟一段 $GN 多系统 NMEA 流程"""
    sentences = [
        _make_sentence('GNGGA,092750.000,2451.3060,N,10251.3180,E,1,10,1.5,1950.0,M,55.2,M,,'),
        _make_sentence('GNRMC,092750.000,A,2451.3060,N,10251.3180,E,0.5,45.0,010123,,,A'),
    ]

    fixes = []
    headings = []
    with patch.object(parser_node.fix_pub, 'publish', side_effect=lambda m: fixes.append(m)), \
         patch.object(parser_node.heading_pub, 'publish', side_effect=lambda m: headings.append(m)):
        for s in sentences:
            parser_node._parse_nmea(s)

    assert len(fixes) == 1
    assert len(headings) == 1
    assert fixes[0].latitude == pytest.approx(24.8551, abs=1e-6)
    assert headings[0].data == pytest.approx(45.0)
    assert parser_node._satellites == 10
