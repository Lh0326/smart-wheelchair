"""测试 mbtiles_server 的核心函数"""
import os
import sqlite3
import tempfile

import pytest

from rtk_map.mbtiles_server import MbtilesReader


@pytest.fixture
def sample_mbtiles(tmp_path):
    """生成一个最小可用的 mbtiles 测试文件"""
    path = tmp_path / "test.mbtiles"
    conn = sqlite3.connect(path)
    c = conn.cursor()
    c.execute("CREATE TABLE metadata (name text, value text)")
    c.execute("INSERT INTO metadata VALUES ('format', 'png')")
    c.execute("INSERT INTO metadata VALUES ('bounds', '121.213,31.282,121.223,31.290')")
    c.execute("""CREATE TABLE tiles (
        zoom_level integer, tile_column integer, tile_row integer, tile_data blob,
        PRIMARY KEY (zoom_level, tile_column, tile_row))""")
    # 插入一张 1x1 透明 PNG
    png_1x1 = b'\x89PNG\r\n\x1a\n\x00\x00\x00\rIHDR\x00\x00\x00\x01\x00\x00\x00\x01\x08\x06\x00\x00\x00\x1f\x15\xc4\x89\x00\x00\x00\rIDATx\x9cc\xf8\x0f\x00\x00\x01\x01\x00\x05\xfe\x02\xfe\xa1Yz\xc6\x00\x00\x00\x00IEND\xaeB`\x82'
    # z=15, x=17000, y=8200 (TMS)
    c.execute("INSERT INTO tiles VALUES (15, 17000, 8200, ?)", (png_1x1,))
    conn.commit()
    conn.close()
    return str(path)


def test_open_existing(sample_mbtiles):
    reader = MbtilesReader(sample_mbtiles)
    assert reader.path == sample_mbtiles


def test_get_metadata(sample_mbtiles):
    reader = MbtilesReader(sample_mbtiles)
    assert reader.get_metadata("format") == "png"
    bounds = reader.get_metadata("bounds")
    assert bounds is not None
    assert "121.213" in bounds


def test_get_tile_found(sample_mbtiles):
    reader = MbtilesReader(sample_mbtiles)
    tile_data = reader.get_tile(z=15, x=17000, y=8200)
    assert tile_data is not None
    assert tile_data.startswith(b'\x89PNG')


def test_get_tile_not_found(sample_mbtiles):
    reader = MbtilesReader(sample_mbtiles)
    tile_data = reader.get_tile(z=15, x=99999, y=99999)
    assert tile_data is None


def test_tms_to_xyz_y():
    """mbtiles 存的是 TMS y，前端请求是 XYZ y，需要转换

    标准公式：xyz_y = (2^z - 1) - tms_y
    z=15 时 2^15 - 1 = 32767，32767 - 8200 = 24567
    """
    from rtk_map.mbtiles_server import tms_to_xyz_y
    assert tms_to_xyz_y(tms_y=8200, z=15) == 24567
