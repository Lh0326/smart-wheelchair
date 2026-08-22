"""测试 download_region.py 的核心函数（不实际下载）"""
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'scripts'))

from download_region import (
    load_region_config,
    compute_bbox,
    validate_bbox,
    tile_ranges,
    describe_bbox_error,
)


def test_load_region_config():
    config = load_region_config('config/region.yaml')
    assert 'region' in config
    assert 'south_west' in config['region']
    assert 'north_east' in config['region']


def test_compute_bbox():
    config = {
        'region': {
            'south_west': {'lat': 31.282, 'lon': 121.213},
            'north_east': {'lat': 31.290, 'lon': 121.223},
        }
    }
    bbox = compute_bbox(config)
    # osmnx bbox 格式：(north, south, east, west)
    assert bbox == (31.290, 31.282, 121.223, 121.213)


def test_validate_bbox_valid():
    bbox = (31.290, 31.282, 121.223, 121.213)
    assert validate_bbox(bbox) is True


def test_validate_bbox_invalid_inverted():
    # 南北颠倒
    bbox = (31.282, 31.290, 121.223, 121.213)
    assert validate_bbox(bbox) is False


def test_tile_ranges_z0_global():
    """z=0 时全球只有 1 张瓦片 (0,0)"""
    bbox = (85, -85, 180, -180)  # north, south, east, west
    x_min, x_max, y_north, y_south = tile_ranges(bbox, 0)
    assert (x_min, x_max) == (0, 0)
    assert (y_north, y_south) == (0, 0)


def test_tile_ranges_z1_quadrants():
    """z=1 时全球 4 张瓦片，2x2 网格"""
    bbox = (85, -85, 180, -180)
    x_min, x_max, y_north, y_south = tile_ranges(bbox, 1)
    assert x_min == 0 and x_max == 1
    assert y_north == 0 and y_south == 1


def test_describe_bbox_error_north_south_inverted():
    """南北颠倒的错误描述"""
    err = describe_bbox_error((31.282, 31.290, 121.223, 121.213))
    assert "north" in err
    assert "south" in err


def test_describe_bbox_error_dateline():
    """跨日界线的错误描述"""
    err = describe_bbox_error((40, 30, -179, 179))
    assert "跨日界线" in err or "east" in err
