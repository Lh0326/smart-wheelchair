#!/usr/bin/env python3
"""
下载测试区域的 OSM 矢量数据和瓦片图。

用法：
    python scripts/download_region.py --config config/region.yaml
"""
import argparse
import math
import os
import sqlite3
import sys
import time
from pathlib import Path

import requests
import yaml

try:
    import osmnx as ox
except ImportError:
    print("ERROR: pip install osmnx", file=sys.stderr)
    sys.exit(1)


TILE_SERVERS = {
    "osm": "https://tile.openstreetmap.org/{z}/{x}/{y}.png",
    "carto_light": "https://a.basemaps.cartocdn.com/light_all/{z}/{x}/{y}.png",
    # 高德卫星图（中国可用，GCJ-02 坐标系）
    "gaode_sat": "https://webst02.is.autonavi.com/appmaptile?style=6&x={x}&y={y}&z={z}",
    # 高德道路图
    "gaode_road": "https://webrd02.is.autonavi.com/appmaptile?&lang=zh_cn&size=1&scale=1&style=8&x={x}&y={y}&z={z}",
}


def load_region_config(config_path: str) -> dict:
    with open(config_path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def compute_bbox(config: dict) -> tuple:
    """返回 osmnx 格式 (north, south, east, west)"""
    sw = config["region"]["south_west"]
    ne = config["region"]["north_east"]
    return (ne["lat"], sw["lat"], ne["lon"], sw["lon"])


def validate_bbox(bbox: tuple) -> bool:
    north, south, east, west = bbox
    if north <= south:
        return False
    if east <= west:
        return False
    if not (-90 <= south < north <= 90):
        return False
    if not (-180 <= west < east <= 180):
        return False
    return True


def describe_bbox_error(bbox: tuple) -> str:
    """生成可读的 bbox 错误描述（用于 validate_bbox 失败时打印原因）"""
    north, south, east, west = bbox
    errors = []
    if north <= south:
        errors.append(f"north({north}) <= south({south})")
    if east <= west:
        errors.append(f"east({east}) <= west({west})，可能跨日界线（本脚本不支持）")
    if not (-90 <= south < north <= 90):
        errors.append(f"纬度范围非法：south={south}, north={north}（应在 -90 到 90 之间，且 south < north）")
    if not (-180 <= west < east <= 180):
        errors.append(f"经度范围非法：west={west}, east={east}（应在 -180 到 180 之间，且 west < east）")
    return "；".join(errors) if errors else "未知错误"


def download_osm_graph(config: dict, output_geojson: str):
    """用 osmnx 下载区域路网，保存为 GeoJSON（供前端 Leaflet 显示）"""
    bbox = compute_bbox(config)
    if not validate_bbox(bbox):
        raise ValueError(f"无效的 bbox: {bbox}")

    # osmnx 2.0 的 graph_from_bbox 接受 (left, bottom, right, top) = (west, south, east, north)
    # 我们的 bbox 是 (north, south, east, west)，需要重新排序
    north, south, east, west = bbox
    osmnx_bbox = (west, south, east, north)

    print(f"[OSM] 下载路网 bbox={bbox} (osmnx 格式: {osmnx_bbox})")
    graph = ox.graph_from_bbox(osmnx_bbox, network_type="walk")
    nodes, edges = ox.graph_to_gdfs(graph)
    edges.to_file(output_geojson, driver="GeoJSON")
    print(f"[OSM] 节点数={len(nodes)}, 边数={len(edges)}，已保存到 {output_geojson}")
    return output_geojson


def _lon_to_x(lon: float, z: int) -> int:
    # OSM/XYZ 瓦片号在 [0, 2^z - 1] 范围内。经度恰为 180° 时公式结果等于 2^z，
    # 越界——夹取到最大有效瓦片号（与 OSM wiki 推荐做法一致）。
    return min(int((lon + 180) / 360 * (2 ** z)), (2 ** z) - 1)


def _lat_to_y(lat: float, z: int) -> int:
    # 同样把南界（lat = -85.0511°附近）夹取到 [0, 2^z - 1] 防止越界
    lat_rad = math.radians(lat)
    return min(int((1 - math.log(math.tan(lat_rad) + 1 / math.cos(lat_rad)) / math.pi) / 2 * (2 ** z)), (2 ** z) - 1)


def tile_ranges(bbox: tuple, zoom: int):
    """计算给定 bbox 和 zoom 下的瓦片行列号范围。

    返回 (x_min, x_max, y_north, y_south)：
    - x: 经度方向，西边小、东边大
    - y: 纬度方向，XYZ 坐标系下 north 对应的 y 较小，south 对应的 y 较大
    """
    north, south, east, west = bbox
    x_min = _lon_to_x(west, zoom)
    x_max = _lon_to_x(east, zoom)
    y_north = _lat_to_y(north, zoom)
    y_south = _lat_to_y(south, zoom)
    return x_min, x_max, y_north, y_south


def download_tiles_to_mbtiles(bbox: tuple, zoom_min: int, zoom_max: int,
                              output_mbtiles: str, tile_source: str = "osm"):
    """下载瓦片并打包到 mbtiles（SQLite 格式）"""
    if os.path.exists(output_mbtiles):
        os.remove(output_mbtiles)

    conn = sqlite3.connect(output_mbtiles)
    c = conn.cursor()
    c.execute("""CREATE TABLE IF NOT EXISTS metadata (name text, value text)""")
    c.execute("""CREATE TABLE IF NOT EXISTS tiles
                 (zoom_level integer, tile_column integer, tile_row integer,
                  tile_data blob, PRIMARY KEY (zoom_level, tile_column, tile_row))""")
    c.execute("""CREATE UNIQUE INDEX tile_index ON tiles (zoom_level, tile_column, tile_row)""")

    # mbtiles metadata
    c.execute("INSERT INTO metadata VALUES ('name', 'rtk region map')")
    c.execute("INSERT INTO metadata VALUES ('format', 'png')")
    c.execute(f"INSERT INTO metadata VALUES ('bounds', '{bbox[3]},{bbox[1]},{bbox[2]},{bbox[0]}')")
    conn.commit()

    url_template = TILE_SERVERS[tile_source]
    headers = {"User-Agent": "rtk-wheelchair/1.0 (educational project)"}
    total = 0

    for z in range(zoom_min, zoom_max + 1):
        x_min, x_max, y_north, y_south = tile_ranges(bbox, z)
        count_in_zoom = 0
        for x in range(x_min, x_max + 1):
            for y in range(y_north, y_south + 1):
                url = url_template.format(z=z, x=x, y=y)
                try:
                    resp = requests.get(url, headers=headers, timeout=10)
                    if resp.status_code == 200:
                        # TMS 翻转 y：mbtiles 用 TMS，瓦片服务器用 XYZ
                        tms_y = (2 ** z - 1) - y
                        c.execute(
                            "INSERT OR REPLACE INTO tiles VALUES (?, ?, ?, ?)",
                            (z, x, tms_y, resp.content),
                        )
                        count_in_zoom += 1
                    time.sleep(0.1)  # 遵守 OSM tile 使用政策
                except requests.RequestException as e:
                    print(f"[TILE] WARN 失败 z={z} x={x} y={y}: {e}")
        conn.commit()
        total += count_in_zoom
        print(f"[TILE] z={z} 完成，{count_in_zoom} 张")

    conn.close()
    print(f"[TILE] 总计 {total} 张瓦片，已保存到 {output_mbtiles}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="config/region.yaml")
    parser.add_argument("--skip-osm", action="store_true", help="跳过 OSM 路网下载")
    parser.add_argument("--skip-tiles", action="store_true", help="跳过瓦片图下载")
    parser.add_argument("--tile-source", default="osm", choices=list(TILE_SERVERS.keys()))
    args = parser.parse_args()

    config = load_region_config(args.config)
    bbox = compute_bbox(config)
    if not validate_bbox(bbox):
        print(f"ERROR: 无效的 bbox: {bbox}。原因：{describe_bbox_error(bbox)}", file=sys.stderr)
        sys.exit(1)

    output = config["output"]
    os.makedirs(os.path.dirname(output["osm_pbf"]) or ".", exist_ok=True)

    if not args.skip_osm:
        download_osm_graph(config, output["roads_geojson"])

    if not args.skip_tiles:
        download_tiles_to_mbtiles(
            bbox,
            config["tiles_zoom_min"],
            config["tiles_zoom_max"],
            output["mbtiles"],
            args.tile_source,
        )

    print("[DONE] 数据准备完成")


if __name__ == "__main__":
    main()
