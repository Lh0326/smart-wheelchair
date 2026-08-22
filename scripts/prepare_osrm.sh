#!/usr/bin/env bash
# 用 OSRM Docker 预处理路径规划图
# 输入：data/region.osm（由 osmnx 生成）
# 输出：data/region.osrm.* 系列
#
# 用法：bash scripts/prepare_osrm.sh

set -euo pipefail

cd "$(dirname "$0")/.."

OSM_FILE="data/region.osm"

# 用 osmnx 生成 OSM XML（OSRM 需要 .osm 格式，不是 GeoJSON）
# 注意：osmnx 2.0.7 的 graph_from_bbox API 参数顺序是 (left, bottom, right, top) = (west, south, east, north)
python3 -c "
import osmnx as ox
import yaml
with open('config/region.yaml') as f:
    cfg = yaml.safe_load(f)
north = cfg['region']['north_east']['lat']
south = cfg['region']['south_west']['lat']
east = cfg['region']['north_east']['lon']
west = cfg['region']['south_west']['lon']
# osmnx 2.0.7: graph_from_bbox(bbox=(left, bottom, right, top)) = (west, south, east, north)
# 注意：simplify=False 必填——OSM XML 必须保存未简化的图（osmnx 默认会合并道路节点）
g = ox.graph_from_bbox((west, south, east, north), network_type='walk', simplify=False)
ox.save_graph_xml(g, filepath='data/region.osm')
print('OSM XML 已保存到 data/region.osm')
"

if [ ! -f "$OSM_FILE" ]; then
    echo "ERROR: $OSM_FILE 生成失败"
    exit 1
fi

# 优先使用当前用户 docker（已加入 docker 组时），否则回退到 sudo docker
# （新加入 docker 组需注销重登才能直接调用 docker）
if docker info >/dev/null 2>&1; then
    DOCKER="docker"
elif sudo -n docker info >/dev/null 2>&1; then
    DOCKER="sudo docker"
else
    echo "ERROR: docker 命令不可用。请先安装 Docker 并将当前用户加入 docker 组（注销重登）。"
    exit 1
fi

echo "[OSRM] 拉取 Docker 镜像（首次约 200MB，请耐心等待）..."
$DOCKER pull osrm/osrm-backend

echo "[OSRM] 提取..."
$DOCKER run --rm -t -v "$(pwd)/data:/data" osrm/osrm-backend osrm-extract \
    -p /opt/foot.lua "/data/region.osm"

echo "[OSRM] 分区..."
$DOCKER run --rm -t -v "$(pwd)/data:/data" osrm/osrm-backend osrm-partition \
    "/data/region.osrm"

echo "[OSRM] 自定义（这一步可能耗时 1-3 分钟）..."
$DOCKER run --rm -t -v "$(pwd)/data:/data" osrm/osrm-backend osrm-customize \
    "/data/region.osrm"

echo "[OSRM] 完成。文件清单："
ls -lh data/region.osrm*
