#!/usr/bin/env bash
# 双源路径规划对比查询
# 用法：bash scripts/check_route.sh LON1 LAT1 LON2 LAT2
# 示例：bash scripts/check_route.sh 102.8553 24.8551 102.8650 24.8600
#
# 输出：本地 OSRM 状态 + 公共 OSRM 状态 + 推荐

set -e

if [ "$#" -ne 4 ]; then
    echo "用法: $0 LON1 LAT1 LON2 LAT2"
    echo "示例: $0 102.8553 24.8551 102.8650 24.8600"
    exit 1
fi

LON1=$1; LAT1=$2; LON2=$3; LAT2=$4

LOCAL_URL="http://localhost:5000/route/v1/walking/${LON1},${LAT1};${LON2},${LAT2}?overview=full&geometries=geojson"
PUBLIC_URL="https://router.project-osrm.org/route/v1/walking/${LON1},${LAT1};${LON2},${LAT2}?overview=full&geometries=geojson"

echo "═══════════════════════════════════════════════════"
echo "起点: (${LAT1}, ${LON1})"
echo "终点: (${LAT2}, ${LON2})"
echo "═══════════════════════════════════════════════════"

echo ""
echo "【本地 OSRM】"
LOCAL_RESP=$(curl -s --max-time 3 "${LOCAL_URL}" 2>&1 || echo "{'code':'CURL_FAIL'}")
echo "$LOCAL_RESP" | python3 -c "
import json, sys
try:
    d = json.load(sys.stdin)
except:
    print('  ❌ 响应解析失败（本地 OSRM 未启动？）')
    sys.exit(0)
code = d.get('code', '?')
if code == 'Ok':
    r = d['routes'][0]
    print(f'  ✅ OK | 距离 {r[\"distance\"]:.0f}m | 时长 {r[\"duration\"]:.0f}s | {len(r[\"geometry\"][\"coordinates\"])} 路径点')
else:
    msg = d.get('message', '?')
    print(f'  ❌ {code} | {msg}')
"

echo ""
echo "【公共 OSRM】"
PUBLIC_RESP=$(curl -s --max-time 5 "${PUBLIC_URL}" 2>&1 || echo "{'code':'CURL_FAIL'}")
echo "$PUBLIC_RESP" | python3 -c "
import json, sys
try:
    d = json.load(sys.stdin)
except:
    print('  ❌ 响应解析失败（网络不通或超时）')
    sys.exit(0)
code = d.get('code', '?')
if code == 'Ok':
    r = d['routes'][0]
    print(f'  ✅ OK | 距离 {r[\"distance\"]:.0f}m | 时长 {r[\"duration\"]:.0f}s | {len(r[\"geometry\"][\"coordinates\"])} 路径点')
else:
    msg = d.get('message', '?')
    print(f'  ❌ {code} | {msg}')
"

echo ""
echo "═══════════════════════════════════════════════════"
echo "$LOCAL_RESP" | python3 -c "
import json, sys
try: local_ok = json.load(sys.stdin).get('code')=='Ok'
except: local_ok = False
print('推荐：' + ('✅ 离线可用（本地 OSRM 就够）' if local_ok else '⚠️  必须联网（本地 OSRM 不通，靠公共 OSRM 兜底）'))
"
echo "═══════════════════════════════════════════════════"
