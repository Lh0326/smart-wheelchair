# OSRM 路径规划结果
# 由 rtk_planner/osrm_planner_node 发布

std_msgs/Header header

# 起点（取自最新 /fix 缓存）
float64  start_lat
float64  start_lon

# 终点（取自触发的 /goal_gps）
float64  goal_lat
float64  goal_lon
string   goal_source       # "map_click" 或 "poi"

# 路径点 WGS84 数组（x=lon, y=lat, z=0）
# status=OK 且 path 非空时有效；status=OK 但距离<min_goal_distance_m 时为单点
geometry_msgs/Point[] path_wgs84

# 元信息
float64  distance_meters
float64  duration_seconds

# 源（用于调试和前端日志）
string   source            # "local_osrm" / "public_osrm" / "noop"

# 状态码
#   OK          - 规划成功
#   NO_ROUTE    - 本地和公共都返回 code != Ok
#   ALL_FAILED  - HTTP 异常或 /fix 为空
string   status

# status != OK 时的详细原因
string   error_message
