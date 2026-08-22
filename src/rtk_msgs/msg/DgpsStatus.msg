# DGPS 差分定位诊断信息
# 由 rtk_gnss/dgps_node 发布到 /gps/diagnostics
#
# 用于前端诊断面板展示基站/流动站状态、修正量大小、数据新鲜度

std_msgs/Header header

# 状态码
#   0 = OK          差分成功，/fix 是修正后位置
#   1 = STALE_DATA  base 或 rover 数据过期（> max_age_sec）
#   2 = NO_BASE     还未收到任何 base 数据
#   3 = NO_ROVER    还未收到任何 rover 数据
#   4 = NO_FIX      base 或 rover 当前 quality=0（未定位）
uint8   status
string  status_message

# 基站实时测量值（含误差）
float64 base_lat
float64 base_lon

# 基站已知坐标（预先标定，配置参数传入）
float64 base_known_lat
float64 base_known_lon

# 基站漂移修正量
float64 delta_lat         # 度
float64 delta_lon         # 度
float64 delta_meters      # |Δ| 投影到地面的米数（诊断用）

# 卫星质量
int32    base_satellites
int32    rover_satellites
float64  base_hdop
float64  rover_hdop

# 数据新鲜度（秒，越小越新鲜）
float64  base_age_sec
float64  rover_age_sec

# 是否有效定位（quality > 0）
bool     base_fix_valid
bool     rover_fix_valid
