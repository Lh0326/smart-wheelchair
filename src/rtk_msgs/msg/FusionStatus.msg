# GPS 融合诊断信息（HDOP 加权 + 滑窗融合节点输出）
# 由 rtk_gnss/gps_fusion_node 发布到 /gps/fusion_diag

std_msgs/Header header

# 状态码
#   0 = OK         融合成功，/fix 是融合后位置
#   1 = STALE      base 或 rover 数据过期
#   2 = NO_BASE    未收到 base
#   3 = NO_ROVER   未收到 rover
#   4 = NO_FIX     base 或 rover 未定位
uint8   status
string  status_message

# 基站（EC20）实测
float64 base_lat
float64 base_lon
float64 base_hdop
int32   base_satellites
bool    base_fix_valid
float64 base_age_sec

# 流动站（DX-GP10）实测
float64 rover_lat
float64 rover_lon
float64 rover_hdop
int32   rover_satellites
bool    rover_fix_valid
float64 rover_age_sec

# HDOP²倒数归一化权重（和=1.0）
float64 base_weight
float64 rover_weight

# 单帧融合位置（不滑窗）
float64 fused_lat_instant
float64 fused_lon_instant

# 滑窗后融合位置（最终输出到 /fix 的位置）
float64 fused_lat
float64 fused_lon

# 滑窗状态
int32   window_samples
int32   window_size

# 融合后水平精度估计（米，1σ）
float64 fused_sigma_meters

# 两机水平距离（米，诊断多径/偏差）
float64 base_rover_meters
