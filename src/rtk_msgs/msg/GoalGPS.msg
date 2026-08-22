# 用户标定的终点经纬度
# 由前端（地图点击或 POI 选择）发布

std_msgs/Header header

string   source       # "map_click" 或 "poi"
string   poi_name     # 如果是 POI 选择，记录名称；否则为空

float64  latitude     # 纬度（WGS84）
float64  longitude    # 经度（WGS84）
float64  altitude     # 海拔（米），可选
