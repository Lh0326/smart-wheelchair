std_msgs/Header header
int32 class_id          # COCO 类 ID(0=人,1=自行车,2=汽车,9=红绿灯...)
string class_name       # 类名(person/bicycle/car/...)
float32 confidence      # 0.0-1.0
float32 distance_m      # base_link 前向距离(x),-1=无效
int32[4] bbox_px        # [x1, y1, x2, y2] 像素坐标
geometry_msgs/Point center_3d   # 在 base_link 系下 3D 位置(m)
