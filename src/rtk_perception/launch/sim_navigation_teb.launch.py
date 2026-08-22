"""TEB 仿真导航 launch（替代 sim_navigation.launch.py 的 VFH+ 版本）。
import os as _os
def _find_ws_root():
    r = _os.environ.get("WS_ROOT")
    if r: return r
    d = _os.path.dirname(_os.path.abspath(__file__))
    for _ in range(6):
        if _os.path.exists(_os.path.join(d, "env.sh")): return d
        d = _os.path.dirname(d)
    return d
_WS_ROOT = _find_ws_root()
_MODELS_ROOT = _os.environ.get("MODELS_ROOT", _os.path.join(_WS_ROOT, "models"))


启动方式：
  source /opt/ros/humble/setup.bash
  source " + _WS_ROOT + "/third_party/teb_ws_install/install/setup.bash
  source " + _WS_ROOT + "/third_party/lidar_ros2_ws/install/setup.bash
  source " + _MODELS_ROOT + "/third_party/ldlidar_ws/install/setup.bash
  source " + _WS_ROOT + "/install/setup.bash
  ros2 launch rtk_perception sim_navigation_teb.launch.py

差异（与 sim_navigation.launch.py）：
  - 移除 vfh_avoidance_node, curb_detector_node
  - 新增 controller_server (TEB plugin，内部自动管理 local_costmap 子组件) + lifecycle_manager
  - 新增 path_feeder_node
  - 新增 map→odom 静态 TF
  - path_to_baselink 的 /target_heading 输出在仿真下无下游消费者，仅保留接口兼容
"""
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('rtk_perception')
    urdf = os.path.join(pkg_dir, 'urdf', 'wheelchair.urdf.xacro')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'perception_teb.rviz')
    params_file = os.path.join(pkg_dir, 'config', 'perception_params.yaml')
    teb_params = os.path.join(pkg_dir, 'config', 'teb_params.yaml')
    costmap_params = os.path.join(pkg_dir, 'config', 'local_costmap_params.yaml')
    ekf_params = os.path.join(pkg_dir, 'config', 'ekf.yaml')
    navsat_params = os.path.join(
        get_package_share_directory('rtk_gnss'), 'config', 'navsat_transform.yaml'
    )

    with open(urdf, 'r') as f:
        robot_desc = f.read()

    # use_real_imu=true 时启用 HWT906P 真实 IMU + navsat_transform + EKF 链路。
    # false（默认）保持纯仿真（sim_chassis 提供 /heading_imu + /fix）。
    use_real_imu = LaunchConfiguration('use_real_imu')
    # use_real_chassis=true 时启用实物底盘串口节点（替代 sim_chassis_node）。
    # false（默认）= 仿真（sim_chassis 提供虚拟位置/heading/odom）。
    # 互锁：use_real_chassis=true 时强制 use_real_imu=true（current_angle 必须真实）。
    use_real_chassis = LaunchConfiguration('use_real_chassis')
    # 互锁表达式：若 use_real_chassis=true，则 use_real_imu 也强制 true
    effective_use_real_imu = PythonExpression([
        "'", LaunchConfiguration('use_real_chassis'), "' == 'true' or '",
        LaunchConfiguration('use_real_imu'), "' == 'true'"
    ])

    return LaunchDescription([
        # === 启动参数 ===
        DeclareLaunchArgument(
            'use_real_imu',
            default_value='true',
            description='true（默认）=启动 HWT906P 真实 IMU + navsat_transform + EKF；'
                        'false=纯仿真（sim_chassis 提供 heading/fix）',
        ),
        DeclareLaunchArgument(
            'use_real_chassis',
            default_value='false',
            description='true=启动 chassis_serial_node 实物底盘（强制连带 use_real_imu=true）；'
                        'false（默认）=仿真（sim_chassis_node）',
        ),
        DeclareLaunchArgument(
            'chassis_serial_port',
            default_value='/dev/wheelchair_chassis',
            description='chassis_serial_node 串口设备路径（udev 别名或 /dev/ttyUSBx）',
        ),
        DeclareLaunchArgument(
            'imu_serial_port',
            default_value='/dev/ttyIMU',
            description='HWT906P/JY901 IMU 串口设备路径（udev 别名、by-path 或 /dev/ttyUSBx）',
        ),
        DeclareLaunchArgument(
            'n10p_serial_port',
            default_value='/dev/lidar_n10p',
            description='N10P 雷达串口设备路径（udev 别名、by-id、by-path 或 ttyUSB）',
        ),
        DeclareLaunchArgument(
            'chassis_start_delay_sec',
            default_value='15.0',
            description='实物底盘串口延迟启动秒数，用于 USB 外设错峰上电',
        ),
        DeclareLaunchArgument(
            'gnss_at_port',
            default_value='/dev/ttyUSB_AT',
            description='EC20 AT 命令串口（用于 AT+QGPS=1 启动 GNSS）',
        ),
        DeclareLaunchArgument(
            'gnss_nmea_port',
            default_value='/dev/ttyUSB_NMEA',
            description='EC20 NMEA 输出串口（发布真实 /fix）',
        ),
        DeclareLaunchArgument(
            'enable_voice_announce',
            default_value='false',
            description='true=启动路径语音播报（voice_announce_node）；'
                        'false（默认）=禁用，避免 TTS 推理瞬时电流叠加电机启动触发 USB Hub OCP。'
                        '排查底盘驱动问题时建议保持关闭。',
        ),

        # === URDF TF ===
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser',
            arguments=['-0.255', '0', '1.47', '0', '0', '0', 'base_link', 'laser'],
        ),
        # TEB 避障修复：补 LD14P 静态 TF（base_link→ld14p_link，前方低位）
        # 不加这个 TF，costmap 无法把 LD14P 点云投影到 base_link 系，全部丢弃
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_ld14p',
            arguments=['0.32', '0', '0.13', '0', '0', '0', 'base_link', 'ld14p_link'],
        ),
        # TEB 硬性需求：map→odom identity TF
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        ),

        # === Phase A 配套 TF：gemini_link → camera_link（桥接 URDF 与 Orbbec 内部 TF）===
        # 只发布这一个桥接 TF；camera_link 内部的 camera_depth_frame / camera_depth_optical_frame /
        # camera_color_frame / camera_color_optical_frame 由 Orbbec driver 自己发布(publish_tf:=true),
        # 避免静态 TF publisher 与 driver 内部 TF publisher 冲突(RViz 深度点云"一条直线"现象)。
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='gemini_link_to_camera_link',
            arguments=['0', '0', '0', '0', '0', '0', 'gemini_link', 'camera_link'],
        ),

        # === 真实雷达驱动 ===
        # C++ lslidar_driver v5.1.1 在本机持续 CRC 失败 + 即使 CRC 通过也无距离数据
        # （雷达硬件可能激光器/接收器故障，但 Python 驱动确认协议解析正确）
        # 改用 Python 驱动：scripts/n10p_python_driver.py
        # 如更换雷达硬件后想回到 C++：注释掉下行 Python 启动，恢复下面 ExecuteProcess
        ExecuteProcess(
            cmd=[
                'python3', _WS_ROOT + '/scripts/n10p_python_driver.py',
                '--ros-args',
                '-p', 'output_topic:=/scan_n10p_raw',
                '-p', 'frame_id:=laser',
                '-p', ['port:=', LaunchConfiguration('n10p_serial_port')],
                # 修复：N10P 雷达硬件 CW 扫描方向，需反转才能与 ROS CCW 标准一致
                # （LD14P 用 laser_scan_dir:=true, Gemini 通过 depthimage_to_laserscan 默认）
                # 设 false 可回滚对比
                '-p', 'flip_horizontal:=true',
            ],
            output='screen',
        ),
        # ExecuteProcess(
        #     cmd=['ros2', 'launch', 'lslidar_driver', 'lsn10p_launch.py'],
        #     output='screen',
        # ),
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ldlidar', 'ldlidar', '--ros-args',
                '-p', 'product_name:=LDLiDAR_LD14P',
                # 发布到 /scan_ld14p_raw(原始,含机身反射),
                # scan_min_range_filter 节点过滤后发布 /scan_ld14p
                '-p', 'topic_name:=scan_ld14p_raw',
                '-p', 'port_name:=/dev/LD14P',
                '-p', 'frame_id:=ld14p_link',
                '-p', 'laser_scan_dir:=true',
                '-p', 'enable_angle_crop_func:=false',
                '-p', 'angle_crop_min:=135.0',
                '-p', 'angle_crop_max:=225.0',
                '-p', 'truncated_mode_:=0',
                '-r', '__node:=ldlidar_publisher_ld14p',
            ],
            output='screen',
        ),

        # N10P 机身屏蔽过滤:以 base_link 几何中心为参考,机身 110×90cm (X±0.55 / Y±0.45)。
        # N10P 位于 base_link 后方 0.255m,换算到 N10P 系:前 0.81m / 后 0.30m / 左右 0.45m。
        Node(
            package='rtk_perception',
            executable='scan_min_range_filter',
            name='n10p_scan_filter',
            parameters=[{
                'input_topic': '/scan_n10p_raw',
                'output_topic': '/scan',
                'rect_x_front': 0.81,
                'rect_x_back': 0.30,
                'rect_y_left': 0.45,
                'rect_y_right': 0.45,
            }],
            output='screen',
        ),

        # LD14P 机身屏蔽过滤:以 base_link 几何中心为参考,机身 110×90cm (X±0.55 / Y±0.45)。
        # LD14P 位于 base_link 前方 0.32m,换算到 LD14P 系:前 0.23m / 后 0.87m / 左右 0.45m。
        # 后方 0.87m 完整覆盖座椅靠背; 左右 0.45m 覆盖扶手+轮子宽度。
        Node(
            package='rtk_perception',
            executable='scan_min_range_filter',
            name='ld14p_min_range_filter',
            parameters=[{
                'input_topic': '/scan_ld14p_raw',
                'output_topic': '/scan_ld14p',
                'rect_x_front': 0.23,
                'rect_x_back': 0.87,
                'rect_y_left': 0.45,
                'rect_y_right': 0.45,
            }],
            output='screen',
        ),

        # === Phase A:Gemini 335L 深度相机(供 Tab 2 显示 + costmap 第三路障碍源)==="
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('orbbec_camera'),
                    'launch', 'gemini_330_series.launch.py'
                )
            ),
            launch_arguments={
                'depth_width': '848', 'depth_height': '480', 'depth_fps': '10',
                'color_width': '640', 'color_height': '480', 'color_fps': '30',
                # depth_registration=true:depth 对齐到 color 分辨率(640x480),
                # 让 YOLO bbox(color)像素与 depth 像素一一对应,_estimate_distance
                # 和 _backproject_to_3d 才能正确反投影距离。
                'depth_registration': 'true',
                # 让 driver 自己发布内部 TF(camera_link → camera_depth_frame /
                # camera_depth_optical_frame / camera_color_frame / camera_color_optical_frame),
                # 不再走静态 TF publisher,避免 TF 冲突。
                'publish_tf': 'true',
                # 单相机连续深度流:关闭 335L 默认触发/帧同步,避免设备在线但 color/depth 0 FPS。
                'software_trigger_enabled': 'false',
                'trigger_out_enabled': 'false',
                'enable_frame_sync': 'false',
                'uvc_backend': 'v4l2',
            }.items(),
        ),
        # === YOLO 检测(供 Tab 2 视频叠加 bbox + 语音前方距离播报)===
        Node(
            package='rtk_perception',
            executable='camera_detect_node',
            name='camera_detect_node',
            parameters=[{
                'camera_base_x_m': -0.255,
                'camera_base_y_m': -0.255,
                'camera_base_z_m': 1.25,
                'camera_yaw_rad': 0.0,
            }],
            output='screen',
        ),
        Node(
            package='rtk_perception',
            executable='detection_to_cloud',
            name='detection_to_cloud',
            output='screen',
        ),
        # 历史清理记录(2026-06-26 TEB 偏离修复时清理,2026-06-27 重新启用):
        #   - depthimage_to_laserscan_gemini: 重新启用(下方),输出 /scan_gemini
        #     供 costmap 第 3 路 source + fusion_scan_node + companion 三色显示
        #   - detection_to_cloud: 已启用，行人等语义目标进入 local_costmap

        # web_video_server:MJPEG 流供 Tab 2 相机视频(端口 8085,避开 mbtiles 8080)
        # companion/index.html 的 <img> 标签直接连这个端口拉 MJPEG
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
            parameters=[{'port': 8085}],
            output='screen',
        ),

        # depthimage_to_laserscan:depth image → /scan_gemini (sensor_msgs/LaserScan)
        # 把 Gemini 335L 的 3D 深度信息压缩为 2D LaserScan,补充 N10P 头部遮挡盲区。
        # range_min=0.5: 屏蔽 50cm 内噪声点云；矩形自车屏蔽在下方 filter 完成
        # scan_height=100: 中心 ±50 行 = ±6° 垂直范围,CPU 约 3-5%
        # output_frame=camera_depth_frame(z≈1.25m,与相机物理高度一致):
        #   costmap 通过 TF 把 LaserScan 投影到 z=1.25m 平面,落在 [0.1, 2.0] 内,
        #   才能进 obstacle layer + inflation layer(关键!用 base_link 时 z=0 被
        #   min_obstacle_height=0.1 过滤掉,点云显示但不避障)
        # fusion_scan_node 内部强制 /scan_fused frame_id=base_link(不影响 RViz 显示)
        # 注意:image_topic/scan 不是有效参数,必须用 remappings 改 topic 名
        # 默认订阅 topic:'depth'(image) + 'depth_camera_info'(CameraInfo,无斜杠)
        # 默认发布 topic:'scan'
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan_gemini',
            parameters=[{
                'output_frame': 'camera_depth_frame',
                'range_min': 0.5,
                'range_max': 5.0,
                'scan_height': 100,
                'scan_time': 0.1,
                'min_height': -10.0,
                'max_height': 10.0,
                'concurrency': 1,
            }],
            remappings=[
                ('depth', '/camera/depth/image_raw'),
                ('depth_camera_info', '/camera/depth/camera_info'),
                ('scan', '/scan_gemini_raw'),
            ],
            output='screen',
        ),

        # Gemini 335L 机身屏蔽过滤:以 base_link 几何中心为参考,机身 110×90cm (X±0.55 / Y±0.45)。
        # Gemini 位于 base_link 右后角(x=-0.255, y=-0.255),换算到相机系:
        #   前 0.81m / 后 0.30m / 左 0.71m (机身大部分在相机左侧) / 右 0.20m。
        Node(
            package='rtk_perception',
            executable='scan_min_range_filter',
            name='gemini_scan_filter',
            parameters=[{
                'input_topic': '/scan_gemini_raw',
                'output_topic': '/scan_gemini',
                'rect_x_front': 0.81,
                'rect_x_back': 0.30,
                'rect_y_left': 0.71,
                'rect_y_right': 0.20,
            }],
            output='screen',
        ),

        # 周期性清空 local_costmap 机器人周围残留障碍（ObstacleLayer 没有时间衰减机制，
        # 轮椅遮挡 + filter 屏蔽区会导致 cell 永不被 raytrace 清除 → 障碍移走后 inflation
        # 残留数秒）。每 2s 调 clear_around_local_costmap 清 4m，1-2s 内残留消失。
        # 节点自动发现 */clear_around_local_costmap service（local_costmap 是 controller_server
        # 内嵌子组件，service 名可能是 /controller_server/local_costmap/...）。
        # 清除瞬间真实障碍"消失"约 33ms（30Hz scan 立刻重新 mark），TEB 50ms 控制周期内恢复。
        # reset_distance 1.5→4m（2026-07-06）：用户报告"远处居多"残留，1.5m 圆够不着；
        # 4m 圆覆盖 TEB 前视 2.7m 决策域。window 保持 10m 不动 — 缩到 6m 会让 TEB 前视
        # 裕量仅 0.3m，候选轨迹可能触窗口边缘数据稀疏区。
        Node(
            package='rtk_perception',
            executable='costmap_periodic_clear',
            name='costmap_periodic_clear',
            parameters=[{
                'clear_period_s': 2.0,
                'reset_distance_m': 4.0,
                'service_suffix': 'clear_around_local_costmap',
                # 如要强制指定（覆盖自动发现），取消注释：
                # 'service_name': '/local_costmap/clear_around_local_costmap',
            }],
            output='screen',
        ),

        # camera_http_streamer:轻量 HTTP JPEG 缓存服务器(端口 8086)
        # 突破 web_video_server snapshot 端点的 10 FPS 上限(snapshot 每次请求都
        # 等下一帧 + 重编码,实测 100ms/帧)。本节点订阅 image_raw,按源频率编码一次,
        # HTTP 端点直接返回最新缓存 → 前端 33ms fetch 可达 30 FPS 显示。
        Node(
            package='rtk_perception',
            executable='camera_http_streamer',
            name='camera_http_streamer',
            parameters=[{
                'image_topic': '/camera/color/image_raw',
                'port': 8086,
                'jpeg_quality': 80,
            }],
            output='screen',
        ),

        # === 前端 + rosbridge + mbtiles ===
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('rtk_map'),
                    'launch', 'mbtiles_server.launch.py'
                )
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('rtk_frontend'),
                    'launch', 'frontend.launch.py'
                )
            ),
        ),

        # === NetworkX 路径规划 ===
        # 仿真模式订阅 /sim_fix；实物模式订阅真实 /fix
        Node(
            package='rtk_planner',
            executable='networkx_planner_node',
            name='networkx_planner',
            parameters=[{
                'osm_path': _WS_ROOT + '/data/region.graphml',
                'walking_speed_mps': 1.4,
                'min_goal_distance_m': 5.0,
            }],
            remappings=[
                ('/fix', '/sim_fix'),
            ],
            condition=UnlessCondition(effective_use_real_imu),
        ),
        Node(
            package='rtk_planner',
            executable='networkx_planner_node',
            name='networkx_planner_real',
            parameters=[{
                'osm_path': _WS_ROOT + '/data/region.graphml',
                'walking_speed_mps': 1.4,
                'min_goal_distance_m': 5.0,
            }],
            condition=IfCondition(effective_use_real_imu),
        ),

        # === path_to_baselink（扩展输出 /nav_path）===
        # 实物模式（use_real_imu=true）：订阅真实 /fix（EC20/DGPS）+ /heading_imu（HWT906P）
        # 仿真模式（默认）：订阅 sim_chassis 的 /sim_fix + /sim_heading_imu
        Node(
            package='rtk_perception',
            executable='path_to_baselink_node',
            name='path_to_baselink_node',
            parameters=[{
                'lookahead_distance_m': 2.0,
                'update_rate_hz': 5.0,    # 10→5:状态机 5Hz 够(IMU 滤波在 _heading_cb 100Hz 不变)
            }],
            remappings=[
                ('/fix', '/sim_fix'),
                ('/heading_imu', '/sim_heading_imu'),
            ],
            condition=UnlessCondition(effective_use_real_imu),
            output='screen',
        ),
        Node(
            package='rtk_perception',
            executable='path_to_baselink_node',
            name='path_to_baselink_node_real',
            parameters=[{
                'lookahead_distance_m': 2.0,
                'update_rate_hz': 5.0,
            }],
            # 实物模式：不 remap，订阅真实 /fix + /heading_imu
            condition=IfCondition(effective_use_real_imu),
            output='screen',
        ),

        # === 实物 IMU 链路（HWT906P + navsat_transform + EKF）===
        # 仅 use_real_imu=true（或 use_real_chassis=true 互锁强制 true）时启动；仿真模式跳过
        GroupAction(
            condition=IfCondition(effective_use_real_imu),
            actions=[
                # EC20 GNSS：实物模式下替代 sim_chassis 的虚拟 /fix。
                # 只发布位置；航向仍由 HWT906P /heading_imu 提供。
                Node(
                    package='rtk_gnss',
                    executable='ec20_gnss',
                    name='ec20_gnss',
                    parameters=[{
                        'at_port': LaunchConfiguration('gnss_at_port'),
                        'nmea_port': LaunchConfiguration('gnss_nmea_port'),
                        'baudrate': 9600,
                        'output_topic': '/fix',
                    }],
                    output='screen',
                ),
                # HWT906P/JY901 驱动（维特标准协议，baud=921600）
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory('rtk_imu'),
                            'launch', 'jy901.launch.py'
                        )
                    ),
                    launch_arguments={
                        'port': LaunchConfiguration('imu_serial_port'),
                        'baud': '921600',
                    }.items(),
                ),
                # navsat_transform：/fix + /imu/data → /odometry/gps + /gps/filtered
                Node(
                    package='robot_localization',
                    executable='navsat_transform_node',
                    name='navsat_transform_node',
                    parameters=[navsat_params],
                    remappings=[
                        ('/imu/data', '/imu/data'),
                        ('/gps/fix', '/fix'),
                    ],
                    output='screen',
                ),
                # EKF：/imu/data + /odometry/gps + /odom → /odometry/filtered + map→odom TF
                Node(
                    package='robot_localization',
                    executable='ekf_node',
                    name='ekf_local_node',
                    parameters=[ekf_params],
                    output='screen',
                ),
            ],
        ),

        # === fusion_scan_node:三路 LaserScan 融合 → /scan_fused (供 RViz 显示) ===
        # 注:costmap 不依赖此节点(三路独立进 costmap)。fusion_scan_node 仅输出 /scan_fused
        # 供 RViz 的 Fused scan 显示项 + 历史兼容。companion 用三路原始 scan 三色同屏。
        Node(
            package='rtk_perception',
            executable='fusion_scan_node',
            name='fusion_scan_node',
            output='screen',
        ),

        # === TEB: controller_server（内部自动创建 local_costmap 子组件）===
        # TEB 默认输出 /cmd_vel，由 safety_chain 接管转 /cmd_vel_safe
        # local_costmap 不需要独立节点：controller_server 加载时会从参数服务器
        # 读取 local_costmap.local_costmap.ros__parameters 节的配置（在 costmap_params 中）
        # 注意：nav2 controller_server 默认会发布 nav_msgs/Path 到 /global_plan topic，
        # 这会与 networkx_planner 的 rtk_msgs/GlobalPlan 类型冲突，导致前端订阅失败。
        # remap 到 /controller_received_path 避免冲突。
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            parameters=[teb_params, costmap_params],
            remappings=[
                ('/global_plan', '/controller_received_path'),
            ],
            output='screen',
        ),

        # === TEB: lifecycle_manager ===
        # 只管 controller_server（local_costmap 是其内部子组件，跟随其 lifecycle）
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            parameters=[{
                'autostart': True,
                'node_names': ['controller_server'],
                'bond_timeout': 4.0,
            }],
            output='screen',
        ),

        # === path_feeder（桥接 /nav_path → FollowPath action）===
        Node(
            package='rtk_perception',
            executable='path_feeder_node',
            name='path_feeder_node',
            parameters=[{
                'path_timeout_sec': 1.0,
                'goal_min_interval_sec': 0.5,
                'nav_control_heartbeat_sec': 0.2,
                'path_signature_resolution_m': 0.25,
                'abort_retry_backoff_sec': 5.0,
                'goal_delay_sec': 2.0,
                'controller_server_name': 'controller_server',
            }],
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ),

        # === safety_chain（保留）===
        Node(
            package='rtk_perception',
            executable='safety_chain_node',
            name='safety_chain_node',
            parameters=[params_file],
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ),

        # === TTS 语音播报链路 ===
        # tts_node（ladar-ai 包）订阅 /tts_request → TTSEngine → 扬声器
        # voice_announce_node 订阅 /global_plan + /fix + /clear_goal → 发 /tts_request
        # 顺序：tts_node 先起（避免 voice_announce 第一条消息丢失）
        Node(
            package='ladar_ai',
            executable='tts_node',
            name='tts_node',
            output='screen',
        ),
        Node(
            package='wheelchair_app',
            executable='voice_announce_node',
            name='voice_announce_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('enable_voice_announce')),
            parameters=[{
                'graphml_path': _WS_ROOT + '/data/region.graphml',
                'turn_ahead_meters': 5.0,
                'arrival_meters': 3.0,
                'offroute_meters': 25.0,
            }],
        ),

        # === TEB 调试可视化（新增） ===
        # 发布 4 个 RViz Marker：cmd_vel_arrow / trail / deviation_line / mode_text
        Node(
            package='rtk_perception',
            executable='teb_debug_node',
            name='teb_debug_node',
            parameters=[{
                'update_rate_hz': 10.0,      # 20→10:RViz 视觉无差别(延迟 50→100ms,人眼无感)
                'trail_duration_sec': 10.0,  # 30→10:1500 点→500 点,LINE_STRIP 构造快 3x
                'cmd_vel_arrow_scale': 2.0,
            }],
            output='screen',
        ),

        # === 虚拟底盘（扩展发布 odom→base_link TF）===
        # 仿真模式（use_real_chassis=false，默认）：sim_chassis 同时发 /fix + /heading_imu 给前端
        #   + 单独 use_real_imu=true（无真实底盘但有真实 IMU 时）：仍用 sim_chassis 但不发 /fix /heading_imu
        # 实物模式（use_real_chassis=true）：完全不启动 sim_chassis，由 chassis_serial_node 提供 odom+TF
        Node(
            package='rtk_perception',
            executable='sim_chassis_node',
            name='sim_chassis_node',
            parameters=[{
                'initial_lat': 24.8551,
                'initial_lon': 102.8553,
                'initial_heading_deg': 0.0,
                'update_rate_hz': 50.0,
                'publish_real_topics': True,
                'eeg_override_hold_sec': 1.5,
            }],
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('use_real_chassis'), "' == 'false' and '",
                LaunchConfiguration('use_real_imu'), "' == 'false'"
            ])),
            output='screen',
        ),
        Node(
            package='rtk_perception',
            executable='sim_chassis_node',
            name='sim_chassis_node',
            parameters=[{
                'initial_lat': 24.8551,
                'initial_lon': 102.8553,
                'initial_heading_deg': 0.0,
                'update_rate_hz': 50.0,
                'publish_real_topics': False,  # 不发 /fix /heading_imu，让真实硬件独占
                'eeg_override_hold_sec': 1.5,
            }],
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('use_real_chassis'), "' == 'false' and '",
                LaunchConfiguration('use_real_imu'), "' == 'true'"
            ])),
            output='screen',
        ),

        # === 实物底盘串口节点（use_real_chassis=true 时启动，替代 sim_chassis）===
        TimerAction(
            period=LaunchConfiguration('chassis_start_delay_sec'),
            actions=[Node(
                package='rtk_perception',
                executable='chassis_serial_node',
                name='chassis_serial_node',
                parameters=[{
                'serial_port': LaunchConfiguration('chassis_serial_port'),
                'baudrate': 115200,
                'serial_reopen_interval_sec': 1.0,
                'update_rate_hz': 100.0,
                'max_speed_mps': 1.5,
                'lead_gain_deg_per_rad_per_sec': 30.0,
                'max_lead_deg': 60.0,
                'heading_sign': -1,
                'nav_chassis_control_enabled': True,
                'nav_control_timeout_sec': 0.8,
                # 实物底盘死区补偿：虚拟底盘能响应很小 Twist，真实电机可能不动。
                # 仅 Nav2/TEB 模式使用；脑控标定帧不受影响。
                # 实物安全优先：只略高于已知电机死区，不再把很小的 TEB 输出
                # 直接抬升到 800/600，避免起步和反复纠偏时产生大电流冲击。
                # 当前实物底盘 300 左右仍可能只保持力矩不转动；提高到可动区，
                # 但继续通过 800/s 斜率限制缓慢爬升，避免启动电流尖峰。
                'nav_min_forward_speed': 550,
                'nav_min_turn_speed': 500,
                'nav_linear_deadband_mps': 0.01,
                'nav_angular_deadband_rad_s': 0.05,
                # floor 触发阈值：0.05 → 0.02。让 TEB 启动初期很小的输出也能进 floor。
                # deadband=0.01 是零速门限，0.02 略高于它即可。
                'nav_floor_linear_threshold_mps': 0.02,
                # 0→400 约 0.5 秒，急停同样平滑回落；Nav2 分支现在也会实际经过该限速器。
                'forward_speed_ramp_per_sec': 650,
                # 允许避障所需转向，但由TEB角加速度和长前视共同抑制急摆。
                'nav_omega_cap_rad_s': 0.45,
                # 0.5m前视对局部轨迹噪声过于敏感；加长并降低LPF系数，
                # 让底盘跟随轨迹趋势而不是逐点来回纠偏。
                'nav_lookahead_distance_m': 0.8,
                'nav_lookahead_lpf_alpha': 0.20,
                'nav_steering_deadband_deg': 2.0,
                'nav_teb_max_vel_x': 0.5,
                # 保留对明显方向/速度阶跃的敏感保护，连续抖动时延长恢复窗口。
                'nav2_lookahead_jump_threshold_deg': 60.0,
                'nav2_cmd_vel_step_linear': 0.40,
                'nav2_cmd_vel_step_angular': 0.55,
                'nav2_ramp_cooldown_ms': 180,
                'nav2_hard_cooldown_ms': 900,
                'nav2_cooldown_escalation_count': 5,
                # 直行锁定 HWT 绝对航向，使机械偏航形成下位机 PID 纠偏误差。
                'nav_heading_hold_enabled': False,
                'nav_steering_trim_deg': 0.0,
                'nav_heading_hold_enter_error_deg': 2.5,
                'nav_heading_hold_exit_error_deg': 6.0,
                'nav_heading_hold_enter_omega_rad_s': 0.03,
                'nav_heading_hold_exit_omega_rad_s': 0.08,
                # 载人实物自主导航默认不执行倒车；脑控后退标定帧不受此参数影响。
                'nav_allow_reverse': False,
                'gps_required_for_motion': True,
                'gps_fix_timeout_sec': 1.5,
                'heading_timeout_sec': 0.5,
                'cmd_vel_timeout_sec': 1.0,
                # 脑控非零动作触发临时接管；用户松开后保持零速 1.5s 再交回 Nav2。
                'eeg_override_hold_sec': 1.5,
                'shutdown_zero_repeat': 3,
                }],
                condition=IfCondition(use_real_chassis),
                output='screen',
                respawn=True,
                respawn_delay=3.0,
            )],
        ),

        # === RViz ===
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
        ),
    ])
