"""仿真导航 launch（半实物仿真：真雷达 + 虚拟底盘）。
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
  source " + _WS_ROOT + "/third_party/lidar_ros2_ws/install/setup.bash
  source " + _MODELS_ROOT + "/third_party/ldlidar_ws/install/setup.bash
  source " + _WS_ROOT + "/install/setup.bash
  ros2 launch rtk_perception sim_navigation.launch.py

与实物模式（run_m4_full.sh）完全隔离：
  - path_to_baselink remap 订阅 /sim_fix + /sim_heading_imu
  - 不启动 EC20 GNSS / imu_heading / orbbec / fusion_scan
  - 新增 sim_chassis_node 执行 cmd_vel_safe
"""
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('rtk_perception')
    urdf = os.path.join(pkg_dir, 'urdf', 'wheelchair.urdf.xacro')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'perception.rviz')
    params_file = os.path.join(pkg_dir, 'config', 'perception_params.yaml')

    with open(urdf, 'r') as f:
        robot_desc = f.read()

    return LaunchDescription([
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

        # === 真实雷达驱动 ===
        ExecuteProcess(
            cmd=['ros2', 'launch', 'lslidar_driver', 'lsn10p_launch.py'],
            output='screen',
        ),
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ldlidar', 'ldlidar', '--ros-args',
                '-p', 'product_name:=LDLiDAR_LD14P',
                '-p', 'topic_name:=scan_ld14p',
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
        ),

        # === 感知节点 ===
        Node(
            package='rtk_perception',
            executable='path_to_baselink_node',
            name='path_to_baselink_node',
            parameters=[{
                'lookahead_distance_m': 2.0,
                'update_rate_hz': 10.0,
            }],
            remappings=[
                ('/fix', '/sim_fix'),
                ('/heading_imu', '/sim_heading_imu'),
            ],
            output='screen',
        ),
        Node(
            package='rtk_perception',
            executable='curb_detector_node',
            name='curb_detector_node',
            parameters=[params_file],
            output='screen',
        ),
        Node(
            package='rtk_perception',
            executable='vfh_avoidance_node',
            name='vfh_avoidance_node',
            parameters=[params_file],
            output='screen',
        ),
        Node(
            package='rtk_perception',
            executable='safety_chain_node',
            name='safety_chain_node',
            parameters=[params_file],
            output='screen',
        ),

        # === 虚拟底盘（同时发 /sim_fix + /fix + /sim_heading_imu + /heading_imu）===
        Node(
            package='rtk_perception',
            executable='sim_chassis_node',
            name='sim_chassis_node',
            parameters=[{
                'initial_lat': 24.8551,
                'initial_lon': 102.8553,
                'initial_heading_deg': 0.0,
                'update_rate_hz': 50.0,
                'eeg_override_hold_sec': 1.5,
            }],
            output='screen',
        ),

        # === RViz ===
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
        ),
    ])
