"""启动 Orbbec Gemini 335L 仅 IMU（关闭彩色+深度，避免 USB 3.0 干扰 GPS）

⚠️ 重要：USB 3.0 高速数据流（5 Gbps）会辐射 1.5-3 GHz 宽频噪声，严重干扰
GPS L1 (1575.42 MHz) 接收灵敏度。实测启用 color+depth 15fps 会让 GPS 完全
收不到卫星（CNR=0），关闭后 GPS 立即恢复正常。

如需彩色/深度做感知（M4 阶段），建议：
1. 把相机接到远离 GPS 的 USB 3.0 端口（独立控制器）
2. 用屏蔽良好的 USB 3.0 线
3. 或加 RF 屏蔽罩隔离

发布话题（仅 IMU）：
  /camera/gyro_accel/sample - 6DoF IMU（陀螺+加速度），200Hz
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    orbbec_dir = get_package_share_directory('orbbec_camera')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(orbbec_dir, 'launch', 'gemini_330_series.launch.py')
            ),
            launch_arguments={
                # USB 3.0 RF 干扰 GPS：降到最低带宽
                # 彩色流降到最低分辨率最低帧率（驱动要求至少一个 video stream）
                'enable_color': 'true',
                'color_width': '424',
                'color_height': '240',
                'color_fps': '5',
                # 关闭深度（深度流带宽最大，干扰最强）
                'enable_depth': 'false',
                'enable_ir': 'false',
                'enable_infra': 'false',
                'enable_infra1': 'false',
                'enable_infra2': 'false',
                # IMU（USB HID 接口，带宽低）
                'enable_accel': 'true',
                'enable_gyro': 'true',
                'enable_sync_output_accel_gyro': 'true',
                'accel_rate': '200hz',
                'gyro_rate': '200hz',
            }.items(),
        ),
    ])

