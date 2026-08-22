"""Gemini 335L RGB + 深度 → YOLO 检测 → 发布 DetectionArray。
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


复用 ladar-ai/yolo_engine(OpenVINO NPU @ 76 FPS)和深度 ROI 距离估算。
检测点发布为 base_link 坐标,距离为相对轮椅中心的前向距离。

ladar-ai 接口(已探查):
  - create_yolo_engine(model_path: str, conf: float, device: str)
  - detect(model, frame) -> List[Dict]  字段:class_id/class_name/confidence/bbox
  - 引擎层已按 TARGET_CLASSES 过滤(person/bicycle/car/motorcycle/bus/truck/traffic_light)
"""
import sys
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_parameters
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from geometry_msgs.msg import Point

from rtk_msgs.msg import Detection, DetectionArray

# 复用 ladar-ai
sys.path.insert(0, _WS_ROOT + '/third_party/ladar_ai/src/ladar_ai')
try:
    from yolo_engine import create_yolo_engine, detect
    LADAR_AI_AVAILABLE = True
    IMPORT_ERROR = ''
except ImportError as e:
    LADAR_AI_AVAILABLE = False
    IMPORT_ERROR = str(e)

try:
    from cv_bridge import CvBridge
    BRIDGE_AVAILABLE = True
except ImportError as e:
    CvBridge = None
    BRIDGE_AVAILABLE = False
    _BRIDGE_IMPORT_ERROR = str(e)

try:
    import numpy as np
except ImportError:
    np = None


YOLO_MODEL_PATH = _MODELS_ROOT + '/yolo/yolo11s_openvino/yolo11s.xml'
DEVICE = 'NPU'
CONF_THRESHOLD = 0.5
DETECTION_HZ = 15.0

# 实物安装：base_link 位于 51cm 正方形轮椅俯视中心，x 前/y 左/z 上。
GEMINI_BASE_X_M = -0.255
GEMINI_BASE_Y_M = -0.255
GEMINI_BASE_Z_M = 1.25
GEMINI_YAW_RAD = 0.0

# COCO 类 ID → 类名(精简:行人 + 交通 + chair)
# 调用 detect() 时传 target_classes=TRACKED_CLASSES 覆盖 ladar-ai 默认过滤
TRACKED_CLASSES = {
    # 行人
    0:  'person',
    # 交通参与者
    1:  'bicycle',
    2:  'car',
    3:  'motorcycle',
    5:  'bus',
    7:  'truck',
    # 交通标识
    9:  'traffic_light',
    11: 'stop_sign',
    # 家具(室内核心障碍)
    56: 'chair',
}


class CameraDetectNode(Node):
    def __init__(self):
        super().__init__('camera_detect_node')

        if not LADAR_AI_AVAILABLE:
            self.get_logger().error(f'ladar_ai 不可用: {IMPORT_ERROR}')
            return
        if not BRIDGE_AVAILABLE:
            self.get_logger().error(f'cv_bridge 不可用: {_BRIDGE_IMPORT_ERROR}')
            return
        if np is None:
            self.get_logger().error('numpy 不可用')
            return

        self._bridge = CvBridge()
        self._latest_color = None
        self._latest_depth = None
        self._camera_info = None
        self._color_lock = threading.Lock()
        self._depth_lock = threading.Lock()
        self._detector = None

        self.declare_parameter('camera_base_x_m', GEMINI_BASE_X_M)
        self.declare_parameter('camera_base_y_m', GEMINI_BASE_Y_M)
        self.declare_parameter('camera_base_z_m', GEMINI_BASE_Z_M)
        self.declare_parameter('camera_yaw_rad', GEMINI_YAW_RAD)
        self._camera_base_x = float(self.get_parameter('camera_base_x_m').value)
        self._camera_base_y = float(self.get_parameter('camera_base_y_m').value)
        self._camera_base_z = float(self.get_parameter('camera_base_z_m').value)
        self._camera_yaw = float(self.get_parameter('camera_yaw_rad').value)

        # YOLO 引擎(NPU 加速)
        try:
            self._detector = create_yolo_engine(
                YOLO_MODEL_PATH, conf=CONF_THRESHOLD, device=DEVICE
            )
            self.get_logger().info(
                f'YOLO 加载完成: {YOLO_MODEL_PATH} @ {DEVICE}'
            )
        except Exception as e:
            self.get_logger().error(f'YOLO 加载失败: {e}')
            return

        # 订阅相机
        self.create_subscription(
            Image, '/camera/color/image_raw', self._on_color, 10
        )
        self.create_subscription(
            Image, '/camera/depth/image_raw', self._on_depth, 10
        )

        # 内参(latched)
        self.create_subscription(
            CameraInfo, '/camera/depth/camera_info', self._on_info,
            qos_profile=qos_profile_parameters
        )

        # 发布检测结果
        self._pub = self.create_publisher(DetectionArray, '/detections', 10)

        # 推理定时器(15Hz,留余量)
        self.create_timer(1.0 / DETECTION_HZ, self._tick)
        self.get_logger().info(
            f'camera_detect_node ready @ {DETECTION_HZ:.1f}Hz'
        )

    def _on_color(self, msg):
        try:
            color = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
            with self._color_lock:
                self._latest_color = color
        except Exception as e:
            self.get_logger().warn(f'color cv 失败: {e}')

    def _on_depth(self, msg):
        try:
            # Orbbec 深度图:16UC1(mm);passthrough 保留原生编码
            depth = self._bridge.imgmsg_to_cv2(msg, 'passthrough')
            with self._depth_lock:
                self._latest_depth = depth
        except Exception as e:
            self.get_logger().warn(f'depth cv 失败: {e}')

    def _on_info(self, msg):
        if self._camera_info is None:
            self.get_logger().info('收到 CameraInfo,可做 3D 反投影')
        self._camera_info = msg

    def _tick(self):
        if self._detector is None:
            return

        with self._color_lock:
            color = self._latest_color
        if color is None:
            return

        with self._depth_lock:
            depth = self._latest_depth

        # YOLO 检测(ladar-ai 的 detect 在引擎层已按 TARGET_CLASSES 过滤)
        try:
            # YOLO 检测(传 target_classes=TRACKED_CLASSES 覆盖 ladar-ai 默认 7 类过滤)
            detections = detect(self._detector, color, target_classes=TRACKED_CLASSES)
        except Exception as e:
            self.get_logger().warn(f'YOLO 推理失败: {e}')
            return

        msg = DetectionArray()
        msg.header = Header()
        msg.header.frame_id = 'base_link'
        msg.header.stamp = self.get_clock().now().to_msg()

        for det in detections:
            # ladar-ai 返回 dict 字段:class_id / class_name / confidence / bbox
            cls_id = det.get('class_id')
            conf = det.get('confidence', 0.0)
            bbox = det.get('bbox')
            if cls_id is None or bbox is None:
                continue
            if cls_id not in TRACKED_CLASSES:
                # 引擎层已过滤,理论上不会到这里,二次保险
                continue

            x1, y1, x2, y2 = bbox
            distance = self._estimate_distance(bbox, depth)

            d = Detection()
            d.class_id = int(cls_id)
            d.class_name = det.get('class_name') or TRACKED_CLASSES[cls_id]
            d.confidence = float(conf)
            d.bbox_px = [int(x1), int(y1), int(x2), int(y2)]
            camera_pt = self._backproject_to_3d(bbox, distance)
            base_pt = self._camera_optical_to_base(camera_pt)
            # 语音和前端 bbox 显示使用“轮椅中心到目标的前向距离”。
            # 例如相机正前方 1.0m: base_x = -0.255 + 1.0 = 0.745m。
            d.distance_m = float(base_pt.x) if distance > 0 else -1.0
            d.center_3d = base_pt
            msg.detections.append(d)

        # 始终发布(即使空,让下游知道节点活着)
        self._pub.publish(msg)

    def _estimate_distance(self, bbox, depth_image):
        """bbox 中心 ROI 的深度中值(mm → m)。"""
        if depth_image is None or np is None:
            return -1.0
        x1, y1, x2, y2 = [int(v) for v in bbox]
        # 中心 40% ROI,避免边缘
        cx1 = int(x1 + (x2 - x1) * 0.3)
        cx2 = int(x2 - (x2 - x1) * 0.3)
        cy1 = int(y1 + (y2 - y1) * 0.3)
        cy2 = int(y2 - (y2 - y1) * 0.3)
        # 边界检查
        h, w = depth_image.shape[:2]
        cx1, cx2 = max(0, cx1), min(w, cx2)
        cy1, cy2 = max(0, cy1), min(h, cy2)
        if cx1 >= cx2 or cy1 >= cy2:
            return -1.0
        roi = depth_image[cy1:cy2, cx1:cx2].astype(np.float32)
        # 过滤无效(0 或 NaN)
        valid = roi[np.isfinite(roi)]
        valid = valid[(valid > 0)]
        # 假设 mm 输出,若值很大除以 1000
        if valid.size == 0:
            return -1.0
        median = float(np.median(valid))
        if median > 100.0:
            median = median / 1000.0   # mm → m
        if median < 0.15 or median > 20.0:
            return -1.0
        return median

    def _backproject_to_3d(self, bbox, distance):
        """bbox 中心 + 距离 → gemini_link_optical 系 3D 坐标。"""
        pt = Point()
        if distance < 0 or self._camera_info is None:
            return pt
        x1, y1, x2, y2 = bbox
        cx = (x1 + x2) / 2.0
        cy = (y1 + y2) / 2.0
        K = self._camera_info.k
        fx, fy = K[0], K[4]
        cx0, cy0 = K[2], K[5]
        if fx <= 0 or fy <= 0:
            return pt
        # 光学系:z 朝前,x 朝右,y 朝下
        pt.z = float(distance)
        pt.x = float((cx - cx0) * distance / fx)
        pt.y = float((cy - cy0) * distance / fy)
        return pt

    def _camera_optical_to_base(self, optical_pt):
        """gemini optical 坐标 → base_link 坐标。

        optical: z 前,x 右,y 下。base_link: x 前,y 左,z 上。
        """
        pt = Point()
        forward = optical_pt.z
        left = -optical_pt.x
        up = -optical_pt.y

        # 当前 Gemini yaw=0；保留 yaw 参数，后续实测偏航可直接调整。
        if np is not None:
            c = float(np.cos(self._camera_yaw))
            s = float(np.sin(self._camera_yaw))
        else:
            c = 1.0
            s = 0.0

        pt.x = self._camera_base_x + forward * c - left * s
        pt.y = self._camera_base_y + forward * s + left * c
        pt.z = self._camera_base_z + up
        return pt


def main(args=None):
    rclpy.init(args=args)
    node = CameraDetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
