"""相机检测节点：Orbbec Gemini 335L RGB + 深度 -> YOLO 检测 -> 发布结果。

通过 ROS2 话题获取 Orbbec 摄像头的 RGB 和深度图像，
定时执行 YOLO 检测并发布结果到 /front_detections。

距离估算策略：
  - 有深度图：取 bbox 中心 ROI 的深度中值（Orbbec 输出毫米，转米）
  - 无深度图：基于 bbox 高度比粗略估算
  - 红绿灯：不需要距离信息
"""
import threading
import logging
import signal

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None

try:
    import cv2
except ImportError:
    cv2 = None

try:
    import numpy as np
except ImportError:
    np = None

from ladar_ai.yolo_engine import (
    create_yolo_engine,
    detect,
    detect_traffic_light_color,
    draw_detections,
)

logger = logging.getLogger(__name__)


def estimate_distance_from_depth(bbox, depth_image, roi_ratio=0.4):
    """从深度图取 bbox 中心区域的稳健距离。

    Orbbec 常见输出为 uint16 毫米；若收到 32FC1 浮点图，则按米处理。
    """
    if depth_image is None or np is None:
        return -1.0

    x1, y1, x2, y2 = bbox
    h, w = depth_image.shape[:2]
    x1, y1 = max(0, x1), max(0, y1)
    x2, y2 = min(w, x2), min(h, y2)

    bw = x2 - x1
    bh = y2 - y1
    if bw <= 0 or bh <= 0:
        return -1.0

    # 中心 ROI 避免 bbox 边缘混入背景；后续用分位数抵抗孔洞和远背景。
    cx1 = int(x1 + bw * (0.5 - roi_ratio / 2))
    cy1 = int(y1 + bh * (0.5 - roi_ratio / 2))
    cx2 = int(x1 + bw * (0.5 + roi_ratio / 2))
    cy2 = int(y1 + bh * (0.5 + roi_ratio / 2))

    roi = depth_image[cy1:cy2, cx1:cx2]
    if roi.size == 0:
        return -1.0

    values = roi.astype(np.float32)
    finite = values[np.isfinite(values)]
    if finite.size == 0:
        return -1.0

    median_raw = float(np.median(finite))
    if median_raw > 100.0:
        values = values / 1000.0

    valid = values[np.isfinite(values)]
    valid = valid[(valid >= 0.15) & (valid <= 20.0)]
    if valid.size < 8:
        return -1.0

    return float(np.percentile(valid, 35))


def estimate_distance_from_bbox(bbox, frame_shape):
    """基于检测框高度比粗略估算距离（与 ai-model 一致）。

    在 720p 画面中，假设成人平均身高 1.7m。
    """
    if frame_shape is None:
        return -1.0

    x1, y1, x2, y2 = bbox
    bbox_height = y2 - y1
    frame_height = frame_shape[0]
    if bbox_height < 10:
        return -1.0

    ratio = bbox_height / frame_height
    if ratio > 0.6:
        return 0.8
    elif ratio > 0.35:
        return 2.0
    elif ratio > 0.2:
        return 3.5
    elif ratio > 0.12:
        return 5.5
    else:
        return 8.0


def is_pedestrian_traffic_light(frame, bbox):
    """保守判断是否更像人行道红绿灯。

    当前 YOLO 只有 COCO traffic_light 类，无法直接区分车行灯/人行灯。
    这里仅对更像人行灯的垂直、小型、靠画面侧边或中低位置目标返回 True。
    """
    if frame is None:
        return False

    x1, y1, x2, y2 = bbox
    h, w = frame.shape[:2]
    bw = max(1, x2 - x1)
    bh = max(1, y2 - y1)
    cx = (x1 + x2) / 2.0 / max(1, w)
    cy = (y1 + y2) / 2.0 / max(1, h)
    aspect = bh / bw
    area_ratio = (bw * bh) / max(1, w * h)

    vertical = aspect >= 1.15
    useful_size = area_ratio >= 0.0005 and bh >= h * 0.035
    not_far_top = cy >= 0.12
    sidewalk_position = cx <= 0.38 or cx >= 0.62 or cy >= 0.35
    return vertical and useful_size and not_far_top and sidewalk_position


class CameraDetectNode(Node):
    """订阅 Orbbec 摄像头 RGB 和深度图，定时执行 YOLO 检测并发布结果。"""

    def __init__(self):
        super().__init__("camera_detect_node")

        from ladar_ai.msg import Detection as DetectionMsg, Detections as DetectionsMsg

        # ---------- 参数 ----------
        self.declare_parameter("model_path", "")
        self.declare_parameter("conf", 0.35)
        self.declare_parameter("fps", 15.0)

        model_path = self.get_parameter("model_path").value
        conf = self.get_parameter("conf").value
        fps = self.get_parameter("fps").value

        # ---------- 加载模型 ----------
        try:
            self._model = create_yolo_engine(model_path, conf=conf)
        except Exception as e:
            logger.error(f"YOLO 模型加载失败: {e}")
            self._model = None

        # ---------- CvBridge ----------
        if CvBridge is not None:
            self._bridge = CvBridge()
        else:
            self._bridge = None
            logger.warning("cv_bridge 未安装")

        # ---------- 帧缓存 ----------
        self._rgb_frame = None
        self._depth_image = None
        self._frame_lock = threading.Lock()
        self._processing = False

        # ---------- 发布器 ----------
        self._pub_detections = self.create_publisher(DetectionsMsg, "/front_detections", 10)
        self._pub_annotated = self.create_publisher(Image, "/annotated_image", 10)

        # ---------- 订阅 ----------
        self._sub_rgb = self.create_subscription(
            Image, "/camera/color/image_raw", self._rgb_callback, 10
        )
        self._sub_depth = self.create_subscription(
            Image, "/camera/depth/image_raw", self._depth_callback, 10
        )

        # ---------- 定时器 ----------
        interval = 1.0 / fps if fps > 0 else 0.1
        self._timer = self.create_timer(interval, self._timer_callback)

        self.get_logger().info(f"CameraDetectNode started ({fps:.1f} fps)")

    def _rgb_callback(self, msg: Image):
        if self._bridge is None:
            return
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            with self._frame_lock:
                self._rgb_frame = frame
                self._rgb_header = msg.header
        except Exception as e:
            logger.warning("RGB image conversion failed: %s", e)

    def _depth_callback(self, msg: Image):
        if self._bridge is None:
            return
        try:
            # Orbbec 深度图为 16-bit (mono16 或 Y16)
            depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            with self._frame_lock:
                self._depth_image = depth
        except Exception as e:
            logger.warning("Depth image conversion failed: %s", e)

    def _timer_callback(self):
        if self._model is None or self._bridge is None:
            return

        if self._processing:
            return
        self._processing = True

        try:
            with self._frame_lock:
                if self._rgb_frame is None:
                    return
                frame = self._rgb_frame.copy()
                depth = self._depth_image.copy() if self._depth_image is not None else None
                header = self._rgb_header

            # YOLO 检测
            detections = detect(self._model, frame)

            # 为每个检测估算距离和红绿灯颜色
            for det in detections:
                bbox = det["bbox"]

                if det["class_name"] == "traffic_light":
                    color = detect_traffic_light_color(frame, bbox)
                    if color and is_pedestrian_traffic_light(frame, bbox):
                        det["extra_info"] = f"ped_{color}"
                    else:
                        det["extra_info"] = f"road_{color}" if color else "road"

                # 有深度图时优先用真实深度；无效时，非红绿灯再退回 bbox 粗估。
                dist = estimate_distance_from_depth(bbox, depth) if depth is not None else -1.0
                if dist > 0:
                    det["distance"] = dist
                elif det["class_name"] != "traffic_light":
                    det["distance"] = estimate_distance_from_bbox(bbox, frame.shape)
                else:
                    det["distance"] = -1.0

            # 绘制标注图（使用 UMat GPU 加速）
            use_umat = cv2.ocl.haveOpenCL() and cv2.ocl.useOpenCL()
            if use_umat:
                annotated = cv2.UMat(frame)
            else:
                annotated = frame.copy()
            draw_detections(annotated, detections)

            # 发布 Detections
            from ladar_ai.msg import Detection as DetectionMsg, Detections as DetectionsMsg

            det_msg = DetectionsMsg()
            det_msg.header = header
            for d in detections:
                dm = DetectionMsg()
                dm.class_name = d["class_name"]
                dm.confidence = float(d["confidence"])
                dm.distance = float(d.get("distance", -1.0))
                dm.extra_info = d.get("extra_info", "")
                det_msg.detections.append(dm)

            self._pub_detections.publish(det_msg)

            # 发布标注图
            try:
                if isinstance(annotated, cv2.UMat):
                    annotated = annotated.get()
                img_msg = self._bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
                img_msg.header = header
                self._pub_annotated.publish(img_msg)
            except Exception as e:
                logger.warning("Annotated image publish failed: %s", e)
        finally:
            self._processing = False


def main(args=None):
    rclpy.init(args=args)
    node = CameraDetectNode()
    signal.signal(signal.SIGINT, lambda *_: rclpy.shutdown())
    signal.signal(signal.SIGTERM, lambda *_: rclpy.shutdown())
    try:
        rclpy.spin(node)
    except Exception:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
