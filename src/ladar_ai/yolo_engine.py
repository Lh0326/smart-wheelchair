"""YOLO 推理引擎：OpenVINO NPU 加速目标检测。
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


使用 YOLO11s OpenVINO IR 模型在 Intel NPU (AI Boost) 上推理（~76 FPS），
自动回退到 GPU → CPU。

设备优先级：NPU → GPU → CPU。
"""
import os
import logging
import threading
from typing import List, Dict, Optional

try:
    import cv2
except ImportError:
    cv2 = None

try:
    import numpy as np
except ImportError:
    np = None

logger = logging.getLogger(__name__)

# COCO 80 类名
COCO_NAMES = [
    "person", "bicycle", "car", "motorcycle", "airplane",
    "bus", "train", "truck", "boat", "traffic_light",
    "fire_hydrant", "stop_sign", "parking_meter", "bench", "bird",
    "cat", "dog", "horse", "sheep", "cow",
    "elephant", "bear", "zebra", "giraffe", "backpack",
    "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports_ball", "kite", "baseball_bat",
    "baseball_glove", "skateboard", "surfboard", "tennis_racket", "bottle",
    "wine_glass", "cup", "fork", "knife", "spoon",
    "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot_dog", "pizza", "donut",
    "cake", "chair", "couch", "potted_plant", "bed",
    "dining_table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell_phone", "microwave", "oven",
    "toaster", "sink", "refrigerator", "book", "clock",
    "vase", "scissors", "teddy_bear", "hair_drier", "toothbrush",
]

# 关注的目标类别 id -> 名称
TARGET_CLASSES: Dict[int, str] = {
    0: "person",
    1: "bicycle",
    2: "car",
    3: "motorcycle",
    5: "bus",
    7: "truck",
    9: "traffic_light",
}

# 各类别绘制颜色 (BGR)
_DRAW_COLORS = {
    "person": (0, 0, 255),
    "traffic_light": (0, 165, 255),
    "car": (0, 165, 255),
    "bus": (0, 165, 255),
    "truck": (0, 165, 255),
    "bicycle": (0, 255, 0),
    "motorcycle": (0, 255, 0),
}


# ==================== OpenVINO NPU 推理 ====================

class YOLODetectorOpenVINO:
    """OpenVINO 原生 YOLO 推理，优先 NPU，自动回退 GPU → CPU。

    使用 YOLO11s OpenVINO IR 模型。
    NPU (Intel AI Boost) 实测 ~76 FPS。
    """

    def __init__(self, model_path: str, device: str = "NPU", conf: float = 0.5):
        import openvino as ov

        self.conf = conf
        self.device = device
        self._lock = threading.Lock()
        self.model_path = model_path  # 当前加载的模型路径，供外部检查

        core = ov.Core()
        available = core.available_devices

        # 设备优先级回退：NPU → GPU → CPU
        if device not in available:
            for fb in ["NPU", "GPU", "CPU"]:
                if fb in available:
                    device = fb
                    break

        logger.info(f"YOLO OpenVINO: 请求设备={self.device}, 实际设备={device}")
        self.device = device

        # 编译时加入 LATENCY 性能提示（适合单帧低延迟场景）
        config = {"PERFORMANCE_HINT": "LATENCY"}
        self.compiled_model = core.compile_model(model_path, device, config)
        self.input_tensor = self.compiled_model.input(0)
        self.output_tensor = self.compiled_model.output(0)

        self.input_shape = self.input_tensor.shape
        self.input_h = self.input_shape[2]
        self.input_w = self.input_shape[3]

        # 同步推理 request（线程安全，多线程调用 detect 会通过 self._lock 串行化）
        self.infer_request = self.compiled_model.create_infer_request()

        self._pad_info = None
        logger.info(f"YOLO 模型加载成功: {model_path} @ {device}")

    def _preprocess(self, frame):
        """Letterbox 预处理：resize + pad + normalize + NCHW。"""
        h, w = frame.shape[:2]
        scale = min(self.input_w / w, self.input_h / h)
        new_w = int(w * scale)
        new_h = int(h * scale)

        resized = cv2.resize(frame, (new_w, new_h))

        pad_w = self.input_w - new_w
        pad_h = self.input_h - new_h
        top = pad_h // 2
        left = pad_w // 2

        padded = np.full((self.input_h, self.input_w, 3), 114, dtype=np.uint8)
        padded[top:top + new_h, left:left + new_w] = resized

        # HWC → CHW, BGR → RGB, normalize
        blob = padded.transpose(2, 0, 1)[::-1].astype(np.float32) / 255.0
        blob = blob[np.newaxis, ...]

        self._pad_info = (scale, top, left)
        return blob

    def _postprocess(self, output, frame_shape):
        """自动检测输出格式并解码。"""
        h, w = frame_shape[:2]
        scale, pad_top, pad_left = self._pad_info
        data = output[0]

        if data.ndim == 2 and data.shape[1] == 6:
            return self._postprocess_decoded(data, h, w, scale, pad_top, pad_left)
        else:
            return self._postprocess_raw(data, h, w, scale, pad_top, pad_left)

    def _postprocess_decoded(self, data, h, w, scale, pad_top, pad_left):
        """YOLO11 格式: [N, 6] = [x1, y1, x2, y2, conf, class_id]。"""
        mask = data[:, 4] >= self.conf
        filtered = data[mask]

        if len(filtered) == 0:
            return []

        x1 = ((filtered[:, 0] - pad_left) / scale).astype(int)
        y1 = ((filtered[:, 1] - pad_top) / scale).astype(int)
        x2 = ((filtered[:, 2] - pad_left) / scale).astype(int)
        y2 = ((filtered[:, 3] - pad_top) / scale).astype(int)

        x1 = np.clip(x1, 0, w)
        y1 = np.clip(y1, 0, h)
        x2 = np.clip(x2, 0, w)
        y2 = np.clip(y2, 0, h)

        size_mask = (x2 - x1 >= 2) & (y2 - y1 >= 2)
        x1, y1, x2, y2 = x1[size_mask], y1[size_mask], x2[size_mask], y2[size_mask]
        confs = filtered[size_mask, 4]
        cls_ids = filtered[size_mask, 5].astype(int)

        results = []
        for i in range(len(x1)):
            cid = int(cls_ids[i])
            if cid not in TARGET_CLASSES:
                continue
            results.append({
                "class_id": cid,
                "class_name": TARGET_CLASSES[cid],
                "confidence": float(confs[i]),
                "bbox": [int(x1[i]), int(y1[i]), int(x2[i]), int(y2[i])],
            })
        return results

    def _postprocess_raw(self, data, h, w, scale, pad_top, pad_left):
        """YOLOv8 格式: [84, 8400] 或 [8400, 84]，需要 NMS。"""
        # 转置为 [N, 84]
        if data.shape[0] == 4 + len(COCO_NAMES):
            data = data.T

        if data.shape[1] < 6:
            return []

        class_scores = data[:, 4:]
        cls_ids = np.argmax(class_scores, axis=1)
        confs = class_scores[np.arange(len(cls_ids)), cls_ids]
        mask = confs >= self.conf

        if not mask.any():
            return []

        filtered = data[mask]
        confs = confs[mask]
        cls_ids = cls_ids[mask]

        # cxcywh → xyxy
        cx = (filtered[:, 0] - pad_left) / scale
        cy = (filtered[:, 1] - pad_top) / scale
        bw = filtered[:, 2] / scale
        bh = filtered[:, 3] / scale

        x1 = np.clip((cx - bw / 2).astype(int), 0, w)
        y1 = np.clip((cy - bh / 2).astype(int), 0, h)
        x2 = np.clip((cx + bw / 2).astype(int), 0, w)
        y2 = np.clip((cy + bh / 2).astype(int), 0, h)

        boxes = np.stack([x1, y1, x2, y2], axis=1).astype(np.float32)
        indices = cv2.dnn.NMSBoxes(
            boxes.tolist(), confs.tolist(), self.conf, 0.45
        )

        # OpenCV 版本兼容：NMSBoxes 返回 flat array 或 nested array
        if isinstance(indices, np.ndarray):
            indices = indices.flatten().tolist()
        elif hasattr(indices, "reshape"):
            indices = indices.reshape(-1).tolist()

        results = []
        for i in indices:
            cid = int(cls_ids[i])
            if cid not in TARGET_CLASSES:
                continue
            bbox = boxes[i].astype(int).tolist()
            results.append({
                "class_id": cid,
                "class_name": TARGET_CLASSES[cid],
                "confidence": float(confs[i]),
                "bbox": bbox,
            })
        return results

    def detect(self, frame) -> List[Dict]:
        """推理：预处理 → 推理 → 后处理（线程安全）。

        当前实现：infer_request 在 self._lock 保护下串行推理，多线程调用
        detect 会自然排队。如需多帧并发，需要未来重构为 AsyncInferQueue
        的 start_async + wait 模式。
        """
        blob = self._preprocess(frame)
        frame_shape = frame.shape

        with self._lock:
            self.infer_request.infer({self.input_tensor: blob})
            output = self.infer_request.get_tensor(self.output_tensor).data

        return self._postprocess(output, frame_shape)


# ==================== Ultralytics 备选后端 ====================

class YOLODetectorUltralytics:
    """ultralytics PyTorch 后端（备选，CPU 开销大）。"""

    def __init__(self, model_path: str, conf: float = 0.5):
        from ultralytics import YOLO
        self.model = YOLO(model_path)
        self.model.conf = conf
        logger.info(f"YOLO ultralytics 后端加载: {model_path}")

    def detect(self, frame) -> List[Dict]:
        results = self.model(frame, verbose=False)
        detections = []
        for result in results:
            boxes = result.boxes
            if boxes is None:
                continue
            for box in boxes:
                cls_id = int(box.cls[0])
                if cls_id not in TARGET_CLASSES:
                    continue
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().tolist()
                detections.append({
                    "class_id": cls_id,
                    "class_name": TARGET_CLASSES[cls_id],
                    "confidence": float(box.conf[0]),
                    "bbox": [int(x1), int(y1), int(x2), int(y2)],
                })
        return detections


# ==================== 工厂函数 ====================

def create_yolo_engine(model_path: str = "", conf: float = 0.5, device: str = "NPU"):
    """加载 YOLO 模型，优先 INT8 量化版 + NPU。

    优先级：
    1. YOLO11s INT8 OpenVINO IR（NPU 兼容 + 量化加速）
    2. YOLO11s FP16 OpenVINO IR（NPU 兼容）
    3. yolo26s.pt ultralytics PyTorch（兜底）
    """
    int8_xml = "" + _MODELS_ROOT + "/models/yolo/yolo11s_int8_openvino/yolo11s.xml"
    fp16_xml = "" + _MODELS_ROOT + "/models/yolo/yolo11s_openvino/yolo11s.xml"
    pt_model = "" + _MODELS_ROOT + "/models/yolo/yolo26s.pt"

    if not model_path:
        # 优先 INT8，其次 FP16，最后 PT
        if os.path.isfile(int8_xml):
            model_path = int8_xml
        elif os.path.isfile(fp16_xml):
            model_path = fp16_xml
        else:
            model_path = pt_model

    # .xml 模型：尝试 NPU → GPU → CPU
    if model_path.endswith(".xml"):
        for dev in [device, "NPU", "GPU", "CPU"]:
            try:
                return YOLODetectorOpenVINO(model_path, device=dev, conf=conf)
            except Exception as e:
                logger.warning(f"OpenVINO {dev} 加载失败: {e}")
        # 全部 OpenVINO 设备失败，回退 ultralytics
        logger.warning("所有 OpenVINO 设备失败，回退 ultralytics PyTorch")
        model_path = pt_model

    return YOLODetectorUltralytics(model_path, conf=conf)


def detect(model, frame, target_classes: Optional[Dict[int, str]] = None) -> List[Dict]:
    """执行 YOLO 检测（兼容两种后端）。"""
    if target_classes is not None:
        # 如果指定了 target_classes，过滤结果
        results = model.detect(frame)
        return [d for d in results if d["class_id"] in target_classes]
    return model.detect(frame)


def detect_traffic_light_color(frame, bbox: List[int]) -> str:
    """通过 HSV 颜色空间 + 亮度过滤检测红绿灯颜色。

    改进点：
    - 只看 bbox 中心 60% 区域，避开黑色边框/外壳干扰
    - 亮度 V 通道阈值 > 140，只统计"发光"的像素（红绿灯本身发光）
    - 加宽绿色阈值（35-95），适应不同色温
    - min_pixels=5 让小目标也能触发
    """
    if frame is None or cv2 is None:
        return ""

    x1, y1, x2, y2 = bbox
    h_img, w_img = frame.shape[:2]
    x1, y1 = max(0, x1), max(0, y1)
    x2, y2 = min(w_img, x2), min(h_img, y2)

    bw = x2 - x1
    bh = y2 - y1
    if bw < 4 or bh < 4:
        return ""

    # 中心 60% 区域（避开边框）
    cmx1 = int(x1 + bw * 0.20)
    cmy1 = int(y1 + bh * 0.20)
    cmx2 = int(x1 + bw * 0.80)
    cmy2 = int(y1 + bh * 0.80)
    roi = frame[cmy1:cmy2, cmx1:cmx2]
    if roi.size == 0:
        return ""

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    # 亮度过滤：只看 V > 140 的发光像素（过滤黑色边框和暗部）
    bright = hsv[:, :, 2] > 140

    # HSV 阈值
    mask_red1 = cv2.inRange(hsv, np.array([0, 80, 140]), np.array([10, 255, 255]))
    mask_red2 = cv2.inRange(hsv, np.array([165, 80, 140]), np.array([180, 255, 255]))
    mask_red = (cv2.bitwise_or(mask_red1, mask_red2) > 0) & bright
    mask_yellow = (cv2.inRange(hsv, np.array([20, 80, 140]), np.array([35, 255, 255])) > 0) & bright
    mask_green = (cv2.inRange(hsv, np.array([35, 80, 140]), np.array([95, 255, 255])) > 0) & bright

    red_count = int(np.sum(mask_red))
    yellow_count = int(np.sum(mask_yellow))
    green_count = int(np.sum(mask_green))

    min_pixels = 5
    # 取像素数最多的颜色（要求 ≥ min_pixels）
    counts = [("red", red_count), ("yellow", yellow_count), ("green", green_count)]
    counts.sort(key=lambda x: -x[1])
    if counts[0][1] >= min_pixels:
        return counts[0][0]

    return ""


def draw_detections(frame, detections: List[Dict]) -> None:
    """在图像上绘制检测框。"""
    if frame is None or cv2 is None:
        return

    for det in detections:
        class_name = det.get("class_name", "")
        bbox = det.get("bbox", [0, 0, 0, 0])
        conf = det.get("confidence", 0.0)
        color = _DRAW_COLORS.get(class_name, (255, 255, 255))

        x1, y1, x2, y2 = bbox
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

        label = f"{class_name} {conf:.2f}"
        distance = det.get("distance")
        if distance is not None and distance > 0:
            label += f" {distance:.1f}m"
        extra = det.get("extra_info", "")
        if extra:
            label += f" [{extra}]"

        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(frame, (x1, y1 - th - 6), (x1 + tw, y1), color, -1)
        cv2.putText(
            frame, label, (x1, y1 - 4),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1,
        )
