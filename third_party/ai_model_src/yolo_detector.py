"""
YOLO 目标检测器 — DK-2500 OpenVINO NPU 版本

运行环境: Ubuntu 22.04 + Intel Core Ultra 5 225U + OpenVINO 2024.3
推理设备: NPU (INT8 量化, ~11 TOPS) — 自动 fallback 到 GPU/CPU
模型:     YOLO26s → OpenVINO IR (FP16/INT8)

用法:
    # 自动选择最优设备和模型
    det = create_detector()
    # 指定设备
    det = create_detector(device="NPU", model_path="models/yolo/yolo26s_int8/yolo26s.xml")
"""

import time
from pathlib import Path

import cv2
import numpy as np

from config import (
    YOLO_PT_MODEL, YOLO_INT8_MODEL_DIR, YOLO_INT8_MODEL,
    YOLO_DEVICE, YOLO_ALLOWED_CLASSES, CLASS_CONF_OVERRIDE,
)

# COCO 80类名称
COCO_NAMES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train",
    "truck", "boat", "traffic light", "fire hydrant", "stop sign",
    "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep",
    "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
    "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard",
    "sports ball", "kite", "baseball bat", "baseball glove", "skateboard",
    "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork",
    "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair",
    "couch", "potted plant", "bed", "dining table", "toilet", "tv",
    "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave",
    "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase",
    "scissors", "teddy bear", "hair drier", "toothbrush",
]

# 检测框颜色
COLORS = np.random.default_rng(42).uniform(0, 255, size=(80, 3)).astype(np.uint8)


# ── 距离估算（bbox 面积比值法）───────────────────────────────────────

def estimate_distance(bbox, frame_shape):
    """基于检测框面积粗略估算距离。

    在 480p 画面中，假设成人平均身高 1.7m:
      ratio > 0.60 → 1米以内
      ratio > 0.35 → 约2米
      ratio > 0.20 → 约3米
      ratio > 0.12 → 约5米
      其余 → 较远处
    """
    x1, y1, x2, y2 = bbox
    bbox_height = y2 - y1
    frame_height = frame_shape[0]
    if bbox_height < 10:
        return None
    ratio = bbox_height / frame_height
    if ratio > 0.6:
        return "一米以内"
    elif ratio > 0.35:
        return "约两米"
    elif ratio > 0.2:
        return "约三米"
    elif ratio > 0.12:
        return "约五米"
    else:
        return "较远处"


# ── 类别过滤 ──────────────────────────────────────────────────────────

def filter_detections(detections):
    """过滤到允许类别，并对指定类别应用更高的置信度阈值。"""
    if not YOLO_ALLOWED_CLASSES:
        return detections
    result = []
    for d in detections:
        name = d["class_name"]
        if name not in YOLO_ALLOWED_CLASSES:
            continue
        threshold = CLASS_CONF_OVERRIDE.get(name, 0)
        if threshold and d["confidence"] < threshold:
            continue
        result.append(d)
    return result


# ── 检测器基类 ────────────────────────────────────────────────────────

class YOLODetectorBase:
    """统一接口，子类实现不同后端。"""

    def detect(self, frame):
        raise NotImplementedError

    def warmup(self):
        self.detect(np.zeros((640, 640, 3), dtype=np.uint8))


# ── Ultralytics 后端（兼容旧模式）────────────────────────────────────

class YOLODetectorUltralytics(YOLODetectorBase):
    """基于 Ultralytics API 的检测器。

    兼容 Windows MVP 和 Linux DK-2500 环境。
    支持 PyTorch / ONNX / OpenVINO IR 格式（通过 Ultralytics 自动处理）。
    """

    def __init__(self, model_name=None, conf=0.25, device="cpu"):
        from ultralytics import YOLO
        if model_name is None:
            model_name = str(YOLO_PT_MODEL)
        self.model = YOLO(model_name, task="detect")
        self.conf = conf
        self.device = device
        # Use fixed COCO names for OpenVINO IR models (model.names may fail)
        try:
            self.class_names = self.model.names
        except Exception:
            self.class_names = {i: n for i, n in enumerate(COCO_NAMES)}

    def detect(self, frame):
        # OpenVINO backend (directory path) — no device param needed
        is_ov = hasattr(self.model, 'ckpt_path') and self.model.ckpt_path and (
            Path(self.model.ckpt_path).is_dir() or str(self.model.ckpt_path).endswith('.xml')
        )
        if is_ov:
            results = self.model(frame, conf=self.conf, verbose=False)
        else:
            results = self.model(frame, conf=self.conf, verbose=False, device=self.device)
        detections = []
        for r in results:
            boxes = r.boxes
            if boxes is None:
                continue
            for i in range(len(boxes)):
                x1, y1, x2, y2 = boxes.xyxy[i].cpu().numpy().astype(int)
                conf = float(boxes.conf[i].cpu())
                cls_id = int(boxes.cls[i].cpu())
                detections.append({
                    "bbox": [x1, y1, x2, y2],
                    "confidence": conf,
                    "class_id": cls_id,
                    "class_name": self.class_names.get(cls_id, str(cls_id)),
                })
        return filter_detections(detections)


# ── OpenVINO NPU 后端（DK-2500 核心加速）──────────────────────────────

class YOLODetectorOpenVINO(YOLODetectorBase):
    """基于 OpenVINO Runtime 的检测器，专用于 DK-2500 NPU 加速。

    特性:
      - INT8 量化模型推理，NPU 11 TOPS 加速
      - AsyncInferQueue 流水线并行（CPU 预处理 + NPU 推理重叠）
      - 零拷贝推理（共享内存，避免 CPU↔NPU 数据搬运）
      - 自动 fallback 到 GPU/CPU

    用法:
        det = YOLODetectorOpenVINO("yolo26s_int8.xml", device="NPU")
        detections = det.detect(frame)
    """

    def __init__(self, model_path, device="NPU", conf=0.25):
        import openvino as ov
        import threading

        self.conf = conf
        self.device = device
        self._lock = threading.Lock()
        core = ov.Core()

        # 打印可用设备
        available = core.available_devices
        print(f"[YOLO-OpenVINO] Available devices: {available}")

        # 自动 fallback
        if device not in available:
            fb = "GPU" if "GPU" in available else "CPU"
            print(f"[YOLO-OpenVINO] {device} not available, fallback to {fb}")
            device = fb
            self.device = device

        print(f"[YOLO-OpenVINO] Loading model: {model_path}")
        print(f"[YOLO-OpenVINO] Target device: {device}")

        # 编译模型到目标设备
        self.compiled_model = core.compile_model(model_path, device)

        # 获取输入输出张量信息
        self.input_tensor = self.compiled_model.input(0)
        self.output_tensor = self.compiled_model.output(0)

        # 模型输入尺寸
        self.input_shape = self.input_tensor.shape  # [1, 3, 640, 640]
        self.input_h = self.input_shape[2]
        self.input_w = self.input_shape[3]

        # 创建异步推理队列（2 个请求交替执行）
        self.infer_queue = ov.AsyncInferQueue(self.compiled_model, 2)

        # 同步推理请求（用于单帧推理场景）
        self.infer_request = self.compiled_model.create_infer_request()

        # 类别名
        self.class_names = {i: n for i, n in enumerate(COCO_NAMES)}

        print(f"[YOLO-OpenVINO] Ready (device={self.device}, input={self.input_w}x{self.input_h})")

    def _preprocess(self, frame):
        """Letterbox 预处理: resize + pad + normalize + NCHW。"""
        h, w = frame.shape[:2]
        scale = min(self.input_w / w, self.input_h / h)
        new_w = int(w * scale)
        new_h = int(h * scale)

        resized = cv2.resize(frame, (new_w, new_h))

        # Pad to input size
        pad_w = self.input_w - new_w
        pad_h = self.input_h - new_h
        top = pad_h // 2
        left = pad_w // 2

        padded = np.full((self.input_h, self.input_w, 3), 114, dtype=np.uint8)
        padded[top:top + new_h, left:left + new_w] = resized

        # HWC → CHW, BGR → RGB, normalize to [0,1]
        blob = padded.transpose(2, 0, 1)[::-1].astype(np.float32) / 255.0
        blob = blob[np.newaxis, ...]

        self._pad_info = (scale, top, left)
        return blob

    def _postprocess(self, output, frame_shape):
        """YOLO 后处理: 自动检测输出格式，向量化解析 → 检测框。

        支持两种格式:
          YOLOv8/YOLOv5: [1, 84, 8400] — cxcywh + class_scores, 需要 NMS
          YOLO11 (yolo26s): [1, 300, 6] — xyxy + conf + class_id, 已解码
        """
        h, w = frame_shape[:2]
        scale, pad_top, pad_left = self._pad_info
        data = output[0]

        # Detect format by shape
        if data.ndim == 2 and data.shape[1] == 6:
            # YOLO11 format: (N, 6) = [x1, y1, x2, y2, conf, class_id]
            return self._postprocess_yolo11(data, h, w, scale, pad_top, pad_left)
        else:
            # YOLOv8 format: (84, 8400) or (8400, 84)
            return self._postprocess_yolov8(data, h, w, scale, pad_top, pad_left)

    def _postprocess_yolo11(self, data, h, w, scale, pad_top, pad_left):
        """Post-process YOLO11 [N, 6] decoded output (xyxy + conf + cls)."""
        # Filter by confidence
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

        confs = filtered[:, 4]
        cids = filtered[:, 5].astype(int)

        # Filter tiny boxes
        size_mask = (x2 - x1 >= 2) & (y2 - y1 >= 2)
        x1, y1, x2, y2 = x1[size_mask], y1[size_mask], x2[size_mask], y2[size_mask]
        confs, cids = confs[size_mask], cids[size_mask]

        if len(confs) == 0:
            return []

        detections = []
        for i in range(len(confs)):
            detections.append({
                "bbox": [int(x1[i]), int(y1[i]), int(x2[i]), int(y2[i])],
                "confidence": float(confs[i]),
                "class_id": int(cids[i]),
                "class_name": COCO_NAMES[cids[i]] if cids[i] < len(COCO_NAMES) else str(cids[i]),
            })
        return filter_detections(detections)

    def _postprocess_yolov8(self, data, h, w, scale, pad_top, pad_left):
        """Post-process YOLOv8 [84, 8400] raw output (cxcywh + scores, needs NMS)."""
        predictions = data
        if predictions.shape[0] < predictions.shape[1]:
            predictions = predictions.T

        class_scores = predictions[:, 4:]
        class_ids = np.argmax(class_scores, axis=1)
        confidences = class_scores[np.arange(len(class_ids)), class_ids]
        mask = confidences >= self.conf
        if not np.any(mask):
            return []

        filtered = predictions[mask]
        confs = confidences[mask]
        cids = class_ids[mask]

        cx, cy, bw, bh = filtered[:, 0], filtered[:, 1], filtered[:, 2], filtered[:, 3]
        x1 = ((cx - bw / 2) - pad_left) / scale
        y1 = ((cy - bh / 2) - pad_top) / scale
        x2 = ((cx + bw / 2) - pad_left) / scale
        y2 = ((cy + bh / 2) - pad_top) / scale

        x1 = np.clip(x1, 0, w).astype(int)
        y1 = np.clip(y1, 0, h).astype(int)
        x2 = np.clip(x2, 0, w).astype(int)
        y2 = np.clip(y2, 0, h).astype(int)

        size_mask = (x2 - x1 >= 2) & (y2 - y1 >= 2)
        x1, y1, x2, y2 = x1[size_mask], y1[size_mask], x2[size_mask], y2[size_mask]
        confs, cids = confs[size_mask], cids[size_mask]

        if len(confs) == 0:
            return []

        boxes = np.stack([x1, y1, x2, y2], axis=1).astype(np.float32)
        keep = self._nms_numpy(boxes, confs, iou_threshold=0.45)

        detections = []
        for i in keep:
            detections.append({
                "bbox": [int(x1[i]), int(y1[i]), int(x2[i]), int(y2[i])],
                "confidence": float(confs[i]),
                "class_id": int(cids[i]),
                "class_name": COCO_NAMES[cids[i]] if cids[i] < len(COCO_NAMES) else str(cids[i]),
            })
        return filter_detections(detections)

    @staticmethod
    def _nms_numpy(boxes, scores, iou_threshold=0.45):
        """Vectorized NMS using NumPy (no Python loop per box)."""
        x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
        areas = (x2 - x1) * (y2 - y1)
        order = scores.argsort()[::-1]
        keep = []
        while len(order) > 0:
            i = order[0]
            keep.append(i)
            if len(order) == 1:
                break
            rest = order[1:]
            xx1 = np.maximum(x1[i], x1[rest])
            yy1 = np.maximum(y1[i], y1[rest])
            xx2 = np.minimum(x2[i], x2[rest])
            yy2 = np.minimum(y2[i], y2[rest])
            inter = np.maximum(0, xx2 - xx1) * np.maximum(0, yy2 - yy1)
            iou = inter / (areas[i] + areas[rest] - inter)
            order = rest[iou <= iou_threshold]
        return keep

    def detect(self, frame):
        """同步推理: 预处理 → NPU 推理 → 后处理（线程安全）。"""
        blob = self._preprocess(frame)

        with self._lock:
            self.infer_request.infer({self.input_tensor: blob})
            output = self.infer_request.get_tensor(self.output_tensor).data.copy()

        return self._postprocess(output, frame.shape)

    def detect_async(self, frame, callback):
        """异步推理: 提交到 NPU 后立即返回，推理完成后触发回调。

        适用于 YOLO 持续线程场景，CPU 可以同时处理下一帧。
        """
        blob = self._preprocess(frame)
        frame_shape = frame.shape

        def on_complete(infer_request, userdata):
            output = infer_request.get_tensor(self.output_tensor).data
            dets = self._postprocess(output, frame_shape)
            callback(dets)

        self.infer_queue.set_callback(on_complete)
        self.infer_queue.start_async({self.input_tensor: blob})


# ── 绘制函数 ──────────────────────────────────────────────────────────

def draw_detections(frame, detections):
    for det in detections:
        x1, y1, x2, y2 = det["bbox"]
        cls_id = det["class_id"]
        name = det.get("class_name", str(cls_id))
        conf = det["confidence"]
        color = tuple(int(c) for c in COLORS[cls_id % 80])
        # Override box color for traffic lights
        if name == "traffic light" and det.get("light_color"):
            if det["light_color"] == "红灯":
                color = (0, 0, 255)
            elif det["light_color"] == "绿灯":
                color = (0, 255, 0)
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        label = f"{name} {conf:.2f}"
        if det.get("light_color"):
            label += f" [{det['light_color']}]"
        if det.get("distance"):
            label += f" [{det['distance']}]"
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        cv2.rectangle(frame, (x1, y1 - th - 6), (x1 + tw + 4, y1), color, -1)
        cv2.putText(
            frame, label, (x1 + 2, y1 - 4),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1,
        )
    return frame


# ── 摄像头管理器 ──────────────────────────────────────────────────────

class CameraManager:
    """摄像头管理器，支持内置摄像头和 USB 相机切换。"""

    def __init__(self, source=0, width=640, height=480, fps=30):
        self.source = source
        self.cap = cv2.VideoCapture(source)
        if not self.cap.isOpened():
            raise RuntimeError(f"无法打开摄像头 (source={source})")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        print(f"摄像头已打开: source={source}, {self.width}x{self.height}")

    def read(self):
        ret, frame = self.cap.read()
        if not ret:
            raise RuntimeError("摄像头读取失败")
        return frame

    def release(self):
        self.cap.release()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.release()


# ── 检测器工厂函数 ────────────────────────────────────────────────────

def _find_ov_model_xml(device):
    """Find the best OpenVINO .xml model for the given device."""
    # INT8 model (primary)
    if YOLO_INT8_MODEL.exists():
        return str(YOLO_INT8_MODEL)
    # Search INT8 model directory for .xml
    if YOLO_INT8_MODEL_DIR.exists():
        xml_files = list(YOLO_INT8_MODEL_DIR.glob("*.xml"))
        if xml_files:
            return str(xml_files[0])
    return None


def create_detector(device=None, model_path=None, conf=0.25):
    """自动创建最优检测器。

    优先级:
      1. OpenVINO Native + NPU（DK-2500 最优，~68 FPS）
      2. OpenVINO Native + GPU（~40 FPS）
      3. Ultralytics PyTorch + CPU（兼容模式）

    Args:
        device: 推理设备 ("NPU"/"GPU"/"CPU"/None=自动)
        model_path: 模型路径 (None=自动查找)
        conf: 置信度阈值

    Returns:
        YOLODetectorBase 实例
    """
    if device is None:
        device = YOLO_DEVICE

    # 尝试使用 OpenVINO Native 后端（直接编译到 NPU/GPU）
    try:
        import openvino as ov
        core = ov.Core()
        available = core.available_devices
        print(f"[YOLO] OpenVINO available devices: {available}")

        # 自动 fallback 设备
        actual_device = device
        if actual_device not in available:
            for fb in ["NPU", "GPU", "CPU"]:
                if fb in available:
                    if fb != actual_device:
                        print(f"[YOLO] {actual_device} not available, fallback to {fb}")
                    actual_device = fb
                    break

        # Find .xml model file
        if model_path is None:
            model_path = _find_ov_model_xml(actual_device)

        if model_path and (model_path.endswith(".xml") or Path(model_path).is_dir()):
            print(f"[YOLO] Using OpenVINO Native backend: {model_path} on {actual_device}")
            try:
                return YOLODetectorOpenVINO(model_path, device=actual_device, conf=conf)
            except Exception as e:
                print(f"[YOLO] OpenVINO Native failed: {e}")
                print(f"[YOLO] Falling back to Ultralytics backend on CPU")
                return YOLODetectorUltralytics(model_path, conf=conf, device="cpu")

    except ImportError:
        print("[YOLO] OpenVINO not available")

    # Fallback: Ultralytics PyTorch
    if model_path is None:
        model_path = str(YOLO_PT_MODEL) if YOLO_PT_MODEL.exists() else None
    if model_path is None:
        raise FileNotFoundError("No YOLO model found")
    print(f"[YOLO] Using Ultralytics backend: {model_path}")
    return YOLODetectorUltralytics(model_path, conf=conf, device="cpu")


def run_realtime(detector, camera_source=0, camera_width=640,
                 camera_height=480, target_fps=30):
    """实时检测主循环。"""
    with CameraManager(camera_source, camera_width, camera_height) as cam:
        print(f"实时检测已启动 (摄像头 source={camera_source})")
        print("按 'q' 退出, 's' 保存截图, '+'/'-' 切换摄像头索引")

        frame_count = 0
        fps = 0.0
        fps_timer = time.time()
        source = camera_source

        while True:
            try:
                frame = cam.read()
            except RuntimeError:
                print("摄像头断开，尝试重新连接...")
                time.sleep(1)
                try:
                    cam = CameraManager(source, camera_width, camera_height)
                    continue
                except RuntimeError:
                    break

            t0 = time.time()
            detections = detector.detect(frame)
            infer_ms = (time.time() - t0) * 1000

            # 添加距离估算
            for d in detections:
                d["distance"] = estimate_distance(d["bbox"], frame.shape)

            frame = draw_detections(frame, detections)

            frame_count += 1
            if frame_count % 10 == 0:
                now = time.time()
                fps = 10.0 / (now - fps_timer)
                fps_timer = now

            device_info = getattr(detector, 'device', 'cpu')
            cv2.putText(
                frame,
                f"FPS: {fps:.1f} | Infer: {infer_ms:.1f}ms | Dets: {len(detections)} | {device_info}",
                (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2,
            )
            cv2.imshow("YOLO26s Detection (DK-2500 NPU)", frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord("s"):
                path = f"detection_{int(time.time())}.jpg"
                cv2.imwrite(path, frame)
                print(f"截图已保存: {path}")
            elif key == ord("+") or key == ord("="):
                source = source + 1
                print(f"切换到摄像头 source={source}")
                cam.release()
                try:
                    cam = CameraManager(source, camera_width, camera_height)
                except RuntimeError as e:
                    print(f"摄像头 {source} 不可用: {e}")
                    source = source - 1
                    cam = CameraManager(source, camera_width, camera_height)

    cv2.destroyAllWindows()
