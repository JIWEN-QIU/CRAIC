#!/usr/bin/env python3
"""
两段式 YOLO 推理 ROS 节点
检测器 + 分类器 on N150 CPU (ONNX 推理)
"""
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import logging
import os
import threading
from typing import Any, cast
from pathlib import Path
from ultralytics import YOLO

# 配置日志
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def get_class_name(names, class_id):
    if isinstance(names, dict):
        return names.get(class_id, f"unknown_{class_id}")
    if isinstance(names, (list, tuple)) and 0 <= class_id < len(names):
        return str(names[class_id])
    return f"unknown_{class_id}"


def ensure_bgr(image):
    if image is None:
        return image
    if image.ndim == 2:
        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    return image


def clamp_int(value, minimum, maximum):
    return max(minimum, min(maximum, int(value)))


def compute_center_roi_box(frame_shape, roi_ratio):
    height, width = frame_shape[:2]
    side = max(1, int(min(height, width) * float(roi_ratio)))
    half_side = side // 2
    center_x = width // 2
    center_y = height // 2

    left = clamp_int(center_x - half_side, 0, width - 1)
    top = clamp_int(center_y - half_side, 0, height - 1)
    right = clamp_int(left + side, left + 1, width)
    bottom = clamp_int(top + side, top + 1, height)
    return left, top, right, bottom


def point_in_box(x, y, box_xyxy):
    left, top, right, bottom = box_xyxy
    return left <= x <= right and top <= y <= bottom


def smooth_frame_for_inference(frame):
    if frame is None or frame.size == 0:
        return frame

    smoothed = cv2.bilateralFilter(frame, d=7, sigmaColor=55, sigmaSpace=55)
    smoothed = cv2.GaussianBlur(smoothed, (3, 3), 0)
    return smoothed


def compute_border_white_ratio(image):
    if image is None or image.size == 0:
        return 1.0

    image_bgr = ensure_bgr(image)
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    value = hsv[:, :, 2]
    saturation = hsv[:, :, 1]
    white_mask = ((value >= 235) & (saturation <= 45)).astype(np.uint8)

    height, width = white_mask.shape[:2]
    border = max(2, int(min(height, width) * 0.10))
    border = min(border, max(2, min(height, width) // 2))

    border_mask = np.zeros_like(white_mask, dtype=bool)
    border_mask[:border, :] = True
    border_mask[-border:, :] = True
    border_mask[:, :border] = True
    border_mask[:, -border:] = True

    border_white = white_mask[border_mask]
    if border_white.size == 0:
        return 1.0
    return float(border_white.mean())


def refine_crop_from_box(frame, box_xyxy, shrink_ratio=0.90, pad_ratio=0.02, min_size=24,
                         white_threshold=0.60, max_refine_steps=3):
    height, width = frame.shape[:2]
    x1, y1, x2, y2 = box_xyxy

    box_w = max(1.0, float(x2 - x1))
    box_h = max(1.0, float(y2 - y1))
    center_x = x1 + box_w / 2.0
    center_y = y1 + box_h / 2.0

    current_w = max(float(min_size), box_w * float(shrink_ratio))
    current_h = max(float(min_size), box_h * float(shrink_ratio))

    def build_box(region_w, region_h):
        pad_w = min(region_w * float(pad_ratio), region_w * 0.15)
        pad_h = min(region_h * float(pad_ratio), region_h * 0.15)

        left = clamp_int(round(center_x - region_w / 2.0 - pad_w), 0, width - 1)
        top = clamp_int(round(center_y - region_h / 2.0 - pad_h), 0, height - 1)
        right = clamp_int(round(center_x + region_w / 2.0 + pad_w), left + 1, width)
        bottom = clamp_int(round(center_y + region_h / 2.0 + pad_h), top + 1, height)
        return left, top, right, bottom

    refined_box = build_box(current_w, current_h)
    crop = frame[refined_box[1]:refined_box[3], refined_box[0]:refined_box[2]]
    white_ratio = compute_border_white_ratio(crop)

    refine_step = 0
    while white_ratio > white_threshold and refine_step < max_refine_steps:
        current_w *= 0.92
        current_h *= 0.92
        if current_w < min_size or current_h < min_size:
            break

        refined_box = build_box(current_w, current_h)
        crop = frame[refined_box[1]:refined_box[3], refined_box[0]:refined_box[2]]
        if crop.size == 0:
            break
        white_ratio = compute_border_white_ratio(crop)
        refine_step += 1

    return crop, refined_box, white_ratio


def extract_topk_predictions(probs, names, top_k=5):
    if probs is None:
        return -1, 0.0, []

    top1 = int(getattr(probs, "top1", -1))
    top1_conf = float(getattr(probs, "top1conf", 0.0))

    top5_ids = list(getattr(probs, "top5", []))[:top_k]
    top5_confs = None
    if hasattr(probs, "top5conf") and probs.top5conf is not None:
        top5_confs = np.array(probs.top5conf.cpu().numpy() if hasattr(probs.top5conf, "cpu") else probs.top5conf).reshape(-1)

    if not top5_ids and hasattr(probs, "data") and probs.data is not None:
        scores = probs.data.cpu().numpy() if hasattr(probs.data, "cpu") else np.asarray(probs.data)
        top_indices = np.argsort(scores)[::-1][:top_k]
        top5_ids = [int(idx) for idx in top_indices]
        top5_confs = scores[top_indices]

    top5 = []
    for idx, class_id in enumerate(top5_ids):
        if top5_confs is not None and idx < len(top5_confs):
            confidence = float(top5_confs[idx])
        elif hasattr(probs, "data") and probs.data is not None:
            scores = probs.data.cpu().numpy() if hasattr(probs.data, "cpu") else np.asarray(probs.data)
            confidence = float(scores[int(class_id)])
        else:
            confidence = 0.0
        top5.append({
            "class_id": int(class_id),
            "class_name": get_class_name(names, int(class_id)),
            "conf": confidence,
        })

    return top1, top1_conf, top5


def build_classification_preview(crop, class_name, class_conf, det_conf, top5, white_ratio, threshold):
    preview = ensure_bgr(crop).copy()
    if preview.size == 0:
        preview = np.zeros((240, 320, 3), dtype=np.uint8)

    max_width = 420
    height, width = preview.shape[:2]
    if width > 0 and height > 0:
        scale = min(max_width / float(width), 1.0)
        if scale != 1.0:
            preview = cv2.resize(preview, (max(1, int(width * scale)), max(1, int(height * scale))))

    top_lines = [
        f"class: {class_name}",
        f"class_conf: {class_conf:.3f}  threshold: {threshold:.3f}",
        f"det_conf: {det_conf:.3f}",
        f"white_border_ratio: {white_ratio:.3f}",
        "top5:",
    ]
    for rank, item in enumerate(top5, start=1):
        top_lines.append(f"{rank}. {item['class_name']} {item['conf']:.3f}")

    line_height = 24
    panel_width = max(preview.shape[1], 420)
    panel_height = preview.shape[0] + 20 + line_height * len(top_lines) + 10
    panel = np.zeros((panel_height, panel_width, 3), dtype=np.uint8)
    panel[:preview.shape[0], :preview.shape[1]] = preview
    cv2.rectangle(panel, (0, preview.shape[0]), (panel_width - 1, panel_height - 1), (30, 30, 30), -1)

    text_y = preview.shape[0] + 28
    for line in top_lines:
        cv2.putText(panel, line, (12, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        text_y += line_height

    return panel


def build_main_display_panel(frame, frame_count, frame_mean, last_result_text, detection_count, roi_box=None):
    preview = ensure_bgr(frame).copy()
    if preview.size == 0:
        preview = np.zeros((480, 640, 3), dtype=np.uint8)

    max_width = 960
    height, width = preview.shape[:2]
    if width > 0 and height > 0:
        scale = min(max_width / float(width), 1.0)
        if scale != 1.0:
            preview = cv2.resize(preview, (max(1, int(width * scale)), max(1, int(height * scale))))

    preview_height, preview_width = preview.shape[:2]
    header_height = 88
    footer_height = 64
    side_padding = 18
    panel_width = preview_width + side_padding * 2
    panel_height = header_height + preview_height + footer_height + 20
    panel = np.full((panel_height, panel_width, 3), 245, dtype=np.uint8)

    warning = frame_mean < 8.0
    header_color = (25, 25, 180) if warning else (35, 120, 35)
    cv2.rectangle(panel, (0, 0), (panel_width - 1, header_height - 1), header_color, -1)
    cv2.rectangle(panel, (side_padding, header_height), (side_padding + preview_width - 1, header_height + preview_height - 1), (30, 30, 30), 2)

    panel[header_height:header_height + preview_height, side_padding:side_padding + preview_width] = preview

    if roi_box is not None:
        roi_left, roi_top, roi_right, roi_bottom = roi_box
        roi_left += side_padding
        roi_right += side_padding
        roi_top += header_height
        roi_bottom += header_height
        cv2.rectangle(panel, (roi_left, roi_top), (roi_right, roi_bottom), (0, 0, 255), 2)
        cv2.putText(panel, "ROI", (roi_left, max(roi_top - 8, 18)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    title = "two_stage_infer"
    cv2.putText(panel, title, (18, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.95, (255, 255, 255), 3)
    cv2.putText(panel, f"frame={frame_count}", (18, 64), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
    cv2.putText(panel, f"mean={frame_mean:.1f} detections={detection_count}", (220, 64), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
    status_text = "INPUT DARK" if warning else "INPUT OK"
    status_color = (0, 255, 255) if warning else (255, 255, 255)
    cv2.putText(panel, status_text, (520, 64), cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 3)

    footer_y = header_height + preview_height + 28
    cv2.rectangle(panel, (0, header_height + preview_height + 6), (panel_width - 1, panel_height - 1), (230, 230, 230), -1)
    if last_result_text:
        cv2.putText(panel, last_result_text[:110], (18, footer_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (20, 20, 20), 2)

    if warning:
        cv2.putText(panel, "WARNING: input frame is near black", (18, footer_y + 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 200), 2)

    return panel


def maybe_publish_debug_artifacts(debug_dir, frame_name, det_index, crop, preview, metadata):
    if not debug_dir:
        return

    debug_dir = Path(debug_dir)
    crop_dir = debug_dir / "crops"
    preview_dir = debug_dir / "previews"
    meta_dir = debug_dir / "meta"
    crop_dir.mkdir(parents=True, exist_ok=True)
    preview_dir.mkdir(parents=True, exist_ok=True)
    meta_dir.mkdir(parents=True, exist_ok=True)

    stem = f"{frame_name}_det{det_index:02d}"
    crop_path = crop_dir / f"{stem}.jpg"
    preview_path = preview_dir / f"{stem}.jpg"
    meta_path = meta_dir / f"{stem}.json"

    cv2.imwrite(str(crop_path), ensure_bgr(crop))
    cv2.imwrite(str(preview_path), ensure_bgr(preview))
    with meta_path.open("w", encoding="utf-8") as handle:
        json.dump(metadata, handle, ensure_ascii=False, indent=2)

class TwoStageInferenceNode:
    def __init__(self):
        # 初始化属性默认值，避免因异常导致未定义
        self.detector = None
        self.classifier = None
        self.bridge: Any = None
        self.image_sub = None
        self.result_pub: Any = None
        self.coords_pub: Any = None
        self.debug_pub: Any = None
        self.status_timer = None
        self.frame_count = 0
        self.last_detections = 0
        self.last_result_text = ""
        self.show_window = False
        self.window_name = "two_stage_infer"
        self.cifar_window_name = "cifar_crop_result"
        self.conf_threshold = 0.25
        self.class_conf_threshold = 0.50
        self.crop_shrink = 0.90
        self.crop_pad = 0.02
        self.white_threshold = 0.60
        self.enable_center_roi = False
        self.center_roi_ratio = 0.40
        self.device = "cpu"
        self.debug_dir = None
        self.max_fps = 3.0
        self.last_process_time = rospy.Time(0)
        self.skipped_frames = 0
        self.gui_lock = threading.Lock()
        self.latest_main_panel = None
        self.latest_crop_panel = None
        self.latest_gui_frame_count = 0
        self.latest_gui_error = ""
        
        rospy.init_node('two_stage_infer_node', anonymous=False)
        
        # 从 ROS 参数服务器读取参数
        det_model_path = rospy.get_param(
            '~det_model_path',
            '/home/n150/CRAIC_ws/src/camera/uav_deployment/models/target_detector.onnx'
        )
        cls_model_path = rospy.get_param(
            '~cls_model_path',
            '/home/n150/CRAIC_ws/src/camera/uav_deployment/models/cifar100_classifier.onnx'
        )
        det_model_path = str(det_model_path)
        cls_model_path = str(cls_model_path)
        self.conf_threshold = float(cast(Any, rospy.get_param('~conf_threshold', 0.25)))
        self.class_conf_threshold = float(cast(Any, rospy.get_param('~class_conf_threshold', 0.50)))
        self.crop_shrink = float(cast(Any, rospy.get_param('~crop_shrink', 0.90)))
        self.crop_pad = float(cast(Any, rospy.get_param('~crop_pad', 0.02)))
        self.white_threshold = float(cast(Any, rospy.get_param('~white_threshold', 0.60)))
        self.enable_center_roi = bool(rospy.get_param('~enable_center_roi', False))
        self.center_roi_ratio = float(cast(Any, rospy.get_param('~center_roi_ratio', 0.40)))
        self.device = str(rospy.get_param('~device', 'cpu'))
        self.show_window = bool(rospy.get_param('~show_window', False))
        self.window_name = str(rospy.get_param('~window_name', 'two_stage_infer'))
        self.cifar_window_name = str(rospy.get_param('~cifar_window_name', 'cifar_crop_result'))
        self.debug_dir = rospy.get_param('~debug_dir', '')
        self.max_fps = float(cast(Any, rospy.get_param('~max_fps', 3.0)))
        camera_topic = str(rospy.get_param('~camera_topic', '/usb_cam/image_raw'))
        
        # 只要 show_window=true 就尝试创建窗口；如果底层创建失败，再退回关闭
        if self.show_window:
            if not os.environ.get('DISPLAY'):
                logger.warning("DISPLAY is not set, but attempting to create GUI windows anyway")
            try:
                cv2.startWindowThread()
                cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
                cv2.namedWindow(self.cifar_window_name, cv2.WINDOW_NORMAL)
                cv2.resizeWindow(self.window_name, 960, 720)
                cv2.resizeWindow(self.cifar_window_name, 420, 420)
            except Exception as e:
                logger.warning(f"Failed to create window: {e}, disabling GUI")
                self.show_window = False
        
        logger.info(f"Loading detector from {det_model_path}")
        logger.info(f"Loading classifier from {cls_model_path}")
        
        try:
            # 直接加载 ONNX 模型文件
            self.detector = YOLO(det_model_path, task='detect')
            self.classifier = YOLO(cls_model_path, task='classify')
            logger.info("Models loaded successfully!")
        except Exception as e:
            logger.error(f"Failed to load models: {e}")
            import traceback
            logger.error(traceback.format_exc())
            rospy.signal_shutdown("Model loading failed")
            return
        
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            camera_topic,
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2 ** 24,
        )
        self.result_pub = rospy.Publisher('~target_info', String, queue_size=10)
        self.coords_pub = rospy.Publisher('~target_coords', Float32MultiArray, queue_size=10)
        self.debug_pub = rospy.Publisher('~debug_image', Image, queue_size=1)
        self.status_timer = rospy.Timer(rospy.Duration(2), self.print_status)
        
        self.logged_image_encoding = False
        
        logger.info(f"Node initialized. Listening on {camera_topic}, max_fps={self.max_fps}")
    
    def image_callback(self, msg):
        if self.detector is None or self.classifier is None:
            return

        if self.should_skip_frame():
            self.skipped_frames += 1
            return
        
        try:
            cv_img = self.ros_image_to_bgr(msg)
        except Exception as e:
            rospy.logerr(f"CV Bridge error: {e}")
            return
        
        self.frame_count += 1
        cv_img = smooth_frame_for_inference(cv_img)
        annotated = cv_img.copy()
        frame_mean = float(cv_img.mean()) if cv_img.size > 0 else 0.0
        crop_preview = None
        roi_box = compute_center_roi_box(cv_img.shape, self.center_roi_ratio) if self.enable_center_roi else None
        self.last_result_text = f"[Frame {self.frame_count}] Image received"
        
        try:
            det_results = self.detector.predict(
                source=cv_img,
                conf=self.conf_threshold,
                verbose=False,
                device=self.device
            )
            det_result = det_results[0]
            boxes = det_result.boxes or []  # type: ignore[assignment]
            
            if len(boxes) > 0:
                self.last_detections = len(boxes)
                boxes_any: Any = boxes
                xyxy_values = boxes_any.xyxy.cpu().numpy()
                conf_values = boxes_any.conf.cpu().numpy()

                crop_preview = None
                for i, (xyxy, det_conf_value) in enumerate(zip(xyxy_values, conf_values)):
                    x1, y1, x2, y2 = map(int, xyxy)
                    det_conf = float(det_conf_value)
                    center_x = (x1 + x2) // 2
                    center_y = (y1 + y2) // 2

                    if roi_box is not None and not point_in_box(center_x, center_y, roi_box):
                        continue

                    crop, refined_box, white_ratio = refine_crop_from_box(
                        cv_img,
                        (x1, y1, x2, y2),
                        shrink_ratio=self.crop_shrink,
                        pad_ratio=self.crop_pad,
                        white_threshold=self.white_threshold,
                    )
                    if crop.size == 0:
                        continue
                    
                    try:
                        cls_results = self.classifier.predict(
                            source=crop,
                            verbose=False,
                            device=self.device
                        )
                        cls_result = cls_results[0]
                        probs = cls_result.probs
                        class_id, class_conf, top5 = extract_topk_predictions(probs, self.classifier.names, top_k=5)
                        predicted_name = get_class_name(self.classifier.names, class_id) if class_id >= 0 else "unknown"
                        if class_conf < self.class_conf_threshold:
                            class_name = "unknown"
                        else:
                            class_name = predicted_name
                    except Exception as e:
                        logger.warning(f"Classification error: {e}")
                        class_id = -1
                        class_conf = 0.0
                        class_name = "error"
                        predicted_name = "error"
                        top5 = []
                    
                    rx1, ry1, rx2, ry2 = refined_box
                    center_x = (rx1 + rx2) // 2
                    center_y = (ry1 + ry2) // 2
                    box_w = rx2 - rx1
                    box_h = ry2 - ry1
                    
                    result_dict = {
                        'box_index': i,
                        'detection_conf': det_conf,
                        'class_id': class_id,
                        'class_name': class_name,
                        'predicted_name': predicted_name,
                        'class_conf': class_conf,
                        'center_x': center_x,
                        'center_y': center_y,
                        'x': x1, 'y': y1, 'w': box_w, 'h': box_h,
                        'box': [rx1, ry1, rx2, ry2],
                        'raw_box': [x1, y1, x2, y2],
                        'white_border_ratio': white_ratio,
                        'top5': top5,
                    }
                    self.result_pub.publish(json.dumps(result_dict))
                    
                    coords_msg = Float32MultiArray(data=[float(center_x), float(center_y), det_conf, class_conf])
                    self.coords_pub.publish(coords_msg)
                    
                    result_text = (
                        f"[Frame {self.frame_count}] Box {i}: x={rx1} y={ry1} w={box_w} h={box_h} "
                        f"center=({center_x},{center_y}) det_conf={det_conf:.3f} class={class_name} "
                        f"conf={class_conf:.3f} top1={predicted_name} white_ratio={white_ratio:.3f}"
                    )
                    self.last_result_text = result_text
                    print(result_text, flush=True)

                    crop_preview = self.make_classification_preview(
                        crop,
                        class_name=class_name,
                        class_conf=class_conf,
                        det_conf=det_conf,
                        class_id=class_id,
                        top5=top5,
                        white_ratio=white_ratio,
                        threshold=self.class_conf_threshold
                    )

                    maybe_publish_debug_artifacts(
                        self.debug_dir,
                        f"frame_{self.frame_count:06d}",
                        i,
                        crop,
                        crop_preview,
                        result_dict,
                    )
                    
                    box_color = (0, 0, 255) if class_name == "unknown" else (0, 255, 0)
                    cv2.rectangle(annotated, (rx1, ry1), (rx2, ry2), box_color, 2)
                    cv2.rectangle(annotated, (x1, y1), (x2, y2), (255, 128, 0), 1)
                    label = f"{class_name} {class_conf:.2f}"
                    cv2.putText(annotated, label, (rx1, max(ry1 - 10, 0)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2)
            else:
                self.last_detections = 0
        except Exception as e:
            self.last_detections = 0
            self.last_result_text = f"[Frame {self.frame_count}] Processing error: {repr(e)}"
            print(f"[two_stage_infer] {self.last_result_text}", flush=True)
        
        if self.show_window:
            try:
                main_panel = build_main_display_panel(
                    annotated,
                    frame_count=self.frame_count,
                    frame_mean=frame_mean,
                    last_result_text=self.last_result_text,
                    detection_count=self.last_detections,
                    roi_box=roi_box,
                )
                with self.gui_lock:
                    self.latest_main_panel = main_panel
                    self.latest_crop_panel = crop_preview
                    self.latest_gui_frame_count = self.frame_count
                    self.latest_gui_error = ""
            except Exception as e:
                print(f"[two_stage_infer] GUI prepare error: {e}", flush=True)
                with self.gui_lock:
                    self.latest_gui_error = repr(e)
        
        if self.frame_count % 3 == 0:
            try:
                debug_msg = self.bridge.cv2_to_imgmsg(annotated, "bgr8")
                debug_msg.header = msg.header
                self.debug_pub.publish(debug_msg)
            except:
                pass
    
    def ros_image_to_bgr(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        encoding = (msg.encoding or "").lower()
        if not self.logged_image_encoding:
            print(f"[two_stage_infer] Input encoding={msg.encoding} shape={getattr(img,'shape',None)}", flush=True)
            self.logged_image_encoding = True
        if img.ndim == 2:
            return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        if encoding in ("rgb8", "rgb16"):
            return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        if encoding in ("rgba8", "rgba16"):
            return cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
        if encoding in ("bgra8", "bgra16"):
            return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        if encoding in ("bgr8", "bgr16"):
            return img
        return self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def make_classification_preview(self, crop, class_name, class_conf, det_conf, class_id,
                                    top5=None, white_ratio=1.0, threshold=0.50):
        return build_classification_preview(
            crop,
            class_name=class_name,
            class_conf=class_conf,
            det_conf=det_conf,
            top5=top5 or [],
            white_ratio=white_ratio,
            threshold=threshold,
        )
    
    def print_status(self, _event):
        if self.frame_count == 0:
            print("[two_stage_infer] Waiting for images...", flush=True)
        else:
            print(f"[two_stage_infer] Alive. frames={self.frame_count} last_detections={self.last_detections}. {self.last_result_text}", flush=True)
            print(f"[two_stage_infer] skipped_frames={self.skipped_frames} max_fps={self.max_fps}", flush=True)

    def should_skip_frame(self):
        if self.max_fps <= 0.0:
            return False
        now = rospy.Time.now()
        min_period = rospy.Duration(1.0 / self.max_fps)
        if self.last_process_time != rospy.Time(0) and now - self.last_process_time < min_period:
            return True
        self.last_process_time = now
        return False

    def spin_gui(self):
        if not self.show_window:
            return

        rospy.loginfo("GUI loop started on main thread")
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            main_panel = None
            crop_panel = None
            gui_error = ""
            with self.gui_lock:
                if self.latest_main_panel is not None:
                    main_panel = self.latest_main_panel.copy()
                if self.latest_crop_panel is not None:
                    crop_panel = self.latest_crop_panel.copy()
                gui_error = self.latest_gui_error

            try:
                if main_panel is not None:
                    cv2.imshow(self.window_name, main_panel)
                if crop_panel is not None:
                    cv2.imshow(self.cifar_window_name, crop_panel)
                key = cv2.waitKey(1)
                if key & 0xFF == ord('q'):
                    rospy.signal_shutdown('User requested quit')
                    break
            except Exception as e:
                rospy.logerr(f"GUI loop error: {e}")
                if gui_error:
                    rospy.logerr(f"Last GUI prepare error: {gui_error}")
                self.show_window = False
                break

            rate.sleep()

def main():
    node = None
    try:
        node = TwoStageInferenceNode()
        print("[two_stage_infer] Started. Press Ctrl+C to stop.", flush=True)
        node.spin_gui()
    except Exception as e:
        logger.error(f"Fatal error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if node and node.show_window:
            cv2.destroyAllWindows()
        print("[two_stage_infer] Shutdown complete")

if __name__ == '__main__':
    main()
