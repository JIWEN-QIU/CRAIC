#!/usr/bin/env python3
"""
本地测试脚本：验证两段式推理在本地运行
不需要无人机、ROS 或摄像头，可用录像或图片文件夹测试
"""
import argparse
import cv2
import json
import logging
import os
import time
from typing import Any
from pathlib import Path
from ultralytics import YOLO
import numpy as np

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


def build_crop_preview(crop, class_name, class_conf, det_conf, top5, white_ratio, threshold):
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


def smooth_frame_for_inference(frame):
    if frame is None or frame.size == 0:
        return frame

    smoothed = cv2.bilateralFilter(frame, d=7, sigmaColor=55, sigmaSpace=55)
    smoothed = cv2.GaussianBlur(smoothed, (3, 3), 0)
    return smoothed


def save_debug_artifacts(output_dir, frame_name, det_index, crop, preview, metadata):
    debug_dir = Path(output_dir) / "debug"
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

    return crop_path, preview_path, meta_path


def classify_crop(classifier, crop, class_conf_threshold):
    cls_results = classifier.predict(source=crop, verbose=False)
    cls_result = cls_results[0]
    probs = cls_result.probs

    top1_id, top1_conf, top5 = extract_topk_predictions(probs, classifier.names, top_k=5)
    predicted_name = get_class_name(classifier.names, top1_id) if top1_id >= 0 else "unknown"

    if top1_conf < class_conf_threshold:
        final_name = "unknown"
    else:
        final_name = predicted_name

    return {
        "class_id": top1_id,
        "predicted_name": predicted_name,
        "class_name": final_name,
        "class_conf": top1_conf,
        "top5": top5,
    }

def test_with_video(det_model_path, cls_model_path, video_path, output_dir=None, conf=0.25,
                    class_conf_threshold=0.50, crop_shrink=0.90, crop_pad=0.02, gui=True):
    """从视频文件进行推理测试"""
    logger.info("Loading models...")
    detector = YOLO(det_model_path)
    classifier = YOLO(cls_model_path)

    logger.info(f"Opening video: {video_path}")
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        logger.error(f"Failed to open video: {video_path}")
        return

    frame_count = 0
    total_det_time = 0
    total_cls_time = 0
    detection_count = 0
    unknown_count = 0

    output_path = None
    writer = None
    if output_dir:
        output_dir = Path(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        output_path = output_dir / "output.mp4"
        fps = int(cap.get(cv2.CAP_PROP_FPS)) or 25
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fourcc = getattr(cv2, "VideoWriter_fourcc")(*"mp4v")
        writer = cv2.VideoWriter(str(output_path),
                                 fourcc,
                                 fps, (width, height))

    gui_enabled = bool(gui and os.environ.get("DISPLAY"))
    main_window_name = "Two-Stage Inference"
    crop_window_name = "Crop Debug"
    if gui_enabled:
        try:
            cv2.startWindowThread()
            cv2.namedWindow(main_window_name, cv2.WINDOW_NORMAL)
            cv2.namedWindow(crop_window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(main_window_name, 1100, 800)
            cv2.resizeWindow(crop_window_name, 520, 720)
        except Exception as exc:
            logger.warning(f"GUI init failed, disabling windows: {exc}")
            gui_enabled = False

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        frame_count += 1
        frame = smooth_frame_for_inference(frame)
        annotated = frame.copy()
        crop_preview = None

        # 检测
        t0 = time.time()
        det_results = detector.predict(source=frame, conf=conf, verbose=False)
        det_time = time.time() - t0
        total_det_time += det_time

        det_result = det_results[0]
        boxes = det_result.boxes or []  # type: ignore[assignment]

        if len(boxes) > 0:
            boxes_any: Any = boxes
            xyxy_values = boxes_any.xyxy.cpu().numpy()
            conf_values = boxes_any.conf.cpu().numpy()
            for i, (xyxy, det_conf_value) in enumerate(zip(xyxy_values, conf_values)):
                x1, y1, x2, y2 = map(int, xyxy)
                det_conf = float(det_conf_value)

                crop, refined_box, white_ratio = refine_crop_from_box(
                    frame,
                    (x1, y1, x2, y2),
                    shrink_ratio=crop_shrink,
                    pad_ratio=crop_pad,
                )
                if crop.size == 0:
                    continue

                # 分类
                t1 = time.time()
                cls_info = classify_crop(classifier, crop, class_conf_threshold=class_conf_threshold)
                cls_time = time.time() - t1
                total_cls_time += cls_time

                class_id = cls_info["class_id"]
                class_conf = cls_info["class_conf"]
                class_name = cls_info["class_name"]
                predicted_name = cls_info["predicted_name"]
                top5 = cls_info["top5"]

                if class_name == "unknown":
                    unknown_count += 1

                rx1, ry1, rx2, ry2 = refined_box
                box_color = (0, 0, 255) if class_name == "unknown" else (0, 255, 0)

                # 绘制
                cv2.rectangle(annotated, (rx1, ry1), (rx2, ry2), box_color, 2)
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (255, 128, 0), 1)
                label = f"{class_name} {class_conf:.2f}"
                cv2.putText(annotated, label, (rx1, max(ry1 - 10, 0)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2)

                detection_count += 1
                crop_preview = build_crop_preview(
                    crop,
                    class_name=class_name,
                    class_conf=class_conf,
                    det_conf=det_conf,
                    top5=top5,
                    white_ratio=white_ratio,
                    threshold=class_conf_threshold,
                )

                if output_path:
                    frame_name = f"frame_{frame_count:06d}"
                    metadata = {
                        "frame": frame_count,
                        "box_index": i,
                        "detection_conf": det_conf,
                        "class_id": class_id,
                        "class_name": class_name,
                        "predicted_name": predicted_name,
                        "class_conf": class_conf,
                        "refined_box": [int(rx1), int(ry1), int(rx2), int(ry2)],
                        "raw_box": [int(x1), int(y1), int(x2), int(y2)],
                        "white_border_ratio": white_ratio,
                        "top5": top5,
                    }
                    save_debug_artifacts(output_path, frame_name, i, crop, crop_preview, metadata)

                if frame_count % 30 == 0:
                    logger.info(
                        f"[Frame {frame_count}] {class_name} conf={class_conf:.3f} "
                        f"top1={predicted_name} white_ratio={white_ratio:.3f}"
                    )

        # 写入输出视频
        if writer is not None:
            writer.write(annotated)

        # 显示 (如果在 GUI 环境)
        if gui_enabled:
            try:
                cv2.imshow(main_window_name, annotated)
                if crop_preview is not None:
                    cv2.imshow(crop_window_name, crop_preview)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            except Exception as exc:
                logger.warning(f"GUI display failed, disabling windows: {exc}")
                gui_enabled = False

    cap.release()
    if writer is not None:
        writer.release()
        logger.info(f"Output saved to {output_path}")
    if gui_enabled:
        cv2.destroyAllWindows()

    # 统计
    logger.info("\n=== Statistics ===")
    logger.info(f"Total frames: {frame_count}")
    logger.info(f"Total detections: {detection_count}")
    logger.info(f"Unknown classifications: {unknown_count}")
    logger.info(f"Avg detection time: {total_det_time / frame_count * 1000:.2f} ms")
    logger.info(
        f"Avg classification time: {total_cls_time / detection_count * 1000:.2f} ms"
        if detection_count > 0 else "No detections"
    )
    logger.info(f"Total inference time: {(total_det_time + total_cls_time) / frame_count * 1000:.2f} ms")

def test_with_image_folder(det_model_path, cls_model_path, image_dir, output_dir=None, conf=0.25,
                           class_conf_threshold=0.50, crop_shrink=0.90, crop_pad=0.02, gui=True):
    """从图片文件夹进行推理测试"""
    logger.info("Loading models...")
    detector = YOLO(det_model_path)
    classifier = YOLO(cls_model_path)

    image_dir = Path(image_dir)
    images = list(image_dir.glob("*.jpg")) + list(image_dir.glob("*.png"))
    logger.info(f"Found {len(images)} images")

    if output_dir:
        output_dir = Path(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)

    total_det_time = 0
    total_cls_time = 0
    detection_count = 0
    unknown_count = 0

    gui_enabled = bool(gui and os.environ.get("DISPLAY"))
    main_window_name = "Two-Stage Inference"
    crop_window_name = "Crop Debug"
    if gui_enabled:
        try:
            cv2.startWindowThread()
            cv2.namedWindow(main_window_name, cv2.WINDOW_NORMAL)
            cv2.namedWindow(crop_window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(main_window_name, 1100, 800)
            cv2.resizeWindow(crop_window_name, 520, 720)
        except Exception as exc:
            logger.warning(f"GUI init failed, disabling windows: {exc}")
            gui_enabled = False

    for img_idx, img_path in enumerate(images[:50]):  # 限制前 50 张
        logger.info(f"Processing {img_idx + 1}/{min(50, len(images))}: {img_path.name}")

        frame = cv2.imread(str(img_path))
        if frame is None:
            logger.warning(f"Failed to load {img_path}")
            continue

        frame = smooth_frame_for_inference(frame)
        annotated = frame.copy()
        crop_preview = None

        # 检测
        t0 = time.time()
        det_results = detector.predict(source=frame, conf=conf, verbose=False)
        det_time = time.time() - t0
        total_det_time += det_time

        det_result = det_results[0]
        boxes = det_result.boxes or []  # type: ignore[assignment]

        if len(boxes) > 0:
            boxes_any: Any = boxes
            xyxy_values = boxes_any.xyxy.cpu().numpy()
            conf_values = boxes_any.conf.cpu().numpy()
            for i, (xyxy, det_conf_value) in enumerate(zip(xyxy_values, conf_values)):
                x1, y1, x2, y2 = map(int, xyxy)
                det_conf = float(det_conf_value)

                crop, refined_box, white_ratio = refine_crop_from_box(
                    frame,
                    (x1, y1, x2, y2),
                    shrink_ratio=crop_shrink,
                    pad_ratio=crop_pad,
                )
                if crop.size == 0:
                    continue

                # 分类
                t1 = time.time()
                cls_info = classify_crop(classifier, crop, class_conf_threshold=class_conf_threshold)
                cls_time = time.time() - t1
                total_cls_time += cls_time

                class_id = cls_info["class_id"]
                class_conf = cls_info["class_conf"]
                class_name = cls_info["class_name"]
                predicted_name = cls_info["predicted_name"]
                top5 = cls_info["top5"]

                if class_name == "unknown":
                    unknown_count += 1

                rx1, ry1, rx2, ry2 = refined_box
                box_color = (0, 0, 255) if class_name == "unknown" else (0, 255, 0)

                # 绘制
                cv2.rectangle(annotated, (rx1, ry1), (rx2, ry2), box_color, 2)
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (255, 128, 0), 1)
                label = f"{class_name} {class_conf:.2f}"
                cv2.putText(annotated, label, (rx1, max(ry1 - 10, 0)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2)

                detection_count += 1
                crop_preview = build_crop_preview(
                    crop,
                    class_name=class_name,
                    class_conf=class_conf,
                    det_conf=det_conf,
                    top5=top5,
                    white_ratio=white_ratio,
                    threshold=class_conf_threshold,
                )

                if output_dir:
                    metadata = {
                        "image": img_path.name,
                        "box_index": i,
                        "detection_conf": det_conf,
                        "class_id": class_id,
                        "class_name": class_name,
                        "predicted_name": predicted_name,
                        "class_conf": class_conf,
                        "refined_box": [int(rx1), int(ry1), int(rx2), int(ry2)],
                        "raw_box": [int(x1), int(y1), int(x2), int(y2)],
                        "white_border_ratio": white_ratio,
                        "top5": top5,
                    }
                    save_debug_artifacts(output_dir, img_path.stem, i, crop, crop_preview, metadata)

                logger.info(
                    f"  Box {i}: {class_name} {class_conf:.3f} "
                    f"top1={predicted_name} white_ratio={white_ratio:.3f}"
                )

        # 保存结果
        if output_dir:
            output_img = output_dir / f"annotated_{img_path.stem}.jpg"
            cv2.imwrite(str(output_img), annotated)

        if gui_enabled:
            try:
                cv2.imshow(main_window_name, annotated)
                if crop_preview is not None:
                    cv2.imshow(crop_window_name, crop_preview)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            except Exception as exc:
                logger.warning(f"GUI display failed, disabling windows: {exc}")
                gui_enabled = False

    # 统计
    logger.info("\n=== Statistics ===")
    logger.info(f"Total images processed: {len(images)}")
    logger.info(f"Total detections: {detection_count}")
    logger.info(f"Unknown classifications: {unknown_count}")
    if len(images) > 0:
        logger.info(f"Avg detection time: {total_det_time / len(images) * 1000:.2f} ms")
    if detection_count > 0:
        logger.info(f"Avg classification time: {total_cls_time / detection_count * 1000:.2f} ms")

    if gui_enabled:
        cv2.destroyAllWindows()

def main():
    parser = argparse.ArgumentParser(description="Local test for two-stage YOLO inference")
    parser.add_argument('--det-model', required=True, help='Detector model path (OpenVINO or PyTorch)')
    parser.add_argument('--cls-model', required=True, help='Classifier model path (OpenVINO or PyTorch)')
    parser.add_argument('--source', required=True, help='Input source: video file or image folder')
    parser.add_argument('--output', default=None, help='Output directory (optional)')
    parser.add_argument('--conf', type=float, default=0.25, help='Detection confidence threshold')
    parser.add_argument('--class-conf', type=float, default=0.50, help='Classification confidence threshold; below this outputs unknown')
    parser.add_argument('--crop-shrink', type=float, default=0.90, help='Initial crop shrink ratio to reduce background')
    parser.add_argument('--crop-pad', type=float, default=0.02, help='Small adaptive pad ratio around the shrunken crop')
    parser.add_argument('--no-gui', action='store_true', help='Disable OpenCV popup windows')
    parser.add_argument('--type', choices=['video', 'images'], default='video', help='Input type')
    
    args = parser.parse_args()
    
    source_path = Path(args.source)
    if not source_path.exists():
        logger.error(f"Source not found: {args.source}")
        return
    
    gui_enabled = not args.no_gui

    if args.type == 'video':
        test_with_video(
            args.det_model,
            args.cls_model,
            args.source,
            args.output,
            args.conf,
            class_conf_threshold=args.class_conf,
            crop_shrink=args.crop_shrink,
            crop_pad=args.crop_pad,
            gui=gui_enabled,
        )
    else:
        test_with_image_folder(
            args.det_model,
            args.cls_model,
            args.source,
            args.output,
            args.conf,
            class_conf_threshold=args.class_conf,
            crop_shrink=args.crop_shrink,
            crop_pad=args.crop_pad,
            gui=gui_enabled,
        )

if __name__ == '__main__':
    main()
