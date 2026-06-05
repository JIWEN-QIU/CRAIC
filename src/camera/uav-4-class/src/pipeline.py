from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional

import cv2
import numpy as np
import onnxruntime as ort

try:
    from .rectify import crop_xyxy, rectify_target
except ImportError:
    from rectify import crop_xyxy, rectify_target


@dataclass
class PipelineConfig:
    detector: Path
    classifier: Path
    classes: list[str]
    det_imgsz: int = 640
    cls_imgsz: int = 128
    det_conf: float = 0.25
    det_iou: float = 0.45
    cls_conf: float = 0.35
    pad_ratio: float = 0.08
    providers: tuple[str, ...] = ("CPUExecutionProvider",)

    @classmethod
    def from_file(cls, path: Path) -> "PipelineConfig":
        data = json.loads(path.read_text(encoding="utf-8"))
        base = path.parent
        return cls(
            detector=(base / data["detector"]).resolve(),
            classifier=(base / data["classifier"]).resolve(),
            classes=list(data["classes"]),
            det_imgsz=int(data.get("det_imgsz", 640)),
            cls_imgsz=int(data.get("cls_imgsz", 128)),
            det_conf=float(data.get("det_conf", 0.25)),
            det_iou=float(data.get("det_iou", 0.45)),
            cls_conf=float(data.get("cls_conf", 0.35)),
            pad_ratio=float(data.get("pad_ratio", 0.08)),
            providers=tuple(data.get("providers", ["CPUExecutionProvider"])),
        )


def letterbox(image: np.ndarray, new_size: int, color=(114, 114, 114)):
    h, w = image.shape[:2]
    scale = min(new_size / h, new_size / w)
    resized_w = int(round(w * scale))
    resized_h = int(round(h * scale))
    pad_w = new_size - resized_w
    pad_h = new_size - resized_h
    left = int(round(pad_w / 2 - 0.1))
    right = int(round(pad_w / 2 + 0.1))
    top = int(round(pad_h / 2 - 0.1))
    bottom = int(round(pad_h / 2 + 0.1))
    resized = cv2.resize(image, (resized_w, resized_h), interpolation=cv2.INTER_LINEAR)
    padded = cv2.copyMakeBorder(resized, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
    return padded, scale, left, top


def sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x))


def softmax(x: np.ndarray) -> np.ndarray:
    x = x.astype(np.float32)
    x = x - np.max(x)
    exp = np.exp(x)
    return exp / np.sum(exp)


def nms(boxes: np.ndarray, scores: np.ndarray, iou_threshold: float) -> list[int]:
    if len(boxes) == 0:
        return []
    x1, y1, x2, y2 = boxes.T
    areas = np.maximum(0, x2 - x1) * np.maximum(0, y2 - y1)
    order = scores.argsort()[::-1]
    keep: list[int] = []
    while order.size > 0:
        i = int(order[0])
        keep.append(i)
        if order.size == 1:
            break
        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])
        inter = np.maximum(0, xx2 - xx1) * np.maximum(0, yy2 - yy1)
        union = areas[i] + areas[order[1:]] - inter + 1e-6
        order = order[1:][inter / union <= iou_threshold]
    return keep


class UAV4ClassPipeline:
    def __init__(self, config: PipelineConfig):
        self.config = config
        self.det_sess = ort.InferenceSession(str(config.detector), providers=list(config.providers))
        self.cls_sess = ort.InferenceSession(str(config.classifier), providers=list(config.providers))
        self.det_input = self.det_sess.get_inputs()[0].name
        self.cls_input = self.cls_sess.get_inputs()[0].name

    def _run_detector(self, frame: np.ndarray) -> list[dict[str, Any]]:
        img, scale, pad_x, pad_y = letterbox(frame, self.config.det_imgsz)
        inp = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        inp = np.transpose(inp, (2, 0, 1))[None, ...]
        out = self.det_sess.run(None, {self.det_input: inp})[0]
        pred = np.squeeze(out)
        if pred.ndim != 2:
            return []
        if pred.shape[0] in (5, 6, 84):
            pred = pred.T

        raw_boxes = []
        scores = []
        h, w = frame.shape[:2]
        for row in pred:
            if row.shape[0] < 5:
                continue
            cx, cy, bw, bh = row[:4].astype(float)
            score = float(row[4])
            if score < self.config.det_conf:
                continue
            x1 = (cx - bw / 2.0 - pad_x) / scale
            y1 = (cy - bh / 2.0 - pad_y) / scale
            x2 = (cx + bw / 2.0 - pad_x) / scale
            y2 = (cy + bh / 2.0 - pad_y) / scale
            x1 = float(np.clip(x1, 0, w - 1))
            y1 = float(np.clip(y1, 0, h - 1))
            x2 = float(np.clip(x2, 0, w - 1))
            y2 = float(np.clip(y2, 0, h - 1))
            if x2 <= x1 or y2 <= y1:
                continue
            raw_boxes.append([x1, y1, x2, y2])
            scores.append(score)

        if not raw_boxes:
            return []
        boxes_np = np.asarray(raw_boxes, dtype=np.float32)
        scores_np = np.asarray(scores, dtype=np.float32)
        keep = nms(boxes_np, scores_np, self.config.det_iou)
        return [{"box": boxes_np[i].tolist(), "det_conf": float(scores_np[i])} for i in keep]

    def _run_classifier(self, image_bgr: np.ndarray) -> tuple[int, str, float, list[dict[str, Any]]]:
        img = cv2.resize(image_bgr, (self.config.cls_imgsz, self.config.cls_imgsz), interpolation=cv2.INTER_AREA)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
        inp = np.transpose(img, (2, 0, 1))[None, ...]
        out = np.squeeze(self.cls_sess.run(None, {self.cls_input: inp})[0])
        probs = softmax(out) if np.any(out < 0) or np.max(out) > 1.0 else out.astype(np.float32)
        top_ids = np.argsort(probs)[::-1]
        topk = []
        for idx in top_ids[: min(4, len(top_ids))]:
            name = self.config.classes[int(idx)] if int(idx) < len(self.config.classes) else str(int(idx))
            topk.append({"id": int(idx), "name": name, "conf": float(probs[int(idx)])})
        top1 = topk[0]
        label = top1["name"] if top1["conf"] >= self.config.cls_conf else "unknown"
        return int(top1["id"]), label, float(top1["conf"]), topk

    def infer(self, frame: np.ndarray) -> list[dict[str, Any]]:
        detections = self._run_detector(frame)
        results: list[dict[str, Any]] = []
        for det in detections:
            box = det["box"]
            roi = crop_xyxy(frame, box, pad_ratio=self.config.pad_ratio)
            rectified = rectify_target(roi, output_size=self.config.cls_imgsz, fallback_resize=True) if roi is not None else None
            if rectified is None:
                continue
            cls_id, cls_name, cls_conf, topk = self._run_classifier(rectified.image)
            x1, y1, x2, y2 = [float(v) for v in box]
            results.append(
                {
                    "class_id": cls_id,
                    "class_name": cls_name,
                    "class_conf": cls_conf,
                    "det_conf": det["det_conf"],
                    "box": [x1, y1, x2, y2],
                    "center": [(x1 + x2) / 2.0, (y1 + y2) / 2.0],
                    "rectify": rectified.method,
                    "topk": topk,
                }
            )
        return results


def draw_results(frame: np.ndarray, results: list[dict[str, Any]]) -> np.ndarray:
    out = frame.copy()
    for item in results:
        x1, y1, x2, y2 = [int(round(v)) for v in item["box"]]
        color = (0, 220, 0) if item["class_name"] != "unknown" else (0, 165, 255)
        cv2.rectangle(out, (x1, y1), (x2, y2), color, 2)
        text = f'{item["class_name"]} {item["class_conf"]:.2f}'
        cv2.putText(out, text, (x1, max(18, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(out, text, (x1, max(18, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1, cv2.LINE_AA)
    return out
