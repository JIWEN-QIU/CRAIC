from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np


@dataclass
class RectifyResult:
    image: np.ndarray
    corners: np.ndarray
    method: str


def order_points(points: np.ndarray) -> np.ndarray:
    pts = np.asarray(points, dtype=np.float32).reshape(4, 2)
    rect = np.zeros((4, 2), dtype=np.float32)
    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]
    diff = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]
    return rect


def clip_box(
    xyxy: Tuple[float, float, float, float],
    width: int,
    height: int,
    pad_ratio: float = 0.08,
) -> Tuple[int, int, int, int]:
    x1, y1, x2, y2 = xyxy
    box_w = max(1.0, x2 - x1)
    box_h = max(1.0, y2 - y1)
    pad = pad_ratio * max(box_w, box_h)
    x1 = int(max(0, round(x1 - pad)))
    y1 = int(max(0, round(y1 - pad)))
    x2 = int(min(width, round(x2 + pad)))
    y2 = int(min(height, round(y2 + pad)))
    return x1, y1, x2, y2


def crop_xyxy(image: np.ndarray, xyxy, pad_ratio: float = 0.08) -> Optional[np.ndarray]:
    h, w = image.shape[:2]
    x1, y1, x2, y2 = clip_box(tuple(xyxy), w, h, pad_ratio=pad_ratio)
    if x2 <= x1 or y2 <= y1:
        return None
    return image[y1:y2, x1:x2].copy()


def _candidate_masks(gray: np.ndarray) -> list[np.ndarray]:
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    masks: list[np.ndarray] = []
    _, otsu = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    masks.append(otsu)
    masks.append(cv2.bitwise_not(otsu))
    edges = cv2.Canny(blurred, 40, 130)
    kernel = np.ones((3, 3), np.uint8)
    edges = cv2.dilate(edges, kernel, iterations=1)
    edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=2)
    masks.append(edges)
    return masks


def find_target_corners(roi_bgr: np.ndarray, min_area_ratio: float = 0.08):
    if roi_bgr is None or roi_bgr.size == 0:
        return None, "empty"

    gray = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2GRAY)
    best_contour = None
    best_area = 0.0
    image_area = float(gray.shape[0] * gray.shape[1])

    for mask in _candidate_masks(gray):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area >= image_area * min_area_ratio and area > best_area:
                best_area = area
                best_contour = contour

    if best_contour is None:
        return None, "not_found"

    peri = cv2.arcLength(best_contour, True)
    for eps in (0.015, 0.02, 0.03, 0.04, 0.06):
        approx = cv2.approxPolyDP(best_contour, eps * peri, True)
        if len(approx) == 4 and cv2.isContourConvex(approx):
            return order_points(approx.reshape(4, 2)), "contour4"

    rect = cv2.minAreaRect(best_contour)
    return order_points(cv2.boxPoints(rect)), "min_area_rect"


def rectify_target(
    roi_bgr: np.ndarray,
    output_size: int = 128,
    min_area_ratio: float = 0.08,
    fallback_resize: bool = True,
) -> Optional[RectifyResult]:
    corners, method = find_target_corners(roi_bgr, min_area_ratio=min_area_ratio)
    if corners is None:
        if not fallback_resize or roi_bgr is None or roi_bgr.size == 0:
            return None
        resized = cv2.resize(roi_bgr, (output_size, output_size), interpolation=cv2.INTER_AREA)
        h, w = roi_bgr.shape[:2]
        fallback_corners = np.array([[0, 0], [w - 1, 0], [w - 1, h - 1], [0, h - 1]], dtype=np.float32)
        return RectifyResult(resized, fallback_corners, f"fallback_{method}")

    dst = np.array(
        [[0, 0], [output_size - 1, 0], [output_size - 1, output_size - 1], [0, output_size - 1]],
        dtype=np.float32,
    )
    matrix = cv2.getPerspectiveTransform(corners, dst)
    warped = cv2.warpPerspective(roi_bgr, matrix, (output_size, output_size))
    return RectifyResult(warped, corners, method)
