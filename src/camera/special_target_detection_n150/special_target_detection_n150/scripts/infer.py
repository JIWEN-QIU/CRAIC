from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

from ultralytics import YOLO

from common import DEFAULT_CONFIG, DEFAULT_WEIGHTS, as_output_dict, load_config, require_cuda_if_requested, resolve_path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run special_target detection inference.")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG), help="Path to training YAML config.")
    parser.add_argument("--weights", default=str(DEFAULT_WEIGHTS), help="Path to trained weights.")
    parser.add_argument("--source", required=True, help="Image, folder, video, camera index, or stream URL.")
    parser.add_argument("--conf", type=float, help="Confidence threshold for present=true.")
    parser.add_argument("--iou", type=float, help="NMS IoU threshold.")
    parser.add_argument("--device", help="Device override, e.g. 0 or cpu.")
    parser.add_argument("--save-vis", action="store_true", help="Save visualization images/video.")
    parser.add_argument("--json", action="store_true", help="Print compact JSON lines for each result.")
    return parser.parse_args()


def source_value(raw_source: str) -> str | int:
    if raw_source.isdigit():
        return int(raw_source)
    path = Path(raw_source)
    if path.exists():
        return str(path)
    resolved = resolve_path(raw_source)
    if resolved.exists():
        return str(resolved)
    return raw_source


def result_to_record(result: Any, conf_threshold: float) -> dict[str, Any]:
    detections: list[dict[str, Any]] = []
    best_conf = 0.0

    if result.boxes is not None:
        for box in result.boxes:
            cls_id = int(box.cls.item())
            conf = float(box.conf.item())
            if cls_id != 0 or conf < conf_threshold:
                continue
            xyxy = [float(value) for value in box.xyxy[0].tolist()]
            detections.append(
                {
                    "class_id": cls_id,
                    "class_name": result.names.get(cls_id, "special_target"),
                    "confidence": round(conf, 6),
                    "xyxy": [round(value, 2) for value in xyxy],
                }
            )
            best_conf = max(best_conf, conf)

    return {
        "source": str(result.path),
        "present": len(detections) > 0,
        "best_confidence": round(best_conf, 6),
        "detections": detections,
    }


def main() -> None:
    args = parse_args()
    cfg = load_config(args.config)

    weights = resolve_path(args.weights)
    if not weights.exists():
        raise FileNotFoundError(f"Weights not found: {weights}")

    conf = args.conf if args.conf is not None else float(cfg.get("conf", 0.25))
    iou = args.iou if args.iou is not None else float(cfg.get("iou", 0.7))
    device = args.device if args.device is not None else cfg.get("device", 0)

    require_cuda_if_requested(device)

    model = YOLO(str(weights))
    results = model.predict(
        source=source_value(args.source),
        conf=conf,
        iou=iou,
        device=device,
        save=args.save_vis,
        project=str(resolve_path(cfg.get("project", "runs/special_target"))),
        name=cfg.get("predict_name", "predict"),
        exist_ok=True,
        stream=True,
        verbose=False,
    )

    for result in results:
        record = result_to_record(result, conf)
        if args.json:
            print(as_output_dict(record))
        else:
            status = "present" if record["present"] else "absent"
            print(f"{record['source']}: {status}, best_confidence={record['best_confidence']}")
            for det in record["detections"]:
                print(f"  {det['class_name']} conf={det['confidence']} xyxy={det['xyxy']}")


if __name__ == "__main__":
    main()
