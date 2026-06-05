from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from pipeline import PipelineConfig, UAV4ClassPipeline, draw_results


IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def open_source(source: str):
    if source.isdigit():
        cap = cv2.VideoCapture(int(source), cv2.CAP_DSHOW)
        if not cap.isOpened():
            cap = cv2.VideoCapture(int(source))
        return cap, "camera"
    path = Path(source)
    if path.exists() and path.suffix.lower() in IMAGE_SUFFIXES:
        data = np.fromfile(str(path), dtype=np.uint8)
        return cv2.imdecode(data, cv2.IMREAD_COLOR), "image"
    return cv2.VideoCapture(source), "video"


def imwrite(path: Path, image) -> bool:
    ext = path.suffix or ".jpg"
    ok, encoded = cv2.imencode(ext, image)
    if not ok:
        return False
    path.parent.mkdir(parents=True, exist_ok=True)
    encoded.tofile(str(path))
    return True


def main() -> None:
    parser = argparse.ArgumentParser(description="UAV 4-class ONNX inference: detect -> rectify -> classify.")
    parser.add_argument("--config", type=Path, default=ROOT / "config.json")
    parser.add_argument("--source", default="0", help="Camera index, image path, or video path.")
    parser.add_argument("--save", type=Path, default=None, help="Output image/video path.")
    parser.add_argument("--show", action="store_true")
    parser.add_argument("--print-json", action="store_true")
    args = parser.parse_args()

    cfg = PipelineConfig.from_file(args.config)
    pipeline = UAV4ClassPipeline(cfg)
    source, mode = open_source(args.source)

    if mode == "image":
        if source is None:
            raise SystemExit(f"Cannot read image: {args.source}")
        results = pipeline.infer(source)
        print(json.dumps(results, ensure_ascii=False))
        if args.save:
            imwrite(args.save, draw_results(source, results))
        return

    if not source.isOpened():
        raise SystemExit(f"Cannot open source: {args.source}")

    writer = None
    frame_id = 0
    last = time.perf_counter()
    while True:
        ok, frame = source.read()
        if not ok:
            break

        results = pipeline.infer(frame)
        now = time.perf_counter()
        fps = 1.0 / max(1e-6, now - last)
        last = now
        annotated = draw_results(frame, results)
        cv2.putText(annotated, f"FPS {fps:.1f}", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        if args.print_json and results:
            print(json.dumps({"frame": frame_id, "results": results}, ensure_ascii=False))

        if args.save and writer is None:
            args.save.parent.mkdir(parents=True, exist_ok=True)
            h, w = annotated.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            writer = cv2.VideoWriter(str(args.save), fourcc, 25.0, (w, h))
        if writer is not None:
            writer.write(annotated)

        if args.show:
            cv2.imshow("uav-4-class", annotated)
            if cv2.waitKey(1) & 0xFF == 27:
                break
        frame_id += 1

    source.release()
    if writer is not None:
        writer.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
