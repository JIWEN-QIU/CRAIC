import argparse
import csv
import time
from pathlib import Path

from ultralytics import YOLO


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", default="../openvino_model/best_openvino_model")
    parser.add_argument("--images", default="../benchmark_inputs")
    parser.add_argument("--imgsz", type=int, default=512)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--repeat", type=int, default=20)
    parser.add_argument("--out", default="n150_openvino_benchmark.csv")
    args = parser.parse_args()

    model = YOLO(args.model, task="segment")
    images = sorted(Path(args.images).glob("*.jpg"))
    rows = []
    for i in range(args.repeat):
        for image in images:
            t0 = time.perf_counter()
            results = model.predict(source=str(image), imgsz=args.imgsz, device=args.device, conf=0.25, verbose=False)
            elapsed_ms = (time.perf_counter() - t0) * 1000.0
            detections = 0 if results[0].boxes is None else len(results[0].boxes)
            rows.append({"repeat": i, "image": image.name, "elapsed_ms": elapsed_ms, "detections": detections})
    with open(args.out, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=["repeat", "image", "elapsed_ms", "detections"])
        writer.writeheader()
        writer.writerows(rows)
    print(f"wrote {args.out}; rows={len(rows)}")


if __name__ == "__main__":
    main()
