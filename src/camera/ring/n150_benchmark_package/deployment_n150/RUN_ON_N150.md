# RUN_ON_N150

This package is for Intel N150 deployment feasibility testing only.

## Offline Image Benchmark

```powershell
cd benchmark_scripts
python run_openvino_image_benchmark.py --model ../openvino_model/best_openvino_model --images ../benchmark_inputs --imgsz 512 --device cpu --repeat 20
```

Repeat with supported OpenVINO device names such as `cpu`, `gpu`, or `auto` if the N150 runtime exposes them.

## Required Records

- Mean, median, p95, and max latency.
- Sustained FPS.
- CPU/iGPU utilization, memory, and temperature if available.
- Missed detections on far-distance samples at lower input sizes.
- ROS/Odin read-only parallel run latency and dropped-frame behavior.

## Safety Boundary

This package must not be used to command flight. ROS/Odin tests are subscription-only and logging-only.
