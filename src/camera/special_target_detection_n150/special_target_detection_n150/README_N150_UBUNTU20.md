# Special Target Detection - N150 Ubuntu 20.04 Deployment

This package contains the inference-side files for the `special_target` YOLO
detector. It is intended for deployment on an Ubuntu 20.04 onboard computer.

## Included Files

- `weights/best.pt`: trained YOLO detector weights.
- `configs/train_special_target.yaml`: runtime defaults, including confidence,
  IoU, model path, and device.
- `scripts/common.py`: shared path/config helpers.
- `scripts/infer.py`: image, directory, video, camera, or stream inference.
- `requirements_ubuntu20.txt`: Python package requirements for inference.

Training data and training logs are not included in this deployment package.

## Recommended Setup

Create or activate your Python environment on Ubuntu 20.04, then install
dependencies:

```bash
python3 -m pip install -r requirements_ubuntu20.txt
```

For GPU inference, install a PyTorch build that matches the CUDA driver/runtime
on the N150 computer before installing Ultralytics. If CUDA is not available,
run inference with `--device cpu` or change `device: cpu` in the config.

## Run Inference

Image or folder:

```bash
python3 scripts/infer.py --weights weights/best.pt --source /path/to/image_or_folder
```

Camera:

```bash
python3 scripts/infer.py --weights weights/best.pt --source 0
```

JSON output:

```bash
python3 scripts/infer.py --weights weights/best.pt --source /path/to/image.jpg --json
```

Save visualization:

```bash
python3 scripts/infer.py --weights weights/best.pt --source /path/to/image_or_folder --save-vis
```

## Output Meaning

The detector returns `present=true` when at least one `special_target` detection
has confidence greater than or equal to `conf` in the config. The default
threshold is `0.25`.

The current model was trained with positive samples only. For robust no-target
behavior on real flight scenes, add real negative samples and retrain.
