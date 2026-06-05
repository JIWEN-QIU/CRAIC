# UAV 4-Class Target Inference

This package runs the full deployment flow:

```text
camera frame -> ONNX YOLO detector -> ROI crop -> perspective rectification -> ONNX 4-class classifier
```

Classes:

```text
apple, motorcycle, sunflower, telephone
```

## Files

```text
uav-4-class/
  config.json
  requirements.txt
  run_onnx.py
  models/
    target_detector.onnx
    target_4class_classifier.onnx
  src/
    pipeline.py
    rectify.py
  ros/uav_4class_infer/
```

## Install

```bash
cd uav-4-class
pip install -r requirements.txt
```

For GPU inference with ONNXRuntime, install the GPU build instead of the CPU package and change providers in `config.json`.

## Run Without ROS

Camera:

```bash
python run_onnx.py --source 0 --show --print-json
```

Image:

```bash
python run_onnx.py --source test.jpg --save runs/test_result.jpg --print-json
```

Video:

```bash
python run_onnx.py --source input.mp4 --save runs/result.mp4 --print-json
```

Output JSON fields:

```text
class_id, class_name, class_conf, det_conf, box, center, rectify, topk
```

## ROS Noetic

Copy this directory to the UAV, then put the ROS package into your catkin workspace:

```bash
cp -r uav-4-class/ros/uav_4class_infer ~/catkin_ws/src/
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch uav_4class_infer uav_4class.launch config_path:=$HOME/uav-4-class/config.json camera_topic:=/usb_cam/image_raw
```

ROS topics:

```text
~target_info    std_msgs/String JSON result list
~target_coords  std_msgs/Float32MultiArray [center_x, center_y, det_conf, class_conf, class_id]
~debug_image    sensor_msgs/Image annotated frame
```

## Notes

The detector outputs horizontal boxes. The deployment pipeline always rectifies each ROI before classification, so rotated square targets are pulled back to a 128x128 square before entering the classifier.
