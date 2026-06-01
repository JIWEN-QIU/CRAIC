# 两段式 YOLO 无人机目标检测系统

> **系统概述**: 集检测 + 分类的实时两段式推理流水线，针对 Intel N150 CPU 优化，通过 OpenVINO 加速

---

## 📋 项目背景

**目标**: 无人机需要识别包含 CIFAR-100 图像的物体靶标，并返回其 2D 坐标和分类结果

**方案**: 
1. **第一阶段（检测）**: YOLOv8n 检测器定位靶标位置，输出边界框
2. **第二阶段（分类）**: YOLOv8n-cls 分类器识别靶标内的 CIFAR 类别

**运行环境**: 
- 开发: Windows/Linux (Intel Core i9 + CUDA)
- 部署: N150 CPU, ROS Noetic, Ubuntu 20.04

---

## 🏗️ 系统架构

```
┌─────────────────────────────────────────────┐
│         USB Camera Frame                     │
└────────────────────┬────────────────────────┘
                     │
                     ▼
        ┌────────────────────────┐
        │   Detection (Stage 1)  │
        │  YOLOv8n @ 640×640     │
        │  ✓ mAP ≈ 0.995         │
        └────────────┬───────────┘
                     │ (boxes)
                     ▼
        ┌────────────────────────────────┐
        │ For each detection box:        │
        │  1. Crop region               │
        │  2. Resize to 128×128         │
        │  3. Classification (Stage 2)  │
        │  4. Extract CIFAR class       │
        └────────────┬──────────────────┘
                     │
                     ▼
        ┌─────────────────────────────┐
        │  Output (ROS Topic)        │
        │  - Coordinates (x, y)      │
        │  - Class name              │
        │  - Confidence scores       │
        └─────────────────────────────┘
```

---

## 📦 模型详情

### 检测器 (YOLOv8n)

| 项目 | 数值 |
|------|------|
| **参数** | 3,005,843 |
| **GFLOPs** | 8.1 @ 640×640 |
| **导出格式** | OpenVINO (11.8 MB) |
| **验证 mAP** | 0.995 (单类靶标) |
| **推理时间** | ~50-100ms (N150) |
| **输入** | 640×640 RGB 图像 |
| **输出** | 边界框 (x1, y1, x2, y2) + 置信度 |

### 分类器 (YOLOv8n-cls)

| 项目 | 数值 |
|------|------|
| **参数** | 1,562,980 |
| **GFLOPs** | 3.4 @ 128×128 |
| **导出格式** | OpenVINO (6.1 MB) |
| **验证准确率** | top1=78.97%, top5=95.86% |
| **类别** | 100 (CIFAR-100) |
| **推理时间** | ~20-50ms (N150) |
| **输入** | 128×128 RGB 图像 |
| **输出** | 类别标签 + 置信度 (0-1) |

---

## 🚀 快速开始

### 最小化步骤

```bash
# 1. 本地组织模型
cd uav_deployment
python3 organize_models.py

# 2. 传输到无人机
scp -r models uav@192.168.1.100:~/uav_models/
scp -r two_stage_infer uav@192.168.1.100:~/catkin_ws/src/
scp requirements.txt uav@192.168.1.100:~

# 3. 无人机端配置
ssh uav@192.168.1.100
pip3 install -r requirements.txt
cd ~/catkin_ws && catkin_make

# 4. 启动
roslaunch two_stage_infer two_stage.launch

# 5. 验证（另一个终端）
rostopic echo /target_info
```

---

## 📊 性能指标

### 推理延迟（N150 CPU）

| 操作 | 耗时 |
|------|------|
| 图像读取 | ~5ms |
| 检测 (Stage 1) | ~60ms |
| Crop 提取 | ~5ms |
| 分类 (Stage 2) | ~30ms |
| 输出格式化 | ~2ms |
| **总计/帧** | **~100ms (10 FPS)** |

*注: 可通过降低 imgsz 参数进一步优化*

### 硬件占用

- **内存**: ~500MB (模型 + 运行时)
- **CPU**: 1-2 核心 (可配置 taskset 绑定)
- **带宽**: ~50Mbps (1080p @ 30FPS 视频传输)

---

## 📂 文件组织

```
.
├── DEPLOYMENT_GUIDE.md        # 详细部署文档
├── QUICK_START.md            # 快速开始（本文件）
├── requirements.txt           # Python 依赖
├── organize_models.py        # 模型组织脚本
├── test_local.py            # 本地测试工具
│
├── models/                   # OpenVINO 模型
│   ├── target_detector_openvino/
│   └── cifar100_classifier_openvino/
│
└── two_stage_infer/          # ROS 包
    ├── scripts/
    │   └── infer_node.py     # 核心推理节点
    ├── launch/
    │   └── two_stage.launch
    ├── package.xml
    └── CMakeLists.txt
```

---

## 🔧 配置参数

### `two_stage.launch` 中可调参数

```xml
<param name="det_model_path" value="~/uav_models/target_detector_openvino" />
<param name="cls_model_path" value="~/uav_models/cifar100_classifier_openvino" />
<param name="conf_threshold" value="0.25" />        <!-- 检测置信度阈值 -->
<param name="device" value="cpu" />                <!-- 'cpu' 或 '0' (GPU) -->
<param name="camera_topic" value="/usb_cam/image_raw" />
```

### 性能调优建议

| 场景 | 推荐配置 |
|------|---------|
| **实时性优先** | imgsz=320, conf=0.3 |
| **准确性优先** | imgsz=640, conf=0.25 |
| **功耗优先** | imgsz=256, conf=0.4, 每 2 帧推理一次 |

---

## 🧪 测试与验证

### 本地测试（无人机）

```bash
# 使用视频文件
python3 test_local.py \
  --det-model models/target_detector_openvino \
  --cls-model models/cifar100_classifier_openvino \
  --source test_video.mp4 \
  --output results/

# 使用图片文件夹
python3 test_local.py \
  --det-model models/target_detector_openvino \
  --cls-model models/cifar100_classifier_openvino \
  --source /path/to/images/ \
  --type images \
  --output results/
```

### 无人机端验证

```bash
# 检查模型加载
python3 -c "from ultralytics import YOLO; YOLO('~/uav_models/target_detector_openvino')"

# 查看 ROS 话题
rostopic echo /target_info
rostopic hz /target_info

# 监控系统资源
top -p $(pgrep -f infer_node.py)
```

---

## 🐛 常见问题

| 问题 | 解决方案 |
|------|---------|
| 摄像头找不到 | 检查 `/dev/video*`，修改 launch 文件 video_device 参数 |
| 模型加载失败 | `pip install ultralytics openvino` |
| 推理速度慢 | 降低 imgsz，检查 CPU 占用，绑定 CPU 核心 |
| 话题无数据输出 | 检查摄像头是否正常工作，查看节点日志 |
| 分类准确度低 | 验证原始 CIFAR 分类器准确度 (0.7897) |

---

## 📈 后续优化方向

- [ ] INT8 量化以进一步提速
- [ ] 动态分辨率调整
- [ ] 与地面站 MAVLink 集成
- [ ] 多线程异步推理
- [ ] 模型微调以适配实际靶标

---

## 📖 更多资料

- [OpenVINO 官方文档](https://docs.openvino.ai/)
- [Ultralytics YOLOv8 文档](https://docs.ultralytics.com/)
- [ROS Noetic 教程](http://wiki.ros.org/noetic)

---

**最后更新**: 2024年  
**维护者**: UAV AI Team
