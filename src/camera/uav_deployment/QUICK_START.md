# 快速开始指南 - 两段式无人机目标检测系统

## 🚀 30秒快速部署

### 本地（开发机器）

1. **组织模型文件**
   ```bash
   cd uav_deployment
   python3 organize_models.py --workspace ..
   ```

2. **本地测试**（可选）
   ```bash
   python3 test_local.py \
     --det-model models/target_detector_openvino \
     --cls-model models/cifar100_classifier_openvino \
     --source /path/to/test/video.mp4 \
     --output results/
   ```

### 无人机端（N150 + ROS Noetic）

1. **传输文件**
   ```bash
   # 从本地开发机传输
   scp -r uav_deployment/models uav@192.168.1.100:~/uav_models/
   scp -r uav_deployment/two_stage_infer uav@192.168.1.100:~/catkin_ws/src/
   scp uav_deployment/requirements.txt uav@192.168.1.100:~
   ```

2. **安装依赖**
   ```bash
   ssh uav@192.168.1.100
   pip3 install -r ~/requirements.txt
   cd ~/catkin_ws && catkin_make
   ```

3. **启动系统**
   ```bash
   roslaunch two_stage_infer two_stage.launch
   ```

4. **查看结果**
   ```bash
   # 新终端
   rostopic echo /target_info
   ```

---

## 📂 项目结构

```
uav_deployment/
├── DEPLOYMENT_GUIDE.md          # 详细部署指南
├── QUICK_START.md              # 本文件
├── README.md                   # 项目概述
├── requirements.txt             # Python 依赖
├── organize_models.py          # 模型组织脚本
├── test_local.py              # 本地测试脚本
│
├── models/                     # OpenVINO 模型（大文件，部署时）
│   ├── target_detector_openvino/
│   │   ├── best.xml
│   │   ├── best.bin
│   │   └── best.mapping
│   └── cifar100_classifier_openvino/
│       ├── best.xml
│       ├── best.bin
│       └── best.mapping
│
└── two_stage_infer/           # ROS 包
    ├── CMakeLists.txt
    ├── package.xml
    ├── scripts/
    │   └── infer_node.py      # 核心推理节点
    └── launch/
        └── two_stage.launch   # 启动配置
```

---

## 🔧 常见问题

### Q: 本地测试失败，提示找不到模型
**A**: 运行 `python3 organize_models.py` 组织模型到 `models/` 目录

### Q: 无人机上 `/dev/video0` 不存在
**A**: 运行 `ls /dev/video*` 检查实际设备号，修改 `two_stage.launch` 中的 `video_device` 参数

### Q: 推理速度太慢（延迟 > 500ms）
**A**: 在 `two_stage.launch` 中降低 `imgsz` 参数（如从 640 降到 320）

### Q: 分类精度不理想
**A**: 检查 CIFAR 100 分类器的原始准确率：
```bash
python3 -c "from ultralytics import YOLO; m = YOLO('runs/classify/...'); print(m.info())"
```

---

## 📊 系统性能参考

| 指标 | 参考值 |
|------|-------|
| 检测模型 | YOLOv8n (3.0M 参数) |
| 分类模型 | YOLOv8n-cls (1.5M 参数) |
| 检测器 FLOPs | 8.1 GFLOPs @ 640×640 |
| 分类器 FLOPs | 3.4 GFLOPs @ 128×128 |
| 单帧检测时间 | ~50-100ms (N150 CPU) |
| 单个分类时间 | ~20-50ms (N150 CPU) |
| 总推理延迟 | ~100-150ms (包括 crop 提取) |

---

## 📡 ROS 话题接口

### 发布话题

**`/target_info`** (String)
```json
{
  "box_index": 0,
  "detection_conf": 0.95,
  "class_id": 5,
  "class_name": "apple",
  "class_conf": 0.87,
  "center_x": 320,
  "center_y": 240,
  "box": [250, 180, 390, 300]
}
```

**`/target_coords`** (Float32MultiArray)
```
[center_x, center_y, det_conf, cls_conf]
```

**`/debug_image`** (Image)
- 带框与标签的标注图像

### 订阅话题

**`/usb_cam/image_raw`** (Image)
- USB 摄像头输入

---

## 🎯 下一步

- [ ] 在本地测试视频上运行 `test_local.py`
- [ ] 组织模型到 `models/` 目录
- [ ] 传输到无人机
- [ ] 安装依赖并编译 ROS 包
- [ ] 启动系统并验证话题输出
- [ ] 配置 systemd 自启动（见完整指南）
- [ ] 与地面站集成（发送坐标）

---

更多详细内容请参考 [DEPLOYMENT_GUIDE.md](DEPLOYMENT_GUIDE.md)
