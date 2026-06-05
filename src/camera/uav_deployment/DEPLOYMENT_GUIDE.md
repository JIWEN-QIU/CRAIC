# 无人机上部署两段式 YOLO 检测 + 分类系统

> **目标硬件**: N150 CPU, ROS Noetic, USB 摄像头  
> **模型优化**: OpenVINO 加速推理  
> **集成方式**: ROS 节点 + 话题通信

---

## 1. 准备阶段（本地训练机器）

### 1.1 模型已成功导出为 OpenVINO 格式

✅ **检测器**：  
```
runs/detect/runs/cifar100_yolo/cifar_target_detector_ft-2/weights/best_openvino_model/
```

✅ **分类器**：  
```
runs/classify/runs/cifar100_yolo/yolov8n_cls_cifar100/weights/best_openvino_model/
```

### 1.2 验证导出文件结构

每个导出文件夹应包含以下文件：
```
best_openvino_model/
├── best.xml      # 模型拓扑结构
├── best.bin      # 模型权重
└── best.mapping  # 输入输出映射
```

---

## 2. 传输文件到无人机

### 方案 A：通过 SCP（推荐，如果网络可用）
在本地训练机上执行：
```bash
# 假设无人机用户名为 uav，IP 为 192.168.1.100
scp -r runs/detect/runs/cifar100_yolo/cifar_target_detector_ft-2/weights/best_openvino_model \
    uav@192.168.1.100:~/uav_models/target_detector_openvino

scp -r runs/classify/runs/cifar100_yolo/yolov8n_cls_cifar100/weights/best_openvino_model \
    uav@192.168.1.100:~/uav_models/cifar100_classifier_openvino

# 传输 ROS 包和依赖
scp -r uav_deployment/two_stage_infer \
    uav@192.168.1.100:~/catkin_ws/src/

scp uav_deployment/requirements.txt \
    uav@192.168.1.100:~
```

### 方案 B：使用 USB 或 SD 卡（网络不可用时）
1. 在本地将以下文件夹复制到 USB/SD 卡：
   - `runs/detect/runs/cifar100_yolo/cifar_target_detector_ft-2/weights/best_openvino_model`
   - `runs/classify/runs/cifar100_yolo/yolov8n_cls_cifar100/weights/best_openvino_model`
   - `uav_deployment/two_stage_infer`

2. 在无人机上将 USB/SD 卡内容复制到：
   ```bash
   mkdir -p ~/uav_models ~/catkin_ws/src
   # 复制模型和 ROS 包
   ```

---

## 3. 无人机端部署

### 3.1 SSH 登录无人机
```bash
ssh uav@192.168.1.100  # 根据实际 IP 修改
```

### 3.2 安装 Python 依赖
```bash
cd ~
pip install -r requirements.txt -i https://mirrors.aliyun.com/pypi/simple/
# 或使用阿里云/清华源以加快速度
```

### 3.3 编译 ROS 包
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 3.4 验证模型加载
```bash
python3 << 'EOF'
from ultralytics import YOLO
print("Testing detector...")
det = YOLO("~/uav_models/target_detector_openvino")
print("Testing classifier...")
cls = YOLO("~/uav_models/cifar100_classifier_openvino")
print("✓ All models loaded successfully!")
EOF
```

### 3.5 验证 OpenVINO 是否可用
```bash
python3 -c "from openvino.runtime import Core; print(Core().available_devices)"
# 输出应包含 ['CPU']
```

---

## 4. 启动系统

### 4.1 启动 ROS Master（终端 1）
```bash
roscore
```

### 4.2 启动 USB 摄像头节点（终端 2）
```bash
source ~/catkin_ws/devel/setup.bash
roslaunch two_stage_infer two_stage.launch
```

或手动启动摄像头：
```bash
rosrun usb_cam usb_cam_node _video_device:=/dev/video0 _pixel_format:=mjpeg
```

### 4.3 启动两段式推理节点（终端 3）
```bash
source ~/catkin_ws/devel/setup.bash
rosrun two_stage_infer infer_node.py
```

或使用 launch 文件一次启动所有节点：
```bash
roslaunch two_stage_infer two_stage.launch
```

---

## 5. 验证系统运行

### 5.1 检查话题发布
```bash
# 在另一个终端查看推理结果
rostopic echo /target_info
# 输出示例：
# {"box_index": 0, "detection_conf": 0.95, "class_id": 5, "class_name": "apple", 
#  "class_conf": 0.87, "center_x": 320, "center_y": 240, "box": [250, 180, 390, 300]}
```

### 5.2 查看摄像头输入
```bash
rosrun image_view image_view image:=/usb_cam/image_raw
```

### 5.3 查看推理结果可视化
```bash
rosrun image_view image_view image:=/debug_image
```

### 5.4 监控节点日志
```bash
rosbag record /target_info -o inference_log.bag
# 后续回放分析
rosbag play inference_log.bag --topics /target_info
```

---

## 6. 性能监控与优化

### 6.1 获取推理延迟
在 `infer_node.py` 中添加计时（可选）：
```python
import time
start = time.time()
# 推理代码
elapsed = (time.time() - start) * 1000
logger.info(f"Inference time: {elapsed:.2f} ms")
```

### 6.2 检测器模型尺寸优化
若实时性不足，可在本地重新导出更小的模型：
```bash
# 使用 imgsz=320 而非 640 (下采样)
python -c "from ultralytics import YOLO; YOLO('best.pt').export(format='openvino', imgsz=320)"
```
修改无人机上的 launch 文件中的推理尺寸。

### 6.3 CPU 亲和性设置
若需要绑定 CPU 核心以提升性能：
```bash
taskset -c 0-3 rosrun two_stage_infer infer_node.py
```

---

## 7. 无人机端的系统化部署（systemd 守护服务）

### 7.1 创建 systemd 服务文件
```bash
sudo nano /etc/systemd/system/uav-infer.service
```

内容如下：
```ini
[Unit]
Description=UAV Two-Stage YOLO Inference
After=network.target ros-core.service

[Service]
Type=simple
User=uav
WorkingDirectory=/home/uav
ExecStart=/bin/bash -c 'source /home/uav/catkin_ws/devel/setup.bash && \
  roslaunch two_stage_infer two_stage.launch'
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
```

### 7.2 启用服务
```bash
sudo systemctl daemon-reload
sudo systemctl enable uav-infer.service
sudo systemctl start uav-infer.service

# 查看运行状态
sudo systemctl status uav-infer.service
```

---

## 8. 故障排除

| 问题 | 症状 | 解决方案 |
|------|------|--------|
| 模型加载失败 | `ModuleNotFoundError: ultralytics` | `pip install ultralytics` |
| OpenVINO 不可用 | `ImportError: openvino` | `pip install openvino openvino-dev` |
| 摄像头无输入 | `/dev/video0` 不存在 | `ls /dev/video*` 检查设备，修改 launch 文件中的设备编号 |
| ROS 话题无数据 | `rostopic echo /target_info` 无输出 | 检查节点日志 `rosnode info /two_stage_infer_node` |
| 推理速度慢 | 延迟 > 500ms | 降低 `imgsz`，检查 CPU 使用率 |
| 分类精度低 | 预测类别不准确 | 验证分类器训练指标，检查裁剪 crop 的有效性 |

---

## 9. 扩展功能

### 9.1 与地面站通信
修改 `infer_node.py` 的发布逻辑，通过 UDP/MAVLink 向地面站发送目标坐标：
```python
import socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto(result_json.encode(), ('192.168.1.10', 5005))
```

### 9.2 OTA 更新模型
在无人机上定期拉取最新模型：
```bash
# 定时任务 (crontab -e)
0 2 * * * cd ~/uav_models && git pull origin main
```

### 9.3 数据记录与离线分析
```bash
rosbag record /target_info /usb_cam/image_raw -o flight_$(date +%Y%m%d_%H%M%S).bag
```

---

## 10. 参考文件结构

```
~/
├── uav_models/
│   ├── target_detector_openvino/
│   │   ├── best.xml
│   │   ├── best.bin
│   │   └── best.mapping
│   └── cifar100_classifier_openvino/
│       ├── best.xml
│       ├── best.bin
│       └── best.mapping
│
├── catkin_ws/
│   └── src/
│       └── two_stage_infer/
│           ├── CMakeLists.txt
│           ├── package.xml
│           ├── scripts/
│           │   └── infer_node.py
│           └── launch/
│               └── two_stage.launch
│
└── requirements.txt
```

---

## 总结

1. ✅ 本地导出 OpenVINO 模型
2. ✅ 传输到无人机 `~/uav_models/`
3. ✅ 安装 Python 依赖 (`pip install -r requirements.txt`)
4. ✅ 编译 ROS 包 (`catkin_make`)
5. ✅ 启动系统 (`roslaunch two_stage_infer two_stage.launch`)
6. ✅ 验证话题发布 (`rostopic echo /target_info`)
7. ✅ （可选）配置 systemd 自启动

祝部署顺利！如有问题，检查日志输出或运行故障排除部分。
