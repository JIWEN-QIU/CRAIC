# ✅ 部署前检查清单

> **用途**: 确保所有文件已正确准备，可安全传输到无人机

---

## 📋 本地检查 (开发机器)

### 模型导出验证
- [x] 检测器 OpenVINO 导出成功 (11.8 MB)
- [x] 分类器 OpenVINO 导出成功 (6.1 MB)
- [x] 两个模型都包含 `.xml`, `.bin`, `metadata.yaml` 文件

### 部署文件完整性
- [x] `infer_node.py` - ROS 节点脚本 (8.01 KB)
- [x] `package.xml` - ROS 包元数据
- [x] `CMakeLists.txt` - ROS 编译配置
- [x] `two_stage.launch` - ROS 启动配置
- [x] `requirements.txt` - Python 依赖 (17 个包)
- [x] `README.md` - 项目文档
- [x] `DEPLOYMENT_GUIDE.md` - 详细部署指南
- [x] `QUICK_START.md` - 快速开始
- [x] `test_local.py` - 本地测试脚本

### 文件夹结构
```
uav_deployment/
├── models/
│   ├── target_detector_openvino/      ✅ 11.8 MB
│   └── cifar100_classifier_openvino/   ✅ 6.1 MB
├── two_stage_infer/                  ✅ ROS 包
├── *.md 文档                          ✅ 
├── *.py 脚本                          ✅
└── requirements.txt                   ✅
```

**总包大小**: ~18 MB (可压缩至 ~12 MB)

---

## 📤 传输前准备

### 网络传输 (SCP)
```bash
# 检查无人机连接
ping 192.168.1.100

# 预留目录
ssh uav@192.168.1.100 'mkdir -p ~/uav_models ~/catkin_ws/src'

# 传输模型 (~18 MB, 预计 1-5 分钟)
scp -r uav_deployment/models uav@192.168.1.100:~/uav_models/

# 传输 ROS 包
scp -r uav_deployment/two_stage_infer uav@192.168.1.100:~/catkin_ws/src/

# 传输依赖清单
scp uav_deployment/requirements.txt uav@192.168.1.100:~
```

### USB/SD 卡传输
- [ ] 检查 U 盘/SD 卡容量 > 100 MB
- [ ] 复制以下目录到 U 盘:
  - `uav_deployment/models/`
  - `uav_deployment/two_stage_infer/`
  - `uav_deployment/requirements.txt`

---

## 🖥️ 无人机端安装检查

### 环境验证
```bash
# SSH 进入无人机
ssh uav@192.168.1.100

# 检查 Python 版本 (应 >= 3.8)
python3 --version

# 检查 ROS Noetic 安装
rosnode list
```

### 依赖安装
```bash
# 安装 Python 包 (~5-10 分钟)
pip3 install -r ~/requirements.txt -i https://mirrors.aliyun.com/pypi/simple/

# 验证关键包
python3 -c "from ultralytics import YOLO; from openvino.runtime import Core; print('✓ Ready')"
```

### 文件位置验证
```bash
ls -la ~/uav_models/target_detector_openvino/    # 检查检测器
ls -la ~/uav_models/cifar100_classifier_openvino/ # 检查分类器
ls -la ~/catkin_ws/src/two_stage_infer/          # 检查 ROS 包
```

### ROS 编译
```bash
cd ~/catkin_ws
catkin_make
# 输出应包含: [100%] Built target ...
```

---

## 🧪 系统测试

### 模型加载测试
```bash
python3 << 'EOF'
from ultralytics import YOLO
print("Loading detector...")
det = YOLO(os.path.expanduser("~/uav_models/target_detector_openvino"))
print("Loading classifier...")
cls = YOLO(os.path.expanduser("~/uav_models/cifar100_classifier_openvino"))
print("✓ Models loaded successfully!")
EOF
```

### 摄像头验证
```bash
# 检查摄像头是否识别
ls -la /dev/video*

# 简单的 OpenCV 测试
python3 << 'EOF'
import cv2
cap = cv2.VideoCapture(0)
if cap.isOpened():
    ret, frame = cap.read()
    print(f"✓ Camera OK, frame shape: {frame.shape}")
else:
    print("✗ Camera not accessible")
cap.release()
EOF
```

### ROS 节点启动测试
```bash
# 终端 1: 启动 roscore
roscore &

# 终端 2: 启动节点
source ~/catkin_ws/devel/setup.bash
rosrun two_stage_infer infer_node.py

# 终端 3: 检查话题
rostopic list           # 应包含 /target_info, /target_coords, /debug_image
rostopic echo /target_info -n 1  # 应输出 JSON 格式结果
```

---

## 🚀 运行时检查清单

### 系统启动
- [ ] `roscore` 成功启动
- [ ] USB 摄像头驱动加载
- [ ] 两段式推理节点启动无错误
- [ ] ROS 话题正常发布

### 性能监控
```bash
# 监控 CPU 使用率
top -p $(pgrep -f infer_node.py)

# 监控推理延迟
rostopic hz /target_info

# 检查错误日志
rosnode info /two_stage_infer_node
```

### 推理输出示例
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

---

## ⚠️ 常见问题速查

| 问题 | 症状 | 快速修复 |
|------|------|---------|
| 模型不加载 | ImportError: openvino | `pip3 install openvino` |
| 摄像头无输入 | 节点运行但无检测 | `ls /dev/video*` 确认设备 |
| 话题无数据 | `rostopic echo` 卡住 | 检查节点状态 `rosnode info` |
| 性能差 | 延迟 > 500ms | 降低 imgsz 参数 (640→320) |
| 分类错误 | 预测全是同一类别 | 验证模型路径，检查 crop 提取 |

---

## 📦 部署包内容清单

### 文档 (4 个)
- `README.md` - 项目概述
- `DEPLOYMENT_GUIDE.md` - 详细指南
- `QUICK_START.md` - 快速开始
- `DEPLOYMENT_CHECKLIST.md` - 本文件

### 脚本 (3 个)
- `infer_node.py` - ROS 推理节点 [核心]
- `test_local.py` - 本地测试工具
- `organize_models.py` - 模型组织脚本

### 配置 (3 个)
- `requirements.txt` - Python 依赖
- `two_stage.launch` - ROS 启动配置
- `package.xml`, `CMakeLists.txt` - ROS 包元数据

### 模型 (2 个)
- `target_detector_openvino/` - 检测器 (11.8 MB)
- `cifar100_classifier_openvino/` - 分类器 (6.1 MB)

---

## ✨ 最终确认

- [x] 所有文件已生成并复制
- [x] OpenVINO 模型导出成功
- [x] ROS 包结构完整
- [x] 文档齐全
- [x] 部署脚本可用

**状态**: ✅ **准备就绪，可安全传输至无人机**

---

**下一步**: 
1. 根据网络情况选择传输方式 (SCP 或 U 盘)
2. 遵循 `DEPLOYMENT_GUIDE.md` 进行无人机端安装
3. 运行此清单中的验证命令确保系统正常
4. 启动完整系统 `roslaunch two_stage_infer two_stage.launch`

祝部署顺利！ 🎉
