# CRAIC Micro UAV Autonomous Flight System

本仓库用于第二十八届中国机器人及人工智能大赛（CRAIC）微型无人机赛项的自主飞行系统开发与实机集成。系统面向室内微型无人机场景，围绕定位、路径规划、目标识别、任务状态机、投放控制与飞控执行构建完整任务流程。

当前基线运行方式为：

```bash
./run.sh [orbit_laps]
```

随后通过 ROS 话题发布开始信号，触发自主任务流程。

---

## 1. Project Overview

本项目的目标是让微型无人机在室内比赛场地中完成如下自主任务：

1. 自主起飞；
2. 识别二维码内容，获得图片靶类别与降落方向；
3. 飞行至障碍物区域并顺时针绕障；
4. 根据二维码类别识别图片靶并执行投放；
5. 识别带信号灯的特殊靶并执行投放；
6. 识别并穿越位置不固定的圆环；
7. 根据二维码中的 left / right 信息选择正确降落点并降落。

系统采用 ROS Noetic 作为中间件，结合 Odin 室内定位、MAVROS、PX4CTRL、SUPER 规划器、视觉识别模块和电磁铁投放模块，实现从感知到决策再到飞控执行的闭环流程。

---

## 2. System Architecture

整体系统可以概括为：

```text
Odin Positioning / Camera Sensors
              |
              v
State Estimation & ROS Adapter
              |
              v
Mission State Machine
              |
      +-------+--------+
      |                |
      v                v
Path Planning       Vision Inference
SUPER Planner       QR / Target / Ring / Special Target
      |                |
      +-------+--------+
              |
              v
PX4CTRL / MAVROS / Actuator Control
              |
              v
Micro UAV Autonomous Flight
```

核心数据流包括：

* Odin 定位数据转换为 PX4CTRL / MAVROS 可用的里程计或位姿输入；
* 任务状态机根据当前任务阶段发布航点或规划目标；
* SUPER 负责实际路径规划与避障执行；
* 视觉模块输出二维码、图片靶、特殊靶和圆环检测结果；
* 电磁铁控制模块根据任务状态机指令释放对应物块；
* PX4CTRL / MAVROS 将控制指令下发至飞控。

---

## 3. Repository Structure

```text
CRAIC/
├── run.sh
├── grep_ekf.sh
├── .gitignore
└── src/
    ├── camera/
    │   ├── uav_deployment/
    │   ├── special_target_detection_n150/
    │   ├── ring/
    │   └── uav-4-class/
    │
    ├── craic_odin_px4_adapter/
    │   ├── config/
    │   ├── launch/
    │   ├── scripts/
    │   └── docs/
    │
    ├── electromagnet_control/
    ├── odin_ros_driver/
    ├── px4ctrl/
    ├── super/
    └── uav_environment_ros/
```

主要模块说明：

| 模块                                     | 作用                                    |
| -------------------------------------- | ------------------------------------- |
| `run.sh`                               | 当前实机基线启动脚本，一键启动主要 ROS 节点              |
| `craic_odin_px4_adapter`               | Odin 定位、PX4/MAVROS/PX4CTRL 适配与比赛任务状态机 |
| `camera/uav_deployment`                | 图片靶两阶段识别系统，包含检测与分类推理                  |
| `camera/special_target_detection_n150` | 特殊靶检测模块                               |
| `camera/ring`                          | 圆环检测与穿环相关模块                           |
| `electromagnet_control`                | 电磁铁投放控制封装                             |
| `odin_ros_driver`                      | Odin 相机/定位驱动                          |
| `px4ctrl`                              | 四旋翼控制器                                |
| `super`                                | 路径规划与避障模块                             |
| `uav_environment_ros`                  | 无人机环境与消息相关依赖                          |

---

## 4. Runtime Baseline

当前推荐基线为：

```text
run.sh + rostopic pub 启动自主任务
```

`run.sh` 会依次启动：

1. Odin ROS driver；
2. MAVROS；
3. Odin 到 MAVROS/PX4CTRL 的位姿适配节点；
4. PX4CTRL；
5. SUPER planner；
6. CRAIC 2026 mission runner；
7. SUPER 可视化。

任务 runner 默认处于暂停状态，等待外部开始信号。

---

## 5. Build

推荐将本仓库作为一个 catkin workspace 使用。

```bash
cd ~/CRAIC_ws
catkin_make
source devel/setup.bash
```

如果使用 zsh 或 bash 自动加载环境，也可以将以下内容加入 shell 配置文件：

```bash
source ~/CRAIC_ws/devel/setup.bash
```

---

## 6. Run

### 6.1 启动完整系统

在仓库根目录执行：

```bash
cd ~/CRAIC_ws
chmod +x run.sh
./run.sh
```

如果需要设置绕障圈数，可以传入正整数参数：

```bash
./run.sh 1
./run.sh 2
```

其中参数表示 `orbit_laps`。

### 6.2 开始自主任务

`run.sh` 启动后，任务状态机默认等待开始信号。确认无人机、遥控器、定位、飞控和安全接管均准备完毕后，在另一个终端执行：

```bash
cd ~/CRAIC_ws
source devel/setup.bash

rostopic pub -1 /craic2026/mission/start std_msgs/Bool "data: true"
```

### 6.3 暂停、继续与中止

暂停任务：

```bash
rostopic pub -1 /craic2026/mission/pause std_msgs/Bool "data: true"
```

继续任务：

```bash
rostopic pub -1 /craic2026/mission/resume std_msgs/Bool "data: true"
```

中止任务：

```bash
rostopic pub -1 /craic2026/mission/abort std_msgs/Bool "data: true"
```

---

## 7. Important ROS Topics

常用检查话题如下：

| 话题                                   | 作用                     |
| ------------------------------------ | ---------------------- |
| `/craic2026/mission/status`          | 比赛任务状态机状态输出            |
| `/craic2026/mission/start`           | 任务开始信号                 |
| `/craic2026/mission/pause`           | 任务暂停信号                 |
| `/craic2026/mission/resume`          | 任务继续信号                 |
| `/craic2026/mission/abort`           | 任务中止信号                 |
| `/mavros/state`                      | MAVROS 与飞控连接/模式状态      |
| `/mavros/local_position/pose`        | MAVROS 本地位姿            |
| `/odin/odom_for_px4ctrl`             | 提供给 PX4CTRL 的 Odin 里程计 |
| `/target_info`                       | 图片靶检测与分类结果             |
| `/special_target/result`             | 特殊靶检测结果                |
| `/ring_detector/center_odom_latched` | 圆环中心估计结果               |
| `/px4ctrl/takeoff_land`              | PX4CTRL 起飞/降落控制话题      |

也可以使用仓库中的脚本快速查看 MAVROS 本地位姿：

```bash
./grep_ekf.sh
```

---

## 8. Mission Configuration

当前比赛任务配置位于：

```text
src/craic_odin_px4_adapter/config/craic2026_mission_no_drop_odom_relative_yflip/
```

其中包含：

```text
mission_00_global.yaml
mission_01_takeoff_qr.yaml
mission_02_orbit.yaml
mission_03_scan_targets.yaml
mission_04_special.yaml
mission_05_ring.yaml
mission_06_land.yaml
mission_main.yaml
```

任务状态机按照 `mission_main.yaml` 中定义的顺序加载各阶段任务。每个任务文件描述该阶段的航点、动作和保持时间等参数。

注意：当前目录名中包含 `no_drop`，这是历史命名。实际是否启用投放由 launch 参数和运行参数决定，不能仅根据目录名判断。

---

## 9. Vision Modules

### 9.1 图片靶识别

图片靶识别采用两阶段视觉流程：

1. 检测阶段：定位图片靶区域；
2. 分类阶段：对靶标内图像进行类别识别。

输出结果用于任务状态机判断是否到达目标靶，并触发对应投放逻辑。

### 9.2 特殊靶识别

特殊靶模块用于检测带有信号灯的特殊靶。任务状态机在特殊靶任务阶段启动检测，并在满足确认条件后执行投放。

### 9.3 圆环识别与穿越

圆环模块用于估计圆环中心位置，并向任务状态机提供可用于穿环的空间目标点。任务状态机根据圆环中心生成接近点、穿越点和退出点，完成闭环穿越流程。

---

## 10. Electromagnet Drop Control

`electromagnet_control` 模块用于封装电磁铁释放控制逻辑。上层任务状态机只需要调用释放服务，不直接处理底层 MAVROS actuator command 细节。

当前投放流程由任务状态机根据二维码识别结果、视觉检测结果和投放顺序决定。

---

## 11. Safety Notes

本仓库涉及实机飞行，运行前必须确认：

1. 遥控器已连接，且具备随时接管能力；
2. 飞控、电池、电机、电调和桨叶保护装置状态正常；
3. Odin 定位稳定，位姿方向和坐标系符合当前配置；
4. MAVROS 与飞控连接正常；
5. PX4CTRL 状态正常；
6. 规划器输出正常，目标点没有明显越界；
7. 场地内无人员进入危险区域；
8. 投放机构固定可靠，物块不会意外脱落；
9. 起飞前先进行低风险单模块测试，再运行完整任务。

建议实机调试顺序：

```text
定位检查 -> MAVROS 检查 -> PX4CTRL 检查 -> 单点悬停 -> 单段任务 -> 完整流程
```

不要在未验证定位、控制和接管链路的情况下直接运行完整自主流程。

---

## 12. Development Notes

常见开发与验证命令：

```bash
# 编译
catkin_make

# 加载环境
source devel/setup.bash

# 查看话题列表
rostopic list

# 查看话题频率
rostopic hz <topic_name>

# 查看任务状态
rostopic echo /craic2026/mission/status

# 查看 MAVROS 状态
rostopic echo /mavros/state

# 查看本地位姿
rostopic echo /mavros/local_position/pose
```

---

## 13. Current Status

当前仓库处于比赛开发与实机集成阶段，重点目标是保证完整任务链路能够稳定运行。仓库中的部分路径、参数和任务配置与实际比赛场地、Odin 坐标系、机载电脑路径和硬件接线相关，迁移到其他无人机平台时需要重新标定和配置。

---

## 14. Maintainer

Maintained by SYSU HILAB 智工小趴虎队.
Members : JIWEN QIU、KAIYUAN YANG、QIYA YANG
