# PX4ctrl

一个基于 ROS 的 PX4 飞控控制器，通过 `mavros` 发送 PVA（位置/速度/加速度）指令实现无人机控制。

## 核心功能

- **有限状态机控制**：5 种状态（手动、悬停、指令控制、自动起飞、自动降落）
- **PVA 控制**：发送位置、速度、加速度和 yaw 指令到 PX4 飞控
- **遥控器支持**：通过 RC 通道切换控制模式
- **自动起降**：支持自动起飞和降落功能
- **无遥控飞行**：可选的无遥控器操作模式
- **坐标系定义**：前 x、左 y、上 z（与 ROS 标准一致）

## 依赖项

- ROS (Noetic)
- mavros
- Eigen3
- quadrotor_msgs (包含在 uav_environment_ros 包中)
- uav_utils (包含在 uav_environment_ros 包中)

## 安装与编译

1. 将 px4ctrl 和 uav_environment_ros 包放入 catkin 工作空间：
   ```bash
   cd ~/catkin_ws/src
   git clone https://gitee.com/LJN2017117574/px4ctrl.git
   git clone https://gitee.com/LJN2017117574/uav_environment_ros.git
   ```

2. 安装依赖（如果先前未安装）：
   ```bash
   sudo apt-get install ros-<distro>-mavros ros-<distro>-mavros-extras
   wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
   sudo ./install_geographiclib_datasets.sh
   ```

3. 编译：
   ```bash
   cd ~/catkin_ws
   catkin_make
   ```

## 快速开始


<span style="color: red; font-weight: bold;">确保飞控能够具备可靠的定位信息（如GNSS，RTK，VIO，LIO）并能在位置模式保持稳定</span>

**如果使用MoCap、VIO、LIO等定位数据，确保具备有效的消息转发，见[odom_to_mavros](https://gitee.com/LJN2017117574/odom_to_mavros)**

1. 启动 mavros 和你的定位模块：
   ```bash
   roslaunch mavros px4.launch
   roslaunch <your_odometry_node>
   ```

2. 启动 px4ctrl：
   ```bash
   roslaunch px4ctrl run_ctrl.launch
   ```

3. 操作流程：
   - 飞控未解锁时，拨动通道 5 切换到悬停模式
   - 拨动通道 6 允许接收控制指令
   - 手动起飞到合适高度
   - 拨动通道 5 切换到自动悬停模式
   - 拨动通道 6 切换到指令控制模式

## 坐标系定义

- **位置 (position)**: 前 x、左 y、上 z
- **速度 (velocity)**: 世界坐标系下的速度（非机体系）
- **姿态 (orientation)**: 四元数表示，yaw 角定义为绕 z 轴的旋转
- **控制指令**: 位置、速度、加速度、jerk、yaw、yaw_rate

注意：如果你的速度数据是在机体系下的，需要修改 `input.cpp` 中的 `#define VEL_IN_BODY` 为 1。

## 配置说明

配置文件位于 `config/ctrl_param_fpv.yaml`：

### 控制参数
- `ctrl_freq_max`: 控制频率 (默认 100Hz)
- `manual_ctrl_mode`: 手动控制模式 (0: 基于悬停位姿积分, 1: 简单位置控制)
- `max_manual_vel`: 最大手动速度 (m/s)
- `max_manual_vel_z`: 最大垂直速度 (m/s)
- `max_manual_yaw_rate`: 最大偏航角速度 (rad/s)
- `manual_ctrl_lookahead_time`: 位置控制模式前瞻时间 (s)

### 遥控器反向设置
- `rc_reverse.roll`: 横滚反向
- `rc_reverse.pitch`: 俯仰反向
- `rc_reverse.yaw`: 偏航反向
- `rc_reverse.throttle`: 油门反向

### 自动起降参数
- `auto_takeoff_land.enable`: 启用自动起降
- `auto_takeoff_land.enable_auto_arm`: 启用自动解锁
- `auto_takeoff_land.no_RC`: 无遥控器模式
- `auto_takeoff_land.takeoff_height`: 起飞高度 (米)
- `auto_takeoff_land.takeoff_land_speed`: 起降速度 (m/s)

### 消息超时
- `msg_timeout.odom`: 里程计超时 (秒)
- `msg_timeout.rc`: 遥控器超时 (秒)
- `msg_timeout.cmd`: 指令超时 (秒)

### 电池参数（用于在终端进行简单的电量估计）
- `battery.series`: 电池串联数
- `battery.cell_full_voltage`: 单电芯满电电压 (V)
- `battery.cell_cutoff_voltage`: 单电芯截止电压 (V)

> 在电量为50%-100%终端显示为绿色，25%-49%显示为黄色，低于25%显示为红色

## Topic 说明

### 订阅的 Topic
- `/mavros/state`: 飞控状态
- `/mavros/extended_state`: 扩展状态
- `~odom`: 里程计数据 (remap 到你的定位模块)
- `~cmd`: 控制指令 (remap 到你的规划器)
- `/mavros/rc/in`: 遥控器输入
- `~takeoff_land`: 起降指令
- `/mavros/battery`: 电池状态

### 发布的 Topic
- `/mavros/setpoint_raw/local`: PVA 控制指令
- `/traj_start_trigger`: 轨迹启动触发信号

### 服务
- `/mavros/set_mode`: 切换飞行模式
- `/mavros/cmd/arming`: 解锁/锁桨
- `/mavros/cmd/command`: 飞控重启

## 遥控器通道设置

- **通道 5**: 飞行模式切换 (手动/自动)
- **通道 6**: 控制指令使能/禁止
- **通道 7**: 紧急停桨
- **通道 8**: 飞控重启 (可选)

## 发送控制指令

控制指令使用 `quadrotor_msgs::PositionCommand` 消息类型。示例：

```cpp
#include <quadrotor_msgs/PositionCommand.h>

quadrotor_msgs::PositionCommand cmd;
cmd.header.stamp = ros::Time::now();
cmd.position.x = 1.0;
cmd.position.y = 0.0;
cmd.position.z = 1.0;
cmd.velocity.x = 0.0;
cmd.velocity.y = 0.0;
cmd.velocity.z = 0.0;
cmd.acceleration.x = 0.0;
cmd.acceleration.y = 0.0;
cmd.acceleration.z = 0.0;
cmd.yaw = 0.0;
cmd.yaw_rate = 0.0;

cmd_pub.publish(cmd);
```

## 故障排除

### 无法切换到悬停模式
- 检查里程计数据是否正常
- 确保没有发送控制指令
- 检查速度是否过大 (>3.0 m/s)

### 切换到悬停后飞机移动
- 检查遥控器油门杆是否回中
- 调整 `max_manual_vel` 参数
- 检查定位模块是否漂移

### 自动起飞失败
- 确保飞机处于解锁状态
- 检查起降高度设置

### 无法解锁
- 检查遥控器连接
- 确保电池电压正常
- 检查飞控状态


## 许可证

GPLv3 - 详见 LICENSE 文件


## 相关项目

- [uav_environment_ros](https://gitee.com/LJN2017117574/uav_environment_ros): 包含 `quadrotor_msgs` 和 `uav_utils` 依赖包