# electromagnet_control

面向外部复用者的电磁铁控制包。

这个包的目标很单一：

- 统一封装通过 **MAVROS `CommandLong` + `MAV_CMD_DO_SET_ACTUATOR`**
  控制电磁铁输出的逻辑

它不负责：

- 轨迹规划
- 释放时序决策
- 抛投几何优化

这些应由上层节点决定。这个包只负责“怎么发控制命令”。

---

## 1. 包含哪些内容

### 1.1 共享接口类

- `include/electromagnet_control/electromagnet_controller.h`
- `src/electromagnet_controller.cpp`

核心类：

- `electromagnet_control::ElectromagnetController`

### 1.2 最小测试节点

- `src/electromagnet_test_node.cpp`

### 1.3 测试 launch

- `launch/electromagnet_test.launch`

---

## 2. 适合什么场景

适合这些场景：

1. 你已经有自己的 FSM / 控制器 / 任务节点
2. 你只需要一个独立的“电磁铁控制模块”
3. 你希望统一管理：
   - slot 映射
   - hold / release 值
   - 持续重发
   - MAVROS 错误处理

---

## 3. 当前控制方式

本包当前使用：

- 服务：`/mavros/cmd/command`
- 消息：`mavros_msgs/CommandLong`
- 命令号：`MAV_CMD_DO_SET_ACTUATOR = 187`

slot 与参数的对应关系：

- slot 1 -> `param1`
- slot 2 -> `param2`
- slot 3 -> `param3`
- slot 4 -> `param4`
- slot 5 -> `param5`
- slot 6 -> `param6`

---

## 4. 核心接口

头文件：

- `include/electromagnet_control/electromagnet_controller.h`

主要接口如下：

```cpp
electromagnet_control::ElectromagnetController controller;

controller.init(nh);

controller.sendHold(slot);
controller.sendRelease(slot);

controller.streamOutputs(now, released_slots);

int slot = controller.getReleaseSlot(release_order);
```

### 4.1 `init(...)`

作用：

- 读取 `magnet/...` 参数
- 建立 `/mavros/cmd/command` client
- 初始化运行时状态

### 4.2 `getReleaseSlot(release_order)`

作用：

- 根据第几次释放，返回应该使用的 actuator slot

例如：

- `magnet/actuator_slots = [3,4,2,1]`

则：

- 第 1 次释放 -> slot 3
- 第 2 次释放 -> slot 4
- 第 3 次释放 -> slot 2
- 第 4 次释放 -> slot 1

### 4.3 `sendHold(slot)` / `sendRelease(slot)`

作用：

- 立即发送一次 hold 或 release 指令

### 4.4 `streamOutputs(now, released_slots)`

作用：

- 周期性重发输出
- 未释放的 slot 发 `hold_value`
- 已释放的 slot 发 `release_value`（若 `hold_low=true`）

这一步很重要，因为很多飞控/链路场景下，单发一次 release 不够稳。

---

## 5. 参数说明

本包读取这些参数：

- `magnet/enable`
- `magnet/actuator_slot`
- `magnet/actuator_slots`
- `magnet/actuator_set_index`
- `magnet/release_value`
- `magnet/hold_value`
- `magnet/hold_low`
- `magnet/release_resend_period`
- `magnet/error_throttle_sec`
- `magnet/max_fail_count`
- `magnet/disable_on_fail`
- `release_order_index`

### 5.1 最常用参数

#### `magnet/actuator_slots`

类型：

- `int[]`

示例：

```yaml
magnet/actuator_slots: [3, 4, 2, 1]
```

#### `magnet/hold_value`

示例：

```yaml
magnet/hold_value: 1.0
```

#### `magnet/release_value`

示例：

```yaml
magnet/release_value: -1.0
```

#### `magnet/actuator_set_index`

示例：

```yaml
magnet/actuator_set_index: 0
```

---

## 6. 最小测试方法

### 6.1 启动测试节点

```bash
roslaunch electromagnet_control electromagnet_test.launch
```

默认：

- `enable_magnet=true`
- `test_slot=3`

### 6.2 发送 release

```bash
rostopic pub -1 /electromagnet_test_node/release std_msgs/Empty "{}"
```

### 6.3 发送 hold

```bash
rostopic pub -1 /electromagnet_test_node/hold std_msgs/Empty "{}"
```

这个测试节点适合验证：

- MAVROS 服务是否通
- slot 配置是否正确
- hold / release 值是否符合飞控映射

---

## 7. 外部节点如何接入

推荐模式：

### 7.1 在你的节点里持有一个 controller

```cpp
electromagnet_control::ElectromagnetController magnet_controller_;
```

### 7.2 启动时初始化

```cpp
electromagnet_control::ElectromagnetController::InitOptions options;
options.enable_default = true;
magnet_controller_.init(nh, options);
```

### 7.3 你自己的 FSM 只负责“时序”

例如：

- 某个时刻需要释放
- 调：

```cpp
int slot = magnet_controller_.getReleaseSlot(release_order);
magnet_controller_.sendRelease(slot, "release");
```

### 7.4 周期回调里做 streaming

```cpp
magnet_controller_.streamOutputs(ros::Time::now(), released_slots_);
```

---

## 8. 推荐集成方式

建议把系统分成两层：

### 上层：时序层

你自己的节点负责：

- 何时 hold
- 何时 release
- 第几次释放

### 下层：执行层

`ElectromagnetController` 负责：

- 发 MAVROS 命令
- 管理 slot
- 管理周期重发
- 处理错误日志节流

这样耦合最低。

---

## 9. 相关限制

当前接口假设：

1. 使用的是 MAVROS
2. 飞控支持 `MAV_CMD_DO_SET_ACTUATOR`
3. 输出口映射已经在飞控侧正确配置

如果你的平台改成：

- 直接 PWM
- `/mavros/actuator_control`
- 串口自定义板卡

那只需要改这个包，不需要改上层时序逻辑。

---

## 10. 和本仓库其他部分的关系

本包只处理“电磁铁控制”。

以下内容不在本包里：

- 释放点偏置模型
- 抛投落点预测
- 轨迹优化
- RViz 可视化

这些还在：

- `plan_manage`
- `manual_throw`
- `traj_opt`

---

## 11. 建议的交付方式

如果你要把“控制电磁铁的代码”交给别人，建议直接给：

1. `src/utils/electromagnet_control/`
2. 这份 `README.md`
3. 根目录的：
   - `ELECTROMAGNET_CONTROL_CN.md`

这样别人既能直接复用，也能快速理解。

