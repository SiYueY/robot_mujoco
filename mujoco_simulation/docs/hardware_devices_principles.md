# MuJoCo 仿真设备原理与当前实现说明

本文档说明 `mujoco_simulation/hardware` 这一层如何将 MuJoCo 的原生对象包装成统一的仿真设备接口。重点不是介绍通用 MuJoCo 用法，而是解释当前仓库中各类设备的工作原理、代码抽象方式，以及已经实现和暂未实现的边界。

当前文档覆盖以下类型：

- `HardwareInterface`
- `HardwareManager`
- `Joint`
- `Imu`
- `Camera`
- `Lidar`
- `MobileBase`

## 总体架构

### `HardwareInterface` 的职责

`HardwareInterface<HardwareData, Command, State>` 是所有设备的统一抽象基类，约束每类设备都实现以下接口：

- `init(const HardwareData&)`
- `reset()`
- `write(const Command&)`
- `read(State&)`
- `last_error()`

这层的含义是：无论底层设备在 MuJoCo 中是 joint、sensor 还是 camera，只要要暴露给上层统一管理，就需要遵守同一生命周期和读写模式。

### `HardwareManager` 的职责

`HardwareManager` 按设备分类维护注册表，并对上层提供统一调度能力。当前按以下类别管理：

- `Joint`
- `Imu`
- `Camera`
- `Lidar`
- `MobileBase`

对每类设备，manager 负责：

- `register_*`
- `unregister_*`
- `read_*`
- `write_*`（如果该设备支持写）
- `read_all_*`

它本身不实现物理仿真，也不拥有控制策略，只负责持有 MuJoCo 上下文并调度具体设备对象。

### MuJoCo 的两个核心数据入口

设备层所有实现都建立在两个 MuJoCo 核心对象之上：

- `mjModel`
  - 静态模型定义
  - 包含 joint、actuator、sensor、camera 的名字、类型、地址和拓扑信息
- `mjData`
  - 运行时状态
  - 包含 `qpos`、`qvel`、`ctrl`、`sensordata`、`qfrc_*` 等动态数组

可以把设备层理解为“对 `mjModel + mjData` 的结构化访问包装”，而不是独立的物理引擎。

## Joint 原理

### MuJoCo 中 joint 与 actuator 的区别

这两个概念在 MuJoCo 里不是同一层：

- `joint`
  - 描述自由度本身
  - 决定系统如何运动
  - 典型类型有 `hinge`、`slide`、`ball`、`free`
- `actuator`
  - 描述如何施加控制输入
  - 决定 `ctrl` 的物理意义
  - 典型语义有 `motor`、`position`、`velocity`、custom 模式

简化理解：

- joint = “能怎么动”
- actuator = “怎么驱动它动”

### 当前 `Joint` 的实现边界

当前 `Joint` 是单关节、单标量状态/命令抽象，只支持：

- `JointType::Hinge`
- `JointType::Slide`

不支持：

- `JointType::Ball`
- `JointType::Free`

原因是当前 `JointState` / `JointCommand` 都是标量字段，只适用于 1-DoF joint。`ball` 和 `free` 在 MuJoCo 中对应多维 `qpos/qvel`，不适合直接复用当前接口。

### `JointType` 与 `ActuatorType`

当前实现会在 `init()` 时解析并缓存：

- `JointType`
  - 来源于 `mjModel::jnt_type`
- `ActuatorType`
  - 来源于 actuator 的 MuJoCo 配置和 bias 参数

其中：

- `JointType` 表示关节的运动学类型
- `ActuatorType` 表示驱动器的控制语义

`JointInfo` 中的 `CommandInterfaceType` 则表示当前上层希望用哪种命令接口控制该关节：

- `None`
- `Position`
- `Velocity`
- `Effort`

这三者不是一回事：

- `JointType` 是模型事实
- `ActuatorType` 是驱动事实
- `CommandInterfaceType` 是使用方式

### 关节状态与命令的 MuJoCo 映射

当前实现使用的核心数据通道如下：

| 语义 | MuJoCo 数据 |
| --- | --- |
| 关节位置 | `mjData::qpos` |
| 关节速度 | `mjData::qvel` |
| actuator 力/扭矩 | `mjData::qfrc_actuator` |
| 外加关节力 | `mjData::qfrc_applied` |
| 控制输入 | `mjData::ctrl` |

读状态时：

- `position <- qpos[qpos_address]`
- `velocity <- qvel[dof_address]`
- `effort <- qfrc_actuator[dof_address] + qfrc_applied[dof_address]`

写命令时：

- `Position` / `Velocity`
  - 写入 `ctrl[actuator_id]`
- `Effort`
  - 对 `Motor` / `Custom` actuator 写 `ctrl[actuator_id]`
  - 对 `Passive` joint 回退写 `qfrc_applied[dof_address]`

### 为什么被动关节走 `qfrc_applied`

如果关节没有绑定 actuator，当前实现会将其视为 `ActuatorType::Passive`。  
这类 joint 没有对应的 `ctrl` 通道，因此只能通过 `qfrc_applied` 直接向关节自由度施加外力。

### 当前实现边界

当前 `Joint` 明确不处理以下能力：

- PID
- transmission
- mimic joint
- 命令接口切换
- 多自由度 joint

这些能力如果后续要接入，需要在现有标量 `Joint` 抽象之上扩展新的层次，而不是继续向当前类内堆逻辑。

## IMU 原理

### MuJoCo 原理层

当前 IMU 不是独立的物理对象，而是由多个 MuJoCo `sensor` 组合出来的复合设备。  
实现依赖以下三类 sensor：

- `framequat`
- `gyro`
- `accelerometer`

它们的数据最终都进入 `mjData::sensordata`。

### 当前代码实现层

`Imu::init()` 通过名字查找并缓存三个 sensor 的地址：

- `framequat_address_`
- `gyro_address_`
- `accelerometer_address_`

读取时：

- orientation 从 `framequat` 对应的 `sensordata` 连续 4 项获取
- angular velocity 从 `gyro` 连续 3 项获取
- linear acceleration 从 `accelerometer` 连续 3 项获取

当前 IMU 是只读设备：

- `write()` 为空操作
- 不存在 command 语义

### 当前实现边界

- 依赖三个 sensor 名称完整配置
- 只读取数据，不建模噪声、偏置、标定
- 协方差字段未实现

## Camera 原理

### MuJoCo 原理层

Camera 与 `sensordata` 不同。  
当前 camera 的数据来自 MuJoCo 渲染管线，而不是来自 `sensor` 数组。

依赖的对象包括：

- `mjOBJ_CAMERA`
- `mjvScene`
- `mjrContext`

Camera 的本质是：在指定相机位姿下，把当前场景渲染成图像，再从 framebuffer 读回像素。

### 当前代码实现层

`Camera::init()` 完成以下步骤：

1. 通过 `camera_name` 找到 MuJoCo camera id
2. 初始化 `mjvCamera` 并设置为 `mjCAMERA_FIXED`
3. 创建 viewport
4. 分配 RGB 和 depth buffer
5. 初始化最小 `CameraInfo`

读取时的主要流程是：

1. `mjv_updateScene`
2. `mjr_render`
3. `mjr_readPixels`

其中：

- RGB 图像来自 `mjr_readPixels` 的 `rgb` 输出
- 深度图来自 `mjr_readPixels` 的 `depth` 输出

### 当前实现边界

- 必须有可用的 `mjvScene` 和 `mjrContext`
- `camera_info` 目前仅做最小填充
- 不涉及异步发布线程
- 不包含相机畸变、曝光、噪声等高级模型

## Lidar 原理

### MuJoCo 原理层

当前 lidar 不是直接封装 MuJoCo 通用 raycast API，而是依赖一组 `rangefinder` 传感器阵列。  
每个 beam 对应一个 MuJoCo sensor，最终数据都进入 `mjData::sensordata`。

### 当前代码实现层

当前实现用以下规则把多个 beam 组装成一帧 `LaserScan`：

- 为设备配置一个 `sensor_prefix`
- beam 名称必须满足 `<prefix>-<index>`
- `index` 用来决定 beam 在 scan 中的顺序

初始化时：

1. 根据 `angle_min / angle_max / angle_increment` 计算 beam 数
2. 遍历 `mjModel::nsensor`
3. 找到所有 `mjSENS_RANGEFINDER`
4. 解析名字并缓存每个 beam 对应的 `sensordata` 地址

读取时：

- 逐 beam 读取 `sensordata[address]`
- 如果范围超出 `[range_min, range_max]`，则写成 `-1.0`
- 最终组装到 `LidarState::laser_scan.ranges`

### 当前实现边界

- 只支持 `LaserScan`
- 不生成点云
- 强依赖 `<prefix>-<index>` 命名约定
- 不处理 intensity 的真实建模

## MobileBase 原理

### 为什么 `MobileBase` 不放进 `Joint`

`Joint` 是单关节抽象，只处理一个 MuJoCo joint 的读写。  
而 mobile base 是多个 traction joints 的组合语义，需要基于底盘运动学把一个底盘速度命令映射到多轮速度。

因此 `MobileBase` 必须是独立抽象，而不是 `Joint` 的一个特例。

### 当前 `MobileBase` 的抽象方式

`MobileBase` 内部不直接访问 MuJoCo 底层数组，而是复用已注册的 `Joint`：

- 通过 `traction_joint_names` 找到一组 traction joints
- 通过 `Joint::write()` 下发轮速
- 通过 `Joint::read()` 回读轮速

也就是说，它是 joint group 的运动学包装层。

### 当前支持的类型

当前只支持两类底盘：

- `MobileBaseType::Differential`
- `MobileBaseType::Omnidirectional`

不支持：

- steering 类底盘
- ackermann
- tricycle

### Differential 原理

输入使用 `MobileBaseCommand`，对应 `Twist` 语义：

- `linear.x`
- `angular.z`

当前实现通过：

- `wheel_radius`
- `track_width`

把底盘速度映射成左右轮速度：

- left wheel velocity
- right wheel velocity

反向读取时，再从左右轮速度恢复：

- `linear.x`
- `angular.z`

### Omnidirectional 原理

当前 omnidirectional 模型假设 4 个 traction joints。  
输入使用：

- `linear.x`
- `linear.y`
- `angular.z`

通过：

- `wheel_radius`
- `track_width`
- `wheel_base`

把底盘速度映射成四轮速度。  
反向读取时，再根据四轮角速度恢复平面速度和角速度。

### 当前实现边界

- 只支持已注册 joint 的组合
- 不做里程计积分
- 不处理 steering joint
- 不处理被动轮的动力学补偿
- 仅封装运动学映射，不实现更高层控制器

## 当前实现边界与后续扩展方向

### 当前统一边界

当前设备层存在以下明确限制：

- `Joint`
  - 仅支持 1-DoF joint
- `Camera`
  - 依赖 render context
- `Lidar`
  - 依赖 beam 命名规则
- `MobileBase`
  - 仅支持 differential / omnidirectional

### 后续可扩展方向

后续如果要扩展，可以优先考虑：

- `ball/free joint`
- `ackermann/tricycle` mobile base
- 点云输出
- ROS 2 bridge 结合
- transmission
- PID
- controller interface / mode switch

这些能力当前都不应该被视为“已经具备”，只能作为后续扩展方向。

## 常见概念对比

### Joint vs Actuator

| 概念 | 含义 |
| --- | --- |
| Joint | 描述自由度本身 |
| Actuator | 描述控制输入如何作用到系统 |

### Camera vs Sensor

| 概念 | 当前实现方式 |
| --- | --- |
| IMU / Lidar | 读取 `sensordata` |
| Camera | 走渲染管线，不读 `sensordata` |

### Lidar beam vs LaserScan

| 概念 | 含义 |
| --- | --- |
| beam | 一个 `rangefinder` sensor |
| LaserScan | 多个 beam 按顺序拼成的一帧扫描 |

这也是为什么当前 lidar 强依赖 `<prefix>-<index>` 命名规则。  
它不是从任意传感器集合自动推断扫描顺序，而是显式按名字组织 beam。
