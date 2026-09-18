# romujoco Architecture

`romujoco` 是工作区中的 MuJoCo 仿真运行时内核。它负责封装 `mjModel` / `mjData`、物理步进、组件调度、离屏渲染和 viewer，同上层 ROS 2 适配层解耦。

本文档只描述当前实现，不记录重构过程、迁移路线或历史接口。

## 模块定位

`romujoco` 负责：

- 加载和校验 MuJoCo MJCF/XML 模型
- 管理 `mjModel`、`mjData` 和物理步进
- 提供统一的 `Simulation` 顶层入口
- 组织组件装配、周期更新、命令写入和状态读取
- 提供 camera 离屏渲染和 viewer

`romujoco` 不负责：

- `ros2_control` 插件导出
- ROS topic/service/clock
- `HardwareInfo`、URDF 或 launch 编排

## 运行时总体架构

当前主结构为：

```text
Simulation
  -> SimulationRuntime
    -> mjModel + mjData
  -> SimulationScheduler
  -> ComponentManager
    -> SimulationComponent
      -> Joint / Gripper / Imu / Lidar / Camera / MobileBase
  -> CommandBuffer / StateBuffer
  -> CameraRenderService (internal)
  -> SimulationViewer
```

关键边界：

- `Simulation` 是唯一公开顶层运行时入口
- `SimulationRuntime` 负责模型加载、forward、step、reset
- `SimulationScheduler` 负责运行态调度和状态机
- `ComponentManager` 负责组件装配、统一更新、命令下发和聚合读状态
- `CameraRenderService` 与 `SimulationViewer` 是独立渲染资源
- `SimulationViewer` 通过受控 `SimulationContext` 同步 MuJoCo 运行时，不直接对外暴露裸 `mjModel* / mjData*` 启动接口
- viewer 启动超时通过 `SimulationConfig.viewer_startup_timeout` 控制
- `mjModel / mjData` 保持单写线程原则，只在 scheduler/reset 安全路径中写入

线程边界：

- scheduler worker thread
  - 连续运行模式下唯一推进物理和组件更新的线程
- viewer render thread
  - 只负责 viewer 渲染与被动同步
- external caller threads
  - 只写 `CommandBuffer`、读取 `StateBuffer`、提交控制请求

## 当前公开接口约定

当前公开返回模型严格区分两类概念：

- `SimulationStatus`
  - 只表示仿真生命周期状态：`Uninitialized`、`Stopped`、`Running`、`Paused`、`Stopping`、`Error`

公开接口规则：

- 运行控制与命令接口统一返回 `bool`
- typed read 统一使用 `bool + out-parameter`
- 调用方应基于返回值分支，不依赖动态错误字符串

`Simulation` 公开的核心能力包括：

- 生命周期与运行控制
  - `initialize(const SimulationConfig&)` / `initialize(const std::string&)`
  - `shutdown()`
  - `start() / stop() / pause() / resume()`
  - `reset()` / `reset(std::string keyframe_name)`
  - `step(std::size_t count = 1)`、`step_count()`、`status()` 与 `time()`
- 组件与命令
  - `write_command(const JointCommand&)`
  - `write_command(const GripperCommand&)`
  - `write_command(const MobileBaseCommand&)`
  - `write_command(const RobotCommand&)`：跨类型原子提交
  - `write_commands(const JointCommands&)`
  - `write_commands(const GripperCommands&)`
  - `write_commands(const MobileBaseCommands&)`
- 状态读取
  - `read_state(std::shared_ptr<const RobotState>&)`
  - `read_state(RobotState&)`
  - 按组件 ID 读取 `JointState`、`GripperState`、`ImuState`、`CameraState`、
    `LidarState` 与 `MobileBaseState`
  - 分别读取 `JointStates`、`GripperStates`、`ImuStates`、`CameraStates`、
    `LidarStates` 与 `MobileBaseStates` 聚合状态

## 组件模型与调度模型

所有设备组件统一继承 `SimulationComponent`。当前不再区分 `SensorComponent` 与非传感器组件。

统一组件能力：

- `name()`
- `init(const SimulationContext&)`
- `reset(...)`
- `update(...)`
- `poll_update(SimTime time)`
- `reset_schedule()`

调度原则：

- 所有组件都参与周期更新
- 是否到期由组件自己的更新节奏决定
- `ComponentManager` 每步统一遍历组件并调用 `update(...)`
- `camera` 的共享渲染资源协调由 manager 特殊处理，但不暴露为单独基类层次
- step 周期固定拆为两段：
  - `update_components_for_step()`
  - `build_state_snapshot()`

缓冲层职责：

- `CommandBuffer`
  - 缓存外部线程提交的最新控制命令；`RobotCommand` 在一个快照中发布其所有类型更新
- `StateBuffer`
  - 每个成功的 physics step 发布最新 `RobotState`，其中包含各类设备的不可变共享状态快照

缓冲接口语义统一使用 `write(...)` / `read(...)`。

缓冲层的并发模型、动态拓扑和确定性实时性能取舍见：

- [CommandBuffer 并发与实时性能设计](./command_buffer.md)
- [StateBuffer 并发与实时性能设计](./state_buffer.md)

这两份文档包含提议架构；本页仍以当前已实现行为为准。

- `StateBuffer`
  - 通过原子发布 `shared_ptr<const RobotState>` 共享最新快照；未更新组件复用已有状态快照

## 设备当前实现语义

### Joint

`Joint` 是单关节、单标量状态/命令抽象，当前主要面向 1-DoF joint。

- 支持 `Hinge` 和 `Slide`
- 不支持 `Ball` 和 `Free`
- 状态主要映射：
  - `position <- qpos`
  - `velocity <- qvel`
  - `effort <- qfrc_actuator`
- 命令主要映射：四种模式都换算为广义力后写入 `ctrl`
  - `Position / Velocity / Hybrid` 由 stiffness / damping 换算
  - `Effort` 直接使用命令力，可叠加重力补偿
  - 结果统一受 `effort_limits` 与 actuator 的 `forcerange` / `ctrlrange` 限制
- 被动关节没有 actuator：不接收命令，也不写 `qfrc_applied`

当前不处理：

- transmission
- mimic joint
- 多自由度 joint

### Gripper

`Gripper` 是双指平行夹爪的设备级抽象，描述总开口宽度而不是单个 finger
joint。详细设计见 [gripper.md](./component/gripper.md)。

- 固定包含两个 finger，finger 对应 `mjJNT_SLIDE` joint，joint 坐标已按
  “增大即张开”规范化
- `width = q0 + q1`，`velocity = dq0 + dq1`，`effort` 取所有主动 finger
  actuator 的最大绝对力
- 至少需要一个 actuator；没有独立 actuator 的 finger 由 MJCF
  equality/tendon 等机械约束驱动
- 命令只提供 `width / velocity / effort`，其中 `velocity` 与 `effort`
  为非负幅值；方向由目标宽度与当前宽度的差决定
- 组件内部维护受 `velocity` 限制的 width reference，`advance()` 按对称
  半宽 reference 对每个主动 finger 执行 PD 控制并写入 `ctrl`
- `update()` 采样状态并按 `width_tolerance / velocity_threshold /
  effort_ratio / timeout` 判定 `stalled`
- reset 保持当前开口，清零 `ctrl`、stall 状态与命令速度/力
- 一个 finger joint 或 actuator 只能由一个组件拥有，配置阶段检测冲突

当前不处理：

- 三指或多指机械手
- 独立 finger 命令
- grasp planning 与精确接触力估计

### Imu

`Imu` 由多个 MuJoCo `sensor` 组合而成，当前是只读设备。

- `orientation` 来自 `framequat`
- `angular_velocity` 来自 `gyro`
- `linear_acceleration` 来自 `accelerometer`

当前不处理：

- 噪声、偏置、标定
- 协方差建模

### Lidar

`Lidar` 依赖一组 `rangefinder` 传感器阵列拼装 `LidarState`。

- beam 名称需满足 `<prefix>-<index>`
- 按 index 决定 beam 顺序
- 每个 beam 数据来自 `sensordata`

当前不处理：

- 点云输出
- intensity 真实建模
- 无命名规则的自动拓扑推断

### Camera

`Camera` 不读取 `sensordata`，而是走 MuJoCo 渲染链路。

- 相机对象来自 `mjOBJ_CAMERA`
- 渲染依赖 `mjvScene` 和 `mjrContext`
- 公开读取结果统一为 `CameraState`
- 支持离屏渲染
- 不依赖 viewer 才能工作
- 物理线程提交最新 `mjData` 快照，专属 worker 完成 OpenGL 渲染与图像读取
- worker 落后时仅保留最新待渲染帧；RobotState 保留上一有效 CameraState

当前不处理：

- 畸变、曝光、噪声等高级相机模型
- 独立的 CameraState 发布接口

### MobileBase

`MobileBase` 是底盘级协调组件：公共接口只有平面速度命令与底盘状态，wheel /
steering actuator 如何驱动属于各 chassis 自己的实现。详细契约见
[mobile_base.md](./component/mobile_base.md)。

- `MobileBaseCommand` 只包含 `PlanarTwist`：`linear_x` / `linear_y` / `angular_z`
- `MobileBaseState` 是 `timestamp + Pose3d + Twist3d` 的地面真值状态
- 当前支持：
  - `Mecanum` + `execution_mode = kinematic`：直接推进 base free joint 与轮 joint
  - `Swerve` + `execution_mode = dynamic`：只写 steering position 与 drive velocity，
    底盘位姿由 MuJoCo 动力学和轮地接触决定
- 新增底盘只需增加 chassis 实现与配置解析，运行时的 `Simulation` /
  `SimulationScheduler` 不变

当前不处理：

- `Ackermann`
- `Tricycle`
- 履带等其余底盘
- wheel odometry 与 ROS 消息
- 更高层控制器

## 与 MuJoCo 核心对象的映射

所有设备实现都建立在两个核心对象之上：

- `mjModel`
  - 静态模型定义
  - 提供 joint、actuator、sensor、camera 的名字、类型、地址和拓扑
- `mjData`
  - 运行时状态
  - 提供 `qpos`、`qvel`、`ctrl`、`sensordata`、`qfrc_*` 等动态数组

可简化理解为：组件层是对 `mjModel + mjData` 的结构化访问封装。

设备与 MuJoCo 的主要数据来源：

| 设备 | 主要来源 |
| --- | --- |
| `Joint` | `qpos` / `qvel` / `ctrl` / `qfrc_actuator` |
| `Imu` | `sensordata` |
| `Lidar` | `sensordata` |
| `Camera` | 渲染管线，不走 `sensordata` |
| `Gripper` | `qpos` / `qvel` / `ctrl` / `actuator_force`（+ MJCF equality） |
| `MobileBase` | 多个 `Joint` 的读写组合 |

## 当前边界与非目标

当前实现明确不包含以下能力：

- 多物理引擎抽象
- GPU 批量并行仿真
- Python API
- 运行时热加载模型
- 点云和高级视觉感知抽象
- 通用 ECS
- 强化学习任务层

当前公开层也不再保留以下旧概念：

- `Status`
- `Result<T>`
- `MuJoCoSimulation`
- `SensorComponent`
- `SensorScheduler`
- `SensorSampleContext`
- 公开 `register_* / unregister_* / read_*` 管理器式接口

## 相关入口

- 包概览与构建方式见 [`../README.md`](../README.md)
