# mujoco_simulation Architecture

`mujoco_simulation` 是工作区中的 MuJoCo 仿真运行时内核。它负责封装 `mjModel` / `mjData`、物理步进、组件调度、离屏渲染和 viewer，同上层 ROS 2 适配层解耦。

本文档只描述当前实现，不记录重构过程、迁移路线或历史接口。

## 模块定位

`mujoco_simulation` 负责：

- 加载和校验 MuJoCo MJCF/XML 模型
- 管理 `mjModel`、`mjData` 和物理步进
- 提供统一的 `Simulation` 顶层入口
- 组织组件装配、周期更新、命令写入和状态读取
- 提供 camera 离屏渲染和 viewer

`mujoco_simulation` 不负责：

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
      -> Joint / Imu / Lidar / Camera / MobileBase
  -> CommandBuffer / StateBuffer
  -> CameraRenderer
  -> SimulationViewer
```

关键边界：

- `Simulation` 是唯一公开顶层运行时入口
- `SimulationRuntime` 负责模型加载、forward、step、reset
- `SimulationScheduler` 负责运行态调度和状态机
- `ComponentManager` 负责组件装配、统一更新、命令下发和聚合读状态
- `CameraRenderer` 与 `SimulationViewer` 是独立渲染资源
- `SimulationViewer` 通过受控 `mjContext` 同步 MuJoCo 运行时，不直接对外暴露裸 `mjModel* / mjData*` 启动接口
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
  - `initialize(...)`
  - `shutdown()`
  - `start() / stop() / pause() / resume()`
  - `step(...)`
  - `request_reset(...) / reset(...)`
  - `request_reset_to_keyframe(...) / reset_to_keyframe(...)`
- 组件与命令
  - `set_joint_command(...)`
  - `set_mobile_base_command(...)`
- 状态读取
  - `joint_state(...)`
  - `imu_state(...)`
  - `lidar_state(...)`
  - `camera_state(...)`
  - `mobile_base_state(...)`
  - `read_state(std::shared_ptr<const RobotState>&)`
  - `read_state(RobotState&)`

## 组件模型与调度模型

所有设备组件统一继承 `SimulationComponent`。当前不再区分 `SensorComponent` 与非传感器组件。

统一组件能力：

- `name()`
- `bind(...)`
- `reset(...)`
- `update(...)`
- `poll_update(mjTime time)`
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
  - 缓存外部线程提交的最新控制命令
- `StateBuffer`
  - 每个成功的 physics step 发布最新 `RobotState`，其中包含各类设备的不可变共享状态快照

缓冲接口语义统一使用 `write(...)` / `read(...)`。

缓冲层的并发模型、动态拓扑和确定性实时性能取舍见：

- [CommandBuffer 并发与实时性能设计](./command_buffer_realtime_design.md)
- [StateBuffer 并发与实时性能设计](./state_buffer_realtime_design.md)

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
  - `effort <- qfrc_actuator + qfrc_applied`
- 命令主要映射：
  - `Position / Velocity` 走 `ctrl`
  - `Effort` 对可驱动 actuator 写 `ctrl`，对被动关节回退到 `qfrc_applied`

当前不处理：

- transmission
- mimic joint
- 多自由度 joint

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

`MobileBase` 是多个 traction joints 的组合运动学包装，不是特殊的 `Joint`。

- 当前支持：
  - `Mecanum`
- 通过轮配置把底盘命令映射为多个关节命令
- 通过多个关节状态恢复底盘状态

当前不处理：

- `Ackermann`
- `Tricycle`
- steering joint 语义
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
| `Joint` | `qpos` / `qvel` / `ctrl` / `qfrc_*` |
| `Imu` | `sensordata` |
| `Lidar` | `sensordata` |
| `Camera` | 渲染管线，不走 `sensordata` |
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
- 工作区级迁移和包边界说明见 [`../../docs/migration_guide.md`](../../docs/migration_guide.md)
