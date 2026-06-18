# robot_mujoco 项目代码架构导读

> **说明：** 本文档主体仍保留“拆分为 `mujoco_simulation_ros` + `mujoco_hardware`”阶段的历史说明，不再代表 2026-06-18 之后的当前主实现。当前真实包边界与验证状态以 [`current_architecture.md`](./current_architecture.md) 为准，统一 ROS adapter 设计与迁移目标以 [`module/robot_mujoco_ros.md`](./module/robot_mujoco_ros.md) 和 [`migration_guide.md`](./migration_guide.md) 为准。

## 1. 项目定位与包划分

`robot_mujoco` 是一个面向 ROS 2 的通用 MuJoCo 仿真工作区。仓库当前默认示例机器人是 Franka / Panda，但代码结构本身并不绑定某一台具体机器人，目标是提供一套可复用的 MuJoCo 运行时、设备抽象、ROS bridge 和 `ros2_control` 接入层。

当前仓库包含五个 ROS 2 包：

- `mujoco_simulation`
  - MuJoCo 仿真运行时内核（纯 C++，不依赖 ROS）。
  - 负责模型加载（`ModelRuntime`）、仿真调度（`SimulationScheduler`）、组件管理（`ComponentManager`）、缓冲区（`CommandBuffer` / `StateBuffer` / `CameraBuffer`）、相机渲染（`CameraRenderer`）以及 viewer。
  - 通过 `Status` / `Result<T>` 暴露统一的错误处理接口。
- `mujoco_simulation_ros`
  - ROS bridge 包。
  - 负责 `/clock` 发布、IMU/Camera/Lidar 传感器消息发布、仿真控制服务（start/stop/pause/resume/step/set_realtime_factor/load_keyframe/reset），以及 ROS node/executor 生命周期管理。
- `mujoco_hardware`
  - `ros2_control` 硬件插件层。
  - 负责把 URDF / `ros2_control` 的 `HardwareInfo` 参数解析成运行时配置，把仿真状态映射为 ROS 侧的 state / command interface，并通过组合 `mujoco_simulation_ros::SimulationRosBridge` 获得 ROS 发布/服务能力。
- `robot_mujoco`
  - 工作区聚合入口（C++ 头文件聚合 + Python robocasa 场景生成 + launch/config）。
- `mujoco_ros2_bridge_msgs`
  - ROS 2 服务接口定义包（`StepSimulation`、`SetRealtimeFactor`、`ResetWorld`），供 `mujoco_simulation_ros` 和 `mujoco_hardware` 共用。

从职责边界看，`mujoco_simulation` 偏仿真运行时，`mujoco_simulation_ros` 偏 ROS 通信桥接，`mujoco_hardware` 偏 `ros2_control` 框架集成，`robot_mujoco` 偏工作区聚合与场景前处理。

## 2. 顶层架构图与调用链

项目核心调用链可以概括为：

```text
ros2_control controller manager
  -> mujoco_hardware::MuJoCoHardwareInterface
    -> mujoco_simulation::Simulation
      -> runtime::ModelRuntime           (模型加载与 mjModel/mjData 生命周期)
      -> runtime::SimulationScheduler    (物理线程、步进节奏、reset 请求队列)
      -> component::ComponentManager    (Joint / Imu / Camera / Lidar / MobileBase 组件)
      -> buffer::CommandBuffer          (命令缓冲、超时处理)
      -> buffer::StateBuffer            (状态快照发布/读取)
      -> buffer::CameraBuffer           (相机采样缓冲)
      -> CameraRenderer                 (离屏 OpenGL 渲染)
      -> Viewer                         (交互式可视化，可选)
```

ROS 通信链：

```text
Simulation 的 SnapshotObserver 回调
  -> MuJoCoHardwareInterface::publish_snapshot_to_ros()
    -> mujoco_simulation_ros::SimulationRosBridge
      -> ImuPublisher / CameraPublisher / LidarPublisher
        -> sensor_msgs topics
      -> /clock publisher
      -> simulation control services
```

可以把运行过程理解为三条并行职责：

- **控制链**：`ros2_control` 通过 `read()` / `write()` 和 mode switch 与 `MuJoCoHardwareInterface` 交互。`write()` 将关节命令写入 `CommandBuffer`，由 `SimulationScheduler` 在物理步进前 flush；`read()` 从 `StateBuffer` 获取最新状态快照。
- **观测链**：`SimulationScheduler` 在每步物理仿真后触发传感器采样；snapshot observer 回调负责将 IMU/Lidar 状态和 Camera 采样通过 `SimulationRosBridge` 发布为 ROS 消息。
- **服务链**：`SimulationRosBridge` 对外暴露 start/stop/pause/resume/step/set_realtime_factor/load_keyframe/reset 八个 ROS service；service 回调通过 `Status` 回调链转发到 `Simulation` 公开 API。

`mujoco_simulation::Simulation` 是运行时中心：它持有 `ModelRuntime`（模型生命周期）、`SimulationScheduler`（调度与物理线程）、`ComponentManager`（组件注册表）、`CommandBuffer` / `StateBuffer` / `CameraBuffer`（数据缓冲）、`CameraRenderer`（渲染）以及可选的 `Viewer`。`mujoco_hardware` 只通过公开接口与它交互，不直接操作 MuJoCo 原生结构。

## 3. `mujoco_simulation_ros`：ROS Bridge 层

`mujoco_simulation_ros::SimulationRosBridge` 是本项目中独立出来的 ROS 通信桥接包。它取代了旧版位于 `mujoco_hardware` 中的 `SensorBridge`。

### 3.1 职责边界

`SimulationRosBridge` 只负责 ROS 侧的发布与服务，不参与仿真控制逻辑：

- **传感器发布**：持有 `ImuPublisher`、`CameraPublisher`、`LidarPublisher` 三个内部类（均在 `simulation_ros_bridge.cpp` 中实现），每个 publisher 自管理 topic 名称、frame_id 和启停标志。
- **时钟发布**：独立线程以可配置频率（默认 250 Hz）在 `/clock` topic 上发布 `rosgraph_msgs::msg::Clock`。
- **控制服务**：提供八个 ROS service 端点 —— `/start`、`/stop`、`/pause`、`/resume`、`/step`、`/set_realtime_factor`、`/load_keyframe`、`/reset`。所有 service 通过 `StatusCallback` 函数对象将请求转发给 `MuJoCoHardwareInterface`，再由其调用 `Simulation` 公开 API。
- **生命周期管理**：内部持有独立的 ROS node、`SingleThreadedExecutor` 和 executor 线程，与 `ros2_control` 线程解耦。

### 3.2 配置结构

```cpp
struct SimulationRosBridgeConfig {
  std::string node_name;
  double clock_publish_rate_hz{250.0};
  std::vector<ImuPublisherConfig> imus;
  std::vector<CameraPublisherConfig> cameras;
  std::vector<LidarPublisherConfig> lidars;
};
```

每种 publisher config 携带 name / frame_id / topic 等 ROS 专有字段，由 `mujoco_hardware` 在 `on_init()` 阶段从 `HardwareConfig` 构建并传入。

### 3.3 与 `mujoco_hardware` 的接线方式

`MuJoCoHardwareInterface` 在 `on_init()` 中创建 `SimulationRosBridge`，注入八个 control status callback，并通过 `Simulation::set_snapshot_observer()` 注册回调 —— 每当 `SimulationScheduler` 发布新状态快照时，自动调用 `publish_snapshot_to_ros()` 将 IMU/Lidar/Camera 数据推送到 ROS topic。

## 4. `mujoco_hardware` 模块拆解

### 4.1 `config.cpp`：把 `HardwareInfo` 解析成运行时配置

`mujoco_hardware/src/config.cpp` 的职责是把 `hardware_interface::HardwareInfo` 解析成 `HardwareConfig`，供后续 `MuJoCoHardwareInterface` 直接使用。

关键仿真配置入口（写入 `SimulationConfig`）：

- `mujoco_model_path`：必填，MuJoCo 模型路径。
- `render_mode`：渲染模式，默认 `headless`，由 `mujoco_simulation::parse_render_mode()` 解析。
- `sim_speed_factor`：仿真速度倍率，默认 `1.0`。
- `initial_keyframe`：可选，初始化时重置到指定 MuJoCo keyframe。
- `control_freq`：控制频率，默认 `100.0` Hz。

关键传感器参数入口：

- **IMU**：`mujoco_orientation_sensor`、`mujoco_gyro_sensor`、`mujoco_accel_sensor`、`frame_id`、`topic`，以及协方差矩阵。
- **Camera**：`mujoco_camera_name`、`optical_frame_id`、`image_topic`、`depth_topic`、`camera_info_topic`、`enable_rgb`、`enable_depth`、`width`、`height`。
- **Lidar**：`frame_id`、`scan_topic`、`sensor_prefix`、`angle_min`、`angle_max`、`angle_increment`、`range_min`、`range_max`。

几个关键实现特征：

- joint 只接受 position / velocity / effort 三类 command interface。
- IMU 在 `mujoco_type` 参数缺失时默认视为 IMU（向后兼容）；camera 和 lidar 在 `mujoco_type` 缺失时显式报错（`is_sensor_type()` 含 `error_message` 参数）。
- camera 不再要求 `render_mode=viewer`——`CameraRenderer` 支持独立的离屏 OpenGL 渲染上下文。
- 配置解析失败时生成清晰的错误字符串，供 `on_init()` 直接中止初始化。

### 4.2 `MuJoCoHardwareInterface`：`ros2_control` 插件入口

`mujoco_hardware::MuJoCoHardwareInterface` 继承 `hardware_interface::SystemInterface`，是 `mujoco_hardware_plugin.xml` 导出的插件实现。

其职责分成四类：

- **生命周期管理**：`on_init()` 调用配置解析，创建 `Simulation` 和 `SimulationRosBridge`，注册 snapshot observer 回调。
- **接口导出**：`export_state_interfaces()` 导出 joint 和 IMU 的 state interface；`export_command_interfaces()` 导出 joint command interface。
- **模式切换**：`prepare_command_mode_switch()` 校验要切换的接口是否合理；`perform_command_mode_switch()` 通过 `Simulation::reconfigure_component()` 下发 mode 变化。
- **运行时读写**：`read()` 从 `Simulation::state_snapshot()` 获取状态快照并更新本地 `JointData` / `ImuData` 等；`write()` 将 controller 命令通过 `Simulation::set_joint_command()` 写入 `CommandBuffer`。

关键设计要点：

- `MuJoCoHardwareInterface` 直接持有 `std::unique_ptr<mujoco_simulation::Simulation>` 和 `std::unique_ptr<mujoco_simulation_ros::SimulationRosBridge>`。旧版 `Robot` 包装类和 `SensorBridge` 均已移除。
- 状态更新走 snapshot 模式：`update_runtime_state()` 调用 `simulation_->state_snapshot()` 获取共享指针，然后从 snapshot 中查找各组件状态。这避免了直接访问 `mjData` 的并发问题。
- ROS 消息发布通过 snapshot observer 异步触发：`SimulationScheduler` 在每步物理仿真后发布 snapshot，observer 回调在 `mujoco_hardware` 侧执行 `publish_snapshot_to_ros()`。

### 4.3 数据结构：`HardwareConfig` 与 `JointData` 等

`mujoco_hardware/data.hpp` 定义了 ROS 适配层所需的数据结构。以 joint 为例，`JointData` 聚合了：

```cpp
struct JointData {
  std::string name;
  std::vector<std::string> command_interfaces;   // ROS 专有：命令接口名列表
  std::vector<std::string> state_interfaces;     // ROS 专有：状态接口名列表
  mujoco_simulation::JointConfig config;         // 设备层原生配置类型
  mujoco_simulation::JointCommand command;       // 设备层原生命令类型
  mujoco_simulation::JointState state;           // 设备层原生状态类型
};
```

IMU / Camera / Lidar / MobileBase 采用相同模式 —— 持有 `mujoco_simulation` 设备层原生类型 + ROS 专有字段（topic / frame_id / 启停标志等）。这意味着 `mujoco_hardware` 不再定义中间数据层，直接复用仿真层的类型定义。

## 5. `mujoco_simulation` 模块拆解

### 5.1 `Simulation`：仿真会话与公开 API

`mujoco_simulation::Simulation` 是整个仿真运行时的中心类，对上层暴露一个稳定的 C++ API。

公开接口分为几组：

- **生命周期**：`initialize(SimulationConfig)` / `shutdown()`
- **控制**：`start()` / `stop()` / `pause()` / `resume()` / `step()` / `reset()` / `request_reset()` / `set_realtime_factor()`
- **设备命令**：`reconfigure_component(ComponentConfig)` / `set_joint_command(JointCommand)` / `set_mobile_base_command(name, MobileBaseCommand)`
- **设备读取**：`joint_state()` / `imu_sample()` / `lidar_sample()` / `camera_sample()` / `mobile_base_state()`
- **快照**：`state_snapshot()` / `set_snapshot_observer(SnapshotObserver)`
- **状态查询**：`step_count()` / `status()` / `simulation_time()`

所有公开 API 返回 `Status` 或 `Result<T>`（见 [8.3 节](#83-错误处理策略)）。

内部持有：

- `ModelRuntime`：`mjModel*` / `mjData*` 的加载、持有与销毁。
- `SimulationScheduler`：物理线程、步进节奏、reset 请求队列、realtime factor 控制。
- `ComponentManager`：组件注册表与读写调度。
- `CommandBuffer`：带超时策略的命令缓冲。
- `StateBuffer`：线程安全的状态快照发布/读取。
- `CameraBuffer`：相机采样的共享指针缓冲。
- `CameraRenderer`：独立的离屏 OpenGL 渲染上下文（支持 headless camera）。
- `Viewer`（可选）：交互式可视化。

### 5.2 `SimulationScheduler`：调度与物理线程

`SimulationScheduler` 是仿真节拍的核心。它运行一个独立的工作线程（`worker_loop()`），该线程是**唯一允许写主 `mjData` 的执行上下文**。

每步仿真周期包含：

1. **Flush commands**：从 `CommandBuffer` 取 snapshot，应用 joint / mobile_base 命令到 `mjData`。
2. **Physics step**：调用 `mj_step()` 推进物理。
3. **Sensor sample**：`ComponentManager::sample_due_sensors()` 按各传感器配置的频率采样 IMU / Camera / Lidar。
4. **Publish snapshot**：构建 `SimulationStateSnapshot`（含 joint / mobile_base state + IMU / Lidar sample），发布到 `StateBuffer`，并触发 snapshot observer。
5. **Viewer sync**：按 viewer 刷新率同步渲染。

关键特性：

- 所有对 `mjData` 的写操作集中在 scheduler 线程，调用方线程只通过 `CommandBuffer` 提交命令或通过 `StateBuffer` 读取状态。
- 支持 `request_reset()` 的异步 reset 语义 —— reset 请求排队后在 scheduler 线程中执行，可返回 `std::future<Status>`。
- 物理循环每轮从 `clock::now()` 重新计算 waketime，消除累积漂移。

### 5.3 `ComponentManager`：组件注册表与调度

`ComponentManager` 取代了旧版的 `HardwareManager`，按统一组件模式管理设备对象：

- `JointComponent`：继承 `SimulationComponent`，封装关节位置/速度/effort 读取和 position/velocity/effort 命令写入。
- `ImuComponent`：继承 `SensorComponent`，从多个 MuJoCo sensor 组合出一个 IMU 状态，支持按配置频率采样。
- `CameraComponent`：继承 `SensorComponent`，通过 `CameraRenderer` 独立渲染 RGB/depth 图像。
- `LidarComponent`：继承 `SensorComponent`，依据 `rangefinder` 传感器阵列拼装 `LaserScan`。
- `MobileBaseComponent`：继承 `SimulationComponent`，为移动底盘提供统一抽象。

对每类组件提供统一风格的接口：`register_*()` / `unregister_*()` / `read_*()` / `write_*()` / `reconfigure_component()`。

内部持有：

- `ComponentRegistry`：类型安全的多态组件注册表（按 `SimulationComponent` 基类存储，按具体类型索引）。
- `SensorScheduler`：传感器采样频率调度器。

### 5.4 组件模式：`SimulationComponent` 与 `SensorComponent`

所有设备组件实现统一的多态接口：

```cpp
class SimulationComponent {
 public:
  virtual std::string_view name() const noexcept = 0;
  virtual Status bind(const mjModel& model) = 0;
  virtual Status reset(const mjModel& model, mjData& data) = 0;
};
```

传感器组件额外继承 `SensorComponent`：

```cpp
class SensorComponent : public SimulationComponent {
 public:
  virtual double update_rate() const noexcept = 0;
  virtual Status sample(const SensorSampleContext& context) = 0;
};
```

每个具体组件（如 `JointComponent`）持有自己的 `*Config`、`*Binding`（MuJoCo 原生索引映射）、内部状态副本和采样序号。组件不再直接持有 `mjModel*` / `mjData*` 指针 —— 这些通过方法参数注入。

### 5.5 缓冲区架构

三个缓冲区类构成了仿真数据流的骨干：

- **`CommandBuffer`**：线程安全的命令缓冲。`set_joint_command()` / `set_mobile_base_command()` 从任意线程调用；`snapshot()` 在 scheduler 线程中调用。支持可配置的命令超时策略（KeepLast / ZeroCommand / HoldPosition）。
- **`StateBuffer`**：线程安全的状态快照缓冲。scheduler 线程通过 `publish()` 写入新快照；调用方线程通过 `read()` 获取共享指针。基于 `shared_ptr` 实现免拷贝的零等待读取。
- **`CameraBuffer`**：相机采样专用缓冲，管理 `CameraSample` 共享指针的生命周期。

### 5.6 `CameraRenderer`：独立渲染上下文

`CameraRenderer` 提供独立的离屏 OpenGL 渲染能力，不再依赖 viewer：

- 内部持有 `OffscreenGlContext`（EGL/GLX 离屏上下文）。
- 支持按需创建和调整离屏 framebuffer（`ensure_offscreen_capacity()`）。
- 在 headless 和 viewer 两种模式下均可使用。
- `Simulation` 在初始化时根据 `CameraRendererConfig` 创建 `camera_renderer_`，与 viewer 完全解耦。

### 5.7 Viewer 集成

viewer 功能由 `Viewer` 类封装，基于 MuJoCo 官方 `simulate` 工程的集成版本。职责仅限于：

- 建立 GLFW / OpenGL 渲染上下文
- 驱动 `mjvScene` 与 `mjrContext`
- 在 `RenderMode::Viewer` 下同步显示仿真画面
- 处理 GUI 事件、键盘鼠标输入

Viewer 与 `CameraRenderer` 完全独立：前者负责交互式显示，后者负责传感器渲染。两者各自管理自己的 OpenGL 上下文。

## 6. RoboCasa 场景生成简述

`robot_mujoco/robocasa` 是一个 Python 子系统，用来把 RoboCasa 的厨房任务场景转换为本项目可直接加载的 MJCF。

核心文件职责如下：

- `scene_config.py`：定义 `SceneConfig`。负责从 YAML 读取 `robocasa`、`robot`、`output` 三组配置，并校验 `task_name`、`layout_id`、`style_id`、`robot_xml_path`、`control_freq` 等字段。
- `scene_cli.py`：提供命令行入口。负责加载基础 YAML、接收参数，并驱动场景生成。
- `scene_runner_cli.py`：统一的 generate-and-launch CLI，编排场景生成到仿真启动的完整链路。
- `cli_options.py`：命令行参数定义的共享模块。
- `scene_generator.py`：主流程入口。负责创建 RoboCasa / robosuite 环境、触发模型加载、提取原始 MJCF、抽取对象摆放和推荐机器人出生位姿，再调用适配器生成最终 XML。
- `scene_data.py`：数据模型定义（`GeneratedScene`、`ObjectPlacement`、`SceneMetadata`、`SpawnPose`）。
- `mjcf_adapter.py`：负责对 RoboCasa 输出的 MJCF 做项目级变换。典型操作包括移除 placeholder robot、清理顶层 runtime 元素、插入项目机器人 `include`、应用 object placement、应用 spawn pose、补 floor geom、隐藏 debug marker。
- `exceptions.py`：异常类型层次定义。

整体链路：

```text
RoboCasa YAML config
  -> SceneConfig
    -> scene_cli.py / scene_runner_cli.py
      -> SceneGenerator.generate()
        -> robosuite / robocasa 生成厨房 MJCF
        -> 提取 object placements / spawn pose
        -> adapt_mjcf()
          -> 替换 placeholder robot
          -> include 本项目 robot XML
          -> 输出最终 MJCF
```

`robot_mujoco/__init__.py` 对外 re-export `SceneConfig`、`GeneratedScene`、`ObjectPlacement`、`SceneMetadata`、`SpawnPose`、`config_from_mapping`、`load_config` 等关键符号。`robot_mujoco/runtime.py` 提供仿真运行时的 Python 侧封装。

> **架构注记：** `robocasa` 是纯 Python 模块 —— 它只依赖 `mujoco`、`robosuite`、`robocasa` 等 Python 包，不调用 `mujoco_simulation` 的任何 C++ 代码。它已从 `mujoco_simulation` 中提取到顶层 `robot_mujoco.robocasa` 包。详见 [8.9 节](#89-robocasa-模块归属与-robot_mujoco-包设计)。

## 7. 当前代码现状与边界

### 7.1 当前仓库现状

- `docs/` 目录包含本文件、`current_architecture.md`、`migration_guide.md`、`performance_baseline.md`、`robocase.md` 等参考文档。
- 仓库已有一份设备层设计文档：[mujoco_simulation/docs/hardware_devices_principles.md](/home/siyuey/workspace/franka/franka_mujoco/robot_mujoco/mujoco_simulation/docs/hardware_devices_principles.md)。
- 根目录 [README.md](/home/siyuey/workspace/franka/franka_mujoco/robot_mujoco/README.md) 覆盖环境初始化、RoboCasa 依赖安装和基础验证。
- 测试基础设施已恢复：`mujoco_simulation/test/`（unit/integration/concurrency/performance）、`mujoco_hardware/tests/`、`mujoco_simulation_ros/tests/`。
- 已完成的架构重构包括：`Simulation*` 中间类型消除、`CommandInterfaceType` 统一为唯一关节控制模式枚举、`robocasa` 提取到顶层 `robot_mujoco` Python 包、`SensorBridge` 迁移为 `mujoco_simulation_ros::SimulationRosBridge`、`MuJoCoSimulation` 重构为 `Simulation`、`HardwareManager` 演进为 `ComponentManager` + buffer 架构、错误处理统一为 `Status`/`Result<T>`、`Impl`/Pimpl 消除、`render_hardware_manager` 消除、`MjContext` 统一注入、`Robot` 类移除、`CameraRenderer` 独立渲染上下文（解耦 camera 与 viewer）。

### 7.2 已知实现边界

- camera 已支持 headless 模式：`CameraRenderer` 提供独立离屏 OpenGL 渲染，`SimulationConfig::camera_renderer` 字段控制配置。
- `mujoco_hardware` 导出 joint state / command interface，以及 IMU state interface；camera 和 lidar 主要通过 ROS topic 输出，而不是通过 `ros2_control` state interface 暴露。
- `SimulationRosBridge` 内部包含 `ImuPublisher`、`CameraPublisher`、`LidarPublisher` 三个独立 publisher 类（均已实现为拆分后的 per-sensor publisher）。
- `MuJoCoHardwareInterface` 直接持有 `Simulation` 和 `SimulationRosBridge`，不再经过中间的 `Robot` 或 `SensorBridge` 包装层。
- RoboCasa 子系统是纯 Python 模块，从 `robot_mujoco.robocasa` import。
- 错误处理统一为 `Status` / `Result<T>`，旧版 `bool + std::string*` 和 `last_error()` 模式已消除。
- 调度器采用单写者模型：只有 `SimulationScheduler` 工作线程可写主 `mjData`；调用方通过 buffer 提交命令和读取状态。

### 7.3 阅读顺序建议

1. 先看 [README.md](/home/siyuey/workspace/franka/franka_mujoco/robot_mujoco/README.md)，建立依赖、目录和运行前置条件的基本认识。
2. 再看本文第 1–7 节，建立项目级与模块级心智模型。
3. 再看本文第 8 节（架构深入与改进方向），理解当前设计中的权衡、已知局限和改进路线图。
4. 如需了解最新包边界快照与迁移说明，看 [`current_architecture.md`](./current_architecture.md) 和 [`migration_guide.md`](./migration_guide.md)。
5. 如需深入设备层实现，看 [mujoco_simulation/docs/hardware_devices_principles.md](/home/siyuey/workspace/franka/franka_mujoco/robot_mujoco/mujoco_simulation/docs/hardware_devices_principles.md)。

---

## 8. 架构深入与改进方向

本章从架构设计角度审视项目当前实现中的关键权衡、已知局限和可优化方向。内容不是缺陷列表，而是帮助开发者在后续迭代中做出知情决策的参考。

### 8.1 线程模型与并发保证

项目中存在以下主要执行上下文：

```text
┌─ ros2_control 线程 ───────────────────────────────────────────────────┐
│  controller_manager -> MuJoCoHardwareInterface::read() / write()      │
│  read():                                                              │
│    -> Simulation::state_snapshot()  (从 StateBuffer 读取共享指针)      │
│    -> 更新本地 JointData / ImuData / LidarData / MobileBaseData       │
│  write():                                                             │
│    -> Simulation::set_joint_command()  (写入 CommandBuffer)           │
│    -> Simulation::set_mobile_base_command()                           │
└───────────────────────────────────────────────────────────────────────┘

┌─ SimulationScheduler 工作线程 ─────────────────────────────────────────┐
│  (唯一允许写主 mjData 的线程)                                          │
│  每步周期:                                                             │
│    -> flush CommandBuffer snapshot                                    │
│    -> mj_step()                                                       │
│    -> ComponentManager::sample_due_sensors()                          │
│    -> publish SimulationStateSnapshot to StateBuffer                  │
│    -> 触发 SnapshotObserver 回调                                       │
│    -> viewer sync (if due)                                            │
└───────────────────────────────────────────────────────────────────────┘

┌─ SimulationRosBridge executor 线程 ────────────────────────────────────┐
│  rclcpp::SingleThreadedExecutor::spin()                                │
│  处理 ROS service 回调 (start/stop/pause/resume/step/...)              │
│  不直接访问主 mjData                                                   │
└───────────────────────────────────────────────────────────────────────┘

┌─ SimulationRosBridge clock 线程 ───────────────────────────────────────┐
│  run_clock_publisher()                                                 │
│  以 clock_publish_rate_hz 频率在 /clock topic 上发布仿真时间            │
│  从 sim_time_ns_ (atomic) 读取时间戳                                   │
└───────────────────────────────────────────────────────────────────────┘

┌─ SnapshotObserver 回调 (在 scheduler 线程内同步执行) ──────────────────┐
│  publish_snapshot_to_ros():                                            │
│    -> ros_bridge_->update_sim_time()                                   │
│    -> ros_bridge_->publish_imu() / publish_camera() / publish_lidar()  │
└───────────────────────────────────────────────────────────────────────┘
```

**数据共享与保护机制：**

| 共享资源 | 访问者 | 保护机制 |
|---------|--------|---------|
| `mjModel*` | 只读，所有线程 | 无需锁（MuJoCo 保证 model 不可变） |
| `mjData*` | Scheduler 线程（唯一写者）；调用方不直接访问 | 单写者保证 |
| `CommandBuffer` | ros2_control 线程（写）+ Scheduler 线程（读） | 内部 mutex |
| `StateBuffer` | Scheduler 线程（写）+ ros2_control 线程（读） | 内部 mutex + shared_ptr |
| `CameraBuffer` | Scheduler 线程（写）+ ros2_control 线程（读） | 内部 mutex + shared_ptr |
| `SimulationScheduler` 状态 | 多线程 | 内部 mutex + condition_variable |
| `SimulationRosBridge::sim_time_ns_` | clock 线程（写）+ ros2_control 线程（写） | `std::atomic<int64_t>` |

**已知风险与注意事项：**

1. **SnapshotObserver 在 scheduler 线程内同步执行**：`publish_snapshot_to_ros()` 中的 ROS publish 操作在 scheduler 线程中执行。若 ROS 中间件因网络或订阅者问题阻塞，可能拖慢整个仿真循环。后续可考虑引入无锁队列将发布操作异步化。
2. **Camera 渲染在 scheduler 线程中执行**：`CameraRenderer` 的 GPU 渲染（`mjv_updateScene()` + `mjr_render()` + `mjr_readPixels()`）在 scheduler 的采样阶段执行，耗时远大于 joint/IMU/Lidar 读取。可考虑将 camera 渲染移到独立线程。
3. **`SimulationRosBridge` 的 executor 和 clock 线程不直接访问仿真数据**：它们通过 atomic 时间戳和 publisher 内部状态工作，与仿真线程完全解耦。当前架构下这两个线程的安全性较好。

### 8.2 数据结构分层

项目中同一设备概念的数据流如下（以 joint 为例）：

```text
┌─ 硬件插件层 (mujoco_hardware) ────────────────────────────────────────┐
│  JointData { name, command_interfaces[], state_interfaces[],          │
│              mujoco_simulation::JointConfig config,                     │
│              mujoco_simulation::JointCommand command,                   │
│              mujoco_simulation::JointState state }                      │
│  职责: ROS HardwareInfo 解析结果 + 设备层原生类型的薄包装               │
│  注: JointData 不再重复定义 position/velocity/effort 字段              │
│      这些字段属于设备层原生类型 JointConfig/JointCommand/JointState    │
└────────────────────────────────────────────────────────────────────────┘

┌─ 仿真公开接口层 (simulation.hpp) ──────────────────────────────────────┐
│  Simulation::set_joint_command(const JointCommand&) -> Status          │
│  Simulation::joint_state(string_view) -> Result<JointState>            │
│  职责: 公开 API 直接使用组件层原生类型，无中间适配类型                  │
└────────────────────────────────────────────────────────────────────────┘

┌─ 组件层 (mujoco_simulation/component/joint) ──────────────────────────┐
│  JointConfig / JointCommand / JointState / JointComponent             │
│  JointBinding (MuJoCo 原生索引映射)                                    │
│  职责: ComponentManager 内部使用的组件定义                              │
└────────────────────────────────────────────────────────────────────────┘
```

相比旧版三层重复定义（`JointData` → `SimulationJointConfiguration` → `JointConfig`），当前消除了 `Simulation*` 中间类型，`JointData` 直接持有设备层类型。IMU、Camera、Lidar、MobileBase 遵循相同模式。

### 8.3 错误处理策略

项目已统一为 `Status` / `Result<T>` 模式。

**`Status`**（[status.hpp](mujoco_simulation/include/mujoco_simulation/status.hpp)）：

```cpp
enum class StatusCode {
  Ok, InvalidArgument, AlreadyExists, InvalidState, FailedPrecondition,
  NotFound, ModelLoadFailed, ModelValidationFailed, BindingFailed,
  CommandRejected, RenderFailed, ThreadFailed, Timeout, Internal,
};

class Status {
 public:
  static Status Ok();
  static Status invalid_argument(std::string message);
  // ... 每种 StatusCode 对应一个具名工厂方法 ...
  bool ok() const noexcept;
  StatusCode code() const noexcept;
  const std::string& message() const noexcept;
};
```

**`Result<T>`**（[result.hpp](mujoco_simulation/include/mujoco_simulation/result.hpp)）：

```cpp
template <typename T>
class Result {
 public:
  Result(const T& value);     // 成功路径
  Result(const Status&);      // 失败路径
  bool ok() const noexcept;
  const Status& status() const noexcept;
  const T& value() const;
};
```

**统一后的签名规范：**

```cpp
// 无返回值操作（初始化、控制）—— 返回 Status
Status initialize(const SimulationConfig& config);
Status start();
Status set_joint_command(const JointCommand& command);

// 有返回值操作（读取状态）—— 返回 Result<T>
Result<JointState> joint_state(std::string_view joint_name) const;
Result<std::shared_ptr<const CameraSample>> camera_sample(std::string_view camera_name) const;
```

调用侧风格：

```cpp
const auto result = simulation.joint_state("joint_1");
if (!result.ok()) {
  RCLCPP_ERROR(logger, "%s", result.status().message().c_str());
  return hardware_interface::return_type::ERROR;
}
const JointState& state = result.value();
```

在 ROS 集成边界（`MuJoCoHardwareInterface`），`Status` 被转换为 `hardware_interface::return_type` 和 `hardware_interface::CallbackReturn`。

### 8.4 枚举与类型体系统一 ✅ 已完成

项目保留 `CommandInterfaceType`（`None` / `Position` / `Velocity` / `Effort`）作为唯一关节控制模式枚举。旧版 `SimulationJointCommandMode` 与相关双向转换函数（`to_backend_command_mode()` / `to_simulation_joint_mode()`）已移除。`config.cpp` 中的 `to_joint_control_mode()` 仅负责字符串到 `CommandInterfaceType` 的解析。

`JointConfig` 新增 `JointControllerType` 枚举（`MuJoCoActuator` / `SoftwarePd`），用于区分直接 actuator 控制和软件 PD 控制两种策略。`JointComponent` 内部根据 `controller_type` 选择不同的命令写入路径。

### 8.5 `Simulation*` 中间类型消除 ✅ 已完成

`MuJoCoSimulation` 公开 API 已直接使用组件层原生类型（`JointConfig`、`JointCommand`、`JointState` 等）。旧版 `SimulationJointConfiguration`、`SimulationJointCommand`、`SimulationJointState` 等中间类型和对应的字段搬运代码均已移除。

### 8.6 `MjContext` 统一注入 ✅ 已完成

设备构造所需的 MuJoCo 运行时资源统一收敛到 `MjContext` 值类型：

```cpp
struct MjContext {
  const mjModel* model{nullptr};
  mjData* data{nullptr};
  mjvScene* scene{nullptr};
  mjrContext* render{nullptr};
};
```

当前组件模式下，`MjContext` 不再直接传递给组件构造函数 —— 组件通过 `bind()` / `reset()` / `read()` / `write()` 等方法参数接收 `mjModel&` / `mjData&` 引用。

### 8.7 `Impl` / Pimpl 消除 ✅ 已完成

`Simulation` 不再使用 Pimpl 模式。`ModelRuntime`、`SimulationScheduler`、`ComponentManager`、`CommandBuffer`、`StateBuffer`、`CameraBuffer`、`CameraRenderer`、`Viewer` 均作为 `Simulation` 的直接成员或 `unique_ptr` 成员。

### 8.8 `render_hardware_manager` 消除 ✅ 已完成

旧版 `Impl` 中存在的双 `HardwareManager`（`hardware_manager` 管理 joint/imu/lidar，`render_hardware_manager` 管理 camera）已消除。`ComponentManager` 统一管理所有组件类型（含 Camera），`CameraRenderer` 提供独立渲染上下文。

### 8.9 robocasa 模块归属与 `robot_mujoco` 包设计 ✅ 已完成

`robocasa` 场景生成模块已从 `mujoco_simulation` 中提取到顶层 `robot_mujoco.robocasa` Python 包。import 路径从 `mujoco_simulation.robocasa` 变更为 `robot_mujoco.robocasa`。

**设计动机（保留为记录）：**

- robocasa 不调用 `mujoco_simulation` 的任何 C++ 代码，两者只是恰好放在同一目录下
- 放在 `mujoco_simulation` 中导致依赖声明不完整、部署耦合、测试隔离差、语言边界模糊
- `robot_mujoco` 包填补了工作区统一入口层的空缺，提供 `robot_mujoco.generate_scene()` 等 Python API

### 8.10 其他已完成的改进

**8.10.1 `config.cpp` 传感器类型缺失时报错 ✅**
`is_sensor_type()` 已增加 `error_message` 参数：`mujoco_type` 缺失时仅 IMU 向后兼容（返回 true），camera/lidar 显式报错。

**8.10.2 MobileBase 公开 API ✅**
`Simulation` 公开 API 已暴露 `set_mobile_base_command()` / `mobile_base_state()`；`HardwareConfig::mobile_bases` 字段已添加；`config.cpp` 支持 `mobile_base_<N>_*` 参数解析。

**8.10.3 物理循环时间精度 ✅**
`SimulationScheduler::worker_loop()` 每轮从 `clock::now()` 重新计算 wake_time，消除累积误差。

**8.10.4 `Robot` 类移除 ✅**
`MuJoCoHardwareInterface` 直接持有 `Simulation` 和 `SimulationRosBridge`，不再经过中间的 `Robot` 包装层。

**8.10.5 `SensorBridge` 迁移 ✅**
ROS 发布功能已从 `mujoco_hardware::SensorBridge` 迁移到 `mujoco_simulation_ros::SimulationRosBridge`。后者内部已实现 per-sensor publisher 拆分（`ImuPublisher` / `CameraPublisher` / `LidarPublisher`），每个 publisher 自管理 topic / frame_id / 启停标志。

**8.10.6 Camera 与 Viewer 解耦 ✅**
`CameraRenderer` 提供独立的离屏 OpenGL 渲染上下文（`OffscreenGlContext`），camera 不再依赖 viewer。支持 `render_mode=headless` 下的 camera 输出。

### 8.11 当前架构改进优先级

综合以上分析，按投入产出比排序：

| 优先级 | 改进项 | 说明 |
|--------|--------|------|
| P1 | ✅ 消除 `Simulation*` 中间类型 | `Simulation` 公开 API 直接使用组件层原生类型 |
| P1 | ✅ 提取 `robocasa` + 新建 `robot_mujoco` 包 | 场景生成已从 `mujoco_simulation` 解耦 |
| P1 | ✅ 合并重复枚举 | 统一为 `CommandInterfaceType` |
| P1 | ✅ `SensorBridge` 迁移为 `SimulationRosBridge` | ROS bridge 独立为 `mujoco_simulation_ros` 包 |
| P1 | ✅ 错误处理统一 | 全项目统一为 `Status` / `Result<T>` |
| P1 | ✅ `MuJoCoSimulation` -> `Simulation` 重构 | 引入 scheduler + buffer + component 架构 |
| P1 | ✅ Camera 与 Viewer 解耦 | `CameraRenderer` 独立离屏渲染 |
| P2 | ✅ 取消 `Impl` 配置 maps 副本 | 状态统一收口到 `ComponentManager` |
| P2 | ✅ 统一设备构造为组件模式 | `SimulationComponent` / `SensorComponent` 多态接口 |
| P2 | ✅ 消除 `render_hardware_manager` | camera 回到单一 `ComponentManager` |
| P2 | ✅ 消除 Pimpl | `Impl` 类移除 |
| P2 | ✅ `Robot` 类移除 | `MuJoCoHardwareInterface` 直接持有 `Simulation` |
| P2 | ✅ Per-sensor publisher 拆分 | `ImuPublisher` / `CameraPublisher` / `LidarPublisher` |
| P2 | ✅ 恢复测试基础设施 | unit / integration / concurrency / performance 测试 |
| P2 | ✅ MobileBase 公开 API | `Simulation` 暴露 mobile base 读写接口 |
| P3 | SnapshotObserver 异步化 | 将 ROS publish 从 scheduler 线程解耦 |
| P3 | Camera 渲染线程独立化 | 避免 GPU 渲染阻塞物理步进 |
| P3 | 补齐 launch 文件与端到端集成测试 | 覆盖完整链路 |
