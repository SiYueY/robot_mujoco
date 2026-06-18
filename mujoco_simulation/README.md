# mujoco_simulation

`mujoco_simulation` 是 `robot_mujoco` 工作区里的 MuJoCo 仿真运行时内核。

它的定位不是“完整的 ROS 2 仿真应用”，而是一个可复用的底层库，负责把 MuJoCo 的 `mjModel` / `mjData`、物理步进、viewer 和各类仿真设备封装成稳定的 C++ 接口，供上层模块复用。

## 参考项目

`mujoco_simulation` 不是凭空设计出来的，模块定位和实现思路明确参考了以下项目：

- `mujoco_ros2_control`
  - https://github.com/ros-controls/mujoco_ros2_control
  - 参考其 “MuJoCo + ROS 2 Control” 的总体接入方向，也就是把 MuJoCo 仿真运行时作为 `ros2_control` 后端来组织
- `mjlab`
  - https://github.com/mujocolab/mjlab
  - 主要参考其对 simulation layer 边界、sensor lifecycle、测试与调试手段的工程化处理，而不是其 manager-based RL framework 本身

当前 `mujoco_simulation` 可以理解为：结合这两类项目的启发，在本工作区里抽出一个更聚焦的“MuJoCo 运行时内核层”。

它既不是简单复刻 `mujoco_ros2_control`，也不是直接照搬 `mjlab`，而是面向当前工作区需求，对 MuJoCo 运行时、viewer 和设备抽象做的一层本地化收敛。

## 这个模块解决什么问题

如果只直接使用 MuJoCo 原生 C API，上层代码通常需要自己处理：

- 模型加载与销毁
- `mjData` 生命周期管理
- 仿真线程和步进节奏
- viewer 启动与同步
- joint、imu、camera、lidar 等对象的名字解析和读写映射

`mujoco_simulation` 把这些能力集中到一个运行时对象里：

```text
Simulation
  -> ComponentManager
    -> Joint / Imu / Camera / Lidar / MobileBase
  -> CameraRenderer + CameraBuffer
    -> mjModel / mjData
```

上层只需要面向 `Simulation` 调用，而不需要到处直接操作 MuJoCo 原生数组。

## 在工作区中的职责边界

这个包负责：

- 加载 MuJoCo MJCF/XML 模型
- 管理物理线程、暂停、重置、步进
- 按需启动 viewer
- 注册并读写仿真设备
- 提供对 MuJoCo 命名对象的查询能力

这个包不负责：

- `ros2_control` 插件导出
- URDF / `HardwareInfo` 参数解析
- ROS topic 发布
- controller 管理与 launch 编排

职责关系可以简单理解为：

- `mujoco_simulation`
  - 仿真运行时和设备抽象层
- `robot_mujoco_ros2`
  - 统一 ROS 2 adapter 层，同时承载 `ros2_control` 适配、ROS bridge、`/clock`、传感器 publisher 和 simulation control services
- `robot_mujoco`
  - 工作区聚合入口和上层运行封装

如果你现在在判断“这个模块该放什么代码”，一个简单原则是：

- 只要逻辑本质上是在操作 MuJoCo 仿真运行时本身，就优先放这里
- 只要逻辑本质上是在适配 ROS 2 接口，就不应该放这里

## 核心入口

主入口类是 [`Simulation`](./include/mujoco_simulation/simulation.hpp)。

它对外暴露的能力主要有：

- 初始化仿真
  - `Status initialize(const SimulationConfig&)`
  - `Status shutdown()`
- 控制运行
  - `Status start()`
  - `Status stop()`
  - `Status pause()`
  - `Status resume()`
  - `Status set_realtime_factor(double)`
  - `Status request_reset(const ResetOptions& = {})`
    - 异步提交 reset 请求，适合 ROS service / callback 线程
  - `Status reset(const ResetOptions& = {})`
    - 同步等待 reset 完成，适合需要明确执行结果的调用路径
  - `Status step(uint32_t)`
- 组件配置与设备访问
  - `SimulationConfig.components`
    - 唯一组件注册入口
  - `Status reconfigure_component(const ComponentConfig&)`
    - 运行时重配置入口；当前保证 `JointConfig` 可用，其他组件类型会显式返回错误
  - `Status set_joint_command(...)`
  - `Result<JointState> joint_state(...)`
  - `Result<ImuSample> imu_sample(...)`
  - `Result<std::shared_ptr<const CameraSample>> camera_sample(...)`
  - `Result<LidarSample> lidar_sample(...)`
  - `Status set_mobile_base_command(...)`
  - `Result<MobileBaseState> mobile_base_state(...)`
  - `state_snapshot()`

## 错误返回模型

公开运行时接口统一返回 `Status` 或 `Result<T>`。

- 成功路径只看 `ok()`
- 失败路径优先看 `code()`，再看 `message()`
- 当前公开错误码语义已经统一为：
  - `ModelLoadFailed`
  - `ModelValidationFailed`
  - `BindingFailed`
  - `CommandRejected`
  - `RenderFailed`
  - `ThreadFailed`
  - `Timeout`
  - 以及通用的 `InvalidArgument` / `InvalidState` / `FailedPrecondition` / `NotFound` / `AlreadyExists` / `Internal`

对下游调用者的建议是：把 `StatusCode` 当作稳定分支条件，把 `message()` 当作诊断信息，而不是反过来依赖字符串匹配。

## 运行配置

`SimulationConfig` 现在已经按重构文档收敛为嵌套配置：

- `model`
  - `model_path` 和 `initial_keyframe`
  - `initial_keyframe` 在 `Simulation::initialize() -> ModelRuntime::load(...)` 主路径中立即生效；缺失 keyframe 会直接导致初始化失败
- `scheduler`
  - `realtime_factor`、`viewer_update_rate` 等调度参数
- `components`
  - 组件配置列表，作为后续 `ComponentManager::build(...)` 的标准输入
  - IMU / Lidar / Camera 现在统一通过 `SensorCommonConfig common` 暴露 `name`、`frame_id`、`update_rate`
- `viewer`
  - viewer 侧配置
- `camera_renderer`
  - 离屏渲染资源配置
- `render_mode`
  - `headless` 或 `viewer`

当前 `render_mode` 的含义：

- `headless`
  - 只跑物理步进，不启动 viewer
- `viewer`
  - 启动 viewer，并以独立频率同步显示；camera 不再依赖 viewer 渲染资源
  - `stop()` 会销毁当前 viewer；后续同一 `Simulation` 实例再次 `start()` 时会按当前配置自动重建 viewer

`ResetOptions` 当前公开字段为：

- `keyframe_name` / `keyframe_id`
- `clear_commands`
- `reset_components`
- `clear_state_buffer`
- `clear_camera_buffer`
- `reset_statistics`

## 设备层能力

设备对象现在统一由 [`ComponentManager`](./include/mujoco_simulation/component/component_manager.hpp) 管理；`CameraRenderer` 和 `CameraBuffer` 只作为 Camera 组件采样时依赖的运行时资源，当前支持以下几类：

| 设备 | 作用 | 当前实现特点 |
| --- | --- | --- |
| `Joint` | 单关节状态/命令读写 | 主要面向 1-DoF joint，支持 position / velocity / effort 命令语义 |
| `Imu` | 组合多个 MuJoCo sensor 输出 IMU 状态 | 只读，依赖 `framequat` / `gyro` / `accelerometer` |
| `Camera` | 从渲染管线读取 RGB / depth 图像 | 通过统一的 `SensorComponent` 路径调度，渲染资源独立于 Viewer |
| `Lidar` | 由 `rangefinder` 传感器阵列拼装 `LaserScan` | 依赖 `<prefix>-<index>` 命名约定 |
| `MobileBase` | 底盘运动学封装 | 当前支持 differential / omnidirectional |

### 设备层的几个关键边界

- `Camera` 当前支持独立的 headless 渲染
  - camera 读取不再要求 `render_mode=viewer`
- `Joint` 当前是单关节、单标量接口
  - 不适合直接承载 `ball` / `free` 这类多自由度 joint
- `Lidar` 当前输出的是 `LaserScan`
  - 还不提供点云抽象
- `MobileBase` 是多个 traction joint 的组合包装
  - 它不是一个特殊 joint，而是更上层的运动学设备

更细的设备原理说明见：

- [`docs/hardware_devices_principles.md`](./docs/hardware_devices_principles.md)
- [`docs/refactor_roadmap.md`](./docs/refactor_roadmap.md)

## Viewer 的定位

viewer 相关代码位于 [`src/viewer`](./src/viewer)。

这里的 viewer 是“被动渲染前端”：

- `Simulation` 持有真正的 `mjModel` / `mjData`
- 物理步进仍然由 `Simulation` 驱动
- `Viewer` 只负责渲染和状态同步

Camera 渲染现在通过独立的 `CameraRenderer` 完成，不再复用 viewer 的渲染资源。

对 `Simulation` 而言，viewer 是可恢复的运行时资源：

- `initialize(render_mode=viewer)` 会创建 viewer
- `stop()` 会停止 scheduler 并销毁当前 viewer
- 后续再次 `start()` 时，如果当前仍是 `RenderMode::Viewer`，会自动重新创建 viewer

## 目录结构

```text
mujoco_simulation/
├── include/mujoco_simulation/
│   ├── simulation.hpp               # 对外主入口
│   ├── simulation_config.hpp        # RenderMode / SimulationConfig
│   ├── reset_options.hpp            # 顶层 reset 选项
│   ├── simulation_status.hpp        # SimulationStatus 兼容入口
│   ├── status.hpp / result.hpp      # 顶层基础返回类型
│   ├── runtime/                     # ModelRuntime / SimulationScheduler / RAII
│   ├── buffer/                      # CommandBuffer / StateBuffer / CameraBuffer
│   ├── scheduler/                   # SensorScheduler
│   ├── component/                   # 组件抽象与按类型分组的组件实现
│   │   ├── component_config.hpp     # ComponentConfig / ComponentConfigList
│   │   ├── joint/ imu/ lidar/       # 已拆出 config / binding / state(sample) 等类型头
│   │   ├── camera/                  # 已拆出 camera_config / binding / sample / renderer config / offscreen context
│   │   └── mobile_base/             # 已拆出 config / binding / command / state
│   ├── viewer/                      # viewer 对外接口
│   └── hardware/                    # 主要剩余 mj_context / data
├── src/
│   ├── runtime/                     # ModelRuntime / SimulationScheduler 实现
│   ├── buffer/                      # 预留给 buffer 独立实现
│   ├── scheduler/                   # SensorScheduler 实现
│   ├── component/                   # 组件管理与组件实现
│   │   ├── joint/ imu/ lidar/       # 新组件实现主路径
│   │   ├── camera/                  # Camera wrapper / CameraRenderer 实现
│   │   └── mobile_base/             # MobileBase 实现
│   ├── simulation.cpp               # 仿真主实现
│   ├── hardware/                    # 主要剩余轻量上下文与设备基础接口
│   └── viewer/                      # viewer 与官方 simulate 集成
├── test/
│   ├── unit/                        # 当前单元测试
│   ├── integration/                 # 预留给集成测试
│   ├── concurrency/                 # 预留给并发测试
│   └── performance/                 # 预留给性能测试
├── docs/
│   ├── hardware_devices_principles.md
│   ├── simulation_api_migration.md
│   ├── mujoco_simulation_refactoring.md
│   └── refactor_roadmap.md
├── CMakeLists.txt
└── package.xml
```

## 构建与依赖

这是一个 `ament_cmake` 包，当前主要依赖：

- `glfw3`
- `OpenGL`
- `mujoco`

核心库当前不再直接依赖 `hardware_interface`、`rclcpp` 或 `sensor_msgs`；这些 ROS 2 相关依赖保留在上层 `robot_mujoco_ros2` 适配层。

构建时直接通过 `find_package(mujoco CONFIG)` 解析 MuJoCo。
路径解析规则为：

- 如果设置了环境变量 `MUJOCO_ROOT`，优先使用它
- 否则默认使用 `/opt/mujoco-3.9.0`

例如：

```bash
export MUJOCO_ROOT=/opt/mujoco-3.9.0
colcon build --packages-select mujoco_simulation
```

在工作区中通常这样构建：

```bash
colcon build --packages-select mujoco_simulation
```

## 典型集成方式

这个包通常不单独作为最终入口使用，而是被上层包调用。

当前工作区里的典型调用链是：

```text
ros2_control
  -> robot_mujoco_ros2::MuJoCoHardwareInterface
    -> mujoco_simulation::Simulation
    -> robot_mujoco_ros2::SimulationRosBridge
```

也就是说：

1. `robot_mujoco_ros2` 解析 URDF / `HardwareInfo`
2. 它创建 `Simulation`
3. 它把 joint、imu、camera、lidar、mobile base 一次性写入 `SimulationConfig.components`
4. 它在 `read()` / `write()` 周期里转发状态和命令

当前 ROS 侧控制服务与传感器发布已经由 `robot_mujoco_ros2::SimulationRosBridge` 提供，统一由 `robot_mujoco_ros2` 包内部接线到 `Status` 风格控制回调。当前控制服务包括：

- `/start`
- `/stop`
- `/pause`
- `/resume`
- `/step`
- `/set_realtime_factor`
- `/load_keyframe`
- `/reset`

这些服务属于 `robot_mujoco_ros2` adapter 层，不属于 `mujoco_simulation` 本体 API；`mujoco_simulation` 只提供底层运行时控制能力。

其中：

- `/step`
  - 已支持显式指定步数
- `/set_realtime_factor`
  - 已支持在运行时更新实时倍率参数
- `/load_keyframe`
  - 已支持传入 keyframe 名称并触发对应 reset

如果你在做的是 ROS 2 接口对接，优先看 `robot_mujoco_ros2`；如果你在做的是 MuJoCo 运行时能力扩展，优先看这个包。

## 适合放在这里的改动

下面这些改动通常适合落在 `mujoco_simulation`：

- 增加新的 MuJoCo 设备抽象
- 扩展 `Simulation` 的运行时控制能力
- 优化 viewer 同步或渲染资源管理
- 增加对 `mjModel` / `mjData` 的结构化访问封装
- 扩展移动底盘的运动学模型

下面这些改动通常不适合放在这里：

- 新增 ROS 参数解析规则
- 新增 topic publisher 或 message bridge
- `ros2_control` interface 导出策略
- launch 文件、控制器编排、机器人应用逻辑

## 当前限制

截至当前实现，使用时需要特别注意：

- camera 已支持独立 headless 渲染
  - 但当前仍以 RGB / depth 采样为主，还没有扩展到 segmentation / object id 等更复杂渲染输出
- joint 抽象目前偏向 1-DoF 控制接口
- lidar 依赖传感器命名规则
- mobile base 只覆盖 differential / omnidirectional
- 这个包本身是底层库，不是开箱即用的完整仿真应用

如果你的目标是“启动一个 ROS 2 机器人仿真”，通常不应直接从这里起步，而应从 `robot_mujoco_ros2` 或 `robot_mujoco/launch` 看整体接入链路。
