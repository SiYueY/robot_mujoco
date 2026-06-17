# robot_mujoco 项目代码架构导读

> 说明：本文件保留为阶段性设计推导记录。自 2026-06-16 起，当前真实包边界与迁移入口以 [`current_architecture.md`](./current_architecture.md) 和 [`migration_guide.md`](./migration_guide.md) 为准；ROS publisher/service 已从 `mujoco_hardware::SensorBridge` 迁移到 `mujoco_simulation_ros::SimulationRosBridge`。

## 1. 项目定位与包划分

`robot_mujoco` 是一个面向 ROS 2 的通用 MuJoCo 仿真工作区。仓库当前默认示例机器人是 Franka / Panda，但代码结构本身并不绑定某一台具体机器人，目标是提供一套可复用的 MuJoCo 运行时、设备抽象和 `ros2_control` 接入层。

当前仓库可以分成三个主要部分：

- `mujoco_hardware`
  - `ros2_control` 硬件插件层。
  - 负责把 URDF / `ros2_control` 的 `HardwareInfo` 参数解析成运行时配置，并把仿真状态映射为 ROS 侧的 state / command interface 和传感器消息。
- `mujoco_simulation`
  - MuJoCo 仿真内核与设备抽象层。
  - 负责模型加载、仿真线程、viewer、设备注册和对 `mjModel` / `mjData` 的结构化访问。
- `robot_mujoco/robocasa`
  - Python 场景生成子系统。
  - 负责从 RoboCasa / robosuite 任务生成 MJCF，再适配成本项目可直接使用的机器人场景。

从职责边界看，`mujoco_hardware` 偏 ROS 集成，`mujoco_simulation` 偏仿真运行时，`robocasa` 偏场景前处理。

> **架构注记：** `robocasa` 子系统现已提取到仓库级 Python 包 `robot_mujoco.robocasa`，不再随 `mujoco_simulation` 的 C++/ament 构建一起安装。这一调整消除了此前的职责错位、依赖污染和部署耦合问题。背景分析见 [7.8 节](#78-robocasa-模块归属与-robot_mujoco-包设计)。

## 2. 顶层架构图与调用链

项目核心调用链可以概括为：

```text
ros2_control controller manager
  -> mujoco_hardware::MuJoCoHardwareInterface
    -> mujoco_simulation::MuJoCoSimulation
      -> mujoco_simulation::HardwareManager
        -> Joint / Imu / Camera / Lidar / MobileBase
          -> MuJoCo mjModel / mjData
```

同时，传感器数据还有一条发布链：

```text
MuJoCoSimulation read_*()
  -> MuJoCoHardwareInterface::update_runtime_state()
    -> SensorBridge
      -> sensor_msgs topics
```

可以把运行过程理解为两条并行职责：

- 控制链
  - `ros2_control` 通过 `read()` / `write()` 和 mode switch 与 `MuJoCoHardwareInterface` 交互。
  - `MuJoCoHardwareInterface` 把关节命令转为仿真层的 joint command，并回读关节状态。
- 观测链
  - 仿真层读取 IMU、Camera、Lidar 状态。
  - `SensorBridge` 把这些状态转成 `sensor_msgs` 并发布。

`mujoco_simulation::MuJoCoSimulation` 是运行时中心：它持有 `mjModel`、`mjData`、物理线程、viewer 以及设备管理器；`mujoco_hardware` 只通过公开接口与它交互，不直接操作 MuJoCo 原生结构。

## 3. `mujoco_hardware` 模块拆解

### 3.1 `config.cpp`：把 `HardwareInfo` 解析成运行时配置

`mujoco_hardware/src/config.cpp` 的职责是把 `hardware_interface::HardwareInfo` 解析成 `HardwareConfig`，供后续 `MuJoCoHardwareInterface` 直接使用。它本质上是“ROS 参数世界”和“MuJoCo 运行时对象”之间的翻译层。

关键仿真配置入口：

- `mujoco_model_path`
  - 必填，MuJoCo 模型路径。
- `render_mode`
  - 渲染模式，默认 `headless`，由 `mujoco_simulation::parse_render_mode()` 解析。
- `sim_speed_factor`
  - 仿真速度倍率，默认 `1.0`。
- `initial_keyframe`
  - 可选，初始化时重置到指定 MuJoCo keyframe。

关键传感器参数入口：

- IMU
  - `mujoco_orientation_sensor`
  - `mujoco_gyro_sensor`
  - `mujoco_accel_sensor`
  - `frame_id`
  - `topic`
- Camera
  - `mujoco_camera_name`
  - `optical_frame_id`，若未设置则回退到 `frame_id`，再回退到传感器名
  - `image_topic`
  - `depth_topic`
  - `camera_info_topic`
  - `enable_rgb`
  - `enable_depth`
  - `width`
  - `height`
- Lidar
  - `frame_id`
  - `scan_topic`
  - `sensor_prefix`
  - `angle_min`
  - `angle_max`
  - `angle_increment`
  - `range_min`
  - `range_max`

几个关键实现特征：

- joint 只接受 position / velocity / effort 三类 command interface。
- IMU 默认被视为传感器默认类型；camera 和 lidar 依赖 `mujoco_type` 区分。
- camera 在当前实现中要求 `render_mode=viewer`，否则解析阶段直接报错。
- 当前项目暂不考虑纯 `headless` 仿真下的 camera 输出能力；camera 依然按 viewer 渲染路径理解。
- 配置解析失败时会生成清晰的错误字符串，供 `on_init()` 直接中止初始化。

### 3.2 `MuJoCoHardwareInterface`：`ros2_control` 插件入口

`mujoco_hardware::MuJoCoHardwareInterface` 继承 `hardware_interface::SystemInterface`，是 `mujoco_hardware_plugin.xml` 导出的插件实现。

其职责分成四类：

- 生命周期管理
  - `on_init()` 调用配置解析，创建 `MuJoCoSimulation`，注册关节和传感器，并为 camera 预填充 `CameraInfo` 内参。
  - `on_activate()` / `on_deactivate()` 控制底层仿真启停。
- 接口导出
  - `export_state_interfaces()` 导出 joint 和 IMU 的 state interface。
  - `export_command_interfaces()` 导出 joint command interface。
- 模式切换
  - `prepare_command_mode_switch()` 校验要切换的接口是否合理。
  - `perform_command_mode_switch()` 把 mode 变化真正下发到底层 `MuJoCoSimulation`。
- 运行时读写
  - `read()` 负责刷新关节和传感器状态，并触发 `SensorBridge` 发布消息。
  - `write()` 负责把 controller 的命令写入仿真层。

这里最重要的调用点是 `update_runtime_state()`：

- 先用仿真时间构造 ROS 时间戳。
- 依次调用 `simulation_->read_joint()`、`read_imu()`、`read_camera()`、`read_lidar()`。
- 读到的 IMU / Camera / Lidar 状态立即交给 `sensor_bridge_` 发布。

这意味着 `MuJoCoHardwareInterface` 不只是控制插件，也是传感器数据从仿真层流向 ROS 消息总线的汇聚点。

### 3.3 `MuJoCoHardwareInterface` 与仿真层的直接连接

当前代码已经移除了 `mujoco_hardware::Robot` 这一层薄包装，`MuJoCoHardwareInterface` 直接持有 `std::unique_ptr<mujoco_simulation::Simulation>`。

这意味着它当前同时承担三类职责：

1. **仿真生命周期管理**：创建 `Simulation`，准备组件配置，并在 activate / deactivate 时直接控制 `start()` / `stop()`
2. **模式切换与读写转发**：直接调用 `reconfigure_component(...)`、`set_joint_command()`、`state_snapshot()` 和 `camera_sample()`
3. **相机内参推导**：通过本地 `fill_camera_info()` helper 将 MuJoCo camera 几何参数映射为 ROS `CameraInfo`

这一改动的直接收益是调用链缩短、状态来源减少，也让 `mujoco_hardware` 内部不再保留一个纯转发型 façade。

### 3.4 `SensorBridge`：ROS 消息发布层

`mujoco_hardware::SensorBridge` 专注于消息发布，不参与控制逻辑。

它在构造时创建：

- `sensor_msgs::msg::Imu` publisher
- `sensor_msgs::msg::Image` publisher
- `sensor_msgs::msg::CameraInfo` publisher
- `sensor_msgs::msg::LaserScan` publisher

发布行为的几个要点：

- 所有消息时间戳都来自仿真时间，通过 `set_time()` 注入。
- camera 的 RGB、depth、camera info 可以分别启停。
- lidar 若没有 intensity 数据，会按 range 数量填充全 0 的 `intensities`。
- `SensorBridge` 依赖前面配置阶段写入的 topic / frame_id，不自己做额外命名推导。

## 4. `mujoco_simulation` 模块拆解

### 4.1 `MuJoCoSimulation`：仿真会话与线程入口

`mujoco_simulation::MuJoCoSimulation` 是整个仿真运行时的中心类，对上层暴露一个稳定的 C++ API。

它负责：

- 加载 MuJoCo 模型
  - `initialize()` 内部调用 `load_model()`
- 根据 `RenderMode` 决定是否启动 viewer
- 在指定 keyframe 上重置
- 管理物理线程
  - `start()`
  - `stop()`
  - `physics_loop()`
- 支持运行时控制
  - `set_paused()`
  - `reset()`
  - `step()`
- 暴露设备注册和读写接口
  - `register_joint()`
  - `register_imu()`
  - `register_camera()`
  - `register_lidar()`
  - `write_joint()`
  - `read_*()`

它内部持有：

- `mjModel* model_`
- `mjData* data_`
- `std::mutex mutex_`
- 物理线程和运行状态原子变量
- `Viewer`
- `Impl`，其中当前只保存 `HardwareManager`

这个类的意义在于：把 MuJoCo 的原生 C API 封装成一个带生命周期、线程和错误返回的仿真会话对象，供上层系统安全调用。

### 4.2 `HardwareManager`：设备注册表与调度层

`mujoco_simulation::HardwareManager` 按类别维护设备对象：

- `Joint`
- `Imu`
- `Camera`
- `Lidar`
- `MobileBase`

对每类设备都提供统一风格的接口：

- `register_*()`
- `unregister_*()`
- `read_*()`
- `write_*()`，若该设备支持写
- `read_*_states()`，部分类型支持批量读取

它持有对 MuJoCo 运行时资源的引用：

- `const mjModel*`
- `mjData*`
- `mjvScene*`
- `mjrContext*`

因此设备对象不直接拥有仿真上下文，而是由 `HardwareManager` 统一注入。这一层是设备抽象的管理边界，不承担高层控制策略。

在当前实现中，`HardwareManager` 不只是设备对象容器，也是设备注册状态的唯一事实源：joint / imu / lidar 的配置快照、camera 的已注册配置以及 viewer 未就绪时的 pending camera spec 都由它内部维护，`MuJoCoSimulation` 不再额外保存一份配置副本。

另外，当前设备构造也已经统一收敛到一个内部 `MjContext` 值类型：`HardwareManager` 持有 `model/data/scene/render` 四元组，并将其作为单一上下文对象传给各设备，而不是在每个构造点手工展开多组 MuJoCo 原生指针。

### 4.3 设备实现：对 `mjModel` / `mjData` 的结构化访问

`mujoco_simulation/hardware` 下的设备类负责把 MuJoCo 原生对象包装成统一接口。更深入的设备原理说明已经写在 [mujoco_simulation/docs/hardware_devices_principles.md](/home/siyuey/workspace/franka/franka_mujoco/robot_mujoco/mujoco_simulation/docs/hardware_devices_principles.md)。

从项目级视角看，可以把这些设备理解为：

- `Joint`
  - 封装关节位置、速度、effort 读取，以及 position / velocity / effort 命令写入。
- `Imu`
  - 从多个 MuJoCo sensor 组合出一个 IMU 状态。
- `Camera`
  - 依赖 MuJoCo 渲染管线生成 RGB / depth 图像和最小相机信息。
- `Lidar`
  - 依据 `rangefinder` 传感器阵列拼装 `LaserScan`。
- `MobileBase`
  - 为移动底盘类对象预留统一抽象，但当前项目主链路仍以 joint 和传感器为主。

这些类共同的设计方向是：对上层暴露统一初始化、读写和错误处理风格，对下层集中消化 MuJoCo 数据布局差异。

### 4.4 viewer 集成：基于官方 `simulate`

`mujoco_simulation/src/viewer/simulate` 目录来自 MuJoCo 官方 `simulate` 工程的集成版本，配合 `viewer/viewer.cpp` 使用。

当前 viewer 层主要承担：

- 建立 GLFW / OpenGL 渲染上下文
- 驱动 `mjvScene` 与 `mjrContext`
- 为 camera 设备读取像素提供渲染依赖
- 在 `RenderMode::Viewer` 下同步显示仿真画面

这部分代码的定位更接近”官方 viewer 的工程化嵌入”，而不是项目特有业务逻辑。

**Camera 与 Viewer 的当前耦合：** 当前实现中 camera 设备强依赖 viewer 渲染路径——`config.cpp` 要求 camera 必须在 `render_mode=viewer` 下使用，否则解析阶段直接报错。这意味着：

- 当前若不启动 viewer，就无法通过正常路径读取 camera
- 可视化工具与传感器功能耦合在一起，违反了单一职责原则
- Camera 的渲染依赖（`mjvScene` / `mjrContext`）目前由 viewer 负责创建和管理，并在 viewer 启动后注入同一个 `HardwareManager`

**当前取舍：** 现阶段暂不把“纯 headless 仿真下的 camera 输出”作为目标，因此这里的核心约束应理解为“camera 依赖 viewer 渲染路径”。当前内部实现已经收敛到单一 `HardwareManager`：camera 配置可先缓存，viewer 就绪后再补注册并复用同一套设备管理路径。

## 5. RoboCasa 场景生成简述

`robot_mujoco/robocasa` 是一个 Python 子系统，用来把 RoboCasa 的厨房任务场景转换为本项目可直接加载的 MJCF。

核心文件职责如下：

- `scene_config.py`
  - 定义 `SceneConfig`。
  - 负责从 YAML 读取 `robocasa`、`robot`、`output` 三组配置，并校验 `task_name`、`layout_id`、`style_id`、`robot_xml_path`、`control_freq` 等字段。
- `scene_cli.py`
  - 提供命令行入口。
  - 负责加载基础 YAML、接收 `--task` / `--layout` / `--style` / `--output` 参数，并驱动场景生成。
- `scene_generator.py`
  - 主流程入口。
  - 负责创建 RoboCasa / robosuite 环境、触发模型加载、提取原始 MJCF、抽取对象摆放和推荐机器人出生位姿，再调用适配器生成最终 XML。
- `mjcf_adapter.py`
  - 负责对 RoboCasa 输出的 MJCF 做项目级变换。
  - 典型操作包括移除 placeholder robot、清理顶层 runtime 元素、插入项目机器人 `include`、应用 object placement、应用 spawn pose、补 floor geom、隐藏 debug marker。

整体链路如下：

```text
RoboCasa YAML config
  -> SceneConfig
    -> scene_cli.py
      -> SceneGenerator.generate()
        -> robosuite / robocasa 生成厨房 MJCF
        -> 提取 object placements / spawn pose
        -> adapt_mjcf()
          -> 替换 placeholder robot
          -> include 本项目 robot XML
          -> 输出最终 MJCF
```

这里的关键点不是 RoboCasa 本身，而是本项目如何”借用 RoboCasa 生成环境，再替换成自己的机器人模型”。因此它更像场景装配流水线，而不是运行时仿真引擎的一部分。

> **架构注记：** `robocasa` 是一个纯 Python 模块——它只依赖 `mujoco`、`robosuite`、`robocasa` 等 Python 包，不调用 `mujoco_simulation` 的任何 C++ 代码。当前它已经提取到顶层 `robot_mujoco.robocasa` 包，不再通过 `mujoco_simulation` 的 `ament_python_install_package` 安装。关于这次提取的设计动机，详见 [7.8 节](#78-robocasa-模块归属与-robot_mujoco-包设计)。

## 6. 当前代码现状与边界

### 6.1 当前仓库现状

- 根目录 `docs/` 在新增本文件之前为空。
- 仓库已经有一份设备层设计文档：[mujoco_simulation/docs/hardware_devices_principles.md](/home/siyuey/workspace/franka/franka_mujoco/robot_mujoco/mujoco_simulation/docs/hardware_devices_principles.md)，其粒度比本文更细，适合在理解项目总结构后继续深入。
- 根目录 [README.md](/home/siyuey/workspace/franka/franka_mujoco/robot_mujoco/README.md) 目前主要覆盖环境初始化、RoboCasa 依赖安装和基础验证，偏“搭建指南”，不是架构说明。
- 当前仓库不包含单元测试代码，测试目录及相关 CMake 配置已移除。
- 已完成的 P1 重构包括：`Simulation*` 中间类型消除、`CommandInterfaceType` 统一为唯一关节控制模式枚举、`robocasa` 提取到顶层 `robot_mujoco` Python 包。

### 6.2 已知实现边界

- camera 依赖 viewer 渲染路径，当前实现要求 `render_mode=viewer`。详见 [4.4 节](#44-viewer-集成基于官方-simulate) 和 [7.5 节](#75-camera-与-viewer-的内部收敛) 的分析。
- 当前阶段暂不考虑纯 `headless` 仿真下的 camera 输出能力，因此 camera-viewer 耦合仍被视为现状约束，而不是当前实现目标。
- `mujoco_hardware` 导出 joint state / command interface，以及 IMU state interface；camera 和 lidar 主要通过 ROS topic 输出，而不是通过 `ros2_control` state interface 暴露。
- `SensorBridge` 只是 publisher 容器，不负责独立线程、节流或录包逻辑。
- `MuJoCoHardwareInterface` 当前直接连接 `MuJoCoSimulation`，不再经过额外的 `Robot` 包装层；相机内参推导 helper 也已并回该类。详见 [3.3 节](#33-mujocohardwareinterface-与仿真层的直接连接)。
- RoboCasa 子系统负责”生成场景 XML”，不负责运行时场景调度。它是纯 Python 模块，现已从 `mujoco_simulation` 中提取到顶层 `robot_mujoco.robocasa` 包。详见 [7.8 节](#78-robocasa-模块归属与-robot_mujoco-包设计)。
- `JointData/ImuData/CameraData/LidarData` 仍然是 `mujoco_hardware` 侧的 ROS 适配包装，但 `MuJoCoSimulation` 公开 API 已直接使用设备层原生类型；原先 `Simulation*` 中间类型已被消除。详见 [7.2 节](#72-数据结构分层与重复)。
- 错误处理在 `MuJoCoSimulation`（`bool + std::string*`）、`HardwareManager`（`last_error()`）和部分内部 helper（纯 `bool`）之间仍有混用，详见 [7.3 节](#73-错误处理策略)。

### 6.3 阅读顺序建议

建议按下面顺序阅读：

1. 先看 [README.md](/home/siyuey/workspace/franka/franka_mujoco/robot_mujoco/README.md)，建立依赖、目录和运行前置条件的基本认识。
2. 再看本文第 1–6 节，建立项目级与模块级心智模型。
3. 再看本文第 7 节（架构深入与改进方向），理解当前设计中的权衡、已知局限和改进路线图。
4. 如果需要深入设备层实现，再看 [mujoco_simulation/docs/hardware_devices_principles.md](/home/siyuey/workspace/franka/franka_mujoco/robot_mujoco/mujoco_simulation/docs/hardware_devices_principles.md)。

这样阅读更接近代码的真实分层：先看工作区怎么搭起来，再看各模块怎么协作，再看架构层面的权衡与改进方向，最后钻进单个设备类的内部原理。

---

## 7. 架构深入与改进方向

本章从架构设计角度审视项目当前实现中的关键权衡、已知局限和可优化方向。内容不是缺陷列表，而是帮助开发者在后续迭代中做出知情决策的参考。

### 7.1 线程模型与并发保证

项目中存在三个主要的执行上下文，理解它们的交互关系对于避免并发 bug 至关重要。

**三类执行线程：**

```text
┌─ ros2_control 线程 ──────────────────────────────────────────────────┐
│  controller_manager → MuJoCoHardwareInterface::read() / write()      │
│  调用栈中包含: MuJoCoHardwareInterface::update_runtime_state()      │
│              → MuJoCoSimulation::read_*()                           │
│              → HardwareManager::read_*() → 设备对象::read()           │
│              → SensorBridge::publish_*()  (同步发布 ROS 消息)         │
└──────────────────────────────────────────────────────────────────────┘

┌─ 物理仿真线程 ───────────────────────────────────────────────────────┐
│  MuJoCoSimulation::physics_loop()                                    │
│  循环中: mj_step() → 更新 mjData                                     │
│  受 mutex_ 保护，与 ros2_control 线程共享 mjModel / mjData            │
└──────────────────────────────────────────────────────────────────────┘

┌─ Viewer 线程 (GLFW 主循环) ──────────────────────────────────────────┐
│  仅在 RenderMode::Viewer 下存在                                       │
│  驱动渲染: mjv_updateScene() → mjr_render()                          │
│  处理 GUI 事件、键盘鼠标输入                                           │
└──────────────────────────────────────────────────────────────────────┘
```

**数据共享与锁策略：**

| 共享资源 | 访问者 | 保护机制 |
|---------|--------|---------|
| `mjModel*` | 只读，所有线程 | 无需锁（MuJoCo 保证 model 不可变） |
| `mjData*` | 物理线程（写） + ros2_control 线程（读） | `MuJoCoSimulation::mutex_`（`mutable std::mutex`） |
| `mjvScene` / `mjrContext` | Viewer 线程 + Camera 设备（ros2_control 线程） | 当前无显式锁，依赖调用时序 |
| `HardwareManager` 设备注册表 | ros2_control 线程 | 初始化阶段写入，运行时只读，无需锁 |
| `std::atomic_bool running_/paused_` | 多线程 | 原子变量，无锁 |

对外暴露的线程安全接口：

- `MuJoCoSimulation::with_locked_data(callback)` — 在持有 `mutex_` 期间执行回调，用于安全的批量数据访问
- `MuJoCoSimulation::copy_data_to(dest)` — 加锁拷贝当前 `mjData` 到用户提供的缓冲区

**已知风险与注意事项：**

1. **Camera 读取在控制线程中执行 GPU 渲染**：`read_camera()` 内部调用 `mjv_updateScene()` + `mjr_render()` + `mjr_readPixels()`，涉及 GPU 管线，耗时远大于 joint/IMU/Lidar 读取。在 `ros2_control` 的 `read()` 周期中同步执行可能超时。若未来对实时性要求严格，可考虑将 camera 读取和发布移到独立线程，通过异步队列将结果传回。

2. **`SensorBridge::publish_*()` 在 `read()` 调用栈中同步执行**：若某个 ROS publisher 因网络或订阅者问题阻塞，可能拖慢整个控制循环。后续可考虑引入无锁队列或独立发布线程。

3. **`mjvScene` / `mjrContext` 的生命周期**：camera 设备在 `read()` 中访问这些对象时，viewer 线程可能同时操作它们。当前这些对象由 viewer 管理生命周期，camera 隐式依赖 viewer 的存在。解耦后需要显式管理渲染上下文的共享所有权。

4. **`last_error()` 模式不是线程安全的**：`HardwareManager` 使用 `last_error_` 字符串存储错误状态，多线程环境下后一次错误会覆盖前一次。这个模式只适用于单线程场景，或仅用于初始化阶段。

### 7.2 数据结构分层与重复

项目中同一设备概念（以 joint 为例）在三层中各自定义了结构体：

```text
┌─ 硬件插件层 (mujoco_hardware) ────────────────────────────────────────┐
│  JointData { name, command_interfaces, state_interfaces,              │
│              mujoco_simulation::JointConfig info,                        │
│              mujoco_simulation::JointCommand command,                  │
│              mujoco_simulation::JointState state }                     │
│  定义于: mujoco_hardware/data.hpp                                      │
│  职责: ROS HardwareInfo 解析结果 + 设备层 info/command/state            │
└────────────────────────────────────────────────────────────────────────┘

┌─ 仿真公开接口层 (mujoco_simulation.hpp) ───────────────────────────────┐
│  SimulationJointConfiguration { name, actuator_name, command_mode }    │
│  SimulationJointCommand { name, position, velocity, acceleration,      │
│                           effort }                                     │
│  SimulationJointState { name, position, velocity, effort }             │
│  定义于: mujoco_simulation/mujoco_simulation.hpp                       │
│  职责: MuJoCoSimulation 公开 API 参数                                   │
└────────────────────────────────────────────────────────────────────────┘

┌─ 设备层 (mujoco_simulation/hardware) ─────────────────────────────────┐
│  JointConfig { name, joint_type, actuator_type, ... }                    │
│  JointCommand { position, velocity, effort }                           │
│  JointState { position, velocity, effort }                             │
│  定义于: mujoco_simulation/hardware/joint.hpp                           │
│  职责: HardwareManager 内部使用的设备数据                                │
└────────────────────────────────────────────────────────────────────────┘
```

**问题分析：**

- 三层结构体存在明显的字段重复（position / velocity / effort 在三层各定义一次）
- `Robot` 的 `register_joint()` / `read_joint()` / `write_joint()` 大量代码是字段搬运：从 `JointData` 取值 → 组装 `SimulationJoint*` → 调用 `MuJoCoSimulation`
- IMU、Camera、Lidar 同样存在类似的三层结构重复
- 新增一种设备类型需要同时修改三层的数据定义和转换代码

**优化方向：**

1. **以设备层类型为权威定义**：仿真公开接口 `MuJoCoSimulation` 直接使用 `JointConfig` / `JointCommand` / `JointState`（即设备层类型），删除 `SimulationJointConfiguration` 等中间类型
2. **`JointData` 简化为薄包装**：只持有设备层类型 + ROS 专有字段（`command_interfaces` / `state_interfaces` 字符串数组），不再重复定义数据字段
3. **`mujoco_hardware` 侧的转换代码大幅减少**：因为 `JointData` 的 info/command/state 子对象就是 `MuJoCoSimulation` 接受的类型

建议的重构后的数据流（以 joint 为例）：

```text
config.cpp 解析 HardwareInfo
  → 直接填充 JointConfig / JointCommand 字段
  → 存入 JointData.info / JointData.command
    → MuJoCoHardwareInterface 直接传递 JointData.info
      → MuJoCoSimulation::register_joint(JointConfig)
        → HardwareManager::register_joint(JointConfig)
```

注意：这一优化的前提是确认 `mujoco_simulation.hpp` 中的公开类型不被外部依赖锁定。若已有稳定 API 约定，可先标记旧类型为 deprecated，逐步迁移。

### 7.3 错误处理策略

当前项目中错误处理存在三种模式的混用：

| 模式 | 位置 | 签名示例 |
|------|------|---------|
| 返回值 + 输出参数 | `MuJoCoSimulation` 公开 API | `bool func(..., std::string *error_message = nullptr)` |
| `last_error()` 模式 | `HardwareManager` | `const std::string& last_error() const` |
| 布尔返回值（无错误信息） | 部分内部方法 | `bool func(...)` |

**各自的问题：**

- **返回值 + `std::string*`**：调用者必须同时检查 `bool` 返回值和非空的 `error_message`，容易遗漏。此外，`nullptr` 作为默认参数意味着错误信息可能被静默丢弃。
- **`last_error()` 模式**：(1) 非线程安全——后续错误会覆盖前一个；(2) 语义模糊——无法区分"无错误"和"未被检查的旧错误"；(3) 调用者必须记得在操作后立即读取 `last_error()`。
- **布尔返回值无错误信息**：调试时只能靠日志，无法将错误上下文传递给调用者。

**推荐方案：**

当前 `MuJoCoSimulation` 已经使用 `bool func(..., std::string* error_message)` 作为公开 API 签名，这是项目中覆盖面最广的模式。推荐的改进方向不是引入新类型或新库，而是**就地修正两个细节，然后全项目统一推广**：

**第 1 步：消除 `= nullptr`，强制错误捕获**

```cpp
// 改造前：nullptr 默认值允许调用者静默丢弃错误信息
bool register_joint(const JointConfig& info, std::string* error_message = nullptr);

// 改造后：引用语义，编译器强制调用者提供 error_message
bool register_joint(const JointConfig& info, std::string& error_message);
```

改动仅一个字符（`*` → `&`，删除 `= nullptr`），效果是调用者无法再写 `register_joint(info)` 而不处理错误——编译器直接报错。

**第 2 步：消灭 `last_error()` 模式**

```cpp
// 改造前：HardwareManager 使用 last_error() 存储错误
manager.register_joint(info);
if (!manager.last_error().empty()) { /* 处理 */ }

// 改造后：统一为 bool + string&
std::string err;
if (!manager.register_joint(info, err)) { /* 处理 */ }
```

`last_error()` 本质上是把调用拆成两步（操作 + 查询），这破坏了原子性——两次调用之间另一个线程可能修改错误状态。改为直传引用后，操作和错误信息在一步内完成。

**统一后的签名规范：**

```cpp
// 无返回值操作（初始化、注册）—— string& 跟在最后一个业务参数之后
bool initialize(const Config& config, std::string& error_message);
bool register_joint(const JointConfig& info, std::string& error_message);

// 有返回值操作（读取状态）—— T* 输出参数 + string& 错误信息
bool read_joint(const std::string& name, JointState* state, std::string& error_message);
bool read_imu(const std::string& name, ImuSample* state, std::string& error_message);
```

调用侧风格一致且直接：

```cpp
std::string err;
JointState state;

if (!simulation.read_joint("joint_1", &state, err)) {
    RCLCPP_ERROR(logger, "read_joint failed: %s", err.c_str());
    return hardware_interface::return_type::ERROR;
}
// 正常使用 state
```

**为什么不引入 `tl::expected` 或自定义 `Result<T>`：**

- 引入新类型意味着项目中所有人需要学习新的调用惯用语法（`.and_then()` / `.or_else()` 等）
- 模板类型在调试器中展开后一行调用变成多层嵌套，增加排查难度
- `bool + string&` 是 C++ 最基础的语法，任何开发者都能一眼看懂，零学习成本
- 项目已经以此模式为主——统一推广的成本远低于替换为新类型

**选择建议**：全项目统一为 `bool func(..., std::string& error_message)`。

在 ROS 集成边界（`MuJoCoHardwareInterface`），错误信息需要转换为 `hardware_interface::return_type` 和 `hardware_interface::CallbackReturn`——这是转换的自然位置，不应让仿真层或设备层直接返回 ROS 类型。

### 7.4 SensorBridge 的职责边界

`SensorBridge` 目前同时管理四种 ROS publisher（IMU、Camera RGB、Camera Depth、CameraInfo、Lidar），并在 `publish_*()` 方法中做数据填充和消息构造。

**当前结构下的潜在问题：**

- 随着传感器类型增加，`SensorBridge` 持续膨胀
- 每个 `publish_*()` 方法内部都有 topic / frame_id / enable 标志的条件判断
- 测试时无法单独验证某类传感器的发布逻辑

**优化方向：**

建议将 `SensorBridge` 拆分为独立的 per-sensor publisher 类：

```text
SensorBridge (holder / coordinator)
  ├── ImuPublisher        → sensor_msgs::msg::Imu
  ├── CameraPublisher     → sensor_msgs::msg::Image (rgb) + Image (depth) + CameraInfo
  └── LidarPublisher      → sensor_msgs::msg::LaserScan
```

每个 publisher 自管理其 topic 名称、frame_id、启停标志。`SensorBridge` 退化为一个工厂/协调器，在 `update_runtime_state()` 中调度各 publisher。

### 7.5 Camera 与 Viewer 的内部收敛

详见 [4.4 节](#44-viewer-集成：基于官方-simulate) 中关于当前耦合的阐述。当前阶段不做 headless camera，但仍值得先把 viewer-only 路径内部收干净。

**本轮已完成的收敛：**

- 保留 `render_mode=viewer` 作为 camera 的唯一支持模式
- 删除 `render_hardware_manager` 专用分支，camera 回到单一 `HardwareManager`
- viewer 启动后将 `mjvScene` / `mjrContext` 注入现有 `HardwareManager`
- 保留“先缓存 camera 配置，viewer 就绪后补注册”的外部初始化语义

**当前边界：**

- camera 仍然依赖 viewer 渲染资源
- 不支持纯 `headless` 仿真下输出 camera
- `config.cpp` 继续在解析阶段拦截 `render_mode!=viewer` 的 camera 配置

**未来若要继续推进，下一跳应是新的能力引入，而不是继续清理当前 viewer-only 链路：**

1. 为 `Camera` 增加脱离 viewer 的独立渲染上下文
2. 再放开 `render_mode=headless` 下的 camera 配置语义
3. 最后才讨论 viewer 与 camera 之间更彻底的资源共享或分离

### 7.6 测试基础设施

当前仓库不包含单元测试代码（测试目录已移除）。对于仿真系统而言，缺少自动化测试带来的风险高于一般应用——物理参数、设备接口、配置解析的正确性无法通过代码审查保证。

**推荐的测试分层：**

| 测试层 | 范围 | 说明 |
|--------|------|------|
| `config.cpp` 单元测试 | 配置解析 | 提供多组 `HardwareInfo` 输入，验证解析结果的正确性和边界错误处理 |
| 设备层单元测试 | `Joint` / `Imu` / `Lidar` | 使用小型 MJCF 测试模型或 hand-crafted `mjModel`，验证读写正确性 |
| `SensorBridge` 单元测试 | ROS 消息 | mock ROS publisher，验证消息格式、字段填充、启停控制 |
| `HardwareManager` 集成测试 | 设备注册与调度 | 验证多设备注册、批量读取、错误传播 |
| 端到端集成测试 | 完整链路 | 加载真实 MJCF 模型，单步仿真后检查 joint state 一致性 |

建议优先恢复 `config.cpp` 和设备层单元测试——这两者覆盖了最容易出 bug 的解析与数据映射逻辑。

### 7.7 架构改进优先级

综合以上分析，按投入产出比排序：

| 优先级 | 改进项 | 理由 |
|--------|--------|------|
| P0 | 恢复测试基础设施 | 避免回归，建立重构信心 |
| P1（已完成） | 消除 `Simulation*` 中间类型 | `MuJoCoSimulation` 已直接使用设备层原生类型 |
| P1（已完成） | 提取 `robocasa` + 新建 `robot_mujoco` 包 | 场景生成已从 `mujoco_simulation` 解耦 |
| P1（已完成） | 合并重复枚举 (`SimulationJointCommandMode` / `CommandInterfaceType`) | 关节控制模式已统一到 `CommandInterfaceType` |
| P1（已完成） | 收敛 Viewer-Only Camera 路径 | 已删除 `render_hardware_manager`，camera 回到单一 `HardwareManager` |
| P2（已完成） | 取消 `Impl` 中的配置 maps 副本 | 设备注册状态已统一收口到 `HardwareManager` |
| P2（已完成） | 统一设备构造函数为 `MjContext` 注入 | 设备构造点已统一改为单一上下文对象注入 |
| P2 | 纯 headless camera 支持 | 当前阶段暂不考虑，需要独立渲染上下文能力 |
| P2 | 文档化/修复线程模型风险 | 防止生产环境并发 bug |
| P2（已完成） | 审视 `Robot` 类的去留 | `MuJoCoHardwareInterface` 已直接持有 `MuJoCoSimulation` |
| P3 | 统一错误处理：`bool + std::string&` | 零成本消除三种模式混用，移除 `=nullptr` + 消灭 `last_error()` |
| P3 | 拆分 `SensorBridge` | 提升可测试性和可扩展性 |
| P3 | 补齐 MobileBase 公开 API / 标记为未开放 | 消除已实现但不可用的"半成品"状态 |
| P3 | ✅ `config.cpp` 传感器类型缺失时报错 | 已完成: `is_sensor_type()` 增加 `error_message` 参数 |
| P3 | ✅ Pimpl 消除 + 物理循环时间补偿 | 已完成: `Impl` 移除, `physics_loop()` 每轮从 `now()` 重算 |

### 7.8 robocasa 模块归属与 `robot_mujoco` 包设计

本节回答两个相互关联的架构问题：(1) `robocasa` 场景生成模块放在 `mujoco_simulation` 中是否合适？(2) 是否应该新建一个 `robot_mujoco` 包来提供项目级的统一入口？当前代码已经按本节建议完成了提取，以下内容保留为设计动机说明。

#### 问题一：robocasa 当前归属是否合适？

**结论：不合适。** 从职责边界、依赖方向和部署粒度三个维度来看，`robocasa` 放在 `mujoco_simulation` 中是一个架构错位。

**职责对比：**

```text
mujoco_simulation 的职责               robocasa 的职责
─────────────────────────────────    ─────────────────────────────
C++ 仿真引擎封装（mjModel/mjData）    调用 RoboCasa/robosuite API
设备抽象（Joint/IMU/Camera/Lidar）    MJCF XML 变换与适配
物理线程管理、viewer、渲染管线          替换 placeholder robot
对上层暴露稳定 C++ API                生成 MuJoCo MjModel 对象
─────────────────────────────────    ─────────────────────────────
运行时基础设施                         离线场景预处理工具
```

robocasa **不调用** `mujoco_simulation` 的任何 C++ 代码，它只依赖 `mujoco` Python 包（`pip install mujoco`）。两者只是恰好放在同一个目录下。

**具体问题：**

1. **依赖声明不完整**：`mujoco_simulation/package.xml` 声明的依赖是 `hardware_interface`、`libglfw3-dev`、`mujoco_vendor`、`rclcpp`、`sensor_msgs`。robocasa 实际需要的 `robosuite`、`robocasa`（第三方 vendored）、`numpy`、`scipy`、`PyYAML` **全部未声明**。依赖正确性由 README 中的手动安装步骤口头保证，不由包管理系统强制执行。

2. **部署耦合**：robocasa 通过 `ament_python_install_package` 绑定在 `mujoco_simulation/CMakeLists.txt` 中构建。想用 robocasa 离线生成场景的用户必须完整构建 mujoco_simulation（GLFW、OpenGL、hardware_interface 等全部 C++ 依赖），而实际需求只是一个纯 Python 脚本。

3. **测试隔离差**：robocasa 的测试需要 RoboCasa 全套资产（约 10 GB），mujoco_simulation 的 C++ 单元测试不需要。混在同一包中意味着 CI 要么全局安装 10 GB 资产，要么跳过所有 Python 测试。

4. **语言边界模糊**：`mujoco_simulation` 是 C++ 项目，核心产物是 `.so` 动态库；`robocasa` 是纯 Python，不需要编译。`CMakeLists.txt` 中 `ament_python_install_package` 只是将 Python 文件复制到安装目录。

#### 问题二：是否需要新建 `robot_mujoco` 包？

**结论：应该。** 它填补当前仓库中缺失的"工作区统一入口"层。

**当前架构的空缺：**

```text
┌─ (空缺) ────────────────────────────────────────────────┐
│  谁来提供统一的 Python API？                               │
│  谁来管理跨包配置（机器人模型路径、场景参数、仿真参数）？      │
│  谁来编排"生成场景 → 加载模型 → 启动仿真"这条完整链路？      │
└──────────────────────────────────────────────────────────┘

┌─ mujoco_hardware ──────────────────────────────────────┐
│  ROS 2 C++ 包。依赖 mujoco_simulation                   │
│  职责: ros2_control 插件，状态/命令映射，传感器消息发布     │
└──────────────────────────────────────────────────────────┘

┌─ mujoco_simulation ────────────────────────────────────┐
│  ROS 2 C++ 包。依赖 MuJoCo C library                    │
│  职责: 仿真运行时，设备抽象，物理线程，viewer              │
└──────────────────────────────────────────────────────────┘

┌─ robocasa (重构前位于 mujoco_simulation 内部) ───────────┐
│  纯 Python。依赖 robosuite, robocasa, mujoco (Python)    │
│  职责: 场景生成与 MJCF 适配                               │
└──────────────────────────────────────────────────────────┘
```

`robot_mujoco` 的价值是填补顶层空缺，提供工作区级的统一入口和编排能力。

**`robot_mujoco` 的职责边界（已实现）：**

```text
robot_mujoco/                           # ROS 2 ament_cmake 包（C++ + Python）
│
├── CMakeLists.txt                      # ament_cmake + ament_cmake_python
├── package.xml                         # build_type: ament_cmake
│
├── include/
│   └── robot_mujoco/
│       └── robot_mujoco.hpp            # C++ 工作区统一头文件（聚拢关键类型）
│
├── config/
│   └── robocasa_scene.example.yaml     # 场景生成示例配置
│
├── __init__.py                         # Python: import robot_mujoco
└── robocasa/                           # Python: import robot_mujoco.robocasa
    ├── __init__.py                     # 公开 API（SceneGenerator 延迟导入）
    ├── scene_config.py                 # YAML → SceneConfig
    ├── scene_generator.py              # RoboCasa API → GeneratedScene
    ├── mjcf_adapter.py                 # XML 变换管线
    ├── scene_cli.py                    # 命令行入口
    ├── scene_data.py                   # 数据模型
    └── exceptions.py                   # 异常类型层次
```

**robot_mujoco 做什么：**
- 提供统一的 Python API：`robot_mujoco.generate_scene(...)` → `robot_mujoco.load_model(...)` 等
- 管理跨包共享的配置（机器人模型路径、场景参数、仿真默认值）
- 编排 robocasa 场景生成 → mujoco_simulation 模型加载的完整链路
- 作为工作区的 Python 入口点和命令行工具

**robot_mujoco 不做什么：**
- 不重新实现仿真运行时（那是 `mujoco_simulation` C++ 库的职责）
- 不重新实现 ROS 2 硬件插件（那是 `mujoco_hardware` 的职责）
- 不变成"god package"——只做编排，不吞并下层

**关键设计决策：**

| 决策 | 实施方案 | 理由 |
|------|---------|------|
| `robot_mujoco` 是否依赖 `mujoco_hardware`？ | 是 | 工作区统一入口聚合所有下层接口：`mujoco_simulation`（仿真运行时）+ `mujoco_hardware`（ros2_control 插件） |
| 包类型 | `ament_cmake`（C++ INTERFACE 库 + `ament_cmake_python`） | 提供 C++ 工作区统一头文件 `<robot_mujoco/robot_mujoco.hpp>` + Python robocasa 模块 |
| C++ 产物 | INTERFACE 库 `robot_mujoco`，依赖 `mujoco_simulation` | 头文件聚拢 `mujoco_simulation` 关键类型；应用代码只需一个 `find_package(robot_mujoco)` |
| Python 安装方式 | `ament_python_install_package()` | Python 源码与 C++ 头文件共存于同一包根目录 |
| robocasa import 路径 | `robot_mujoco.robocasa` | 替代重构前的 `mujoco_simulation.robocasa` |
| `mujoco_simulation` 包变更 | 移除 `ament_cmake_python` 依赖 + `config/` 安装 + `robocasa/` 子模块 | 彻底解除 robocasa 与 C++ 仿真运行时的构建耦合 |

**改造后的三层架构：**

```text
                    ┌───────────────────────────────────┐
                    │  robot_mujoco (C++ + Python)       │
                    │  工作区统一入口 + 场景生成           │
                    └────────┬──────────────┬───────────┘
                             │ 依赖          │ 依赖
                    ┌────────▼──────┐ ┌──────▼──────────┐
                    │ mujoco_sim    │ │ mujoco_hardware  │
                    │ (C++)         │ │ (C++)            │
                    │ 仿真运行时     │ │ ros2_control插件  │
                    └───────────────┘ └────────┬─────────┘
                                               │ 依赖
                                    ┌──────────▼─────────┐
                                    │ mujoco_sim (C++)    │
                                    └────────────────────┘
```

`robot_mujoco` 直接依赖 `mujoco_simulation` 和 `mujoco_hardware`。应用代码只需一个 `find_package(robot_mujoco)`。

#### 实施路径

**第一阶段：提取 robocasa + 建立 ROS 2 包结构 ✅ 已完成**

1. ✅ 将 `mujoco_simulation/robocasa/` 移动到 `robot_mujoco/robot_mujoco/robocasa/`
2. ✅ import 路径统一为 `robot_mujoco.robocasa`
3. ✅ 创建 `package.xml`（`ament_python` 构建类型）
4. ✅ 创建 `setup.py`（含 `robocasa_generate_scene` 命令行入口）
5. ✅ 创建 `resource/robot_mujoco` 标记文件
6. ✅ 迁移 `config/robocasa_scene.example.yaml` 到 `robot_mujoco/config/`
7. ✅ 完善 `robot_mujoco/__init__.py`（re-export `SceneConfig` 等关键符号）
8. ✅ 清理 `mujoco_simulation/CMakeLists.txt`（移除 `ament_cmake_python` + `config/` 安装）
9. ✅ 清理 `mujoco_simulation/__init__.py` robocasa 子模块引用

**第二阶段：添加编排层（中等风险）**

1. 在 `robot_mujoco/` 中添加 `config/` 模块，提供统一的配置加载
2. 添加 `launch/` 模块，提供一键启动入口
3. 开发 workspace 级别 CLI

**第三阶段（未来）：**
- 考虑是否将 `ros2_control` 的 launch 文件集中到 `robot_mujoco/launch/`
- 考虑是否添加 `robot_mujoco` 的 ROS 2 package wrapper（若需要作为 ament 包发布）

### 7.9 `MuJoCoSimulation` 公开 API 冗余——`Simulation*` 类型的消除

这是 [7.2 节](#72-数据结构分层与重复) 中"数据结构三层重复"问题的具体实例化。当前代码已经完成该项 P1 重构，以下内容保留为问题来源与改造动机说明。

#### 历史状态：每个 register/read/write 方法都是字段搬运

以 joint 为例，`MuJoCoSimulation::register_joint()` 的完整流程：

```cpp
// mujoco_simulation.cpp — 这是一个完整的 register 方法
bool MuJoCoSimulation::register_joint(const SimulationJointConfiguration& configuration,
                                      std::string* error_message) {
  std::lock_guard<std::mutex> lock(mutex_);
  // ... 空值检查 ...

  JointConfig data;                                          // 创建后端类型
  data.name = configuration.name;                          // 逐字段复制
  data.actuator_name = configuration.actuator_name;        // 逐字段复制
  data.command_mode = to_backend_command_mode(              // 枚举转换
      configuration.command_mode);                         // 逐字段复制

  if (!impl_->hardware_manager->register_joint(data)) {    // 调用后端
    // ... 错误传播 ...
  }
  impl_->joints[configuration.name] = configuration;       // 存副本到 Impl map
  // ... 清理 error_message ...
  return true;
}
```

同样的模式在 `read_joint()` / `read_imu()` / `read_camera()` / `read_lidar()` 中反复出现——从后端类型读出后逐字段复制到 `Simulation*State` 返回。这导致旧版 `mujoco_simulation.cpp` 中大量代码只是纯字段搬运，`robot.cpp` 中还有额外的一层搬运。

#### 根源

`mujoco_simulation.hpp` 定义的 `SimulationJointConfiguration` / `SimulationJointState` 等公开类型，与 `hardware/joint.hpp` 定义的 `JointConfig` / `JointState` 等设备层类型，在语义上等价但类型上不同。这是"三层重复"中"仿真公开接口层"的体现。

#### 解决方案

将 `MuJoCoSimulation` 的公开 API 的参数类型从 `Simulation*` 替换为设备层的原生类型：

```text
改造前：
  bool register_joint(const SimulationJointConfiguration&, string*);
  bool read_joint(const string&, SimulationJointState*, string*);

改造后：
  bool register_joint(const JointConfig&, string*);
  bool read_joint(const string&, JointState*, string*);
```

影响范围：
- `mujoco_simulation.hpp`：删除所有 `Simulation*` 结构体定义和对应方法签名
- `mujoco_simulation.cpp`：删除 `to_public_joint_state()` / `to_public_imu_state()` 等匿名命名空间转换函数
- `robot.cpp`：`register_joint(JointData)` 中直接传 `joint.info` 而非逐字段复制到 `SimulationJointConfiguration`
- `mujoco_hardware/data.hpp`：`JointData` 已持有 `JointConfig` / `JointCommand` / `JointState`，无需更改

这项工作现已完成：`MuJoCoSimulation` 公开 API 直接使用设备层类型，`mujoco_hardware` 侧也不再需要额外的中间适配类型或纯转发包装层。

### 7.10 枚举重复与类型体系统一

这一项也已在当前代码中完成。历史上，项目中曾存在语义等价但定义为不同类型的枚举：

| 枚举 | 定义位置 | 值 |
|------|---------|-----|
| `CommandInterfaceType` | `mujoco_simulation/hardware/joint.hpp` | `None`, `Position`, `Velocity`, `Effort` |
| `SimulationJointCommandMode` | `mujoco_simulation/mujoco_simulation.hpp` | `None`, `Position`, `Velocity`, `Effort` |

两者语义完全相同（都表示"用哪种方式控制关节"），但由于定义在不同位置和命名空间中，代码中一度出现了三处转换函数：

| 转换函数 | 位置 | 方向 |
|---------|------|------|
| `to_backend_command_mode()` | `mujoco_simulation.cpp:22` | `SimulationJointCommandMode` → `CommandInterfaceType` |
| `to_simulation_joint_mode()` | `robot.cpp:8` | `CommandInterfaceType` → `SimulationJointCommandMode` |
| `to_joint_control_mode()` | `config.cpp:140` | 字符串 → `CommandInterfaceType` |

**当前结果**：项目已经保留 `CommandInterfaceType` 作为唯一关节控制模式枚举，`SimulationJointCommandMode` 与相关双向转换层已被移除。`config.cpp` 中保留的 `to_joint_control_mode()` 仅负责字符串到 `CommandInterfaceType` 的解析，不再承担类型桥接角色。

### 7.11 设备构造抽象的不一致

这一项已经在当前代码中完成。历史上，`mujoco_simulation/hardware` 下的五种设备构造函数签名并不统一：

| 设备 | 构造函数参数 | 说明 |
|------|-------------|------|
| `Joint` | `(const mjModel*, mjData*)` | 直接持有 MuJoCo 指针 |
| `Imu` | `(const mjModel*, mjData*)` | 直接持有 MuJoCo 指针 |
| `Camera` | `(const mjModel*, mjData*, mjvScene*, mjrContext*)` | 额外需要渲染上下文 |
| `Lidar` | `(const mjModel*, mjData*)` | 直接持有 MuJoCo 指针 |
| `MobileBase` | `(const std::vector<Joint*>&)` | **不持有 MuJoCo 指针，组合已有 Joint** |

MobileBase 的模式（组合而非直接访问 MuJoCo）在架构上更优——调用者不需要传递 MuJoCo 原生指针。但其他四种设备都直接依赖 `mjModel*` / `mjData*`，导致所有设备构造点（`HardwareManager::register_*()`）必须持有并传递 MuJoCo 上下文。

**当前结果**：项目已经引入统一的 `MjContext` 值类型，用来收敛设备构造所需的 MuJoCo 运行时资源：

```cpp
// 遵循 MuJoCo 官方 mj 前缀命名约定（mjModel / mjData / mjvScene / mjrContext）
struct MjContext {
    const mjModel* model;
    mjData* data;
    mjvScene* scene = nullptr;      // optional: 仅 Camera 需要
    mjrContext* render = nullptr;   // optional: 仅 Camera 需要
};
```

当前实现中的统一方式是：

- `Joint / Imu / Lidar / Camera` 构造函数统一改为接收 `const MjContext&`
- `MobileBase` 也改为通过相同的 `MjContext` + traction joints 构造，哪怕当前主要使用的仍是 `Joint*` 组合关系
- `HardwareManager` 内部只维护一份 `MjContext`，在设备创建时统一传递，不再在每个注册函数中手工拆开 `model/data/scene/render`

这项改动的价值主要在于：

- 让设备构造点拥有统一形状，降低后续扩展新上下文字段的改动面
- 让 `Camera` 与其他设备的差异从“构造签名不同”收敛为“使用 `MjContext` 的字段子集不同”
- 为未来若要继续收紧 `MuJoCoHardwareInterface` 职责或重构 `HardwareManager` 内部结构，提供更稳定的设备注入边界

### 7.12 `Impl` 中的配置副本——双重维护风险

这一项已经在当前代码中完成，历史问题与结果如下。

**历史问题：** `MuJoCoSimulation::Impl` 曾维护与 `HardwareManager` 内部设备注册表**语义重叠**的配置 maps：

```cpp
class MuJoCoSimulation::Impl {
 public:
  std::unique_ptr<HardwareManager> hardware_manager;  // 内部有 joints_/imus_ 等 maps
  std::map<std::string, JointConfig> joints;       // ← 仍与 HardwareManager 重复
  std::map<std::string, ImuConfig> imus;           // ← 仍与 HardwareManager 重复
  std::map<std::string, CameraConfig> cameras;     // ← 仍与 HardwareManager 重复
  std::map<std::string, LidarConfig> lidars;       // ← 仍与 HardwareManager 重复
};
```

历史上的 joint mode switch 需要同时更新 `HardwareManager`（unregister + re-register）和 `Impl::joints` map（更新配置）。这是经典的"同一事实的两个记录"问题——两者可能在代码演进中不同步。

**当前结果：**

- `Impl` 中的 `joints / imus / cameras / lidars` 配置 maps 已删除
- `load_model()` 只重建 `hardware_manager`，不再同步清理额外配置副本
- 设备是否已注册、相应配置快照以及 camera 的 pending spec 都由 `HardwareManager` 单点维护
- 当前公开入口已经进一步收口为 `Simulation::reconfigure_component(ComponentConfig{updated_joint})`

因此，这个“双重维护”问题在 `MuJoCoSimulation` 这一层已经被消除；后续若继续优化，重点应放在 `HardwareManager` 内部接口的清晰度，而不是继续在上层保存镜像状态。

### 7.13 `render_hardware_manager` 的消除路径

这一项已经在当前代码中完成，历史问题与结果如下。

**历史问题：** 过去 `Impl` 中有两个 `HardwareManager`：

```cpp
std::unique_ptr<HardwareManager> hardware_manager;        // 管理 joint / imu / lidar
std::unique_ptr<HardwareManager> render_hardware_manager;  // 管理 camera (需要渲染上下文)
```

分离的唯一原因是 `Camera` 的构造函数需要 `mjvScene*` / `mjrContext*`，而其他设备不需要。这曾导致：
- `register_camera()` 有双路径：有 viewer 时立即创建专用 manager 并注册，无 viewer 时仅记录配置
- `read_camera()` 有懒初始化：首次 read 时重建专用 manager 并批量 re-register 所有已记录的 camera

**当前结果：**

- `render_hardware_manager` 已被删除
- `load_model()` 只创建一个 `hardware_manager`
- `start_viewer()` 成功后将 render resources 注入现有 `hardware_manager`
- `register_camera()` / `read_camera()` 与 joint / imu / lidar 一样都通过同一 manager 进入设备层
- camera 的 pending spec 也已迁入 `HardwareManager`，用于承接“先缓存配置、viewer 就绪后补注册”的语义

因此，后续若继续做 camera 演进，重点应转向“是否支持脱离 viewer 的新渲染后端”，而不是回头维护双 manager 架构。

### 7.14 其他细节发现

**7.14.1 `config.cpp` 传感器类型默认识别 ✅ 已修复**
~~`is_sensor_type()` 在 `mujoco_type` 参数缺失时静默回退为 IMU。~~ 已增加 `error_message` 参数：`mujoco_type` 缺失时仅 IMU 向后兼容（返回 true），camera/lidar 显式报错。

**7.14.2 MobileBase 公开 API 缺失 ✅ 已补齐**
~~`HardwareManager` 和 `MobileBase` 类已完整实现，但 `MuJoCoSimulation` 公开 API 未暴露。~~ MobileBase 公开 API 已补齐：`register_mobile_base()` / `write_mobile_base()` / `read_mobile_base()` 已添加到 `MuJoCoSimulation`，`HardwareConfig::mobile_bases` 字段已添加，`config.cpp` 支持 `mobile_base_<N>_*` 参数解析。

**7.14.3 物理循环时间精度 ✅ 已修复**
~~`physics_loop()` 使用 `sleep_until()` 累加 `next_step += period` → 累积漂移。~~ 已改为每轮从 `clock::now()` 重新计算 `wake_time`，消除累积误差，移除 >1s catch-up hack。

**7.14.4 Pimpl 模式 ✅ 已消除**
~~`Impl` 仅包含 `HardwareManager`，编译防火墙价值为零。~~ `Impl` 类已移除，`HardwareManager` 作为 `MuJoCoSimulation` 直接成员 `hardware_manager_`。

**7.14.5 `Joint::effort` 读写通道共享 `qfrc_applied` ✅ 已修复**

~~`effort = qfrc_actuator + qfrc_applied` → Effort 命令写入 `qfrc_applied` → read 回读命令值，造成反馈环。~~
原因分析：MuJoCo 的 `qfrc_applied` 是用户手动写入通道（非物理引擎产生量），不应混入 state 读数。修复：`effort = qfrc_actuator[dof]` 仅报告真实 actuator 力；被动关节无 actuator → effort=0（正确）；Effort 命令由 `command_.effort` 独立追踪。
