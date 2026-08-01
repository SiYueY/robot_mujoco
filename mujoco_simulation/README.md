# mujoco_simulation

`mujoco_simulation`  MuJoCo 仿真运行时内核，其定位不是“完整的 ROS 2 仿真应用”，而是可复用的底层库，负责把 MuJoCo 的 `mjModel` / `mjData`、物理步进、viewer 和各类仿真设备封装成稳定的 C++ 接口，供上层模块复用。

它以固定 1.0 实时因子按 `std::chrono::steady_clock` 推进物理仿真，目标是复现真实机器人控制系统的周期与时序；仿真过载时记录 deadline miss 并重新对齐，不高速补跑历史物理步。

## 参考项目

`mujoco_simulation` 模块定位和实现思路参考以下项目：

- `mujoco_ros2_control`
  - https://github.com/ros-controls/mujoco_ros2_control
  - 参考其 “MuJoCo + ROS 2 Control” 的总体接入方向，也就是把 MuJoCo 仿真运行时作为 `ros2_control` 后端来组织
- `mjlab`
  - https://github.com/mujocolab/mjlab
  - 主要参考其对 simulation layer 边界、sensor lifecycle、测试与调试手段的工程化处理，而不是其 manager-based RL framework 本身

当前 `mujoco_simulation` 既不是简单复刻 `mujoco_ros2_control`，也不是直接照搬 `mjlab`，而是面向当前工作区需求，对 MuJoCo 运行时、viewer 和设备抽象做的一层本地化收敛。

## mujoco_simulation 模块解决什么问题

如果直接使用 MuJoCo 原生 C API，上层代码通常需要自己处理：

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
  -> SimulationScheduler
  -> CommandBus / StateBuffer
  -> CameraRenderer
  -> SimulationViewer
    -> mjModel / mjData
```

上层只需要面向 `Simulation` 调用，而不需要到处直接操作 MuJoCo 原生数组。

当前线程模型收敛为：

- scheduler worker thread
  - 唯一连续推进 `mjModel / mjData` 的线程
- camera render worker thread
  - 只读取物理线程复制的私有 `mjData`，执行离屏渲染和图像转换
- viewer render thread 与 viewer sync worker
  - 使用 viewer 私有的 `mjModel` / `mjData` 副本；物理线程仅提交 latest-only 状态副本
- external caller threads
  - 只负责写命令、读 buffer、提交 lifecycle/reset 请求

## 职责边界

`mujoco_simulation` 模块负责：

- 加载 MuJoCo MJCF/XML 模型
- 管理物理线程、暂停、重置、步进
- 按需启动 viewer
- 注册并读写仿真设备
- 提供对 MuJoCo 命名对象的查询能力

而不负责：

- `ros2_control` 插件导出
- URDF / `HardwareInfo` 参数解析
- ROS topic 发布
- controller 管理与 launch 编排

职责关系可以简单理解为：`mujoco_simulation` 是仿真运行时和设备抽象层。

如果你现在在判断“这个模块该放什么代码”，一个简单原则是：

- 只要逻辑本质上是在操作 MuJoCo 仿真运行时本身，就优先放这里
- 只要逻辑本质上是在适配 ROS 2 接口，就不应该放这里

## 核心入口

主入口类是 [`Simulation`](./include/mujoco_simulation/simulation.hpp)。

配置文件加载入口是 [`SimulationConfigParser`](./include/mujoco_simulation/config/simulation_config.hpp)。

`Simulation` 对外暴露的能力主要有：

- 初始化仿真
  - `bool initialize(const SimulationConfig&)`
  - `bool shutdown()`
- 控制运行
  - `bool start()`
  - `bool stop()`
  - `bool pause()`
  - `bool resume()`
  - `bool reset()`
  - `bool reset(std::string keyframe_name)`
- 组件配置与设备访问
  - `SimulationConfig.components`
    - 初始化时传入的组件配置入口
  - `bool write_command(std::string, const JointCommand&)`
  - `bool write_command(std::string, const MobileBaseCommand&)`
  - `bool write_command(JointId, const JointCommand&)`
  - `bool write_command(MobileBaseId, const MobileBaseCommand&)`
  - `template <typename Command> bool write_commands(const CommandBatch<Command>&)`
  - `bool read_state(std::shared_ptr<const RobotState>&)`
  - `bool read_state(RobotState&)`
  - `bool read_state(std::string, JointState&)`
  - `bool read_state(std::string, ImuState&)`
  - `bool read_state(std::string, CameraState&)`
  - `bool read_state(std::string, LidarState&)`
  - `bool read_state(std::string, MobileBaseState&)`
  - 按 `JointId`、`ImuId`、`CameraId`、`LidarId`、`MobileBaseId` 的单组件读取重载
  - 按组件类型读取完整稀疏状态数组的重载
- 查询状态
  - `bool step(std::size_t count = 1)`
  - `uint64_t step_count() const`
  - `double time() const`
  - `SimulationStatus status() const`

内部约束：

- `mjModel / mjData` 保持单写线程原则
- `ComponentManager` 负责组件更新、命令分发和状态汇总
- `CommandBus / StateBuffer` 统一采用 `write(...) / read(...)` 语义
  - 每个成功的 physics step 都向 `StateBuffer` 原子发布一个 `RobotState`；组件状态（包括相机）以不可变共享快照聚合

## 错误返回模型

当前 `Simulation` 的操作接口以 `bool` 表示结果：成功返回 `true`，初始化状态不满足、
无效名称或底层操作失败时返回 `false`。状态读取接口同样使用 `bool + 输出参数`；
读取失败时不提供额外的公开诊断对象。

- `SimulationStatus`
  - 定义在 `simulation_status.hpp`，表达生命周期状态：`Uninitialized`、`Stopped`、
    `Running`、`Paused`、`Stopping`、`Error`
  - 通过 `Simulation::status()` 查询；viewer 或 scheduler 任务失败时可报告 `Error`

`SimulationConfigParser::load_file(...)` 支持可选的 `ConfigError*` 诊断参数：

- 返回 `bool`
- 成功时写入 `SimulationConfig`
- 失败时返回 `false`；传入 `ConfigError` 时可取得首个错误的 XML 行号、元素、属性和说明
- Parser 负责 XML 语法、类型提取与行号诊断；
  `SimulationConfigValidator` 负责 XML 与直接 C++ 配置共用的语义约束

## 运行配置

`SimulationConfig` 定义在 `config/simulation_config.hpp`，包含：

- `model`
  - `model_path` 和 `initial_keyframe`
  - `initial_keyframe` 非空时，初始化会按名称复位到对应 MuJoCo keyframe；找不到
    keyframe 会导致初始化失败
- `scheduler`
  - `physics_period` 同时定义 MuJoCo timestep 与墙钟物理调度周期
  - `viewer_period` 定义 Viewer latest-only 同步请求周期
- `components`
  - `JointInfo`、`ImuInfo`、`CameraConfig`、`LidarInfo` 与 `MobileBaseInfo` 的变体列表
  - 每个组件必须提供类型内唯一的显式 ID；ID 直接作为状态、命令和组件 vector 的下标
  - 空洞为保留槽位，默认允许范围是 `0..256`；`max_component_id` 可配置
  - Camera 的宽、高必须在 `1..8192`；单个 Camera 启用的 RGB8（3 B/像素）和 depth float（4 B/像素）输出合计不得超过 256 MiB
  - 所有组件以 `period`（秒）配置采样周期；`period="0"` 表示每个 physics step 更新。
    正周期必须是 `physics.period` 的整数倍，且不得短于该物理周期。
    旧的 `update_rate`（Hz）属性不受支持。
- `camera_renderer`
  - 离屏渲染资源配置
- `viewer_startup_timeout`
  - viewer 启动等待超时

此外，`SimulationConfigParser::load_file(const std::string&, SimulationConfig&, ConfigError*)`
提供 `robot_mujoco.xml -> SimulationConfig` 的解析路径：

- 解析 `<mujoco><mjcf>` 及 `<robot>` 下的 `joint`、`imu`、`camera`、`lidar`、`mobile_base`。
- 所有组件 XML 元素都必须包含 `id` 与 `name` 属性；根元素可设置
  `max_component_id`。
- `<mjcf>` 相对路径相对 `robot_mujoco.xml` 所在目录解析
- `initial_keyframe` 仍通过 C++ `SimulationConfig` 配置。
- XML 必须提供：
  ```xml
  <simulation>
    <physics period="0.001"/>
    <viewer period="0.0166666667"/>
  </simulation>
  ```
  `physics.period` 会覆盖 MJCF 中的 timestep，固定实时因子为 1.0。
  组件可选使用 `period="秒"` 属性；Joint、IMU、MobileBase 未配置时每物理步更新，
  Camera 与 Lidar 默认分别为 `1 / 30` 秒和 `1 / 10` 秒。

`Simulation` 默认在初始化时启动 viewer，并以独立频率同步显示；将
`SimulationConfig::viewer_enabled` 设为 `false`，或使用 XML
`<viewer period="..." enabled="false"/>`，可用于 headless 部署或测试。Camera 不依赖 viewer 的渲染
资源。`stop()` 会销毁当前 viewer；后续同一 `Simulation` 实例再次 `start()` 时会自动重建
viewer。

`reset()` 复位到默认 MuJoCo 数据状态；`reset(std::string keyframe_name)` 复位到指定
keyframe。两者均执行完整重置事务：运行态 scheduler 会先停止，随后复位 MuJoCo
runtime、清空旧命令、复位并立即更新组件采样、重置 step/sequence，并发布新的状态
快照。Viewer 模式会异步提交 reset 后的显示状态；MuJoCo runtime 或组件失败会使
reset 返回 `false` 并进入错误状态，Viewer 故障仅禁用可视化。成功后恢复调用前的
Running 状态；Paused 和 Stopped 状态保持不变。Error 状态可通过 `reset()` 恢复为
Stopped，前提是完整重置事务成功。

`stop()` 停止 scheduler、Camera worker 与 Viewer，并清空命令；最后发布的
`RobotState` 保留，仍可通过 `read_state(...)` 查询。只有 `shutdown()` 会清空状态
快照并释放 runtime 资源。

生命周期的可读状态契约如下：

| 操作 | 成功后的状态 | 已发布 `RobotState` |
| --- | --- | --- |
| `initialize()` | `Stopped` | 发布初始快照 |
| `start()` / `resume()` | `Running` | 保持可读，并持续更新 |
| `pause()` | `Paused` | 保持可读；返回时当前 physics task 已结束 |
| `stop()` | `Stopped` | 保留最后快照 |
| reset 本体失败 | `Error` | 保留最后成功快照以供诊断 |
| reset 后恢复 scheduler 失败 | `Error` | 保留新发布的 reset 快照 |
| `shutdown()` | `Uninitialized` | 清空 |

## 设备层能力

设备对象现在统一由 [`ComponentManager`](./include/mujoco_simulation/component/component_manager.hpp) 管理；`CameraRenderer` 是 Camera 组件共享的图像生成资源，当前支持以下几类：

| 设备 | 作用 | 当前实现特点 |
| --- | --- | --- |
| `Joint` | 单关节状态/命令读写 | 主要面向 1-DoF joint，支持 position / velocity / effort 命令语义 |
| `Imu` | 组合多个 MuJoCo sensor 输出 IMU 状态 | 只读，依赖 `framequat` / `gyro` / `accelerometer` |
| `Camera` | 从渲染管线读取 RGB / depth 图像 | latest-only 异步离屏渲染，资源独立于 Viewer |
| `Lidar` | 由 `rangefinder` 传感器阵列拼装 `LaserScan` | 依赖 `<prefix>-<index>` 命名约定 |
| `MobileBase` | 底盘运动学封装 | 当前支持四轮 mecanum |

### 设备层的几个关键边界

- `Camera` 当前支持独立的离屏渲染
  - camera 读取不依赖 viewer 显示
  - physics 线程只复制相机快照；OpenGL 渲染和像素读取由专属 worker 执行
  - `RobotState` 可保留上一有效图像，Camera sequence 允许相对 physics sequence 跳跃
- `Joint` 当前是单关节、单标量接口
  - 不适合直接承载 `ball` / `free` 这类多自由度 joint
- `Lidar` 当前输出的是 `LaserScan`
  - 还不提供点云抽象
- `MobileBase` 是多个 traction joint 的组合包装
  - 它不是一个特殊 joint，而是更上层的运动学设备

更完整的当前实现说明见：

- [`docs/architecture.md`](./docs/architecture.md)

## Viewer 的定位

viewer 相关代码位于 [`src/viewer`](./src/viewer)。

这里的 viewer 是“被动渲染前端”：

- `Simulation` 持有真正的 `mjModel` / `mjData`；Viewer 使用私有副本
- 物理步进仍然由 `Simulation` 驱动
- `SimulationViewer` 只负责渲染和异步状态同步；GUI 卡顿、关闭或同步失败不影响物理仿真

Camera 渲染现在通过独立的 `CameraRenderer` 完成，不再复用 viewer 的渲染资源。

`CameraRenderer::submit()` 返回可等待的 `CameraRenderTicket`。`wait(ticket,
result)` 只读取该 ticket 的结果：被 newer submit 覆盖的 pending ticket 会完成为
`superseded`，而不是误读新批次；完成结果保留数量由
`CameraRendererConfig::completed_ticket_history` 明确限定，超出历史窗口的 ticket
会返回失败。初始化和 reset 等待的 ticket 要求全部 Camera 成功；运行期仍采用
latest-only 降级策略，失败 Camera 保留上一帧而成功 Camera 正常发布。

对 `Simulation` 而言，viewer 是可恢复的运行时资源：

- `initialize(...)` 会创建 viewer
- `stop()` 会停止 scheduler 并销毁当前 viewer
- 后续再次 `start()` 时会自动重新创建 viewer

## 目录结构

```text
mujoco_simulation/
├── include/mujoco_simulation/
│   ├── simulation.hpp               # 对外主入口
│   ├── simulation_status.hpp         # 生命周期状态
│   ├── buffer/                       # CommandBus / StateBuffer
│   ├── common/                       # 数学和通用辅助类型
│   ├── component/                    # 组件基类、管理器及设备类型
│   ├── config/                       # SimulationConfig 与 XML 解析器
│   ├── data/                         # CommandBatch / CommandSnapshot / RobotState
│   ├── mujoco/                       # mjContext 与 CameraRenderer
│   ├── runtime/                      # SimulationRuntime / SimulationScheduler
│   └── viewer/                       # SimulationViewer 对外接口
├── src/
│   ├── buffer/                       # 缓冲实现
│   ├── component/                    # 组件管理与设备实现
│   ├── config/                       # XML 配置解析
│   ├── mujoco/                       # 相机渲染实现
│   ├── runtime/                      # 运行时与调度器实现
│   ├── simulation.cpp                # 仿真主实现
│   └── viewer/                       # viewer、lodepng 与 simulate 集成
├── third_party/                      # 裁剪后的外部源码
│   ├── tinyxml2/
│   └── easyloggingpp/
└── docs/                             # 模块文档
```

## 构建与依赖

这是一个独立的 Linux C++17 库，当前主要依赖：

- `glfw3`
- `OpenGL`
- `mujoco`

核心库当前不再直接依赖 `hardware_interface`、`rclcpp` 或 `sensor_msgs`；这些 ROS 2 相关依赖保留在上层 `robot_mujoco_ros2` 适配层。

构建时不调用 `find_package(mujoco)`；MuJoCo 的路径解析规则为：

- 如果设置了环境变量 `MUJOCO_ROOT`，优先使用它
- 否则默认使用 `/opt/mujoco-3.9.0`

例如：

```bash
export MUJOCO_ROOT=/opt/mujoco-3.9.0
cmake -S mujoco_simulation -B build/mujoco_simulation \\
  -DMUJOCO_SIMULATION_BUILD_SHARED=ON
cmake --build build/mujoco_simulation
cmake --install build/mujoco_simulation --prefix /desired/install/prefix
```

可选构建开关为：

- `MUJOCO_SIMULATION_BUILD_SHARED`：构建 shared（默认）或 static 库；
- `MUJOCO_SIMULATION_BUILD_TESTS`：包含 `tests/`；
- `MUJOCO_SIMULATION_BUILD_EXAMPLES`：包含 `examples/`；
- `MUJOCO_SIMULATION_INSTALL`：生成安装规则（默认开启）；
- `MUJOCO_SIMULATION_ENABLE_ASAN`：为库启用 AddressSanitizer。
- `MUJOCO_SIMULATION_ENABLE_TSAN`：为库启用 ThreadSanitizer；不能与 ASan 同时启用。

TSAN 建议使用独立构建目录，并只运行纯 CPU 并发测试：

```bash
cmake -S mujoco_simulation -B build/mujoco_simulation-tsan \
  -DMUJOCO_SIMULATION_BUILD_TESTS=ON \
  -DMUJOCO_SIMULATION_ENABLE_TSAN=ON
cmake --build build/mujoco_simulation-tsan
ctest --test-dir build/mujoco_simulation-tsan -L tsan --output-on-failure
```

GLFW/EGL/MuJoCo 图形后端依赖通常未用 TSAN 插桩，因此 CameraRenderer 和 GUI Viewer
测试不属于 TSAN 门禁。

某些容器或受限宿主无法提供 TSAN 所需的地址空间布局，并会在启动时报告
`unexpected memory mapping`。这类 runner 应继续执行 TSAN 编译，但显式配置：

```bash
cmake -S mujoco_simulation -B build/mujoco_simulation-tsan \
  -DMUJOCO_SIMULATION_ENABLE_TSAN=ON \
  -DMUJOCO_SIMULATION_TSAN_RUNTIME_TESTS=OFF
```

这样仅禁用带 `tsan` 标签的运行测试；支持 TSAN 的 CI runner 保持默认 `ON` 并执行该标签。

安装后，下游工程只需：

```cmake
find_package(mujoco_simulation REQUIRED)
target_link_libraries(app PRIVATE mujoco_simulation::mujoco_simulation)
```

下游同样需要通过 `MUJOCO_ROOT`（或默认 `/opt/mujoco-3.9.0`）提供 MuJoCo；OpenGL、EGL、
GLFW 和线程依赖会由包配置自动发现。

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

当前 ROS 侧控制服务与传感器发布已经由 `robot_mujoco_ros2::SimulationRosBridge` 提供，统一由 `robot_mujoco_ros2` 包内部接线到 `bool` 风格控制回调。当前控制服务包括：

- `/start`
- `/stop`
- `/pause`
- `/resume`
- `/load_keyframe`
- `/reset`

这些服务属于 `robot_mujoco_ros2` adapter 层，不属于 `mujoco_simulation` 本体 API；`mujoco_simulation` 只提供底层运行时控制能力。

其中：

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

- camera 已支持独立离屏渲染
  - 但当前仍以 RGB / depth 采样为主，还没有扩展到 segmentation / object id 等更复杂渲染输出
- joint 抽象目前偏向 1-DoF 控制接口
- lidar 依赖传感器命名规则
- mobile base 当前只覆盖四轮 mecanum
- 这个包本身是底层库，不是开箱即用的完整仿真应用

如果你的目标是“启动一个 ROS 2 机器人仿真”，通常不应直接从这里起步，而应从 `robot_mujoco_ros2` 或 `robot_mujoco/launch` 看整体接入链路。
