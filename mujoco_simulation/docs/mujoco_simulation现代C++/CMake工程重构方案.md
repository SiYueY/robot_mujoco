# `mujoco_simulation` 现代 C++ / CMake 工程完整重构方案

## 1. 文档目的

本文档用于指导 `mujoco_simulation` 模块完成目录结构、公共 API、内部实现、Camera 批量渲染、符号可见性、CMake targets、安装包、静态链接和测试体系的系统性重构。

本次重构的总体目标是：

> 工程内部保持 Runtime、Render、Viewer 分层和清晰依赖；对外只提供一个完整、固定能力、易于使用的 `mujoco_simulation::mujoco_simulation` 库。

最终库始终包含：

* MuJoCo 物理仿真；
* Component 管理；
* CommandBuffer；
* StateBuffer；
* XML 配置解析；
* Camera 离屏渲染；
* 交互式 Viewer；
* 调度、线程和生命周期管理。

运行时可以不启动 Viewer，也可以不配置 Camera，但构建产物始终包含完整图形能力。

---

# 2. 需要解决的问题

本次重构重点解决以下问题：

1. 公共头文件与内部实现头文件混杂；
2. `Simulation` 公共头直接暴露 Buffer、Runtime、Renderer 和 Viewer；
3. 公共模板命令接口导致内部命令容器无法隐藏；
4. Parser 和 Validator 被外部代码直接依赖；
5. 内部类被错误地作为导出 API；
6. Runtime、Renderer、Viewer 和第三方源码混在一个 target 中；
7. `CameraComponent` 直接依赖 `CameraRenderer`，分层关系不清晰；
8. Camera 异步 worker 可能访问主 `mjData`，并发契约不明确；
9. 单 Camera 请求接口会破坏现有多 Camera batch 语义；
10. Camera batch ticket、wait、latest-only 和 supersede 行为没有形成明确接口合同；
11. `CommandBus` 命名与实际缓冲区职责不一致；
12. 错误结果类型过多，不符合当前项目的错误处理方式；
13. 内部 STATIC target 与最终静态库聚合存在 archive 依赖问题；
14. 内部 target 和 `_objects` target 出现重复模型；
15. 第三方 STATIC target 不安装会导致 static consumer 链接不完整；
16. Package Components 和多套安装能力增加了不必要复杂度；
17. CMake 最低版本为 3.16，却使用了不兼容的新变量或机制；
18. 内部测试需求迫使实现头进入公共安装范围；
19. PImpl 的 ABI 收益范围表述过强；
20. 实施阶段混合了文件迁移、API 变更和架构变更。

---

# 3. 最终产品边界

## 3.1 对外只提供一个 target

最终安装包只导出：

```text
mujoco_simulation::mujoco_simulation
```

consumer 使用方式：

```cmake
find_package(mujoco_simulation REQUIRED)

target_link_libraries(
  application
  PRIVATE
    mujoco_simulation::mujoco_simulation
)
```

不再对外提供：

```text
mujoco_simulation::runtime
mujoco_simulation::render
mujoco_simulation::viewer
mujoco_simulation::simulation
```

不再提供：

```cmake
find_package(
  mujoco_simulation
  COMPONENTS runtime render viewer
)
```

Runtime、Render、Viewer 只作为工程内部 OBJECT targets。

## 3.2 始终构建完整能力

所有构建始终包含：

```text
Runtime
Render
Viewer
```

因此 CMake 配置阶段始终查找：

```text
MuJoCo
Threads
OpenGL
EGL
GLFW
dl
```

删除以下选项：

```text
MUJOCO_SIMULATION_BUILD_RENDER
MUJOCO_SIMULATION_BUILD_VIEWER
```

不再提供：

* headless-only 安装包；
* render-only 安装包；
* viewer-only 安装包；
* Null Render Service；
* Null Viewer Service；
* Render/Viewer 能力工厂；
* 多套 Package Config；
* 多套 export sets；
* 按能力划分的 consumer 测试矩阵。

## 3.3 Headless 的含义

本项目中的 headless 只表示：

> 运行时不启动交互式 Viewer。

当前公共配置字段统一使用：

```cpp
config.viewer_enabled = false;
```

不得使用未定义的：

```cpp
config.viewer.enabled = false;
```

当：

```cpp
config.viewer_enabled == false
```

时：

* 不初始化交互式 Viewer；
* 不创建 Viewer 窗口；
* 不启动 Viewer UI 线程；
* 不执行 Viewer 同步；
* 物理仿真正常运行；
* Camera 离屏渲染仍可使用；
* 库仍链接 OpenGL、EGL 和 GLFW。

如果没有配置 Camera：

* 不生成 Camera batch；
* 不提交离屏渲染任务；
* Camera worker 可以不启动，或启动后保持空闲；
* 不影响库的构建和链接能力。

---

# 4. 重构范围

本次重构允许受控的公共 API 变更。

明确变更包括：

* 删除 `config_result.hpp`；
* 删除 `simulation_result.hpp`；
* 删除 `ConfigLoadResult`；
* 删除 `SimulationResult`；
* 删除 `Simulation::last_result()`；
* 删除公共配置加载函数；
* XML Parser 和 Validator 完全内部化；
* `CommandBus` 重命名为 `CommandBuffer`；
* 删除公共模板 `write_command<T>()`；
* 使用有限的强类型命令 overload；
* `Simulation` 使用 PImpl；
* Camera 渲染改为内部 batch-oriented `CameraRenderService`；
* Runtime、Render、Viewer 改为内部 OBJECT targets；
* 内置第三方源码改为 OBJECT targets；
* 最终只安装一个 STATIC 或 SHARED library。

原则上不改变：

* XML 配置格式；
* `SimulationConfig` 现有字段语义；
* 稀疏 Component ID；
* 多命令通道；
* 命令验证；
* 命令 sequence；
* 命令快照读取；
* 状态快照发布；
* 物理调度周期；
* 多 Camera 同 tick 状态一致性；
* Camera batch ticket；
* Camera batch wait；
* Camera batch latest-only；
* Camera batch supersede；
* Viewer latest-only；
* initialize、start、stop、pause、resume、reset、step、shutdown 的主要行为。

当前阶段不新增 `component_context.hpp`，也不系统修改 Component 对 `mjModel*`、`mjData*` 的整体使用方式。

---

# 5. 错误处理模型

## 5.1 对外统一使用 `bool`

以下接口失败时统一：

1. 记录错误日志；
2. 根据生命周期需要进入 `SimulationStatus::Error`；
3. 返回 `false`。

包括：

```cpp
bool initialize(const std::string& config_path);
bool initialize(const SimulationConfig& config);

bool start();
bool stop();
bool pause();
bool resume();

bool reset();
bool reset(const std::string& keyframe);

bool step(std::size_t count = 1);
bool shutdown();

bool write_command(...);
bool write_commands(...);

bool read_state(...);
```

不向调用方提供结构化错误结果。

## 5.2 `SimulationStatus::Error`

`SimulationStatus::Error` 仅表示：

> 当前 Simulation 生命周期已经进入错误状态。

它不提供：

* 错误码；
* 错误消息；
* 错误链；
* 上次失败操作；
* 可恢复建议。

具体错误原因通过日志输出。

## 5.3 删除的公共文件与类型

删除：

```text
include/mujoco_simulation/config_result.hpp
include/mujoco_simulation/simulation_result.hpp
```

删除：

```cpp
ConfigLoadResult
SimulationResult
load_simulation_config()
Simulation::last_result()
```

## 5.4 日志内容要求

错误日志至少应包含：

* 操作名称；
* 当前生命周期状态；
* 配置文件路径；
* XML 行号或字段位置；
* Component 名称或 ID；
* MuJoCo 对象名称；
* 底层失败原因。

示例：

```text
Failed to initialize simulation from '/path/simulation.xml':
camera component 'wrist_camera' references unknown MuJoCo camera 'wrist_rgb'.
```

---

# 6. 最终目录结构

```text
mujoco_simulation/
├── include/
│   └── mujoco_simulation/
│       ├── simulation.hpp
│       ├── simulation_config.hpp
│       ├── simulation_status.hpp
│       ├── robot_state.hpp
│       ├── component_id.hpp
│       ├── export.hpp
│       ├── version.hpp
│       │
│       ├── common/
│       │   ├── enum.hpp
│       │   └── math.hpp
│       ├── config/
│       │   ├── config_limits.hpp
│       │   └── simulation_config.hpp
│       ├── data/
│       │   └── robot_state.hpp
│       └── component/
│           ├── component_id.hpp
│           ├── joint.hpp
│           ├── imu.hpp
│           ├── camera.hpp
│           ├── lidar.hpp
│           └── mobile_base.hpp
│
├── src/
│   ├── common/
│   │   ├── logging.hpp
│   │   └── macro.hpp
│   │
│   ├── data/
│   │   └── command_snapshot.hpp
│   │
│   ├── facade/
│   │   └── simulation.cpp
│   │
│   ├── buffer/
│   │   ├── command_buffer.hpp
│   │   ├── command_buffer.cpp
│   │   ├── command_channel.hpp
│   │   ├── state_buffer.hpp
│   │   └── state_buffer.cpp
│   │
│   ├── component/
│   │   ├── component.hpp
│   │   ├── component.cpp
│   │   ├── component_manager.hpp
│   │   ├── component_manager.cpp
│   │   │
│   │   ├── joint/
│   │   │   ├── joint_component.hpp
│   │   │   └── joint_component.cpp
│   │   ├── imu/
│   │   │   ├── imu_component.hpp
│   │   │   └── imu_component.cpp
│   │   ├── camera/
│   │   │   ├── camera_component.hpp
│   │   │   ├── camera_component.cpp
│   │   │   └── camera_render_service.hpp
│   │   ├── lidar/
│   │   │   ├── lidar_component.hpp
│   │   │   └── lidar_component.cpp
│   │   └── mobile_base/
│   │       ├── mobile_base_component.hpp
│   │       └── mobile_base_component.cpp
│   │
│   ├── config/
│   │   ├── simulation_config_parser.hpp
│   │   ├── simulation_config_parser.cpp
│   │   ├── simulation_config_validator.hpp
│   │   ├── simulation_config_validator.cpp
│   │   └── simulation_config_detail.hpp
│   │
│   ├── runtime/
│   │   ├── context.hpp
│   │   ├── type.hpp
│   │   ├── simulation_runtime.hpp
│   │   ├── simulation_runtime.cpp
│   │   ├── simulation_scheduler.hpp
│   │   └── simulation_scheduler.cpp
│   │
│   ├── render/
│   │   ├── camera_render_service_impl.hpp
│   │   ├── camera_render_service_impl.cpp
│   │   ├── camera_renderer.hpp
│   │   ├── camera_renderer.cpp
│   │   └── offscreen_gl_context.hpp
│   │
│   └── viewer/
│       ├── simulation_viewer.hpp
│       ├── simulation_viewer.cpp
│       ├── lodepng/
│       └── simulate/
│
├── tests/
│   ├── public/
│   ├── unit/
│   └── package/
│
├── examples/
├── third_party/
├── cmake/
│   ├── Dependencies.cmake
│   ├── FindMujoco.cmake
│   ├── Sanitizers.cmake
│   ├── Install.cmake
│   └── mujoco_simulationConfig.cmake.in
│
└── CMakeLists.txt
```

不再需要：

```text
camera_render_service_factory_null.cpp
camera_render_service_factory_gl.cpp
viewer_service_factory_null.cpp
viewer_service_factory_glfw.cpp
null_camera_render_service.hpp
null_viewer_service.hpp
config_result.hpp
simulation_result.hpp
```

---

# 7. 公共 API 组织

## 7.1 顶层公共头

最终公共 API 的入口头包括：

```text
simulation.hpp
simulation_config.hpp
simulation_status.hpp
robot_state.hpp
component_id.hpp
export.hpp
version.hpp
```

为使这些入口头自包含，安装包还包含其值类型依赖：

```text
common/enum.hpp
common/math.hpp
config/config_limits.hpp
config/simulation_config.hpp
data/robot_state.hpp
component/component_id.hpp
```

它们只承载公开值类型，不得包含 Runtime、Renderer、Viewer、Parser 或
Buffer 等内部实现。

## 7.2 一组件一个公共头

采用：

```text
component/joint.hpp
component/imu.hpp
component/camera.hpp
component/lidar.hpp
component/mobile_base.hpp
```

不再拆分：

```text
joint_command.hpp
joint_state.hpp
joint_config.hpp
```

## 7.3 `joint.hpp`

包含：

```text
JointId
JointControlMode
JointCommand
JointCommandBatch
JointState
JointConfig
JointLimit
```

## 7.4 `imu.hpp`

包含：

```text
ImuId
ImuState
ImuConfig
```

## 7.5 `camera.hpp`

包含：

```text
CameraId
CameraConfig
Image
CameraInfo
CameraFrame
CameraState
```

`CameraInfo` 承载相机内参、畸变与投影矩阵；Camera 的异步渲染状态属于
内部 `CameraRenderTaskResult`，不作为公共 `CameraStatus` 类型暴露。

不得包含：

```text
CameraRenderService
CameraRenderer
OffscreenGlContext
GLFWwindow
EGLDisplay
EGLContext
mjModel
mjData
mjvScene
mjrContext
```

## 7.6 `lidar.hpp`

包含：

```text
LidarId
LidarConfig
LaserScan
LidarState
```

## 7.7 `mobile_base.hpp`

包含：

```text
MobileBaseId
WheelConfig
MobileBaseCommand
MobileBaseCommandBatch
MobileBaseState
MobileBaseConfig
```

---

# 8. XML 配置解析内部化

## 8.1 Parser 和 Validator

以下类型完全内部化：

```text
SimulationConfigParser
SimulationConfigValidator
```

文件位于：

```text
src/config/
```

它们：

* 不安装；
* 不导出；
* 不进入公共 include path；
* 不提供公共自由函数；
* 不作为扩展点。

## 8.2 `Simulation::initialize(path)`

公共接口：

```cpp
bool Simulation::initialize(
    const std::string& config_path);
```

内部流程：

```text
读取 XML
→ SimulationConfigParser
→ SimulationConfig
→ SimulationConfigValidator
→ 初始化 MuJoCo
→ 初始化 ComponentManager
→ 初始化 CameraRenderServiceImpl
→ 按 viewer_enabled 决定是否初始化 Viewer
```

任何步骤失败：

```text
记录日志
→ 清理已创建资源
→ 必要时进入 Error
→ 返回 false
```

## 8.3 `Simulation::initialize(config)`

保留：

```cpp
bool Simulation::initialize(
    const SimulationConfig& config);
```

该接口用于：

* 程序化配置；
* 单元测试；
* 仓库内原本先解析再修改配置的调用方；
* 不使用 XML 的内部场景。

## 8.4 仓库内迁移

原本直接使用 Parser 的代码改为以下两种方式之一。

### 直接加载 XML

```cpp
Simulation simulation;

if (!simulation.initialize(config_path)) {
  return false;
}
```

### 程序化构造配置

```cpp
SimulationConfig config;
config.viewer_enabled = false;
// 填充其他字段

Simulation simulation;

if (!simulation.initialize(config)) {
  return false;
}
```

---

# 9. CommandBuffer 与 StateBuffer

## 9.1 重命名

原：

```text
CommandBus
command_bus.hpp
command_bus.cpp
command_bus_
```

统一改为：

```text
CommandBuffer
command_buffer.hpp
command_buffer.cpp
command_buffer_
```

## 9.2 命名理由

`CommandBuffer` 更符合其实际职责：

* 接收命令写入；
* 按命令类型维护通道；
* 验证命令；
* 保存最新命令；
* 发布读取快照；
* 管理 sequence；
* 为物理线程提供一致视图。

它不是：

* 跨进程总线；
* 消息代理；
* 发布订阅系统；
* 网络路由器。

该名称也与 `StateBuffer` 形成对称：

```text
CommandBuffer
StateBuffer
```

## 9.3 行为保持不变

本次仅统一命名，不改变：

* 多命令类型通道；
* Component ID 映射；
* CommandTraits；
* 命令验证；
* sequence；
* batch 写入；
* 最新命令快照；
* 写线程与物理线程同步策略；
* 无新命令时的保持策略。

---

# 10. 公共命令 API 去模板化

## 10.1 删除公共模板

删除：

```cpp
template <typename Command>
bool write_command(
    ComponentId id,
    const Command& command);
```

删除所有要求下游看到 `CommandBuffer`、`CommandTraits` 或模板实现的公共接口。

## 10.2 强类型 overload

公共接口改为：

```cpp
bool write_command(
    JointId id,
    const JointCommand& command);

bool write_command(
    MobileBaseId id,
    const MobileBaseCommand& command);

bool write_commands(
    const JointCommandBatch& commands);

bool write_commands(
    const MobileBaseCommandBatch& commands);
```

状态读取：

```cpp
bool read_state(
    JointId id,
    JointState& state) const;

bool read_state(
    ImuId id,
    ImuState& state) const;

bool read_state(
    CameraId id,
    CameraState& state) const;

bool read_state(
    LidarId id,
    LidarState& state) const;

bool read_state(
    MobileBaseId id,
    MobileBaseState& state) const;

bool read_state(
    RobotState& state) const;
```

## 10.3 实现方式

实现全部位于 `.cpp`：

```cpp
bool Simulation::write_command(
    JointId id,
    const JointCommand& command) {
  if (!impl_->command_buffer.write(id, command)) {
    LOG(ERROR)
        << "Failed to write joint command, id="
        << id.value();
    return false;
  }

  return true;
}
```

不采用：

```text
void*
std::any
std::type_index
公共 std::variant 命令
```

---

# 11. `Simulation` PImpl

## 11.1 公共接口

```cpp
class Simulation {
public:
  Simulation();
  ~Simulation();

  Simulation(const Simulation&) = delete;
  Simulation& operator=(const Simulation&) = delete;

  Simulation(Simulation&&) = delete;
  Simulation& operator=(Simulation&&) = delete;

  bool initialize(const SimulationConfig& config);
  bool initialize(const std::string& config_path);

  bool start();
  bool stop();
  bool pause();
  bool resume();

  bool reset();
  bool reset(const std::string& keyframe);

  bool step(std::size_t count = 1);
  bool shutdown();

  SimulationStatus status() const;

  std::uint64_t step_count() const;
  double time() const;

  bool write_command(
      JointId id,
      const JointCommand& command);

  bool write_command(
      MobileBaseId id,
      const MobileBaseCommand& command);

  bool write_commands(
      const JointCommandBatch& commands);

  bool write_commands(
      const MobileBaseCommandBatch& commands);

  bool read_state(RobotState& state) const;

  // 保留快照读取和同类组件批量读取，供需要共享不可变状态快照的调用方使用。
  bool read_state(std::shared_ptr<const RobotState>& state) const;

  bool read_state(
      JointId id,
      JointState& state) const;

  bool read_state(
      ImuId id,
      ImuState& state) const;

  bool read_state(
      CameraId id,
      CameraState& state) const;

  bool read_state(
      LidarId id,
      LidarState& state) const;

  bool read_state(
      MobileBaseId id,
      MobileBaseState& state) const;

  bool read_state(JointStates& states) const;
  bool read_state(ImuStates& states) const;
  bool read_state(CameraStates& states) const;
  bool read_state(LidarStates& states) const;
  bool read_state(MobileBaseStates& states) const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};
```

实际头文件以 `MUJOCO_SIMULATION_PUBLIC` 标注每个非内联构造、析构和成员
函数，而不是标注整个类；该宏由最终 `mujoco_simulation` target 的
`GenerateExportHeader` 生成。

不再包含：

```text
last_result()
SimulationResult
ConfigLoadResult
```

## 11.2 `Simulation::Impl`

定义在：

```text
src/facade/simulation.cpp
```

建议结构：

```cpp
class Simulation::Impl {
public:
  SimulationConfig config;

  CommandBuffer command_buffer;
  StateBuffer state_buffer;
  ComponentManager component_manager;

  std::unique_ptr<SimulationRuntime> runtime;
  std::unique_ptr<SimulationScheduler> scheduler;

  std::unique_ptr<CameraRenderService>
      camera_render_service;

  // 同步任务会在锁外短暂持有 Viewer，因此使用 shared_ptr 取得稳定快照。
  std::shared_ptr<SimulationViewer> viewer;

  std::mutex lifecycle_mutex;
  std::mutex mujoco_mutex;

  std::atomic<bool> runtime_failed{false};

  std::uint64_t applied_command_sequence{0};
  std::uint64_t published_state_sequence{0};
};
```

## 11.3 具体实现创建

因为 Renderer 和 Viewer 始终构建，Facade 直接创建具体实现：

```cpp
camera_render_service =
    std::make_unique<CameraRenderServiceImpl>();

viewer =
    std::make_shared<SimulationViewer>(
        config.viewer_startup_timeout);
```

不再使用：

* factory；
* Null implementation；
* capability registry；
* 构建时实现选择。

Viewer 是否运行由：

```cpp
config.viewer_enabled
```

决定。

## 11.4 移动语义

本轮禁止移动，原因包括：

* worker callback 可能捕获外层地址；
* Runtime、Renderer、Viewer 存在线程；
* 对象地址可能被内部回调引用；
* 当前没有业务上的移动需求。

## 11.5 PImpl 验收条件

`simulation.hpp` 不得包含：

```text
CommandBuffer
StateBuffer
ComponentManager
SimulationRuntime
SimulationScheduler
CameraRenderService
CameraRenderer
OffscreenGlContext
SimulationViewer
std::mutex
std::atomic
mjModel
mjData
OpenGL
EGL
GLFW
```

## 11.6 ABI 表述

PImpl 仅能：

> 降低 `Simulation` 私有成员布局和内部依赖变化导致 ABI 破坏的概率。

不能宣称整个库 ABI 稳定。

以下仍属于 ABI 合同：

```text
SimulationConfig
SimulationStatus
RobotState
CameraFrame
JointCommand
std::vector
std::string
std::variant
枚举布局
公共函数签名
编译器 ABI
libstdc++ ABI
```

---

# 12. 内部 OBJECT target 架构

内部只保留三个主要 target：

```text
mujoco_simulation_runtime
mujoco_simulation_render
mujoco_simulation_viewer
```

三者本身均为：

```text
OBJECT library
```

不再创建：

```text
mujoco_simulation_runtime_objects
mujoco_simulation_render_objects
mujoco_simulation_viewer_objects
```

也不再把 Runtime、Render、Viewer 定义为独立 STATIC 或 SHARED library。

最终只有：

```text
mujoco_simulation
```

是实际的 STATIC 或 SHARED library。

---

# 13. Runtime OBJECT target

## 13.1 定义

```cmake
add_library(
  mujoco_simulation_runtime
  OBJECT
    ${MUJOCO_SIMULATION_RUNTIME_SOURCES}
)
```

## 13.2 包含内容

Runtime target 包含：

```text
CommandBuffer
StateBuffer
Component base
ComponentManager
JointComponent
ImuComponent
CameraComponent
LidarComponent
MobileBaseComponent
SimulationConfigParser
SimulationConfigValidator
SimulationRuntime
SimulationScheduler
CameraRenderService 抽象及 batch 数据类型
```

## 13.3 属性

```cmake
set_target_properties(
  mujoco_simulation_runtime
  PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)
```

## 13.4 编译依赖

```cmake
target_link_libraries(
  mujoco_simulation_runtime
  PRIVATE
    mujoco::mujoco
    Threads::Threads
    mujoco_simulation_sanitizers
)
```

Runtime 源码不得依赖：

```text
CameraRenderer
CameraRenderServiceImpl
OffscreenGlContext
SimulationViewer
OpenGL
EGL
GLFW
```

---

# 14. Render OBJECT target

## 14.1 定义

```cmake
add_library(
  mujoco_simulation_render
  OBJECT
    ${MUJOCO_SIMULATION_RENDER_SOURCES}
)
```

## 14.2 包含内容

Render target 包含：

```text
CameraRenderServiceImpl
CameraRenderer
OffscreenGlContext
Camera batch queue
CameraRenderTicket
wait/query/result 实现
mjData snapshot pool
latest-only
batch supersede
Render worker
```

## 14.3 属性

```cmake
set_target_properties(
  mujoco_simulation_render
  PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)
```

## 14.4 编译依赖

```cmake
target_link_libraries(
  mujoco_simulation_render
  PRIVATE
    mujoco::mujoco
    Threads::Threads
    OpenGL::GL
    OpenGL::EGL
    glfw
    ${CMAKE_DL_LIBS}
    mujoco_simulation_sanitizers
)
```

Render 使用 Runtime 内部定义的：

```text
CameraRenderService
CameraRenderBatchRequest
CameraRenderTicket
CameraRenderBatchResult
```

但 Runtime 不依赖 Render。

---

# 15. Viewer OBJECT target

## 15.1 定义

```cmake
add_library(
  mujoco_simulation_viewer
  OBJECT
    ${MUJOCO_SIMULATION_VIEWER_SOURCES}
)
```

## 15.2 包含内容

Viewer target 包含：

```text
SimulationViewer
MuJoCo simulate backend
Viewer UI
GLFW adapter
lodepng integration
```

## 15.3 属性

```cmake
set_target_properties(
  mujoco_simulation_viewer
  PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)
```

## 15.4 编译依赖

```cmake
target_link_libraries(
  mujoco_simulation_viewer
  PRIVATE
    mujoco::mujoco
    Threads::Threads
    OpenGL::GL
    glfw
    ${CMAKE_DL_LIBS}
    mujoco_simulation_sanitizers
)
```

Viewer 的源码可以使用 Render 提供的相关内部能力，但最终不会生成独立 Viewer archive。

---

# 16. 最终库

## 16.1 Library 类型

```cmake
if(MUJOCO_SIMULATION_BUILD_SHARED)
  set(MUJOCO_SIMULATION_LIBRARY_TYPE SHARED)
else()
  set(MUJOCO_SIMULATION_LIBRARY_TYPE STATIC)
endif()
```

## 16.2 定义

```cmake
add_library(
  mujoco_simulation
  ${MUJOCO_SIMULATION_LIBRARY_TYPE}
    src/facade/simulation.cpp

    $<TARGET_OBJECTS:mujoco_simulation_runtime>
    $<TARGET_OBJECTS:mujoco_simulation_render>
    $<TARGET_OBJECTS:mujoco_simulation_viewer>

    $<TARGET_OBJECTS:mujoco_simulation_tinyxml2>
    $<TARGET_OBJECTS:mujoco_simulation_logging>
    $<TARGET_OBJECTS:mujoco_simulation_viewer_backend>
)
```

对外别名：

```cmake
add_library(
  mujoco_simulation::mujoco_simulation
  ALIAS mujoco_simulation
)
```

## 16.3 对象唯一归属

每个 Runtime、Render、Viewer 源文件只能属于一个 OBJECT target。

不得同时：

* 加入 OBJECT target；
* 又直接加入最终 `mujoco_simulation` source list；
* 又加入另一个 STATIC/SHARED target。

最终库只通过：

```cmake
$<TARGET_OBJECTS:...>
```

取得内部对象。

---

# 17. 第三方 OBJECT targets

## 17.1 tinyxml2

```cmake
add_library(
  mujoco_simulation_tinyxml2
  OBJECT
    third_party/tinyxml2/tinyxml2.cpp
)

set_target_properties(
  mujoco_simulation_tinyxml2
  PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)
```

## 17.2 easylogging++

```cmake
add_library(
  mujoco_simulation_logging
  OBJECT
    third_party/easyloggingpp/easylogging++.cc
)

set_target_properties(
  mujoco_simulation_logging
  PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)
```

easylogging++ 实现只并入最终库一次，避免多个共享对象中出现重复日志全局状态。

## 17.3 Viewer backend

```cmake
add_library(
  mujoco_simulation_viewer_backend
  OBJECT
    ${MUJOCO_SIMULATION_VIEWER_BACKEND_SOURCES}
)

set_target_properties(
  mujoco_simulation_viewer_backend
  PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)
```

## 17.4 第三方 target 规则

这些 OBJECT targets：

```text
mujoco_simulation_tinyxml2
mujoco_simulation_logging
mujoco_simulation_viewer_backend
```

必须：

* 不安装；
* 不导出；
* 不进入 Package Config；
* 不出现在最终 imported target 的 `INTERFACE_LINK_LIBRARIES`；
* 不以独立 archive 形式要求 consumer 链接。

---

# 18. CameraRenderService 批量抽象

## 18.1 保留抽象的目的

尽管 Renderer 始终构建，`CameraRenderService` 仍应保留，原因包括：

* Runtime 不直接依赖 OpenGL 实现；
* ComponentManager 不直接访问 CameraRenderer；
* batch snapshot 所有权集中；
* ticket、wait、query 和 result 集中管理；
* latest-only 与 supersede 语义集中；
* Render 可独立单元测试；
* 后续替换 Renderer 实现时无需修改 Runtime。

该接口是内部架构边界，不是公共插件 API。

## 18.2 CameraRenderTask

```cpp
struct CameraRenderTask {
  CameraId camera_id{};

  int mujoco_camera_id{-1};

  std::uint32_t width{0};
  std::uint32_t height{0};

  CameraPixelFormat pixel_format{
      CameraPixelFormat::Rgb8};

  bool render_depth{false};
};
```

它只描述单个 Camera 的渲染任务，不拥有：

* `mjData`；
* OpenGL context；
* worker；
* 输出队列；
* ticket 生命周期。

## 18.3 CameraRenderBatchRequest

```cpp
struct CameraRenderBatchRequest {
  std::uint64_t generation{0};
  std::uint64_t sequence{0};

  std::uint64_t simulation_step{0};
  double simulation_time{0.0};

  const mjModel* model{nullptr};
  const mjData* data{nullptr};

  std::vector<CameraRenderTask> tasks;
};
```

该请求表示：

> 在同一个 MuJoCo 动态状态快照上渲染的一批 Camera。

`model` 和 `data` 均为借用指针。

其中：

* `data` 仅在 `submit()` 调用期间有效；
* `model` 可以被 worker 只读共享，但生命周期必须覆盖全部渲染任务。

## 18.4 CameraRenderTicket

```cpp
struct CameraRenderTicket {
  std::uint64_t generation{0};
  std::uint64_t sequence{0};

  bool valid() const noexcept {
    return generation != 0 || sequence != 0;
  }
};
```

Ticket 表示整个 batch，而不是单个 Camera。

## 18.5 Submit result

```cpp
enum class CameraRenderSubmitResult {
  Accepted,
  ReplacedPendingBatch,
  InvalidRequest,
  Stopped,
  Failed,
};
```

不再需要：

```text
Unavailable
```

因为 Renderer 始终存在。

## 18.6 Wait 状态

```cpp
enum class CameraRenderWaitStatus {
  Completed,
  PartiallyFailed,
  Failed,
  Superseded,
  Stale,
  Cancelled,
  Timeout,
  InvalidTicket,
  Stopped,
};
```

## 18.7 Camera task result

```cpp
enum class CameraTaskStatus {
  Pending,
  Rendering,
  Completed,
  Failed,
  Superseded,
  Stale,
  Cancelled,
};

struct CameraRenderTaskResult {
  CameraId camera_id{};

  CameraTaskStatus status{
      CameraTaskStatus::Pending};

  CameraFrame frame;

  // Same batch metadata is copied into every Camera result so callers can
  // verify that no cross-batch frame was merged.
  std::uint64_t generation{0};
  std::uint64_t batch_sequence{0};
  std::uint64_t simulation_step{0};
  double simulation_time{0.0};
  std::uint64_t sequence{0};
  std::uint64_t timestamp{0};
  std::string message;
};
```

## 18.8 Batch result

```cpp
enum class CameraBatchStatus {
  Pending,
  Rendering,
  Completed,
  PartiallyFailed,
  Failed,
  Superseded,
  Stale,
  Cancelled,
};

struct CameraRenderBatchResult {
  CameraRenderTicket ticket;

  CameraBatchStatus status{
      CameraBatchStatus::Pending};

  std::uint64_t simulation_step{0};
  double simulation_time{0.0};

  std::vector<CameraRenderTaskResult> cameras;
};
```

成功与失败的聚合规则应保持当前实现语义，不在本次结构重构中额外修改。

## 18.9 Service 接口

```cpp
class CameraRenderService {
public:
  virtual ~CameraRenderService() = default;

  CameraRenderService(
      const CameraRenderService&) = delete;

  CameraRenderService& operator=(
      const CameraRenderService&) = delete;

  virtual bool initialize(
      const SimulationConfig& config,
      const mjModel* model) = 0;

  virtual CameraRenderSubmitResult submit(
      const CameraRenderBatchRequest& request,
      CameraRenderTicket& ticket) = 0;

  virtual CameraRenderWaitStatus wait(
      const CameraRenderTicket& ticket,
      std::chrono::milliseconds timeout) = 0;

  virtual CameraRenderWaitStatus query(
      const CameraRenderTicket& ticket) const = 0;

  virtual bool read_batch_result(
      const CameraRenderTicket& ticket,
      CameraRenderBatchResult& result) = 0;

  virtual bool reset() = 0;
  virtual bool shutdown() = 0;

protected:
  CameraRenderService() = default;
};
```

该接口：

* 位于 `src/`；
* 不安装；
* 不导出；
* 不作为第三方插件接口。

---

# 19. CameraComponent 与 ComponentManager

## 19.1 CameraComponent 职责

CameraComponent 负责：

* 判断当前 tick 是否到期；
* 构造自身的 `CameraRenderTask`；
* 维护 CameraConfig；
* 接收自身 Camera 结果；
* 更新 CameraState。

推荐接口：

```cpp
class CameraComponent final : public Component {
public:
  bool due(
      std::uint64_t simulation_step,
      double simulation_time) const;

  bool build_render_task(
      CameraRenderTask& task) const;

  void apply_render_result(
      const CameraRenderTaskResult& result);
};
```

CameraComponent 不直接调用：

```cpp
CameraRenderService::submit()
```

## 19.2 ComponentManager 职责

ComponentManager 负责：

```text
遍历 CameraComponent
→ 收集当前 tick 所有 due CameraRenderTask
→ 构造一个 CameraRenderBatchRequest
→ 调用一次 submit()
→ 保存 CameraRenderTicket
→ query 或 wait batch
→ 读取 batch result
→ 将子结果分发给对应 CameraComponent
```

初始化接口：

```cpp
bool ComponentManager::initialize(
    const SimulationConfig& config,
    const mjModel* model,
    CameraRenderService* camera_render_service);
```

`CameraRenderService*`：

* 非拥有；
* 生命周期由 `Simulation::Impl` 管理；
* 在 ComponentManager 运行期间始终有效。

## 19.3 Batch 提交流程

```cpp
std::vector<CameraRenderTask> due_tasks;

for (auto& camera : camera_components_) {
  if (!camera->due(step, time)) {
    continue;
  }

  CameraRenderTask task;

  if (camera->build_render_task(task)) {
    due_tasks.push_back(std::move(task));
  }
}

if (!due_tasks.empty()) {
  CameraRenderBatchRequest request;
  request.generation = camera_generation_;
  request.sequence = next_camera_sequence_++;
  request.simulation_step = step;
  request.simulation_time = time;
  request.model = model;
  request.data = data;
  request.tasks = std::move(due_tasks);

  CameraRenderTicket ticket;

  const auto result =
      camera_render_service_->submit(
          request,
          ticket);

  handle_camera_submit_result(
      result,
      ticket);
}
```

该调用必须发生在主 `mjData` 锁保护范围内。

---

# 20. Camera 数据所有权与并发契约

## 20.1 单 batch 单快照

一个 Camera batch 必须只创建一次 MuJoCo 动态状态快照。

正确流程：

```text
物理线程持有主 mjData 锁
→ 收集所有 due Camera
→ submit(batch)
→ submit 内取得 owned mjData
→ 执行一次 mj_copyData
→ batch snapshot 入队
→ submit 返回
→ 释放主 mjData 锁
→ Render worker 使用 owned snapshot
```

禁止退化为：

```text
N 个 Camera
→ N 次 mj_makeData
→ N 次 mj_copyData
```

## 20.2 `submit()` 强制合同

`submit()` 返回前必须：

* 校验请求；
* 取得可用的 owned `mjData`；
* 对整个 batch 执行一次 `mj_copyData()`；
* 完成 batch snapshot；
* 将 snapshot 放入 active 或 pending 调度结构；
* 生成或返回 batch ticket。

`submit()` 返回后：

* worker 不得保存 `request.data`；
* worker 不得访问主仿真 `mjData`；
* worker 不得持有主 MuJoCo 数据锁；
* 调用方无需延长主 `mjData` 生命周期等待渲染完成。

## 20.3 禁止的实现

禁止：

```text
submit() 保存 request.data
→ 立即返回
→ worker 稍后访问 request.data
```

也禁止：

```text
worker 稍后获取 Simulation 的主数据锁
→ 再从主 mjData 复制
```

原因：

* 引入跨模块锁耦合；
* 可能访问已重建或释放的数据；
* 容易产生锁反转；
* 容易造成 shutdown 死锁；
* 破坏物理线程和渲染线程隔离。

## 20.4 Batch snapshot

```cpp
struct CameraRenderBatchSnapshot {
  CameraRenderTicket ticket;

  std::uint64_t simulation_step{0};
  double simulation_time{0.0};

  const mjModel* model{nullptr};

  std::unique_ptr<mjData, MjDataDeleter> data;

  std::vector<CameraRenderTask> tasks;
};
```

该对象由 Render Service 完整拥有。

## 20.5 `mjModel` 合同

允许 Render worker 只读共享 `mjModel`，但必须满足：

1. `mjModel` 生命周期覆盖 worker；
2. worker 仅只读访问；
3. 运行时不并发修改模型结构；
4. 重载或释放模型前必须停止并 join Render worker；
5. 若未来支持运行时修改 `mjModel`，需要另行设计同步机制。

## 20.6 快照池

允许复用 `mjData`：

```text
从空闲池取得 owned mjData
→ submit 内执行 mj_copyData
→ snapshot 入队
→ worker 渲染整批 Camera
→ 完成后归还池
```

快照池不得改变以下合同：

* 每 batch 一次主数据复制；
* 原始 `request.data` 不进入队列；
* reset 清理旧 generation；
* shutdown 在释放池前先停止 worker。

---

# 21. Camera batch 行为合同

## 21.1 同 tick 多 Camera 一致性

同一个 `CameraRenderTicket` 中的所有 Camera 必须共享：

```text
generation
sequence
simulation_step
simulation_time
初始 mjData snapshot
```

因此同一 batch 中的 Camera 帧都来自同一个仿真时刻。

即使 Camera 串行渲染，也必须共享同一初始状态：

```text
qpos
qvel
act
mocap_pos
mocap_quat
body transforms
site transforms
camera transforms
其他 mjData 动态字段
```

## 21.2 latest-only 的单位

latest-only 的基本调度单位是：

```text
完整 Camera batch
```

不是单个 Camera。

Render Service 最多保留：

* 一个 active batch；
* 一个 pending batch。

## 21.3 整批 supersede

假设：

```text
batch A 正在执行
batch B 正在等待
batch C 新提交
```

则：

```text
batch A 继续
batch B 整批变为 Superseded
batch C 成为最新 pending
```

batch B 的：

* snapshot；
* tasks；
* wait condition；
* pending result；

必须统一处理。

## 21.4 禁止跨 batch 按 Camera 合并

例如：

```text
batch B: camera 1, camera 2
batch C: camera 2, camera 3
```

不能产生：

```text
camera 1 来自 B
camera 2 来自 C
camera 3 来自 C
```

否则破坏同一 tick 状态一致性。

## 21.5 wait

```cpp
wait(ticket, timeout)
```

等待整个 batch 进入终态：

```text
Completed
PartiallyFailed
Failed
Superseded
Stale
Cancelled
```

而不是等待单个 Camera。

## 21.6 reset

reset 时：

1. 停止提交旧 generation；
2. generation 增加；
3. pending 旧 batch 整批 stale 或 cancel；
4. active 旧 batch 即使完成也不得发布；
5. 唤醒旧 ticket 的 wait；
6. 释放旧 batch snapshot；
7. 防止旧结果覆盖 reset 后 CameraState。

## 21.7 shutdown

推荐顺序：

```text
停止 ComponentManager 生成 Camera batch
→ CameraRenderService 停止接受 submit
→ pending batch 整批 Cancelled
→ 唤醒 Render worker
→ worker 完成或退出 active batch
→ join worker
→ 唤醒所有 ticket wait
→ 清理 snapshots/results/tickets
→ 销毁 ComponentManager
→ 销毁 mjData
→ 销毁 mjModel
```

---

# 22. Viewer 行为

Viewer 始终编译，但运行时按：

```cpp
config.viewer_enabled
```

决定是否启动。

示例：

```cpp
if (config.viewer_enabled) {
  if (!viewer_->initialize(model, data)) {
    LOG(ERROR)
        << "Failed to initialize simulation viewer.";

    status_ = SimulationStatus::Error;
    return false;
  }

  if (!viewer_->start()) {
    LOG(ERROR)
        << "Failed to start simulation viewer.";

    status_ = SimulationStatus::Error;
    return false;
  }
}
```

当：

```cpp
config.viewer_enabled == false
```

时：

* 不初始化 Viewer；
* 不创建窗口；
* 不启动 Viewer 线程；
* 不同步 Viewer；
* 物理仿真继续；
* Camera 离屏渲染仍可工作。

本轮不引入 `ViewerService` 抽象，直接由 `Simulation::Impl` 持有 `SimulationViewer`。

---

# 23. 最终外部链接依赖

最终库始终依赖：

```text
MuJoCo
Threads
OpenGL
EGL
GLFW
dl
```

最终 target 应显式声明依赖。

建议初始配置：

```cmake
target_link_libraries(
  mujoco_simulation
  PUBLIC
    mujoco::mujoco
    Threads::Threads
    OpenGL::GL
    OpenGL::EGL
    glfw
  PRIVATE
    ${CMAKE_DL_LIBS}
    mujoco_simulation_sanitizers
)
```

对于静态库：

* OpenGL；
* EGL；
* GLFW；
* MuJoCo；
* Threads；
* `dl`；

是否必须全部出现在 exported interface 中，应以真实 static consumer 测试为准。

不要仅依赖：

```text
PRIVATE
$<LINK_ONLY:...>
```

的偶然传播结果。

---

# 24. 构建选项

## 24.1 顶层判断

保持 CMake 3.16，不使用 `PROJECT_IS_TOP_LEVEL`。

```cmake
if(CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR)
  set(MUJOCO_SIMULATION_IS_TOP_LEVEL ON)
else()
  set(MUJOCO_SIMULATION_IS_TOP_LEVEL OFF)
endif()
```

## 24.2 保留选项

```cmake
option(
  MUJOCO_SIMULATION_BUILD_SHARED
  "Build shared library"
  ON
)

option(
  MUJOCO_SIMULATION_BUILD_TESTS
  "Build tests"
  ${MUJOCO_SIMULATION_IS_TOP_LEVEL}
)

option(
  MUJOCO_SIMULATION_BUILD_EXAMPLES
  "Build examples"
  ${MUJOCO_SIMULATION_IS_TOP_LEVEL}
)

option(
  MUJOCO_SIMULATION_INSTALL
  "Generate installation rules"
  ${MUJOCO_SIMULATION_IS_TOP_LEVEL}
)

option(
  MUJOCO_SIMULATION_ENABLE_ASAN
  "Enable AddressSanitizer"
  OFF
)

option(
  MUJOCO_SIMULATION_ENABLE_UBSAN
  "Enable UndefinedBehaviorSanitizer"
  OFF
)

option(
  MUJOCO_SIMULATION_ENABLE_TSAN
  "Enable ThreadSanitizer"
  OFF
)
```

## 24.3 删除选项

删除：

```text
MUJOCO_SIMULATION_BUILD_RENDER
MUJOCO_SIMULATION_BUILD_VIEWER
```

## 24.4 Sanitizer 约束

```cmake
if(MUJOCO_SIMULATION_ENABLE_ASAN AND
   MUJOCO_SIMULATION_ENABLE_TSAN)
  message(
    FATAL_ERROR
    "ASan and TSan cannot be enabled together."
  )
endif()
```

---

# 25. 依赖发现

## 25.1 始终查找图形依赖

```cmake
find_package(Threads REQUIRED)

find_package(
  OpenGL
  REQUIRED
  COMPONENTS OpenGL EGL
)

find_package(
  glfw3
  CONFIG
  REQUIRED
)
```

## 25.2 MuJoCo 查找顺序

```cmake
if(NOT TARGET mujoco::mujoco)
  find_package(mujoco CONFIG QUIET)
endif()

if(NOT TARGET mujoco::mujoco)
  include(
    "${CMAKE_CURRENT_LIST_DIR}/cmake/FindMujoco.cmake"
  )

  mujoco_simulation_find_mujoco()
endif()

if(NOT TARGET mujoco::mujoco)
  message(
    FATAL_ERROR
    "MuJoCo was not found."
  )
endif()
```

## 25.3 `MUJOCO_ROOT`

```cmake
set(
  MUJOCO_ROOT
  ""
  CACHE PATH
  "MuJoCo installation prefix"
)
```

Fallback 顺序：

```text
MUJOCO_ROOT cache
MUJOCO_ROOT environment
/opt/mujoco
/opt/mujoco-3.9.0
```

具体版本目录只作为兼容 fallback，不作为安装包长期契约。

## 25.4 共用 Find 模块

构建树和安装包共用：

```text
cmake/FindMujoco.cmake
```

避免维护两套：

```text
find_path
find_library
mujoco::mujoco target 创建逻辑
```

---

# 26. 符号可见性

## 26.1 对外导出

主要导出：

```text
Simulation
```

以及确实存在非内联实现的稳定公共类型。

## 26.2 不导出

```text
SimulationRuntime
SimulationScheduler
Component
ComponentManager
JointComponent
ImuComponent
CameraComponent
LidarComponent
MobileBaseComponent
CommandBuffer
StateBuffer
CameraRenderService
CameraRenderServiceImpl
CameraRenderer
OffscreenGlContext
SimulationViewer
SimulationConfigParser
SimulationConfigValidator
Viewer backend
```

## 26.3 禁止公开内部资源

不得提供：

```cpp
mjData* data();
mjModel* model();
SimulationRuntime* runtime();
CommandBuffer* command_buffer();
CameraRenderer* renderer();
CameraRenderService* render_service();
SimulationViewer* viewer();
```

---

# 27. `GenerateExportHeader`

导出头针对最终 target：

```cmake
include(GenerateExportHeader)

set(
  MUJOCO_SIMULATION_GENERATED_INCLUDE_DIR
  "${CMAKE_CURRENT_BINARY_DIR}/generated"
)

generate_export_header(
  mujoco_simulation
  EXPORT_MACRO_NAME MUJOCO_SIMULATION_PUBLIC
  NO_EXPORT_MACRO_NAME MUJOCO_SIMULATION_LOCAL
  STATIC_DEFINE MUJOCO_SIMULATION_STATIC_DEFINE
  EXPORT_FILE_NAME
    "${MUJOCO_SIMULATION_GENERATED_INCLUDE_DIR}/mujoco_simulation/export.hpp"
)
```

公共 include path：

```cmake
target_include_directories(
  mujoco_simulation
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${MUJOCO_SIMULATION_GENERATED_INCLUDE_DIR}>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)
```

内部 OBJECT targets 使用：

```cmake
target_include_directories(
  mujoco_simulation_runtime
  PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)
```

Render、Viewer 同理。

---

# 28. Sanitizer

建立内部 INTERFACE target：

```cmake
add_library(
  mujoco_simulation_sanitizers
  INTERFACE
)
```

ASan：

```cmake
target_compile_options(
  mujoco_simulation_sanitizers
  INTERFACE
    -fsanitize=address
    -fno-omit-frame-pointer
)

target_link_options(
  mujoco_simulation_sanitizers
  INTERFACE
    -fsanitize=address
)
```

UBSan：

```text
-fsanitize=undefined
```

TSan：

```text
-fsanitize=thread
```

所有以下 targets 都应接入 Sanitizer：

```text
mujoco_simulation_runtime
mujoco_simulation_render
mujoco_simulation_viewer
mujoco_simulation_tinyxml2
mujoco_simulation_logging
mujoco_simulation_viewer_backend
mujoco_simulation
tests
examples
```

该 target：

* 不安装；
* 不导出；
* 不进入 Package Config。

---

# 29. CMake 模块化

```text
cmake/
├── Dependencies.cmake
├── FindMujoco.cmake
├── Sanitizers.cmake
├── Install.cmake
└── mujoco_simulationConfig.cmake.in
```

## 29.1 `Dependencies.cmake`

负责：

* Threads；
* MuJoCo；
* OpenGL；
* EGL；
* GLFW；
* `dl`。

## 29.2 `Sanitizers.cmake`

负责：

* Sanitizer options；
* 参数设置；
* 互斥验证；
* 内部 INTERFACE target。

## 29.3 `Install.cmake`

负责：

* 最终 target 安装；
* 公共头安装；
* 生成头安装；
* Config；
* ConfigVersion；
* `FindMujoco.cmake` 安装。

## 29.4 顶层 `CMakeLists.txt`

保留：

* `project()`；
* options；
* source lists；
* OBJECT target 创建；
* 最终 target 创建；
* include 和 link；
* tests/examples 子目录。

不继续拆分成大量过小 CMake 文件。

---

# 30. 安装与导出

## 30.1 只安装最终 target

```cmake
set_target_properties(
  mujoco_simulation
  PROPERTIES
    EXPORT_NAME mujoco_simulation
)

install(
  TARGETS mujoco_simulation
  EXPORT mujoco_simulationTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
```

## 30.2 单一 export set

```cmake
install(
  EXPORT mujoco_simulationTargets
  FILE mujoco_simulationTargets.cmake
  NAMESPACE mujoco_simulation::
  DESTINATION ${MUJOCO_SIMULATION_CMAKE_INSTALL_DIR}
)
```

最终 imported target：

```text
mujoco_simulation::mujoco_simulation
```

不得安装或导出：

```text
mujoco_simulation_runtime
mujoco_simulation_render
mujoco_simulation_viewer
mujoco_simulation_tinyxml2
mujoco_simulation_logging
mujoco_simulation_viewer_backend
mujoco_simulation_sanitizers
```

## 30.3 Config

Config 始终加载全部外部依赖：

```cmake
include(CMakeFindDependencyMacro)

find_dependency(Threads REQUIRED)

find_dependency(
  OpenGL
  REQUIRED
  COMPONENTS OpenGL EGL
)

find_dependency(
  glfw3
  CONFIG
  REQUIRED
)

# 查找或创建 mujoco::mujoco

include(
  "${CMAKE_CURRENT_LIST_DIR}/mujoco_simulationTargets.cmake"
)
```

不包含：

* Package Components；
* runtime/render/viewer 条件；
* capability metadata；
* 分离 Targets 文件；
* 构建能力判断。

---

# 31. 公共头安装

安装入口头及其公开值类型依赖。使用显式 `install(FILES ...)` 列表，而不
直接安装整个 `include/` 目录，以防把仅供构建树使用的内部辅助头安装出去：

```cmake
install(
  FILES
    include/mujoco_simulation/simulation.hpp
    include/mujoco_simulation/simulation_config.hpp
    include/mujoco_simulation/robot_state.hpp
    include/mujoco_simulation/component_id.hpp
    include/mujoco_simulation/simulation_status.hpp
  DESTINATION
    "${CMAKE_INSTALL_INCLUDEDIR}/mujoco_simulation"
)
```

生成头安装：

```cmake
install(
  FILES
    "${MUJOCO_SIMULATION_GENERATED_INCLUDE_DIR}/mujoco_simulation/export.hpp"
    "${MUJOCO_SIMULATION_GENERATED_INCLUDE_DIR}/mujoco_simulation/version.hpp"
  DESTINATION
    "${CMAKE_INSTALL_INCLUDEDIR}/mujoco_simulation"
)
```

安装目录不得出现：

```text
command_buffer.hpp
state_buffer.hpp
component_manager.hpp
simulation_runtime.hpp
simulation_scheduler.hpp
camera_render_service.hpp
camera_render_service_impl.hpp
camera_renderer.hpp
offscreen_gl_context.hpp
simulation_viewer.hpp
simulation_config_parser.hpp
simulation_config_validator.hpp
```

---

# 32. 测试结构

```text
tests/
├── public/
│   ├── simulation_lifecycle_test.cpp
│   └── component_header_contract_test.cpp
│
├── unit/
│   ├── command_buffer_test.cpp
│   ├── state_buffer_concurrency_test.cpp
│   ├── simulation_scheduler_realtime_test.cpp
│   ├── simulation_config_timing_test.cpp
│   ├── simulation_component_period_test.cpp
│   ├── component_manager_camera_reset_test.cpp
│   ├── camera_renderer_async_test.cpp
│   ├── camera_snapshot_tsan_test.cpp
│   └── simulation_viewer_headless_test.cpp
│
└── package/
    ├── shared_consumer/
    └── static_consumer/
```

## 32.1 Public tests

只能包含安装公共头，例如：

```cpp
#include <mujoco_simulation/simulation.hpp>
#include <mujoco_simulation/component/joint.hpp>
```

不得添加：

```text
src/
```

到 include path。

## 32.2 Unit tests

Unit tests 可以：

* 访问 `src/` 内部头；
* 直接测试 CommandBuffer；
* 直接测试 CameraRenderServiceImpl；
* 直接测试 Scheduler；
* 链接最终库；
* 或复用相关 OBJECT target。

需要避免把整个 Runtime 对象集合无差别并入每个小测试导致：

* 链接时间过长；
* 重复 main；
* 非必要依赖过多。

可以按测试需要：

* 直接列入被测源码；
* 建立测试专用细粒度 OBJECT target；
* 或链接最终库并通过私有头访问内部类。

测试辅助 target 不安装、不导出。

## 32.3 Package tests

Package tests只保留：

```text
shared_consumer
static_consumer
```

consumer 只允许：

```cmake
find_package(mujoco_simulation REQUIRED)

target_link_libraries(
  consumer
  PRIVATE
    mujoco_simulation::mujoco_simulation
)
```

不得显式链接：

```text
runtime
render
viewer
tinyxml2
easylogging++
lodepng
viewer backend
```

---

# 33. Camera 测试要求

## 33.1 单 batch 单快照

提交包含多个 Camera 的 batch，验证：

```text
mj_copyData 调用次数 == 1
```

不能等于 Camera 数量。

## 33.2 多 Camera 状态一致性

同一 batch 中所有 Camera 结果必须携带相同：

```text
generation
sequence
simulation_step
simulation_time
```

并来自同一个初始 snapshot。

## 33.3 Batch supersede

场景：

```text
A active
B pending
C submitted
```

应得到：

```text
A continues
B becomes Superseded
C becomes pending
```

不得只替换 B 中部分 Camera。

## 33.4 Batch wait

等待 B 的 ticket，应返回：

```text
Superseded
```

等待完成 ticket，应返回整个 batch 终态。

## 33.5 快照隔离

流程：

1. 调用 `submit()`；
2. `submit()` 返回；
3. 立即修改主 `mjData`；
4. worker 渲染；
5. 验证结果对应提交时快照。

## 33.6 reset generation

验证 reset 前 batch 的结果不会更新 reset 后任何 CameraState。

## 33.7 shutdown wait

shutdown 时所有：

```text
wait(ticket)
```

必须被唤醒并返回：

```text
Cancelled
Stopped
```

之一，不能永久阻塞。

## 33.8 TSan

在 TSan 下验证：

```text
物理线程更新主 mjData
submit() 持锁复制一次
Render worker 使用 owned snapshot
```

不得报告主 `mjData` 的跨线程数据竞争。

---

# 34. 实施阶段

## 阶段 0：设计冻结

确认：

1. 对外只有一个完整库；
2. Runtime、Render、Viewer 始终构建；
3. 三个内部 target 均为 OBJECT；
4. 不存在 `_objects` 重复 target；
5. 内部 target 不安装、不导出；
6. headless 仅表示 `viewer_enabled=false`；
7. 删除 ConfigLoadResult 和 SimulationResult；
8. 删除公共配置加载函数；
9. Parser 和 Validator 完全内部化；
10. `CommandBus` 重命名为 `CommandBuffer`；
11. Camera 保留 batch service；
12. 单 batch 单 snapshot；
13. static/shared 最终库均聚合全部对象；
14. 保持 CMake 3.16。

## 阶段 1：纯文件迁移

只做：

* 内部头移入 `src/`；
* 更新 include；
* 将 `src` 设为 PRIVATE include path；
* 调整 unit test include。

不做：

* PImpl；
* target 重构；
* API 重构；
* CommandBuffer 重命名；
* Camera service 抽象。

## 阶段 2：公共组件数据头重组

建立：

```text
component/joint.hpp
component/imu.hpp
component/camera.hpp
component/lidar.hpp
component/mobile_base.hpp
```

迁移仓库内 consumer。

## 阶段 3：配置 API 收敛

完成：

* 删除公共 Parser；
* 删除公共配置加载函数；
* 删除 ConfigLoadResult；
* 删除 `config_result.hpp`；
* 迁移为 `initialize(path)` 或 `initialize(config)`。

## 阶段 4：CommandBuffer 重命名

统一修改：

```text
CommandBus → CommandBuffer
command_bus.hpp → command_buffer.hpp
command_bus.cpp → command_buffer.cpp
command_bus_ → command_buffer_
command_bus_test → command_buffer_test
```

保持语义不变。

## 阶段 5：命令 API 去模板化

删除公共模板命令接口，增加强类型 overload。

## 阶段 6：PImpl 与符号收敛

完成：

* `Simulation::Impl`；
* 删除公共内部 include；
* 禁止移动；
* 删除内部类导出宏；
* GenerateExportHeader。

## 阶段 7：Camera batch service 抽象

完成：

* CameraRenderTask；
* CameraRenderBatchRequest；
* CameraRenderTicket；
* CameraRenderBatchResult；
* batch submit/wait/query；
* 单 batch 单 snapshot；
* latest-only；
* 整批 supersede；
* reset/shutdown 合同。

## 阶段 8：内部 OBJECT targets

建立：

```text
mujoco_simulation_runtime
mujoco_simulation_render
mujoco_simulation_viewer
```

三者均为 OBJECT targets。

## 阶段 9：第三方 OBJECT targets

建立：

```text
mujoco_simulation_tinyxml2
mujoco_simulation_logging
mujoco_simulation_viewer_backend
```

并聚合进最终库。

## 阶段 10：最终库与单一安装包

完成：

* 最终 STATIC/SHARED library；
* 唯一 export set；
* 唯一 Config；
* 唯一 imported target；
* shared/static consumer tests。

## 阶段 11：CMake 工程治理

完成：

* Dependencies；
* FindMujoco；
* Sanitizers；
* Install；
* CMake 3.16 兼容；
* tests/examples 默认行为。

---

# 35. 验收标准

## 35.1 对外 target

安装后只存在：

```text
mujoco_simulation::mujoco_simulation
```

不得存在：

```text
mujoco_simulation::runtime
mujoco_simulation::render
mujoco_simulation::viewer
mujoco_simulation::simulation
```

## 35.2 内部 target 模型

构建树中必须存在：

```text
mujoco_simulation_runtime          OBJECT
mujoco_simulation_render           OBJECT
mujoco_simulation_viewer           OBJECT
```

不得存在：

```text
mujoco_simulation_runtime_objects
mujoco_simulation_render_objects
mujoco_simulation_viewer_objects
```

也不得存在独立 STATIC/SHARED runtime、render、viewer。

## 35.3 源码唯一编译

每个内部 `.cpp` 只能归属于一个 OBJECT target。

最终库不直接重复列入这些源码。

## 35.4 Viewer 配置

代码和文档统一使用：

```cpp
config.viewer_enabled
```

不得出现：

```cpp
config.viewer.enabled
```

## 35.5 公共头

不得存在：

```text
config_result.hpp
simulation_result.hpp
```

公共头不得暴露内部实现类型。

## 35.6 配置 API

不得存在：

```text
ConfigLoadResult
SimulationResult
load_simulation_config()
Simulation::last_result()
```

## 35.7 错误行为

所有失败操作统一：

```text
记录日志
返回 false
```

`SimulationStatus::Error` 仅表示生命周期错误状态。

## 35.8 CommandBuffer

代码中不得再存在：

```text
CommandBus
command_bus.hpp
command_bus.cpp
command_bus_
```

统一为：

```text
CommandBuffer
command_buffer.hpp
command_buffer.cpp
command_buffer_
```

## 35.9 固定构建能力

CMake 配置始终查找：

```text
MuJoCo
Threads
OpenGL
EGL
GLFW
```

不得存在：

```text
BUILD_RENDER
BUILD_VIEWER
NullCameraRenderService
NullViewerService
Render/Viewer factory
Package Components
```

## 35.10 Runtime 依赖边界

Runtime 源码不得直接包含：

```text
CameraRenderer
CameraRenderServiceImpl
OffscreenGlContext
SimulationViewer
OpenGL
EGL
GLFW
```

允许依赖内部 `CameraRenderService` 抽象。

## 35.11 静态库闭环

static consumer 只链接：

```text
mujoco_simulation::mujoco_simulation
```

不得要求显式链接：

```text
runtime
render
viewer
tinyxml2
easylogging++
lodepng
viewer backend
```

## 35.12 导出文件

安装后的 CMake 文件不得引用：

```text
mujoco_simulation_runtime
mujoco_simulation_render
mujoco_simulation_viewer
mujoco_simulation_tinyxml2
mujoco_simulation_logging
mujoco_simulation_viewer_backend
```

## 35.13 Camera 数据安全

必须满足：

```text
submit 返回前完成 batch snapshot
每 batch 只复制一次 mjData
worker 不保存主 mjData*
worker 不访问主 mjData*
worker 不持有主 MuJoCo 数据锁
mjModel 只读共享且生命周期受控
```

## 35.14 Camera batch 行为

必须保持：

```text
batch submit
单 batch 单 snapshot
batch ticket
batch wait
batch latest-only
整批 supersede
禁止跨 batch 合并 Camera
同 tick 多 Camera 状态一致性
```

## 35.15 行为回归

必须保持：

* initialize；
* shutdown；
* start；
* stop；
* pause；
* resume；
* reset；
* manual step；
* 稀疏 ID；
* 多通道 CommandBuffer；
* Command 验证；
* Command sequence；
* State snapshot；
* Camera batch；
* Viewer latest-only；
* XML 解析错误日志。

---

# 36. 最终架构

```text
Public API
├── Simulation
├── SimulationConfig
├── SimulationStatus
├── RobotState
└── component data contracts

Internal Runtime OBJECT target
├── SimulationRuntime
├── SimulationScheduler
├── ComponentManager
├── CommandBuffer
├── StateBuffer
├── Parser
├── Validator
├── Components
└── CameraRenderService abstraction

Internal Render OBJECT target
├── CameraRenderServiceImpl
├── CameraRenderer
├── Camera batch queue
├── Camera ticket/wait/result
├── snapshot pool
└── OffscreenGlContext

Internal Viewer OBJECT target
├── SimulationViewer
└── MuJoCo simulate backend

Final public library
└── Simulation::Impl
    ├── Runtime objects
    ├── Render objects
    ├── Viewer objects
    └── embedded third-party objects
```

运行时模式：

```text
viewer_enabled = true
→ 物理仿真 + Camera + Viewer

viewer_enabled = false
→ 物理仿真 + 可选 Camera，不启动 Viewer

无 Camera 配置
→ 物理仿真 + 可选 Viewer，不提交 Camera batch
```

所有运行模式使用同一个完整构建产物。

---

# 37. 最终结论

重构完成后，`mujoco_simulation` 应具备以下特征：

* 工程内部保持 Runtime、Render、Viewer 清晰分层；
* 三个内部层均使用 OBJECT target；
* 不存在重复的 `_objects` target；
* 最终只有一个 STATIC 或 SHARED library；
* 对外只导出 `mujoco_simulation::mujoco_simulation`；
* 不再维护多种能力安装包；
* headless 仅表示运行时 `viewer_enabled=false`；
* XML Parser 和 Validator 完全内部化；
* 删除公共配置结果和仿真结果对象；
* 失败统一记录日志并返回 `false`；
* `SimulationStatus::Error` 只表示生命周期错误状态；
* `CommandBus` 统一重命名为 `CommandBuffer`；
* CommandBuffer 与 StateBuffer 形成对称的数据边界；
* 公共模板命令 API 被强类型 overload 替代；
* `Simulation` 使用 PImpl；
* Runtime 通过 batch-oriented `CameraRenderService` 与 Renderer 解耦；
* Camera 保留单 batch 单 snapshot、ticket、wait、latest-only 和整批 supersede；
* 异步 worker 只访问 Renderer 自己拥有的 `mjData` 快照；
* 内部和第三方对象均直接聚合进最终库；
* static consumer 不依赖未安装的内部 targets；
* 安装包只包含一个导出 target 和一套 Config；
* shared/static 均通过独立 consumer 工程验证。

该方案实现了内部架构清晰、数据所有权明确、静态链接闭环以及对外使用简单之间的统一。
