# robot_mujoco 目标四层包架构设计

本文描述 `robot_mujoco` 工作区后续希望收敛到的目标包边界、依赖规则和迁移方案。

当前真实代码边界、主调用链和验证状态仍以 [`current_architecture.md`](./current_architecture.md) 为准。本文是目标设计文档，不代表代码已经完成迁移，也不替代现状快照。

## 1. 背景与问题

当前 `robot_mujoco` 工作区内与 MuJoCo 运行时直接相关的包主要包括：

- `mujoco_simulation`
- `mujoco_simulation_ros`
- `mujoco_hardware`
- `mujoco_ros2_bridge_msgs`
- `robot_mujoco`

现有边界已经比早期版本清晰，但仍然存在两个核心问题。

### 1.1 ROS adapter 在运行时层面仍然紧耦合

当前文档已经把：

- `mujoco_simulation` 定义为纯 MuJoCo runtime
- `mujoco_simulation_ros` 定义为 ROS bridge
- `mujoco_hardware` 定义为 `ros2_control` 适配层

但从实际调用链看，`mujoco_hardware` 并不是只依赖一个抽象的 ROS adapter 接口，而是直接创建并持有 `mujoco_simulation_ros::SimulationRosBridge`，同时把仿真状态发布、控制服务和 `ros2_control` 读写链路组织在一起。

这说明当前的问题不在于“有没有拆成多个包”，而在于 ROS 2 适配层虽然在包名上分开了，在运行时职责上还没有真正收口为一个统一边界。

### 1.2 `robot_mujoco` 当前职责过宽

`robot_mujoco` 目前同时承担了多种角色：

- 工作区级聚合入口
- C++ 头文件聚合入口
- launch/config 提供者
- RoboCasa Python 集成入口

这种设计虽然在项目早期有利于快速整合，但长期看会带来两个问题：

- 顶层包边界不稳定，既像元包，又像产品实现包
- 下游容易把 `robot_mujoco` 误用为统一底层 C++ API 入口，进一步模糊 runtime、ROS adapter 和产品入口三层职责

### 1.3 需要解决的根因

当前需要解决的根因不是“包数量太多”，而是下面三层职责还没有完全收口：

- runtime 层
- ROS adapter 层
- 产品入口层

因此，目标架构的重点应是固定这三层边界，而不是仅做目录重命名。

## 2. 目标四层架构总览

目标架构固定为以下四层：

- `mujoco_simulation`
- `robot_mujoco_msgs`
- `robot_mujoco_ros2`
- `robot_mujoco`

依赖方向固定如下：

```text
robot_mujoco
  -> robot_mujoco_ros2
    -> robot_mujoco_msgs
    -> mujoco_simulation
```

配套约束如下：

- `mujoco_simulation` 必须保持为纯 C++ MuJoCo runtime，不依赖 ROS
- `robot_mujoco_msgs` 只承载仿真控制相关 `srv/msg`，不包含业务逻辑
- `robot_mujoco_ros2` 是唯一 ROS 2 适配层
- `robot_mujoco` 是产品入口包，不是底层 API 聚合层

目标架构下，`mujoco_simulation` 与 ROS 2 之间的所有接线都应通过 `robot_mujoco_ros2` 完成，`robot_mujoco` 只负责把这些能力组织成工作区级产品入口和 RoboCasa 集成流程。

## 3. 四层职责定义

### 3.1 `mujoco_simulation`

`mujoco_simulation` 的定位固定为纯 C++ MuJoCo runtime，负责：

- `Simulation`
- `Status` / `Result<T>`
- `ModelRuntime`
- `SimulationScheduler`
- `ComponentManager`
- `Viewer`
- `CameraRenderer`
- 与仿真组件、状态快照、命令缓冲相关的纯运行时能力

该层不允许出现：

- `rclcpp`
- `rosgraph_msgs`
- `sensor_msgs`
- `std_srvs`
- `hardware_interface`
- `pluginlib`

判断原则只有一个：只要逻辑本质上是在操作 MuJoCo runtime 本身，就应优先放在 `mujoco_simulation`；只要逻辑本质上是在适配 ROS 2 接口，就不应放在这里。

### 3.2 `robot_mujoco_msgs`

`robot_mujoco_msgs` 的定位固定为仿真控制接口类型包，负责承载：

- `StepSimulation.srv`
- `SetRealtimeFactor.srv`
- `ResetWorld.srv`

未来新增的仿真控制相关 `srv/msg` 也应统一收口到这个包中。

该层只负责 ROS 接口类型定义，不允许放入：

- runtime 逻辑
- ROS node / publisher / service 实现
- `ros2_control` plugin 逻辑
- 配置解析逻辑

它不应反向依赖任何上层包，也不应包含对 `mujoco_simulation` 的调用代码。

### 3.3 `robot_mujoco_ros2`

`robot_mujoco_ros2` 的定位固定为统一 ROS 2 适配层。该包合并当前：

- `mujoco_simulation_ros`
- `mujoco_hardware`

并统一承载以下职责：

- `/clock` 发布
- IMU / Camera / Lidar publisher
- simulation control services
- ROS message / service 响应适配
- ROS node / executor 生命周期管理
- `ros2_control` hardware plugin
- `pluginlib` 导出
- 对 `mujoco_simulation` 的 ROS 2、`rclcpp_lifecycle` 和 `hardware_interface` 适配

该包内部采用 `hardware_plugin` 主入口模式。也就是说：

- `hardware_plugin` 是系统主入口
- `hardware_plugin` 是 `mujoco_simulation::Simulation` 的唯一 owner
- `ros_bridge` 是由 `hardware_plugin` 组合出来的 ROS 2 通信组件
- `ros_bridge` 不是独立 runtime owner，也不是与 `hardware_plugin` 对等的系统主入口

在这个前提下，该包内部不是一个无边界的大杂烩实现，而是固定为同一包边界内的两个核心组件：

- `ros_bridge`
- `hardware_plugin`

此外，该包内部可以存在配置翻译、消息映射等支撑代码，但这些属于内部实现细节，不应上升为与 `ros_bridge`、`hardware_plugin` 并列的架构层。

这意味着目标架构下，`robot_mujoco_ros2` 必须同时包含：

- 现 `SimulationRosBridge` 对应的 ROS bridge 能力
- 现 `MuJoCoHardwareInterface` 对应的 `ros2_control` plugin 能力

### 3.3.1 `robot_mujoco_ros2` 包内结构

`robot_mujoco_ros2` 的包内结构固定为“一个主入口组件 + 一个被组合的 ROS 通信组件 + 一组内部支撑代码”。

#### `hardware_plugin`

`hardware_plugin` 负责：

- 作为系统主入口接入 `controller_manager`
- 持有并管理 `mujoco_simulation::Simulation`
- `ros2_control` 的 `SystemInterface` 实现
- state / command interface 导出
- mode switch
- `read()` / `write()` 调用链
- 安装 snapshot observer 或等价发布回调
- 创建并持有 `ros_bridge`
- 向 `ros_bridge` 注入控制服务回调和只读发布输入

它依赖：

- `mujoco_simulation`
- `ros_bridge`

它不负责：

- 自己直接创建 ROS publisher / service
- 自己重复实现 ROS message 填充逻辑
- 把 `ros_bridge` 暴露成独立 runtime owner

#### `ros_bridge`

`ros_bridge` 负责：

- `/clock`
- IMU / Camera / Lidar publisher
- simulation control services
- ROS node / executor 生命周期

它只依赖：

- `mujoco_simulation` 的公开运行时读取与控制接口
- `robot_mujoco_msgs`

它不负责：

- `ros2_control` 的 `SystemInterface` 实现
- 持有 `Simulation`
- `HardwareInfo` 解析
- 决定 runtime 生命周期
- 运行时总配置组装

#### 内部支撑代码（非架构层）

`robot_mujoco_ros2` 内部允许存在以下支撑代码，但它们不是与 `ros_bridge`、`hardware_plugin` 并列的层级：

- 配置翻译代码
  - 负责把 `HardwareInfo`、ROS 参数和 topic/frame 配置拆成 runtime 配置、bridge 配置和 plugin 映射配置
- 消息映射代码
  - 负责 `Status/Result` 到 ROS response 的适配
  - 负责 sample/snapshot 到 ROS message 的映射
- 其他仅供组装使用的局部 glue 代码

这些支撑代码可以是私有类、局部 helper 或内部头文件，但不应被定义成独立架构层，也不应被作为稳定对外入口暴露。

### 3.3.2 `robot_mujoco_ros2` 包内依赖与公开入口

`robot_mujoco_ros2` 的核心依赖方向固定如下：

```text
robot_mujoco
  -> robot_mujoco_ros2::hardware_plugin
    -> mujoco_simulation::Simulation
    -> robot_mujoco_ros2::ros_bridge
      -> robot_mujoco_msgs
      -> mujoco_simulation

robot_mujoco_ros2::ros_bridge
  -> hardware callback inputs
  -> publish/service execution
```

固定禁止以下反向依赖：

- `ros_bridge` 不能依赖 `hardware_plugin` 的 `SystemInterface` 类型
- `ros_bridge` 不能拥有 `Simulation`
- `mujoco_simulation` 不知道 `robot_mujoco_ros2` 的存在
- 配置翻译和消息映射代码不能演化成独立的对外层级或稳定入口

`robot_mujoco_ros2` 对外公开的稳定入口固定如下：

- 面向 `ros2_control` 使用者
  - `MuJoCoHardwareInterface` 的等价主类型
- 面向直接 ROS bridge 集成者
  - `SimulationRosBridge` 的等价主类型
- 面向配置构建者
  - bridge 配置类型
  - 与 hardware plugin 直接相关的必要配置入口

不作为公开稳定入口的内容固定如下：

- publisher 内部类
- 配置翻译 helper
- 消息映射 helper
- 仅供 plugin 组装使用的局部 glue 类型

目标架构下，`robot_mujoco_ros2` 对外公开的是“`hardware_plugin` 主类型 + `ros_bridge` 主类型 + 必要配置类型”，而不是所有内部支撑代码。

### 3.3.3 `robot_mujoco_ros2` 运行时数据流与线程边界

`robot_mujoco_ros2` 的内部主数据流固定为三条，同时线程模型也必须围绕“runtime 与 ROS 发布彻底解耦”来设计。

#### 控制写入链

```text
controller manager
  -> hardware_plugin::write()
    -> mujoco_simulation::set_joint_command() / set_mobile_base_command()
```

#### 状态发布链

```text
SimulationScheduler
  -> 生成 state snapshot / camera sample
    -> hardware_plugin callback
      -> enqueue PublishBundle

ros_bridge publish worker
  -> dequeue PublishBundle
    -> topic publisher
```

#### 服务控制链

```text
/start /stop /pause /resume /step /set_realtime_factor /load_keyframe /reset
  -> ros_bridge service callback
    -> hardware_plugin status callback
      -> mujoco_simulation public API
```

线程模型固定如下：

- `mujoco_simulation` worker 线程是唯一允许写主 `mjData` 的线程
- `robot_mujoco_ros2` 至少包含两个 ROS 执行面：
  - service executor 线程
    - 只处理 ROS service callback 和必要的 control-facing callback dispatch
  - topic publish worker 线程
    - 只处理 `/clock`、IMU、Camera、Lidar 等 topic 发布
- `hardware_plugin` 的 `read()` / `write()` 与通过 `ros_bridge` 转发回来的 service callback 都只能通过 runtime public API 和只读状态快照交互，不能直接访问主 `mjData`
- `ros_bridge` 的任何线程都不能直接访问 `Simulation` 内部可变状态，更不能访问主 `mjData`

`PublishBundle` 的设计约束固定如下：

- 它是只读发布载荷，来源于：
  - `SimulationStateSnapshot`
  - 必要的 camera sample 或其等价只读视图
  - 对应的仿真时间戳
- `SimulationScheduler` 或其 observer 回调只负责构造并投递 `PublishBundle`
- ROS 消息构造、图像内存拷贝、publisher 调用全部在 topic publish worker 中完成
- 物理线程禁止同步等待 ROS publish 完成

`/clock` 的线程关系固定如下：

- `/clock` 属于 `ros_bridge` 的 topic 发布面
- `/clock` 与其他 topic 一样，只消费最新仿真时间，不反向驱动 runtime
- 不要求单独定义 clock 专用架构层；可作为 publish worker 的一个固定发布职责

发布队列策略固定如下：

- runtime -> ROS 发布之间使用无阻塞或近似无阻塞队列
- 队列积压时优先允许丢弃旧发布包，而不是阻塞 `SimulationScheduler`
- 设计目标是“latest wins”，而不是保证每一帧都被 ROS 完整发布

service 执行路径固定如下：

- `ros_bridge` 只持有由 `hardware_plugin` 注入的控制回调
- service callback 在 executor 线程中执行
- 最终控制操作由 `hardware_plugin` 进入 `Simulation` public API
- `ros_bridge` 不持有 `Simulation` ownership，也不直接承担 lifecycle 决策

配置来源关系也固定如下：

- 原始输入来自 `HardwareInfo` 和 ROS 参数
- `robot_mujoco_ros2` 内部配置翻译代码负责把这些输入拆成 runtime 配置、bridge 配置和 hardware plugin 映射配置
- `ros_bridge` 不再自行推导 `HardwareInfo`
- `mujoco_simulation` 不负责理解 ROS topic 或 frame 命名

### 3.4 `robot_mujoco`

`robot_mujoco` 的目标角色固定为产品入口包，而不是严格意义上的极简元包。

它保留以下职责：

- launch 文件
- config 文件
- 默认产品级启动入口
- 工作区级集成编排
- RoboCasa 场景集成
- 面向最终使用流程的上层封装

但它不再承担以下职责：

- 作为统一底层 C++ include 入口
- 继续 re-export `mujoco_simulation` 与 `robot_mujoco_ros2` 的底层类型
- 承担 ROS adapter 与 runtime 的边界整合职责

换句话说，`robot_mujoco` 仍然可以是工作区的产品入口包，但不应继续演化成“所有底层 C++ 能力的总入口”。

## 4. 依赖规则

目标架构的依赖规则固定如下。

| 包名 | 主要职责 | 允许依赖 | 禁止内容 | 主要对外入口 |
| --- | --- | --- | --- | --- |
| `mujoco_simulation` | 纯 MuJoCo runtime | C++ 标准库、MuJoCo、渲染/线程等运行时依赖 | ROS 头、ROS message、`hardware_interface`、`pluginlib` | `mujoco_simulation::Simulation` 及相关运行时类型 |
| `robot_mujoco_msgs` | 仿真控制相关 `srv/msg` | `rosidl` 生成所需依赖 | runtime 逻辑、ROS adapter 逻辑、plugin 逻辑 | ROS `srv/msg` 类型 |
| `robot_mujoco_ros2` | `hardware_plugin` 主入口 + `ros_bridge` 通信组件，辅以内部配置/消息转换支撑代码 | `mujoco_simulation`、`robot_mujoco_msgs`、`rclcpp`、`hardware_interface`、`pluginlib` | MuJoCo runtime 核心逻辑、产品级 launch/RoboCasa 集成 | ROS node、publisher/service、hardware plugin、必要配置类型 |
| `robot_mujoco` | 产品入口、launch/config、RoboCasa 集成 | `mujoco_simulation`、`robot_mujoco_msgs`、`robot_mujoco_ros2` | 底层 C++ 聚合头继续扩张、ROS adapter 核心实现、runtime 核心实现 | launch、config、RoboCasa 工作流、产品入口 |

更具体的依赖约束如下：

- `robot_mujoco_ros2 -> mujoco_simulation`
- `robot_mujoco_ros2 -> robot_mujoco_msgs`
- `robot_mujoco_msgs` 不反向依赖任何上层
- `robot_mujoco` 可以依赖前三层
- 前三层不能依赖 `robot_mujoco`

同时固定以下禁止项：

- `mujoco_simulation` 禁止包含 ROS 头
- `robot_mujoco_msgs` 禁止放适配逻辑
- `robot_mujoco` 禁止继续作为底层 C++ 聚合头入口演化

## 5. 现状到目标的映射

现有 5 个相关包到目标 4 层的映射关系固定如下：

```text
当前结构
  mujoco_simulation
  mujoco_simulation_ros
  mujoco_hardware
  mujoco_ros2_bridge_msgs
  robot_mujoco

目标结构
  mujoco_simulation
  robot_mujoco_ros2
  robot_mujoco_msgs
  robot_mujoco
```

具体映射表如下：

| 当前包 | 目标归属 | 迁移说明 |
| --- | --- | --- |
| `mujoco_simulation` | `mujoco_simulation` | 保持为纯 runtime，不引入 ROS 依赖 |
| `mujoco_ros2_bridge_msgs` | `robot_mujoco_msgs` | 统一命名为工作区级仿真接口类型包 |
| `mujoco_simulation_ros` | `robot_mujoco_ros2` | 并入统一 ROS 2 adapter 层 |
| `mujoco_hardware` | `robot_mujoco_ros2` | 并入统一 ROS 2 adapter 层 |
| `robot_mujoco` | `robot_mujoco` | 保留为产品入口包，但退出底层 C++ 聚合中心地位 |

该映射同时说明一个关键结论：目标架构不是把 runtime 与 ROS 更紧地揉在一起，而是把所有 ROS 2 适配职责集中到单一包边界中，再由 `robot_mujoco` 负责产品级组织。

## 6. 命名迁移与公开入口

目标命名固定如下：

- `mujoco_simulation`
- `robot_mujoco_msgs`
- `robot_mujoco_ros2`
- `robot_mujoco`

其中，重命名映射固定为：

- `mujoco_ros2_bridge_msgs -> robot_mujoco_msgs`
- `mujoco_simulation_ros + mujoco_hardware -> robot_mujoco_ros2`

下游推荐入口也固定如下：

- runtime 开发者：直接依赖 `mujoco_simulation`
- ROS 2 集成开发者：依赖 `robot_mujoco_ros2`
- ROS 接口调用者：依赖 `robot_mujoco_msgs`
- 产品启动与 RoboCasa 使用者：依赖 `robot_mujoco`

当前的 `robot_mujoco.hpp` 应被视为现状遗留的聚合入口。目标架构下，它不再是底层 C++ 能力的中心入口，后续应逐步退出这一角色，避免继续扩大顶层包职责。

## 7. 迁移阶段

迁移按以下阶段进行，阶段顺序固定，不留开放决策。

### 阶段 0：新增目标文档并冻结命名

目标：

- 新增本文档
- 固定目标四层命名
- 固定 `robot_mujoco` 的目标角色为产品入口包

完成标志：

- 目标命名不再继续讨论
- 后续实施任务默认以本文为准

### 阶段 1：先迁移消息包到 `robot_mujoco_msgs`

目标：

- 将 `mujoco_ros2_bridge_msgs` 重命名为 `robot_mujoco_msgs`
- 统一更新服务类型引用与构建依赖

完成标志：

- 仿真控制相关 `srv/msg` 已全部由 `robot_mujoco_msgs` 提供
- 下游推荐 `find_package` 与 include 已更新到新包名
- 旧命名如需保留，只剩兼容壳或过渡说明

### 阶段 2：新建 `robot_mujoco_ros2`，先迁入 `SimulationRosBridge`

目标：

- 建立统一 ROS 2 adapter 包
- 先迁入当前 `mujoco_simulation_ros` 中的 bridge 能力

完成标志：

- `robot_mujoco_ros2` 已形成 `ros_bridge` 组件和必要内部支撑代码的边界
- `/clock`、sensor publisher、simulation control services 已由 `robot_mujoco_ros2` 提供
- `SimulationRosBridge` 或其等价替代实现已位于新包中
- 旧 `mujoco_simulation_ros` 如需保留，只剩兼容壳或转发层

### 阶段 3：迁入 `MuJoCoHardwareInterface` 与 plugin 导出

目标：

- 将 `mujoco_hardware` 中的 `ros2_control` 适配能力并入 `robot_mujoco_ros2`
- 保持 ROS bridge 与 hardware plugin 同属一个包边界

完成标志：

- `MuJoCoHardwareInterface` 或其等价替代实现位于 `robot_mujoco_ros2`
- 原 `mujoco_hardware` 的配置解析职责已并入 `robot_mujoco_ros2` 内部支撑代码
- `hardware_plugin` 已固定为系统主入口和 `Simulation` 唯一 owner
- `ros_bridge` 已固定为由 `hardware_plugin` 组合的通信组件，而不是并列主入口
- `pluginlib` 导出迁移完成
- 旧 `mujoco_hardware` 如需保留，只剩兼容壳或被删除

### 阶段 4：更新 `robot_mujoco` 为产品入口包

目标：

- 保留 launch/config/RoboCasa 集成
- 收缩 `robot_mujoco` 的底层 C++ 聚合定位

完成标志：

- `robot_mujoco` 不再作为下游默认底层 C++ include 入口推荐
- `robot_mujoco.hpp` 退出中心地位或转为过渡入口
- `robot_mujoco` 的文档定位明确为产品入口与集成包

### 阶段 5：统一更新 build/test/launch/docs/CI

目标：

- 同步更新构建目标、测试目标、launch 依赖、CI 包选择和文档导航

完成标志：

- build/test 命令中的包名已切换到目标结构
- launch/config 引用已切换到目标结构
- `current_architecture.md`、`migration_guide.md` 等文档已同步更新关联关系
- CI 与脚本不再依赖旧包名作为主路径
- 测试已按 bridge、plugin 和必要内部支撑逻辑组织
- 文档和构建入口不再以 `mujoco_simulation_ros` / `mujoco_hardware` 为主路径

## 8. 兼容与文档策略

迁移过程中，短期允许以下兼容措施：

- 过渡别名
- 转发 include
- 迁移说明
- 兼容壳包

但长期目标同样固定：

- 不长期维持双命名并存
- 不把 ROS 依赖重新引回 `mujoco_simulation`
- 不让 `robot_mujoco` 重新膨胀为底层 API 聚合中心

文档职责关系也固定如下：

- [`current_architecture.md`](./current_architecture.md)
  - 描述当前真实代码边界、调用链和验证状态
- [`migration_guide.md`](./migration_guide.md)
  - 面向下游使用者的迁移入口
- `target_package_architecture.md`
  - 描述目标四层包架构、依赖规则和实施路线

## 9. 实施验收标准

后续实施应以以下标准作为验收基线：

- 相关 5 个现有包都已映射到目标 4 层，不存在未归属模块
- `robot_mujoco_ros2` 明确同时包含 ROS bridge 和 `ros2_control` plugin
- `robot_mujoco_ros2` 已明确采用 `hardware_plugin` 主入口模式
- `robot_mujoco_ros2` 的两个核心组件及其主从关系明确固定
- `robot_mujoco` 明确是产品入口包，而不是严格极简元包
- `robot_mujoco` 保留 RoboCasa 集成，但退出底层 C++ 聚合入口定位
- 依赖方向始终满足“runtime 不依赖 ROS”
- 迁移阶段可以直接拆解为工程任务，不需要再补充命名或边界决策

## 10. 默认假设

本文基于以下默认假设成立：

- 当前只规划目标文档，不同时修改代码
- `robot_mujoco_ros2` 的核心结构固定为两个组件：
  - `ros_bridge`
  - `hardware_plugin`
- 配置翻译和消息映射属于内部支撑代码，不被提升为架构层
- `hardware_plugin` 是系统主入口，也是 `Simulation` 唯一 owner
- 当前不需要在文档里给出精确目录树或文件名草案，只固定核心组件和依赖边界
- `robot_mujoco` 的目标角色固定为产品入口包
- 目标命名固定采用：
  - `mujoco_simulation`
  - `robot_mujoco_msgs`
  - `robot_mujoco_ros2`
  - `robot_mujoco`

如果后续代码实现与本文发生偏离，应优先修正实现或更新本文，而不是在局部路径上重新引入新的包层级定义。
