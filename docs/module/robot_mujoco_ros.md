# robot_mujoco_ros2 模块设计与迁移开发计划

本文描述目标 ROS 2 adapter 模块的详细设计和迁移开发计划。

目标包名以 [`target_package_architecture.md`](../target_package_architecture.md) 为准，固定为
`robot_mujoco_ros2`。本文文件名沿用 `robot_mujoco_ros.md`，仅作为模块设计文档入口，不代表目标
包名回退为 `robot_mujoco_ros`。

当前真实代码边界、主调用链和验证状态仍以 [`current_architecture.md`](../current_architecture.md)
为准。本文是目标模块设计文档，不代表代码已经完成迁移。

## 1. 模块定位

`robot_mujoco_ros2` 是目标四层架构中的统一 ROS 2 adapter 层，位于：

```text
robot_mujoco
  -> robot_mujoco_ros2
    -> robot_mujoco_msgs
    -> mujoco_simulation
```

该模块的职责不是再实现一套 MuJoCo runtime，而是把：

- `mujoco_simulation` 的纯运行时能力
- `robot_mujoco_msgs` 的 ROS 接口类型
- `rclcpp` / `ros2_control` / `pluginlib` 的 ROS 2 基础设施

收敛成一个清晰、单向依赖、可测试的 ROS 2 适配边界。

它统一替代当前两类包职责：

- `mujoco_simulation_ros`
- `mujoco_hardware`

模块设计的中心原则固定如下：

- `hardware_plugin` 是系统主入口
- `hardware_plugin` 是 `mujoco_simulation::Simulation` 的唯一 owner
- `ros_bridge` 是被 `hardware_plugin` 组合出来的 ROS 通信组件
- runtime 与 ROS 发布线程严格隔离
- `mujoco_simulation` 永远不知道 ROS adapter 的存在

## 2. 目标职责边界

`robot_mujoco_ros2` 统一负责以下能力：

- `ros2_control` `SystemInterface` 主入口
- `Simulation` 生命周期管理
- `/clock` 发布
- IMU / Camera / Lidar topic 发布
- 仿真控制服务
- `Status/Result` 到 ROS response 的适配
- `HardwareInfo`、ROS 参数到运行时配置的翻译
- `pluginlib` 导出和 ROS node / executor 生命周期管理

以下内容不属于该模块：

- MuJoCo 物理运行时核心逻辑
- `SimulationScheduler`、`ComponentManager`、`Viewer` 等 runtime 内核
- 纯 `srv/msg` 定义
- launch、产品级 config、RoboCasa 集成

因此边界固定为：

- 运行时内核在 `mujoco_simulation`
- 接口类型在 `robot_mujoco_msgs`
- ROS 2 adapter 在 `robot_mujoco_ros2`
- 产品入口在 `robot_mujoco`

### 2.1 规则分层

本文后续出现的大量“固定规则”并不意味着所有内容都是同等强度的长期硬约束。为了避免文档演化成
“完全封闭的运行时规范系统”，`robot_mujoco_ros2` 的规则固定分为三层：

- 核心不变量
  - 违反后会破坏包边界、线程安全、生命周期一致性或下游接口语义
- 默认策略
  - 当前推荐实现方式，未来允许在保持兼容的前提下演进
- 可替换实现点
  - 只要满足核心不变量和公开契约，就允许替换具体实现

在本文中，默认按以下方式理解：

- 核心不变量
  - `hardware_plugin` 是唯一系统主入口
  - `Simulation` ownership 固定在 `hardware_plugin`
  - runtime 不依赖 ROS
  - `read()` / `write()` 的 RT 契约
  - 统一状态机
  - 同一 `PublishBundle` 的时间一致性
- 默认策略
  - `MultiThreadedExecutor + callback groups`
  - `latest wins`
  - 当前 QoS 基线
  - 当前 `position / velocity / effort` 默认语义
  - 当前 rate mismatch 处理方式
- 可替换实现点
  - 队列具体实现是 `SPSC ring buffer` 还是等价结构
  - 是否使用 `realtime_tools` 还是等价 RT 设施
  - mapper / adapter 的内部文件组织方式
  - QoS 默认值的覆盖机制

## 3. 核心设计

### 3.1 主入口模式

`robot_mujoco_ros2` 采用 `hardware_plugin` 主入口模式，而不是 `ros_bridge` 主入口模式。

固定调用链如下：

```text
controller_manager
  -> hardware_plugin
    -> Simulation
    -> ros_bridge
```

这样设计的原因是：

- 系统最终是以 `ros2_control` plugin 方式接入
- `read()` / `write()` 与 mode switch 必须围绕同一个 runtime owner 组织
- 控制服务与 topic 发布只能是 runtime 的附属接口面，不能反向主导 runtime 生命周期

因此 `ros_bridge` 的角色固定为：

- 提供 ROS service / topic 接口
- 持有 ROS node、publisher、service、executor
- 消费只读状态快照和控制回调

而不是：

- 拥有 `Simulation`
- 决定系统生命周期
- 直接管理 `HardwareInfo`

### 3.2 两个核心组件

`robot_mujoco_ros2` 的核心结构固定为两个组件：

- `hardware_plugin`
- `ros_bridge`

模块内部可以存在配置翻译、消息映射、局部 glue 代码，但这些是内部支撑实现，不上升为与这两个
组件并列的架构层。

不过，这些内部支撑实现不能继续散落在各个 publisher、service callback 和 plugin glue 中，而应
收敛为几组固定的包内标准子模块：

- control interface adapter
- message mapper
- config builder / schema / validation
- queue / publish bundle glue

#### `hardware_plugin`

`hardware_plugin` 负责：

- 接入 `controller_manager`
- 创建、初始化、启动、关闭 `Simulation`
- 实现 `SystemInterface`
- 导出 state / command interface
- 处理 mode switch
- 管理 runtime 读写链路
- 安装状态快照或 sample 观察回调
- 创建并持有 `ros_bridge`
- 向 `ros_bridge` 注入控制回调和发布输入

它不负责：

- 自己直接实现 topic publisher
- 自己直接填充 ROS message
- 自己运行独立 ROS executor 逻辑以替代 `ros_bridge`

#### `ros_bridge`

`ros_bridge` 负责：

- `/clock` publisher
- IMU / Camera / Lidar publisher
- `/start`、`/stop`、`/pause`、`/resume`
- `/step`、`/set_realtime_factor`、`/load_keyframe`、`/reset`
- ROS node 生命周期
- callback group、executor 和 publish worker 管理

它只通过两类输入工作：

- 来自 `hardware_plugin` 注入的控制回调
- 来自 `hardware_plugin` 或 runtime observer 投递的只读发布载荷

它不负责：

- 持有 `Simulation`
- 解析 `HardwareInfo`
- 直接操作主 `mjData`
- 定义 runtime 生命周期策略

### 3.3 依赖方向

固定依赖关系如下：

```text
robot_mujoco
  -> robot_mujoco_ros2
    -> robot_mujoco_msgs
    -> mujoco_simulation
```

模块内主依赖关系如下：

```text
hardware_plugin
  -> mujoco_simulation
  -> ros_bridge

ros_bridge
  -> robot_mujoco_msgs
  -> rclcpp
  -> injected callbacks / publish bundle
```

固定禁止项如下：

- `ros_bridge` 不能依赖 `hardware_plugin` 的 `SystemInterface` 类型
- `ros_bridge` 不能持有 `Simulation` ownership
- `hardware_plugin` 不能直接承担 ROS message 组装职责
- `mujoco_simulation` 不能包含 ROS 头
- `robot_mujoco` 不能重新吸收该模块内部实现
- `robot_mujoco_ros2` 内的任何实现代码都不能依赖 `robot_mujoco` 的头文件、产品级配置类型、
  launch 资源路径解析逻辑或 RoboCasa 集成代码
- `ros_bridge` 与 `hardware_plugin` 只能面向 `robot_mujoco_ros2`、`robot_mujoco_msgs`、
  `mujoco_simulation` 和 ROS 2 基础设施编程，不能通过任何“临时便利接口”反向耦合产品入口包

## 4. 生命周期设计

### 4.1 owner 关系

owner 关系固定如下：

- `hardware_plugin` 持有 `Simulation`
- `hardware_plugin` 持有 `ros_bridge`
- `ros_bridge` 只持有 node、publisher、service、executor、publish queue

这意味着系统初始化顺序固定为：

```text
hardware_plugin::on_init()
  -> 解析 HardwareInfo / ROS 参数
  -> 构造 runtime config
  -> 构造 Simulation
  -> 构造 ros_bridge
  -> 注入 callbacks / publish settings
```

系统激活顺序固定为：

```text
hardware_plugin::on_activate()
  -> initialize/start Simulation
  -> 启动 ros_bridge executor / publish worker
```

系统停止顺序固定为：

```text
hardware_plugin::on_deactivate() / on_shutdown()
  -> 停止 ros_bridge executor / publish worker
  -> stop/shutdown Simulation
```

设计目标是先停 ROS 面，再停 runtime，避免 ROS callback 在 runtime 已销毁后仍访问旧回调。

### 4.2 失败处理原则

失败处理原则固定如下：

- runtime 初始化失败：整体初始化失败，`ros_bridge` 不应独立存活
- `ros_bridge` 初始化失败：整体初始化失败，不能留下半激活 runtime
- service 执行失败：返回 `Status.message()` 映射后的 ROS response，不影响 owner 关系
- publish 队列积压：允许丢弃旧数据，不阻塞 runtime
- plugin 相关资源初始化必须在 `on_init()` 或等价初始化阶段完成，不允许在 `read()`、`write()`、
  service callback 或 publish 热路径做 lazy initialization

### 4.3 统一状态机

当前设计不能只停留在“owner 关系正确”，还必须固定单一系统状态机，避免 ROS 面和 runtime 面各自演化
出一套隐式 lifecycle。

系统状态的单一真源固定在 `hardware_plugin`，`ros_bridge` 不维护独立状态机，只读取并遵守
`hardware_plugin` 暴露的当前系统状态。

推荐的系统状态集合固定如下：

- `Unconfigured`
- `Inactive`
- `Activating`
- `Active`
- `Deactivating`
- `Error`
- `Shutdown`

状态职责固定如下：

- `Unconfigured`
  - 仅允许配置解析和对象构造
- `Inactive`
  - `Simulation` 尚未进入正常运行
  - ROS callback 可存在，但所有会改变 runtime 的 service 必须返回 not active / rejected
- `Activating`
  - 禁止新的控制类 service 进入 runtime
  - publish worker 不发布业务数据
- `Active`
  - `Simulation`、ROS callback、发布链全部正常工作
- `Deactivating`
  - 停止接收新的控制类 service
  - 允许清空或丢弃未消费的发布包
- `Error`
  - 只保留最小诊断与受控停机能力
- `Shutdown`
  - ROS executor、publish worker、runtime 均已停止

回调可用性规则固定如下：

- `ros_bridge` 在 dispatch service callback 前必须检查系统状态
- `Inactive / Activating / Deactivating / Error / Shutdown` 下，不允许新的 runtime 改写操作进入
  `Simulation`
- ROS shutdown 不能早于系统状态切换到 `Deactivating` 或 `Shutdown`
- runtime stop 完成前，必须先让 ROS callback 脱离可调度状态

这意味着文档层面必须承认：系统只有一个 lifecycle manager，即 `hardware_plugin`；`ros_bridge`
只是状态受控的接口面。

## 5. 线程模型

`robot_mujoco_ros2` 的线程设计必须围绕一条硬约束：

- `mujoco_simulation` worker 线程是唯一允许写主 `mjData` 的线程

在此基础上，`robot_mujoco_ros2` 固定包含三个执行面：

- runtime worker 线程
  - 由 `SimulationScheduler` 驱动
  - 负责 command flush、physics step、sensor sample、state publish
- ROS callback executor
  - 由 `ros_bridge` 持有
  - 采用 `MultiThreadedExecutor + callback groups`
  - 负责 service、timer、参数等 ROS callback 调度
- topic publish worker 线程
  - 由 `ros_bridge` 持有
  - 只负责消费 runtime 出站发布队列并完成消息构造与发布

线程关系如下：

```text
SimulationScheduler worker
  -> sample sensors / publish snapshot
  -> enqueue PublishBundle

ROS callback executor
  -> service/timer callback
  -> hardware_plugin callback
  -> Simulation public API

topic publish worker
  -> dequeue PublishBundle
  -> build ROS messages
  -> publish /clock and sensor topics
```

固定线程约束如下：

- `read()` / `write()` 不能直接访问主 `mjData`
- service callback 不能直接访问主 `mjData`
- publish worker 不能直接访问主 `mjData`
- 所有 ROS 线程只能通过 runtime public API 或只读快照与仿真交互
- `ros_bridge` 不单独管理第二套 ROS callback 调度体系；只有一个 ROS callback executor，publish
  worker 只是 runtime 出站队列消费者，不承担 ROS callback 调度职责
- topic publish worker 属于 user thread，不属于 ROS callback 调度中心
- topic publish worker 不允许 `spin()` executor，不允许等待 ROS future，不允许承载 callback group
  语义

### 5.1 并发契约

当前线程模型不仅要“逻辑分离”，还必须固定并发契约，否则实现阶段很容易退化成带锁的大杂烩。

固定约束如下：

- runtime 热路径到 ROS 发布面的传递必须使用有界队列或等价的有界最新值通道
- producer 侧不能因队列满而阻塞 `SimulationScheduler`
- consumer 落后时允许丢弃旧发布包，遵循 `latest wins`
- `PublishBundle` 必须是预定义结构，不能在热路径动态扩容
- service 控制链不要求强制 lock-free，但不能把锁带入 runtime 热路径

推荐但不强制的实现形态如下：

- 单 producer、单 consumer 的场景优先使用 `SPSC ring buffer`
- 只需保留最新状态的场景可使用 `atomic latest-snapshot`
- 需要跨线程共享大对象时，优先使用只读句柄或预分配槽位，而不是运行时 copy-on-write 扩容

默认发布队列策略进一步固定如下：

- 小载荷发布面
  - 例如 `/clock`、joint state、IMU、低维状态快照
  - 默认使用 latest-snapshot 或等价单槽/双槽覆盖策略
  - 目标是最小化排队延迟，而不是保留完整历史
- 大载荷发布面
  - 例如 camera、lidar 或其他高带宽 sample
  - 默认使用预分配的有界 `SPSC ring buffer`
  - 队列槽位中只放只读句柄、索引或预分配帧缓冲引用，不在热路径新分配大对象
- service 控制链
  - 不经过这条发布队列
  - 仍通过 callback executor 与 runtime public API 交互

这意味着实现阶段不应再把所有 topic 统一塞进同一种队列抽象里。默认上：

- 小载荷优先 latest-snapshot
- 大载荷优先 bounded SPSC ring
- 只有在明确证明收益时，才引入第三种发布通道

这里不把具体实现写死为唯一方案，但必须把“有界、非阻塞、热路径无堆分配”写成不可退让的约束。

### 5.2 Real-Time 契约

`ros2_control` 接入后的热路径主要是 `read()` 和 `write()`。文档必须明确以下约束：

- `read()` 必须 zero allocation
- `write()` 必须 zero allocation
- `write()` 必须 zero blocking
- `read()` 不允许等待 ROS publish、service 或 executor
- `write()` 不允许等待 physics step 完成
- 热路径中不允许 `std::string` 构造或增长
- 热路径中不允许 `std::vector`、`std::unordered_map` 等容器动态扩容
- state / command interface 所需缓存必须在初始化阶段预分配完成

这意味着：

- interface handle 索引表在 `on_init()` 完成构建
- topic 名称、frame 名称、joint 名称等字符串在非热路径完成归一化
- runtime 命令对象和状态拷贝缓冲在激活前完成预分配

## 6. 数据流设计

### 6.1 控制写入链

```text
controller command
  -> hardware_plugin::write()
    -> Simulation::set_joint_command() / set_mobile_base_command()
      -> CommandBuffer
      -> SimulationScheduler worker flush
```

控制写入链只负责提交命令，不负责同步等待 physics 结果。

### 6.2 状态读取与发布链

```text
SimulationScheduler worker
  -> 生成 SimulationStateSnapshot / sensor sample
  -> hardware_plugin observer
  -> enqueue PublishBundle

topic publish worker
  -> dequeue PublishBundle
  -> map to ROS message
  -> publish
```

这里的 `PublishBundle` 约束固定如下：

- 只包含只读数据
- 至少包含仿真时间戳
- 可包含状态快照、IMU sample、Lidar sample、Camera sample 或其只读视图
- 不持有可写 runtime 状态
- 生命周期和内存占用必须可预估
- 构造过程不能在 runtime 热路径触发动态增长

发布策略固定如下：

- `latest wins`
- 允许丢帧
- 禁止 runtime 同步等待 ROS publisher

### 6.2.1 时间一致性模型

仅仅定义 `snapshot`、`PublishBundle` 和 `latest wins` 还不够，文档还必须固定时间一致性策略。

默认策略固定如下：

- 单个 `PublishBundle` 必须对应一个明确的 simulation tick
- `PublishBundle` 内携带的仿真时间戳是该 bundle 的唯一时间基准
- `/clock`、joint state、IMU、lidar 如果来自同一 bundle，则必须共享同一仿真时间戳
- 不能把不同 tick 的 joint state、IMU 和 `/clock` 混装成一个逻辑发布包

对慢速或异步传感器，规则固定如下：

- camera 等高成本传感器允许降频或丢帧
- 降频后的 sample 仍必须携带其真实采样 tick 的时间戳
- 不允许为了“追平 topic 频率”而伪造当前 tick 时间

因此，系统对时间一致性的承诺应固定为：

- 同一 `PublishBundle` 内的数据是 frame-consistent 的
- 不同 topic 允许因降频而跨 tick，但其 header stamp 必须反映真实采样时间
- `/clock` 发布的是当前最新 runtime 时间，不回填历史传感器时间

这样可以避免：

- TF 时间错配
- 传感器与状态时间混乱
- 下游 Nav2、MoveIt、SLAM 对时间域的错误假设

### 6.2.2 延迟语义

时间一致性并不等于零延迟。文档还必须固定“时间戳表示什么”和“发布延迟如何解释”。

默认延迟语义固定如下：

- ROS message 的 `header.stamp` 表示 simulated acquisition time，而不是 publish arrival time
- topic publish worker 可以晚于采样时刻发布消息，但不能改写采样时间戳
- 下游如果观察到消息晚到，应将其理解为 publish latency，而不是采样时间漂移

这意味着：

- IMU 的 `header.stamp` 对应 IMU sample 的真实仿真采样时刻
- lidar 的 `header.stamp` 对应 lidar sample 的真实仿真采样时刻
- camera 的 `header.stamp` 对应 camera frame 的真实仿真采样时刻，即使图像编码或拷贝更慢

默认不强制为每类传感器引入完整延迟模型，但允许通过 ROS 接口配置扩展以下字段：

- `fixed_latency`
- `max_publish_delay`
- `jitter_budget`

如果后续系统需要更强的 SLAM / 感知时间建模能力，应在不破坏本文时间一致性规则的前提下，沿着这组
扩展字段演进，而不是改写 `header.stamp` 的语义。

### 6.3 服务控制链

```text
/start /stop /pause /resume /step /set_realtime_factor /load_keyframe /reset
  -> ros_bridge service callback
  -> hardware_plugin injected callback
  -> Simulation public API
  -> Status/Result -> ROS response
```

服务控制链的关键点是：

- `ros_bridge` 只负责接 ROS service
- `hardware_plugin` 负责把 service 意图翻译成 runtime 操作
- runtime 只暴露 `Status` / `Result<T>` 风格公共 API

### 6.4 `/clock` 设计

`/clock` 固定属于 `ros_bridge` 的发布面，不单独定义额外架构层。

其约束如下：

- 只消费最新仿真时间
- 与 sensor topic 使用同一 publish worker 机制即可
- 不反向驱动 runtime
- 队列积压时遵循 `latest wins`

### 6.5 QoS 基线

该模块应固定一组默认 QoS 基线，避免后续实现阶段各自随意选择：

- `/clock`
  - `KEEP_LAST(1)`
  - `BEST_EFFORT`
  - 不使用 `transient_local`
- IMU / lidar
  - `KEEP_LAST(1)`
  - 默认 `BEST_EFFORT`
- camera
  - `KEEP_LAST(1)`
  - 默认 `BEST_EFFORT`
- 低频状态类 topic
  - `KEEP_LAST(1)`
  - 默认 `RELIABLE`

如果下游场景确实需要更严格 QoS，应通过 ROS 接口配置显式覆盖，而不是在实现中写死多个分散默认值。

## 7. 配置设计

`robot_mujoco_ros2` 需要处理两类原始输入：

- `HardwareInfo`
- ROS 参数

但这些原始输入不应直接扩散到整个模块内部。模块内部配置翻译代码需要把它们固定拆成三类配置产物：

- runtime 配置
  - 最终进入 `mujoco_simulation::SimulationConfig`
- ROS 接口配置
  - node 名称、发布开关、topic 名称、frame id、服务开关
- hardware 映射配置
  - joint command/state interface
  - sensor state interface
  - mode switch 所需元数据

这里明确使用“ROS 接口配置”而不是“bridge 配置”作为公开语言，避免把内部组件概念暴露成用户配置心智模型。

配置边界固定如下：

- `hardware_plugin` 负责调用配置翻译逻辑
- `ros_bridge` 只接收整理后的 ROS 接口配置
- `Simulation` 只接收纯 runtime 配置
- runtime 不理解 topic 名称或 frame 命名

### 7.1 配置 schema 与 builder

当前文档不能只停留在“来源于 `HardwareInfo` 和 ROS 参数”，还必须固定配置处理流程。

建议在 `robot_mujoco_ros2` 包内收敛为统一的配置构建入口：

```text
ConfigBuilder
  -> validate()
  -> normalize()
  -> build_runtime_config()
  -> build_ros_interface_config()
  -> build_hardware_mapping_config()
```

对应的逻辑职责固定如下：

- `validate()`
  - 校验必填项、重复 joint、非法接口组合、缺失 frame/topic 命名
- `normalize()`
  - 填充默认值、归一化命名、展开兼容别名
- `build_runtime_config()`
  - 生成 `mujoco_simulation::SimulationConfig`
- `build_ros_interface_config()`
  - 生成 node/QoS/topic/frame/service 配置
- `build_hardware_mapping_config()`
  - 生成 state / command interface 映射与 mode switch 元数据

同时固定以下规则：

- schema 校验失败必须在初始化阶段失败，不允许延后到 `read()` / `write()`
- 配置版本号或 schema 版本必须可扩展，避免未来新增字段时出现静默兼容
- `ros_bridge` 不能自己推断配置缺省逻辑

### 7.2 ros2_control 接口映射

当前“hardware 映射配置”还不够具体，文档应进一步固定一层标准化适配逻辑：

```text
ControlInterfaceAdapter
  - JointStateAdapter
  - JointCommandAdapter
  - BaseCommandAdapter
```

其职责如下：

- `JointStateAdapter`
  - 定义 runtime joint state 到 ROS 2 state interface 的映射
- `JointCommandAdapter`
  - 定义 `position / velocity / effort` 等命令接口到 runtime command 的映射
  - 负责 mode switch 后的命令写入策略
- `BaseCommandAdapter`
  - 定义 mobile base command/state 与 runtime base component 的映射

这组 adapter 属于 `robot_mujoco_ros2` 包内标准子模块，不是新的架构层，但必须固定存在，避免未来
MoveIt、Nav2 或自定义 controller 接入时每次重写映射逻辑。

### 7.3 Controller 兼容契约

`ros2_control` 集成不能只停留在“有 adapter”，还必须固定控制语义，否则下游 controller 会得到
不稳定甚至互相矛盾的行为。

至少应固定以下契约：

- `position` command 语义
  - 是目标位置、目标步进，还是直接写入 runtime 位置命令
- `velocity` command 语义
  - 是否经过限幅、是否允许符号翻转、是否支持 base/joint 独立策略
- `effort` command 语义
  - 是否直接映射为力矩/力控制，还是经过安全裁剪
- mode switch 语义
  - 切换后旧命令是否清零、保持，还是按新模式重新解释

默认建议固定如下：

- `position` 模式
  - 以“目标位置命令”语义进入 runtime
- `velocity` 模式
  - 进入 runtime 前执行显式限幅
- `effort` 模式
  - 进入 runtime 前执行显式安全裁剪
- mode switch
  - 切换后旧模式残留命令不得继续隐式生效

这些语义必须写成文档承诺，而不是留给每个 controller 接入时自行猜测。

### 7.4 ros2_control 时序模型

`ros2_control` 的实际执行模型是：

```text
controller update loop
  -> read()
  -> controller update()
  -> write()
```

而 `Simulation` 自身可能运行在不同步频上，因此文档必须固定 rate mismatch 策略。

固定规则如下：

- controller update rate 与 simulation step rate 允许不相等
- `write()` 只提交“最近一次 controller 输出”的命令快照，不等待当前控制周期内立即生效
- `read()` 返回“最近一次已提交的 runtime 状态快照”，而不是强制与本次 `write()` 同步配对
- runtime 可比 controller 更快，也可更慢，但两者之间必须通过明确定义的快照/命令缓冲解耦

推荐的默认策略如下：

- controller 比 simulation 快时
  - 后写入命令覆盖前写入命令，遵循 latest wins
- simulation 比 controller 快时
  - runtime 在多个仿真步中复用最近一次有效命令

这保证了：

- `read()/write()` 保持 RT 友好
- 不因 rate mismatch 引入隐式阻塞
- controller 行为可预测

### 7.4.1 Controller Feedback Closure 契约

除了控制语义和时序模型，还必须固定 feedback closure 语义，否则 controller 很容易对反馈延迟做出
错误假设。

默认契约固定如下：

- `write()` 提交命令后，不保证同一 controller cycle 内的后续 `read()` 立即观察到命令效果
- controller 在一次 `read() -> update() -> write()` 周期中，`read()` 看到的是此前已经提交完成的
  runtime 状态快照
- `write()` 提交的命令只有在后续 runtime commit tick 完成后，才可能反映到新的状态快照中

因此，系统默认不承诺 same-cycle feedback closure，而是承诺：

- 命令提交有确定的缓冲边界
- 状态反馈有确定的快照边界
- feedback delay 至少跨过一个 runtime commit 与一次后续有效 `read()`

这对下游 controller 的意义固定如下：

- PID / impedance / trajectory controller 必须把本系统视为“有界延迟闭环”
- MoveIt / 自定义控制器不能假设 `write()` 之后立即读回新状态
- 如果未来要支持更强的反馈时延保证，应以“bounded feedback delay”形式增强，而不是破坏当前快照/
  命令解耦模型

### 7.5 推荐实现设施

文档不强制绑定唯一实现库，但可以固定推荐优先级，降低后续实现分歧：

- 最新命令缓冲优先考虑 `realtime_tools::RealtimeBuffer` 或等价实现
- 高频发布路径如需 RT 友好发布，可评估 `realtime_tools::RealtimePublisher`
- runtime 到 ROS 发布面的队列优先考虑 SPSC ring buffer 或等价有界结构

这里的关键不是必须使用某个库，而是任何替代实现都必须满足本文前面已经固定的 RT 和并发契约。

## 8. 消息映射与公开入口设计

### 8.1 统一消息映射入口

消息映射不应继续散落在 `ros_bridge`、publisher helper 和 service callback 中，而应收敛为统一入口：

```text
MessageMapper
  - ImuMapper
  - CameraMapper
  - JointStateMapper
  - LidarMapper
  - StatusResponseMapper
```

其职责固定如下：

- sample / snapshot 到 ROS message 的字段映射
- `Status/Result` 到 service response 的映射
- header stamp / frame id 的统一填充规则

固定禁止项如下：

- `hardware_plugin` 不直接拼装 ROS message
- `ros_bridge` 不在多个 callback 中复制散落的字段映射逻辑
- 不允许同一种消息在多个位置维护不同映射规则

### 8.2 公开入口设计

该模块对外公开的稳定入口固定为：

- `hardware_plugin` 主类型
  - 面向 `ros2_control` 使用者
- `ros_bridge` 主类型
  - 面向需要独立嵌入 bridge 的开发者
- 必要配置类型
  - 包括 ROS 接口配置和与 hardware plugin 直接相关的公开配置

不作为稳定公开入口的内容如下：

- publisher 内部类
- service glue helper
- 消息映射 helper
- 配置翻译 helper
- 局部线程同步工具

换句话说，对外公开的是“主类型 + 必要配置”，而不是整个实现细节面。

### 8.3 component node 兼容策略

考虑 ROS 2 工程实践，`ros_bridge` 可以额外提供 component wrapper，以便：

- 支持 composable deployment
- 便于与其他 ROS 2 系统做进程内组合
- 为后续 intra-process 优化预留空间

但这不是主设计前提。主设计仍然固定为：

- `hardware_plugin` 是系统主入口
- `ros_bridge` 是可嵌入的 node-backed 组件
- component wrapper 只是可选包装层，不改变 owner 关系

## 9. 测试设计

迁移完成后，测试组织建议固定为三类：

- `ros_bridge` 测试
  - 验证 `/clock`、topic、service 行为
- `hardware_plugin` 测试
  - 验证 `read()` / `write()`、接口导出、mode switch、runtime owner 行为
- 集成链路测试
  - 验证 `controller_manager -> hardware_plugin -> Simulation -> ros_bridge` 主链

测试重点固定为：

- service callback 不直接访问主 `mjData`
- publish queue 积压不会阻塞 runtime
- `/clock` 和传感器 topic 使用只读发布载荷
- runtime 停止顺序不会留下悬空 ROS callback
- ROS 接口配置与 runtime 配置边界清晰
- `read()` / `write()` 满足 zero allocation / zero blocking 契约
- interface adapter 与 message mapper 的规则可单元测试验证
- 状态机切换时 service 可用性与 callback gate 行为可验证
- 同一 `PublishBundle` 内的时间戳一致性可验证
- controller rate 与 simulation rate 不一致时行为可验证
- publish latency 不改写 acquisition timestamp 的语义可验证
- feedback closure 至少跨一个 runtime commit 的契约可验证

## 10. 迁移开发计划

本文假定阶段 1“消息包迁移到 `robot_mujoco_msgs`”已经单独推进；`robot_mujoco_ros2` 迁移从统一
ROS adapter 包开始。

### 阶段 A：建立新包骨架

目标：

- 新建 `robot_mujoco_ros2`
- 冻结包名、namespace、公开主类型命名
- 建立最小 `CMakeLists.txt`、`package.xml`、导出规则

开发任务：

- 创建新包并声明对 `mujoco_simulation`、`robot_mujoco_msgs`、`rclcpp`、`hardware_interface`、
  `pluginlib` 的依赖
- 建立 `hardware_plugin` 主类型与 `ros_bridge` 主类型的最小骨架
- 预留 plugin description 和导出目标

完成标志：

- 工作区可识别 `robot_mujoco_ros2`
- 新包可独立编译
- 公开入口命名冻结，不再继续讨论

### 阶段 B：先迁移 `ros_bridge`

目标：

- 将当前 `mujoco_simulation_ros` 的发布和服务能力迁入新包
- 先不改变主 owner 逻辑，只先完成代码搬迁和接口收口

开发任务：

- 迁移 `/clock` publisher
- 迁移 IMU / Camera / Lidar publisher
- 迁移仿真控制 services
- 迁移 `Status/Result -> ROS response` 适配
- 在新包中引入 `MultiThreadedExecutor + callback groups`
- 建立 runtime 出站 publish worker，但不再额外定义第二套 ROS executor

完成标志：

- `ros_bridge` 已在 `robot_mujoco_ros2` 中提供等价功能
- 旧 `mujoco_simulation_ros` 只剩兼容壳或转发层
- `ros_bridge` 不持有 `Simulation`
- ROS callback 调度与 runtime 出站发布职责已分离

### 阶段 C：迁移 `hardware_plugin` 并固定 owner 关系

目标：

- 将当前 `mujoco_hardware` 迁入新包
- 把 `hardware_plugin` 固定为 `Simulation` 唯一 owner

开发任务：

- 迁移 `SystemInterface` 实现
- 迁移 `HardwareInfo` 解析和运行时配置组装逻辑
- 将 `ros_bridge` 改为由 `hardware_plugin` 创建和持有
- 收敛现有跨包组合逻辑为包内组合
- 固定单一系统状态机与 callback gate 逻辑

完成标志：

- `hardware_plugin` 成为系统唯一主入口
- `Simulation` ownership 不再分散
- 旧 `mujoco_hardware` 只剩兼容壳或被删除
- runtime lifecycle 与 ROS callback lifecycle 已由统一状态机收口

### 阶段 D：重构状态发布链为异步发布模型

目标：

- 从“`read()` 里直接拼消息并发布”为主，迁到“runtime 投递只读发布包，ROS 线程异步发布”

开发任务：

- 定义 `PublishBundle`
- 为状态快照和传感器 sample 建立 observer / enqueue 机制
- 把 ROS 消息构造移动到 publish worker
- 增加队列溢出策略和丢帧策略
- 固定有界队列、预分配缓存和热路径 non-blocking 契约
- 固定 `PublishBundle` 的时间一致性策略与 `/clock` 对齐规则

完成标志：

- runtime 不再被 ROS 发布路径同步阻塞
- `read()` 只负责控制面读取，不承担重发布载荷构造
- `/clock` 与传感器 topic 共享统一发布模型
- 并发契约已经明确为“有界、非阻塞、热路径无堆分配”
- 同一发布包内的数据时间语义已固定且可测试

### 阶段 E：收口配置与公开入口

目标：

- 收紧配置边界和公开 API 面

开发任务：

- 建立 `ConfigBuilder`、schema 校验和 normalize 流程
- 建立 `ControlInterfaceAdapter`
- 建立统一 `MessageMapper`
- 固定 controller 语义、rate mismatch 策略和推荐 RT 实现设施
- 固定 feedback closure 契约与延迟语义扩展点
- 明确核心不变量、默认策略与可替换实现点的边界
- 明确公开配置类型与私有 helper 的边界
- 清理不应暴露的内部辅助类型

完成标志：

- 用户面对的是 ROS 接口配置和 plugin 配置，而不是内部 bridge 细节
- 对外公开入口只剩主类型和必要配置类型
- interface mapping、message mapping、config validation 均已有固定归属
- 传感器延迟语义与时间戳语义已明确分离
- 规范已区分核心不变量、默认策略和可替换实现点

### 阶段 F：切换工作区引用并删除旧主路径

目标：

- 全面切换构建、测试、示例、launch 和文档入口

开发任务：

- 更新 `find_package`、plugin XML、URDF 模板、测试引用
- 更新 `robot_mujoco` 产品入口依赖
- 更新文档导航和 CI 选择包

完成标志：

- 工作区主路径切换到 `robot_mujoco_ros2`
- `mujoco_simulation_ros` / `mujoco_hardware` 不再作为主路径出现

## 11. 风险与控制点

迁移中最容易失控的点固定如下：

- 把 `ros_bridge` 重新做成独立 owner
- 让 service callback 直接访问 runtime 内部可变状态
- 让 `read()` 承担越来越重的 ROS 发布逻辑
- 让配置类型暴露内部 bridge 细节
- 让 lazy initialization 或全局可变单例进入热路径
- 让 ROS callback 生命周期与 runtime 状态机脱节
- 让不同 topic 的时间戳语义漂移
- 让默认策略被误当成长期不可替换的硬规范
- 让 controller 错误假设 same-cycle feedback closure
- 让兼容壳长期存在并重新固化旧命名

对应控制点如下：

- 所有 owner 关系以 `hardware_plugin` 为中心审查
- 线程审查只看一条规则：谁能碰主 `mjData`
- 发布链审查只看一条规则：runtime 是否会被 ROS 同步阻塞
- 热路径审查只看一条规则：是否出现分配、扩容或阻塞等待
- 对外 API 审查只看一条规则：是否暴露了内部实现心智模型
- 配置审查只看一条规则：是否先 validate/normalize 再进入运行时
- 生命周期审查只看一条规则：系统状态真源是否唯一
- 时间审查只看一条规则：同一发布包是否对应明确 simulation tick
- 规范审查只看一条规则：是否区分核心不变量、默认策略与可替换实现点
- 控制闭环审查只看一条规则：是否误承诺 same-cycle feedback

## 12. 实施验收标准

`robot_mujoco_ros2` 后续实施完成后，应满足以下标准：

- `hardware_plugin` 是唯一系统主入口
- `Simulation` ownership 固定在 `hardware_plugin`
- `ros_bridge` 是被组合的通信组件，而不是并列 owner
- runtime worker、ROS callback executor、publish worker 三个执行面边界清晰
- ROS callback 调度采用 `MultiThreadedExecutor + callback groups`
- `/clock`、sensor topic、控制服务全部由 `robot_mujoco_ros2` 提供
- ROS 线程只能通过 public API 或只读快照访问 runtime
- `read()` / `write()` 满足 zero allocation / zero blocking 契约
- runtime 到 ROS 发布面的并发契约满足“有界、非阻塞、热路径无堆分配”
- `ControlInterfaceAdapter`、`MessageMapper`、`ConfigBuilder` 已形成固定归属
- 系统状态机唯一且 ROS callback 可用性受其约束
- `PublishBundle` 和 `/clock` 的时间一致性策略已固定
- controller 语义与 rate mismatch 策略已固定
- 已明确区分核心不变量、默认策略和可替换实现点
- 传感器延迟语义不再与时间戳语义混淆
- feedback closure 契约已固定为 bounded delayed feedback，而不是 same-cycle feedback
- 配置对外不暴露内部 bridge 心智模型
- 旧 `mujoco_simulation_ros` 和 `mujoco_hardware` 不再作为主实现路径

如果实现偏离上述设计，应优先修正实现或更新本文，而不是在局部路径上重新引入新的 owner、线程
或包边界模型。
