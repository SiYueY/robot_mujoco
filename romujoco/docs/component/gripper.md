# RoMuJoCo Gripper 组件 V1 设计

## 1. 目标与范围

`Gripper` 是 `romujoco` 中与 `Joint`、`MobileBase`、`IMU`、`Camera`、`Lidar` 同级的设备组件，用于描述和仿真**双指平行夹爪（two-finger parallel gripper）**。

V1 的核心目标是提供统一的设备级控制和状态接口：

```text
GripperCommand
    width
    velocity
    effort

        ↓

GripperComponent

        ↓

MuJoCo fingers / actuators

        ↓

GripperState
    width
    velocity
    effort
    stalled
```

V1 面向：

* Franka Hand；
* Robotiq 类双指平行夹爪；
* 单 actuator + 机械耦合结构；
* 双 actuator 独立驱动结构。

V1 不尝试抽象任意机器人手。以下内容不属于本阶段：

* 三指或多指机械手；
* 独立 finger command；
* tendon / adaptive gripper 的专用控制语义；
* suction gripper；
* grasp planning；
* `is_grasped` 判断；
* 接触力精确估计；
* ROS 2 Action 生命周期。

因此 V1 的设计原则是：

> `Gripper` 明确定义为双指平行夹爪，而不是通过复杂抽象提前兼容所有末端执行器。

---

## 2. 公共数据模型

### 2.1 通用 Limit

公共配置增加通用区间类型：

```cpp
struct Limit {
    double min{-std::numeric_limits<double>::infinity()};
    double max{std::numeric_limits<double>::infinity()};
};
```

`Limit` 本身只表达数值区间，不包含 Joint、Gripper 等设备语义。

现有 Joint 配置也应逐步统一使用：

```cpp
Limit position_limits;
Limit velocity_limits;
Limit effort_limits;
```

避免分别维护 `JointLimit`、`GripperLimit` 等重复结构。

---

### 2.2 GripperCommand

```cpp
using GripperId = std::size_t;

struct GripperCommand {
    GripperId id{0};

    double width{0.0};
    double velocity{0.0};
    double effort{0.0};
};
```

字段定义：

| 字段         | 单位  | 含义           |
| ---------- | --- | ------------ |
| `width`    | m   | 目标夹爪总开口宽度    |
| `velocity` | m/s | 最大开合速度幅值     |
| `effort`   | N   | 允许使用的最大驱动力幅值 |

`velocity` 和 `effort` 都是非负幅值。

运动方向只由：

```text
command.width - current_width
```

决定。

因此：

```text
target > current    opening
target < current    closing
```

不使用正负 `velocity` 表示方向。

零值具有明确含义：

```text
velocity = 0
    不继续推进 width reference

effort = 0
    actuator 不输出驱动力
```

不使用“0 表示采用默认值”之类的隐式规则。

---

### 2.3 GripperState

```cpp
struct GripperState {
    GripperId id{0};
    double timestamp{0.0};

    double width{0.0};
    double velocity{0.0};
    double effort{0.0};

    bool stalled{false};
};
```

字段定义：

| 字段         | 单位  | 含义             |
| ---------- | --- | -------------- |
| `width`    | m   | 当前总开口宽度        |
| `velocity` | m/s | 当前实际开合速度       |
| `effort`   | N   | 当前实际驱动力幅值      |
| `stalled`  | -   | 当前运动是否因外部阻挡而停滞 |

与 Command 不同：

```text
Command.velocity
    非负速度限制

State.velocity
    带符号实际速度
```

即：

```text
state.velocity > 0    opening
state.velocity < 0    closing
```

V1 不提供：

```cpp
bool reached_goal;
```

因为它取决于：

```text
Command + State + tolerance
```

属于 Controller / ROS Action 层状态，而不是设备物理状态。

V1 同样不提供：

```cpp
bool is_grasped;
```

因为可靠的 grasp detection 需要额外定义接触对象、接触位置、持续时间和接触力等规则。

---

### 2.4 Gripper 配置

每个夹爪固定包含两个 Finger：

```cpp
struct GripperFingerInfo {
    std::string joint_name;
    std::string actuator_name;
};
```

其中：

```text
joint_name
    Finger 在 MuJoCo 中对应的运动 joint

actuator_name
    驱动该 Finger 的 actuator
```

`actuator_name` 可以为空。

为空表示：

> 该 Finger 没有独立 actuator，其运动由 MJCF 中的 equality、tendon 或其他机械约束负责。

因此两种结构都可以表达：

```text
Franka Hand

Finger 0
 ├── joint
 └── actuator

Finger 1
 └── joint

两个 joint 由 MJCF equality coupling
```

以及：

```text
Dual-actuator gripper

Finger 0
 ├── joint
 └── actuator 0

Finger 1
 ├── joint
 └── actuator 1
```

完整配置：

```cpp
struct GripperControl {
    double stiffness{0.0};
    double damping{0.0};
};

struct GripperStallDetection {
    double width_tolerance{0.001};
    double velocity_threshold{0.001};
    double effort_ratio{0.9};
    double timeout{0.25};
};

struct GripperInfo {
    GripperId id{0};
    std::string name;

    std::array<GripperFingerInfo, 2> fingers;

    double period{0.001};

    GripperControl control;

    Limit width_limits{
        0.0,
        std::numeric_limits<double>::infinity()};

    Limit velocity_limits{
        0.0,
        std::numeric_limits<double>::infinity()};

    Limit effort_limits{
        0.0,
        std::numeric_limits<double>::infinity()};

    GripperStallDetection stall;
};
```

使用：

```cpp
std::array<GripperFingerInfo, 2>
```

是有意为之。

它正式表达：

> Gripper V1 是双指平行夹爪。

不使用 `std::vector` 制造对多指夹爪的表面兼容，因为当前 `width / velocity / effort` 本身就是双指平行夹爪的设备语义。

---

## 3. MuJoCo 模型与配置契约

### 3.1 XML 配置

配置使用设备领域概念：

```xml
<gripper
    id="0"
    name="left_gripper"
    period="0.001">

  <finger
      joint="left_fr3v2_1_finger_joint1"
      actuator="left_fr3v2_1_finger_motor"/>

  <finger
      joint="left_fr3v2_1_finger_joint2"/>

  <control
      stiffness="..."
      damping="..."/>

  <limit>
    <width min="0.0" max="0.08"/>
    <velocity min="0.0" max="..."/>
    <effort min="0.0" max="..."/>
  </limit>

  <stall
      width_tolerance="0.001"
      velocity_threshold="0.001"
      effort_ratio="0.9"
      timeout="0.25"/>

</gripper>
```

不使用：

```text
driver_joint
follower_joint
```

因为这是特定机械拓扑产生的角色，不属于 Gripper 公共配置。

同样不在 Gripper 层显式描述：

```text
equality
mimic
tendon
coupling
```

机械关系属于 MJCF 模型。

---

### 3.2 Finger 模型约束

V1 要求两个 Finger 对应：

```text
mjJNT_SLIDE
```

并约定 MuJoCo joint coordinate 已经按照设备语义规范化：

> joint position 增大表示对应 Finger 向外打开。

因此：

```text
q0 >= 0
q1 >= 0
```

且夹爪总宽度定义为：

```text
width = q0 + q1
```

速度：

```text
width_velocity = dq0 + dq1
```

V1 不增加：

```text
direction
scale
sign
offset
```

等额外映射参数。

如果某个 MJCF 模型使用相反的 joint coordinate，应优先在模型层规范化，而不是增加 Gripper API 复杂度。

---

### 3.3 Actuator 模型

每个 Finger：

```text
0 或 1 个 actuator
```

整个 Gripper 至少需要一个 actuator。

因此支持：

```text
2 fingers + 1 actuator
2 fingers + 2 actuators
```

不支持：

```text
2 fingers + 0 actuators
```

V1 actuator 要求直接作用于对应 finger joint：

```text
mjTRN_JOINT
```

并使用可直接解释为 generalized force 的简单 motor topology。

建议保持与现有 JointComponent 相同的约束：

```text
dyntype  = mjDYN_NONE
gaintype = mjGAIN_FIXED
biastype = mjBIAS_NONE
gear     = 1
```

配置中的：

```text
finger.actuator_name
```

必须与 MuJoCo 中 actuator 实际绑定的 joint 一致。

---

### 3.4 Resource Ownership

一个 finger joint 或 actuator 只能由一个高层 Component 拥有。

例如：

```text
finger_joint1
finger_joint2
```

已经属于：

```text
GripperComponent
```

则不得同时配置为：

```text
JointComponent
```

同一个 actuator 也不得同时被：

```text
Joint
MobileBase
Gripper
```

多个组件控制。

初始化阶段必须检测这些 resource ownership conflict。

原则是：

> 一个物理执行资源只有一个 Component owner。

对于 MFR3Duo：

```text
FR3 joint1 ... joint7
    → JointComponent

finger_joint1 + finger_joint2
    → GripperComponent
```

---

### 3.5 配置校验

公共配置首先检查：

```text
id 合法
name 非空
period > 0

两个 finger joint 非空
两个 joint 不相同

至少存在一个 actuator

stiffness >= 0
damping >= 0
```

Limit 必须满足：

```text
min <= max
```

Gripper V1 进一步要求：

```text
width_limits.min >= 0

velocity_limits.min == 0
velocity_limits.max > 0

effort_limits.min == 0
effort_limits.max > 0
```

保持：

```text
velocity = 0
effort = 0
```

始终具有安全且明确的意义。

Stall 参数要求：

```text
width_tolerance >= 0
velocity_threshold >= 0

0 < effort_ratio <= 1

timeout >= 0
```

模型初始化阶段继续检查：

* joint 存在；
* joint 类型为 `mjJNT_SLIDE`；
* actuator 存在；
* actuator 与对应 Finger joint 匹配；
* actuator topology 满足 V1 契约；
* joint / actuator 没有被其他 Component 重复占用。

---

## 4. 控制与状态模型

### 4.1 Command 处理

收到：

```cpp
GripperCommand
```

后先检查所有值：

```text
finite
velocity >= 0
effort >= 0
```

`NaN`、`Inf` 或负值直接拒绝。

合法命令按照配置限制：

```cpp
target_width =
    std::clamp(
        command.width,
        width_limits.min,
        width_limits.max);

velocity_limit =
    std::clamp(
        command.velocity,
        0.0,
        velocity_limits.max);

effort_limit =
    std::clamp(
        command.effort,
        0.0,
        effort_limits.max);
```

组件保存目标，而不是收到命令时直接一次性写 `ctrl`。

---

### 4.2 Velocity-limited Width Reference

组件内部维护：

```cpp
double target_width_;
double reference_width_;
double reference_velocity_;
```

每个 physics step 根据：

```text
dt
```

推进 reference：

```cpp
const double error =
    target_width_ - reference_width_;

reference_velocity_ =
    std::clamp(
        error / dt,
        -command_.velocity,
         command_.velocity);

reference_width_ +=
    reference_velocity_ * dt;
```

因此：

```text
abs(reference_velocity)
    <= command.velocity
```

`velocity` 是真实生效的运动约束，而不只是保存于 Command 中的元数据。

---

### 4.3 Finger Reference

V1 是对称双指平行夹爪，因此：

```text
q_ref = reference_width / 2

dq_ref = reference_velocity / 2
```

两个 Finger 使用相同的几何 reference。

对于没有 actuator 的 Finger：

> 不直接控制，由 MJCF mechanical coupling 负责运动。

对于有 actuator 的 Finger：

> 独立执行内部控制器。

---

### 4.4 内部控制

每个主动 Finger 使用简单 PD generalized-force controller：

```text
u =
    Kp * (q_ref - q)
  + Kd * (dq_ref - dq)
```

其中：

```text
Kp = stiffness
Kd = damping
```

然后进行 effort 限制：

```cpp
u = std::clamp(
    u,
    -command_.effort,
     command_.effort);
```

最后写入：

```cpp
data->ctrl[actuator_id] = u;
```

如果有两个 actuator，则两个 active Finger 分别执行相同的设备级 command。

因此 `GripperCommand::effort` 定义为：

> 每个主动 Finger actuator 执行本次 GripperCommand 时允许使用的最大 effort 幅值。

它不是两指接触力之和，也不等价于精确的物体夹持力。

这样可以同时支持：

```text
Franka
    1 actuator

Dual actuator gripper
    2 actuators
```

而无需在公共 Command 中暴露 actuator 数量。

---

### 4.5 State

每次组件状态更新：

```cpp
state.width =
    q0 + q1;

state.velocity =
    dq0 + dq1;
```

`effort` 定义为当前所有主动 Finger actuator 的最大绝对 effort：

```text
state.effort =
    max(abs(actuator0_force),
        abs(actuator1_force))
```

对于单 actuator Gripper：

```text
state.effort =
    abs(actuator_force)
```

这样 `Command.effort` 和 `State.effort` 具有一致的比较语义。

---

### 4.6 Stall Detection

`stalled` 用于表达：

> 当前目标仍未达到，但夹爪已经基本停止运动，同时 actuator 持续达到较高 effort。

条件：

```text
abs(target_width - state.width)
    > width_tolerance

abs(state.velocity)
    <= velocity_threshold

state.effort
    >= command.effort * effort_ratio
```

并持续：

```text
>= timeout
```

后：

```cpp
state.stalled = true;
```

任意条件解除后：

```cpp
state.stalled = false;
```

并清除 stall timer。

当：

```text
command.effort == 0
```

时不进行 stall 判断。

---

### 4.7 Reset

Simulation reset 后：

```text
target_width
    = current_width

reference_width
    = current_width

reference_velocity
    = 0

command.velocity
    = 0

command.effort
    = 0

actuator ctrl
    = 0

stalled
    = false
```

reset 不主动打开或关闭夹爪，也不会突然施加 actuator force。

---

## 5. RoMuJoCo 集成

### 5.1 ComponentConfig

增加：

```cpp
GripperInfo
```

最终：

```cpp
using ComponentConfig =
    std::variant<
        JointInfo,
        GripperInfo,
        ImuInfo,
        CameraConfig,
        LidarInfo,
        MecanumMobileBaseInfo,
        SwerveMobileBaseInfo>;
```

---

### 5.2 Command / State

增加：

```cpp
using GripperCommands =
    std::vector<GripperCommand>;
```

`RobotCommand`：

```cpp
struct RobotCommand {
    std::uint64_t sequence{0};

    JointCommands joints;
    GripperCommands grippers;
    MobileBaseCommands mobile_bases;
};
```

State 增加：

```cpp
using GripperStates =
    StateSnapshots<GripperState>;
```

`RobotState`：

```cpp
struct RobotState {
    std::uint64_t sequence{0};
    std::uint64_t timestamp{0};
    double simulation_time{0.0};
    std::uint64_t step{0};

    JointStates joints;
    GripperStates grippers;
    MobileBaseStates mobile_bases;
    ImuStates imus;
    LidarStates lidars;
    CameraStates cameras;

    ContactStates contacts;
};
```

---

### 5.3 Simulation API

新增：

```cpp
bool write_command(
    const GripperCommand& command);

bool write_commands(
    const GripperCommands& commands);

bool read_state(
    GripperState& state) const;

bool read_state(
    GripperStates& states) const;
```

保持和现有：

```text
Joint
MobileBase
IMU
Camera
Lidar
```

相同的公共 API 风格。

---

### 5.4 ComponentManager

增加：

```cpp
std::vector<GripperComponent::UniquePtr>
    gripper_components_;

GripperStates grippers_;
```

生命周期：

```text
initialize
    ↓
GripperComponent::init()

reset
    ↓
GripperComponent::reset()

每个 physics step
    ↓
GripperComponent::advance()

按 period
    ↓
GripperComponent::update()
```

其中：

```text
advance()
    trajectory
    controller
    actuator command

update()
    state sampling
    stall detection
    state publication
```

同一个 `RobotCommand` 中不允许出现重复的：

```text
GripperId
```

重复 ID 直接返回失败，不采用隐式 last-command-wins。

---

### 5.5 ROS 2 映射

`romujoco` Core 不依赖：

```text
rclcpp
control_msgs
sensor_msgs
```

ROS adapter 负责协议转换。

经典：

```text
control_msgs/GripperCommand

position
    → width

max_effort
    → effort
```

由于经典接口没有 velocity：

```text
velocity
```

由 ROS adapter 配置决定。

对于：

```text
ParallelGripperCommand
```

adapter 负责在：

```text
finger joint representation
```

与：

```text
Gripper width representation
```

之间转换。

例如对称双指：

```text
finger position =
    width / 2

finger velocity =
    velocity / 2
```

`stalled` 可以直接映射。

`reached_goal` 由 adapter 根据：

```text
abs(command.width - state.width)
    <= goal_tolerance
```

计算，而不加入 `GripperState`。

---

## 6. MFR3Duo 与验证要求

当前 MFR3Duo 左侧 Franka Hand：

```text
left_gripper

Finger 0
    joint:
        left_fr3v2_1_finger_joint1

    actuator:
        left_fr3v2_1_finger_motor

Finger 1
    joint:
        left_fr3v2_1_finger_joint2

    actuator:
        none
```

右侧对应：

```text
right_gripper

Finger 0
    right_fr3v2_1_finger_joint1
    right_fr3v2_1_finger_motor

Finger 1
    right_fr3v2_1_finger_joint2
```

当前两个 Finger joint：

```text
range = 0 .. 0.04 m
```

因此：

```text
width_limits =
    0 .. 0.08 m
```

Finger 间同步继续由：

```text
MJCF equality coupling
```

负责。

`GripperComponent` 不复制该机械约束。

---

### 测试要求

V1 至少覆盖四类测试。

#### 配置与初始化

验证：

* 合法单 actuator Gripper；
* 合法双 actuator Gripper；
* 缺少 joint；
* 非 `slide` joint；
* actuator 不存在；
* actuator 绑定错误 joint；
* 两个 Finger 使用相同 joint；
* 两个 Finger 均无 actuator；
* 非法 Limit；
* 重复 Component ID；
* joint / actuator ownership conflict。

#### Command 与控制

验证：

* open；
* close；
* width limit；
* velocity limit；
* effort limit；
* `velocity == 0`；
* `effort == 0`；
* negative velocity / effort 拒绝；
* NaN / Inf 拒绝；
* duplicate GripperId 拒绝。

特别需要验证：

```text
command.velocity
```

确实限制 reference width 的变化速度，而不是只被保存。

同样需要验证：

```text
command.effort
```

确实限制 actuator output。

#### State 与 Stall

验证：

```text
width = q0 + q1
velocity = dq0 + dq1
effort 正确
timestamp 正确
```

构造固定障碍物，使夹爪闭合受阻，并验证：

```text
目标未达到
+
velocity 很小
+
effort 接近 limit
+
持续 timeout

→ stalled == true
```

解除阻挡后：

```text
stalled == false
```

#### MFR3Duo 集成测试

必须直接加载实际：

```text
mfr3duo.xml
```

验证：

* 左 Gripper 初始化；
* 右 Gripper 初始化；
* 左右独立开合；
* 左右同时控制；
* width 正确；
* velocity 正确；
* effort limit 生效；
* equality-coupled Finger 正确跟随；
* reset 行为正确；
* 夹持固定物体时 stalled 正确。

只有完成实际模型集成测试后，才能认为：

> MFR3Duo Franka Hand 已由 `romujoco::Gripper` 正式支持。

---

## 实施顺序

整个实现控制在四个阶段：

1. **Public Model / Config**
   增加 `Limit`、`GripperInfo`、`GripperCommand`、`GripperState`，完成 XML parser、validator 和 resource ownership 校验。

2. **GripperComponent**
   实现初始化、reference trajectory、PD 控制、单/双 actuator 支持、state 和 stall detection。

3. **Runtime Integration**
   接入 `ComponentManager`、`RobotCommand`、`RobotState` 和 `Simulation` 公共 API。

4. **Tests / MFR3Duo**
   完成 unit tests、config tests、runtime tests 和真实 `mfr3duo.xml` 集成测试。

V1 不借此机会重构其他 Component，也不增加通用 EndEffector、Actuator 或 Transmission 抽象。

最终公共控制接口冻结为：

```cpp
struct GripperCommand {
    GripperId id{0};

    double width{0.0};
    double velocity{0.0};
    double effort{0.0};
};

struct GripperState {
    GripperId id{0};
    double timestamp{0.0};

    double width{0.0};
    double velocity{0.0};
    double effort{0.0};

    bool stalled{false};
};
```

公共配置核心冻结为：

```cpp
struct GripperFingerInfo {
    std::string joint_name;
    std::string actuator_name;
};

struct GripperInfo {
    GripperId id{0};
    std::string name;

    std::array<GripperFingerInfo, 2> fingers;

    double period{0.001};

    GripperControl control;

    Limit width_limits;
    Limit velocity_limits;
    Limit effort_limits;

    GripperStallDetection stall;
};
```

整体边界保持：

```text
Application / ROS 2
        │
        ▼
GripperCommand
 width / velocity / effort
        │
        ▼
GripperComponent
 trajectory + control + limits
        │
        ▼
Finger actuators
        │
        ▼
MuJoCo mechanical model
 joint / equality / contacts
        │
        ▼
GripperState
 width / velocity / effort / stalled
```

这样 `Gripper` 保持为清晰的设备级抽象，同时不会把 Franka Hand 的单 actuator 结构硬编码进公共接口。
