# romujoco MobileBase 组件重构方案

## 1. 重构目标与设计原则

### 1.1 背景

当前 `romujoco` 的 `MobileBase` 实现围绕四轮 Mecanum 底盘设计，公共接口、配置结构和运行时实现均包含明显的 Mecanum 假设：

* `MobileBaseType` 当前只有 `Mecanum`；
* `MobileBaseCommand` 同时暴露 `Twist`、`WheelLinear`、`WheelAngular`；
* wheel state 固定为 `Vector4d`；
* `MobileBaseInfo` 直接包含 `MecanumInfo` 和四轮配置；
* `MobileBaseComponent` 直接持有 `MecanumMobileBase`；
* 当前底盘状态为 `(x, y, yaw)` 平面状态；
* 当前实现直接修改 base free joint 的 `qpos/qvel`，属于纯运动学底盘。

这些假设在仅支持 Mecanum 时成立，但无法自然扩展到：

* Differential / Skid Steer；
* Omni / Kiwi；
* Ackermann / Bicycle / Tricycle；
* Swerve；
* Tracked Vehicle；
* 具有 passive caster、rocker、suspension 等自由度的真实移动底盘。

当前公共 API 明确包含 Mecanum 专有字段。 当前 `MobileBaseComponent` 也直接依赖 `MecanumMobileBase`，并承担平面 base 状态推进。

MFR3 Duo 的 TMR 底盘进一步暴露了这一问题：其两个主动模块分别使用 steering position actuator 和 drive velocity actuator，底盘运动应由 wheel-ground contact 和 MuJoCo dynamics 决定，而不是直接覆盖 free-base 状态。

因此本次重构目标不是简单增加：

```cpp
MobileBaseType::Swerve
```

而是重新建立一个能够长期支持不同移动底盘的稳定组件边界。

### 1.2 核心目标

重构后的 MobileBase 应满足：

1. 公共运行时 API 与具体底盘类型解耦；
2. Mecanum、Swerve、Differential、Ackermann 等可以作为独立实现扩展；
3. 不以产品名称建立底盘类型，例如不出现 `TmrMobileBase`；
4. 不建立万能 `WheelModule` 或万能车辆模型；
5. 上层统一使用底盘级速度命令；
6. 具体 chassis 自己负责运动学和 actuator contract；
7. Kinematic 与 Dynamic 实现共享公共 Command/State，但拥有不同状态所有权；
8. Dynamic 模式下 MuJoCo 是物理状态唯一权威；
9. passive caster、rocker、suspension 等自由度允许完全由 MuJoCo 求解；
10. 后续新增底盘不应要求修改 Simulation / Scheduler 等核心运行时。

### 1.3 设计原则

整个重构遵循以下原则：

```text
1. MobileBase 是底盘级协调组件，不是 wheel actuator 通用抽象。

2. 公共 API 描述“期望底盘运动”和“实际底盘状态”。

3. wheel、steering、track、module 等属于 chassis-specific implementation。

4. 不建立一个巨大 MobileBaseType switch。

5. 不把不同底盘强制映射到统一 WheelTarget。

6. 不要求所有 chassis 同时具有 Kinematic 和 Dynamic 两种实现。

7. Dynamic 实现只控制 actuator，不直接决定 base pose。

8. Kinematic 实现可以直接推进其拥有的 base state。

9. chassis-specific 配置允许不同，不追求一个万能 XML schema。

10. 产品型号不能进入 romujoco 通用底盘类型体系。
```

---

## 2. 公共 API 与组件架构

### 2.1 公共 API 的稳定边界

公共 API 只保留两个核心概念：

```text
MobileBaseCommand
MobileBaseState
```

调用方只关心：

```text
希望底盘如何运动？
底盘当前实际状态是什么？
```

调用方不需要知道：

```text
几个轮子
几个 steering module
wheel radius
actuator 类型
Swerve module 数量
履带 contact 模型
```

总体结构：

```text
                  MobileBase API
                        │
             ┌──────────┴──────────┐
             │                     │
     MobileBaseCommand      MobileBaseState
             │                     ▲
             ▼                     │
       chassis implementation      │
             │                     │
 ┌───────────┼───────────┬─────────┼─────────┐
 ▼           ▼           ▼         ▼         ▼
Diff      Mecanum      Omni    Steering    Swerve
 │           │           │         │         │
 └───────────┴───────────┴─────────┴─────────┘
                        │
                        ▼
              MuJoCo joint / actuator
                        │
                        ▼
                 dynamics / contact
```

### 2.2 MobileBaseCommand

删除当前：

```cpp
enum class MobileBaseControlMode {
    Twist,
    WheelLinear,
    WheelAngular
};
```

以及：

```cpp
Vector4d wheel_linear;
Vector4d wheel_angular;
```

这些字段只适用于当前 Mecanum 模型，不能成为跨底盘稳定 API。

推荐定义：

```cpp
struct PlanarTwist {
    double linear_x{0.0};
    double linear_y{0.0};
    double angular_z{0.0};
};

struct MobileBaseCommand {
    ComponentId id{kInvalidComponentId};
    PlanarTwist velocity{};
};
```

公共命令明确限定为地面移动底盘平面速度：

```text
vx
vy
wz
```

不同底盘负责判断自身能够支持哪些自由度：

| 底盘           | vx | vy | wz |
| ------------ | -: | -: | -: |
| Differential |  ✓ |  × |  ✓ |
| Mecanum      |  ✓ |  ✓ |  ✓ |
| Omni         |  ✓ |  ✓ |  ✓ |
| Ackermann    |  ✓ |  × |  ✓ |
| Swerve       |  ✓ |  ✓ |  ✓ |

对于不支持的自由度必须返回错误，不允许静默截断。

例如 Differential：

```cpp
if (std::abs(command.velocity.linear_y) > tolerance) {
    return false;
}
```

不能：

```cpp
linear_y = 0.0;
```

### 2.3 MobileBaseState

当前：

```cpp
Vector3d pose;
```

实际上只能表达：

```text
x
y
yaw
```

而当前运动学实现也明确把 `z`、roll、pitch 固定。

对于 Dynamic Swerve、坡地、碰撞、悬挂以及机械臂反作用力，这种状态模型不足。

推荐增加通用几何类型：

```cpp
struct Pose3d {
    Vector3d position{};
    Quaterniond orientation{1.0, 0.0, 0.0, 0.0};
};

struct Twist3d {
    Vector3d linear{};
    Vector3d angular{};
};
```

`Quaterniond` 固定约定：

```text
{w, x, y, z}
```

与 MuJoCo `qpos` quaternion 顺序保持一致。

公共状态：

```cpp
struct MobileBaseState {
    ComponentId id{kInvalidComponentId};
    double timestamp{0.0};

    std::string reference_frame_id;
    std::string base_frame_id;

    Pose3d pose{};
    Twist3d twist{};
};
```

其中：

```text
pose
    = T_reference_base

twist.linear
    = base frame linear velocity

twist.angular
    = base frame angular velocity
```

### 2.4 Ground Truth 与 Odometry 分离

当前 `MobileBaseState` 包含：

```cpp
odom_frame_id
pose_covariance
twist_covariance
```

这实际上混入了 ROS `nav_msgs/Odometry` 语义。

重构后规定：

```text
MobileBaseState
    = simulation ground truth
```

不再表达 wheel odometry。

删除：

```cpp
odom_frame_id
pose_covariance
twist_covariance
```

改为：

```cpp
reference_frame_id
base_frame_id
```

将来如果需要：

```text
wheel encoder
    ↓
FK
    ↓
dead reckoning
```

应独立形成：

```text
Odometry / BaseOdometry
```

而不是覆盖 simulation ground truth。

这样才能真实表达：

```text
wheel odometry vx = 1.0 m/s
actual base vx    = 0.8 m/s
```

例如发生 wheel slip 时两者应允许不同。

### 2.5 MobileBaseComponent

当前 concrete `MobileBaseComponent`：

```text
MobileBaseComponent
    └── MecanumMobileBase
```

应调整为公共 interface：

```cpp
class MobileBaseComponent : public SimulationComponent {
public:
    virtual ~MobileBaseComponent() = default;

    virtual bool write(
        const SimulationContext& context,
        const MobileBaseCommand& command) = 0;

    virtual bool read_state(
        std::shared_ptr<const MobileBaseState>& state) const = 0;
};
```

具体实现：

```text
MobileBaseComponent
        ▲
        │
 ┌──────┼─────────┬────────────┐
 │      │         │            │
Mecanum Swerve Differential Ackermann
```

不允许公共 interface 中出现：

```text
wheel count
steering
track
module
```

等 chassis-specific 概念。

---

## 3. 配置模型、底盘实现与 Factory

### 3.1 公共配置

运行时 API 与配置 API 分开。

公共配置仅包含所有移动底盘真正共有的信息：

```cpp
enum class MobileBaseExecutionMode : std::uint8_t {
    Kinematic,
    Dynamic,
};

struct MobileBaseCommonInfo {
    ComponentId id{kInvalidComponentId};

    std::string name;

    std::string reference_frame_id{"world"};
    std::string base_frame_id{"base_link"};

    std::string base_body_name;

    MobileBaseExecutionMode execution_mode{
        MobileBaseExecutionMode::Dynamic};

    double period{0.0};
};
```

这里保留：

```cpp
MobileBaseExecutionMode
```

但它只用于：

```text
配置验证
implementation selection
```

不意味着建立：

```text
GenericKinematicExecutor
GenericDynamicExecutor
```

也不要求所有 chassis 支持两种模式。

例如 V1：

```text
Mecanum + Kinematic    ✓
Swerve  + Dynamic      ✓

Mecanum + Dynamic      暂不支持
Swerve  + Kinematic    暂不支持
```

如果请求不存在的组合，应初始化失败并明确报告：

```text
unsupported mobile-base execution mode
```

### 3.2 删除 MobileBaseType

删除：

```cpp
enum class MobileBaseType {
    Mecanum
};
```

具体配置类型本身已经表达 chassis：

```text
MecanumMobileBaseInfo
SwerveMobileBaseInfo
DifferentialMobileBaseInfo
AckermannMobileBaseInfo
```

XML 中仍可以保留：

```xml
<mobile_base type="mecanum">
```

但：

```text
type="..."
```

只用于 parser dispatch，不进入运行时 `switch(type)`。

也就是说：

```text
XML
 ↓
parser
 ↓
specific config type
 ↓
factory
 ↓
specific MobileBase implementation
```

而不是：

```text
XML
 ↓
MobileBaseInfo.type
 ↓
runtime switch
```

### 3.3 Mecanum 配置

将所有 Mecanum 专属配置迁移到：

```text
romujoco/component/mobile_base/mecanum.hpp
```

建议：

```cpp
enum class MecanumWheelIndex : std::size_t {
    FrontLeft,
    FrontRight,
    RearLeft,
    RearRight,
    Count,
};

inline constexpr std::size_t kMecanumWheelCount =
    static_cast<std::size_t>(MecanumWheelIndex::Count);

struct MecanumWheelInfo {
    std::string joint_name;

    double radius{0.0};
    double direction{1.0};
    double speed_response{0.0};
};

struct MecanumMobileBaseInfo {
    MobileBaseCommonInfo common;

    double wheel_base{0.0};
    double track_width{0.0};

    std::array<MecanumWheelInfo, kMecanumWheelCount> wheels;
};
```

V1 Mecanum 继续使用：

```text
execution_mode = Kinematic
```

从而保留当前行为和测试基线。

### 3.4 Swerve 配置

Swerve 采用：

```text
N × independently steered drive module
```

而不是建立：

```text
TwoModuleSwerve
FourModuleSwerve
TMR
```

等专用类型。

建议：

```cpp
struct SwerveModuleInfo {
    std::string name;

    double position_x{0.0};
    double position_y{0.0};

    double wheel_radius{0.0};

    std::string steering_joint_name;
    std::string steering_actuator_name;

    std::string drive_joint_name;
    std::string drive_actuator_name;
};

struct SwerveMobileBaseInfo {
    MobileBaseCommonInfo common;

    std::vector<SwerveModuleInfo> modules;
};
```

配置阶段允许 `std::vector`。

因为：

```text
parse / init
```

不是 physics fast path。

初始化后应把：

```text
joint names
actuator names
```

全部解析为：

```text
joint id
actuator id
qpos address
qvel address
```

physics step 内不得再进行：

```text
string lookup
allocation
vector growth
```

MFR3 TMR 只是一个：

```text
Swerve
module_count = 2
execution_mode = Dynamic
```

的配置实例。

其中：

```text
front:
  steering = tmrv0_2_joint_0
  drive    = tmrv0_2_joint_1

rear:
  steering = tmrv0_2_joint_2
  drive    = tmrv0_2_joint_3
```

对应现有 MJCF position/velocity actuator。

### 3.5 后续底盘扩展

V1 不要求全部实现，但架构应允许：

```text
DifferentialMobileBaseInfo
OmniMobileBaseInfo
AckermannMobileBaseInfo
TrackedMobileBaseInfo
```

不同 config 可以完全不同。

例如 Differential：

```cpp
struct DifferentialMobileBaseInfo {
    MobileBaseCommonInfo common;

    double wheel_separation{0.0};

    std::vector<DriveWheelInfo> left_wheels;
    std::vector<DriveWheelInfo> right_wheels;
};
```

Ackermann：

```cpp
struct AckermannMobileBaseInfo {
    MobileBaseCommonInfo common;

    // steering geometry
    // steering joint bindings
    // traction bindings
};
```

不追求把所有底盘重新统一成：

```cpp
GenericWheelModule
```

### 3.6 Factory

Factory 负责：

```text
specific config
        ↓
specific implementation
```

例如：

```cpp
std::unique_ptr<MobileBaseComponent> create_mobile_base(
    const MecanumMobileBaseInfo& info);

std::unique_ptr<MobileBaseComponent> create_mobile_base(
    const SwerveMobileBaseInfo& info);
```

或者内部使用：

```cpp
class MobileBaseFactory
```

都可以。

关键约束是：

```text
factory 创建完成以后，
runtime 不再基于 chassis type switch。
```

当前：

```cpp
if (info_.type != MobileBaseType::Mecanum)
```

这种逻辑应完全消失。

### 3.7 ComponentConfig

当前：

```cpp
using ComponentConfig =
    std::variant<
        JointInfo,
        ImuInfo,
        CameraConfig,
        LidarInfo,
        MobileBaseInfo>;
```

本次不同时重构整个配置系统。

暂时调整为：

```cpp
using ComponentConfig =
    std::variant<
        JointInfo,
        ImuInfo,
        CameraConfig,
        LidarInfo,
        MecanumMobileBaseInfo,
        SwerveMobileBaseInfo>;
```

将来增加：

```text
DifferentialMobileBaseInfo
AckermannMobileBaseInfo
```

即可。

是否彻底删除公共 `std::variant` 应作为独立 architecture task，不与本次 MobileBase 重构耦合。

---

## 4. Kinematic / Dynamic 实现规则

### 4.1 两种执行模型共享 API

无论内部实现方式如何：

```text
MobileBaseCommand
MobileBaseState
```

始终相同。

区别仅在于：

```text
谁拥有 physical state。
```

### 4.2 Kinematic ownership

Kinematic implementation 可以直接决定：

```text
base qpos/qvel
owned wheel qpos/qvel
```

典型流程：

```text
MobileBaseCommand
        ↓
kinematics
        ↓
wheel target
        ↓
kinematic response model
        ↓
base velocity
        ↓
pose integration
        ↓
write qpos/qvel
```

现有 Mecanum 基本保持：

```text
Twist
 ↓
Mecanum IK
 ↓
wheel speed
 ↓
first-order response
 ↓
Mecanum FK
 ↓
SE(2) integration
 ↓
base qpos/qvel
```

当前实现就是直接推进 wheel 和 base state。

重构时主要变化是：

* 将平面推进逻辑移出公共 `MobileBaseComponent`；
* 由 `KinematicMecanumMobileBase` 自己拥有；
* 最终转换为统一 `Pose3d/Twist3d` state。

### 4.3 Dynamic ownership

Dynamic implementation 不允许写：

```text
base qpos
base qvel
passive joint state
```

原则：

```text
Dynamic MobileBase
    owns actuator command

MuJoCo
    owns physical state
```

流程：

```text
MobileBaseCommand
        ↓
chassis kinematics
        ↓
actuator targets
        ↓
data.ctrl[]
        ↓
mj_step()
        ↓
joint dynamics
contact
friction
passive DOF
        ↓
actual base qpos/qvel
        ↓
MobileBaseState
```

### 4.4 Dynamic Swerve

对于 Swerve：

```text
vx
vy
wz
 ↓
Swerve IK
 ↓
module 0:
    steering position
    drive velocity

module 1:
    steering position
    drive velocity

...
 ↓
data.ctrl[]
```

Swerve IK 建议参考 Franka 官方行为：

```text
vxi = vx - wz * yi
vyi = vy + wz * xi

speed_i =
    sqrt(vxi² + vyi²) / radius

steering_i =
    atan2(vyi, vxi)
```

并必须支持：

```text
steering shortest path
wheel direction reversal
zero-speed steering hold
```

Franka 官方 `SwerveKinematics` 已实现这套逻辑，可作为行为基线。

MFR3 Duo 中的 actuator mapping：

```text
steering:
    position actuator

drive:
    velocity actuator
```

不再经过当前 JointComponent 的 torque-PD 路径。

直接：

```cpp
data.ctrl[steering_actuator] = steering_target;
data.ctrl[drive_actuator] = drive_velocity_target;
```

当前 TMR MJCF 已明确使用 `<position>` 与 `<velocity>` actuator。

### 4.5 passive joints

Dynamic MobileBase 只拥有主动 actuator DOF。

例如 MFR3 Duo：

```text
主动：
tmrv0_2_joint_0
tmrv0_2_joint_1
tmrv0_2_joint_2
tmrv0_2_joint_3

被动：
caster steering
caster wheel
rocker arm
```

被动自由度：

```text
不注册为 MobileBase command target
不在 advance() 中写 qpos/qvel
```

只由：

```text
MuJoCo contact/dynamics
```

求解。

### 4.6 base state 获取

Dynamic implementation 应从 MuJoCo 获取：

```text
position
orientation
linear velocity
angular velocity
```

不再从 wheel FK 推导 actual state。

wheel FK 可以用于：

```text
odometry estimate
diagnostics
controller feedback
```

但不能替代 ground truth。

### 4.7 scheduler

当前 scheduler：

```text
write command
    ↓
component advance
    ↓
mj_forward
    ↓
mj_step
    ↓
component update
```

整体顺序无需重构。

对于 Kinematic：

```text
advance
    → prescribed state
```

对于 Dynamic：

```text
write
    → ctrl target

advance
    → optional controller-side calculation
      不写 physical state
```

随后：

```text
mj_step()
```

生成实际状态。

因此本次不需要修改：

```text
Simulation
SimulationRuntime
SimulationScheduler
CommandBuffer
StateBuffer
```

除非具体实现暴露新的独立问题。

---

## 5. 代码组织、迁移与验证计划

### 5.1 推荐目录

公共头文件：

```text
include/romujoco/
├── common/
│   ├── math.hpp
│   └── geometry.hpp
│
└── component/
    ├── mobile_base.hpp
    │
    └── mobile_base/
        ├── common.hpp
        ├── mecanum.hpp
        └── swerve.hpp
```

内部实现：

```text
src/component/mobile_base/
├── mobile_base_component.hpp
├── mobile_base_factory.hpp
├── mobile_base_factory.cpp
│
├── mecanum/
│   ├── mecanum_kinematics.hpp
│   ├── mecanum_kinematics.cpp
│   ├── kinematic_mecanum_mobile_base.hpp
│   └── kinematic_mecanum_mobile_base.cpp
│
└── swerve/
    ├── swerve_kinematics.hpp
    ├── swerve_kinematics.cpp
    ├── dynamic_swerve_mobile_base.hpp
    └── dynamic_swerve_mobile_base.cpp
```

暂时不要创建：

```text
executor/
kinematic/
dynamic/
wheel_module/
vehicle/
```

等高层抽象目录。

只有当第二种 Kinematic 或第二种 Dynamic implementation 出现明确大量共享代码后，再提取公共层。

### 5.2 第一阶段：公共 API 与配置拆分

完成：

```text
PlanarTwist
Pose3d
Twist3d
MobileBaseCommand
MobileBaseState
MobileBaseCommonInfo
```

删除：

```text
MobileBaseType
MobileBaseControlMode
WheelLinear
WheelAngular
public Vector4d wheel state
```

新增：

```text
MecanumMobileBaseInfo
SwerveMobileBaseInfo
```

调整：

```text
SimulationConfig
parser
validator
ComponentManager
```

本阶段不改变 Mecanum 仿真结果。

### 5.3 第二阶段：Mecanum 迁移

把现有：

```text
MobileBaseComponent
+
MecanumMobileBase
+
advance_planar_base()
```

重组为：

```text
KinematicMecanumMobileBase
```

保留现有：

```text
Mecanum IK/FK
speed_response
SE(2) integration
wheel state advance
```

确保与当前行为一致。

这一阶段的目的不是增强 Mecanum，而是验证：

> 新公共 API 能否承载现有实现而不引入额外抽象。

### 5.4 第三阶段：Dynamic Swerve

实现：

```text
SwerveKinematics
DynamicSwerveMobileBase
```

支持：

```text
N modules
steering position target
drive velocity target
shortest steering path
drive reversal
zero-speed angle hold
```

先以：

```text
module_count = 2
```

的 MFR3 Duo 为 integration target。

Dynamic implementation：

```text
禁止写 base qpos/qvel
禁止写 passive joint state
只写 actuator ctrl
```

### 5.5 第四阶段：MFR3 Duo 集成与验证

增加正式 MFR3 Duo simulation config。

至少覆盖：

```text
base body
2 × Swerve module
4 × active actuator
passive caster
rocker
ground contact
```

测试必须验证：

1. 静止：

   * base 不漂移；
   * steering 不异常跳变；

2. 前进：

   * `vx > 0`；
   * 两 drive module 同方向；
   * base 实际前进；

3. 横移：

   * `vy != 0`；
   * steering 正确旋转；
   * base 产生 lateral motion；

4. 原地旋转：

   * `wz != 0`；
   * 前后 module steering/drive target 正确；

5. 组合运动：

   * `vx + vy + wz`；

6. 零速：

   * steering angle 保持；
   * drive target 为零；

7. steering reversal：

   * 超过 90° 时使用 wheel reversal；

8. 动力学真实性：

   * base pose 来源于 MuJoCo；
   * wheel/contact friction 能影响运动；
   * 碰撞能够阻止底盘继续穿透；
   * rocker/caster 保持 passive dynamics。

MFR3 Duo scene 已经包含显式 wheel-ground contact，可作为 Dynamic Swerve 验证基础。

### 5.6 第五阶段：架构扩展验证

在 Swerve 完成后，不立即实现所有底盘。

推荐只增加：

```text
Differential
```

作为第二个结构差异明显的 family。

目的不是增加功能数量，而是验证：

```text
MobileBaseCommand
MobileBaseState
MobileBaseComponent
Factory
Config
```

是否无需修改即可支持新 chassis。

如果加入 Differential 时只需要：

```text
DifferentialMobileBaseInfo
DifferentialKinematics
Dynamic/KinematicDifferentialMobileBase
parser branch
factory overload
```

而无需改变公共 API，则说明架构成功。

之后再考虑：

```text
Steering family
    Bicycle
    Tricycle
    Ackermann

Omni

Tracked
```

### 5.7 必须避免的实现方向

本次重构明确禁止以下设计：

```text
1. 一个巨大的 MobileBaseType enum。

2. MobileBaseComponent 中 switch(type)。

3. GenericWheelModule 描述所有底盘。

4. GenericWheelTarget 同时承担：
   velocity / steering / track 等语义。

5. 所有底盘共用固定 Vector4d。

6. TMR / Franka / MFR3Duo 成为公共底盘类型。

7. Dynamic Swerve 直接写 base qpos/qvel。

8. 为了适配 TMR 修改 MJCF 为普通 motor，
   再由 MobileBase 重复实现 position/velocity servo。

9. 同时大规模重构 SimulationConfig、
   ComponentConfig、Scheduler 和整个 component framework。

10. 为尚未存在的底盘提前设计复杂通用 vehicle framework。
```

### 5.8 完成标准

MobileBase 重构完成后，应满足以下架构标准：

```text
公共 API：
    不包含 Mecanum / Swerve / wheel-specific 类型。

Mecanum：
    当前 Kinematic 行为保持。

Swerve：
    支持 N-module。
    MFR3 Duo 两模块 TMR 可以直接配置。

Dynamic：
    只输出 actuator command。
    MuJoCo 拥有 physical state。

State：
    支持完整 SE(3) pose / twist。
    与 wheel odometry 解耦。

配置：
    chassis-specific。
    不存在产品专有类型。

运行时：
    physics step 内无字符串查找。
    无配置解析。
    无动态结构增长。

扩展：
    新增 Differential 时无需修改公共 Command/State。
```

最终期望架构为：

```text
                       Application
                           │
                    MobileBaseCommand
                      vx / vy / wz
                           │
                           ▼
                  MobileBaseComponent
                    common interface
                           │
        ┌──────────────────┼──────────────────┐
        │                  │                  │
        ▼                  ▼                  ▼
 KinematicMecanum    DynamicSwerve       Differential
        │                  │                  │
    Mecanum IK          Swerve IK          Diff IK
        │                  │                  │
 prescribed state   actuator targets     joint targets
        │                  │                  │
        └──────────────────┼──────────────────┘
                           ▼
                         MuJoCo
                    joint / actuator
                    contact / dynamics
                           │
                           ▼
                   MobileBaseState
                    Pose3d / Twist3d
                           │
                           ▼
                      Application
```

这套结构把三个问题彻底分开：

```text
公共 API
    负责底盘级语义；

chassis implementation
    负责运动学、机械拓扑和 actuator contract；

MuJoCo
    负责真实物理状态。
```

这应作为后续 `romujoco` MobileBase 实现和评审的架构基线。
