# MobileBase：纯运动学 Mecanum 底盘设计方案

## 1. 目标、边界与设计原则

`MobileBase` 是建立在多个轮关节和底盘自由关节之上的底盘级运动学组件。

它面向：

* 导航；
* 移动操作；
* 策略训练；
* 上层底盘控制；
* 不要求真实轮胎动力学的机器人仿真场景。

调用方可以下发底盘速度或四轮速度，`MobileBase` 在每个 physics step 中根据 Mecanum 运动学推进底盘 SE(2) 位姿和轮子状态，并稳定产生：

* 底盘位姿；
* 底盘速度；
* 轮角速度；
* 轮线速度。

本方案采用**纯运动学模式**。

底盘轨迹不由以下因素决定：

* 轮胎—地面摩擦；
* wheel actuator；
* 轮速 PD 控制；
* MuJoCo 接触求解；
* 底盘受到的外力或反作用力。

因此本组件不用于验证：

* 轮胎摩擦参数；
* 打滑；
* 悬挂；
* 轮胎接触力；
* 坡地牵引能力；
* 高动态移动过程中的底盘动力学；
* whole-body dynamics 中严格的底盘—机械臂动力学耦合。

### 1.1 设计目标

* 支持四轮 Mecanum 的 `Twist`、`WheelLinear`、`WheelAngular` 命令；
* 每个 physics step 推进底盘和轮子，而不是由状态发布周期决定推进频率；
* 使用一阶轮速响应模拟有限的轮电机速度响应；
* 配置、公开结构和内部代码统一使用 `speed_response`；
* 不设置软件侧 per-wheel `sign`；
* MJCF wheel hinge 的轴方向必须符合 `MobileBase` 的轮坐标约定；
* 底盘轨迹、底盘速度和轮速状态全部来源于同一组响应后轮速；
* 保留现有 `MobileBaseCommand` / `MobileBaseState` 底盘级接口；
* 底盘最终运动状态由 `MobileBase` authoritative override，不受 wheel-ground contact 影响。

### 1.2 不包含

当前方案不包含：

* TMR / Mobile FR3 Duo swerve 运动学；
* Ackermann 运动学；
* differential drive；
* 里程计漂移；
* 编码器量化；
* 轮胎动力学；
* 轮胎打滑；
* 悬挂；
* 底盘碰撞导致自身轨迹改变；
* 自动避障；
* 高保真移动基座惯性模型。

### 1.3 核心设计原则

整个 `MobileBase` 遵循以下原则：

```text
1. MobileBase 是底盘级组件，不是 wheel actuator controller。

2. wheel_name 表示 MobileBase 中的轮子绑定名称。

3. speed_response 是轮速从 target 向 feedback 收敛的响应参数。

4. XML、配置结构、runtime 和局部代码统一使用 speed_response。

5. 不使用 actuator、PD、damping 或 data->ctrl。

6. wheel_angular_feedback 是整个底盘运动状态的唯一运动学源。

7. Mecanum FK/IK 使用：
   rotation_radius =
       (wheel_base + track_width) / 2

8. base twist 使用 base frame，
   free-joint translation qvel 使用 world frame。

9. odom 与 MuJoCo world 分离：
   T_world_base =
       T_world_odom * T_odom_base

10. 每 physics step 推进运动，
    period 只控制状态发布。

11. scheduler 固定采用：
    mj_step
      -> advance
      -> mj_forward
      -> state publication

12. MobileBase authoritative override 自己拥有的
    base free joint 和 four wheel hinge。

13. wheel-ground friction 不参与底盘轨迹。

14. reset 必须同时清除 command target、
    wheel feedback 和 odom runtime state。

15. 纯运动学底盘不能被解释为高保真的
    移动基座动力学模型。
```

---

## 2. 总体模型与责任划分

### 2.1 数据流

```text
MobileBaseCommand
        |
        v
命令校验与归一化
        |
        v
Mecanum 逆运动学
        |
        v
wheel_angular_target[4]
        |
        v
一阶轮速响应
        |
        v
wheel_angular_feedback[4]
        |
        +-----------------------------+
        |                             |
        v                             v
写入 wheel hinge              Mecanum 正运动学
qvel / qpos                          |
                                      v
                              (vx, vy, wz)
                                      |
                                      v
                               SE(2) 积分
                                      |
                                      v
                           写入 base free joint
                            qpos / qvel
                                      |
                                      v
                                mj_forward()
                                      |
                                      v
                              MobileBaseState
```

其中：

```text
wheel_angular_target
```

表示当前命令要求的目标轮角速度；

```text
wheel_angular_feedback
```

表示经过 `speed_response` 一阶响应后的实际仿真轮角速度。

**所有底盘状态必须由 `wheel_angular_feedback` 生成。**

不能混合：

```text
目标轮速
MuJoCo 实际底盘速度
body xpos/xmat
wheel qvel
```

作为不同字段的数据源。

### 2.2 责任划分

| 层            | 职责                                                            |
| ------------ | ------------------------------------------------------------- |
| 应用层          | 将导航目标、操纵杆输入或策略动作转换成 `(vx, vy, wz)`，必要时负责限速                    |
| `MobileBase` | 命令处理、Mecanum 正逆运动学、轮速一阶响应、SE(2) 积分、wheel/base 状态写入和状态发布       |
| MJCF         | 定义 wheel hinge、base free joint、joint axis、初始姿态、几何模型及非轮胎接触模型   |
| MuJoCo       | 机械臂及其它动态自由度的动力学计算，并在 MobileBase 写入状态后通过 `mj_forward()` 更新派生数据 |

`MobileBase` 对以下自由度具有独占写权限：

* 四个 wheel hinge；
* 一个 base free joint。

它们不得同时注册为通用 `JointComponent` 接收控制命令。

否则会出现：

```text
JointComponent
      |
      +--> 写 wheel qpos/qvel

MobileBase
      |
      +--> 写 wheel qpos/qvel
```

两个组件在同一个 physics step 中竞争同一状态的问题。

---

## 3. 配置接口与 MJCF 约定

### 3.1 WheelInfo

```cpp
struct WheelInfo {
    std::string wheel_name;
    double radius{0.0};
    double direction{1.0};
    double speed_response{0.0};  // s, 0 = ideal instantaneous response
};
```

#### `wheel_name`

表示该轮对应的 MuJoCo wheel hinge 名称。

虽然底层绑定到 MuJoCo joint，但 `WheelInfo` 是底盘级轮子抽象，因此公开字段统一使用：

```cpp
wheel_name
```

而不是：

```cpp
joint_name
wheel_joint_name
```

#### `speed_response`

表示该轮角速度的一阶响应参数，单位：

```text
s
```

其作用是控制：

> 当前轮速向目标轮速逼近的快慢。

数值越小：

```text
轮速响应越快
```

数值越大：

```text
轮速响应越慢
```

特殊值：

```text
speed_response == 0
```

表示理想瞬时响应：

```text
wheel_angular_feedback = wheel_angular_target
```

`speed_response` 在数学意义上等价于一阶系统时间常数，但公开接口和代码统一使用 `speed_response`，不额外引入：

```text
response_time
velocity_time_constant
tau
```

等其它名称。

#### `radius` 与 `direction`

`radius` 是该轮的有效半径，必须大于零。`direction` 必须为 `-1` 或 `1`，表示
MuJoCo wheel joint 正方向相对 Mecanum canonical 正方向的符号。公开的
`WheelAngular`、`WheelLinear` 与 wheel state 始终使用 joint 坐标；运行时仅在
FK/IK 边界应用 `direction`。

### 3.2 MobileBaseInfo

```cpp
struct MobileBaseInfo {
    MobileBaseId id{0};

    std::string mobile_base_name;

    MobileBaseType type{MobileBaseType::Mecanum};

    std::string base_frame_id{"base_link"};
    std::string odom_frame_id{"odom"};

    std::string base_body_name;
    std::string base_joint_name;

    MecanumInfo mecanum_info;
    MecanumWheelInfo mecanum_wheels;

    double period{0.0};
};
```

其中：

```cpp
struct MecanumInfo {
    double wheel_base{0.0};
    double track_width{0.0};
};
```

`wheel_base` 表示：

```text
前轮中心线与后轮中心线之间的完整距离
```

`track_width` 表示：

```text
左轮中心线与右轮中心线之间的完整距离
```

不是 half wheel base / half track width。

### 3.3 轮顺序

`mecanum_wheels` 固定顺序：

```text
FrontLeft
FrontRight
RearLeft
RearRight
```

即：

```text
FL
FR
RL
RR
```

该顺序是：

> Mecanum 运动学矩阵的 slot/index 约定。

它不是额外的软件旋转方向修正。

### 3.4 XML 配置

建议配置：

```xml
<mobile_base
    id="0"
    name="base"
    type="mecanum"
    base_body="base_link"
    base_joint="base_free"
    wheel_base="0.410"
    track_width="0.360">

  <wheel
      index="front_left"
      name="front_left_wheel_joint"
      radius="0.076"
      direction="1"
      speed_response="0.08"/>

  <wheel
      index="front_right"
      name="front_right_wheel_joint"
      radius="0.076"
      direction="1"
      speed_response="0.08"/>

  <wheel
      index="rear_left"
      name="rear_left_wheel_joint"
      radius="0.076"
      direction="1"
      speed_response="0.08"/>

  <wheel
      index="rear_right"
      name="rear_right_wheel_joint"
      radius="0.076"
      direction="1"
      speed_response="0.08"/>

</mobile_base>
```

配置文件、解析结构和运行时代码统一使用：

```text
speed_response
```

不做 XML → `response_time` → runtime `tau` 等多次重命名。

### 3.5 配置校验

初始化时必须检查以下内容。

#### Wheel

四个 `wheel_name`：

* 非空；
* 互不相同；
* 均能在 MJCF 中找到；
* 均绑定到一自由度 `hinge`；
* 必须是 unlimited hinge，不能设置 joint range；
* 均具有合法 `qpos` / `dof` 地址；
* 不能同时属于通用 `JointComponent`。

每个：

```text
radius > 0
direction ∈ {-1, 1}
speed_response >= 0
```

否则初始化失败。

#### Base

`base_joint_name`：

* 非空；
* 必须存在；
* 必须是 `free` joint；
* 必须属于 `base_body_name`。

`base_body_name`：

* 非空；
* 必须存在；
* 必须与 `base_joint_name` 的 body 对应。

#### Mecanum geometry

以下参数必须严格大于零：

```text
wheel_base
track_width
```

即：

```cpp
wheel_base > 0.0
track_width > 0.0
```

#### Actuator

纯运动学模式：

* 不要求 wheel actuator 存在；
* 不解析 wheel actuator；
* 不读取 actuator 参数；
* 不读取 damping；
* 不读取 gain；
* 不写 `data->ctrl`。

因此以下配置全部从 `MobileBase` 中删除：

```text
actuator_name
damping
gain
effort_limit
```

### 3.6 MJCF 轮坐标约定

代码不得增加：

```cpp
double sign;
```

或：

```cpp
wheel_sign[4]
```

等 per-wheel 符号配置。

对于每个 wheel：

```text
wheel_angular > 0
```

定义为：

> 对应 MuJoCo wheel hinge 的 `qvel > 0`。

Mecanum 正逆运动学矩阵本身已经定义了：

```text
FL / FR / RL / RR
```

四个 slot 的正方向语义。

因此系统约定为：

> MJCF 中每个 wheel hinge 的 `axis` 必须与 `MobileBase` 固定的 wheel coordinate convention 保持一致。

而不是由软件读取 joint axis 后自动推断方向。

换句话说：

```text
Mecanum matrix
+
wheel slot
+
MJCF joint axis
```

共同构成模型契约。

如果轮子视觉旋转方向、正向横移或旋转方向不符合约定，应修改：

* MJCF joint `axis`；
* wheel slot 配置；

而不是增加软件 `sign`。

---

## 4. 命令、轮速响应与 Mecanum 运动学

### 4.1 坐标与几何约定

底盘坐标：

```text
+x : 前
+y : 左
+z : 上
```

底盘 twist：

```text
vx : base frame 前向线速度
vy : base frame 左向线速度
wz : 绕 +z 的角速度
```

固定轮顺序：

```text
[FL, FR, RL, RR]
```

设：

```text
r_i = wheel[i].radius
L = wheel_base
W = track_width
```

因为 `L` 和 `W` 是完整轮距，所以：

```text
rotation_radius = (L + W) / 2
```

即：

```cpp
const double rotation_radius =
    (wheel_base + track_width) * 0.5;
```

不得使用：

```text
L + W
```

作为旋转系数。

### 4.2 命令模型

`MobileBaseCommand` 采用：

```text
latest-command
```

语义。

对于每个 `MobileBaseId`：

> 每个 physics step 使用最近一次通过完整校验的命令，直到收到下一条有效命令，或者执行 `reset()` / `shutdown()`。

#### Twist

只使用：

```cpp
base_linear.x
base_linear.y
base_angular.z
```

其它分量必须：

* 忽略；
* 或要求为零并校验；

具体策略应保持现有公开 API 行为一致。

经过 Mecanum IK 后得到：

```cpp
wheel_angular_target[4]
```

#### WheelLinear

输入：

```cpp
wheel_linear[4]
```

转换：

```text
wheel_angular_target[i] =
    wheel_linear[i] / wheel[i].radius
```

#### WheelAngular

输入直接作为：

```cpp
wheel_angular_target[4]
```

#### 模式切换

任何新的有效命令：

```text
Twist
WheelLinear
WheelAngular
```

都立即整体替换该底盘之前的目标。

例如：

```text
Twist
  ↓
WheelAngular
```

收到新的 `WheelAngular` 后：

```text
旧 Twist 不再参与计算
```

不存在模式间目标叠加。

### 4.3 Mecanum 逆运动学

`Twist` 模式下：

```text
[vx, vy, wz]
```

转换成：

```text
[wFL, wFR, wRL, wRR]
```

关系为：

```text
wFL = ( vx - vy - rotation_radius * wz) / r
wFR = ( vx + vy + rotation_radius * wz) / r
wRL = ( vx + vy - rotation_radius * wz) / r
wRR = ( vx - vy + rotation_radius * wz) / r
```

矩阵形式：

```text
[wFL]   1/r [ 1  -1  -rotation_radius ] [vx]
[wFR] = 1/r [ 1   1   rotation_radius ] [vy]
[wRL]   1/r [ 1   1  -rotation_radius ] [wz]
[wRR]   1/r [ 1  -1   rotation_radius ]
```

### 4.4 轮速一阶响应

每个 wheel 保存：

```cpp
double wheel_angular_target;
double wheel_angular_feedback;
```

以及配置：

```cpp
double speed_response;
```

每个 physics step：

```cpp
if (speed_response == 0.0) {
    wheel_angular_feedback = wheel_angular_target;
} else {
    const double alpha =
        1.0 - std::exp(-dt / speed_response);

    wheel_angular_feedback +=
        alpha *
        (wheel_angular_target - wheel_angular_feedback);
}
```

内部命名统一使用：

```text
speed_response
```

而不是：

```text
tau
time_constant
response_time
velocity_response
```

这是：

> 一阶轮速响应模型。

不是：

* PID；
* PD；
* motor torque model；
* electrical motor model；
* actuator dynamics；
* MuJoCo actuator filter。

它不会：

```text
计算 torque
```

也不会：

```text
写 data->ctrl
```

状态中的：

```cpp
wheel_angular
```

必须来自：

```cpp
wheel_angular_feedback
```

状态中的：

```cpp
wheel_linear
```

必须为：

```text
wheel[i].radius * wheel_angular_feedback
```

不得发布：

```text
wheel_angular_target
```

作为轮速反馈。

### 4.5 Mecanum 正运动学

对应正运动学：

```text
vx =
    r / 4 *
    (wFL + wFR + wRL + wRR)

vy =
    r / 4 *
    (-wFL + wFR + wRL - wRR)

wz =
    r / (4 * rotation_radius) *
    (-wFL + wFR - wRL + wRR)
```

正逆运动学必须使用完全相同的：

```text
rotation_radius
```

定义。

---

## 5. SE(2) 状态、MuJoCo 写入与运行时顺序

### 5.1 内部 odom 状态

`MobileBase` 内部维护：

```text
odom_x
odom_y
odom_yaw
```

表示：

```text
T_odom_base
```

其坐标系为：

```text
odom frame
```

由：

```cpp
wheel_angular_feedback
```

做 Mecanum FK 得到：

```text
(vx, vy, wz)
```

其中：

```text
vx / vy
```

始终是：

```text
base_frame
```

速度。

`wz` 是：

```text
base z axis angular velocity
```

### 5.2 SE(2) 积分

每个 physics step：

```text
yaw_next =
    wrap_pi(odom_yaw + wz * dt)

x_next =
    odom_x +
    (cos(yaw_next) * vx -
     sin(yaw_next) * vy) * dt

y_next =
    odom_y +
    (sin(yaw_next) * vx +
     cos(yaw_next) * vy) * dt
```

即使用半隐式 yaw：

```text
先更新 yaw
再用 yaw_next 积分平移
```

随后：

```text
odom_x   = x_next
odom_y   = y_next
odom_yaw = yaw_next
```

### 5.3 odom 与 MuJoCo world

不能默认：

```text
odom == MuJoCo world
```

因为 MJCF 中机器人可能具有非零初始位姿。

定义：

```text
T_world_base =
    T_world_odom * T_odom_base
```

初始化或 `reset()` 后：

```text
T_odom_base = Identity
```

即：

```text
odom_x   = 0
odom_y   = 0
odom_yaw = 0
```

同时从 MJCF 模型初始状态获得：

```text
T_world_base_initial
```

并设置：

```text
T_world_odom =
    T_world_base_initial
```

因此：

```text
T_world_base =
T_world_odom * Identity
=
T_world_base_initial
```

这样：

* 对导航接口而言初始 odom 是 `(0, 0, 0)`；
* 对 MuJoCo world 而言机器人保持 MJCF 配置的实际初始位姿；
* 下一 physics step 不会跳到 world origin。

### 5.4 写入 base free joint

`MobileBase` 根据：

```text
T_world_base
```

写入 base free joint。

free joint `qpos`：

```text
position:
    world x
    world y
    初始 world z

orientation:
    只允许 yaw
```

纯 SE(2) 模式：

* 不主动改变 z；
* roll = 0；
* pitch = 0；
* yaw 来自 `T_world_base`。

如果 MJCF 初始 base 包含必须保留的固定 roll/pitch，则应在实现前明确是否支持；当前方案推荐要求移动底盘初始姿态符合水平 SE(2) 假设。

### 5.5 写入 free joint qvel

Mecanum FK 得到：

```text
vx
vy
wz
```

其中：

```text
vx / vy
```

是 base frame。

MuJoCo free-joint 平移速度需要写为 world-frame velocity。

因此：

```text
vx_world =
    cos(world_yaw) * vx -
    sin(world_yaw) * vy

vy_world =
    sin(world_yaw) * vx +
    cos(world_yaw) * vy
```

写入：

```cpp
qvel[0] = vx_world;
qvel[1] = vy_world;
qvel[2] = 0.0;
```

free-joint 旋转速度遵循 MuJoCo 的 body-local angular velocity 约定。

当前模型只有 yaw，因此：

```cpp
qvel[3] = 0.0;
qvel[4] = 0.0;
qvel[5] = wz;
```

因为在纯 SE(2) 情况下：

```text
local +z
```

与：

```text
world +z
```

重合。

不能将这一行为推广到未来包含 roll/pitch 的 6-DoF base。

### 5.6 写入 wheel hinge

每个 wheel：

```cpp
data->qvel[wheel.dof_address] =
    wheel_angular_feedback;
```

轮子视觉旋转通过：

```cpp
data->qpos[wheel.qpos_address] +=
    wheel_angular_feedback * dt;
```

进行积分。

对于 unlimited hinge，可周期性对视觉角度归一化以避免长时间运行产生过大的数值，例如：

```text
[-pi, pi)
```

或：

```text
[-2pi, 2pi)
```

但该归一化不能改变：

```text
wheel_angular_feedback
```

的语义。

### 5.7 physics step 执行顺序

底盘采用：

> **post-step kinematic override**

语义。

每个 physics step 固定执行：

```text
1. 读取 CommandBuffer latest snapshot

2. 将有效 MobileBaseCommand
   转换成 wheel_angular_target

3. 执行 mj_step()
   推进 MuJoCo 其它动态部分

4. ComponentManager::advance(context)

   其中 `MobileBaseComponent` 覆写通用的 `SimulationComponent::advance()`，推进自身运动学。

   4.1 更新 speed_response
       -> wheel_angular_feedback

   4.2 Mecanum FK
       -> vx / vy / wz

   4.3 推进 T_odom_base

   4.4 计算 T_world_base

   4.5 写 wheel qpos/qvel

   4.6 写 base free joint qpos/qvel

5. mj_forward()

6. 按各组件 period 采样状态

7. 写入 StateBuffer
```

这种顺序必须由集成测试固定。

不得存在两个可能顺序：

```text
MobileBase -> mj_step
```

和：

```text
mj_step -> MobileBase
```

同时作为合法实现。

### 5.8 `advance()` 与 `update()`

`SimulationComponent` 提供每个 physics step 调用的通用生命周期方法：

```cpp
virtual bool advance(const mjContext& context);
```

该方法通过 `context.data` 写入 MuJoCo 状态，并使用：

```cpp
const double dt = context.model->opt.timestep;
```

获得 physics step 时长。若组件未初始化、MuJoCo binding 缺失或状态写入条件不满足，
必须返回 `false`，由调度器中止本次仿真 step。

其职责：

```text
更新目标
        ↓
speed_response
        ↓
wheel_angular_feedback
        ↓
Mecanum FK
        ↓
SE(2)
        ↓
wheel/base qpos/qvel
```

它与：

```cpp
update()
```

必须严格区分。

`MobileBaseComponent::advance()`：

* 每个 physics step 调用；
* 负责运动学；
* 负责一阶轮速响应；
* 负责 wheel angle 积分；
* 负责 base SE(2) 积分；
* 负责写 MuJoCo state。

`update()`：

* 由 `MobileBaseInfo::period` 控制；
* 只负责生成 `MobileBaseState`；
* 只负责发布 state snapshot；
* 不得负责推进运动。

否则：

```text
period = 20 ms
```

和：

```text
period = 100 ms
```

会得到不同轨迹，这是不可接受的。

---

## 6. 状态、命令生命周期与错误处理

### 6.1 MobileBaseState

字段语义统一定义如下：

| 字段              | 来源                                      | 坐标系                      |
| --------------- | --------------------------------------- | ------------------------ |
| `pose`          | `T_odom_base`                           | `odom_frame_id`          |
| `base_linear`   | Mecanum FK 的 `(vx, vy, 0)`              | `base_frame_id`          |
| `base_angular`  | Mecanum FK 的 `(0, 0, wz)`               | `base_frame_id`          |
| `wheel_angular` | `wheel_angular_feedback`                | wheel joint coordinate   |
| `wheel_linear`  | `wheel[i].radius * wheel_angular_feedback` | wheel joint coordinate |
| `timestamp`     | 状态发布时的 simulation time                  | simulation clock         |

必须满足：

```text
wheel_angular
      |
      v
Mecanum FK
      |
      v
base_linear / base_angular
      |
      v
SE(2) 积分
      |
      v
pose
```

即所有状态属于同一个运动学模型。

禁止：

```text
pose         <- MuJoCo body xpos
base_twist   <- wheel feedback FK
wheel state  <- target command
```

这种混合来源。

### 6.2 reset()

`reset()` 必须：

1. 恢复 MuJoCo 模型初始 base free joint；
2. 恢复四个 wheel hinge 的初始状态；
3. 重建 `T_world_odom`；
4. 设置：

```text
T_odom_base = Identity
```

5. 清零：

```cpp
wheel_angular_target
wheel_angular_feedback
base_linear
base_angular
```

6. 清除该底盘缓存的 latest command；
7. 调用：

```cpp
mj_forward()
```

完成后：

```text
base twist = 0
wheel speed = 0
odom pose = Identity
```

后续没有新命令时不得自行恢复 reset 前的速度。

### 6.3 shutdown()

`shutdown()` 应清除：

* 当前命令；
* `wheel_angular_target`；
* `wheel_angular_feedback`；
* 内部 SE(2) 状态；
* MuJoCo binding information；
* cached component state。

不允许 shutdown 后重新 initialize 时继承上一 generation 的底盘速度。

### 6.4 无效命令

以下命令均视为无效：

* 未知 `MobileBaseId`；
* 未知控制模式；
* 数组尺寸错误；
* `NaN`；
* `Inf`；
* 非有限 twist；
* 非有限 wheel speed。

命令必须：

> 完整校验后一次性提交。

禁止：

```text
先修改 FL
发现 FR 无效
返回失败
```

导致部分目标已被更新。

因此推荐：

```text
validate
   ↓
转换到临时 wheel_angular_target
   ↓
全部成功
   ↓
commit
```

---

## 7. 接触、动力学与传感器语义

### 7.1 Wheel-ground contact

纯运动学底盘不允许 wheel-ground contact 决定底盘轨迹。

如果 wheel collision geom 不需要任何碰撞：

```xml
<geom
    name="front_left_wheel_collision"
    type="cylinder"
    contype="0"
    conaffinity="0"
    .../>
```

这样禁用该 geom 的所有 contact。

如果 wheel 仍需要与其它对象接触，则应通过：

```text
contype
conaffinity
```

位掩码仅排除：

```text
wheel <-> ground
```

而不是使用全局 contact exclude。

仅用于显示的轮子可使用 visual geom。

需要注意：

```xml
discardvisual="true"
```

可能使纯 visual geom 在模型编译阶段被移除。

### 7.2 Base 与环境碰撞

建议采用：

> **底盘自身轨迹 authoritative，但底盘 geom 仍可影响外部动态对象。**

例如：

```text
运动学底盘撞到动态箱子
      |
      +--> 箱子可以受到 MuJoCo 接触作用
      |
      +--> 底盘最终轨迹仍由 MobileBase 覆盖
```

因此：

```text
外部物体不能通过碰撞改变底盘轨迹
```

这实际上是一种：

> infinite-authority kinematic base

模型。

如果某具体场景不希望底盘与环境产生任何接触，可进一步关闭 chassis collision。

### 7.3 与机械臂动力学的关系

机械臂等其它自由度仍由：

```cpp
mj_step()
```

推进。

但必须明确：

> MobileBase 的最终 qpos/qvel 是在 `mj_step()` 之后覆盖得到的，因此该模型不等价于 MuJoCo 中严格 prescribed 的动态移动基座。

所以该方案适用于：

* Nav2；
* MoveIt；
* 普通移动操作；
* policy training；
* perception；
* deterministic navigation simulation。

但不应用于验证：

* 高速底盘加减速下机械臂惯性；
* 基座运动产生的精确 Coriolis / inertial coupling；
* dynamic whole-body control；
* reaction force；
* base 被机械臂反作用力推动。

需要这些能力时应设计单独的动力学移动底盘模型。

### 7.4 `mj_forward()` 与传感器

完成 base/wheel 状态覆盖后调用：

```cpp
mj_forward(model, data);
```

用于刷新：

* `xpos`；
* `xquat`；
* `xmat`；
* velocity-dependent quantities；
* sensor derived quantities；
* constraint-related derived data。

但是必须注意：

> `mj_forward()` 根据当前 MuJoCo state 和 dynamics model 重新计算加速度等动力学量。

因此，对于：

```text
accelerometer
frame acceleration
```

等依赖加速度的 sensor，其结果不一定严格对应 `speed_response` 运动学轨迹的解析加速度。

如果以后底盘导航需要高一致性的 IMU，建议单独定义：

```text
Kinematic IMU
```

由 MobileBase 的：

```text
pose
base twist
twist difference
```

计算，而不是默认将 MuJoCo 动力学 accelerometer 当成纯运动学底盘的严格真值。

---

## 8. 代码实现与迁移方案

### 8.1 Runtime 数据结构

运行时 wheel binding 可以设计为：

```cpp
struct WheelRuntime {
    int joint_id{-1};
    int qpos_address{-1};
    int dof_address{-1};

    double speed_response{0.0};

    double wheel_angular_target{0.0};
    double wheel_angular_feedback{0.0};
};
```

这里继续使用：

```cpp
speed_response
```

和公开配置保持一致。

不要改名为：

```text
response
tau
time_constant
filter_constant
```

内部 MobileBase runtime：

```cpp
struct MobileBaseRuntime {
    int base_body_id{-1};
    int base_joint_id{-1};
    int base_qpos_address{-1};
    int base_dof_address{-1};

    std::array<WheelRuntime, 4> wheels;

    double odom_x{0.0};
    double odom_y{0.0};
    double odom_yaw{0.0};

    double world_odom_x{0.0};
    double world_odom_y{0.0};
    double world_odom_yaw{0.0};

    Vector3 base_linear{};
    Vector3 base_angular{};
};
```

如果项目已有统一 Pose/Transform 类型，应优先复用，不必人为拆成多个 `double`。

### 8.2 实施步骤

#### Step 1：收敛配置

修改：

```cpp
struct WheelInfo {
    std::string wheel_name;
    double speed_response{0.0};
};
```

删除：

```text
actuator_name
damping
sign
```

增加：

```cpp
base_joint_name
```

#### Step 2：修改 XML

统一采用：

```xml
speed_response="0.08"
```

删除 wheel actuator 依赖配置。

#### Step 3：修改解析和校验

校验：

* four wheel hinge；
* base free joint；
* wheel uniqueness；
* geometry；
* `speed_response >= 0`；
* wheel ownership conflict。

#### Step 4：重写 wheel binding

删除：

```text
mjWheel
actuator lookup
actuator id
ctrl range
data->ctrl
wheel PD
damping
```

改为绑定：

```text
joint id
qpos address
dof address
```

#### Step 5：覆写 `advance()`

```cpp
advance(context)
```

每 physics step 执行：

```text
command
 -> target
 -> speed_response
 -> feedback
 -> FK
 -> SE(2)
 -> wheel/base state override
```

#### Step 6：调整 scheduler

固定：

```text
mj_step
 -> ComponentManager::advance
 -> mj_forward
 -> state publication
```

不得由状态 `period` 决定运动积分。

#### Step 7：调整 State

`MobileBaseState` 全部从：

```text
internal odom
wheel_angular_feedback
Mecanum FK
```

产生。

移除原来的：

```text
body xpos/xmat 作为 pose 真值
```

逻辑。

#### Step 8：reset/shutdown

保证：

```text
target = 0
feedback = 0
twist = 0
```

并正确恢复：

```text
T_world_odom
T_odom_base
```

#### Step 9：修改 MJCF 示例

* base 提供 free joint；
* wheel hinge 保留；
* wheel actuator 可删除；
* wheel-ground collision 关闭；
* 修正 wheel axis 与 wheel slot convention。

#### Step 10：清理旧设计

删除或改写：

```text
wheel PD
wheel damping
actuator torque
friction-driven base
ground-truth body pose
```

相关文档、测试和注释。

---

## 9. 验收测试、限制与后续扩展

### 9.1 配置测试

以下情况初始化失败：

* 缺 wheel；
* wheel 重复；
* wheel 非 hinge；
* wheel 不存在；
* base joint 缺失；
* base joint 非 free；
* base body 不匹配；
* 任一 `wheel.radius <= 0`；
* `wheel_base <= 0`；
* `track_width <= 0`；
* `speed_response < 0`；
* wheel 同时属于 `JointComponent`。

以下情况必须成功：

```text
wheel actuator 完全不存在
```

### 9.2 运动学测试

#### 正向运动

给定：

```text
wFL = wFR = wRL = wRR > 0
```

应得到：

```text
vx > 0
vy = 0
wz = 0
```

并且 world 轨迹与当前 yaw 一致。

#### 横移

给定匹配横移的四轮速度组合，应得到：

```text
vx = 0
vy != 0
wz = 0
```

符号符合：

```text
+y = left
```

#### 原地旋转

给定匹配旋转的 wheel speed，应得到：

```text
vx = 0
vy = 0
wz != 0
```

并验证旋转系数：

```text
rotation_radius =
    (wheel_base + track_width) / 2
```

### 9.3 `speed_response` 测试

对于固定目标：

```text
wheel_angular_target
```

验证：

```cpp
alpha =
    1.0 - std::exp(-dt / speed_response);
```

响应结果符合解析一阶曲线。

当：

```text
speed_response == 0
```

时，一个 physics step 内：

```text
feedback == target
```

### 9.4 命令测试

一次 Twist 命令后，不再发送新命令：

```text
target
```

必须跨多个 physics step 保持。

同一 MobileBase：

```text
Twist
  ↓
WheelLinear
  ↓
WheelAngular
```

任意新有效命令必须立即完整替换旧目标。

如果系统存在多个 `MobileBase`：

```text
base A command
```

不得修改：

```text
base B target / feedback / pose
```

### 9.5 状态一致性测试

必须验证：

```text
wheel_linear[i] ==
    wheel[i].radius * wheel_angular[i]
```

以及：

```text
wheel_angular
 -> Mecanum FK
```

等于发布：

```text
base_linear
base_angular
```

并且 pose 差分与对应 base twist 在离散积分误差范围内一致。

### 9.6 周期独立性

相同：

```text
physics dt
command sequence
speed_response
```

分别设置：

```text
period = 10 ms
period = 50 ms
period = 100 ms
```

最终轨迹必须相同。

仅允许：

```text
状态发布时间点不同
```

### 9.7 reset 测试

运动过程中调用：

```cpp
reset();
```

随后验证：

```text
wheel_angular_target   == 0
wheel_angular_feedback == 0
base_linear            == 0
base_angular           == 0
odom pose              == Identity
```

且没有新命令时继续 step：

```text
底盘保持停止
```

不得自动恢复 reset 前的速度。

### 9.8 MJCF 非零初始位姿测试

设置：

```text
world x/y/yaw != 0
```

初始化后：

```text
MobileBaseState.pose = Identity
```

但：

```text
MuJoCo base world pose
```

仍应等于 MJCF 初始位姿。

第一次 physics step 不得出现跳变。

### 9.9 free-joint qvel 坐标系测试

令底盘：

```text
world yaw = 90 deg
```

同时：

```text
vx > 0
vy = 0
```

验证：

```text
world qvel.x ≈ 0
world qvel.y > 0
```

不能直接把 body-frame `vx` 写到 world `qvel.x`。

### 9.10 接触隔离测试

关闭 wheel-ground contact 后：

使用不同：

```text
ground friction
wheel friction
```

运行完全相同命令序列。

底盘：

```text
pose
twist
wheel feedback
```

必须完全一致或在浮点容差内一致。

### 9.11 限制与后续扩展

当前模型不会产生：

* wheel slip；
* tire force；
* traction limit；
* suspension motion；
* slope traction loss；
* wheel torque saturation；
* 被外力推动后的被动底盘运动；
* 严格的运动基座动力学耦合。

如果以后需要这些能力，应增加独立：

```text
DynamicMobileBase
```

或其它动力学模式。

不得同时启用：

```text
纯运动学 SE(2) override
```

和：

```text
wheel contact 驱动同一 base DOF
```

否则会形成两个相互冲突的运动来源。

未来可以在保持当前核心原则的前提下扩展：

* swerve；
* differential drive；
* odom noise；
* encoder quantization；
* wheel failure；
* speed bias；
* command delay；
* per-wheel fault injection。

其中每轮独立：

```text
speed_response
```

当前设计已经支持，无需后续修改公开数据结构。
