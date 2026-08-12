# MuJoCo 通用机器人关节仿真设计方案

## 1. 设计目标

构建基于 MuJoCo 的通用机器人关节仿真框架，支持：

-   人形机器人
-   双臂机器人
-   移动操作机器人
-   工业机械臂
-   轮式机器人
-   线性执行机构

设计目标：

1.  控制接口与真实机器人控制思想一致；
2.  MuJoCo 负责动力学执行；
3.  支持 Position、Velocity、Effort、Hybrid 四种控制模式；
4.  支持 Revolute 和 Prismatic 两类关节；
5.  使用 stiffness/damping 表达关节阻抗参数。

------------------------------------------------------------------------

# 2. 总体架构

    Application
    (MoveIt / MPC / WBC / RL)

            |

    JointCommand

            |

    Joint Controller

            |

    Safety Layer

            |

    Generalized Effort

            |

    MuJoCo Motor Actuator

            |

    Physics Simulation

            |

    JointState

------------------------------------------------------------------------

# 3. Joint Type

由于 MJCF 已经描述具体关节类型，通用接口只保留：

``` cpp
enum class JointType
{
    Revolute,
    Prismatic
};
```

## Revolute

旋转关节。

对应 MuJoCo：

    hinge

单位：

-   position: rad
-   velocity: rad/s
-   effort: Nm

应用：

-   机械臂关节
-   人形机器人关节
-   轮子

## Prismatic

直线关节。

对应 MuJoCo：

    slide

单位：

-   position: m
-   velocity: m/s
-   effort: N

应用：

-   升降机构
-   伸缩机构

------------------------------------------------------------------------

# 4. Joint Limit

MJCF 已经负责：

-   关节范围；
-   位置约束；
-   无限旋转配置。

MuJoCo 负责模型级的关节范围和约束；控制层仍在 `JointInfo` 中维护
`JointLimit`，分别对 position、velocity 和 effort 命令执行安全钳制。两者共同生效：
控制层先限制命令，MuJoCo 再执行模型约束。

例如：

有限旋转：

    hinge + range

无限旋转：

    hinge without range

控制层仅维护安全限制：

``` yaml
position:
  min:
  max:

velocity:
  min:
  max:

effort:
  min:
  max:
```

------------------------------------------------------------------------

# 5. Actuator设计

不对外暴露 ActuatorType。

所有控制模式最终转换为：

    effort

MuJoCo 使用：

    <motor>

作为统一执行器。

------------------------------------------------------------------------

# 6. JointMode

``` cpp
enum class JointMode
{
    Position,
    Velocity,
    Effort,
    Hybrid
};
```

------------------------------------------------------------------------

# 7. JointCommand

``` cpp
struct JointCommand
{
    uint8_t mode;  // 0: Position, 1: Velocity, 2: Effort, 3: Hybrid

    double position;

    double velocity;

    double effort;

    double stiffness;

    double damping;
};
```

字段说明：

  字段        说明
  ----------- ------------
  position    目标位置
  velocity    目标速度
  effort      前馈广义力
  stiffness   位置刚度
  damping     速度阻尼

------------------------------------------------------------------------

# 8. JointState

``` cpp
struct JointState
{
    uint8_t mode;  // 0: Position, 1: Velocity, 2: Effort, 3: Hybrid

    double position;

    double velocity;

    double effort;
};
```

------------------------------------------------------------------------

# 9. Position模式

## 输入

    mode = Position

    position = q_des

## 参数

每个关节独立配置：

``` yaml
left_shoulder_pitch:
  position:
    stiffness: 300
    damping: 10
```

可选属性 `gravity_compensation` 用于在控制律中叠加随当前姿态变化的重力补偿：

``` xml
<position stiffness="2000" damping="200" gravity_compensation="true"/>
```

缺省为 `false`，以保持已有模型行为不变。

## 控制

$$
effort = \tau_g + K_p(q_d - q) + K_d(\dot{q}_d - \dot{q})
$$

其中：

$$
K_p = stiffness
$$

$$
K_d = damping
$$

仅在 `gravity_compensation="true"` 时叠加 $\tau_g$；Position 模式的
$\dot{q}_d$ 为 0。

------------------------------------------------------------------------

# 10. Velocity模式

## 输入

    mode = Velocity

    velocity = dq_des

## 参数

每个关节独立配置：

``` yaml
wheel_left:
  velocity:
    damping: 20
```

## 控制

$$
effort = \tau_g + K_d(\dot{q}_d - \dot{q})
$$

其中 $\tau_g$ 仅在 `gravity_compensation="true"` 时叠加。

其中：

    stiffness = 0

------------------------------------------------------------------------

# 11. Effort模式

## 输入

    mode = Effort

    effort = command

控制：

$$
effort = effort_{cmd}
$$

Effort 模式默认不补偿重力，避免与上层已计算的补偿重复。只有显式配置
`<effort gravity_compensation="true"/>` 时，控制输出才为
$effort_{cmd} + \tau_g$。

------------------------------------------------------------------------

# 12. Hybrid模式

用于：

-   人形机器人；
-   双臂机器人；
-   全身控制；
-   强化学习。

## 输入

    position

    velocity

    effort

    stiffness

    damping

## 控制

$$
effort = \tau_g + effort_{ff} + K_p(q_d - q) + K_d(\dot{q}_d - \dot{q})
$$

其中 $\tau_g$ 仅在 `gravity_compensation="true"` 时叠加。

------------------------------------------------------------------------

# 13. 参数配置

每个关节独立配置。

示例：

``` yaml
joints:

  left_shoulder_pitch:
    position:
      stiffness: 300
      damping: 10

  wheel_left:
    velocity:
      damping: 20

  elevator:
    position:
      stiffness: 500
      damping: 30
```

`robot_mujoco.xml` 中的实际配置放在 `<joint><control>` 的各模式节点内，例如：

``` xml
<joint id="17" name="elevator_joint" actuator="elevator_joint_motor" mode="position" period="0.001">
  <control>
    <position stiffness="2000" damping="200" gravity_compensation="true"/>
    <hybrid stiffness="2000" damping="200" gravity_compensation="true"/>
    <velocity damping="100" gravity_compensation="true"/>
    <effort gravity_compensation="false"/>
  </control>
</joint>
```

内核以当前 MuJoCo 姿态计算 $\tau_g$，随后与控制项相加，并依次应用 joint
effort、actuator force 和 ctrl 限幅。升降等承载关节可在启用后适当降低 Position
模式的刚度和阻尼，减少纯高刚度保持造成的振荡与冲击。

------------------------------------------------------------------------

# 14. 示例机器人映射

## 双臂

    JointType:
    Revolute

    JointMode:
    Hybrid

使用：

    position
    velocity
    stiffness
    damping
    effort

------------------------------------------------------------------------

## 底盘轮

    JointType:
    Revolute

    MJCF:
    hinge without range

    JointMode:
    Velocity

------------------------------------------------------------------------

## 升降机构

    JointType:
    Prismatic

    JointMode:
    Position

------------------------------------------------------------------------

# 15. 最终设计

    Joint

     |
     +-- JointType
     |
     +-- MJCF Joint Definition


    Controller

     |
     +-- JointMode

     |
     +-- Joint Parameters


    Output

     |
     +-- effort


    MuJoCo

     |
     +-- motor actuator

------------------------------------------------------------------------

# 16. 总结

最终支持：

## JointType

    Revolute
    Prismatic

## JointMode

    Position
    Velocity
    Effort
    Hybrid

## JointCommand

``` cpp
uint8_t mode;  // 0: Position, 1: Velocity, 2: Effort, 3: Hybrid

double position;

double velocity;

double effort;

double stiffness;

double damping;
```

## JointState

``` cpp
uint8_t mode;  // 0: Position, 1: Velocity, 2: Effort, 3: Hybrid

double position;

double velocity;

double effort;
```

该方案：

-   接近 Isaac Sim/Lab 的 stiffness/damping 表达；
-   保留 Unitree Hybrid 控制思想；
-   与 MuJoCo MJCF 解耦；
-   支持人形机器人、机械臂、底盘和升降机构；
-   便于映射真实机器人控制接口。
