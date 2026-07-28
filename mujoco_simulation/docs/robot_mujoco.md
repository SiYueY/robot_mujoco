# MuJoCo 通用机器人关节仿真描述格式设计需求

## 1. 设计目标

设计基于 MuJoCo 的通用机器人关节仿真描述格式。

支持： - 人形机器人 - 双臂机器人 - 工业机械臂 - 移动操作机器人 -
轮式底盘 - 升降机构

目标： - 单一机器人描述文件入口； - 保持 MJCF 原生兼容； - 控制配置与
MJCF 关联； - 支持仿真与真实机器人控制接口统一。

------------------------------------------------------------------------

## 2. 文件结构

入口文件：

    robot_mujoco.xml

结构：

    robot_mujoco.xml

    <robot_mujoco>

        <mujoco>
            MJCF文件引用
        </mujoco>

        <robot>
            控制配置
        </robot>

    </robot_mujoco>

------------------------------------------------------------------------

## 3. MuJoCo模型引用

``` xml
<mujoco>
    <mjcf>
        ./robot.xml
    </mjcf>
</mujoco>
```

应用程序解析该路径，然后调用：

    mj_loadXML()

加载 MJCF。

------------------------------------------------------------------------

## 4. Joint设计

joint name 必须与 MJCF 完全一致。

MJCF：

``` xml
<joint name="left_shoulder_pitch" type="hinge"/>
```

扩展配置：

``` xml
<joint name="left_shoulder_pitch">
</joint>
```

不重复定义 joint type。

类型由 MJCF 获取：

  MuJoCo        类型
  ------------- -----------
  mjJNT_HINGE   Revolute
  mjJNT_SLIDE   Prismatic

------------------------------------------------------------------------

## 5. JointControlMode

``` cpp
enum class JointControlMode
{
    Position,
    Velocity,
    Effort,
    Hybrid
};
```

------------------------------------------------------------------------

## 6. JointCommand

``` cpp
struct JointCommand
{
    JointControlMode mode;

    double position;

    double velocity;

    double effort;

    double stiffness;

    double damping;
};
```

------------------------------------------------------------------------

## 7. JointState

``` cpp
struct JointState
{
    JointControlMode mode;

    double position;

    double velocity;

    double effort;
};
```

------------------------------------------------------------------------

## 8. 控制参数

采用：

-   stiffness
-   damping

替代：

-   kp
-   kd

对应：

    stiffness = kp
    damping   = kd

------------------------------------------------------------------------

## 9. Position模式

输入：

    position

配置：

``` xml
<position>
    <stiffness>300</stiffness>
    <damping>10</damping>
</position>
```

控制：

    effort = stiffness*(q_des-q)
           + damping*(dq_des-dq)

------------------------------------------------------------------------

## 10. Velocity模式

输入：

    velocity

配置：

``` xml
<velocity>
    <damping>20</damping>
</velocity>
```

控制：

    effort = damping*(dq_des-dq)

------------------------------------------------------------------------

## 11. Effort模式

输入：

    effort

直接输出：

    effort = command

------------------------------------------------------------------------

## 12. Hybrid模式

输入：

    position
    velocity
    effort
    stiffness
    damping

控制：

    effort =
    effort_ff
    + stiffness*(q_des-q)
    + damping*(dq_des-dq)

------------------------------------------------------------------------

## 13. Limit设计

采用统一元素形式：

``` xml
<limit>

    <position>
        <min>-2.8</min>
        <max>2.8</max>
    </position>
    <velocity>
        <max>5</max>
    </velocity>
    <effort>
        <max>100</max>
    </effort>
</limit>
```

------------------------------------------------------------------------

## 14. XML规范

### 参数值

使用：

``` xml
<tag>
    value
</tag>
```

例如：

``` xml
<stiffness>300</stiffness>
```

### 属性

用于：

-   名称；
-   默认值；
-   标识。

例如：

``` xml
<joint name="xxx"/>

<mode default="hybrid"/>
```

------------------------------------------------------------------------

## 15. Actuator设计

不对外暴露 ActuatorType。

所有控制模式最终输出：

    effort

通过：

    mjData.ctrl

驱动：

    <motor>

------------------------------------------------------------------------

## 16. 核心原则

MJCF负责：

-   body；
-   joint；
-   actuator；
-   sensor；
-   physics。

robot_mujoco负责：

-   控制模式；
-   stiffness/damping；
-   安全限制；
-   软件配置。

核心原则：

> MJCF 是机器人模型唯一来源；robot_mujoco 是机器人软件配置扩展。

------------------------------------------------------------------------

## 17. 当前 v1 已支持的 XML 子集

`mujoco_simulation::SimulationConfigParser::load_file(...)` 当前只解析以下内容：

- 根节点 `<robot_mujoco>`
- `<mujoco><mjcf>...</mjcf></mujoco>`
- 必需的实时调度配置：
  ```xml
  <simulation>
    <physics period="0.001"/>
    <viewer period="0.0166666667"/>
  </simulation>
  ```
  `physics.period` 覆盖 MJCF timestep，并同时作为固定 1.0 实时因子的墙钟物理周期。
- `<robot>` 下的 Joint、IMU、Camera、Lidar、MobileBase 配置；每个组件具有显式 `id` 与类型内唯一名称
  - 各组件可选使用 `period="秒"` 指定采样周期；`period="0"` 表示每个 physics step 更新。
    正周期必须是 `physics.period` 的整数倍且不短于它。`update_rate`（Hz）已删除且会被拒绝。
- `<position><stiffness>` / `<position><damping>`
- `<velocity><damping>`
- `<limit><position|velocity|effort><min|max>`

当前未解析：

- `initial_keyframe`
- realtime factor、无节流运行模式、命令延迟与命令超时
- 任何未在本文档中列出的扩展标签

未支持的元素会直接报错，不会静默忽略。

------------------------------------------------------------------------

## 18. 路径解析规则

`<mjcf>` 内如果是相对路径，则始终相对 `robot_mujoco.xml` 文件所在目录解析。

例如：

``` xml
<robot_mujoco>
    <mujoco>
        <mjcf>../model/franka_fr3.xml</mjcf>
    </mujoco>
</robot_mujoco>
```

如果配置文件位于：

    /path/to/config/robot_mujoco.xml

则最终 MJCF 路径解析为：

    /path/to/model/franka_fr3.xml
