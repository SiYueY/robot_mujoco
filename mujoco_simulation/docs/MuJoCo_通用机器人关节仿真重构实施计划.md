# MuJoCo 通用机器人关节仿真重构实施计划

## 目标

本计划以 `MuJoCo_通用机器人关节仿真设计方案.md` 为唯一目标设计，采用破坏性迁移：每条命令携带控制模式，所有模式在运行时内核计算为广义 effort，并仅通过 MuJoCo `<motor>` 执行。

## 已实施范围

1. 关节公共模型收敛为 `Revolute`、`Prismatic` 和 `Position`、`Velocity`、`Effort`、`Hybrid` 四种控制模式。
2. `JointCommand` 包含 mode、position、velocity、effort、stiffness、damping；`JointState` 回传最后一次成功执行的 mode。
3. `JointConfig` 提供 Position/Velocity 固定阻抗以及独立的位置、速度、effort 安全范围；MJCF 继续管理物理关节范围。
4. `JointComponent` 只绑定 hinge/slide 与 `<motor>`，在写入 `ctrl` 前计算、限制 effort。
5. ROS 2 适配层按关节维护激活接口集合。标准 position/velocity/effort 单接口对应基本模式；position、velocity、effort、stiffness、damping 五接口同时激活对应 Hybrid。
6. FR3 与 TurtleBot MJCF 改用 `<motor>`；FR3 URDF 暴露 Hybrid 接口，TurtleBot 继续暴露 Velocity 接口。

## 验收矩阵

| 场景 | 验收条件 |
| --- | --- |
| Revolute/Prismatic | hinge 和 slide 均能绑定 motor 并写入 effort |
| 四种模式 | Position、Velocity、Effort、Hybrid 都通过同一 motor 输出 |
| 安全限制 | 每类命令范围与 motor 的 ctrl/force 范围均生效 |
| ROS 混合机器人 | 不同关节可以分别激活 Hybrid、Velocity、Position |
| 模型迁移 | FR3/TurtleBot 不再声明 position 或 velocity actuator |

## 验证命令

```bash
colcon build --packages-select mujoco_simulation robot_mujoco_ros2 robot_mujoco
colcon test --packages-select mujoco_simulation robot_mujoco_ros2
colcon test-result --all --verbose --test-result-base build
```
