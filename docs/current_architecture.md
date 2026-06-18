# robot_mujoco 当前架构快照

当前 baseline 入口与样例格式见 [`performance_baseline.md`](./performance_baseline.md) 和
[`performance_baseline.sample.json`](./performance_baseline.sample.json)。运行时/下游 API
迁移入口见 [`migration_guide.md`](./migration_guide.md)。

## 1. 当前包边界

截至 2026-06-18，`robot_mujoco` 工作区内与 MuJoCo 运行时直接相关的 C++ 包边界固定为：

- `mujoco_simulation`
  - 纯 MuJoCo 运行时内核。
  - 负责 `Simulation`、`Status/Result`、`ModelRuntime`、`SimulationScheduler`、`ComponentManager`、`CameraRenderer`、`Viewer`。
  - 不依赖 `rclcpp`、ROS message header 或 `hardware_interface`。
- `robot_mujoco_ros2`
  - 统一 ROS 2 adapter 包。
  - 同时负责 `ros2_control` `SystemInterface`、`HardwareInfo -> HardwareConfig` 解析、`Simulation` 生命周期、state/command interface、模式切换、`/clock`、IMU/Camera/Lidar publisher、simulation control services、`Status -> ROS response` 适配、ROS node/executor 生命周期与 pluginlib 导出。
- `robot_mujoco`
  - 工作区聚合入口。
  - 提供头文件聚合、launch/config、Python 侧场景封装。

## 2. 主调用链

```text
ros2_control controller manager
  -> robot_mujoco_ros2::MuJoCoHardwareInterface
    -> mujoco_simulation::Simulation
      -> runtime::ModelRuntime
      -> runtime::SimulationScheduler
      -> component::ComponentManager
      -> buffer::CommandBuffer / StateBuffer / CameraBuffer

MuJoCoHardwareInterface
  -> robot_mujoco_ros2::SimulationRosBridge
    -> /clock
    -> sensor publishers
    -> simulation control services
```

## 3. 线程模型

- `SimulationScheduler` 工作线程
  - 唯一允许写主 `mjData` 的线程。
  - 负责 command flush、physics step、sensor sample、state publish、viewer sync。
- 调用方线程
  - 通过 `set_joint_command()`、`set_mobile_base_command()`、`request_reset()` 等接口提交请求。
  - 通过 `state_snapshot()`、`camera_sample()`、`joint_state()` 等只读接口取样。
- `SimulationRosBridge` executor 线程
  - 专用于 ROS publisher/service 回调。
  - 不直接访问主 `mjData`。

## 4. 数据流

### 4.1 控制写入

```text
controller command
  -> MuJoCoHardwareInterface::write()
    -> Simulation::set_joint_command() / set_mobile_base_command()
      -> CommandBuffer
      -> SimulationScheduler worker flush
```

### 4.2 状态读取

```text
SimulationScheduler worker
  -> ComponentManager::read_* / sample_sensors()
  -> StateBuffer publish snapshot
  -> CameraBuffer publish shared CameraSample

MuJoCoHardwareInterface::read()
  -> Simulation::state_snapshot()
  -> Simulation::camera_sample()
  -> SimulationRosBridge::publish_*
```

### 4.3 ROS 控制服务

```text
/start /stop /pause /resume /step /set_realtime_factor /load_keyframe /reset
  -> SimulationRosBridge service callback
  -> Status callback provided by MuJoCoHardwareInterface
  -> Simulation public API
  -> Status.message() mapped back to ROS response
```

## 5. 公开运行时接口

`mujoco_simulation::Simulation` 当前对下游承诺的核心接口保持为：

- 生命周期：`initialize()`、`shutdown()`
- 控制：`start()`、`stop()`、`pause()`、`resume()`、`step()`、`reset()`、`request_reset()`、`set_realtime_factor()`
- 设备命令：`reconfigure_component()`、`set_joint_command()`、`set_mobile_base_command()`
- 设备读取：`joint_state()`、`imu_sample()`、`lidar_sample()`、`camera_sample()`、`mobile_base_state()`
- 快照：`state_snapshot()`
- 错误模型：`Status` / `Result<T>` + `StatusCode`

其中：

- `reset()` / `request_reset()` 统一通过 `ResetOptions` 表达 keyframe 与缓存清理策略
- `reconfigure_component()` 是唯一公开运行时组件重配置入口；`robot_mujoco_ros2` 的 joint mode switch 已通过更新后的 `JointConfig` 走该入口

## 6. 当前验证状态

当前代码树已验证：

- `colcon build --packages-select mujoco_simulation robot_mujoco_ros2 robot_mujoco`
- `colcon test --packages-select mujoco_simulation robot_mujoco_ros2 --return-code-on-test-failure`
- 基线采集入口固定为：
  - `mujoco_simulation/test/performance/perf_runtime_scenarios.cpp`
  - `robot_mujoco_ros2/tests/performance/perf_read_write_loop.cpp`
  - `mujoco_simulation/test/performance/run_baseline.py`
