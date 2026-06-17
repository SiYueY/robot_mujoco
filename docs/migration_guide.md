# robot_mujoco 迁移指南

当前包边界快照见 [`current_architecture.md`](./current_architecture.md)，性能/soak
baseline 入口与样例见 [`performance_baseline.md`](./performance_baseline.md) 和
[`performance_baseline.sample.json`](./performance_baseline.sample.json)。

## 1. 适用范围

本指南面向仍按旧包边界或旧顶层类型接入 `robot_mujoco` 的下游代码。当前推荐入口如下：

- 运行时 API：`mujoco_simulation::Simulation`
- ROS bridge：`mujoco_simulation_ros::SimulationRosBridge`
- ros2_control 适配：`mujoco_hardware::MuJoCoHardwareInterface`

更细的 `Simulation` API 迁移说明见 [`mujoco_simulation/docs/simulation_api_migration.md`](../mujoco_simulation/docs/simulation_api_migration.md)。

## 2. 包边界迁移

### 2.1 从 `mujoco_hardware::SensorBridge` 迁移

旧结构：

```text
mujoco_hardware
  -> SensorBridge
  -> ROS publishers / services
```

新结构：

```text
mujoco_hardware
  -> SimulationRosBridgeConfig
  -> mujoco_simulation_ros::SimulationRosBridge
```

迁移要点：

- 删除对 `mujoco_hardware/sensor_bridge.hpp` 的包含。
- 改为依赖 `mujoco_simulation_ros/simulation_ros_bridge.hpp`。
- ROS service callback 统一改成返回 `mujoco_simulation::Status`。
- ROS response 的错误消息只来自 `status.message()`。

### 2.2 `mujoco_hardware` 依赖面变化

`mujoco_hardware` 继续依赖：

- `hardware_interface`
- `rclcpp`
- `rclcpp_lifecycle`
- `mujoco_simulation`
- `mujoco_simulation_ros`

`sensor_msgs`、`rosgraph_msgs`、`std_srvs`、`mujoco_ros2_bridge_msgs` 的 publisher/service 实现职责已经迁出 `mujoco_hardware` 本体。

## 3. 公开命名迁移

- `JointInfo` -> `JointConfig`
- `ImuInfo` -> `ImuConfig`
- `LidarInfo` -> `LidarConfig`
- `CameraSpec` -> `CameraConfig`
- `MobileBaseInfo` -> `MobileBaseConfig`
- `ImuState` -> `ImuSample`
- `LidarState` -> `LidarSample`

## 4. 错误处理迁移

统一规则：

- 运行时与 bridge 内部逻辑优先返回 `Status`
- 组件读取型接口返回 `Result<T>`
- 仅在 ROS / `hardware_interface` 框架边界适配成 `bool` 或 `return_type`

推荐写法：

```cpp
const auto status = simulation.start();
if (!status.ok()) {
  RCLCPP_ERROR(logger, "%s", status.message().c_str());
  return hardware_interface::CallbackReturn::ERROR;
}
```

不再推荐：

- `bool + std::string*`
- `last_error()`
- 依赖字符串匹配做错误分支

## 5. 构建迁移

当前标准构建集为：

```bash
colcon build --packages-select mujoco_simulation mujoco_simulation_ros mujoco_hardware robot_mujoco
```

如果工作区依赖外层 underlay 提供 `mujoco_ros2_bridge_msgs`，先 source 外层安装空间，再 source 当前 `robot_mujoco/install`。

## 6. 测试迁移

- 旧 `test_sensor_bridge.cpp` 已迁入 `mujoco_simulation_ros/tests/test_simulation_ros_bridge.cpp`
- `mujoco_hardware` 测试现在只保留 ros2_control 适配、模式切换、读写和 bridge 接线验证
- 性能与 soak 入口统一改为：

```bash
python3 mujoco_simulation/test/performance/run_baseline.py \
  --workspace-root /path/to/robot_mujoco \
  --output build/performance_baseline.json
```
