# Simulation API Migration

`mujoco_simulation::Simulation` is now the primary top-level runtime entry point.

Workspace-level package migration is summarized in [`../../docs/migration_guide.md`](../../docs/migration_guide.md).

## What Changed

- New public header: `mujoco_simulation/simulation.hpp`
- New top-level type: `mujoco_simulation::Simulation`
- `mujoco_hardware` now depends on `Simulation` directly
- ROS publishers and control services now live in `mujoco_simulation_ros`
- Public runtime control now uses `Status` / `Result<T>`
- `SimulationConfig.components` is the only public component registration entry
- Public `register_*`, `read_*`, `with_locked_data()`, and `copy_data_to()` have been removed

## Recommended Migration

Replace:

```cpp
#include "mujoco_simulation/mujoco_simulation.hpp"

mujoco_simulation::MuJoCoSimulation simulation;
```

with:

```cpp
#include "mujoco_simulation/simulation.hpp"

mujoco_simulation::Simulation simulation;
```

`mujoco_simulation/mujoco_simulation.hpp` and the `MuJoCoSimulation` alias have now been removed from the codebase. Downstream code should migrate both the include path and the top-level type name.

## Runtime API Notes

- `initialize(...)` now returns `Status` directly instead of `bool + std::string&`
- Use `reconfigure_component(...)`, `set_joint_command(...)`, and `set_mobile_base_command(...)`
- Joint mode switch should be expressed by copying the current `JointConfig`, updating
  `command_mode`, and calling `reconfigure_component(ComponentConfig{updated_joint})`
- Use `joint_state(...)`, `imu_sample(...)`, `lidar_sample(...)`, `camera_sample(...)`, and `mobile_base_state(...)`
- `request_reset(...)` and `reset(...)` now take `ResetOptions`
  - `keyframe_name` / `keyframe_id` select the reset pose
  - `clear_commands` / `reset_components` / `clear_state_buffer` /
    `clear_camera_buffer` / `reset_statistics` control follow-up cleanup
  - `request_reset(...)` is the asynchronous submission path
  - `reset(...)` is the synchronous wait-for-completion wrapper
- Camera consumers that still need the old aggregate image/info view can convert via `camera_state_from_sample(...)`
- `ImuConfig`, `LidarConfig`, and `CameraConfig` now share `SensorCommonConfig common`
  - migrate flat `.name/.frame_id/.update_rate` initializers to `.common = {.name = ..., .frame_id = ..., .update_rate = ...}`
- `Simulation` no longer exposes `model()`, `config()`, `has_*()`, `*_id()`, `realtime_factor()`, or `is_initialized()` helper APIs

## Error Handling

- Downstream code should branch on both `status.ok()` and `status.code()`, not only on the message text
- Public runtime and component failures now use the unified `StatusCode` taxonomy:
  - `ModelLoadFailed` for MJCF/XML loading failures
  - `ModelValidationFailed` / `BindingFailed` for MuJoCo element mismatch and bind failures
  - `CommandRejected` for semantically invalid runtime commands
  - `RenderFailed` for GLFW/EGL/offscreen rendering failures
  - `ThreadFailed` / `Timeout` for asynchronous worker or viewer failures
- `mujoco_hardware` keeps framework-required `bool` / `return_type` signatures only at the ROS boundary; internal control flow is now `Status`-driven and error text should come from `status.message()`
