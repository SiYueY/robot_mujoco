# mujoco_simulation 模块重构优化开发文档

## 1. 文档信息

| 项目     | 内容                        |
| ------ | ------------------------- |
| 项目名称   | robot_mujoco              |
| 目标模块   | mujoco_simulation         |
| 文档类型   | 架构重构与开发实施文档               |
| 目标语言   | C++17                     |
| 运行平台   | Ubuntu 22.04              |
| ROS 版本 | ROS 2 Humble              |
| 仿真引擎   | MuJoCo                    |
| 参考项目   | mujoco_ros2_control、mjlab |
| 目标版本   | mujoco_simulation 2.0     |
| 文档状态   | In Progress（2026-06-17 已按当前代码与测试结果更新） |

---

## 当前重构进度（2026-06-17）

本节依据当前仓库代码静态核对更新，核对范围包含：

* `mujoco_simulation/include/`、`mujoco_simulation/src/`、`mujoco_simulation/test/`；
* `mujoco_simulation_ros/`；
* `mujoco_hardware/` 对 `mujoco_simulation::Simulation` 的接入方式；
* `docs/current_architecture.md`、`docs/migration_guide.md`、`docs/performance_baseline.md`；
* `colcon build --packages-select mujoco_simulation mujoco_simulation_ros mujoco_hardware robot_mujoco`；
* `colcon test --packages-select mujoco_simulation mujoco_simulation_ros mujoco_hardware`；
* `colcon test-result --all --verbose --test-result-base build` 当前结果。

当前结论：

* 新目录结构、`Simulation` / `ModelRuntime` / `SimulationScheduler` / `CommandBuffer` / `StateBuffer` / `CameraBuffer` / `CameraRenderer` 等骨架已经落地；
* 顶层 `Simulation` 公开运行时接口已经统一到 `Status` / `Result<T>` 风格，公开 `register_*` / `read_*`、`with_locked_data()`、`copy_data_to()`、`viewer_error()` 已移除；
* `SimulationConfig.components` 已成为唯一公开组件注册入口，公开类型命名已统一到 `*Config` / `*Sample` / `*State`；
* `mujoco_hardware` 已同步迁移到新 API，读写主链路为 `state_snapshot()`、`camera_sample()`、`set_joint_command()`、`set_mobile_base_command()`；
* `ComponentManager::build(...)` 已落地，`Simulation` 初始化阶段已不再自行逐类调用私有 `register_*_locked(...)` helper 装配组件；
* `ComponentRegistry` 已收口为 owning container + typed index，`ComponentManager` 不再维护独立的 joint / imu / lidar / camera / mobile base 注册表；
* joint mode switch 已不再通过顶层专用 helper 暴露；`mujoco_hardware` 现改为复制 `JointConfig`、更新 `command_mode`，再调用 `Simulation::reconfigure_component(ComponentConfig{updated_joint})`，而具体的 joint 重绑定流程已下沉到 `ComponentManager::reconfigure_joint(...)`；
* `Simulation` 的 typed 状态读取路径已继续下沉到 `StateBuffer` / `SimulationStateSnapshot`：`joint_state()`、`imu_sample()`、`lidar_sample()`、`mobile_base_state()` 不再直接操作快照内部容器，`test_state_buffer` 也已覆盖首帧前 `InvalidState` 和 typed lookup 行为；
* joint 配置在 `SimulationConfig.components` 中的回写已收口到通用 component 配置 helper；`simulation.cpp` 不再自行了解 variant 容器里 joint 条目的内部更新方式，`test_component_config` 已覆盖该行为；
* `Simulation` 现已不再公开 `set_joint_command_mode()`，也不再直接拼装 joint mode 重配置流程；顶层只保留通用的 `reconfigure_component(...)` 入口，`ComponentManager` 负责按组件类型分派到具体重配置实现；
* Camera 已并入统一 `ComponentManager` / `SensorComponent` / `SensorScheduler` 路径，`Simulation` 不再维护私有 `camera_scheduler_` / `camera_configs_`；
* `MobileBaseComponent` 已继承 `SimulationComponent`，并切换到 `Status` 风格的 `bind/reset/read/write` 接口，不再依赖 `MjContext` 或公开 `last_error()`；
* `MobileBaseConfig` 的公开配置入口已完全切到语义化轮名字段，`mujoco_hardware` 的 mobile base 参数解析也已不再保留 `traction_joint_names` 风格；
* `ModelRuntime::load()` 已支持初始化校验与 `initial_keyframe` 应用；`Simulation::initialize()` 现在已直接把完整 `ModelConfig` 前移到 `ModelRuntime::load(...)` 主路径，缺失 keyframe 会在初始化阶段直接以 `ModelValidationFailed` 失败并回滚到未加载状态；
* `test_model_runtime` 现已覆盖坏 XML、坏路径、非法 timestep、缺失 keyframe、运行时 reset 缺失 keyframe、重复 unload/reload 等阶段 1 验收路径；
* `SimulationScheduler` 的 write/physics/read/publish 回调顺序、reset-statistics、运行态线程失败语义已补充单测覆盖，统计输出已固定到可回归断言的行为；
* 本地 `scheduler_1khz` 基线命令 `build/mujoco_simulation/perf_runtime_scenarios --scenario scheduler_1khz --steps 2000` 当前产出 `throughput_hz=1000.97`、`realtime_factor=1.00047`、`jitter_ms_p95=1.07604`，已补齐阶段 2 的 1 kHz 实测证据；
* `test_mobile_base` 现已同时覆盖 differential / omnidirectional 的正逆运动学、wheel integration、ground-truth pose、reset 清理和 joint 重配置稳定性；
* `Viewer` 已切换到 `Status` 风格接口，公开头中已删除 `last_error()`；
* `Viewer` 的异步失败传播现已具备稳定注入测试：`test_viewer` 使用私有 render-thread 测试缝在无显示环境下复现“启动成功后后台失败，再经 `sync()` 返回 `ThreadFailed`”的路径，补齐了阶段 8 最后缺失的错误传播证据；
* `StatusCode` 已扩展并统一到完整 taxonomy：`InvalidArgument`、`InvalidState`、`FailedPrecondition`、`NotFound`、`AlreadyExists`、`ModelLoadFailed`、`ModelValidationFailed`、`BindingFailed`、`CommandRejected`、`RenderFailed`、`ThreadFailed`、`Timeout`、`Internal`；
* `SimulationScheduler`、`CameraRenderer`、`Viewer` 运行时实现中的 `last_error_` side-channel 已删除，跨线程失败统一通过 `Status` 传播，`Simulation` 仅保留 `runtime_error_` 作为 `Error` 状态原因快照；
* `SimulationScheduler` 的阶段化回调现已补齐为 `write_commands -> step_physics -> read_components -> publish_state_snapshot -> sync_viewer_if_due`；Camera 到期采样继续通过 `publish_state_snapshot` 阶段内的 `ComponentManager::sample_due_sensors(...)` 触发，但 Viewer 定频同步已不再由 `Simulation` 的独立线程承担，而是完全由 scheduler 回调驱动；
* `Simulation` 顶层公开接口已进一步收口：`model()`、`config()`、`has_*()` / `*_id()`、`set_joint_command_mode()`、`realtime_factor()`、`is_initialized()` 已从公开头删除；顶层现主要保留生命周期、运行控制、命令写入、typed 状态读取、`camera_sample()`、`state_snapshot()`、`status()`、`simulation_time()`、`step_count()`、`set_realtime_factor()` 和通用 `reconfigure_component(...)`；
* reset 主流程已切到公开 `ResetOptions`：支持 `keyframe_name` / `keyframe_id` 以及 `clear_commands` / `reset_components` / `clear_state_buffer` / `clear_camera_buffer` / `reset_statistics` 细粒度开关；`request_reset(...)` / `reset(...)` 的公开签名也已统一到该类型；
* reset 提交语义现已直接收口到两条公开 API：`Simulation::request_reset(...)` 负责异步提交，`Simulation::reset(...)` 负责等待完成；不再保留额外的 `RequestResetOptions` / `wait_for_completion` 中间层；
* `mujoco_hardware` 内部控制辅助链路已切到 `Status` 驱动，ROS / `hardware_interface` 边界层只保留必要的 `bool` / `return_type` 适配；
* `mujoco_simulation` 已拆分出内部 `components/runtime/camera/viewer_support` target，但对下游导出的唯一公开目标仍是 `mujoco_simulation`；
* 已新增 `mujoco_simulation_ros` 包，并将原 `mujoco_hardware::SensorBridge` 的 ROS publisher/service 责任迁移到 `SimulationRosBridge`；
* 已新增 `docs/current_architecture.md`、`docs/migration_guide.md`、`docs/performance_baseline.md` 作为阶段 0 基线输入，并补充了 repo 内固定样例 `docs/performance_baseline.sample.json`；
* 已新增 `mujoco_simulation/test/performance` 与 `mujoco_hardware/tests/performance` 下的 benchmark/采集脚本，以及 `scripts/ci/*.sh`、`.github/workflows/*.yml` 作为阶段 11 的标准入口；
* `colcon test --packages-select mujoco_simulation mujoco_simulation_ros mujoco_hardware` 已实际执行测试；结合 `colcon test-result --all --verbose --test-result-base build` 的当前汇总结果为 `110 tests / 0 errors / 0 failures`；
* `./scripts/ci/build_and_test.sh` 与 `./scripts/ci/run_clang_tidy.sh` 已在当前嵌套工作区本地重新验证通过；`scripts/ci/common.sh` 现在会按已安装 overlay 包自动补充 `--allow-overriding`，避免 `colcon-override-check` 警告在未来 colcon 版本中升级成硬错误；
* `python3 mujoco_simulation/test/performance/run_baseline.py --workspace-root "$(pwd)" --output build/performance_baseline.sample.json --repeat 1 --soak-seconds 5` 已可在本地工作区直接产出完整 baseline JSON，覆盖 `scheduler_1khz`、`headless_camera`、`hardware_read_write_loop` 和 soak RSS 摘要；
* `SOAK_SECONDS=2 OUTPUT_PATH=build/performance_baseline.quick.json BASELINE_REFERENCE=docs/performance_baseline.sample.json COMPARISON_OUTPUT=build/performance_comparison.quick.json COMPARISON_MARKDOWN_OUTPUT=build/performance_comparison.quick.md ./scripts/ci/run_soak_and_perf.sh` 当前可直接生成 baseline、comparison JSON 和 Markdown summary 三份产物，且 `compare_performance_baseline.py` 返回 `comparison_status=pass`；soak RSS 现已切到单进程采样，并在 comparison 中输出剥离启动瞬态后的 `rss_kb_growth_ratio` / `rss_kb_growth_kb` 趋势项；
* `scripts/ci/common.sh`、`run_sanitizers.sh`、`run_tsan_subset.sh` 的 shell/setup 与 `colcon` 调用问题已修正；当前 `BUILD_BASE=build_asan_ubsan_core INSTALL_BASE=install_asan_ubsan_core LOG_BASE=log_asan_ubsan_core ./scripts/ci/run_sanitizers.sh` 已可在干净目录下稳定跑通 `mujoco_simulation` runtime-core 的 ASan/UBSan 套件，结果为 `83 tests / 0 errors / 0 failures`；`run_tsan_subset.sh` 现会先执行最小 TSan preflight，并统一落盘 `log_tsan/tsan_preflight.json`、`log_tsan/tsan_summary.json` 和 `log_tsan/tsan_summary.md`，避免 nightly 永远停在无诊断的硬崩；
* 尚未完成的主要事项集中在 nightly 真实长稳数据以及性能回归阈值的持续跟踪；`mujoco_simulation_ros` 首测崩溃、`mujoco_hardware` 中 ROS 2 `rcutils/rclcpp` 路径的 `new-delete-type-mismatch`，以及 `simulate`/Mesa viewer 路径的 UBSan/LSan 噪声已被确认属于当前上游依赖边界，因此 ASan/UBSan gate 现聚焦于 headless/runtime core，而 TSan nightly 已先通过 preflight + summary artifact 区分“代码 race”与“宿主机 runtime 不支持”两类失败，ROS/Viewer 路径继续由常规 CI 与 nightly soak 覆盖。
* 阶段 11 的 perf/soak/TSan 现在已补到“生成结果 + 比较报告/摘要 + workflow artifact”链路：`scripts/ci/run_soak_and_perf.sh` 支持基线比较并生成 Markdown 摘要，`scripts/ci/compare_performance_baseline.py` 已对 scheduler realtime/throughput 与 headless camera sample frequency 执行 gating，同时把 latency、hardware loop 与 soak 指标整理为 trend checks；其中 soak RSS 已明确收口到单进程采样 + 启动瞬态剥离后的稳态头尾窗口比较，不再依赖重复拉起短进程的弱证据。`scripts/ci/run_tsan_subset.sh` 则会为 preflight 或实际测试结果统一生成 `tsan_summary.{json,md}`；nightly workflow 还会进一步汇总生成 `nightly_summary.{json,md}`、`nightly_metrics.json`、`nightly_trend.{json,md}` 和 `nightly_history.{json,md}`，把 perf 与 TSan 结果整合成单次总览、相邻两次对比以及最近多次成功 nightly 的滚动历史摘要；`scripts/ci/tests/test_nightly_ci_scripts.py` 也已接入 `ci.yml` / `nightly.yml`，为这组脚本提供基础回归保护。

### 分阶段进度判断

| 阶段 | 当前判断 | 说明 |
| --- | --- | --- |
| 阶段 0：建立重构基线 | 完成 | 已补充 `docs/current_architecture.md`、`docs/migration_guide.md`、`docs/performance_baseline.md`、性能采集脚本和测试目录，并新增 repo 内固定样例 `docs/performance_baseline.sample.json`；本地已验证 `run_baseline.py --repeat 1 --soak-seconds 5` 可直接生成覆盖 scheduler/camera/hardware/soak 的标准 JSON 产物。 |
| 阶段 1：基础类型与 ModelRuntime | 完成 | `status.hpp`、`result.hpp`、`runtime/mujoco_raii.hpp`、`runtime/model_runtime.*` 已稳定落地；`test_model_runtime` 现在覆盖坏 XML、坏路径、非法 timestep、load-time/runtime keyframe 失败、forward、copy、重复 unload/reload 等验收路径，初始化失败也会回滚到未加载状态。 |
| 阶段 2：SimulationScheduler | 完成 | `runtime/simulation_scheduler.*`、状态机、工作线程、manual step 限制、reset request、deadline 累积逻辑和统计接口都已实现；`test_simulation_scheduler` 已覆盖顺序、pause/resume、reset、lag recovery、运行态线程失败，且本地 `scheduler_1khz` 基线已给出 `throughput_hz=1000.97` / `realtime_factor=1.00047` 的实测证据。当前实现已经固定为 `write_commands -> step_physics -> read_components -> publish_state_snapshot -> sync_viewer_if_due` 的阶段化回调顺序。 |
| 阶段 3：Component 基础和 JointComponent | 完成 | `SimulationComponent`、`SensorComponent`、`ComponentManager`、`ComponentRegistry`、`JointComponent` 已全部落地；`ComponentRegistry` 现在同时承担 owning container + typed index，`ComponentManager::build(...)` 与 `test_joint_component`、`test_component_registry` 已覆盖该阶段要求的组装、校验与 reset 行为。 |
| 阶段 4：MobileBaseComponent | 完成 | 已切到 `SimulationComponent` 生命周期，`bind/reset/read/write` 已统一返回 `Status`，`MjContext` 和公开 `last_error()` 已移除；公开配置与 `mujoco_hardware` 解析均已收口到语义化轮名字段，`test_mobile_base` 覆盖 differential / omnidirectional、wheel integration、ground-truth pose、reset 和重配置稳定性，且 `build_asan_ubsan` 下定向 `test_mobile_base` 已通过。 |
| 阶段 5：CommandBuffer 与 StateBuffer | 完成 | `CommandBuffer`、`StateBuffer`、`SimulationStateSnapshot` 已落地，`mujoco_hardware` 通过 `state_snapshot()` 与命令接口交互，`Simulation` 公开 API 已不再暴露主 `mjData` 旁路。 |
| 阶段 6：SensorComponent 和 SensorScheduler | 完成 | IMU/Lidar/Camera 已共用一套 `SensorScheduler` 和统一 `sample(context)` 路径，公开配置也已收口到共享 `SensorCommonConfig`；`test_sensor_scheduler` 现覆盖 `update_rate <= physics_rate`、稳定 cadence、reset 和 missed update 统计，`test_sensor_sampling` 覆盖统一时间戳、非法配置失败和 reset 后无负 `scan_time`。 |
| 阶段 7：CameraRenderer 和 CameraComponent | 完成 | 独立 GL context、headless、RGB/Depth 渲染、多相机共享 Renderer 与 CameraBuffer 路径均已落地，且 Camera 注册事实源已迁入 `ComponentManager`；`test_camera_rendering` 已覆盖 RGB 方向、Depth 米制、CameraInfo 投影、多相机独立频率等核心路径，`test_camera_runtime` 中 viewer 相关 parity/可用性用例则需显式设置 `MUJOCO_ENABLE_VIEWER_TESTS=1` 才会执行。 |
| 阶段 8：Viewer 解耦 | 完成 | Viewer 已独立成 `viewer/` 子模块，公开接口已切到 `Status`，不再暴露 `last_error()`；Viewer 定频同步也已不再依赖 `Simulation` 私有线程，而是通过 scheduler 的 `sync_viewer_if_due()` 回调按 `viewer_update_rate` 统一驱动。默认无 GUI 环境下仍以 headless 路径与非 viewer 用例为主，`test_camera_runtime` / `test_viewer` 中关于 Viewer 关闭后 camera 持续可用、低 viewer 频率不阻塞物理线程、restart/stop/shutdown 安全性、同一 `Simulation` 实例 stop/start 后自动重建 viewer，以及异步失败注入后 `sync()` 返回 `ThreadFailed` 的覆盖，需要显式设置 `MUJOCO_ENABLE_VIEWER_TESTS=1`。 |
| 阶段 9：Simulation 顶层接口 | 完成 | 旧 `MuJoCoSimulation` 文件已删除，顶层入口已切为 `Simulation`；公开 API 已统一到 `Status` / `Result<T>`，初始化路径已固定为 `initialize() -> ModelRuntime::load(ModelConfig) -> ComponentManager::build(...)`；`initial_keyframe` 已前移到主 load 路径，公开 reset 已切换到 `ResetOptions`，同步/异步 reset 直接分别由 `reset(...)` / `request_reset(...)` 表达，`model()` / `config()` / `has_*()` / `*_id()` / `set_joint_command_mode()` / `realtime_factor()` / `is_initialized()` 已删除，joint mode switch 已统一改走 `reconfigure_component(...)`。 |
| 阶段 10：ROS 2 集成迁移 | 完成 | 已拆出 `mujoco_simulation_ros`，`SensorBridge` 相关测试和实现已迁移，`mujoco_hardware` 仅保留 ros2_control 适配与 bridge 接线。 |
| 阶段 11：清理和稳定化 | 基本完成 | 已补充 GitHub Actions、sanitizer/clang-tidy/TSan/soak/perf 脚本入口，并修正了 CI shell/setup、`colcon test-result` 汇总口径、嵌套工作区 `colcon-override-check` 噪声以及 perf/soak/TSan 比较链路；`build_and_test.sh`、`run_clang_tidy.sh`、`run_soak_and_perf.sh`、`run_tsan_subset.sh` 均已在当前工作区重新验证通过。soak RSS 现已切到单进程采样并输出剥离启动瞬态后的稳态增长趋势，nightly 现可生成 `performance_baseline.json`、`performance_comparison.json`、`performance_comparison.md`、`tsan_summary.json`、`tsan_summary.md`、`nightly_summary.json`、`nightly_summary.md`、`nightly_metrics.json`、`nightly_trend.json`、`nightly_trend.md`、`nightly_history.json`、`nightly_history.md` 和 TSan/soak artifact，`run_sanitizers.sh` 已稳定收口到 `mujoco_simulation` runtime-core 的 ASan/UBSan gate，`run_tsan_subset.sh` 也会在宿主机不支持 TSan runtime 时产出 preflight + summary 报告。仍需在真实 CI/nightly 中积累长稳结果与趋势数据。 |

### 当前已完成的关键事项

* 已引入 `Status` / `Result<T>` 基础类型。
* 已引入 MuJoCo RAII 封装和 `ModelRuntime`。
* 已引入 `SimulationScheduler`，并具备 start/stop/pause/resume/manual step/reset request。
* 已引入 `SimulationComponent` / `SensorComponent` / `ComponentManager` / `ComponentRegistry`。
* 已将 typed index 从 `ComponentManager` 收口到 `ComponentRegistry`，manager 只保留装配、reset、读写和采样调度职责。
* 已实现 `JointComponent` 的 actuator 类型检查、控制写入与 reset 清理。
* 已实现 `CommandBuffer`、`StateBuffer`、`CameraBuffer`。
* 已将 `Simulation` 的 typed 状态读取继续下沉到 `StateBuffer` / `SimulationStateSnapshot`，并补充 `test_state_buffer` 回归覆盖。
* 已将 joint config variant 回写从 `simulation.cpp` 下沉到 component 配置层 helper，并补充 `test_component_config` 回归覆盖。
* 已将 joint mode 重配置与 command mode 解析从 `simulation.cpp` 下沉到 `ComponentManager` 高层接口，并用 `test_buffered_simulation` 持续覆盖顶层 API 行为。
* 已实现 `SensorScheduler`，并对 IMU/Lidar/Camera 进行按频率采样。
* 已将 `ImuConfig`、`LidarConfig`、`CameraConfig` 的公共字段收口到 `SensorCommonConfig`。
* 已实现独立 `CameraRenderer`，支持 Viewer 关闭时的 headless 相机渲染。
* 已将 Viewer 抽出到独立目录，不再直接复用 Camera 的渲染输出。
* 已补齐 viewer/headless camera parity、Viewer restart/stop smoke、同一 `Simulation` 实例 stop/start 后 viewer 自动恢复、Viewer 异步失败传播和低 viewer 频率不阻塞物理线程的回归测试；其中 Viewer 依赖用例默认通过 `MUJOCO_ENABLE_VIEWER_TESTS=1` 显式启用。
* 已将 `Simulation` 顶层公开接口统一到 `Status` / `Result<T>` 风格。
* 已删除 `Simulation` 公开 `register_*` / `read_*` / `with_locked_data()` / `copy_data_to()` / `viewer_error()`。
* 已将 `SimulationConfig.components` 收口为唯一公开组件注册入口。
* 已同步迁移 `mujoco_hardware` 到新 API 和新命名。
* 已完成 `Camera` 归一、`MobileBaseComponent` 收口和 `mujoco_simulation` 内部 target 拆分。
* 已补充并跑通 `mujoco_simulation` 与 `mujoco_hardware` 测试闭环。

### 当前未完成或只完成一半的关键事项

* 仍未完成真实 nightly soak 数据沉淀和性能回归阈值持续跟踪；当前本地 Ubuntu 22.04 / GCC TSan runtime 的最小 probe 仍会直接报 `unexpected memory mapping`，所以是否能在 CI 主机上真正跑通 TSan 子集还需要 `nightly_summary.*`、`nightly_metrics.json`、`nightly_trend.*`、`nightly_history.*` 和底层 artifact 持续给证据。
* ROS 2 bridge / Viewer 路径在 ASan/UBSan 下仍存在上游 `rclcpp`/`rmw` 与 MuJoCo `simulate`/Mesa 噪声，因此当前 sanitizer gate 已明确收口到 runtime core；若后续依赖版本升级消除这些问题，再考虑把这部分重新并回强制 gate。
# 2. 背景

`mujoco_simulation` 当前承担以下职责：

* 加载和销毁 MuJoCo MJCF 模型；
* 管理 `mjModel` 与 `mjData`；
* 运行物理仿真循环；
* 管理仿真开始、停止、暂停、恢复和重置；
* 管理 Joint、IMU、Camera、Lidar、MobileBase 等设备；
* 写入关节和底盘控制命令；
* 读取关节状态和传感器数据；
* 启动和同步 MuJoCo Viewer；
* 使用 Viewer 渲染资源获取 Camera 图像；
* 为上层 ROS 2 和 `ros2_control` 提供访问接口。

当前实现已经具备 MuJoCo 机器人仿真运行时的基本能力，但随着机器人模型、控制接口和传感器类型增加，现有架构逐渐暴露出以下问题：

1. 顶层仿真类承担过多职责；
2. 模型运行、线程调度、设备管理和渲染逻辑耦合；
3. `HardwareManager` 同时管理关节、传感器和组合设备，语义不准确；
4. Joint 同时包含关节状态和执行器命令，但抽象边界不明确；
5. MobileBase 保存裸 `Joint*`，存在生命周期和悬空指针风险；
6. Camera 与 Viewer 共享 OpenGL 和 MuJoCo 渲染资源；
7. Viewer 更新频率与物理仿真频率绑定；
8. ROS 线程可能直接访问或修改主 `mjData`；
9. 不同传感器缺少统一更新频率调度；
10. reset 过程未覆盖全部组件状态和命令缓存；
11. CameraInfo、图像方向和深度单位不完整；
12. 核心运行时与 ROS 2 依赖边界不清晰；
13. 配置对象、模型绑定信息和运行状态混合；
14. 错误处理主要依赖 `bool + last_error`；
15. 现有目录中的 `hardware`、`core`、`entity` 等名称无法准确表达职责。

本次重构以建立一个清晰、可测试、可扩展的 C++ MuJoCo 运行时为目标。

---

# 3. 重构定位

重构后的 `mujoco_simulation` 定位为：

> 面向单机器人或少量机器人实时仿真的通用 C++ MuJoCo Runtime，支持控制组件、传感器组件、交互式 Viewer、无窗口 RGB-D 渲染，以及 ROS 2 和 ros2_control 适配。

目标架构吸收：

* `mujoco_ros2_control` 的 ROS 2 生命周期、控制接口和仿真控制经验；
* `mjlab` 的配置与运行对象分离、组件化组织和管理器设计思想。

不直接照搬：

* `mjlab` 的 GPU 批量环境；
* MuJoCo-Warp；
* Reward、Termination、Curriculum 等强化学习层；
* Python 动态配置体系；
* Isaac Lab API；
* 通用 Entity-Component-System；
* 为抽象而抽象的复杂 Manager 层。

---

# 4. 重构目标

## 4.1 核心目标

重构后应满足：

1. `mjModel` 和主 `mjData` 所有权唯一；
2. 主 `mjData` 只能由仿真线程修改；
3. 外部线程通过命令缓冲提交控制命令；
4. 外部线程通过状态缓存读取仿真状态；
5. Joint、IMU、Lidar、Camera、MobileBase 使用统一组件模型；
6. 不引入含义模糊的 Entity 抽象；
7. 不将 Actuator 设计为独立顶层模块；
8. Joint 内部明确区分 joint binding 与 actuator binding；
9. Camera 渲染不依赖 Viewer；
10. Viewer 和 CameraRenderer 使用独立渲染资源；
11. 支持无 Viewer 的 headless RGB-D 相机；
12. 支持不同传感器独立采样频率；
13. reset 覆盖模型、命令、组件、传感器和缓存；
14. 核心运行时不依赖 `rclcpp` 和 ROS 消息；
15. ROS 2 和 `ros2_control` 作为外部适配层接入；
16. 所有配置在运行前完成校验和模型绑定；
17. 错误处理使用统一 `Status` 和 `Result`；
18. 保留一个轻量顶层 `Simulation` 对象作为系统入口；
19. 删除原有承担过多职责的 `MuJoCoSimulation` 实现；
20. 支持单元测试、并发测试和性能测试。

## 4.2 非目标

本阶段不实现：

* GPU 批量并行仿真；
* MuJoCo-Warp；
* 多进程仿真；
* 分布式仿真；
* 强化学习任务管理器；
* 通用 ECS；
* 运行时编辑 MJCF；
* 多物理引擎抽象；
* 完整仿真 GUI 编辑器；
* Python API；
* 动态热加载模型。

---

# 5. 核心设计原则

## 5.1 单一职责

各模块职责如下：

| 模块                    | 职责                                |
| --------------------- | --------------------------------- |
| `Simulation`          | 顶层入口和子系统组合                        |
| `ModelRuntime`        | 管理 `mjModel`、`mjData` 和 MuJoCo 操作 |
| `SimulationScheduler` | 管理仿真线程、时间和执行顺序                    |
| `ComponentManager`    | 管理所有仿真组件                          |
| `JointComponent`      | 关节状态读取和控制命令写入                     |
| `SensorComponent`     | 传感器采样公共接口                         |
| `MobileBaseComponent` | 底盘运动学、命令分解和里程计                    |
| `CommandBuffer`       | 跨线程提交控制命令                         |
| `StateBuffer`         | 跨线程发布状态快照                         |
| `CameraBuffer`        | 缓存 Camera 采样结果                    |
| `CameraRenderer`      | 执行无窗口 RGB-D 渲染                    |
| `Viewer`              | 交互式场景显示                           |
| ROS Adapter           | ROS 消息、服务、TF 和 ros2_control       |

## 5.2 唯一数据所有权

```text
Simulation
└── ModelRuntime
    ├── 独占 mjModel
    └── 独占主 mjData
```

其他对象只能借用：

```text
const mjModel&
const mjData&
mjData&（仅仿真线程调用路径）
```

禁止组件长期保存无约束的可写 `mjData*`。

## 5.3 仿真线程唯一写入

以下操作只能由仿真线程执行：

* 修改 `data->ctrl`；
* 修改 `data->qfrc_applied`；
* 修改 `qpos`、`qvel`；
* 调用 `mj_step`；
* 调用 `mj_forward`；
* 调用 `mj_resetData`；
* 应用 keyframe；
* 写入主 `mjData`；
* 写入控制命令；
* 执行组件 reset。

ROS、GUI 和用户线程只允许：

* 向 `CommandBuffer` 写命令；
* 向 Scheduler 提交状态请求；
* 从 `StateBuffer` 读状态；
* 从 `CameraBuffer` 读图像；
* 请求 start、stop、pause、resume、reset。

## 5.4 配置、绑定和运行状态分离

每类组件使用三类对象：

```text
Config
  ↓ 根据 mjModel 解析
Binding
  ↓ 运行时访问
State / Command / Sample
```

例如 Joint：

```text
JointConfig
JointBinding
JointCommand
JointState
```

Camera：

```text
CameraConfig
CameraBinding
CameraSample
```

配置对象不保存 MuJoCo 地址；绑定对象不保存动态状态。

## 5.5 组合优于继承

只建立必要的公共接口：

```text
SimulationComponent
└── SensorComponent
```

不建立无明确行为的：

```text
Entity
Actuator
SceneObject
HardwareDevice
```

Joint 和 MobileBase 直接继承 `SimulationComponent`。

IMU、Lidar、Camera 继承 `SensorComponent`。

## 5.6 不为未来过早抽象

当前只有 Camera 使用离屏渲染，因此：

* 不建立顶层 `render/` 模块；
* 使用 `component/camera/CameraRenderer`；
* 不建立通用 RenderGraph；
* 不建立多后端插件系统，先提供可扩展接口。

当未来出现 segmentation、object ID、多个渲染后端时，再提取独立 `rendering/` 模块。

---

# 6. 总体架构

```text
┌───────────────────────────────────────────────────────┐
│                  ROS 2 Integration                     │
│                                                       │
│ ros2_control  Sensor Publisher  TF  Clock  Services   │
└─────────────────────────┬─────────────────────────────┘
                          │
               command / state / samples
                          │
┌─────────────────────────▼─────────────────────────────┐
│                       Simulation                       │
│              顶层入口 / Composition Root              │
└───────┬───────────┬────────────┬────────────┬─────────┘
        │           │            │            │
┌───────▼──────┐ ┌──▼─────────┐ ┌▼─────────┐ ┌▼────────┐
│ ModelRuntime │ │ Components │ │ Buffers  │ │ Viewer  │
│ model/data   │ │ Manager    │ │ command  │ │ display │
│ step/reset   │ │ components │ │ state    │ └─────────┘
└───────┬──────┘ └────┬───────┘ │ camera   │
        │             │         └──────────┘
        │             │
┌───────▼─────────────▼─────────────────────────────────┐
│                SimulationScheduler                     │
│ thread / time / requests / simulation execution flow  │
└─────────────────────────┬─────────────────────────────┘
                          │ snapshots
                 ┌────────▼────────┐
                 │ CameraRenderer  │
                 │ offscreen RGB-D │
                 └─────────────────┘
```

---

# 7. 顶层对象 Simulation

## 7.1 定位

`Simulation` 是整个模块唯一推荐给上层使用的入口对象。

它的职责是：

* 创建和连接子系统；
* 提供稳定的公共 API；
* 控制整体初始化和销毁；
* 将命令和状态操作转发给对应子系统；
* 隐藏内部线程和组件管理细节。

它不负责：

* 直接调用 `mj_step`；
* 直接执行组件控制；
* 直接渲染 Camera；
* 承担所有业务逻辑；
* 直接管理 ROS。

## 7.2 为什么不保留 MuJoCoSimulation

原 `MuJoCoSimulation` 名称和实现不再保留，原因：

1. 类名与模块名重复；
2. 原类承担模型、线程、设备、Viewer、Camera 等过多职责；
3. 容易继续演变为 God Object；
4. 整体重构允许同步修改上层调用；
5. 新架构需要明确新的接口契约。

新的顶层类型命名为：

```cpp
mujoco_simulation::Simulation
```

## 7.3 建议接口

```cpp
class Simulation {
public:
    Simulation();
    ~Simulation();

    Simulation(const Simulation&) = delete;
    Simulation& operator=(const Simulation&) = delete;

    Status initialize(const SimulationConfig& config);
    Status shutdown();

    Status start();
    Status stop();
    Status pause();
    Status resume();

    Status step(std::size_t count = 1);
    Status reset(const ResetOptions& options = {});

    Status set_joint_command(
        std::string_view component_name,
        const JointCommand& command);

    Status set_mobile_base_command(
        std::string_view component_name,
        const MobileBaseCommand& command);

    Result<JointState> joint_state(
        std::string_view component_name) const;

    Result<MobileBaseState> mobile_base_state(
        std::string_view component_name) const;

    Result<ImuSample> imu_sample(
        std::string_view component_name) const;

    Result<LidarSample> lidar_sample(
        std::string_view component_name) const;

    Result<CameraSample> camera_sample(
        std::string_view component_name) const;

    std::shared_ptr<const SimulationStateSnapshot>
    state_snapshot() const;

    SimulationStatus status() const;
    double simulation_time() const;
    std::uint64_t step_count() const;

private:
    std::unique_ptr<ModelRuntime> model_runtime_;
    std::unique_ptr<ComponentManager> component_manager_;

    std::unique_ptr<CommandBuffer> command_buffer_;
    std::unique_ptr<StateBuffer> state_buffer_;
    std::unique_ptr<CameraBuffer> camera_buffer_;

    std::unique_ptr<CameraRenderer> camera_renderer_;
    std::unique_ptr<Viewer> viewer_;

    std::unique_ptr<SimulationScheduler> scheduler_;
};
```

---

# 8. ModelRuntime

## 8.1 职责

`ModelRuntime` 是 MuJoCo 模型与数据的唯一所有者。

负责：

* 加载 XML/MJB；
* 创建 `mjData`；
* 销毁模型和数据；
* 执行 `mj_step`；
* 执行 `mj_forward`；
* 执行 reset；
* 应用 keyframe；
* 返回仿真时间和 timestep；
* 复制 `mjData` 快照；
* 提供受控模型访问。

不负责：

* 启动线程；
* 控制命令缓存；
* 组件调度；
* Viewer；
* Camera；
* ROS。

## 8.2 RAII

```cpp
struct MjModelDeleter {
    void operator()(mjModel* model) const noexcept {
        if (model != nullptr) {
            mj_deleteModel(model);
        }
    }
};

struct MjDataDeleter {
    void operator()(mjData* data) const noexcept {
        if (data != nullptr) {
            mj_deleteData(data);
        }
    }
};

using MjModelPtr =
    std::unique_ptr<mjModel, MjModelDeleter>;

using MjDataPtr =
    std::unique_ptr<mjData, MjDataDeleter>;
```

## 8.3 接口

```cpp
class ModelRuntime {
public:
    Status load(const ModelConfig& config);
    void unload();

    bool is_loaded() const noexcept;

    Status step();
    Status step(std::size_t count);
    Status forward();

    Status reset(const ResetOptions& options = {});

    const mjModel& model() const;
    const mjData& data() const;
    mjData& mutable_data();

    double simulation_time() const noexcept;
    double timestep() const noexcept;

    Status copy_data_to(mjData& destination) const;

private:
    MjModelPtr model_;
    MjDataPtr data_;
};
```

`mutable_data()` 只能在 Scheduler 控制的仿真线程调用路径中使用。

不向普通上层 API 暴露。

---

# 9. SimulationScheduler

## 9.1 职责

`SimulationScheduler` 负责：

* 管理仿真线程；
* 管理运行状态；
* 管理物理频率和实时因子；
* 接收 start、stop、pause、resume、reset 请求；
* 执行每个仿真周期；
* 写入命令；
* 调用 `mj_step`；
* 读取组件；
* 发布状态快照；
* 调度 Camera 和 Viewer；
* 统计实时因子和循环耗时。

## 9.2 状态机

```cpp
enum class SimulationStatus {
    Uninitialized,
    Stopped,
    Running,
    Paused,
    Stopping,
    Error
};
```

状态转换：

```text
Uninitialized
    ↓ initialize
Stopped
    ↓ start
Running
    ↔ pause/resume
Paused
    ↓ stop
Stopped
    ↓ shutdown
Uninitialized
```

## 9.3 状态约束

* `start()` 只允许在 `Stopped`；
* `pause()` 只允许在 `Running`；
* `resume()` 只允许在 `Paused`；
* `step()` 只允许在 `Stopped` 或 `Paused`；
* `reset()` 通过请求队列提交；
* `shutdown()` 必须等待仿真线程退出；
* `Error` 状态只允许 stop、shutdown 或明确 recover。

## 9.4 SchedulerConfig

```cpp
struct SchedulerConfig {
    bool realtime_sync{true};
    double realtime_factor{1.0};

    double state_update_rate{1000.0};
    double viewer_update_rate{60.0};

    std::chrono::milliseconds max_schedule_lag{100};
};
```

## 9.5 实时同步

禁止每轮重新以当前时间为基准 sleep。

应使用累积 deadline：

```cpp
auto next_tick = clock::now();

while (running) {
    step_once();

    next_tick += period;

    if (realtime_sync) {
        std::this_thread::sleep_until(next_tick);
    }

    const auto now = clock::now();

    if (now > next_tick + max_lag) {
        next_tick = now;
    }
}
```

仿真周期：

```text
wall period = model timestep / realtime factor
```

## 9.6 每步执行流程

不强制建立公开 `SimulationPipeline` 类型。

执行流程作为 Scheduler 内部方法：

```cpp
Status SimulationScheduler::step_once()
{
    RETURN_IF_ERROR(process_requests());
    RETURN_IF_ERROR(write_commands());
    RETURN_IF_ERROR(step_physics());
    RETURN_IF_ERROR(read_components());
    RETURN_IF_ERROR(publish_state_snapshot());
    RETURN_IF_ERROR(schedule_camera_rendering());
    RETURN_IF_ERROR(sync_viewer_if_due());

    return Status::Ok();
}
```

执行顺序：

```text
1. 处理停止、暂停和 reset 请求
2. 获取 CommandBuffer 快照
3. 写入 Joint/MobileBase 命令
4. mj_step
5. 读取 Joint 状态
6. 读取 MobileBase 状态
7. 采样到期 IMU/Lidar
8. 发布 StateSnapshot
9. 提交到期 Camera 渲染任务
10. 按 Viewer 频率同步显示
```

---

# 10. Component 组件模型

## 10.1 为什么使用 Component

`Entity` 在当前项目中含义不明确：

* 可能表示机器人；
* 可能表示 MuJoCo body；
* 可能表示场景对象；
* 可能表示所有可注册设备；
* 难以判断是否包含 Sensor。

因此不使用 `Entity`。

`SimulationComponent` 定义为：

> 一个绑定到 MuJoCo 模型并参与初始化、重置、命令写入、状态读取或传感器采样的运行时模块。

## 10.2 基础接口

```cpp
class SimulationComponent {
public:
    virtual ~SimulationComponent() = default;

    virtual std::string_view name() const noexcept = 0;

    virtual Status bind(const mjModel& model) = 0;

    virtual Status reset(
        const mjModel& model,
        mjData& data) = 0;
};
```

不要求所有组件实现相同的 read/write 接口。

具体行为由明确的类型接口提供。

## 10.3 SensorComponent

```cpp
class SensorComponent : public SimulationComponent {
public:
    virtual double update_rate() const noexcept = 0;

    virtual Status sample(
        const mjModel& model,
        const mjData& data,
        double simulation_time,
        std::uint64_t step_count) = 0;
};
```

派生类型：

```text
SensorComponent
├── ImuComponent
├── LidarComponent
└── CameraComponent
```

Camera 的实际像素渲染由 `CameraRenderer` 完成，但 CameraComponent 负责：

* 传感器配置；
* MuJoCo camera binding；
* 采样频率；
* frame_id；
* CameraInfo；
* CameraSample 组装；
* 与 CameraBuffer 交互。

## 10.4 非传感器组件

```text
SimulationComponent
├── JointComponent
├── MobileBaseComponent
└── SensorComponent
```

Joint 和 MobileBase 不作为 Sensor。

---

# 11. ComponentManager

## 11.1 职责

`ComponentManager` 负责：

* 根据配置创建组件；
* 组件名称唯一性检查；
* 调用组件 bind；
* 管理组件生命周期；
* 按类型查找组件；
* 执行统一 reset；
* 写入命令型组件；
* 读取状态型组件；
* 调用 Sensor 采样；
* 管理组件之间的依赖。

## 11.2 接口

```cpp
class ComponentManager {
public:
    Status build(
        const ComponentConfigList& configs,
        const mjModel& model);

    Status reset_all(
        const mjModel& model,
        mjData& data);

    Status write_commands(
        const mjModel& model,
        mjData& data,
        const CommandSnapshot& commands);

    Status read_states(
        const mjModel& model,
        const mjData& data,
        double simulation_time,
        std::uint64_t step_count,
        SimulationStateSnapshot& snapshot);

    Status sample_due_sensors(
        const mjModel& model,
        const mjData& data,
        double simulation_time,
        std::uint64_t step_count);

    JointComponent* joint(std::string_view name);
    MobileBaseComponent* mobile_base(std::string_view name);
    ImuComponent* imu(std::string_view name);
    LidarComponent* lidar(std::string_view name);
    CameraComponent* camera(std::string_view name);

private:
    ComponentRegistry registry_;
    SensorScheduler sensor_scheduler_;
};
```

## 11.3 ComponentRegistry

内部可使用：

```cpp
std::unordered_map<
    std::string,
    std::unique_ptr<SimulationComponent>>
    components_;
```

同时维护类型索引：

```cpp
std::unordered_map<std::string, JointComponent*> joints_;
std::unordered_map<std::string, SensorComponent*> sensors_;
```

组件的 owning pointer 只保存在统一容器中。

类型索引只在 `ComponentManager` 生命周期内有效。

禁止组件互相长期保存其他组件的裸指针。

组件依赖使用稳定 Binding 或名称解析结果。

---

# 12. JointComponent

## 12.1 定位

JointComponent 同时提供：

* 关节状态读取；
* 关节控制命令写入。

不再将 Joint 拆分为：

```text
JointActuator
JointStateSensor
```

原因是 ROS 2 control 的 joint 通常同时具有 state interface 和 command interface。

## 12.2 JointConfig

```cpp
enum class JointCommandMode {
    None,
    Position,
    Velocity,
    Effort
};

enum class JointControllerType {
    MuJoCoActuator,
    SoftwarePd
};

struct JointConfig {
    std::string name;
    std::string joint_name;

    std::optional<std::string> actuator_name;

    JointCommandMode command_mode{
        JointCommandMode::None};

    JointControllerType controller_type{
        JointControllerType::MuJoCoActuator};

    double position_kp{0.0};
    double velocity_kd{0.0};

    std::optional<double> command_min;
    std::optional<double> command_max;
};
```

## 12.3 JointBinding

```cpp
struct JointBinding {
    int joint_id{-1};
    int qpos_address{-1};
    int dof_address{-1};
    int joint_type{-1};

    int actuator_id{-1};
    int actuator_type{-1};

    bool has_actuator{false};
};
```

绑定在初始化阶段完成。

运行期间不重复调用 `mj_name2id()`。

## 12.4 JointCommand

```cpp
struct JointCommand {
    std::optional<double> position;
    std::optional<double> velocity;
    std::optional<double> effort;

    std::uint64_t timestamp_ns{0};
};
```

## 12.5 JointState

```cpp
struct JointState {
    double position{0.0};
    double velocity{0.0};
    double effort{0.0};

    std::uint64_t timestamp_ns{0};
};
```

## 12.6 actuator 类型校验

必须严格检查 command mode 和 actuator 类型。

### Position

允许：

* MuJoCo position actuator；
* 明确配置的软件 PD 控制。

不允许：

* 将 position target 直接写入普通 motor actuator。

### Velocity

允许：

* MuJoCo velocity actuator；
* 明确配置的软件速度控制。

### Effort

允许：

* motor/general actuator；
* 直接写入 `qfrc_applied` 的无 actuator 模式。

## 12.7 软件 PD 控制

```text
tau =
    kp × (q_target - q)
  + kd × (dq_target - dq)
  + effort_feedforward
```

必须包含：

* command limit；
* actuator control limit；
* force limit；
* NaN 检查；
* 非法状态检查；
* optional damping。

## 12.8 effort state 语义

默认 JointState 的 effort 定义为：

```text
data->qfrc_actuator[dof_address]
```

该值表示 actuator 对关节自由度的作用力。

需要在文档中明确：

* 不等于完整关节总力矩；
* 不包含全部外力和约束力；
* 可后续增加 effort source 配置。

## 12.9 reset

JointComponent reset 必须清理：

* 内部最后命令；
* `data->ctrl`；
* `data->qfrc_applied`；
* 软件控制器状态；
* command timeout 状态。

---

# 13. MobileBaseComponent

## 13.1 定位

MobileBase 是多个 Joint 组成的组合控制组件。

它负责：

* 接收底盘速度命令；
* 将速度命令转换为轮速；
* 提交轮关节命令；
* 根据轮速计算底盘速度；
* 可选计算轮式里程计；
* 可选读取 ground truth body pose。

## 13.2 不保存 Joint 指针

禁止：

```cpp
std::vector<JointComponent*> joints_;
```

使用解析后的 JointBinding 或关节组件名称：

```cpp
struct DifferentialBaseBinding {
    JointBinding left_wheel;
    JointBinding right_wheel;
};
```

或者由配置构建时复制必要的地址信息。

## 13.3 差速底盘配置

```cpp
struct DifferentialBaseConfig {
    std::string name;

    std::string left_wheel_joint;
    std::string right_wheel_joint;

    double wheel_radius{0.0};
    double track_width{0.0};

    OdometrySource odometry_source{
        OdometrySource::WheelIntegration};

    std::optional<std::string> base_body_name;
};
```

## 13.4 Mecanum 配置

必须使用有语义的字段：

```cpp
struct MecanumBaseConfig {
    std::string name;

    std::string front_left_joint;
    std::string front_right_joint;
    std::string rear_left_joint;
    std::string rear_right_joint;

    double wheel_radius{0.0};
    double wheel_base{0.0};
    double track_width{0.0};
};
```

禁止使用无顺序语义的 joint vector。

## 13.5 命令和状态

```cpp
struct MobileBaseCommand {
    double linear_x{0.0};
    double linear_y{0.0};
    double angular_z{0.0};

    std::uint64_t timestamp_ns{0};
};

struct MobileBaseState {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};

    double linear_x{0.0};
    double linear_y{0.0};
    double angular_z{0.0};

    std::vector<double> wheel_velocities;

    std::uint64_t timestamp_ns{0};
};
```

## 13.6 OdometrySource

```cpp
enum class OdometrySource {
    WheelIntegration,
    GroundTruthBodyPose
};
```

WheelIntegration 模拟真实轮式里程计。

GroundTruthBodyPose 用于评估和仿真调试。

---

# 14. SensorComponent 与 SensorScheduler

## 14.1 SensorConfig 公共字段

```cpp
struct SensorCommonConfig {
    std::string name;
    std::string frame_id;
    double update_rate{0.0};
};
```

## 14.2 SensorScheduler

负责：

* 判断每个 Sensor 是否到期；
* 根据仿真时间调度；
* reset 后清理下次采样时间；
* 统计 missed update；
* 防止传感器更新率高于物理频率。

```cpp
class SensorScheduler {
public:
    Status register_sensor(
        std::string_view name,
        double update_rate,
        double physics_rate);

    bool is_due(
        std::string_view name,
        double simulation_time) const;

    void mark_sampled(
        std::string_view name,
        double simulation_time);

    void reset();
};
```

## 14.3 推荐频率

| 组件               |   默认频率 |
| ---------------- | -----: |
| Joint state      |  每个物理步 |
| MobileBase state |  每个物理步 |
| IMU              | 200 Hz |
| Lidar            |  10 Hz |
| RGB Camera       |  30 Hz |
| Depth Camera     |  30 Hz |
| Viewer           |  60 Hz |

Sensor 的频率必须满足：

```text
update_rate <= physics_rate
```

否则初始化失败或明确降频，不能静默运行。

---

# 15. ImuComponent

## 15.1 配置

```cpp
struct ImuConfig {
    SensorCommonConfig common;

    std::string orientation_sensor;
    std::string angular_velocity_sensor;
    std::string linear_acceleration_sensor;

    std::array<double, 9> orientation_covariance{};
    std::array<double, 9> angular_velocity_covariance{};
    std::array<double, 9> linear_acceleration_covariance{};
};
```

## 15.2 Binding

```cpp
struct ImuBinding {
    int orientation_sensor_id{-1};
    int orientation_address{-1};

    int angular_velocity_sensor_id{-1};
    int angular_velocity_address{-1};

    int linear_acceleration_sensor_id{-1};
    int linear_acceleration_address{-1};
};
```

## 15.3 初始化校验

必须检查：

* sensor 名称存在；
* orientation 类型正确；
* orientation dimension 为 4；
* gyro 类型正确；
* gyro dimension 为 3；
* accelerometer 类型正确；
* accelerometer dimension 为 3；
* sensor address 合法。

## 15.4 四元数

MuJoCo 顺序：

```text
w, x, y, z
```

内部统一顺序：

```text
x, y, z, w
```

转换必须集中在公共工具函数。

## 15.5 ImuSample

```cpp
struct Quaternion {
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double w{1.0};
};

struct Vector3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

struct ImuSample {
    std::uint64_t sequence{0};
    std::uint64_t timestamp_ns{0};

    std::string frame_id;

    Quaternion orientation;
    Vector3 angular_velocity;
    Vector3 linear_acceleration;

    std::array<double, 9> orientation_covariance{};
    std::array<double, 9> angular_velocity_covariance{};
    std::array<double, 9> linear_acceleration_covariance{};
};
```

---

# 16. LidarComponent

## 16.1 定位

当前 Lidar 基于多个 MuJoCo rangefinder sensor 组合。

组件名称可使用：

```text
LidarComponent
```

内部实现类型：

```text
RangefinderLidar
```

为以后 `mj_ray` 或其他 raycast 实现保留空间。

## 16.2 配置

```cpp
struct LidarConfig {
    SensorCommonConfig common;

    std::string sensor_prefix;

    double angle_min{0.0};
    double angle_max{0.0};
    double angle_increment{0.0};

    double range_min{0.0};
    double range_max{0.0};
};
```

## 16.3 Binding

```cpp
struct LidarBeamBinding {
    std::size_t beam_index{0};
    int sensor_id{-1};
    int sensor_address{-1};
};

struct LidarBinding {
    std::vector<LidarBeamBinding> beams;
};
```

## 16.4 无效距离

无检测或越界使用：

```cpp
std::numeric_limits<float>::infinity()
```

不使用 `-1.0`。

## 16.5 时间语义

基于同一仿真时刻的多 rangefinder 采样：

```text
scan_time = 1 / update_rate
time_increment = 0
```

## 16.6 LidarSample

```cpp
struct LidarSample {
    std::uint64_t sequence{0};
    std::uint64_t timestamp_ns{0};

    std::string frame_id;

    float angle_min{0.0F};
    float angle_max{0.0F};
    float angle_increment{0.0F};

    float range_min{0.0F};
    float range_max{0.0F};

    float scan_time{0.0F};
    float time_increment{0.0F};

    std::vector<float> ranges;
    std::vector<float> intensities;
};
```

---

# 17. CameraComponent

## 17.1 职责

CameraComponent 负责：

* CameraConfig；
* MuJoCo camera 名称绑定；
* 采样频率；
* frame_id 和 optical_frame_id；
* CameraInfo 计算；
* 请求 CameraRenderer 渲染；
* 组装 CameraSample；
* 写入 CameraBuffer。

CameraComponent 不负责：

* 创建 Viewer；
* 复用 Viewer 的 OpenGL context；
* 直接对 ROS 消息赋值；
* 管理 ROS publisher。

## 17.2 CameraConfig

```cpp
struct CameraConfig {
    SensorCommonConfig common;

    std::string camera_name;
    std::string optical_frame_id;

    std::uint32_t width{640};
    std::uint32_t height{480};

    bool enable_color{true};
    bool enable_depth{false};
};
```

## 17.3 CameraBinding

```cpp
struct CameraBinding {
    int camera_id{-1};
    double fovy_degrees{0.0};
};
```

## 17.4 CameraSample

`CameraSample` 属于 Camera 组件数据模型，因此放在：

```text
component/camera/camera_sample.hpp
```

不放在独立 render 目录。

```cpp
enum class PixelFormat {
    Rgb8,
    Bgr8,
    Rgba8
};

enum class DepthFormat {
    Float32Meters
};

struct ImageBuffer {
    std::uint32_t width{0};
    std::uint32_t height{0};
    std::uint32_t step{0};

    PixelFormat format{PixelFormat::Rgb8};

    std::vector<std::uint8_t> data;
};

struct DepthBuffer {
    std::uint32_t width{0};
    std::uint32_t height{0};

    DepthFormat format{
        DepthFormat::Float32Meters};

    std::vector<float> data;
};

struct CameraIntrinsics {
    double fx{0.0};
    double fy{0.0};
    double cx{0.0};
    double cy{0.0};

    std::array<double, 9> k{};
    std::array<double, 12> p{};
};

struct CameraSample {
    std::uint64_t sequence{0};
    std::uint64_t timestamp_ns{0};

    std::string frame_id;
    std::string optical_frame_id;

    std::optional<ImageBuffer> color;
    std::optional<DepthBuffer> depth;

    CameraIntrinsics intrinsics;
};
```

---

# 18. CameraRenderer

## 18.1 作用

CameraRenderer 的作用是：

> 在不依赖交互式 Viewer 窗口的情况下，使用独立渲染上下文将 MuJoCo Camera 渲染为 RGB 和 Depth 数据。

它用于：

* headless Camera；
* RGB-D 传感器；
* Viewer 与 Camera 并行运行；
* 避免 OpenGL context 跨线程共享；
* 避免 Camera 修改 Viewer 的 `mjvScene`。

## 18.2 为什么不复用 Viewer

Viewer 面向用户：

* 窗口显示；
* 鼠标交互；
* 自由视角；
* UI；
* perturb。

Camera 面向算法：

* 固定 camera；
* 固定分辨率；
* RGB；
* Depth；
* 固定采样率；
* 仿真时间戳。

两者必须独立。

## 18.3 资源模型

推荐一个 CameraRenderer 服务多个 CameraComponent：

```text
CameraRenderer
├── 独立 GL context
├── 独立 mjvScene
├── 独立 mjrContext
├── 独立 mjData snapshot
└── 渲染多个 CameraComponent
```

不为每个 Camera 创建一套完整 OpenGL context。

## 18.4 接口

```cpp
struct RenderedCameraBuffers {
    std::optional<ImageBuffer> color;
    std::optional<DepthBuffer> depth;
};

class CameraRenderer {
public:
    Status initialize(
        const mjModel& model,
        const CameraRendererConfig& config);

    Status shutdown();

    Status copy_simulation_data(
        const mjModel& model,
        const mjData& source);

    Result<RenderedCameraBuffers> render(
        const mjModel& model,
        const CameraBinding& binding,
        const CameraConfig& config);

private:
    MjDataPtr render_data_;

    mjvScene scene_{};
    mjrContext render_context_{};

    std::unique_ptr<OffscreenGlContext>
        gl_context_;
};
```

## 18.5 线程模型

所有 OpenGL 操作必须在 CameraRenderer 所属线程执行。

可采用：

```text
Simulation thread
    ↓ copy mjData snapshot
Camera render thread
    ↓ render
CameraBuffer
    ↓ readers
```

初期也可以让 Scheduler 在固定渲染线程中同步提交，但禁止 ROS callback 线程直接调用 OpenGL。

## 18.6 Headless 定义

Headless 表示：

```text
不创建交互 Viewer
```

不表示禁用 Camera 渲染。

允许：

```text
viewer.enabled = false
camera.enabled = true
```

## 18.7 图像方向

OpenGL 图像原点通常为左下角。

输出图像必须转换为左上角原点。

RGB 和 Depth 均需逐行垂直翻转。

## 18.8 CameraInfo

根据垂直视场角计算：

```text
fy = height / (2 × tan(fovy / 2))
fx = fy
cx = (width - 1) / 2
cy = (height - 1) / 2
```

矩阵：

```text
K =
[fx  0 cx
  0 fy cy
  0  0  1]

P =
[fx  0 cx 0
  0 fy cy 0
  0  0  1 0]
```

## 18.9 Depth

`mjr_readPixels` 获取的 depth buffer 不能直接声明为米制深度。

必须根据 MuJoCo 当前版本的渲染深度定义和 near/far clipping 转换为线性距离。

转换后：

```text
DepthFormat::Float32Meters
```

必须增加已知距离模型测试。

---

# 19. Viewer

## 19.1 职责

Viewer 只负责：

* 创建交互窗口；
* 显示仿真场景；
* 用户相机控制；
* MuJoCo UI；
* overlay；
* 可选 perturb；
* 展示仿真状态。

不负责：

* 物理步进；
* Camera Sensor；
* RGB-D 数据；
* ROS service；
* `ros2_control`；
* 组件注册。

## 19.2 更新频率

Viewer 更新频率独立于物理频率。

例如：

```text
physics = 1000 Hz
viewer = 60 Hz
```

不允许每个物理步都调用 Viewer 同步。

## 19.3 资源封装

Viewer 不向外暴露：

```cpp
mjvScene*
mjrContext*
GLFWwindow*
```

所有操作通过 Viewer 方法或命令队列完成。

## 19.4 异常处理

Viewer 渲染线程异常必须：

* 保存错误信息；
* 通知初始化线程；
* 将 Viewer 状态置为 Error；
* 不吞掉异常；
* 支持安全 shutdown。

---

# 20. CommandBuffer

## 20.1 作用

CommandBuffer 用于外部线程向仿真线程提交命令。

外部线程不直接写：

```text
data->ctrl
data->qfrc_applied
```

## 20.2 CommandSnapshot

```cpp
struct CommandSnapshot {
    std::unordered_map<
        std::string,
        JointCommand> joint_commands;

    std::unordered_map<
        std::string,
        MobileBaseCommand> mobile_base_commands;

    std::uint64_t sequence{0};
};
```

## 20.3 接口

```cpp
class CommandBuffer {
public:
    Status set_joint_command(
        std::string_view component_name,
        const JointCommand& command);

    Status set_mobile_base_command(
        std::string_view component_name,
        const MobileBaseCommand& command);

    CommandSnapshot snapshot() const;

    void clear();

private:
    mutable std::mutex mutex_;
    CommandSnapshot pending_;
};
```

## 20.4 命令超时

```cpp
enum class CommandTimeoutBehavior {
    KeepLast,
    ZeroCommand,
    HoldPosition
};

struct CommandTimeoutConfig {
    bool enabled{true};
    double timeout_seconds{0.2};

    CommandTimeoutBehavior behavior{
        CommandTimeoutBehavior::ZeroCommand};
};
```

超时策略：

* velocity：归零；
* effort：归零；
* position：保持最后目标或当前位置；
* MobileBase：速度归零。

---

# 21. StateBuffer

## 21.1 作用

StateBuffer 用于仿真线程发布不可变状态快照。

其他线程不直接访问主 `mjData`。

## 21.2 SimulationStateSnapshot

```cpp
struct SimulationStateSnapshot {
    std::uint64_t sequence{0};
    std::uint64_t timestamp_ns{0};

    double simulation_time{0.0};
    std::uint64_t step_count{0};

    std::unordered_map<
        std::string,
        JointState> joints;

    std::unordered_map<
        std::string,
        MobileBaseState> mobile_bases;

    std::unordered_map<
        std::string,
        ImuSample> imus;

    std::unordered_map<
        std::string,
        LidarSample> lidars;
};
```

Camera 图像不放入主状态快照，原因：

* 数据量大；
* 更新频率不同；
* 拷贝成本高；
* Camera 使用独立缓存更合适。

## 21.3 双缓冲

推荐：

```cpp
class StateBuffer {
public:
    void publish(
        std::shared_ptr<
            const SimulationStateSnapshot> snapshot);

    std::shared_ptr<
        const SimulationStateSnapshot> read() const;

private:
    std::shared_ptr<
        const SimulationStateSnapshot> current_;
};
```

实现使用原子 shared pointer 交换或受控 mutex。

同一个 snapshot 内的：

* Joint；
* MobileBase；
* IMU；
* Lidar；
* simulation time；
* step count；

必须对应同一个仿真步或明确的采样时刻。

---

# 22. CameraBuffer

## 22.1 作用

CameraBuffer 独立缓存各相机最近一次采样。

```cpp
class CameraBuffer {
public:
    void publish(
        std::string_view camera_name,
        std::shared_ptr<const CameraSample> sample);

    Result<std::shared_ptr<const CameraSample>>
    read(std::string_view camera_name) const;

    void clear();

private:
    mutable std::mutex mutex_;

    std::unordered_map<
        std::string,
        std::shared_ptr<const CameraSample>>
        samples_;
};
```

CameraSample 使用共享不可变对象，避免大图像重复复制。

---

# 23. Reset 设计

## 23.1 ResetOptions

```cpp
struct ResetOptions {
    std::optional<std::string> keyframe_name;
    std::optional<int> keyframe_id;

    bool clear_commands{true};
    bool reset_components{true};
    bool clear_state_buffer{true};
    bool clear_camera_buffer{true};
    bool reset_statistics{false};
};
```

## 23.2 Reset 提交语义

当前代码状态：

* `Simulation::request_reset(...)` 负责异步提交 reset 请求；
* `Simulation::reset(...)` 负责提交后等待完成；
* scheduler 内部仍通过 `ResetRequest + completion promise` 支撑等待式封装，但该差异不再暴露成额外公开类型。

## 23.3 Reset 顺序

```text
1. Scheduler 接收 reset 请求
2. 暂停命令写入
3. 清空 CommandBuffer
4. mj_resetData
5. 应用 keyframe
6. mj_forward
7. reset JointComponent
8. reset MobileBaseComponent
9. reset Imu/Lidar/Camera
10. reset SensorScheduler
11. 清空 StateBuffer
12. 清空 CameraBuffer
13. 生成初始状态快照
14. 重置实时同步 deadline
15. 恢复 reset 前运行状态
```

上面的顺序已基本对应当前实现；其中 `ModelRuntime::reset(...)` 已只负责 model/data reset + keyframe 应用，`clear_commands`、buffer 清理、statistics reset 和初始快照发布由 `Simulation` / `SimulationScheduler` 解释执行。

## 23.4 失败处理

任一步失败：

* 返回完整 `Status`；
* Scheduler 进入 `Error` 或安全 `Paused`；
* 不允许继续使用部分 reset 状态；
* 错误信息包含失败阶段和组件名称。

---

# 24. 错误处理

## 24.1 Status

```cpp
enum class ErrorCode {
    Ok,
    InvalidArgument,
    InvalidState,
    NotFound,
    AlreadyExists,
    ModelLoadFailed,
    ModelValidationFailed,
    BindingFailed,
    CommandRejected,
    RenderFailed,
    ThreadFailed,
    Timeout,
    InternalError
};

class Status {
public:
    static Status Ok();

    Status(
        ErrorCode code,
        std::string message);

    bool ok() const noexcept;
    ErrorCode code() const noexcept;
    const std::string& message() const noexcept;

private:
    ErrorCode code_{ErrorCode::Ok};
    std::string message_;
};
```

## 24.2 Result

```cpp
template<typename T>
class Result {
public:
    Result(T value);
    Result(Status status);

    bool ok() const noexcept;
    const Status& status() const noexcept;

    T& value();
    const T& value() const;

private:
    std::optional<T> value_;
    Status status_;
};
```

## 24.3 禁止共享 last_error

不再使用：

```cpp
bool read(...);
const std::string& last_error() const;
```

原因：

* 状态共享；
* 并发不安全；
* 错误来源不明确；
* 后续调用可能覆盖错误。

## 24.4 错误信息要求

错误应包含：

* 操作；
* 组件名称；
* MuJoCo 元素名称；
* 期望值；
* 实际值；
* 当前 SimulationStatus。

示例：

```text
Failed to bind JointComponent 'left_wheel':
actuator 'left_wheel_motor' is a motor actuator,
but command mode is Position and controller type is MuJoCoActuator.
```

---

# 25. 配置系统

## 25.1 SimulationConfig

```cpp
struct SimulationConfig {
    ModelConfig model;
    SchedulerConfig scheduler;
    ComponentConfigList components;
    ViewerConfig viewer;
    CameraRendererConfig camera_renderer;
};
```

## 25.2 ModelConfig

```cpp
struct ModelConfig {
    std::string model_path;
    std::optional<std::string> initial_keyframe;
};
```

## 25.3 配置构建流程

```text
Parse external config
    ↓
Convert to C++ config
    ↓
Validate generic fields
    ↓
Load MuJoCo model
    ↓
Resolve bindings
    ↓
Validate model-specific fields
    ↓
Build components
    ↓
Initialize buffers
    ↓
Initialize renderer/viewer
    ↓
Start scheduler
```

任何阶段失败均完整回滚。

## 25.4 外部配置解析

核心库不直接依赖 YAML 或 ROS parameter。

可由以下模块转换：

```text
robot_mujoco
mujoco_simulation_ros
mujoco_simulation_config
```

## 25.5 YAML 示例

```yaml
simulation:
  model_path: models/h10_w/scene.xml
  initial_keyframe: home

scheduler:
  realtime_sync: true
  realtime_factor: 1.0
  state_update_rate: 1000.0
  viewer_update_rate: 60.0

viewer:
  enabled: true

camera_renderer:
  enabled: true
  backend: glfw_hidden

components:
  joints:
    - name: left_wheel
      joint_name: left_wheel_joint
      actuator_name: left_wheel_motor
      command_mode: velocity
      controller_type: mujoco_actuator

    - name: right_wheel
      joint_name: right_wheel_joint
      actuator_name: right_wheel_motor
      command_mode: velocity
      controller_type: mujoco_actuator

  mobile_bases:
    - name: base
      type: differential
      left_wheel_joint: left_wheel
      right_wheel_joint: right_wheel
      wheel_radius: 0.1
      track_width: 0.45
      odometry_source: wheel_integration

  imus:
    - name: base_imu
      frame_id: imu_link
      update_rate: 200.0
      orientation_sensor: imu_quaternion
      angular_velocity_sensor: imu_gyro
      linear_acceleration_sensor: imu_accelerometer

  lidars:
    - name: front_lidar
      frame_id: lidar_link
      update_rate: 10.0
      sensor_prefix: lidar_ray
      angle_min: -3.1415926
      angle_max: 3.1415926
      angle_increment: 0.00872665
      range_min: 0.1
      range_max: 20.0

  cameras:
    - name: front_camera
      camera_name: front_camera
      frame_id: front_camera_link
      optical_frame_id: front_camera_optical_frame
      update_rate: 30.0
      width: 640
      height: 480
      enable_color: true
      enable_depth: true
```

---

# 26. ROS 2 适配边界

## 26.1 包划分

建议最终包结构：

```text
mujoco_simulation
    纯 C++ MuJoCo Runtime

mujoco_hardware
    ros2_control SystemInterface

mujoco_simulation_ros
    Clock、Sensor Publisher、TF、Services

robot_mujoco
    模型、配置、launch、应用程序集成
```

## 26.2 ros2_control 数据路径

```text
ControllerManager
    ↓ write()
mujoco_hardware
    ↓ CommandBuffer
SimulationScheduler
    ↓ JointComponent write
mj_step
    ↓ ComponentManager read
StateBuffer
    ↓ mujoco_hardware read()
ControllerManager
```

`mujoco_hardware::write()` 不直接访问 `mjData`。

`mujoco_hardware::read()` 不直接读取 `mjData`。

## 26.3 Sensor 发布

```text
StateBuffer
├── JointState
├── IMU
├── Lidar
└── MobileBase

CameraBuffer
└── CameraSample
```

ROS Adapter 转换为：

* `sensor_msgs/msg/JointState`；
* `sensor_msgs/msg/Imu`；
* `sensor_msgs/msg/LaserScan`；
* `sensor_msgs/msg/Image`；
* `sensor_msgs/msg/CameraInfo`；
* `nav_msgs/msg/Odometry`；
* TF。

核心库不包含 ROS message 类型。

## 26.4 ROS clock

ROS Adapter 根据 StateSnapshot 中的：

```text
simulation_time
```

发布 `/clock`。

## 26.5 Service

建议支持：

```text
/start
/stop
/pause
/resume
/reset
/step
/load_keyframe
/set_realtime_factor
```

Service callback 只提交 Scheduler request。

不得在 callback 线程直接调用：

```text
mj_step
mj_resetData
mj_forward
```

---

# 27. 修改后的目录结构

```text
mujoco_simulation/
├── CMakeLists.txt
├── package.xml
├── README.md
│
├── include/mujoco_simulation/
│   ├── simulation.hpp
│   ├── simulation_config.hpp
│   ├── simulation_status.hpp
│   ├── reset_options.hpp
│   ├── status.hpp
│   ├── result.hpp
│   │
│   ├── runtime/
│   │   ├── model_runtime.hpp
│   │   ├── simulation_scheduler.hpp
│   │   ├── simulation_clock.hpp
│   │   ├── simulation_request.hpp
│   │   ├── scheduler_config.hpp
│   │   └── mujoco_raii.hpp
│   │
│   ├── component/
│   │   ├── simulation_component.hpp
│   │   ├── sensor_component.hpp
│   │   ├── component_manager.hpp
│   │   ├── component_registry.hpp
│   │   ├── component_config.hpp
│   │   │
│   │   ├── joint/
│   │   │   ├── joint_component.hpp
│   │   │   ├── joint_config.hpp
│   │   │   ├── joint_binding.hpp
│   │   │   ├── joint_command.hpp
│   │   │   └── joint_state.hpp
│   │   │
│   │   ├── mobile_base/
│   │   │   ├── mobile_base_component.hpp
│   │   │   ├── mobile_base_config.hpp
│   │   │   ├── mobile_base_binding.hpp
│   │   │   ├── mobile_base_command.hpp
│   │   │   └── mobile_base_state.hpp
│   │   │
│   │   ├── imu/
│   │   │   ├── imu_component.hpp
│   │   │   ├── imu_config.hpp
│   │   │   ├── imu_binding.hpp
│   │   │   └── imu_sample.hpp
│   │   │
│   │   ├── lidar/
│   │   │   ├── lidar_component.hpp
│   │   │   ├── lidar_config.hpp
│   │   │   ├── lidar_binding.hpp
│   │   │   └── lidar_sample.hpp
│   │   │
│   │   └── camera/
│   │       ├── camera_component.hpp
│   │       ├── camera_config.hpp
│   │       ├── camera_binding.hpp
│   │       ├── camera_sample.hpp
│   │       ├── camera_renderer.hpp
│   │       ├── camera_renderer_config.hpp
│   │       └── offscreen_gl_context.hpp
│   │
│   ├── buffer/
│   │   ├── command_buffer.hpp
│   │   ├── command_snapshot.hpp
│   │   ├── state_buffer.hpp
│   │   ├── simulation_state_snapshot.hpp
│   │   └── camera_buffer.hpp
│   │
│   ├── scheduler/
│   │   └── sensor_scheduler.hpp
│   │
│   └── viewer/
│       ├── viewer.hpp
│       └── viewer_config.hpp
│
├── src/
│   ├── simulation.cpp
│   │
│   ├── runtime/
│   │   ├── model_runtime.cpp
│   │   ├── simulation_scheduler.cpp
│   │   └── simulation_clock.cpp
│   │
│   ├── component/
│   │   ├── component_manager.cpp
│   │   ├── component_registry.cpp
│   │   ├── joint/
│   │   ├── mobile_base/
│   │   ├── imu/
│   │   ├── lidar/
│   │   └── camera/
│   │
│   ├── buffer/
│   │   ├── command_buffer.cpp
│   │   ├── state_buffer.cpp
│   │   └── camera_buffer.cpp
│   │
│   ├── scheduler/
│   │   └── sensor_scheduler.cpp
│   │
│   └── viewer/
│       └── viewer.cpp
│
├── test/
│   ├── unit/
│   ├── integration/
│   ├── concurrency/
│   ├── performance/
│   └── models/
│
└── docs/
    ├── architecture.md
    ├── component_development.md
    ├── camera_rendering.md
    ├── ros2_integration.md
    └── migration_guide.md
```

---

# 28. 构建目标

建议按依赖拆分 CMake target：

```cmake
add_library(mujoco_simulation_runtime ...)
add_library(mujoco_simulation_components ...)
add_library(mujoco_simulation_camera ...)
add_library(mujoco_simulation_viewer ...)
add_library(mujoco_simulation ...)
```

依赖关系：

```text
runtime
  ↑
components
  ↑
mujoco_simulation

runtime ← camera
runtime ← viewer
components ← camera
```

纯物理、无 Camera、无 Viewer 场景可以只链接：

```text
mujoco_simulation_runtime
mujoco_simulation_components
```

Viewer 和 OpenGL 不应成为 ModelRuntime 的强制依赖。

---

# 29. 初始化与销毁流程

## 29.1 初始化

```text
Simulation::initialize
    ↓
校验基础配置
    ↓
ModelRuntime::load
    ↓
ComponentManager::build
    ↓
创建 CommandBuffer / StateBuffer / CameraBuffer
    ↓
初始化 CameraRenderer（如启用）
    ↓
初始化 Viewer（如启用）
    ↓
初始化 SimulationScheduler
    ↓
执行初始 reset
    ↓
生成初始 StateSnapshot
    ↓
状态置为 Stopped
```

## 29.2 初始化失败回滚

任何失败必须按逆序销毁：

```text
Scheduler
Viewer
CameraRenderer
Buffers
Components
ModelRuntime
```

失败后：

```text
SimulationStatus = Uninitialized
```

允许重新 initialize。

## 29.3 shutdown

```text
请求 Scheduler stop
    ↓
等待仿真线程退出
    ↓
停止 CameraRenderer
    ↓
停止 Viewer
    ↓
清空 Buffers
    ↓
销毁 Components
    ↓
ModelRuntime::unload
    ↓
状态置为 Uninitialized
```

---

# 30. 分阶段开发计划

## 阶段 0：建立重构基线

### 任务

* 固定当前稳定 commit；
* 记录现有目录和接口；
* 为当前 Joint、IMU、Lidar、Camera、MobileBase 建立测试；
* 记录性能基线；
* 创建重构分支；
* 启用 ASan 和 UBSan 构建。

### 输出

* `docs/current_architecture.md`
* `docs/migration_guide.md`
* 基础测试模型；
* 性能基线报告。

### 验收

* 当前功能可重复构建和运行；
* 各组件至少一个集成测试；
* 保存物理频率、CPU、内存、Viewer 和 Camera 数据。

---

## 阶段 1：基础类型与 ModelRuntime

### 任务

* 增加 `Status`；
* 增加 `Result<T>`；
* 增加 MuJoCo RAII；
* 新建 `ModelRuntime`；
* 从旧类迁移 load、step、forward、reset；
* 保证初始化失败完整回滚；
* 增加 ModelRuntime 单元测试。

### 验收

* XML 加载失败无泄漏；
* `mjData` 正确销毁；
* reset/keyframe 可测试；
* 重复 initialize/shutdown 正常；
* 不再分散手工销毁 model/data。

---

## 阶段 2：SimulationScheduler

### 任务

* 新建 Scheduler；
* 实现状态机；
* 实现仿真线程；
* 修复累积 deadline；
* 实现 pause/resume；
* 限制 manual step；
* 引入 request queue；
* 增加 Scheduler 统计。

### 验收

* 未初始化不能 start；
* Running 不能 manual step；
* pause 状态不执行 `mj_step`；
* reset 请求在线程内执行；
* 1 kHz 仿真实时因子优于旧实现；
* start/stop 循环无死锁。

---

## 阶段 3：Component 基础和 JointComponent

### 任务

* 新建 `SimulationComponent`；
* 新建 `ComponentManager`；
* 新建 `ComponentRegistry`；
* 重构 JointConfig/Binding/Command/State；
* 严格 actuator 类型检查；
* 支持 MuJoCo actuator 和软件 PD；
* reset 时清理 ctrl/qfrc；
* 删除旧 Joint 直接持有 `MjContext` 方式。

### 验收

* Position/Velocity/Effort 正确；
* 错误配置初始化失败；
* reset 后命令清零；
* 运行期不重复 `mj_name2id`；
* Joint 单元测试通过。

---

## 阶段 4：MobileBaseComponent

### 任务

* 删除 MobileBase 中裸 `Joint*`；
* 增加 Differential binding；
* 增加 Mecanum binding；
* 实现速度命令分解；
* 实现 wheel integration；
* 支持 ground truth pose；
* 增加运动学测试。

### 验收

* 无 use-after-free；
* ASan 通过；
* 差速正逆运动学通过；
* Mecanum 正逆运动学通过；
* reset 后里程计归零；
* 关节重配置不会破坏 MobileBase。

---

## 阶段 5：CommandBuffer 与 StateBuffer

### 任务

* 新建 CommandBuffer；
* 所有命令通过 Buffer；
* 仿真线程集中写入命令；
* 实现 command timeout；
* 新建 StateBuffer；
* 发布不可变 StateSnapshot；
* 上层 read 改为读取快照。

### 验收

* 外部线程不写 `mjData`；
* read 不锁住物理步进；
* command 在一个物理周期内生效；
* snapshot 时间一致；
* 并发读写无数据竞争。

---

## 阶段 6：SensorComponent 和 SensorScheduler

### 任务

* 新建 SensorComponent；
* 新建 SensorScheduler；
* 重构 IMU；
* 重构 Lidar；
* 增加仿真时间戳；
* 增加 sensor 类型和维度校验；
* Lidar 无效距离使用 infinity；
* reset 调度器。

### 验收

* IMU 更新率稳定；
* Lidar 更新率稳定；
* reset 后无负 scan time；
* 非法 sensor 配置初始化失败；
* 同一采样内时间戳一致。

---

## 阶段 7：CameraRenderer 和 CameraComponent

### 任务

* Camera 脱离 Viewer；
* 实现独立 GL context；
* 实现独立 `mjvScene` 和 `mjrContext`；
* 实现 render data snapshot；
* 支持 headless Camera；
* 垂直翻转 RGB/Depth；
* 计算 CameraInfo；
* 深度转换为米；
* 新建 CameraBuffer；
* 支持多个 Camera 共享 Renderer。

### 验收

* Viewer 关闭时 Camera 可用；
* Viewer 与 Camera 并行无冲突；
* RGB 方向正确；
* Depth 米制正确；
* CameraInfo 投影正确；
* 多 Camera 可按频率工作；
* Camera 线程安全测试通过。

---

## 阶段 8：Viewer 解耦

### 任务

* Viewer 不参与物理循环；
* Viewer 频率独立；
* 不暴露渲染裸指针；
* 增加线程错误传递；
* 增加安全 stop；
* 支持可选 perturb command queue。

### 验收

* 物理 1 kHz、Viewer 60 Hz；
* Viewer 关闭不影响仿真；
* Viewer 崩溃可返回明确错误；
* shutdown 无线程残留。

---

## 阶段 9：Simulation 顶层接口

### 任务

* 新建 `Simulation`；
* 组合所有子系统；
* 删除旧 `MuJoCoSimulation`；
* 更新 demo；
* 更新 tests；
* 更新上层调用；
* 编写迁移文档。

### 验收

* 上层只依赖 `Simulation`；
* 顶层类不包含具体组件业务逻辑；
* 所有旧 API 已迁移；
* 无过渡 Facade 长期遗留。

---

## 阶段 10：ROS 2 集成迁移

### 任务

* `mujoco_hardware` 使用 CommandBuffer；
* `mujoco_hardware` 使用 StateBuffer；
* Sensor publisher 使用 Samples；
* `/clock` 使用 simulation time；
* Service 使用 Scheduler request；
* 核心移除 `rclcpp`；
* 核心移除 `sensor_msgs`。

### 验收

* ros2_control 正常运行；
* JointState/IMU/Lidar/Camera 时间一致；
* reset service 不阻塞 executor；
* 核心库无 ROS 环境可编译。

---

## 阶段 11：清理和稳定化

### 任务

* 删除旧代码；
* 删除 deprecated API；
* 清理未使用依赖；
* 增加 clang-tidy；
* 增加 CI；
* 完善文档；
* 增加性能测试；
* 增加长时间稳定性测试。

### 验收

* `mujoco_simulation` runtime-core ASan/UBSan 通过，TSan 子集在支持的宿主机上通过，或至少稳定产出 `tsan_preflight.json` + `tsan_summary.{json,md}` 区分“runtime 不支持”与真实 race；
* CI 全部通过；
* 运行一小时无明显内存增长；
* 新增 Sensor 不需要修改 Scheduler 主流程；
* 新增 Component 不需要修改 ModelRuntime。

---

# 31. 测试方案

## 31.1 ModelRuntime

* XML 加载；
* MJB 加载；
* 非法模型；
* `mjData` 创建；
* step；
* multi-step；
* reset；
* keyframe；
* forward；
* unload；
* 重复 initialize/shutdown。

## 31.2 Scheduler

* 状态转换；
* start/stop；
* pause/resume；
* manual step；
* reset request；
* realtime factor；
* deadline lag recovery；
* thread shutdown；
* Error 状态。

## 31.3 Joint

* hinge；
* slide；
* 无 actuator；
* position actuator；
* velocity actuator；
* motor actuator；
* 软件 PD；
* limit；
* NaN command；
* timeout；
* reset。

## 31.4 MobileBase

* differential kinematics；
* mecanum kinematics；
* wheel order；
* wheel integration；
* ground truth；
* reset；
* command timeout。

## 31.5 IMU

* sensor type；
* dimension；
* quaternion order；
* timestamp；
* covariance；
* reset。

## 31.6 Lidar

* beam discovery；
* beam order；
* missing beam；
* invalid range；
* infinity；
* angle fields；
* update rate；
* reset。

## 31.7 Camera

* camera binding；
* RGB；
* Depth；
* vertical flip；
* CameraInfo；
* depth linearization；
* headless；
* Viewer parallel；
* multiple cameras；
* CameraBuffer；
* shutdown。

## 31.8 并发

使用：

* AddressSanitizer；
* UndefinedBehaviorSanitizer；
* ThreadSanitizer；
* Valgrind，可选；
* Helgrind，可选。

重点测试：

* 高频 command 写入；
* 高频 state 读取；
* reset 与 read 并发；
* start/stop 重复；
* Camera 与 Viewer 并发；
* shutdown 时 CameraRenderer 退出；
* model 销毁时无悬空引用。

## 31.9 性能指标

| 指标              |        目标 |
| --------------- | --------: |
| 1 kHz 物理实时因子    |    ≥ 0.95 |
| Joint 命令延迟      | ≤ 1 个物理周期 |
| Joint 状态延迟      | ≤ 1 个物理周期 |
| Viewer 更新率      |  60 Hz 稳定 |
| Camera 更新率      |  30 Hz 稳定 |
| Lidar 更新率       |  10 Hz 稳定 |
| 运行 1 小时内存增长     |      接近 0 |
| 无 Viewer CPU 占用 |    不高于旧实现 |
| reset           |  无死锁、结果可测 |

---

# 32. 代码规范

## 32.1 C++ 规范

* 使用 C++17；
* owning pointer 必须使用 RAII；
* 禁止裸 owning pointer；
* 借用指针必须有明确生命周期；
* 公共 API 明确线程安全性；
* 使用 `enum class`；
* 配置使用强类型；
* 不使用异常作为普通流程；
* 第三方异常转换为 `Status`；
* 避免运行期字符串解析；
* 避免每步动态分配；
* 快照使用预分配或双缓冲优化。

## 32.2 命名规范

| 类型          | 命名                     |
| ----------- | ---------------------- |
| 顶层入口        | `Simulation`           |
| MuJoCo 运行数据 | `ModelRuntime`         |
| 调度器         | `SimulationScheduler`  |
| 组件          | `XxxComponent`         |
| 配置          | `XxxConfig`            |
| 模型绑定        | `XxxBinding`           |
| 命令          | `XxxCommand`           |
| 状态          | `XxxState`             |
| 传感器输出       | `XxxSample`            |
| 缓存          | `XxxBuffer`            |
| 快照          | `XxxSnapshot`          |
| 错误结果        | `Status` / `Result<T>` |

不再使用含义模糊的：

```text
Entity
Info
Spec
Data
Core
Hardware
```

除非语义确实准确。

## 32.3 日志

核心库定义抽象日志接口或回调。

核心不直接使用：

```cpp
RCLCPP_INFO
RCLCPP_ERROR
```

ROS Adapter 负责映射到 rclcpp logger。

---

# 33. 风险与应对

## 33.1 重构范围大

风险：

* 多模块同时变化；
* 上层接口大量调整；
* 回归定位困难。

应对：

* 分阶段提交；
* 每阶段保持可编译；
* 先写测试；
* 每阶段完成后再进入下一阶段；
* 禁止长时间保留新旧两套并行架构。

## 33.2 Component 抽象过度

风险：

* 所有组件被迫使用相同接口；
* 出现大量空实现。

应对：

* `SimulationComponent` 只包含 bind/reset；
* 具体能力由具体类型提供；
* 只为 Sensor 提供额外基类；
* 不设计通用 read/write 虚函数。

## 33.3 Camera 渲染兼容性

风险：

* GLFW hidden window、EGL 和 OSMesa 差异；
* CI 无显示环境；
* OpenGL 线程限制。

应对：

* 第一阶段只实现 GLFW hidden；
* GL 操作固定单线程；
* 后端封装在 `OffscreenGlContext`；
* Camera 功能可选编译；
* 后续再支持 EGL。

## 33.4 Snapshot 拷贝成本

风险：

* map 和 vector 每步复制；
* 多关节状态开销增大。

应对：

* 初期优先正确性；
* Camera 独立 Buffer；
* 后续改为预分配；
* 使用稳定 index 替代内部 map；
* 对外查询仍可保留名称接口。

## 33.5 Component 依赖

风险：

* MobileBase 依赖 Joint；
* Camera 依赖 Renderer；
* 组件构建顺序复杂。

应对：

* 配置解析后统一 resolve；
* 使用 Binding，不保存组件裸指针；
* ComponentManager 分两阶段：

  * create；
  * bind/resolve。

## 33.6 ROS 2 Humble 仅 C++17

风险：

* 无 `std::expected`；
* 部分原子 shared pointer 能力受限。

应对：

* 自定义 `Result<T>`；
* 必要时使用 mutex 保护 shared pointer；
* 不引入 C++20 依赖。

---

# 34. 完成定义

重构完成必须满足：

1. 原 `MuJoCoSimulation` 类已删除；
2. 对外入口统一为 `Simulation`；
3. `ModelRuntime` 独占 `mjModel/mjData`；
4. 主 `mjData` 只由仿真线程写；
5. 外部命令通过 CommandBuffer；
6. 外部状态通过 StateBuffer；
7. Camera 使用 CameraBuffer；
8. 所有设备采用 Component 模型；
9. 不存在 Entity 顶层抽象；
10. 不存在独立 Actuator 顶层模块；
11. Joint 内部区分 JointBinding 与 ActuatorBinding；
12. MobileBase 不保存 Joint 裸指针；
13. Sensor 支持独立更新率；
14. reset 覆盖全部组件和缓存；
15. Viewer 与 CameraRenderer 独立；
16. headless 模式支持 RGB-D；
17. CameraInfo 正确；
18. 图像方向正确；
19. Depth 为米制；
20. 核心不依赖 ROS 2；
21. ros2_control 通过适配层使用 command/state；
22. ASan、UBSan、TSan 通过；
23. 主要组件有单元和集成测试；
24. 文档和迁移指南完整；
25. 性能不低于旧架构基线。

---

# 35. 首批实施任务

建议第一轮只执行以下三个 Sprint。

## Sprint 1：基础运行时

* [x] 新增 `Status`
* [x] 新增 `Result<T>`
* [x] 新增 MuJoCo RAII
* [x] 实现 `ModelRuntime`
* [x] 实现 load/step/reset/forward
* [x] 实现初始化失败回滚
* [x] 增加 ModelRuntime 测试源码
* [x] 建立 ASan/UBSan 构建

## Sprint 2：调度器

* [x] 实现 `SimulationScheduler`
* [x] 实现状态机
* [x] 实现 request queue
* [x] 实现 start/stop/pause/resume
* [x] 实现 manual step 限制
* [x] 修复 realtime deadline
* [x] 增加 Scheduler 测试源码
* [x] 记录实时因子对比

## Sprint 3：JointComponent

* [x] 实现 `SimulationComponent`
* [x] 实现 `ComponentManager`
* [x] 实现 Joint 配置结构（当前命名仍为 `JointConfig`）
* [x] 实现 `JointBinding`
* [x] 实现 `JointCommand`
* [x] 实现 `JointState`
* [x] 严格校验 actuator 类型
* [x] reset 清理 ctrl/qfrc
* [x] 增加 Joint 测试源码

前三个 Sprint 稳定后，再开始：

* MobileBase；
* CommandBuffer；
* StateBuffer；
* Sensor；
* CameraRenderer；
* ROS 2 迁移。

---

# 36. 总结

修改后的目标架构为：

```text
Simulation
├── ModelRuntime
├── SimulationScheduler
├── ComponentManager
│   ├── JointComponent
│   ├── MobileBaseComponent
│   └── SensorComponent
│       ├── ImuComponent
│       ├── LidarComponent
│       └── CameraComponent
├── CommandBuffer
├── StateBuffer
├── CameraBuffer
├── CameraRenderer
└── Viewer
```

核心边界如下：

```text
ModelRuntime 管 MuJoCo model/data
SimulationScheduler 管线程、时间和执行顺序
ComponentManager 管运行组件
JointComponent 管关节状态与控制
SensorComponent 管传感器采样
CommandBuffer 管输入
StateBuffer 管普通状态输出
CameraBuffer 管图像输出
CameraRenderer 管无窗口相机渲染
Viewer 管交互显示
Simulation 管系统组合和公共 API
```

该架构比原来的：

```text
MuJoCoSimulation
+ HardwareManager
+ Viewer resources shared by Camera
```

具有更清晰的职责边界、更可靠的线程模型和更好的扩展性，同时保持对 ROS 2 control、IMU、Lidar、RGB-D Camera 和移动底盘仿真的直接支持。
