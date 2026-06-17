#pragma once

#include <mujoco/mujoco.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>

#include "mujoco_simulation/buffer/camera_buffer.hpp"
#include "mujoco_simulation/buffer/command_buffer.hpp"
#include "mujoco_simulation/buffer/state_buffer.hpp"
#include "mujoco_simulation/component/camera/camera_component.hpp"
#include "mujoco_simulation/component/camera/camera_config.hpp"
#include "mujoco_simulation/component/camera/camera_renderer.hpp"
#include "mujoco_simulation/component/camera/camera_sample.hpp"
#include "mujoco_simulation/component/component_manager.hpp"
#include "mujoco_simulation/component/imu/imu_config.hpp"
#include "mujoco_simulation/component/imu/imu_sample.hpp"
#include "mujoco_simulation/component/joint/joint_command.hpp"
#include "mujoco_simulation/component/joint/joint_config.hpp"
#include "mujoco_simulation/component/joint/joint_state.hpp"
#include "mujoco_simulation/component/lidar/lidar_config.hpp"
#include "mujoco_simulation/component/lidar/lidar_sample.hpp"
#include "mujoco_simulation/component/mobile_base/mobile_base_command.hpp"
#include "mujoco_simulation/component/mobile_base/mobile_base_config.hpp"
#include "mujoco_simulation/component/mobile_base/mobile_base_state.hpp"
#include "mujoco_simulation/reset_options.hpp"
#include "mujoco_simulation/runtime/model_runtime.hpp"
#include "mujoco_simulation/runtime/simulation_scheduler.hpp"
#include "mujoco_simulation/simulation_config.hpp"

namespace mujoco_simulation {

class Viewer;

class Simulation {
 public:
  Simulation();
  ~Simulation();

  Simulation(const Simulation &) = delete;
  Simulation &operator=(const Simulation &) = delete;

  Status initialize(const SimulationConfig &config);
  Status shutdown();

  Status start();
  Status stop();
  Status pause();
  Status resume();
  Status set_realtime_factor(double realtime_factor);
  Status request_reset(const ResetOptions &options = {});
  Status reset(const ResetOptions &options = {});
  Status step(uint32_t steps);

  Status reconfigure_component(const ComponentConfig &updated_component);
  Status set_joint_command(const JointCommand &command);
  Result<JointState> joint_state(std::string_view joint_name) const;

  Result<ImuSample> imu_sample(std::string_view imu_name) const;
  Result<std::shared_ptr<const CameraSample>> camera_sample(std::string_view camera_name) const;
  Result<LidarSample> lidar_sample(std::string_view lidar_name) const;

  Status set_mobile_base_command(std::string_view name, const MobileBaseCommand &command);
  Result<MobileBaseState> mobile_base_state(std::string_view name) const;

  uint64_t step_count() const;
  SimulationStatus status() const;
  double simulation_time() const;
  std::shared_ptr<const SimulationStateSnapshot> state_snapshot() const;

 private:
  struct PendingStateSnapshot {
    std::unordered_map<std::string, JointState> joints;
    std::unordered_map<std::string, MobileBaseState> mobile_bases;
  };

  Status initialize_scheduler();
  Status initialize_components();
  Status load_model(const ModelConfig &model_config);
  Status apply_component_reconfiguration_locked(const ComponentConfig &updated_component);
  Status read_component_states_locked(bool increment_step_count);
  Status publish_state_snapshot_locked(bool increment_step_count);
  Status scheduler_apply_commands_locked();
  Status scheduler_read_components_locked(bool increment_step_count);
  Status scheduler_step_physics_locked();
  Status scheduler_sync_viewer_if_due();
  Status scheduler_reset_locked(const ResetOptions &options);
  double scheduler_timestep_locked() const;
  Status start_viewer();

  SimulationConfig config_;
  std::unique_ptr<CameraBuffer> camera_buffer_;
  std::unique_ptr<CameraRenderer> camera_renderer_;
  std::unique_ptr<CommandBuffer> command_buffer_;
  std::unique_ptr<ModelRuntime> model_runtime_;
  std::unique_ptr<SimulationScheduler> scheduler_;
  std::unique_ptr<StateBuffer> state_buffer_;
  mjModel *model_ = nullptr;
  mjData *data_ = nullptr;

  ComponentManager component_manager_;
  std::unique_ptr<Viewer> viewer_;
  std::optional<PendingStateSnapshot> pending_state_snapshot_;
  std::chrono::steady_clock::time_point next_viewer_sync_time_{};
  std::string runtime_error_;
  std::uint64_t state_snapshot_sequence_{0};
  std::uint64_t state_snapshot_step_count_{0};

  mutable std::mutex mutex_;
};

}  // namespace mujoco_simulation
