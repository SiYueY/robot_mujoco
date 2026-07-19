#pragma once

#include <mujoco/mujoco.h>

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>

#include "mujoco_simulation/buffer/camera_buffer.hpp"
#include "mujoco_simulation/buffer/command_buffer.hpp"
#include "mujoco_simulation/buffer/state_buffer.hpp"
#include "mujoco_simulation/component/camera/camera_component.hpp"
#include "mujoco_simulation/component/camera/camera_data.hpp"
#include "mujoco_simulation/component/camera/camera_renderer.hpp"
#include "mujoco_simulation/component/component_manager.hpp"
#include "mujoco_simulation/component/imu/imu_data.hpp"
#include "mujoco_simulation/component/joint/joint_data.hpp"
#include "mujoco_simulation/component/lidar/lidar_data.hpp"
#include "mujoco_simulation/component/mobile_base/mobile_base_data.hpp"
#include "mujoco_simulation/config/simulation_config.hpp"
#include "mujoco_simulation/result_code.hpp"
#include "mujoco_simulation/runtime/simulation_scheduler.hpp"
#include "mujoco_simulation/simulation_status.hpp"

namespace mujoco_simulation {

class SimulationViewer;
class SimulationRuntime;
class SimulationScheduler;

class Simulation {
 public:
  using SnapshotObserver = std::function<void(std::shared_ptr<const StateSnapshot> snapshot)>;

  Simulation();
  ~Simulation();

  Simulation(const Simulation &) = delete;
  Simulation &operator=(const Simulation &) = delete;

  ResultCode initialize(const SimulationConfig &config);
  ResultCode shutdown();

  ResultCode start();
  ResultCode stop();
  ResultCode pause();
  ResultCode resume();
  ResultCode set_realtime_factor(double realtime_factor);
  ResultCode request_reset();
  ResultCode request_reset_to_keyframe_name(std::string_view keyframe_name);
  ResultCode request_reset_to_keyframe_id(int keyframe_id);
  ResultCode reset();
  ResultCode reset_to_keyframe_name(std::string_view keyframe_name);
  ResultCode reset_to_keyframe_id(int keyframe_id);
  ResultCode step(uint32_t steps);

  ResultCode set_joint_command(const JointCommand &command);
  bool joint_state(std::string joint_name, JointState *out) const;

  bool imu_state(std::string imu_name, ImuState *out) const;
  bool camera_state(std::string camera_name, CameraState *out) const;
  bool lidar_state(std::string lidar_name, LidarState *out) const;

  ResultCode set_mobile_base_command(std::string name, const MobileBaseCommand &command);
  bool mobile_base_state(std::string name, MobileBaseState *out) const;

  uint64_t step_count() const;
  SimulationStatus status() const;
  double simulation_time() const;
  std::shared_ptr<const StateSnapshot> state_snapshot() const;
  void set_snapshot_observer(SnapshotObserver observer);

 private:
  static constexpr auto kDefaultViewerPeriod = std::chrono::milliseconds(16);

  bool initialize_scheduler();
  bool initialize_components();
  bool load_model(const ModelConfig &model_config);
  bool request_reset_internal(const ResetRequest &request);
  bool reset_internal(ResetRequest request);
  bool write_state_snapshot();
  bool write_state_snapshot_locked(std::shared_ptr<const StateSnapshot> *published_snapshot);
  bool update_components_for_step_locked(std::uint64_t step_count, double simulation_time);
  bool build_state_snapshot_locked(std::uint64_t step_count, double simulation_time,
                                   std::shared_ptr<const StateSnapshot> *published_snapshot);
  bool scheduler_run_cycle();
  bool scheduler_write_commands();
  bool scheduler_update_components();
  bool scheduler_step_physics();
  bool scheduler_sync_viewer_if_due();
  bool scheduler_reset(const ResetRequest &request);
  double scheduler_timestep() const;
  bool start_viewer();
  bool stop_viewer();
  bool build_state_snapshot(StateSnapshot *snapshot) const;

  SimulationConfig config_;
  std::unique_ptr<CameraBuffer> camera_buffer_;
  std::unique_ptr<CameraRenderer> camera_renderer_;
  std::unique_ptr<CommandBuffer> command_buffer_;
  std::unique_ptr<SimulationRuntime> simulation_runtime_;
  std::unique_ptr<SimulationScheduler> scheduler_;
  std::unique_ptr<StateBuffer> state_buffer_;

  ComponentManager component_manager_;
  std::unique_ptr<SimulationViewer> viewer_;
  std::chrono::steady_clock::time_point next_viewer_sync_time_{};
  bool runtime_failed_{false};
  std::uint64_t step_count_{0};
  std::uint64_t state_snapshot_sequence_{0};
  SnapshotObserver snapshot_observer_;

  mutable std::mutex runtime_mutex_;
  mutable std::mutex viewer_mutex_;
};

}  // namespace mujoco_simulation
