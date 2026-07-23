#pragma once

#include <mujoco/mujoco.h>

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

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
#include "mujoco_simulation/data/robot_command.hpp"
#include "mujoco_simulation/data/robot_state.hpp"
#include "mujoco_simulation/result_code.hpp"
#include "mujoco_simulation/simulation_status.hpp"

namespace mujoco_simulation {

class SimulationViewer;
class SimulationRuntime;
class SimulationScheduler;

class Simulation {
 public:
  using SnapshotObserver = std::function<void(std::shared_ptr<const RobotState> snapshot)>;

  Simulation();
  ~Simulation();

  Simulation(const Simulation &) = delete;
  Simulation &operator=(const Simulation &) = delete;

  bool initialize(const SimulationConfig &config);
  bool shutdown();

  bool start();
  bool stop();
  bool pause();
  bool resume();
  bool reset();
  bool reset(std::string keyframe_name);

  bool write_command(std::string name, const JointCommand &command);
  bool write_command(std::string name, const MobileBaseCommand &command);
  bool write_command(const RobotCommand &command);

  bool read_state(std::shared_ptr<const RobotState> &state) const;
  bool read_state(RobotState &state) const;
  bool read_state(std::string name, JointState &state) const;
  bool read_state(std::string name, ImuState &state) const;
  bool read_state(std::string name, CameraState &state) const;
  bool read_state(std::string name, LidarState &state) const;
  bool read_state(std::string name, MobileBaseState &state) const;

  uint64_t step() const;
  SimulationStatus status() const;
  double time() const;
  void set_snapshot_observer(SnapshotObserver observer);

 private:
  static constexpr auto kDefaultViewerPeriod = std::chrono::milliseconds(16);

  bool initialize_scheduler();
  bool initialize_components();
  bool load_model(const ModelConfig &model_config);
  bool write_state_snapshot();
  bool write_state_snapshot_locked(std::shared_ptr<const RobotState> &published_snapshot);
  bool update_components_for_step_locked(std::uint64_t step, double simulation_time);
  bool build_state_snapshot_locked(std::uint64_t step, double simulation_time,
                                   std::shared_ptr<const RobotState> &published_snapshot);
  // Scheduler
  bool scheduler_run_task();
  bool scheduler_write_commands();
  bool scheduler_update_components();
  bool scheduler_step_physics();
  bool scheduler_sync_viewer_if_due();
  bool start_viewer();
  bool stop_viewer();
  bool build_state_snapshot(RobotState &snapshot) const;

  // Configuration
  SimulationConfig config_;
  // Buffer
  std::unique_ptr<CameraBuffer> camera_buffer_;
  std::unique_ptr<CameraRenderer> camera_renderer_;
  std::unique_ptr<CommandBuffer> command_buffer_;
  std::unique_ptr<StateBuffer> state_buffer_;
  // Runtime
  std::unique_ptr<SimulationRuntime> runtime_;
  mutable std::mutex runtime_mutex_;
  // Scheduler
  std::unique_ptr<SimulationScheduler> scheduler_;
  // Component
  ComponentManager component_manager_;
  // Viewer
  std::unique_ptr<SimulationViewer> viewer_;
  mutable std::mutex viewer_mutex_;
  std::chrono::steady_clock::time_point next_sync_time_{};
  bool runtime_failed_{false};
  std::uint64_t step_{0};
  std::uint64_t sequence_{0};
  // Snapshot Observer
  SnapshotObserver snapshot_observer_;
};

}  // namespace mujoco_simulation
