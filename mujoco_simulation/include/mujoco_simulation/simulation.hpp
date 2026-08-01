#pragma once

#include <mujoco/mujoco.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "mujoco_simulation/buffer/command_bus.hpp"
#include "mujoco_simulation/buffer/state_buffer.hpp"
#include "mujoco_simulation/component/camera/camera_component.hpp"
#include "mujoco_simulation/component/camera/camera_data.hpp"
#include "mujoco_simulation/component/component_manager.hpp"
#include "mujoco_simulation/component/component_name_index.hpp"
#include "mujoco_simulation/component/imu/imu_data.hpp"
#include "mujoco_simulation/component/joint/joint_data.hpp"
#include "mujoco_simulation/component/lidar/lidar_data.hpp"
#include "mujoco_simulation/component/mobile_base/mobile_base_data.hpp"
#include "mujoco_simulation/config/simulation_config.hpp"
#include "mujoco_simulation/data/command_snapshot.hpp"
#include "mujoco_simulation/data/robot_state.hpp"
#include "mujoco_simulation/mujoco/camera_renderer.hpp"
#include "mujoco_simulation/simulation_status.hpp"
#include "mujoco_simulation/visibility.hpp"

namespace mujoco_simulation {

class SimulationViewer;
class SimulationRuntime;
class SimulationScheduler;

class MUJOCO_SIMULATION_PUBLIC Simulation {
public:
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
  template <typename Command>
  bool write_command(ComponentId id, const Command &command) {
    static_assert(
        has_command_traits_v<Command>,
        "write_command<T> requires CommandTraits<T>::validate(const T&)");
    return command_bus_.write<Command>(id, command);
  }
  template <typename Command>
  bool write_commands(const CommandBatch<Command> &batch) {
    static_assert(
        has_command_traits_v<Command>,
        "write_commands<T> requires CommandTraits<T>::validate(const T&)");
    return command_bus_.write(batch);
  }

  bool read_state(std::shared_ptr<const RobotState> &state) const;
  bool read_state(RobotState &state) const;
  bool read_state(std::string name, JointState &state) const;
  bool read_state(std::string name, ImuState &state) const;
  bool read_state(std::string name, CameraState &state) const;
  bool read_state(std::string name, LidarState &state) const;
  bool read_state(std::string name, MobileBaseState &state) const;
  bool read_state(JointId id, JointState &state) const;
  bool read_state(ImuId id, ImuState &state) const;
  bool read_state(CameraId id, CameraState &state) const;
  bool read_state(LidarId id, LidarState &state) const;
  bool read_state(MobileBaseId id, MobileBaseState &state) const;
  bool read_state(JointStates &states) const;
  bool read_state(ImuStates &states) const;
  bool read_state(CameraStates &states) const;
  bool read_state(LidarStates &states) const;
  bool read_state(MobileBaseStates &states) const;

  bool step(std::size_t count = 1);
  uint64_t step_count() const;
  SimulationStatus status() const;
  double time() const;

private:
  bool initialize_camera_renderer();
  bool initialize_scheduler();
  bool initialize_components();
  bool load_model(const ModelConfig &model_config);
  bool write_state_snapshot_locked();
  bool build_state_snapshot_locked(std::uint64_t step, double simulation_time);
  // Scheduler
  bool scheduler_run_task();
  bool scheduler_submit_viewer_sync_if_due();
  bool start_viewer();
  bool stop_viewer();
  bool build_state_snapshot(RobotState &snapshot) const;
  bool reset_runtime_locked(const std::string *keyframe_name);
  void clear_name_indexes();

  // Configuration
  SimulationConfig config_;
  std::shared_ptr<const ComponentNameIndex> name_index_;
  // Buffer
  std::unique_ptr<CameraRenderer> camera_renderer_;
  CommandBus command_bus_;
  StateBuffer state_buffer_;
  // Runtime
  std::unique_ptr<SimulationRuntime> runtime_;
  mutable std::mutex lifecycle_mutex_;
  mutable std::mutex mujoco_mutex_;
  // Scheduler
  std::unique_ptr<SimulationScheduler> scheduler_;
  // Component
  ComponentManager component_manager_;
  // Viewer
  std::shared_ptr<SimulationViewer> viewer_;
  mutable std::mutex viewer_mutex_;
  std::chrono::steady_clock::time_point next_sync_time_{};
  std::atomic<bool> runtime_failed_{false};
  std::atomic<std::uint64_t> step_{0};
  std::uint64_t sequence_{0};
  CommandSnapshot applied_command_;
  std::uint64_t applied_command_sequence_{0};
  bool has_applied_command_{false};
};

} // namespace mujoco_simulation
