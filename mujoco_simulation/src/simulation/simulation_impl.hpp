#pragma once
// Private Simulation PImpl contract. This header is never installed.

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "buffer/command_buffer.hpp"
#include "buffer/state_buffer.hpp"
#include "component/camera/camera_render_service.hpp"
#include "component/component_manager.hpp"
#include "data/command_snapshot.hpp"
#include "mujoco_simulation/simulation.hpp"
#include "runtime/simulation_runtime.hpp"
#include "runtime/simulation_scheduler.hpp"
#include "viewer/simulation_viewer.hpp"

namespace mujoco_simulation {

class Simulation::Impl {
public:
    bool initialize(const std::string& config_path);
    bool initialize(const SimulationConfig& config);
    bool shutdown();
    bool start();
    bool stop();
    bool pause();
    bool resume();
    bool reset(const std::string* keyframe_name);

    bool write_command(JointId id, const JointCommand& command);
    bool write_command(MobileBaseId id, const MobileBaseCommand& command);
    bool write_command(const RobotCommand& command);
    bool write_commands(const JointCommands& commands);
    bool write_commands(const MobileBaseCommands& commands);
    bool read_state(std::shared_ptr<const RobotState>& out) const;
    bool read_state(RobotState& out) const;
    bool read_state(JointId id, JointState& out) const;
    bool read_state(ImuId id, ImuState& out) const;
    bool read_state(CameraId id, CameraState& out) const;
    bool read_state(LidarId id, LidarState& out) const;
    bool read_state(MobileBaseId id, MobileBaseState& out) const;
    bool read_state(JointStates& out) const;
    bool read_state(ImuStates& out) const;
    bool read_state(CameraStates& out) const;
    bool read_state(LidarStates& out) const;
    bool read_state(MobileBaseStates& out) const;
    bool step(std::size_t count);
    std::uint64_t step_count() const;
    SimulationStatus status() const;
    double time() const;

    bool initialize_camera_renderer();
    bool initialize_scheduler();
    bool initialize_components();
    bool load_model(const ModelConfig& model_config);
    bool scheduler_run_task();
    bool write_state_snapshot_locked();
    bool build_state_snapshot_locked(std::uint64_t step, double simulation_time);
    bool build_state_snapshot(RobotState& snapshot) const;
    bool start_viewer();
    bool stop_viewer();
    bool scheduler_submit_viewer_sync_if_due();

    SimulationConfig config_;
    std::unique_ptr<CameraRenderService> camera_render_service_;
    CommandBuffer command_buffer_;
    StateBuffer state_buffer_;
    std::unique_ptr<SimulationRuntime> runtime_;
    mutable std::mutex lifecycle_mutex_;
    mutable std::mutex mujoco_mutex_;
    std::unique_ptr<SimulationScheduler> scheduler_;
    ComponentManager component_manager_;
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

}  // namespace mujoco_simulation
