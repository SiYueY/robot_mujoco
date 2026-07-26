#include "mujoco_simulation/simulation.hpp"

#include <chrono>
#include <cmath>
#include <exception>
#include <memory>
#include <utility>

#include "common/logging.hpp"
#include "mujoco_simulation/common/macro.hpp"
#include "mujoco_simulation/runtime/simulation_runtime.hpp"
#include "mujoco_simulation/runtime/simulation_scheduler.hpp"
#include "mujoco_simulation/viewer/simulation_viewer.hpp"

namespace mujoco_simulation {

Simulation::Simulation() = default;
Simulation::~Simulation() { UNUSED(shutdown()); }

bool Simulation::initialize(const SimulationConfig &config) {
  std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
  if (runtime_ != nullptr) {
    LOG_ERROR << "simulation is already initialized.";
    return false;
  }
  if (!std::isfinite(config.scheduler.viewer_update_rate) ||
      config.scheduler.viewer_update_rate <= 0.0) {
    LOG_ERROR << "viewer update rate must be finite and positive.";
    return false;
  }
  config_ = config;
  command_buffer_.clear();
  state_buffer_.clear();
  const auto cleanup = [this] {
    if (scheduler_ != nullptr) {
      UNUSED(scheduler_->shutdown());
      scheduler_.reset();
    }
    UNUSED(stop_viewer());
    std::lock_guard<std::mutex> mujoco_lock(mujoco_mutex_);
    camera_renderer_.reset();
    component_manager_.clear();
    runtime_.reset();
    runtime_failed_.store(false);
    step_.store(0);
    sequence_ = 0;
  };
  if (!load_model(config.model)) {
    LOG_ERROR << "failed to load the simulation model.";
    cleanup();
    return false;
  }
  if (!initialize_camera_renderer()) {
    LOG_ERROR << "failed to initialize the camera renderer.";
    cleanup();
    return false;
  }
  if (!initialize_components()) {
    LOG_ERROR << "failed to initialize simulation components.";
    cleanup();
    return false;
  }
  bool initial_state_published = false;
  {
    std::lock_guard<std::mutex> mujoco_lock(mujoco_mutex_);
    initial_state_published = write_state_snapshot_locked();
  }
  if (!initial_state_published) {
    LOG_ERROR << "failed to publish the initial simulation state.";
    cleanup();
    return false;
  }
  if (!initialize_scheduler()) {
    LOG_ERROR << "failed to initialize the simulation scheduler.";
    cleanup();
    return false;
  }
  if (!start_viewer()) {
    LOG_ERROR << "failed to start the simulation viewer.";
    cleanup();
    return false;
  }
  return true;
}

bool Simulation::shutdown() {
  std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
  if (scheduler_ != nullptr) {
    UNUSED(scheduler_->shutdown());
    scheduler_.reset();
  }
  UNUSED(stop_viewer());
  {
    std::lock_guard<std::mutex> mujoco_lock(mujoco_mutex_);
    camera_renderer_.reset();
    component_manager_.clear();
    runtime_.reset();
    runtime_failed_.store(false);
    step_.store(0);
    sequence_ = 0;
  }
  command_buffer_.clear();
  state_buffer_.clear();
  return true;
}

bool Simulation::start() {
  std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
  if (scheduler_ == nullptr) {
    LOG_ERROR << "simulation scheduler is not initialized.";
    return false;
  }
  bool needs_viewer = false;
  {
    std::lock_guard<std::mutex> viewer_lock(viewer_mutex_);
    needs_viewer = viewer_ == nullptr;
  }
  if (needs_viewer) {
    if (!start_viewer()) {
      LOG_ERROR << "failed to restart the simulation viewer.";
      return false;
    }
  }
  if (!scheduler_->start()) {
    LOG_ERROR << "failed to start the simulation scheduler.";
    return false;
  }
  return true;
}

bool Simulation::stop() {
  std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
  if (scheduler_ != nullptr) {
    if (!scheduler_->stop()) {
      LOG_ERROR << "failed to stop the simulation scheduler.";
      return false;
    }
  }
  runtime_failed_.store(false);
  command_buffer_.clear();
  state_buffer_.clear();
  UNUSED(stop_viewer());
  return true;
}

bool Simulation::pause() {
  std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
  if (scheduler_ == nullptr) {
    LOG_ERROR << "simulation scheduler is not initialized.";
    return false;
  }
  if (!scheduler_->pause()) {
    LOG_ERROR << "failed to pause the simulation scheduler.";
    return false;
  }
  return true;
}

bool Simulation::resume() {
  std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
  if (scheduler_ == nullptr) {
    LOG_ERROR << "simulation scheduler is not initialized.";
    return false;
  }
  if (!scheduler_->resume()) {
    LOG_ERROR << "failed to resume the simulation scheduler.";
    return false;
  }
  return true;
}

bool Simulation::reset() { return reset_runtime_locked(nullptr); }

bool Simulation::reset(std::string keyframe_name) {
  return reset_runtime_locked(&keyframe_name);
}

bool Simulation::reset_runtime_locked(const std::string *keyframe_name) {
  std::unique_lock<std::mutex> lifecycle_lock(lifecycle_mutex_);
  if (scheduler_ == nullptr) {
    LOG_ERROR << "simulation scheduler is not initialized.";
    return false;
  }
  if (runtime_ == nullptr) {
    LOG_ERROR << "simulation must be initialized before reset.";
    return false;
  }
  const SimulationStatus previous_status = scheduler_->status();
  if (previous_status == SimulationStatus::Uninitialized) {
    LOG_ERROR << "simulation scheduler is not initialized.";
    return false;
  }
  if (previous_status == SimulationStatus::Error) {
    LOG_ERROR << "cannot reset a simulation scheduler in the error state.";
    return false;
  }

  const bool restart_after_reset = previous_status == SimulationStatus::Running;
  if (restart_after_reset) {
    if (!scheduler_->stop()) {
      LOG_ERROR << "failed to stop the scheduler before reset.";
      return false;
    }
  }

  bool succeeded = true;
  {
    std::lock_guard<std::mutex> mujoco_lock(mujoco_mutex_);
    if (!runtime_->is_initialized()) {
      LOG_ERROR << "simulation runtime is not initialized.";
      succeeded = false;
    } else if (keyframe_name == nullptr) {
      succeeded = runtime_->reset();
    } else {
      succeeded = runtime_->reset(*keyframe_name);
    }
    if (succeeded) {
      succeeded = component_manager_.reset(runtime_->context());
      if (!succeeded) {
        LOG_ERROR << "failed to reset simulation components.";
      }
    }

    if (succeeded) {
      command_buffer_.clear();
      state_buffer_.clear();
      step_.store(0);
      sequence_ = 0;

      if (!component_manager_.update(runtime_->context())) {
        LOG_ERROR << "failed to update components after reset.";
        succeeded = false;
      }
    }

    if (succeeded) {
      succeeded = write_state_snapshot_locked();
    }
    if (!succeeded) {
      LOG_ERROR << "failed to publish the reset simulation state.";
    }
  }

  if (!succeeded) {
    command_buffer_.clear();
    state_buffer_.clear();
    runtime_failed_.store(true);
    return false;
  }

  {
    std::lock_guard<std::mutex> viewer_lock(viewer_mutex_);
    if (viewer_ != nullptr) {
      if (!viewer_->is_running()) {
        LOG_ERROR << "simulation viewer is no longer running after reset.";
        command_buffer_.clear();
        state_buffer_.clear();
        runtime_failed_.store(true);
        return false;
      }
      if (!viewer_->is_ready()) {
        LOG_ERROR << "simulation viewer is not ready after reset.";
        command_buffer_.clear();
        state_buffer_.clear();
        runtime_failed_.store(true);
        return false;
      }
      if (!viewer_->sync(false)) {
        LOG_ERROR << "failed to synchronize the viewer after reset.";
        command_buffer_.clear();
        state_buffer_.clear();
        runtime_failed_.store(true);
        return false;
      }
      const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(1.0 /
                                        config_.scheduler.viewer_update_rate));
      next_sync_time_ = std::chrono::steady_clock::now() + period;
    }
  }

  if (restart_after_reset) {
    if (!scheduler_->start()) {
      LOG_ERROR << "failed to restart the scheduler after reset.";
      command_buffer_.clear();
      state_buffer_.clear();
      runtime_failed_.store(true);
      return false;
    }
  }

  return true;
}

bool Simulation::write_command(std::string name, const JointCommand &command) {
  JointCommand resolved = command;
  resolved.joint_name = std::move(name);
  return command_buffer_.write_joint_command(resolved.joint_name, resolved);
}

bool Simulation::write_command(const RobotCommand &command) {
  return command_buffer_.write_command(command);
}

bool Simulation::write_command(std::string name,
                               const MobileBaseCommand &command) {
  return command_buffer_.write_mobile_base_command(std::move(name), command);
}

bool Simulation::read_state(std::shared_ptr<const RobotState> &out) const {
  out = state_buffer_.read();
  return out != nullptr;
}

bool Simulation::read_state(RobotState &out) const {
  std::shared_ptr<const RobotState> snapshot;
  if (!read_state(snapshot)) {
    return false;
  }
  out = *snapshot;
  return true;
}

bool Simulation::read_state(std::string name, JointState &out) const {
  return state_buffer_.read_joint_state(std::move(name), out);
}
bool Simulation::read_state(std::string name, ImuState &out) const {
  return state_buffer_.read_imu_state(std::move(name), out);
}
bool Simulation::read_state(std::string name, CameraState &out) const {
  return state_buffer_.read_camera_state(std::move(name), out);
}
bool Simulation::read_state(std::string name, LidarState &out) const {
  return state_buffer_.read_lidar_state(std::move(name), out);
}
bool Simulation::read_state(std::string name, MobileBaseState &out) const {
  return state_buffer_.read_mobile_base_state(std::move(name), out);
}

uint64_t Simulation::step() const { return step_.load(); }

SimulationStatus Simulation::status() const {
  if (runtime_failed_.load()) {
    return SimulationStatus::Error;
  }
  std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
  if (scheduler_ != nullptr) {
    return scheduler_->status();
  }
  std::lock_guard<std::mutex> mujoco_lock(mujoco_mutex_);
  if (runtime_ == nullptr) {
    return SimulationStatus::Uninitialized;
  }
  if (!runtime_->is_initialized()) {
    return SimulationStatus::Uninitialized;
  }
  return SimulationStatus::Stopped;
}

double Simulation::time() const {
  std::lock_guard<std::mutex> mujoco_lock(mujoco_mutex_);
  if (runtime_ == nullptr) {
    return 0.0;
  }
  if (!runtime_->is_initialized()) {
    return 0.0;
  }
  return runtime_->time();
}

bool Simulation::initialize_camera_renderer() {
  if (camera_renderer_ != nullptr) {
    LOG_ERROR << "camera renderer is already initialized.";
    return false;
  }
  try {
    camera_renderer_ =
        std::make_unique<CameraRenderer>(config_.camera_renderer);
  } catch (const std::exception &) {
    LOG_ERROR << "failed to create the camera renderer.";
    return false;
  } catch (...) {
    LOG_ERROR << "failed to create the camera renderer.";
    return false;
  }
  return true;
}

bool Simulation::initialize_scheduler() {
  if (scheduler_ != nullptr) {
    LOG_ERROR << "simulation scheduler is already initialized.";
    return false;
  }
  auto scheduler = std::make_unique<SimulationScheduler>();
  if (!scheduler->initialize()) {
    LOG_ERROR << "failed to initialize the simulation scheduler.";
    return false;
  }
  if (!scheduler->register_task([this] { return scheduler_run_task(); })) {
    LOG_ERROR << "failed to register the simulation scheduler task.";
    return false;
  }
  scheduler_ = std::move(scheduler);
  return true;
}

bool Simulation::initialize_components() {
  std::lock_guard<std::mutex> mujoco_lock(mujoco_mutex_);
  if (runtime_ == nullptr) {
    LOG_ERROR << "simulation runtime is not available.";
    return false;
  }
  if (!runtime_->is_initialized()) {
    LOG_ERROR << "simulation runtime is not initialized.";
    return false;
  }
  if (camera_renderer_ == nullptr) {
    LOG_ERROR << "camera renderer is not initialized.";
    return false;
  }
  if (!component_manager_.init(runtime_->context(), config_.components,
                               *camera_renderer_)) {
    LOG_ERROR << "failed to initialize simulation components.";
    return false;
  }
  if (!component_manager_.update(runtime_->context())) {
    LOG_ERROR << "failed to update initial component state.";
    return false;
  }
  return true;
}

bool Simulation::load_model(const ModelConfig &model_config) {
  auto runtime = std::make_unique<SimulationRuntime>();
  if (!runtime->init(model_config)) {
    LOG_ERROR << "failed to initialize the MuJoCo runtime.";
    return false;
  }
  std::lock_guard<std::mutex> mujoco_lock(mujoco_mutex_);
  runtime_ = std::move(runtime);
  component_manager_.clear();
  next_sync_time_ = std::chrono::steady_clock::now();
  runtime_failed_.store(false);
  step_.store(0);
  sequence_ = 0;
  return true;
}

bool Simulation::scheduler_run_task() {
  if (runtime_failed_.load()) {
    LOG_ERROR << "simulation runtime is in the error state.";
    return false;
  }
  if (!scheduler_write_commands()) {
    LOG_ERROR << "failed to write commands to the simulation.";
    return false;
  }
  if (!scheduler_step_physics()) {
    LOG_ERROR << "failed to advance simulation physics.";
    return false;
  }
  if (!scheduler_update_components()) {
    LOG_ERROR << "failed to update simulation components.";
    return false;
  }
  if (!write_state_snapshot()) {
    LOG_ERROR << "failed to publish the simulation state.";
    return false;
  }
  if (!scheduler_sync_viewer_if_due()) {
    LOG_ERROR << "failed to synchronize the simulation viewer.";
    return false;
  }
  return true;
}

bool Simulation::scheduler_step_physics() {
  std::lock_guard<std::mutex> mujoco_lock(mujoco_mutex_);
  if (runtime_ == nullptr) {
    LOG_ERROR << "simulation runtime is not available.";
    return false;
  }
  if (!runtime_->is_initialized()) {
    LOG_ERROR << "simulation runtime is not initialized.";
    return false;
  }
  if (!runtime_->step()) {
    LOG_ERROR << "MuJoCo physics step failed.";
    return false;
  }
  ++step_;
  return true;
}

bool Simulation::scheduler_write_commands() {
  const RobotCommand snapshot = command_buffer_.read();
  std::lock_guard<std::mutex> mujoco_lock(mujoco_mutex_);
  if (runtime_ == nullptr) {
    LOG_ERROR << "simulation runtime is not available.";
    return false;
  }
  if (!runtime_->is_initialized()) {
    LOG_ERROR << "simulation runtime is not initialized.";
    return false;
  }
  if (!component_manager_.write_command(runtime_->context(), snapshot)) {
    LOG_ERROR << "component manager rejected a command snapshot.";
    return false;
  }
  return true;
}

bool Simulation::scheduler_update_components() {
  std::lock_guard<std::mutex> mujoco_lock(mujoco_mutex_);
  if (runtime_ == nullptr) {
    LOG_ERROR << "simulation runtime is not available.";
    return false;
  }
  if (!runtime_->is_initialized()) {
    LOG_ERROR << "simulation runtime is not initialized.";
    return false;
  }
  if (!component_manager_.update(runtime_->context())) {
    LOG_ERROR << "component manager update failed.";
    return false;
  }
  return true;
}

bool Simulation::update_components_for_step_locked(std::uint64_t, double) {
  if (runtime_ == nullptr) {
    LOG_ERROR << "simulation runtime is not available.";
    return false;
  }
  if (!runtime_->is_initialized()) {
    LOG_ERROR << "simulation runtime is not initialized.";
    return false;
  }
  if (!component_manager_.update(runtime_->context())) {
    LOG_ERROR << "component manager update failed.";
    return false;
  }
  return true;
}

bool Simulation::write_state_snapshot() {
  std::lock_guard<std::mutex> mujoco_lock(mujoco_mutex_);
  if (!write_state_snapshot_locked()) {
    LOG_ERROR << "failed to build the simulation state snapshot.";
    return false;
  }
  return true;
}

bool Simulation::write_state_snapshot_locked() {
  if (runtime_ == nullptr) {
    LOG_ERROR << "simulation runtime is not available.";
    return false;
  }
  if (!runtime_->is_initialized()) {
    LOG_ERROR << "simulation runtime is not initialized.";
    return false;
  }
  return build_state_snapshot_locked(step_.load(), runtime_->time());
}

bool Simulation::build_state_snapshot_locked(std::uint64_t step,
                                             double simulation_time) {
  auto snapshot = std::make_shared<RobotState>();
  if (!build_state_snapshot(*snapshot)) {
    LOG_ERROR << "component manager failed to build the state snapshot.";
    return false;
  }
  snapshot->sequence = ++sequence_;
  snapshot->simulation_time = simulation_time;
  snapshot->timestamp = 0;
  if (simulation_time > 0.0) {
    snapshot->timestamp = static_cast<std::uint64_t>(simulation_time * 1.0e9);
  }
  snapshot->step = step;
  state_buffer_.write(std::move(snapshot));
  return true;
}

bool Simulation::start_viewer() {
  auto viewer =
      std::make_unique<SimulationViewer>(config_.viewer_startup_timeout);
  {
    std::lock_guard<std::mutex> mujoco_lock(mujoco_mutex_);
    if (runtime_ == nullptr) {
      LOG_ERROR << "simulation runtime is not available.";
      return false;
    }
    if (!runtime_->is_initialized()) {
      LOG_ERROR << "simulation runtime is not initialized.";
      return false;
    }
    if (!viewer->start(runtime_->context(), config_.model.model_path)) {
      LOG_ERROR << "viewer startup failed.";
      return false;
    }
  }
  std::lock_guard<std::mutex> viewer_lock(viewer_mutex_);
  viewer_ = std::move(viewer);
  next_sync_time_ = std::chrono::steady_clock::now();
  return true;
}

bool Simulation::scheduler_sync_viewer_if_due() {
  const auto now = std::chrono::steady_clock::now();
  if (now < next_sync_time_) {
    return true;
  }
  std::lock_guard<std::mutex> viewer_lock(viewer_mutex_);
  if (viewer_ == nullptr) {
    return true;
  }
  if (!viewer_->is_running()) {
    LOG_ERROR << "simulation viewer is no longer running.";
    runtime_failed_.store(true);
    return false;
  }
  if (!viewer_->is_ready()) {
    LOG_ERROR << "simulation viewer is not ready.";
    runtime_failed_.store(true);
    return false;
  }
  if (!viewer_->sync(false)) {
    LOG_ERROR << "simulation viewer synchronization failed.";
    runtime_failed_.store(true);
    return false;
  }
  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 /
                                    config_.scheduler.viewer_update_rate));
  next_sync_time_ = now + period;
  return true;
}

bool Simulation::stop_viewer() {
  std::unique_ptr<SimulationViewer> viewer;
  {
    std::lock_guard<std::mutex> viewer_lock(viewer_mutex_);
    viewer = std::move(viewer_);
  }
  if (viewer != nullptr) {
    viewer->stop();
  }
  return true;
}

bool Simulation::build_state_snapshot(RobotState &snapshot) const {
  if (runtime_ == nullptr) {
    LOG_ERROR << "simulation runtime is not available.";
    return false;
  }
  if (!runtime_->is_initialized()) {
    LOG_ERROR << "simulation runtime is not initialized.";
    return false;
  }
  if (!component_manager_.read_state(runtime_->context(), snapshot)) {
    LOG_ERROR << "component manager failed to read the simulation state.";
    return false;
  }
  return true;
}

} // namespace mujoco_simulation
