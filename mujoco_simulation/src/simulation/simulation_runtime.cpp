#include "simulation/simulation_impl.hpp"

#include <algorithm>
#include <chrono>
#include <exception>
#include <memory>
#include <mutex>
#include <utility>

#include "buffer/command_channel.hpp"
#include "common/logging.hpp"
#include "render/camera_render_service_impl.hpp"

namespace mujoco_simulation {
namespace {

template <typename Info>
CommandChannelLayout command_valid_ids(const ComponentConfigList &components) {
  ComponentId largest_id = 0;
  bool found = false;
  for (const ComponentConfig &component : components) {
    const Info *info = std::get_if<Info>(&component);
    if (info != nullptr) {
      largest_id = std::max(largest_id, info->id);
      found = true;
    }
  }
  CommandChannelLayout valid_ids(found ? largest_id + 1U : 0U, 0U);
  for (const ComponentConfig &component : components) {
    const Info *info = std::get_if<Info>(&component);
    if (info != nullptr)
      valid_ids[info->id] = 1U;
  }
  return valid_ids;
}

} // namespace

bool Simulation::Impl::initialize_camera_renderer() {
  if (camera_render_service_ != nullptr) {
    LOG_ERROR << "camera renderer is already initialized.";
    return false;
  }
  try {
    CameraRendererConfig renderer_config = config_.camera_renderer;
    renderer_config.max_camera_id = config_.max_component_id;
    camera_render_service_ =
        std::make_unique<CameraRenderServiceImpl>(std::move(renderer_config));
  } catch (const std::exception &) {
    LOG_ERROR << "failed to create the camera renderer.";
    return false;
  } catch (...) {
    LOG_ERROR << "failed to create the camera renderer.";
    return false;
  }
  return true;
}

bool Simulation::Impl::initialize_scheduler() {
  if (scheduler_ != nullptr) {
    LOG_ERROR << "simulation scheduler is already initialized.";
    return false;
  }
  auto scheduler = std::make_unique<SimulationScheduler>();
  if (!scheduler->initialize(
          std::chrono::duration<double>(config_.scheduler.physics_period))) {
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

bool Simulation::Impl::initialize_components() {
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
    if (camera_render_service_ == nullptr) {
      LOG_ERROR << "camera renderer is not initialized.";
      return false;
    }
    if (!component_manager_.init(runtime_->context(), config_.components,
                                 config_.max_component_id,
                                 *camera_render_service_)) {
      LOG_ERROR << "failed to initialize simulation components.";
      return false;
    }
    if (component_manager_.has_cameras() &&
        !camera_render_service_->initialize(config_,
                                            runtime_->context().model)) {
      LOG_ERROR << "failed to initialize the camera render worker.";
      return false;
    }
    if (!component_manager_.update(runtime_->context())) {
      LOG_ERROR << "failed to update initial component state.";
      return false;
    }
  }
  if (!component_manager_.wait_for_camera_results()) {
    LOG_ERROR << "failed to obtain the initial camera frame.";
    return false;
  }
  if (!command_buffer_.configure_channel<JointCommand>(
          command_valid_ids<JointInfo>(config_.components)) ||
      !command_buffer_.configure_channel<MobileBaseCommand>(
          command_valid_ids<MobileBaseInfo>(config_.components)) ||
      !command_buffer_.finalize_configuration()) {
    LOG_ERROR << "failed to initialize the command bus.";
    return false;
  }
  return true;
}

bool Simulation::Impl::load_model(const ModelConfig &model_config) {
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

bool Simulation::Impl::scheduler_run_task() {
  if (runtime_failed_.load()) {
    LOG_ERROR << "simulation runtime is in the error state.";
    return false;
  }
  if (!has_applied_command_ ||
      command_buffer_.read_if_updated(applied_command_sequence_,
                                      applied_command_)) {
    applied_command_sequence_ = applied_command_.sequence;
    has_applied_command_ = true;
  }
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
    if (!component_manager_.apply_commands(runtime_->context(),
                                           applied_command_)) {
      LOG_ERROR << "component manager rejected a command snapshot.";
      return false;
    }
    if (!runtime_->step()) {
      LOG_ERROR << "MuJoCo physics step failed.";
      return false;
    }
    ++step_;
    if (!component_manager_.update(runtime_->context())) {
      LOG_ERROR << "component manager update failed.";
      return false;
    }
    if (!write_state_snapshot_locked()) {
      LOG_ERROR << "failed to publish the simulation state.";
      return false;
    }
  }
  if (!scheduler_submit_viewer_sync_if_due()) {
    LOG_ERROR
        << "failed to submit a simulation viewer synchronization request.";
    return false;
  }
  return true;
}

bool Simulation::Impl::step(std::size_t count) {
  std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
  if (scheduler_ == nullptr) {
    LOG_ERROR << "simulation scheduler is not initialized.";
    return false;
  }
  if (runtime_failed_.load()) {
    LOG_ERROR
        << "simulation is in an error state; reset or shutdown is required "
           "before stepping again.";
    return false;
  }
  return scheduler_->step(count);
}

bool Simulation::Impl::write_state_snapshot_locked() {
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

bool Simulation::Impl::build_state_snapshot_locked(std::uint64_t step,
                                                   double simulation_time) {
  auto snapshot = std::make_shared<RobotState>();
  if (!build_state_snapshot(*snapshot)) {
    LOG_ERROR << "component manager failed to build the state snapshot.";
    return false;
  }
  snapshot->sequence = ++sequence_;
  snapshot->simulation_time = simulation_time;
  snapshot->timestamp =
      simulation_time > 0.0
          ? static_cast<std::uint64_t>(simulation_time * 1.0e9)
          : 0;
  snapshot->step = step;
  state_buffer_.write(std::move(snapshot));
  return true;
}

bool Simulation::Impl::build_state_snapshot(RobotState &snapshot) const {
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
