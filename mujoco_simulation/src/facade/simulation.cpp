#include "mujoco_simulation/simulation.hpp"

// Public facade implementation; all mutable runtime state is in Impl.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <exception>
#include <memory>
#include <mutex>
#include <optional>
#include <utility>
#include <vector>

#include "buffer/command_buffer.hpp"
#include "buffer/state_buffer.hpp"
#include "common/logging.hpp"
#include "common/macro.hpp"
#include "component/camera/camera_render_service.hpp"
#include "component/component_manager.hpp"
#include "config/simulation_config_parser.hpp"
#include "render/camera_render_service_impl.hpp"
#include "runtime/simulation_runtime.hpp"
#include "runtime/simulation_scheduler.hpp"
#include "viewer/simulation_viewer.hpp"

namespace mujoco_simulation {

class Simulation::Impl {
public:
  // This self-reference lets the established implementation use the same
  // local aliases as facade methods while all state and helpers live in Impl.
  Impl *impl_{this};
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

  bool initialize_camera_renderer();
  bool initialize_scheduler();
  bool initialize_components();
  bool load_model(const ModelConfig &model_config);
  bool write_state_snapshot_locked();
  bool build_state_snapshot_locked(std::uint64_t step, double simulation_time);
  bool scheduler_run_task();
  bool scheduler_submit_viewer_sync_if_due();
  bool start_viewer();
  bool stop_viewer();
  bool build_state_snapshot(RobotState &snapshot) const;
  bool reset_runtime_locked(const std::string *keyframe_name);
};

// Keep the implementation migration local to this translation unit.  The
// facade header contains only Impl, while the established implementation can
// continue to use its descriptive member names.
#define config_ impl_->config_
#define camera_render_service_ impl_->camera_render_service_
#define command_buffer_ impl_->command_buffer_
#define state_buffer_ impl_->state_buffer_
#define runtime_ impl_->runtime_
#define lifecycle_mutex_ impl_->lifecycle_mutex_
#define mujoco_mutex_ impl_->mujoco_mutex_
#define scheduler_ impl_->scheduler_
#define component_manager_ impl_->component_manager_
#define viewer_ impl_->viewer_
#define viewer_mutex_ impl_->viewer_mutex_
#define next_sync_time_ impl_->next_sync_time_
#define runtime_failed_ impl_->runtime_failed_
#define step_ impl_->step_
#define sequence_ impl_->sequence_
#define applied_command_ impl_->applied_command_
#define applied_command_sequence_ impl_->applied_command_sequence_
#define has_applied_command_ impl_->has_applied_command_

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

Simulation::Simulation() : impl_(std::make_unique<Impl>()) {}
Simulation::~Simulation() { UNUSED(shutdown()); }

bool Simulation::initialize(const std::string &config_path) {
  SimulationConfig config;
  ConfigError error;
  SimulationConfigParser parser;
  if (!parser.load_file(config_path, config, &error)) {
    LOG_ERROR << "failed to initialize simulation from '" << config_path
              << "': " << error.message;
    return false;
  }
  return initialize(config);
}

bool Simulation::initialize(const SimulationConfig &config) {
  std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
  if (runtime_ != nullptr) {
    LOG_ERROR << "simulation is already initialized.";
    return false;
  }
  ConfigError config_error;
  if (!SimulationConfigValidator::validate(config, &config_error)) {
    LOG_ERROR << "invalid simulation configuration: " << config_error.message;
    return false;
  }
  config_ = config;
  has_applied_command_ = false;
  applied_command_ = {};
  command_buffer_.shutdown();
  state_buffer_.clear();
  const auto cleanup = [this] {
    if (scheduler_ != nullptr) {
      UNUSED(scheduler_->shutdown());
      scheduler_.reset();
    }
    if (camera_render_service_ != nullptr) {
      UNUSED(camera_render_service_->shutdown());
    }
    UNUSED(impl_->stop_viewer());
    std::lock_guard<std::mutex> mujoco_lock(mujoco_mutex_);
    camera_render_service_.reset();
    component_manager_.clear();
    runtime_.reset();
    runtime_failed_.store(false);
    step_.store(0);
    sequence_ = 0;
    command_buffer_.shutdown();
    has_applied_command_ = false;
    applied_command_ = {};
  };
  if (!impl_->load_model(config.model)) {
    LOG_ERROR << "failed to load the simulation model.";
    cleanup();
    return false;
  }
  bool physics_period_applied = false;
  {
    std::lock_guard<std::mutex> mujoco_lock(mujoco_mutex_);
    physics_period_applied =
        runtime_ != nullptr &&
        runtime_->set_timestep(config_.scheduler.physics_period);
  }
  if (!physics_period_applied) {
    LOG_ERROR << "failed to apply the configured physics period.";
    cleanup();
    return false;
  }
  if (!impl_->initialize_camera_renderer()) {
    LOG_ERROR << "failed to initialize the camera renderer.";
    cleanup();
    return false;
  }
  if (!impl_->initialize_components()) {
    LOG_ERROR << "failed to initialize simulation components.";
    cleanup();
    return false;
  }
  bool initial_state_published = false;
  {
    std::lock_guard<std::mutex> mujoco_lock(mujoco_mutex_);
    initial_state_published = impl_->write_state_snapshot_locked();
  }
  if (!initial_state_published) {
    LOG_ERROR << "failed to publish the initial simulation state.";
    cleanup();
    return false;
  }
  if (!impl_->initialize_scheduler()) {
    LOG_ERROR << "failed to initialize the simulation scheduler.";
    cleanup();
    return false;
  }
  if (config_.viewer_enabled && !impl_->start_viewer()) {
    LOG_ERROR << "failed to start the simulation viewer.";
    cleanup();
    runtime_failed_.store(true);
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
  if (camera_render_service_ != nullptr) {
    UNUSED(camera_render_service_->shutdown());
  }
  UNUSED(impl_->stop_viewer());
  {
    std::lock_guard<std::mutex> mujoco_lock(mujoco_mutex_);
    camera_render_service_.reset();
    component_manager_.clear();
    runtime_.reset();
    runtime_failed_.store(false);
    step_.store(0);
    sequence_ = 0;
  }
  command_buffer_.shutdown();
  state_buffer_.clear();
  has_applied_command_ = false;
  applied_command_ = {};
  return true;
}

bool Simulation::start() {
  std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
  if (scheduler_ == nullptr) {
    LOG_ERROR << "simulation scheduler is not initialized.";
    return false;
  }
  if (runtime_failed_.load()) {
    LOG_ERROR
        << "simulation is in an error state; reset or shutdown is required "
           "before starting again.";
    return false;
  }
  bool needs_viewer = false;
  bool viewer_started_here = false;
  bool camera_started_here = false;
  const auto rollback_started_resources = [this, &viewer_started_here,
                                           &camera_started_here] {
    if (camera_started_here && camera_render_service_ != nullptr) {
      UNUSED(camera_render_service_->reset());
    }
    if (viewer_started_here) {
      UNUSED(impl_->stop_viewer());
    }
  };
  if (config_.viewer_enabled) {
    std::lock_guard<std::mutex> viewer_lock(viewer_mutex_);
    needs_viewer = viewer_ == nullptr;
  }
  if (needs_viewer) {
    if (impl_->start_viewer()) {
      viewer_started_here = true;
    } else {
      LOG_ERROR << "failed to restart the simulation viewer.";
      runtime_failed_.store(true);
      return false;
    }
  }
  bool camera_start_failed = false;
  if (component_manager_.has_cameras()) {
    std::lock_guard<std::mutex> mujoco_lock(mujoco_mutex_);
    if (runtime_ == nullptr) {
      LOG_ERROR << "simulation runtime is not available.";
      camera_start_failed = true;
    }
    if (!camera_start_failed && !runtime_->is_initialized()) {
      LOG_ERROR << "simulation runtime is not initialized.";
      camera_start_failed = true;
    }
    if (!camera_start_failed && camera_render_service_ == nullptr) {
      LOG_ERROR << "camera renderer is not available.";
      camera_start_failed = true;
    }
    if (!camera_start_failed && !camera_render_service_->initialize(
                                    config_, runtime_->context().model)) {
      LOG_ERROR << "failed to restart the camera render worker.";
      camera_start_failed = true;
    } else if (!camera_start_failed) {
      camera_started_here = true;
    }
  }
  if (camera_start_failed) {
    rollback_started_resources();
    return false;
  }
  if (!scheduler_->start()) {
    LOG_ERROR << "failed to start the simulation scheduler.";
    rollback_started_resources();
    return false;
  }
  return true;
}

bool Simulation::stop() {
  std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
  const bool was_error = runtime_failed_.load() ||
                         (scheduler_ != nullptr &&
                          scheduler_->status() == SimulationStatus::Error);
  if (scheduler_ != nullptr) {
    if (!scheduler_->stop()) {
      LOG_ERROR << "failed to stop the simulation scheduler.";
      return false;
    }
  }
  // Stopping joins the worker, but it is not a runtime recovery operation.
  // Preserve an existing failure so only reset() or shutdown() can make this
  // Simulation startable again.
  runtime_failed_.store(was_error);
  if (camera_render_service_ != nullptr) {
    UNUSED(camera_render_service_->reset());
  }
  {
    std::lock_guard<std::mutex> mujoco_lock(mujoco_mutex_);
    component_manager_.clear_camera_states();
  }
  command_buffer_.clear();
  UNUSED(impl_->stop_viewer());
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

bool Simulation::reset() { return impl_->reset_runtime_locked(nullptr); }

bool Simulation::reset(std::string keyframe_name) {
  return impl_->reset_runtime_locked(&keyframe_name);
}

bool Simulation::Impl::reset_runtime_locked(const std::string *keyframe_name) {
  std::unique_lock<std::mutex> lifecycle_lock(lifecycle_mutex_);
  if (scheduler_ == nullptr) {
    LOG_ERROR << "simulation scheduler is not initialized.";
    return false;
  }
  if (runtime_ == nullptr) {
    LOG_ERROR << "simulation must be initialized before reset.";
    return false;
  }
  const auto fail_reset = [this] {
    // A failed reset never destroys the last observable state. Commands are
    // discarded because their targets may no longer match the runtime.
    command_buffer_.clear();
    runtime_failed_.store(true);
    return false;
  };
  const SimulationStatus previous_status = scheduler_->status();
  if (previous_status == SimulationStatus::Uninitialized) {
    LOG_ERROR << "simulation scheduler is not initialized.";
    return false;
  }
  const bool restart_running = previous_status == SimulationStatus::Running;
  const bool restart_paused = previous_status == SimulationStatus::Paused;
  const bool recover_from_error = previous_status == SimulationStatus::Error;
  if (restart_running || restart_paused || recover_from_error) {
    if (!scheduler_->stop()) {
      LOG_ERROR << "failed to stop the scheduler before reset.";
      return fail_reset();
    }
  }

  if (camera_render_service_ != nullptr) {
    if (!camera_render_service_->reset()) {
      LOG_ERROR << "failed to stop the camera render worker before reset.";
      return fail_reset();
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
      step_.store(0);
      sequence_ = 0;

      if (component_manager_.has_cameras()) {
        if (camera_render_service_ == nullptr) {
          LOG_ERROR << "camera renderer is not available after reset.";
          succeeded = false;
        } else if (!camera_render_service_->initialize(
                       config_, runtime_->context().model)) {
          LOG_ERROR << "failed to initialize camera rendering after reset.";
          succeeded = false;
        }
      }
    }

    if (succeeded) {
      if (!component_manager_.update(runtime_->context())) {
        LOG_ERROR << "failed to update components after reset.";
        succeeded = false;
      }
    }
  }

  if (succeeded && !component_manager_.wait_for_camera_results()) {
    LOG_ERROR << "failed to obtain the reset camera frame.";
    succeeded = false;
  }

  if (succeeded) {
    std::lock_guard<std::mutex> mujoco_lock(mujoco_mutex_);
    succeeded = impl_->write_state_snapshot_locked();
  }
  if (!succeeded) {
    LOG_ERROR << "failed to publish the reset simulation state.";
  }

  if (!succeeded) {
    // Keep the last successfully published state readable for diagnosis.  A
    // shutdown, rather than a failed reset, is the lifecycle boundary that
    // invalidates externally visible state.
    return fail_reset();
  }

  runtime_failed_.store(false);

  std::shared_ptr<SimulationViewer> viewer;
  {
    std::lock_guard<std::mutex> viewer_lock(viewer_mutex_);
    viewer = viewer_;
  }
  if (viewer != nullptr) {
    SimulationViewer::ViewerSnapshot snapshot;
    {
      std::lock_guard<std::mutex> mujoco_lock(mujoco_mutex_);
      if (runtime_ != nullptr && runtime_->is_initialized() &&
          !viewer->capture_snapshot(runtime_->context(), snapshot)) {
        LOG_WARNING << "failed to capture the reset viewer snapshot.";
      }
    }
    if (snapshot && !viewer->submit(std::move(snapshot))) {
      LOG_WARNING << "failed to enqueue the reset viewer snapshot.";
    }
  }
  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(config_.scheduler.viewer_period));
  next_sync_time_ = std::chrono::steady_clock::now() + period;

  if (restart_running) {
    if (!scheduler_->start()) {
      LOG_ERROR << "failed to restart the scheduler after reset.";
      return fail_reset();
    }
  } else if (restart_paused) {
    if (!scheduler_->start_paused()) {
      LOG_ERROR << "failed to restore the scheduler paused state after reset.";
      return fail_reset();
    }
  }

  return true;
}

bool Simulation::write_command(JointId id, const JointCommand &command) {
  if (!command_buffer_.write<JointCommand>(id, command)) {
    LOG_ERROR << "joint command was rejected for component id " << id << ".";
    return false;
  }
  return true;
}

bool Simulation::write_command(MobileBaseId id,
                               const MobileBaseCommand &command) {
  if (!command_buffer_.write<MobileBaseCommand>(id, command)) {
    LOG_ERROR << "mobile base command was rejected for component id " << id
              << ".";
    return false;
  }
  return true;
}

bool Simulation::write_commands(const JointCommandBatch &commands) {
  if (!command_buffer_.write(CommandBatch<JointCommand>{commands})) {
    LOG_ERROR << "joint command batch was rejected.";
    return false;
  }
  return true;
}

bool Simulation::write_commands(const MobileBaseCommandBatch &commands) {
  if (!command_buffer_.write(CommandBatch<MobileBaseCommand>{commands})) {
    LOG_ERROR << "mobile base command batch was rejected.";
    return false;
  }
  return true;
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

bool Simulation::read_state(JointId id, JointState &out) const {
  return state_buffer_.read_joint_state(id, out);
}
bool Simulation::read_state(ImuId id, ImuState &out) const {
  return state_buffer_.read_imu_state(id, out);
}
bool Simulation::read_state(CameraId id, CameraState &out) const {
  return state_buffer_.read_camera_state(id, out);
}
bool Simulation::read_state(LidarId id, LidarState &out) const {
  return state_buffer_.read_lidar_state(id, out);
}
bool Simulation::read_state(MobileBaseId id, MobileBaseState &out) const {
  return state_buffer_.read_mobile_base_state(id, out);
}
bool Simulation::read_state(JointStates &out) const {
  const auto snapshot = state_buffer_.read();
  if (snapshot == nullptr)
    return false;
  out = snapshot->joints;
  return out != nullptr;
}
bool Simulation::read_state(ImuStates &out) const {
  const auto snapshot = state_buffer_.read();
  if (snapshot == nullptr)
    return false;
  out = snapshot->imus;
  return out != nullptr;
}
bool Simulation::read_state(CameraStates &out) const {
  const auto snapshot = state_buffer_.read();
  if (snapshot == nullptr)
    return false;
  out = snapshot->cameras;
  return out != nullptr;
}
bool Simulation::read_state(LidarStates &out) const {
  const auto snapshot = state_buffer_.read();
  if (snapshot == nullptr)
    return false;
  out = snapshot->lidars;
  return out != nullptr;
}
bool Simulation::read_state(MobileBaseStates &out) const {
  const auto snapshot = state_buffer_.read();
  if (snapshot == nullptr)
    return false;
  out = snapshot->mobile_bases;
  return out != nullptr;
}

bool Simulation::step(std::size_t count) {
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

uint64_t Simulation::step_count() const { return step_.load(); }

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
    if (component_manager_.has_cameras()) {
      if (!camera_render_service_->initialize(config_,
                                              runtime_->context().model)) {
        LOG_ERROR << "failed to initialize the camera render worker.";
        return false;
      }
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
  snapshot->timestamp = 0;
  if (simulation_time > 0.0) {
    snapshot->timestamp = static_cast<std::uint64_t>(simulation_time * 1.0e9);
  }
  snapshot->step = step;
  state_buffer_.write(std::move(snapshot));
  return true;
}

bool Simulation::Impl::start_viewer() {
  auto viewer =
      std::make_shared<SimulationViewer>(config_.viewer_startup_timeout);
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
    if (!viewer->prepare(runtime_->context())) {
      LOG_ERROR << "viewer preparation failed.";
      return false;
    }
  }
  if (!viewer->start(config_.model.model_path)) {
    LOG_ERROR << "viewer startup failed.";
    return false;
  }
  std::lock_guard<std::mutex> viewer_lock(viewer_mutex_);
  viewer_ = std::move(viewer);
  next_sync_time_ = std::chrono::steady_clock::now();
  return true;
}

bool Simulation::Impl::scheduler_submit_viewer_sync_if_due() {
  const auto now = std::chrono::steady_clock::now();
  if (now < next_sync_time_) {
    return true;
  }
  std::shared_ptr<SimulationViewer> viewer;
  {
    std::lock_guard<std::mutex> viewer_lock(viewer_mutex_);
    viewer = viewer_;
  }
  if (viewer != nullptr) {
    SimulationViewer::ViewerSnapshot snapshot;
    {
      std::lock_guard<std::mutex> mujoco_lock(mujoco_mutex_);
      if (runtime_ != nullptr && runtime_->is_initialized() &&
          !viewer->capture_snapshot(runtime_->context(), snapshot)) {
        LOG_WARNING << "failed to capture a viewer synchronization snapshot.";
      }
    }
    if (snapshot && !viewer->submit(std::move(snapshot))) {
      LOG_WARNING << "failed to enqueue a viewer synchronization request.";
    }
  }
  const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(config_.scheduler.viewer_period));
  next_sync_time_ = now + period;
  return true;
}

bool Simulation::Impl::stop_viewer() {
  std::shared_ptr<SimulationViewer> viewer;
  {
    std::lock_guard<std::mutex> viewer_lock(viewer_mutex_);
    viewer = std::move(viewer_);
  }
  if (viewer != nullptr) {
    viewer->stop();
  }
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

#undef config_
#undef camera_render_service_
#undef command_buffer_
#undef state_buffer_
#undef runtime_
#undef lifecycle_mutex_
#undef mujoco_mutex_
#undef scheduler_
#undef component_manager_
#undef viewer_
#undef viewer_mutex_
#undef next_sync_time_
#undef runtime_failed_
#undef step_
#undef sequence_
#undef applied_command_
#undef applied_command_sequence_
#undef has_applied_command_
