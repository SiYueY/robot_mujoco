#include "mujoco_simulation/simulation.hpp"

#include <chrono>
#include <future>
#include <memory>
#include <stdexcept>

#include "mujoco_simulation/viewer/viewer.hpp"

namespace mujoco_simulation {
namespace {
constexpr auto kDefaultViewerPeriod = std::chrono::milliseconds(16);
}  // namespace

Simulation::Simulation() = default;

Simulation::~Simulation() { (void)shutdown(); }

Status Simulation::initialize(const SimulationConfig& config) {
  if (model_runtime_ != nullptr && model_runtime_->is_loaded()) {
    return Status::already_exists("Simulation is already initialized.");
  }
  if (config.scheduler.viewer_update_rate <= 0.0) {
    return Status::invalid_argument("Viewer update rate must be greater than zero.");
  }

  config_ = config;

  Status status = load_model(config.model);
  if (!status.ok()) {
    (void)shutdown();
    return status;
  }

  if (config.render_mode == RenderMode::Viewer) {
    status = start_viewer();
    if (!status.ok()) {
      (void)shutdown();
      return status;
    }
  }

  status = initialize_scheduler();
  if (!status.ok()) {
    (void)shutdown();
    return status;
  }

  status = initialize_components();
  if (!status.ok()) {
    (void)shutdown();
    return status;
  }

  return Status::Ok();
}

Status Simulation::shutdown() {
  (void)stop();

  std::unique_ptr<Viewer> viewer_to_stop;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    viewer_to_stop = std::move(viewer_);
    if (scheduler_ != nullptr) {
      (void)scheduler_->shutdown();
      scheduler_.reset();
    }
    camera_renderer_.reset();
    camera_buffer_.reset();
    state_buffer_.reset();
    command_buffer_.reset();
    component_manager_.clear();
    model_runtime_.reset();
    data_ = nullptr;
    model_ = nullptr;
    pending_state_snapshot_.reset();
    runtime_error_.clear();
  }

  if (viewer_to_stop != nullptr) {
    viewer_to_stop->stop();
  }

  return Status::Ok();
}

Status Simulation::start() {
  if (scheduler_ == nullptr) {
    return Status::invalid_state("Simulation scheduler is not initialized.");
  }

  bool should_start_viewer = false;
  if (config_.render_mode == RenderMode::Viewer) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (viewer_ == nullptr) {
      if (model_runtime_ == nullptr || !model_runtime_->is_loaded() || model_ == nullptr ||
          data_ == nullptr) {
        return Status::failed_precondition("MuJoCo model is not loaded.");
      }
      should_start_viewer = true;
    }
  }

  if (should_start_viewer) {
    const Status viewer_status = start_viewer();
    if (!viewer_status.ok()) {
      return viewer_status;
    }
  }

  return scheduler_->start();
}

Status Simulation::stop() {
  if (scheduler_ != nullptr) {
    const Status status = scheduler_->stop();
    if (!status.ok()) {
      return status;
    }
  }
  std::unique_ptr<Viewer> viewer_to_stop;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    viewer_to_stop = std::move(viewer_);
    runtime_error_.clear();
  }
  if (viewer_to_stop != nullptr) {
    viewer_to_stop->stop();
  }
  return Status::Ok();
}

Status Simulation::pause() {
  if (scheduler_ == nullptr) {
    return Status::invalid_state("Simulation scheduler is not initialized.");
  }
  return scheduler_->pause();
}

Status Simulation::resume() {
  if (scheduler_ == nullptr) {
    return Status::invalid_state("Simulation scheduler is not initialized.");
  }
  return scheduler_->resume();
}

Status Simulation::set_realtime_factor(double realtime_factor) {
  if (scheduler_ == nullptr) {
    return Status::invalid_state("Simulation scheduler is not initialized.");
  }

  const Status status = scheduler_->set_realtime_factor(realtime_factor);
  if (!status.ok()) {
    return status;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  config_.scheduler.realtime_factor = realtime_factor;
  return Status::Ok();
}

Status Simulation::request_reset(const ResetOptions& options) {
  if (scheduler_ == nullptr) {
    return Status::invalid_state("Simulation scheduler is not initialized.");
  }
  return scheduler_->request_reset({.options = options});
}

Status Simulation::reset(const ResetOptions& options) {
  if (scheduler_ == nullptr) {
    return Status::invalid_state("Simulation scheduler is not initialized.");
  }

  std::future<Status> completion = scheduler_->request_reset_waitable({.options = options});
  return completion.get();
}

Status Simulation::step(uint32_t steps) {
  if (steps == 0) {
    return Status::invalid_argument("Step count must be greater than zero.");
  }

  if (scheduler_ == nullptr) {
    return Status::invalid_state("Simulation scheduler is not initialized.");
  }

  return scheduler_->step(steps);
}

Status Simulation::reconfigure_component(const ComponentConfig& updated_component) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (model_ == nullptr) {
    return Status::invalid_state("MuJoCo model is not loaded.");
  }
  const Status status = component_manager_.reconfigure_component(*model_, updated_component);
  if (!status.ok()) {
    return status;
  }
  return apply_component_reconfiguration_locked(updated_component);
}

Status Simulation::set_joint_command(const JointCommand& command) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (command_buffer_ == nullptr) {
    return Status::invalid_state("MuJoCo command buffer is not initialized.");
  }
  return command_buffer_->set_joint_command(command.name, command);
}

Result<JointState> Simulation::joint_state(std::string_view joint_name) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_buffer_ == nullptr) {
    return Status::invalid_state("MuJoCo state buffer is not initialized.");
  }
  return state_buffer_->joint_state(joint_name);
}

Result<ImuSample> Simulation::imu_sample(std::string_view imu_name) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_buffer_ == nullptr) {
    return Status::invalid_state("MuJoCo state buffer is not initialized.");
  }
  return state_buffer_->imu_sample(imu_name);
}

Result<std::shared_ptr<const CameraSample>> Simulation::camera_sample(
    std::string_view camera_name) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (camera_buffer_ == nullptr) {
    return Status::invalid_state("MuJoCo camera buffer is not initialized.");
  }
  return camera_buffer_->read(camera_name);
}

Result<LidarSample> Simulation::lidar_sample(std::string_view lidar_name) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_buffer_ == nullptr) {
    return Status::invalid_state("MuJoCo state buffer is not initialized.");
  }
  return state_buffer_->lidar_sample(lidar_name);
}

Status Simulation::set_mobile_base_command(std::string_view name,
                                           const MobileBaseCommand& command) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (command_buffer_ == nullptr) {
    return Status::invalid_state("MuJoCo command buffer is not initialized.");
  }
  return command_buffer_->set_mobile_base_command(name, command);
}

Result<MobileBaseState> Simulation::mobile_base_state(std::string_view name) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_buffer_ == nullptr) {
    return Status::invalid_state("MuJoCo state buffer is not initialized.");
  }
  return state_buffer_->mobile_base_state(name);
}

uint64_t Simulation::step_count() const {
  if (scheduler_ == nullptr) {
    return 0;
  }
  return scheduler_->statistics().physics_steps;
}

SimulationStatus Simulation::status() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!runtime_error_.empty()) {
    return SimulationStatus::Error;
  }
  if (scheduler_ != nullptr) {
    return scheduler_->status();
  }
  return model_runtime_ != nullptr && model_runtime_->is_loaded() ? SimulationStatus::Stopped
                                                                  : SimulationStatus::Uninitialized;
}

double Simulation::simulation_time() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (model_runtime_ == nullptr || !model_runtime_->is_loaded()) {
    return 0.0;
  }
  return model_runtime_->simulation_time();
}

std::shared_ptr<const SimulationStateSnapshot> Simulation::state_snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return state_buffer_ == nullptr ? nullptr : state_buffer_->read();
}

Status Simulation::initialize_scheduler() {
  camera_buffer_ = std::make_unique<CameraBuffer>();
  camera_renderer_ = std::make_unique<CameraRenderer>(config_.camera_renderer);
  command_buffer_ = std::make_unique<CommandBuffer>();
  state_buffer_ = std::make_unique<StateBuffer>();
  state_snapshot_sequence_ = 0;
  state_snapshot_step_count_ = 0;

  auto scheduler = std::make_unique<SimulationScheduler>();
  SchedulerCallbacks callbacks;
  callbacks.timestep_provider = [this]() { return scheduler_timestep_locked(); };
  callbacks.write_commands = [this]() { return scheduler_apply_commands_locked(); };
  callbacks.step_physics = [this]() { return scheduler_step_physics_locked(); };
  callbacks.read_components = [this]() { return scheduler_read_components_locked(true); };
  callbacks.publish_state_snapshot = [this]() {
    std::lock_guard<std::mutex> lock(mutex_);
    return publish_state_snapshot_locked(true);
  };
  callbacks.sync_viewer_if_due = [this]() { return scheduler_sync_viewer_if_due(); };
  callbacks.reset_runtime = [this](const ResetOptions& options) {
    return scheduler_reset_locked(options);
  };

  SchedulerConfig scheduler_config = config_.scheduler;
  scheduler_config.realtime_sync = config_.scheduler.realtime_sync;
  scheduler_config.realtime_factor =
      config_.scheduler.realtime_factor > 0.0 ? config_.scheduler.realtime_factor : 1.0;

  const Status status = scheduler->initialize(scheduler_config, std::move(callbacks));
  if (!status.ok()) {
    return status;
  }

  scheduler_ = std::move(scheduler);
  return Status::Ok();
}

Status Simulation::initialize_components() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (model_ == nullptr) {
    return Status::invalid_state("MuJoCo model is not loaded.");
  }
  Status status = component_manager_.build(*model_, config_.components);
  if (!status.ok()) {
    return status;
  }
  status = read_component_states_locked(false);
  if (!status.ok()) {
    return status;
  }
  return publish_state_snapshot_locked(false);
}

Status Simulation::load_model(const ModelConfig& model_config) {
  auto runtime = std::make_unique<ModelRuntime>();
  const Status status = runtime->load(model_config);
  if (!status.ok()) {
    return status;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  model_runtime_ = std::move(runtime);
  model_ = &model_runtime_->mutable_model();
  data_ = &model_runtime_->mutable_data();
  component_manager_.clear();
  viewer_.reset();
  pending_state_snapshot_.reset();
  next_viewer_sync_time_ = std::chrono::steady_clock::now();
  runtime_error_.clear();
  return Status::Ok();
}

Status Simulation::apply_component_reconfiguration_locked(
    const ComponentConfig& updated_component) {
  if (!replace_component_config(config_.components, updated_component)) {
    return Status::internal(
        "Component '" + std::string(component_config_name(updated_component)) +
        "' was reconfigured but its SimulationConfig entry could not be updated.");
  }
  pending_state_snapshot_.reset();
  const Status read_status = read_component_states_locked(false);
  if (!read_status.ok()) {
    return read_status;
  }
  return publish_state_snapshot_locked(false);
}

Status Simulation::scheduler_step_physics_locked() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (model_runtime_ == nullptr || !model_runtime_->is_loaded()) {
    return Status::failed_precondition("MuJoCo model is not loaded.");
  }
  return model_runtime_->step();
}

Status Simulation::scheduler_apply_commands_locked() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (command_buffer_ == nullptr || model_ == nullptr || data_ == nullptr) {
    return Status::failed_precondition("MuJoCo command application is not initialized.");
  }

  const CommandSnapshot snapshot = command_buffer_->snapshot(
      CommandBuffer::Clock::now(),
      [this](std::string_view name) { return component_manager_.joint_command_mode(name); });
  return component_manager_.write_commands(*model_, *data_, snapshot);
}

Status Simulation::scheduler_read_components_locked(bool increment_step_count) {
  std::lock_guard<std::mutex> lock(mutex_);
  return read_component_states_locked(increment_step_count);
}

Status Simulation::read_component_states_locked(bool increment_step_count) {
  if (model_runtime_ == nullptr || !model_runtime_->is_loaded() || model_ == nullptr ||
      data_ == nullptr) {
    return Status::failed_precondition("MuJoCo state reads are not initialized.");
  }

  PendingStateSnapshot snapshot;
  Status status = component_manager_.read_joint_states(*data_, snapshot.joints);
  if (!status.ok()) {
    return status;
  }
  status = component_manager_.read_mobile_base_states(*data_, snapshot.mobile_bases);
  if (!status.ok()) {
    return status;
  }
  pending_state_snapshot_ = std::move(snapshot);
  return Status::Ok();
}

Status Simulation::publish_state_snapshot_locked(bool increment_step_count) {
  if (state_buffer_ == nullptr || model_runtime_ == nullptr || !model_runtime_->is_loaded() ||
      model_ == nullptr || data_ == nullptr) {
    return Status::failed_precondition("MuJoCo state snapshotting is not initialized.");
  }

  const std::uint64_t snapshot_step_count =
      increment_step_count ? state_snapshot_step_count_ + 1 : state_snapshot_step_count_;
  const double simulation_time = model_runtime_->simulation_time();
  Status status =
      component_manager_.sample_due_sensors(*model_, *data_, simulation_time, snapshot_step_count,
                                            camera_renderer_.get(), camera_buffer_.get());
  if (!status.ok()) {
    return status;
  }

  auto snapshot = std::make_shared<SimulationStateSnapshot>();
  if (pending_state_snapshot_.has_value()) {
    snapshot->joints = std::move(pending_state_snapshot_->joints);
    snapshot->mobile_bases = std::move(pending_state_snapshot_->mobile_bases);
    pending_state_snapshot_.reset();
    status = component_manager_.read_imu_states(snapshot->imus);
    if (!status.ok()) {
      return status;
    }
    status = component_manager_.read_lidar_states(snapshot->lidars);
    if (!status.ok()) {
      return status;
    }
  } else {
    status = component_manager_.read_states(*data_, *snapshot);
    if (!status.ok()) {
      return status;
    }
  }
  snapshot->sequence = ++state_snapshot_sequence_;
  snapshot->simulation_time = simulation_time;
  snapshot->timestamp_ns = snapshot->simulation_time <= 0.0
                               ? 0
                               : static_cast<std::uint64_t>(snapshot->simulation_time * 1.0e9);
  snapshot->step_count = snapshot_step_count;
  state_buffer_->publish(std::move(snapshot));
  state_snapshot_step_count_ = snapshot_step_count;
  return Status::Ok();
}

Status Simulation::scheduler_reset_locked(const ResetOptions& options) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (model_runtime_ == nullptr || !model_runtime_->is_loaded()) {
    return Status::failed_precondition("MuJoCo model is not loaded.");
  }

  Status status = model_runtime_->reset(options);
  if (!status.ok()) {
    return status;
  }

  if (options.reset_components && model_ != nullptr && data_ != nullptr) {
    status = component_manager_.reset_all(*model_, *data_);
    if (!status.ok()) {
      return status;
    }
  }
  if (options.clear_commands && command_buffer_ != nullptr) {
    command_buffer_->clear();
  }
  if (options.clear_state_buffer && state_buffer_ != nullptr) {
    state_buffer_->clear();
  }
  if (options.clear_camera_buffer && camera_buffer_ != nullptr) {
    camera_buffer_->clear();
  }
  pending_state_snapshot_.reset();
  next_viewer_sync_time_ = std::chrono::steady_clock::now();
  if (options.reset_statistics) {
    state_snapshot_step_count_ = 0;
    state_snapshot_sequence_ = 0;
  }

  status = read_component_states_locked(false);
  if (!status.ok()) {
    return status;
  }
  if (state_buffer_ != nullptr) {
    status = publish_state_snapshot_locked(false);
    if (!status.ok()) {
      return status;
    }
  }

  return Status::Ok();
}

double Simulation::scheduler_timestep_locked() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (model_runtime_ == nullptr || !model_runtime_->is_loaded()) {
    return 0.0;
  }
  return model_runtime_->timestep();
}

Status Simulation::start_viewer() {
  auto viewer = std::make_unique<Viewer>();
  const Status status = viewer->start(model_, data_, config_.model.model_path);
  if (!status.ok()) {
    return status;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    viewer_ = std::move(viewer);
    next_viewer_sync_time_ = std::chrono::steady_clock::now();
    runtime_error_.clear();
  }
  return Status::Ok();
}

Status Simulation::scheduler_sync_viewer_if_due() {
  std::unique_ptr<Viewer> viewer_to_stop;
  Status failure_status = Status::Ok();

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (viewer_ == nullptr) {
      return Status::Ok();
    }

    const auto now = std::chrono::steady_clock::now();
    std::chrono::nanoseconds period = kDefaultViewerPeriod;
    if (config_.scheduler.viewer_update_rate > 0.0) {
      period = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(1.0 / config_.scheduler.viewer_update_rate));
    }
    if (now < next_viewer_sync_time_) {
      return Status::Ok();
    }

    if (!viewer_->is_running()) {
      failure_status =
          Status::invalid_state("MuJoCo viewer stopped before the next synchronization cycle.");
      viewer_to_stop = std::move(viewer_);
    } else if (!viewer_->is_ready()) {
      failure_status =
          Status::invalid_state("MuJoCo viewer became unavailable before synchronization.");
      viewer_to_stop = std::move(viewer_);
    } else {
      const Status sync_status = viewer_->sync(false);
      if (!sync_status.ok()) {
        failure_status =
            sync_status.message().empty()
                ? Status::internal(
                      "MuJoCo viewer synchronization failed with an empty error message.")
                : sync_status;
        viewer_to_stop = std::move(viewer_);
      } else {
        next_viewer_sync_time_ = now + period;
        return Status::Ok();
      }
    }

    runtime_error_ = failure_status.message();
  }

  if (viewer_to_stop != nullptr) {
    viewer_to_stop->stop();
  }
  return failure_status;
}

RenderMode parse_render_mode(const std::string& value) {
  if (value == "headless") {
    return RenderMode::Headless;
  }
  if (value == "viewer") {
    return RenderMode::Viewer;
  }
  throw std::invalid_argument("render_mode must be 'headless' or 'viewer'.");
}

const char* to_string(RenderMode mode) {
  switch (mode) {
    case RenderMode::Headless:
      return "headless";
    case RenderMode::Viewer:
      return "viewer";
  }
  return "unknown";
}

}  // namespace mujoco_simulation
