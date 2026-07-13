#include "mujoco_simulation/simulation.hpp"

#include <chrono>
#include <future>
#include <memory>
#include <stdexcept>

#include "mujoco_simulation/runtime/model_runtime.hpp"
#include "mujoco_simulation/runtime/simulation_scheduler.hpp"
#include "mujoco_simulation/viewer/mujoco_viewer.hpp"

namespace mujoco_simulation {
namespace {
constexpr auto kDefaultViewerPeriod = std::chrono::milliseconds(16);
}  // namespace

Simulation::Simulation() = default;

Simulation::~Simulation() { (void)shutdown(); }

ResultCode Simulation::initialize(const SimulationConfig& config) {
  if (model_runtime_ != nullptr && model_runtime_->is_loaded()) {
    return ResultCode::AlreadyExists;
  }
  if (config.scheduler.viewer_update_rate <= 0.0) {
    return ResultCode::InvalidArgument;
  }

  config_ = config;

  ResultCode status = load_model(config.model);
  if (status != ResultCode::Ok) {
    (void)shutdown();
    return status;
  }

  if (config.render_mode == RenderMode::Viewer) {
    status = start_viewer();
    if (status != ResultCode::Ok) {
      (void)shutdown();
      return status;
    }
  }

  status = initialize_scheduler();
  if (status != ResultCode::Ok) {
    (void)shutdown();
    return status;
  }

  status = initialize_components();
  if (status != ResultCode::Ok) {
    (void)shutdown();
    return status;
  }

  return ResultCode::Ok;
}

ResultCode Simulation::shutdown() {
  (void)stop();

  std::unique_ptr<MuJoCoViewer> viewer_to_stop;
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
    runtime_error_ = ResultCode::Ok;
  }

  if (viewer_to_stop != nullptr) {
    viewer_to_stop->stop();
  }

  return ResultCode::Ok;
}

ResultCode Simulation::start() {
  if (scheduler_ == nullptr) {
    return ResultCode::InvalidState;
  }

  bool should_start_viewer = false;
  if (config_.render_mode == RenderMode::Viewer) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (viewer_ == nullptr) {
      if (model_runtime_ == nullptr || !model_runtime_->is_loaded() || model_ == nullptr ||
          data_ == nullptr) {
        return ResultCode::FailedPrecondition;
      }
      should_start_viewer = true;
    }
  }

  if (should_start_viewer) {
    const ResultCode viewer_status = start_viewer();
    if (viewer_status != ResultCode::Ok) {
      return viewer_status;
    }
  }

  return scheduler_->start();
}

ResultCode Simulation::stop() {
  if (scheduler_ != nullptr) {
    const ResultCode status = scheduler_->stop();
    if (status != ResultCode::Ok) {
      return status;
    }
  }
  std::unique_ptr<MuJoCoViewer> viewer_to_stop;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    viewer_to_stop = std::move(viewer_);
    runtime_error_ = ResultCode::Ok;
  }
  if (viewer_to_stop != nullptr) {
    viewer_to_stop->stop();
  }
  return ResultCode::Ok;
}

ResultCode Simulation::pause() {
  if (scheduler_ == nullptr) {
    return ResultCode::InvalidState;
  }
  return scheduler_->pause();
}

ResultCode Simulation::resume() {
  if (scheduler_ == nullptr) {
    return ResultCode::InvalidState;
  }
  return scheduler_->resume();
}

ResultCode Simulation::set_realtime_factor(double realtime_factor) {
  if (scheduler_ == nullptr) {
    return ResultCode::InvalidState;
  }

  const ResultCode status = scheduler_->set_realtime_factor(realtime_factor);
  if (status != ResultCode::Ok) {
    return status;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  config_.scheduler.realtime_factor = realtime_factor;
  return ResultCode::Ok;
}

ResultCode Simulation::request_reset(const ResetOptions& options) {
  if (scheduler_ == nullptr) {
    return ResultCode::InvalidState;
  }
  return scheduler_->request_reset({.options = options});
}

ResultCode Simulation::reset(const ResetOptions& options) {
  if (scheduler_ == nullptr) {
    return ResultCode::InvalidState;
  }

  std::future<ResultCode> completion = scheduler_->request_reset_waitable({.options = options});
  return completion.get();
}

ResultCode Simulation::step(uint32_t steps) {
  if (steps == 0) {
    return ResultCode::InvalidArgument;
  }

  if (scheduler_ == nullptr) {
    return ResultCode::InvalidState;
  }

  return scheduler_->step(steps);
}

ResultCode Simulation::reconfigure_component(const ComponentConfig& updated_component) {
  std::shared_ptr<const StateSnapshot> published_snapshot;
  SnapshotObserver snapshot_observer;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (model_ == nullptr) {
      return ResultCode::InvalidState;
    }
    const ResultCode status = component_manager_.reconfigure_component(*model_, updated_component);
    if (status != ResultCode::Ok) {
      return status;
    }
    const ResultCode apply_status =
        apply_component_reconfiguration_locked(updated_component, &published_snapshot);
    if (apply_status != ResultCode::Ok) {
      return apply_status;
    }
    snapshot_observer = snapshot_observer_;
  }
  if (snapshot_observer != nullptr && published_snapshot != nullptr) {
    snapshot_observer(std::move(published_snapshot));
  }
  return ResultCode::Ok;
}

ResultCode Simulation::set_joint_command(const JointCommand& command) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (command_buffer_ == nullptr) {
    return ResultCode::InvalidState;
  }
  return command_buffer_->set_joint_command(command.name, command);
}

bool Simulation::joint_state(std::string joint_name, JointState* out) const {
  if (out == nullptr) {
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_buffer_ == nullptr) {
    return false;
  }
  return state_buffer_->joint_state(joint_name, out);
}

bool Simulation::imu_state(std::string imu_name, ImuState* out) const {
  if (out == nullptr) {
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_buffer_ == nullptr) {
    return false;
  }
  return state_buffer_->imu_state(imu_name, out);
}

bool Simulation::camera_state(std::string camera_name, CameraState* out) const {
  if (out == nullptr) {
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (camera_buffer_ == nullptr) {
    return false;
  }
  return camera_buffer_->read(camera_name, out);
}

bool Simulation::lidar_state(std::string lidar_name, LidarState* out) const {
  if (out == nullptr) {
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_buffer_ == nullptr) {
    return false;
  }
  return state_buffer_->lidar_state(lidar_name, out);
}

ResultCode Simulation::set_mobile_base_command(std::string name, const MobileBaseCommand& command) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (command_buffer_ == nullptr) {
    return ResultCode::InvalidState;
  }
  return command_buffer_->set_mobile_base_command(name, command);
}

bool Simulation::mobile_base_state(std::string name, MobileBaseState* out) const {
  if (out == nullptr) {
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_buffer_ == nullptr) {
    return false;
  }
  return state_buffer_->mobile_base_state(name, out);
}

uint64_t Simulation::step_count() const {
  if (scheduler_ == nullptr) {
    return 0;
  }
  return scheduler_->statistics().physics_steps;
}

SimulationStatus Simulation::status() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (runtime_error_ != ResultCode::Ok) {
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

std::shared_ptr<const StateSnapshot> Simulation::state_snapshot() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return state_buffer_ == nullptr ? nullptr : state_buffer_->read();
}

void Simulation::set_snapshot_observer(SnapshotObserver observer) {
  std::lock_guard<std::mutex> lock(mutex_);
  snapshot_observer_ = std::move(observer);
}

ResultCode Simulation::initialize_scheduler() {
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
  callbacks.publish_state_snapshot = [this]() { return publish_state_snapshot(true); };
  callbacks.sync_viewer_if_due = [this]() { return scheduler_sync_viewer_if_due(); };
  callbacks.reset_runtime = [this](const ResetOptions& options) {
    return scheduler_reset_locked(options);
  };

  SchedulerConfig scheduler_config = config_.scheduler;
  scheduler_config.realtime_sync = config_.scheduler.realtime_sync;
  scheduler_config.realtime_factor =
      config_.scheduler.realtime_factor > 0.0 ? config_.scheduler.realtime_factor : 1.0;

  const ResultCode status = scheduler->initialize(scheduler_config, std::move(callbacks));
  if (status != ResultCode::Ok) {
    return status;
  }

  scheduler_ = std::move(scheduler);
  return ResultCode::Ok;
}

ResultCode Simulation::initialize_components() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (model_ == nullptr) {
      return ResultCode::InvalidState;
    }
    ResultCode status = component_manager_.build(*model_, config_.components);
    if (status != ResultCode::Ok) {
      return status;
    }
    status = read_component_states_locked(false);
    if (status != ResultCode::Ok) {
      return status;
    }
  }
  return publish_state_snapshot(false);
}

ResultCode Simulation::load_model(const ModelConfig& model_config) {
  auto runtime = std::make_unique<ModelRuntime>();
  const ResultCode status = runtime->load(model_config);
  if (status != ResultCode::Ok) {
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
  runtime_error_ = ResultCode::Ok;
  return ResultCode::Ok;
}

ResultCode Simulation::apply_component_reconfiguration_locked(
    const ComponentConfig& updated_component,
    std::shared_ptr<const StateSnapshot>* published_snapshot) {
  if (!replace_component_config(config_.components, updated_component)) {
    return ResultCode::Internal;
  }
  pending_state_snapshot_.reset();
  const ResultCode read_status = read_component_states_locked(false);
  if (read_status != ResultCode::Ok) {
    return read_status;
  }
  return publish_state_snapshot_locked(false, published_snapshot);
}

ResultCode Simulation::scheduler_step_physics_locked() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (model_runtime_ == nullptr || !model_runtime_->is_loaded()) {
    return ResultCode::FailedPrecondition;
  }
  return model_runtime_->step();
}

ResultCode Simulation::scheduler_apply_commands_locked() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (command_buffer_ == nullptr || model_ == nullptr || data_ == nullptr) {
    return ResultCode::FailedPrecondition;
  }

  const CommandSnapshot snapshot = command_buffer_->snapshot(
      CommandBuffer::Clock::now(),
      [this](std::string name) { return component_manager_.joint_command_mode(name); });
  return component_manager_.write_commands(*model_, *data_, snapshot);
}

ResultCode Simulation::scheduler_read_components_locked(bool increment_step_count) {
  std::lock_guard<std::mutex> lock(mutex_);
  return read_component_states_locked(increment_step_count);
}

ResultCode Simulation::read_component_states_locked(bool increment_step_count) {
  if (model_runtime_ == nullptr || !model_runtime_->is_loaded() || model_ == nullptr ||
      data_ == nullptr) {
    return ResultCode::FailedPrecondition;
  }

  PendingStateSnapshot snapshot;
  ResultCode status = component_manager_.read_joint_states(*data_, snapshot.joints);
  if (status != ResultCode::Ok) {
    return status;
  }
  status = component_manager_.read_mobile_base_states(*data_, snapshot.mobile_bases);
  if (status != ResultCode::Ok) {
    return status;
  }
  pending_state_snapshot_ = std::move(snapshot);
  return ResultCode::Ok;
}

ResultCode Simulation::publish_state_snapshot(bool increment_step_count) {
  std::shared_ptr<const StateSnapshot> published_snapshot;
  SnapshotObserver snapshot_observer;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const ResultCode status =
        publish_state_snapshot_locked(increment_step_count, &published_snapshot);
    if (status != ResultCode::Ok) {
      return status;
    }
    snapshot_observer = snapshot_observer_;
  }
  if (snapshot_observer != nullptr && published_snapshot != nullptr) {
    snapshot_observer(std::move(published_snapshot));
  }
  return ResultCode::Ok;
}

ResultCode Simulation::publish_state_snapshot_locked(
    bool increment_step_count, std::shared_ptr<const StateSnapshot>* published_snapshot) {
  if (state_buffer_ == nullptr || model_runtime_ == nullptr || !model_runtime_->is_loaded() ||
      model_ == nullptr || data_ == nullptr) {
    return ResultCode::FailedPrecondition;
  }

  const std::uint64_t snapshot_step_count =
      increment_step_count ? state_snapshot_step_count_ + 1 : state_snapshot_step_count_;
  const double simulation_time = model_runtime_->simulation_time();
  ResultCode status =
      component_manager_.update_components(*model_, *data_, simulation_time, snapshot_step_count,
                                           camera_renderer_.get(), camera_buffer_.get());
  if (status != ResultCode::Ok) {
    return status;
  }

  auto snapshot = std::make_shared<StateSnapshot>();
  if (pending_state_snapshot_.has_value()) {
    snapshot->joints = std::move(pending_state_snapshot_->joints);
    snapshot->mobile_bases = std::move(pending_state_snapshot_->mobile_bases);
    pending_state_snapshot_.reset();
    status = component_manager_.read_imu_states(snapshot->imus);
    if (status != ResultCode::Ok) {
      return status;
    }
    status = component_manager_.read_lidar_states(snapshot->lidars);
    if (status != ResultCode::Ok) {
      return status;
    }
  } else {
    status = component_manager_.read_states(*data_, *snapshot);
    if (status != ResultCode::Ok) {
      return status;
    }
  }
  snapshot->sequence = ++state_snapshot_sequence_;
  snapshot->simulation_time = simulation_time;
  snapshot->timestamp_ns = snapshot->simulation_time <= 0.0
                               ? 0
                               : static_cast<std::uint64_t>(snapshot->simulation_time * 1.0e9);
  snapshot->step_count = snapshot_step_count;
  std::shared_ptr<const StateSnapshot> published = snapshot;
  state_buffer_->write(published);
  state_snapshot_step_count_ = snapshot_step_count;
  if (published_snapshot != nullptr) {
    *published_snapshot = std::move(published);
  }
  return ResultCode::Ok;
}

ResultCode Simulation::scheduler_reset_locked(const ResetOptions& options) {
  std::shared_ptr<const StateSnapshot> published_snapshot;
  SnapshotObserver snapshot_observer;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (model_runtime_ == nullptr || !model_runtime_->is_loaded()) {
      return ResultCode::FailedPrecondition;
    }

    ResultCode status = model_runtime_->reset(options);
    if (status != ResultCode::Ok) {
      return status;
    }

    if (options.reset_components && model_ != nullptr && data_ != nullptr) {
      status = component_manager_.reset_all(*model_, *data_);
      if (status != ResultCode::Ok) {
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
    if (status != ResultCode::Ok) {
      return status;
    }
    if (state_buffer_ != nullptr) {
      status = publish_state_snapshot_locked(false, &published_snapshot);
      if (status != ResultCode::Ok) {
        return status;
      }
    }
    snapshot_observer = snapshot_observer_;
  }

  if (snapshot_observer != nullptr && published_snapshot != nullptr) {
    snapshot_observer(std::move(published_snapshot));
  }

  return ResultCode::Ok;
}

double Simulation::scheduler_timestep_locked() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (model_runtime_ == nullptr || !model_runtime_->is_loaded()) {
    return 0.0;
  }
  return model_runtime_->timestep();
}

ResultCode Simulation::start_viewer() {
  auto viewer = std::make_unique<MuJoCoViewer>(config_.viewer_startup_timeout);
  const ResultCode status = viewer->start(model_, data_, config_.model.model_path);
  if (status != ResultCode::Ok) {
    return status;
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    viewer_ = std::move(viewer);
    next_viewer_sync_time_ = std::chrono::steady_clock::now();
    runtime_error_ = ResultCode::Ok;
  }
  return ResultCode::Ok;
}

ResultCode Simulation::scheduler_sync_viewer_if_due() {
  std::unique_ptr<MuJoCoViewer> viewer_to_stop;
  ResultCode failure_status = ResultCode::Ok;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (viewer_ == nullptr) {
      return ResultCode::Ok;
    }

    const auto now = std::chrono::steady_clock::now();
    std::chrono::nanoseconds period = kDefaultViewerPeriod;
    if (config_.scheduler.viewer_update_rate > 0.0) {
      period = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(1.0 / config_.scheduler.viewer_update_rate));
    }
    if (now < next_viewer_sync_time_) {
      return ResultCode::Ok;
    }

    if (!viewer_->is_running()) {
      failure_status = ResultCode::InvalidState;
      viewer_to_stop = std::move(viewer_);
    } else if (!viewer_->is_ready()) {
      failure_status = ResultCode::InvalidState;
      viewer_to_stop = std::move(viewer_);
    } else {
      const ResultCode sync_status = viewer_->sync(false);
      if (sync_status != ResultCode::Ok) {
        failure_status = sync_status;
        viewer_to_stop = std::move(viewer_);
      } else {
        next_viewer_sync_time_ = now + period;
        return ResultCode::Ok;
      }
    }

    runtime_error_ = failure_status;
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
