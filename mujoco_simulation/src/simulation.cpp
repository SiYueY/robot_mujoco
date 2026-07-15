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
  (void)stop_viewer();

  {
    std::lock_guard<std::mutex> lock(runtime_mutex_);
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
    runtime_error_ = ResultCode::Ok;
  }

  return ResultCode::Ok;
}

ResultCode Simulation::start() {
  if (scheduler_ == nullptr) {
    return ResultCode::InvalidState;
  }

  bool should_start_viewer = false;
  if (config_.render_mode == RenderMode::Viewer) {
    {
      std::lock_guard<std::mutex> runtime_lock(runtime_mutex_);
      if (model_runtime_ == nullptr || !model_runtime_->is_loaded() || model_ == nullptr ||
          data_ == nullptr) {
        return ResultCode::FailedPrecondition;
      }
    }
    std::lock_guard<std::mutex> viewer_lock(viewer_mutex_);
    if (viewer_ == nullptr) {
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
  {
    std::lock_guard<std::mutex> lock(runtime_mutex_);
    runtime_error_ = ResultCode::Ok;
  }
  (void)stop_viewer();
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

  std::lock_guard<std::mutex> lock(runtime_mutex_);
  config_.scheduler.realtime_factor = realtime_factor;
  return ResultCode::Ok;
}

ResultCode Simulation::request_reset(const ResetOptions& options) {
  return request_reset_internal({.target = {}, .options = options});
}

ResultCode Simulation::request_reset_to_keyframe_name(std::string_view keyframe_name,
                                                      const ResetOptions& options) {
  ResetRequest request;
  request.target.type = ResetTargetType::KeyframeName;
  request.target.keyframe_name = std::string(keyframe_name);
  request.options = options;
  return request_reset_internal(request);
}

ResultCode Simulation::request_reset_to_keyframe_id(int keyframe_id, const ResetOptions& options) {
  ResetRequest request;
  request.target.type = ResetTargetType::KeyframeId;
  request.target.keyframe_id = keyframe_id;
  request.options = options;
  return request_reset_internal(request);
}

ResultCode Simulation::request_reset_internal(const ResetRequest& request) {
  if (scheduler_ == nullptr) {
    return ResultCode::InvalidState;
  }
  return scheduler_->request_reset(request);
}

ResultCode Simulation::reset(const ResetOptions& options) {
  return reset_internal({.target = {}, .options = options});
}

ResultCode Simulation::reset_to_keyframe_name(std::string_view keyframe_name,
                                              const ResetOptions& options) {
  ResetRequest request;
  request.target.type = ResetTargetType::KeyframeName;
  request.target.keyframe_name = std::string(keyframe_name);
  request.options = options;
  return reset_internal(std::move(request));
}

ResultCode Simulation::reset_to_keyframe_id(int keyframe_id, const ResetOptions& options) {
  ResetRequest request;
  request.target.type = ResetTargetType::KeyframeId;
  request.target.keyframe_id = keyframe_id;
  request.options = options;
  return reset_internal(std::move(request));
}

ResultCode Simulation::reset_internal(ResetRequest request) {
  if (scheduler_ == nullptr) {
    return ResultCode::InvalidState;
  }

  std::future<ResultCode> completion = scheduler_->request_reset_waitable(std::move(request));
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
    std::lock_guard<std::mutex> lock(runtime_mutex_);
    if (model_ == nullptr) {
      return ResultCode::InvalidState;
    }
    if (!component_manager_.reconfigure_component(*model_, updated_component)) {
      return ResultCode::Internal;
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
  std::lock_guard<std::mutex> lock(runtime_mutex_);
  CommandBuffer* command_buffer = command_buffer_.get();
  if (command_buffer == nullptr) {
    return ResultCode::InvalidState;
  }
  return command_buffer->write_joint_command(command.joint, command);
}

bool Simulation::joint_state(std::string joint_name, JointState* out) const {
  if (out == nullptr) {
    return false;
  }
  std::lock_guard<std::mutex> lock(runtime_mutex_);
  StateBuffer* state_buffer = state_buffer_.get();
  if (state_buffer == nullptr) {
    return false;
  }
  return state_buffer->read_joint_state(joint_name, out);
}

bool Simulation::imu_state(std::string imu_name, ImuState* out) const {
  if (out == nullptr) {
    return false;
  }
  std::lock_guard<std::mutex> lock(runtime_mutex_);
  StateBuffer* state_buffer = state_buffer_.get();
  if (state_buffer == nullptr) {
    return false;
  }
  return state_buffer->read_imu_state(imu_name, out);
}

bool Simulation::camera_state(std::string camera_name, CameraState* out) const {
  if (out == nullptr) {
    return false;
  }
  std::lock_guard<std::mutex> lock(runtime_mutex_);
  CameraBuffer* camera_buffer = camera_buffer_.get();
  if (camera_buffer == nullptr) {
    return false;
  }
  return camera_buffer->read(camera_name, out);
}

bool Simulation::lidar_state(std::string lidar_name, LidarState* out) const {
  if (out == nullptr) {
    return false;
  }
  std::lock_guard<std::mutex> lock(runtime_mutex_);
  StateBuffer* state_buffer = state_buffer_.get();
  if (state_buffer == nullptr) {
    return false;
  }
  return state_buffer->read_lidar_state(lidar_name, out);
}

ResultCode Simulation::set_mobile_base_command(std::string name, const MobileBaseCommand& command) {
  std::lock_guard<std::mutex> lock(runtime_mutex_);
  CommandBuffer* command_buffer = command_buffer_.get();
  if (command_buffer == nullptr) {
    return ResultCode::InvalidState;
  }
  return command_buffer->write_mobile_base_command(name, command);
}

bool Simulation::mobile_base_state(std::string name, MobileBaseState* out) const {
  if (out == nullptr) {
    return false;
  }
  std::lock_guard<std::mutex> lock(runtime_mutex_);
  StateBuffer* state_buffer = state_buffer_.get();
  if (state_buffer == nullptr) {
    return false;
  }
  return state_buffer->read_mobile_base_state(name, out);
}

uint64_t Simulation::step_count() const {
  if (scheduler_ == nullptr) {
    return 0;
  }
  return scheduler_->statistics().physics_steps;
}

SimulationStatus Simulation::status() const {
  std::lock_guard<std::mutex> lock(runtime_mutex_);
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
  std::lock_guard<std::mutex> lock(runtime_mutex_);
  if (model_runtime_ == nullptr || !model_runtime_->is_loaded()) {
    return 0.0;
  }
  return model_runtime_->simulation_time();
}

std::shared_ptr<const StateSnapshot> Simulation::state_snapshot() const {
  std::lock_guard<std::mutex> lock(runtime_mutex_);
  StateBuffer* state_buffer = state_buffer_.get();
  return state_buffer == nullptr ? nullptr : state_buffer->read();
}

void Simulation::set_snapshot_observer(SnapshotObserver observer) {
  std::lock_guard<std::mutex> lock(runtime_mutex_);
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
  callbacks.timestep_provider = [this]() { return scheduler_timestep(); };
  callbacks.write_commands = [this]() { return scheduler_write_commands(); };
  callbacks.step_physics = [this]() { return scheduler_step_physics(); };
  callbacks.update_components = [this]() { return scheduler_update_components(); };
  callbacks.write_state_snapshot = [this]() { return write_state_snapshot(true); };
  callbacks.sync_viewer_if_due = [this]() { return scheduler_sync_viewer_if_due(); };
  callbacks.reset_runtime = [this](const ResetRequest& request) {
    return scheduler_reset(request);
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
    std::lock_guard<std::mutex> lock(runtime_mutex_);
    if (model_ == nullptr) {
      return ResultCode::InvalidState;
    }
    if (!component_manager_.build(*model_, config_.components)) {
      return ResultCode::Internal;
    }
    const double simulation_time = model_runtime_->simulation_time();
    const ResultCode status = update_components_for_step_locked(0, simulation_time);
    if (status != ResultCode::Ok) {
      return status;
    }
  }
  return write_state_snapshot(false);
}

ResultCode Simulation::load_model(const ModelConfig& model_config) {
  auto runtime = std::make_unique<ModelRuntime>();
  const ResultCode status = runtime->load(model_config);
  if (status != ResultCode::Ok) {
    return status;
  }

  std::lock_guard<std::mutex> lock(runtime_mutex_);
  model_runtime_ = std::move(runtime);
  model_ = &model_runtime_->mutable_model();
  data_ = &model_runtime_->mutable_data();
  component_manager_.clear();
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
  const ResultCode update_status = update_components_for_step_locked(
      state_snapshot_step_count_, model_runtime_->simulation_time());
  if (update_status != ResultCode::Ok) {
    return update_status;
  }
  return write_state_snapshot_locked(false, published_snapshot);
}

ResultCode Simulation::scheduler_step_physics() {
  std::lock_guard<std::mutex> lock(runtime_mutex_);
  if (model_runtime_ == nullptr || !model_runtime_->is_loaded()) {
    return ResultCode::FailedPrecondition;
  }
  return model_runtime_->step();
}

ResultCode Simulation::scheduler_write_commands() {
  std::lock_guard<std::mutex> lock(runtime_mutex_);
  if (command_buffer_ == nullptr || model_ == nullptr || data_ == nullptr) {
    return ResultCode::FailedPrecondition;
  }

  const CommandSnapshot snapshot = command_buffer_->read(CommandBuffer::Clock::now());
  return component_manager_.write_commands(*model_, *data_, snapshot) ? ResultCode::Ok
                                                                      : ResultCode::Internal;
}

ResultCode Simulation::scheduler_update_components() {
  std::lock_guard<std::mutex> lock(runtime_mutex_);
  if (model_runtime_ == nullptr || !model_runtime_->is_loaded() || model_ == nullptr ||
      data_ == nullptr) {
    return ResultCode::FailedPrecondition;
  }

  const std::uint64_t snapshot_step_count = state_snapshot_step_count_ + 1;
  const double simulation_time = model_runtime_->simulation_time();
  return update_components_for_step_locked(snapshot_step_count, simulation_time);
}

ResultCode Simulation::update_components_for_step_locked(std::uint64_t step_count,
                                                         double simulation_time) {
  if (model_runtime_ == nullptr || !model_runtime_->is_loaded() || model_ == nullptr ||
      data_ == nullptr) {
    return ResultCode::FailedPrecondition;
  }
  return component_manager_.update_components(*model_, *data_, simulation_time, step_count,
                                              camera_renderer_.get(), camera_buffer_.get())
             ? ResultCode::Ok
             : ResultCode::Internal;
}

ResultCode Simulation::write_state_snapshot(bool increment_step_count) {
  std::shared_ptr<const StateSnapshot> published_snapshot;
  SnapshotObserver snapshot_observer;
  {
    std::lock_guard<std::mutex> lock(runtime_mutex_);
    const ResultCode status =
        write_state_snapshot_locked(increment_step_count, &published_snapshot);
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

ResultCode Simulation::write_state_snapshot_locked(
    bool increment_step_count, std::shared_ptr<const StateSnapshot>* published_snapshot) {
  if (state_buffer_ == nullptr || model_runtime_ == nullptr || !model_runtime_->is_loaded() ||
      model_ == nullptr || data_ == nullptr) {
    return ResultCode::FailedPrecondition;
  }

  const std::uint64_t snapshot_step_count =
      increment_step_count ? state_snapshot_step_count_ + 1 : state_snapshot_step_count_;
  const double simulation_time = model_runtime_->simulation_time();
  return build_state_snapshot_locked(snapshot_step_count, simulation_time, published_snapshot);
}

ResultCode Simulation::build_state_snapshot_locked(
    std::uint64_t step_count, double simulation_time,
    std::shared_ptr<const StateSnapshot>* published_snapshot) {
  auto snapshot = std::make_shared<StateSnapshot>();
  const ResultCode status = build_state_snapshot(snapshot.get());
  if (status != ResultCode::Ok) {
    return status;
  }
  snapshot->sequence = ++state_snapshot_sequence_;
  snapshot->simulation_time = simulation_time;
  snapshot->timestamp_ns =
      simulation_time <= 0.0 ? 0 : static_cast<std::uint64_t>(simulation_time * 1.0e9);
  snapshot->step_count = step_count;
  std::shared_ptr<const StateSnapshot> published = snapshot;
  state_buffer_->write(published);
  state_snapshot_step_count_ = step_count;
  if (published_snapshot != nullptr) {
    *published_snapshot = std::move(published);
  }
  return ResultCode::Ok;
}

ResultCode Simulation::scheduler_reset(const ResetRequest& request) {
  std::shared_ptr<const StateSnapshot> published_snapshot;
  SnapshotObserver snapshot_observer;
  {
    std::lock_guard<std::mutex> lock(runtime_mutex_);
    if (model_runtime_ == nullptr || !model_runtime_->is_loaded()) {
      return ResultCode::FailedPrecondition;
    }

    ResultCode status = ResultCode::Ok;
    switch (request.target.type) {
      case ResetTargetType::Default:
        status = model_runtime_->reset();
        break;
      case ResetTargetType::KeyframeName:
        status = model_runtime_->reset_to_keyframe_name(request.target.keyframe_name);
        break;
      case ResetTargetType::KeyframeId:
        status = model_runtime_->reset_to_keyframe_id(request.target.keyframe_id);
        break;
    }
    if (status != ResultCode::Ok) {
      return status;
    }

    if (request.options.reset_components && model_ != nullptr && data_ != nullptr) {
      if (!component_manager_.reset_all(*model_, *data_)) {
        return ResultCode::Internal;
      }
    }
    if (request.options.clear_commands && command_buffer_ != nullptr) {
      command_buffer_->clear();
    }
    if (request.options.clear_state_buffer && state_buffer_ != nullptr) {
      state_buffer_->clear();
    }
    if (request.options.clear_camera_buffer && camera_buffer_ != nullptr) {
      camera_buffer_->clear();
    }
    next_viewer_sync_time_ = std::chrono::steady_clock::now();
    if (request.options.reset_statistics) {
      state_snapshot_step_count_ = 0;
      state_snapshot_sequence_ = 0;
    }

    status = update_components_for_step_locked(state_snapshot_step_count_,
                                               model_runtime_->simulation_time());
    if (status != ResultCode::Ok) {
      return status;
    }
    if (state_buffer_ != nullptr) {
      status = write_state_snapshot_locked(false, &published_snapshot);
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

double Simulation::scheduler_timestep() const {
  std::lock_guard<std::mutex> lock(runtime_mutex_);
  if (model_runtime_ == nullptr || !model_runtime_->is_loaded()) {
    return 0.0;
  }
  return model_runtime_->timestep();
}

ResultCode Simulation::start_viewer() {
  ViewerRuntimeHandle runtime_handle;
  {
    std::lock_guard<std::mutex> runtime_lock(runtime_mutex_);
    if (model_runtime_ == nullptr || !model_runtime_->is_loaded() || model_ == nullptr ||
        data_ == nullptr) {
      return ResultCode::FailedPrecondition;
    }
    runtime_handle = model_runtime_->viewer_runtime_handle();
  }

  auto viewer = std::make_unique<MuJoCoViewer>(config_.viewer_startup_timeout);
  const ResultCode status = viewer->start(runtime_handle, config_.model.model_path);
  if (status != ResultCode::Ok) {
    return status;
  }

  {
    std::lock_guard<std::mutex> lock(viewer_mutex_);
    viewer_ = std::move(viewer);
  }
  {
    std::lock_guard<std::mutex> lock(runtime_mutex_);
    next_viewer_sync_time_ = std::chrono::steady_clock::now();
    runtime_error_ = ResultCode::Ok;
  }
  return ResultCode::Ok;
}

ResultCode Simulation::scheduler_sync_viewer_if_due() {
  std::unique_ptr<MuJoCoViewer> viewer_to_stop = nullptr;
  ResultCode failure_status = ResultCode::Ok;
  std::chrono::nanoseconds period = kDefaultViewerPeriod;

  {
    std::lock_guard<std::mutex> runtime_lock(runtime_mutex_);
    const auto now = std::chrono::steady_clock::now();
    if (config_.scheduler.viewer_update_rate > 0.0) {
      period = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(1.0 / config_.scheduler.viewer_update_rate));
    }
    if (now < next_viewer_sync_time_) {
      return ResultCode::Ok;
    }
  }

  {
    std::lock_guard<std::mutex> lock(viewer_mutex_);
    if (viewer_ == nullptr) {
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
        std::lock_guard<std::mutex> runtime_lock(runtime_mutex_);
        const auto now = std::chrono::steady_clock::now();
        next_viewer_sync_time_ = now + period;
        return ResultCode::Ok;
      }
    }
  }
  {
    std::lock_guard<std::mutex> runtime_lock(runtime_mutex_);
    runtime_error_ = failure_status;
  }

  if (viewer_to_stop != nullptr) {
    viewer_to_stop->stop();
  }
  return failure_status;
}

ResultCode Simulation::stop_viewer() {
  std::unique_ptr<MuJoCoViewer> viewer_to_stop;
  {
    std::lock_guard<std::mutex> lock(viewer_mutex_);
    viewer_to_stop = std::move(viewer_);
  }
  if (viewer_to_stop != nullptr) {
    viewer_to_stop->stop();
  }
  return ResultCode::Ok;
}

ResultCode Simulation::build_state_snapshot(StateSnapshot* snapshot) const {
  if (snapshot == nullptr || data_ == nullptr) {
    return ResultCode::InvalidArgument;
  }
  return component_manager_.build_state_snapshot(*data_, *snapshot) ? ResultCode::Ok
                                                                    : ResultCode::Internal;
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
