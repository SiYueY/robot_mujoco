#include "mujoco_simulation/mujoco_simulation.hpp"

#include <chrono>
#include <memory>
#include <stdexcept>

#include "mujoco_simulation/hardware/hardware_manager.hpp"
#include "mujoco_simulation/viewer/viewer.hpp"

namespace mujoco_simulation {
namespace {
constexpr int kLoadErrorLength = 1024;

int model_name_to_id(const mjModel* model, int object_type, const std::string& name) {
  if (model == nullptr || name.empty()) {
    return -1;
  }
  return mj_name2id(model, object_type, name.c_str());
}
}  // namespace

MuJoCoSimulation::MuJoCoSimulation() = default;

MuJoCoSimulation::~MuJoCoSimulation() {
  stop();

  std::lock_guard<std::mutex> lock(mutex_);
  if (data_ != nullptr) {
    mj_deleteData(data_);
    data_ = nullptr;
  }
  if (model_ != nullptr) {
    mj_deleteModel(model_);
    model_ = nullptr;
  }
}

bool MuJoCoSimulation::initialize(const SimulationConfig& config, std::string& error_message) {
  if (is_initialized()) {
    error_message = "MuJoCoSimulation is already initialized.";
    return false;
  }

  config_ = config;

  if (!load_model(config.model_path, error_message)) {
    return false;
  }

  if (config.render_mode == RenderMode::Viewer && !start_viewer(error_message)) {
    return false;
  }

  if (!config.initial_keyframe.empty() && !reset(config.initial_keyframe, error_message)) {
    return false;
  }

  return true;
}

void MuJoCoSimulation::start() {
  if (running_.exchange(true)) {
    return;
  }
  physics_thread_ = std::thread([this]() { physics_loop(); });
}

void MuJoCoSimulation::stop() {
  running_.store(false);
  if (physics_thread_.joinable()) {
    physics_thread_.join();
  }
  if (viewer_ != nullptr) {
    viewer_->stop();
  }
  std::lock_guard<std::mutex> lock(mutex_);
  viewer_started_ = false;
  viewer_render_resources_injected_ = false;
}

bool MuJoCoSimulation::set_paused(bool paused) {
  paused_.store(paused);
  return true;
}

bool MuJoCoSimulation::paused() const { return paused_.load(); }

bool MuJoCoSimulation::reset(const std::string& keyframe, std::string& error_message) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (model_ == nullptr || data_ == nullptr) {
    error_message = "MuJoCo model is not loaded.";
    return false;
  }

  if (keyframe.empty()) {
    mj_resetData(model_, data_);
  } else {
    const int id = keyframe_id(keyframe);
    if (id < 0) {
      error_message = "MuJoCo keyframe not found: " + keyframe;
      return false;
    }
    mj_resetDataKeyframe(model_, data_, id);
  }

  mj_forward(model_, data_);
  step_count_.store(0);
  return true;
}

bool MuJoCoSimulation::step(uint32_t steps, std::string& error_message) {
  if (steps == 0) {
    error_message = "Step count must be greater than zero.";
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (model_ == nullptr || data_ == nullptr) {
    error_message = "MuJoCo model is not loaded.";
    return false;
  }

  for (uint32_t i = 0; i < steps; ++i) {
    mj_step(model_, data_);
    ++step_count_;
  }
  if (viewer_ != nullptr && !viewer_->sync(false, error_message)) {
    return false;
  }
  return true;
}

bool MuJoCoSimulation::register_joint(const JointInfo& configuration, std::string& error_message) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (hardware_manager_ == nullptr) {
    error_message = "MuJoCo hardware manager is not initialized.";
    return false;
  }

  if (!hardware_manager_->register_joint(configuration)) {
    error_message = hardware_manager_->last_error();
    return false;
  }
  error_message.clear();
  return true;
}

bool MuJoCoSimulation::configure_joint_command_mode(const std::string& joint_name,
                                                    CommandInterfaceType command_mode,
                                                    std::string& error_message) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (hardware_manager_ == nullptr) {
    error_message = "MuJoCo hardware manager is not initialized.";
    return false;
  }

  if (!hardware_manager_->configure_joint_command_mode(joint_name, command_mode)) {
    error_message = hardware_manager_->last_error();
    return false;
  }
  error_message.clear();
  return true;
}

bool MuJoCoSimulation::write_joint(const JointCommand& command, std::string& error_message) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (hardware_manager_ == nullptr) {
    error_message = "MuJoCo hardware manager is not initialized.";
    return false;
  }

  if (!hardware_manager_->write_joint(command.name, command)) {
    error_message = hardware_manager_->last_error();
    return false;
  }
  error_message.clear();
  return true;
}

bool MuJoCoSimulation::read_joint(const std::string& joint_name, JointState* state,
                                  std::string& error_message) {
  if (state == nullptr) {
    error_message = "Joint state output pointer must not be null.";
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (hardware_manager_ == nullptr) {
    error_message = "MuJoCo hardware manager is not initialized.";
    return false;
  }

  if (!hardware_manager_->read_joint(joint_name, *state)) {
    error_message = hardware_manager_->last_error();
    return false;
  }
  error_message.clear();
  return true;
}

bool MuJoCoSimulation::register_imu(const ImuInfo& configuration, std::string& error_message) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (hardware_manager_ == nullptr) {
    error_message = "MuJoCo hardware manager is not initialized.";
    return false;
  }

  if (!hardware_manager_->register_imu(configuration)) {
    error_message = hardware_manager_->last_error();
    return false;
  }
  error_message.clear();
  return true;
}

bool MuJoCoSimulation::read_imu(const std::string& imu_name, ImuState* state,
                                std::string& error_message) {
  if (state == nullptr) {
    error_message = "IMU state output pointer must not be null.";
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (hardware_manager_ == nullptr) {
    error_message = "MuJoCo hardware manager is not initialized.";
    return false;
  }

  if (!hardware_manager_->read_imu(imu_name, *state)) {
    error_message = hardware_manager_->last_error();
    return false;
  }
  error_message.clear();
  return true;
}

bool MuJoCoSimulation::register_camera(const CameraSpec& configuration,
                                       std::string& error_message) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (hardware_manager_ == nullptr) {
    error_message = "MuJoCo hardware manager is not initialized.";
    return false;
  }
  if (configuration.name.empty()) {
    error_message = "Camera name must not be empty.";
    return false;
  }
  if (!hardware_manager_->register_camera(configuration)) {
    error_message = hardware_manager_->last_error();
    return false;
  }

  error_message.clear();
  return true;
}

bool MuJoCoSimulation::read_camera(const std::string& camera_name, CameraState* state,
                                   std::string& error_message) {
  if (state == nullptr) {
    error_message = "Camera state output pointer must not be null.";
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (hardware_manager_ == nullptr) {
    error_message = "MuJoCo hardware manager is not initialized.";
    return false;
  }
  if (!viewer_render_resources_ready_locked()) {
    error_message = "Camera access requires render_mode=viewer.";
    return false;
  }
  if (!hardware_manager_->ensure_camera_registered(camera_name)) {
    error_message = hardware_manager_->last_error();
    return false;
  }

  if (!hardware_manager_->read_camera(camera_name, *state)) {
    error_message = hardware_manager_->last_error();
    return false;
  }
  error_message.clear();
  return true;
}

bool MuJoCoSimulation::register_lidar(const LidarInfo& configuration, std::string& error_message) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (hardware_manager_ == nullptr) {
    error_message = "MuJoCo hardware manager is not initialized.";
    return false;
  }

  if (!hardware_manager_->register_lidar(configuration)) {
    error_message = hardware_manager_->last_error();
    return false;
  }
  error_message.clear();
  return true;
}

bool MuJoCoSimulation::read_lidar(const std::string& lidar_name, LidarState* state,
                                  std::string& error_message) {
  if (state == nullptr) {
    error_message = "Lidar state output pointer must not be null.";
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (hardware_manager_ == nullptr) {
    error_message = "MuJoCo hardware manager is not initialized.";
    return false;
  }

  if (!hardware_manager_->read_lidar(lidar_name, *state)) {
    error_message = hardware_manager_->last_error();
    return false;
  }
  error_message.clear();
  return true;
}

bool MuJoCoSimulation::register_mobile_base(const MobileBaseInfo& configuration,
                                            std::string& error_message) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (hardware_manager_ == nullptr) {
    error_message = "MuJoCo hardware manager is not initialized.";
    return false;
  }

  if (!hardware_manager_->register_mobile_base(configuration)) {
    error_message = hardware_manager_->last_error();
    return false;
  }
  error_message.clear();
  return true;
}

bool MuJoCoSimulation::write_mobile_base(const std::string& name, const MobileBaseCommand& command,
                                         std::string& error_message) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (hardware_manager_ == nullptr) {
    error_message = "MuJoCo hardware manager is not initialized.";
    return false;
  }

  if (!hardware_manager_->write_mobile_base(name, command)) {
    error_message = hardware_manager_->last_error();
    return false;
  }
  error_message.clear();
  return true;
}

bool MuJoCoSimulation::read_mobile_base(const std::string& name, MobileBaseState* state,
                                        std::string& error_message) {
  if (state == nullptr) {
    error_message = "Mobile base state output pointer must not be null.";
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (hardware_manager_ == nullptr) {
    error_message = "MuJoCo hardware manager is not initialized.";
    return false;
  }

  if (!hardware_manager_->read_mobile_base(name, *state)) {
    error_message = hardware_manager_->last_error();
    return false;
  }
  error_message.clear();
  return true;
}

uint64_t MuJoCoSimulation::step_count() const { return step_count_.load(); }

double MuJoCoSimulation::simulation_time() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (data_ == nullptr) {
    return 0.0;
  }
  return data_->time;
}

bool MuJoCoSimulation::has_joint(const std::string& joint_name) const {
  return joint_id(joint_name) >= 0;
}

bool MuJoCoSimulation::has_body(const std::string& body_name) const {
  return body_id(body_name) >= 0;
}

bool MuJoCoSimulation::has_site(const std::string& site_name) const {
  return site_id(site_name) >= 0;
}

bool MuJoCoSimulation::has_camera(const std::string& camera_name) const {
  return camera_id(camera_name) >= 0;
}

int MuJoCoSimulation::joint_id(const std::string& joint_name) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return model_name_to_id(model_, mjOBJ_JOINT, joint_name);
}

int MuJoCoSimulation::body_id(const std::string& body_name) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return model_name_to_id(model_, mjOBJ_BODY, body_name);
}

int MuJoCoSimulation::site_id(const std::string& site_name) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return model_name_to_id(model_, mjOBJ_SITE, site_name);
}

int MuJoCoSimulation::camera_id(const std::string& camera_name) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return model_name_to_id(model_, mjOBJ_CAMERA, camera_name);
}

bool MuJoCoSimulation::camera_fovy(const std::string& camera_name, double* fovy_degrees,
                                   std::string& error_message) const {
  if (fovy_degrees == nullptr) {
    error_message = "Camera fovy output pointer must not be null.";
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (model_ == nullptr) {
    error_message = "MuJoCo model is not loaded.";
    return false;
  }

  const int camera_id = model_name_to_id(model_, mjOBJ_CAMERA, camera_name);
  if (camera_id < 0) {
    error_message = "MuJoCo camera not found: " + camera_name;
    return false;
  }

  *fovy_degrees = static_cast<double>(model_->cam_fovy[camera_id]);
  error_message.clear();
  return true;
}

const mjModel* MuJoCoSimulation::model() const { return model_; }

bool MuJoCoSimulation::is_initialized() const { return model_ != nullptr && data_ != nullptr; }

bool MuJoCoSimulation::is_running() const { return running_.load(); }

const SimulationConfig& MuJoCoSimulation::config() const { return config_; }

void MuJoCoSimulation::with_locked_data(
    const std::function<void(const mjModel&, mjData&)>& callback) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (model_ != nullptr && data_ != nullptr) {
    callback(*model_, *data_);
  }
}

void MuJoCoSimulation::with_locked_data(
    const std::function<void(const mjModel&, const mjData&)>& callback) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (model_ != nullptr && data_ != nullptr) {
    callback(*model_, *data_);
  }
}

bool MuJoCoSimulation::copy_data_to(mjData* dest) const {
  if (dest == nullptr) {
    return false;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (model_ == nullptr || data_ == nullptr) {
    return false;
  }
  mj_copyData(dest, model_, data_);
  return true;
}

void MuJoCoSimulation::physics_loop() {
  using clock = std::chrono::steady_clock;

  while (running_.load()) {
    double timestep = 0.001;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (model_ != nullptr) {
        timestep = model_->opt.timestep;
      }
    }

    const double speed = config_.sim_speed_factor > 0.0 ? config_.sim_speed_factor : 1.0;
    const auto period = std::chrono::duration_cast<clock::duration>(
        std::chrono::duration<double>(timestep / speed));

    auto wake_time = clock::now();

    if (!paused_.load()) {
      std::string ignored;
      step(1, ignored);
    }

    wake_time += period;
    std::this_thread::sleep_until(wake_time);
  }
}

bool MuJoCoSimulation::load_model(const std::string& model_path, std::string& error_message) {
  if (model_path.empty()) {
    error_message = "mujoco_model_path must not be empty.";
    return false;
  }

  char load_error[kLoadErrorLength] = {0};
  mjModel* new_model = mj_loadXML(model_path.c_str(), nullptr, load_error, kLoadErrorLength);
  if (new_model == nullptr) {
    error_message = std::string("Failed to load MuJoCo model: ") + load_error;
    return false;
  }

  mjData* new_data = mj_makeData(new_model);
  if (new_data == nullptr) {
    mj_deleteModel(new_model);
    error_message = "Failed to allocate MuJoCo data.";
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (data_ != nullptr) {
    mj_deleteData(data_);
  }
  if (model_ != nullptr) {
    mj_deleteModel(model_);
  }
  model_ = new_model;
  data_ = new_data;
  hardware_manager_ = std::make_unique<HardwareManager>(model_, data_);
  viewer_.reset();
  viewer_started_ = false;
  viewer_render_resources_injected_ = false;
  mj_forward(model_, data_);
  return true;
}

bool MuJoCoSimulation::start_viewer(std::string& error_message) {
  viewer_ = std::make_unique<Viewer>();
  if (!viewer_->start(model_, data_, config_.model_path, error_message)) {
    viewer_.reset();
    return false;
  }
  viewer_started_ = true;

  constexpr auto kViewerStartupTimeout = std::chrono::seconds(5);
  const auto deadline = std::chrono::steady_clock::now() + kViewerStartupTimeout;
  while (std::chrono::steady_clock::now() < deadline) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (inject_viewer_render_resources_locked(error_message)) {
        viewer_started_ = true;
        viewer_render_resources_injected_ = true;
        return true;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  error_message = "MuJoCo viewer failed to expose render resources within startup timeout.";
  viewer_->stop();
  viewer_.reset();
  return false;
}

bool MuJoCoSimulation::inject_viewer_render_resources_locked(std::string& error_message) {
  if (viewer_ == nullptr || hardware_manager_ == nullptr) {
    error_message = "MuJoCo viewer or hardware manager is not initialized.";
    return false;
  }
  if (!viewer_render_resources_ready_locked()) {
    error_message = "MuJoCo viewer render resources are not ready.";
    return false;
  }
  if (viewer_render_resources_injected_) {
    error_message.clear();
    return true;
  }

  hardware_manager_->set_render_resources(viewer_->scene(), viewer_->render_context());
  if (!hardware_manager_->register_pending_cameras()) {
    error_message = hardware_manager_->last_error();
    return false;
  }
  error_message.clear();
  return true;
}

bool MuJoCoSimulation::viewer_render_resources_ready_locked() const {
  return viewer_ != nullptr && viewer_->scene() != nullptr && viewer_->render_context() != nullptr;
}

int MuJoCoSimulation::keyframe_id(const std::string& keyframe) const {
  return mj_name2id(model_, mjOBJ_KEY, keyframe.c_str());
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
