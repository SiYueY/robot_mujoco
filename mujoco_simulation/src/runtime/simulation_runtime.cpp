#include "mujoco_simulation/runtime/simulation_runtime.hpp"

#include <filesystem>
#include <string>

#include "mujoco_simulation/common/logging.hpp"

namespace mujoco_simulation {

SimulationRuntime::SimulationRuntime(SimulationRuntime&& other) noexcept
    : context_(std::move(other.context_)), initialized_(other.initialized_) {
  other.initialized_ = false;
}

SimulationRuntime& SimulationRuntime::operator=(SimulationRuntime&& other) noexcept {
  if (this != &other) {
    context_ = std::move(other.context_);
    initialized_ = other.initialized_;
    other.initialized_ = false;
  }
  return *this;
}

bool SimulationRuntime::load_model(const std::string& file_path, mjModel*& model) {
  model = nullptr;
  if (file_path.empty()) {
    LOG_ERROR << "model file path must not be empty.";
    return false;
  }

  const std::filesystem::path path(file_path);
  if (!std::filesystem::is_regular_file(path)) {
    LOG_ERROR << "model path '" << path << "' is not a regular file.";
    return false;
  }

  if (path.extension() == ".mjb") {
    LOG_ERROR << "binary MuJoCo model files are not supported: '" << path << "'.";
    return false;
  }

  mjModel* mj_model = nullptr;
  const int error_message_length = 1024;
  char error_message[error_message_length] = {0};
  mj_model = mj_loadXML(path.c_str(), nullptr, error_message, error_message_length);
  if (mj_model == nullptr) {
    LOG_ERROR << "failed to load xml model file '" << path << "': " << error_message;
    return false;
  }

  if (!(mj_model->opt.timestep > 0.0)) {
    LOG_ERROR << "model timestep must be positive.";
    mj_deleteModel(mj_model);
    return false;
  }

  model = mj_model;
  return true;
}

bool SimulationRuntime::init(const ModelConfig& config) {
  initialized_ = false;
  context_.clear();

  mjModel* mj_model = nullptr;
  if (!load_model(config.model_path, mj_model)) {
    return false;
  }

  mjData* data = mj_makeData(mj_model);
  if (data == nullptr) {
    mj_deleteModel(mj_model);
    LOG_ERROR << "failed to allocate MuJoCo data.";
    return false;
  }

  context_ = mjContext(mj_model, data);

  if (!config.initial_keyframe.empty()) {
    const int keyframe_id = mj_name2id(context_.model, mjOBJ_KEY, config.initial_keyframe.c_str());
    if (keyframe_id < 0) {
      LOG_ERROR << "keyframe '" << config.initial_keyframe << "' was not found.";
      context_.clear();
      return false;
    }
    if (!reset_to_keyframe(keyframe_id)) {
      context_.clear();
      return false;
    }
  } else {
    mj_forward(context_.model, context_.data);
  }

  initialized_ = true;
  return true;
}

bool SimulationRuntime::reset_to_default() {
  mj_resetData(context_.model, context_.data);
  mj_forward(context_.model, context_.data);
  return true;
}

bool SimulationRuntime::reset_to_keyframe(int keyframe_id) {
  mj_resetDataKeyframe(context_.model, context_.data, keyframe_id);
  mj_forward(context_.model, context_.data);
  return true;
}

bool SimulationRuntime::is_initialized() const noexcept { return initialized_; }

bool SimulationRuntime::step() { return step(1); }

bool SimulationRuntime::step(std::size_t count) {
  if (count == 0) {
    LOG_ERROR << "step count must be greater than zero.";
    return false;
  }
  if (!is_initialized()) {
    LOG_ERROR << "model runtime is not loaded.";
    return false;
  }

  for (std::size_t i = 0; i < count; ++i) {
    mj_step(context_.model, context_.data);
  }
  return true;
}

bool SimulationRuntime::forward() {
  if (!is_initialized()) {
    LOG_ERROR << "model runtime is not loaded.";
    return false;
  }
  mj_forward(context_.model, context_.data);
  return true;
}

bool SimulationRuntime::reset() {
  if (!is_initialized()) {
    LOG_ERROR << "model runtime is not loaded.";
    return false;
  }
  return reset_to_default();
}

bool SimulationRuntime::reset_to_keyframe_name(std::string_view keyframe_name) {
  if (!is_initialized()) {
    LOG_ERROR << "model runtime is not loaded.";
    return false;
  }
  if (keyframe_name.empty()) {
    LOG_ERROR << "keyframe name must not be empty.";
    return false;
  }
  const int keyframe_id = mj_name2id(context_.model, mjOBJ_KEY, std::string(keyframe_name).c_str());
  if (keyframe_id < 0) {
    LOG_ERROR << "keyframe '" << keyframe_name << "' was not found.";
    return false;
  }
  return reset_to_keyframe(keyframe_id);
}

bool SimulationRuntime::reset_to_keyframe_id(int keyframe_id) {
  if (!is_initialized()) {
    LOG_ERROR << "model runtime is not loaded.";
    return false;
  }
  if (keyframe_id < 0) {
    LOG_ERROR << "keyframe id must not be negative.";
    return false;
  }
  if (keyframe_id >= context_.model->nkey) {
    LOG_ERROR << "keyframe id " << keyframe_id << " was not found.";
    return false;
  }
  return reset_to_keyframe(keyframe_id);
}

const mjContext& SimulationRuntime::context() const noexcept { return context_; }

double SimulationRuntime::simulation_time() const noexcept {
  return context_.data == nullptr ? 0.0 : context_.data->time;
}

double SimulationRuntime::timestep() const noexcept {
  return context_.model == nullptr ? 0.0 : context_.model->opt.timestep;
}

}  // namespace mujoco_simulation
