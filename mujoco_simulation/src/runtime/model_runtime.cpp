#include "mujoco_simulation/runtime/model_runtime.hpp"

#include <filesystem>
#include <string>

namespace mujoco_simulation {
namespace {

constexpr int kLoadErrorLength = 1024;

Status load_model_from_path(const std::string& model_path, MjModelPtr* model) {
  if (model == nullptr) {
    return Status::invalid_argument("ModelRuntime requires a non-null output model pointer.");
  }
  if (model_path.empty()) {
    return Status::invalid_argument("Model path must not be empty.");
  }

  const std::filesystem::path path(model_path);
  if (!std::filesystem::exists(path)) {
    return Status::model_load_failed("Failed to load MuJoCo model from '" + model_path +
                                     "': model path does not exist.");
  }

  mjModel* raw_model = nullptr;
  if (path.extension() == ".mjb") {
    raw_model = mj_loadModel(model_path.c_str(), nullptr);
    if (raw_model == nullptr) {
      return Status::model_load_failed("Failed to load MuJoCo binary model from '" + model_path +
                                       "'.");
    }
  } else {
    char load_error[kLoadErrorLength] = {0};
    raw_model = mj_loadXML(model_path.c_str(), nullptr, load_error, kLoadErrorLength);
    if (raw_model == nullptr) {
      return Status::model_load_failed("Failed to load MuJoCo XML model from '" + model_path +
                                       "': " + load_error);
    }
  }

  *model = MjModelPtr(raw_model);
  return Status::Ok();
}

}  // namespace

Status ModelRuntime::load(const ModelConfig& config) {
  MjModelPtr new_model;
  const Status load_status = load_model_from_path(config.model_path, &new_model);
  if (!load_status.ok()) {
    return load_status;
  }

  MjDataPtr new_data(mj_makeData(new_model.get()));
  if (new_data == nullptr) {
    return Status::internal("Failed to allocate MuJoCo data for model '" + config.model_path +
                            "'.");
  }

  model_ = std::move(new_model);
  data_ = std::move(new_data);

  Status status = validate_loaded_model(config);
  if (!status.ok()) {
    unload();
    return status;
  }

  if (!config.initial_keyframe.empty()) {
    status = reset({.keyframe_name = config.initial_keyframe});
    if (!status.ok()) {
      unload();
      return status.code() == StatusCode::NotFound
                 ? Status::model_validation_failed("MuJoCo model '" + config.model_path +
                                                   "' does not define initial keyframe '" +
                                                   config.initial_keyframe + "'.")
                 : status;
    }
  } else {
    mj_forward(model_.get(), data_.get());
  }

  return Status::Ok();
}

void ModelRuntime::unload() {
  data_.reset();
  model_.reset();
}

bool ModelRuntime::is_loaded() const noexcept { return model_ != nullptr && data_ != nullptr; }

Status ModelRuntime::step() { return step(1); }

Status ModelRuntime::step(std::size_t count) {
  if (count == 0) {
    return Status::invalid_argument("Step count must be greater than zero.");
  }
  if (!is_loaded()) {
    return Status::failed_precondition("MuJoCo model is not loaded.");
  }

  for (std::size_t i = 0; i < count; ++i) {
    mj_step(model_.get(), data_.get());
  }
  return Status::Ok();
}

Status ModelRuntime::forward() {
  if (!is_loaded()) {
    return Status::failed_precondition("MuJoCo model is not loaded.");
  }
  mj_forward(model_.get(), data_.get());
  return Status::Ok();
}

Status ModelRuntime::reset(const ResetOptions& options) {
  if (!is_loaded()) {
    return Status::failed_precondition("MuJoCo model is not loaded.");
  }

  if (options.keyframe_name.has_value() && options.keyframe_id.has_value()) {
    return Status::invalid_argument(
        "ResetOptions may specify either keyframe_name or keyframe_id, but not both.");
  }

  if (!options.keyframe_name.has_value() && !options.keyframe_id.has_value()) {
    mj_resetData(model_.get(), data_.get());
  } else {
    int keyframe_id = -1;
    if (options.keyframe_name.has_value()) {
      keyframe_id = mj_name2id(model_.get(), mjOBJ_KEY, options.keyframe_name->c_str());
      if (keyframe_id < 0) {
        return Status::not_found("MuJoCo keyframe not found: " + *options.keyframe_name);
      }
    } else {
      keyframe_id = *options.keyframe_id;
    }
    if (keyframe_id < 0) {
      return Status::invalid_argument("MuJoCo keyframe id must be non-negative.");
    }
    if (keyframe_id >= model_->nkey) {
      return Status::not_found("MuJoCo keyframe id out of range: " + std::to_string(keyframe_id));
    }
    mj_resetDataKeyframe(model_.get(), data_.get(), keyframe_id);
  }

  mj_forward(model_.get(), data_.get());
  return Status::Ok();
}

const mjModel& ModelRuntime::model() const { return *model_; }

mjModel& ModelRuntime::mutable_model() { return *model_; }

const mjData& ModelRuntime::data() const { return *data_; }

mjData& ModelRuntime::mutable_data() { return *data_; }

double ModelRuntime::simulation_time() const noexcept {
  return data_ == nullptr ? 0.0 : data_->time;
}

double ModelRuntime::timestep() const noexcept {
  return model_ == nullptr ? 0.0 : model_->opt.timestep;
}

Status ModelRuntime::copy_data_to(mjData& destination) const {
  if (!is_loaded()) {
    return Status::failed_precondition("MuJoCo model is not loaded.");
  }
  mj_copyData(&destination, model_.get(), data_.get());
  return Status::Ok();
}

Status ModelRuntime::validate_loaded_model(const ModelConfig& config) const {
  if (!is_loaded()) {
    return Status::failed_precondition("MuJoCo model is not loaded.");
  }
  if (!(model_->opt.timestep > 0.0)) {
    return Status::model_validation_failed("MuJoCo model '" + config.model_path +
                                           "' must define a positive physics timestep.");
  }
  return Status::Ok();
}

}  // namespace mujoco_simulation
