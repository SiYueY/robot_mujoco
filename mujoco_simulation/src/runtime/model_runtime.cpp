#include "mujoco_simulation/runtime/model_runtime.hpp"

#include <filesystem>
#include <memory>
#include <string>

namespace mujoco_simulation {
namespace {

constexpr int kLoadErrorLength = 1024;

struct MjModelDeleter {
  void operator()(mjModel* model) const noexcept {
    if (model != nullptr) {
      mj_deleteModel(model);
    }
  }
};

struct MjDataDeleter {
  void operator()(mjData* data) const noexcept {
    if (data != nullptr) {
      mj_deleteData(data);
    }
  }
};

using MjModelPtr = std::unique_ptr<mjModel, MjModelDeleter>;
using MjDataPtr = std::unique_ptr<mjData, MjDataDeleter>;

ResultCode load_model_from_path(const std::string& model_path, MjModelPtr* model) {
  if (model == nullptr) {
    return ResultCode::InvalidArgument;
  }
  if (model_path.empty()) {
    return ResultCode::InvalidArgument;
  }

  const std::filesystem::path path(model_path);
  if (!std::filesystem::exists(path)) {
    return ResultCode::ModelLoadFailed;
  }

  mjModel* raw_model = nullptr;
  if (path.extension() == ".mjb") {
    raw_model = mj_loadModel(model_path.c_str(), nullptr);
    if (raw_model == nullptr) {
      return ResultCode::ModelLoadFailed;
    }
  } else {
    char load_error[kLoadErrorLength] = {0};
    raw_model = mj_loadXML(model_path.c_str(), nullptr, load_error, kLoadErrorLength);
    if (raw_model == nullptr) {
      return ResultCode::ModelLoadFailed;
    }
  }

  *model = MjModelPtr(raw_model);
  return ResultCode::Ok;
}

ResultCode reset_to_default(mjModel* model, mjData* data) {
  mj_resetData(model, data);
  mj_forward(model, data);
  return ResultCode::Ok;
}

ResultCode reset_to_keyframe_name(mjModel* model, mjData* data, std::string_view keyframe_name) {
  if (keyframe_name.empty()) {
    return ResultCode::InvalidArgument;
  }
  const int keyframe_id = mj_name2id(model, mjOBJ_KEY, std::string(keyframe_name).c_str());
  if (keyframe_id < 0) {
    return ResultCode::NotFound;
  }
  mj_resetDataKeyframe(model, data, keyframe_id);
  mj_forward(model, data);
  return ResultCode::Ok;
}

ResultCode reset_to_keyframe_id(mjModel* model, mjData* data, int keyframe_id) {
  if (keyframe_id < 0) {
    return ResultCode::InvalidArgument;
  }
  if (keyframe_id >= model->nkey) {
    return ResultCode::NotFound;
  }
  mj_resetDataKeyframe(model, data, keyframe_id);
  mj_forward(model, data);
  return ResultCode::Ok;
}

}  // namespace

struct ModelRuntime::Impl {
  MjModelPtr model;
  MjDataPtr data;
};

ViewerRuntimeHandle::ViewerRuntimeHandle(const ModelRuntime* runtime) noexcept
    : runtime_(runtime) {}

bool ViewerRuntimeHandle::valid() const noexcept { return model() != nullptr && data() != nullptr; }

const mjModel* ViewerRuntimeHandle::model() const noexcept {
  return runtime_ == nullptr || !runtime_->is_loaded() ? nullptr : &runtime_->model();
}

const mjData* ViewerRuntimeHandle::data() const noexcept {
  return runtime_ == nullptr || !runtime_->is_loaded() ? nullptr : &runtime_->data();
}

ModelRuntime::ModelRuntime() : impl_(std::make_unique<Impl>()) {}

ModelRuntime::~ModelRuntime() = default;

ModelRuntime::ModelRuntime(ModelRuntime&&) noexcept = default;

ModelRuntime& ModelRuntime::operator=(ModelRuntime&&) noexcept = default;

ResultCode ModelRuntime::load(const ModelConfig& config) {
  MjModelPtr new_model;
  const ResultCode load_status = load_model_from_path(config.model_path, &new_model);
  if (load_status != ResultCode::Ok) {
    return load_status;
  }

  MjDataPtr new_data(mj_makeData(new_model.get()));
  if (new_data == nullptr) {
    return ResultCode::Internal;
  }

  impl_->model = std::move(new_model);
  impl_->data = std::move(new_data);

  ResultCode status = validate_loaded_model(config);
  if (status != ResultCode::Ok) {
    unload();
    return status;
  }

  if (!config.initial_keyframe.empty()) {
    status = reset_to_keyframe_name(config.initial_keyframe);
    if (status != ResultCode::Ok) {
      unload();
      return status == ResultCode::NotFound ? ResultCode::ModelValidationFailed : status;
    }
  } else {
    mj_forward(impl_->model.get(), impl_->data.get());
  }

  return ResultCode::Ok;
}

void ModelRuntime::unload() {
  impl_->data.reset();
  impl_->model.reset();
}

bool ModelRuntime::is_loaded() const noexcept {
  return impl_ != nullptr && impl_->model != nullptr && impl_->data != nullptr;
}

ResultCode ModelRuntime::step() { return step(1); }

ResultCode ModelRuntime::step(std::size_t count) {
  if (count == 0) {
    return ResultCode::InvalidArgument;
  }
  if (!is_loaded()) {
    return ResultCode::FailedPrecondition;
  }

  for (std::size_t i = 0; i < count; ++i) {
    mj_step(impl_->model.get(), impl_->data.get());
  }
  return ResultCode::Ok;
}

ResultCode ModelRuntime::forward() {
  if (!is_loaded()) {
    return ResultCode::FailedPrecondition;
  }
  mj_forward(impl_->model.get(), impl_->data.get());
  return ResultCode::Ok;
}

ResultCode ModelRuntime::reset() {
  if (!is_loaded()) {
    return ResultCode::FailedPrecondition;
  }
  return reset_to_default(impl_->model.get(), impl_->data.get());
}

ResultCode ModelRuntime::reset_to_keyframe_name(std::string_view keyframe_name) {
  if (!is_loaded()) {
    return ResultCode::FailedPrecondition;
  }
  return mujoco_simulation::reset_to_keyframe_name(impl_->model.get(), impl_->data.get(),
                                                   keyframe_name);
}

ResultCode ModelRuntime::reset_to_keyframe_id(int keyframe_id) {
  if (!is_loaded()) {
    return ResultCode::FailedPrecondition;
  }
  return mujoco_simulation::reset_to_keyframe_id(impl_->model.get(), impl_->data.get(),
                                                 keyframe_id);
}

const mjModel& ModelRuntime::model() const { return *impl_->model; }

mjModel& ModelRuntime::mutable_model() { return *impl_->model; }

const mjData& ModelRuntime::data() const { return *impl_->data; }

mjData& ModelRuntime::mutable_data() { return *impl_->data; }

double ModelRuntime::simulation_time() const noexcept {
  return impl_ == nullptr || impl_->data == nullptr ? 0.0 : impl_->data->time;
}

double ModelRuntime::timestep() const noexcept {
  return impl_ == nullptr || impl_->model == nullptr ? 0.0 : impl_->model->opt.timestep;
}

ViewerRuntimeHandle ModelRuntime::viewer_runtime_handle() const noexcept {
  return ViewerRuntimeHandle(this);
}

ResultCode ModelRuntime::validate_loaded_model(const ModelConfig& config) const {
  if (!is_loaded()) {
    return ResultCode::FailedPrecondition;
  }
  if (!(impl_->model->opt.timestep > 0.0)) {
    return ResultCode::ModelValidationFailed;
  }
  return ResultCode::Ok;
}

}  // namespace mujoco_simulation
