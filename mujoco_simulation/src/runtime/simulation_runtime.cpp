#include "runtime/simulation_runtime.hpp"

#include <cmath>
#include <filesystem>
#include <string>

#include "log/logging.hpp"

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
        SIM_ERROR << "model file path must not be empty.";
        return false;
    }

    const std::filesystem::path path(file_path);
    if (!std::filesystem::is_regular_file(path)) {
        SIM_ERROR << "model path '" << path << "' is not a regular file.";
        return false;
    }

    if (path.extension() == ".mjb") {
        SIM_ERROR << "binary MuJoCo model files are not supported: '" << path << "'.";
        return false;
    }

    mjModel* mj_model = nullptr;
    const int error_message_length = 1024;
    char error_message[error_message_length] = {0};
    mj_model = mj_loadXML(path.c_str(), nullptr, error_message, error_message_length);
    if (mj_model == nullptr) {
        SIM_ERROR << "failed to load xml model file '" << path << "': " << error_message;
        return false;
    }

    if (!(mj_model->opt.timestep > 0.0)) {
        SIM_ERROR << "model timestep must be positive.";
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
        SIM_ERROR << "failed to allocate MuJoCo data.";
        return false;
    }

    context_ = mjContext(mj_model, data);

    if (!config.initial_keyframe.empty()) {
        const int keyframe_id =
            mj_name2id(context_.model, mjOBJ_KEY, config.initial_keyframe.c_str());
        if (keyframe_id < 0) {
            SIM_ERROR << "keyframe '" << config.initial_keyframe << "' was not found.";
            context_.clear();
            return false;
        }
        if (!reset_to_keyframe(keyframe_id)) {
            context_.clear();
            return false;
        }
    } else {
        reset_to_default();
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
        SIM_ERROR << "step count must be greater than zero.";
        return false;
    }
    if (!is_initialized()) {
        SIM_ERROR << "model runtime is not loaded.";
        return false;
    }

    for (std::size_t i = 0; i < count; ++i) {
        mj_step(context_.model, context_.data);
    }
    return true;
}

bool SimulationRuntime::forward() {
    if (!is_initialized()) {
        SIM_ERROR << "model runtime is not loaded.";
        return false;
    }
    mj_forward(context_.model, context_.data);
    return true;
}

bool SimulationRuntime::set_timestep(double timestep) {
    if (!is_initialized()) {
        SIM_ERROR << "model runtime is not loaded.";
        return false;
    }
    if (!std::isfinite(timestep) || timestep <= 0.0) {
        SIM_ERROR << "MuJoCo timestep must be finite and positive.";
        return false;
    }
    mjModel* model = const_cast<mjModel*>(context_.model);
    model->opt.timestep = timestep;
    return forward();
}

bool SimulationRuntime::reset() {
    if (!is_initialized()) {
        SIM_ERROR << "model runtime is not loaded.";
        return false;
    }
    return reset_to_default();
}

bool SimulationRuntime::reset(std::string keyframe_name) {
    if (!is_initialized()) {
        SIM_ERROR << "model runtime is not loaded.";
        return false;
    }
    if (keyframe_name.empty()) {
        SIM_ERROR << "keyframe name must not be empty.";
        return false;
    }
    const int keyframe_id =
        mj_name2id(context_.model, mjOBJ_KEY, std::string(keyframe_name).c_str());
    if (keyframe_id < 0) {
        SIM_ERROR << "keyframe '" << keyframe_name << "' was not found.";
        return false;
    }
    return reset_to_keyframe(keyframe_id);
}

const mjContext& SimulationRuntime::context() const noexcept { return context_; }

double SimulationRuntime::time() const noexcept {
    return context_.data == nullptr ? 0.0 : context_.data->time;
}

double SimulationRuntime::timestep() const noexcept {
    return context_.model == nullptr ? 0.0 : context_.model->opt.timestep;
}

}  // namespace mujoco_simulation
