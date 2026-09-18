#include "component/gripper/gripper_component.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "common/compare.hpp"
#include "common/macro.hpp"
#include "log/logging.hpp"

namespace romujoco {

GripperComponent::GripperComponent(GripperInfo info)
: SimulationComponent(info.name, info.period), info_(std::move(info)) {}

bool GripperComponent::init(const SimulationContext& context) {
    initialized_ = false;
    fingers_ = {};
    if (!configure(context) || !validate_info()) return false;

    std::size_t actuator_count = 0;
    for (std::size_t index = 0; index < fingers_.size(); ++index) {
        if (!bind_finger(context, info_.fingers[index], fingers_[index])) return false;
        if (fingers_[index].actuator_id >= 0) ++actuator_count;
    }
    if (actuator_count == 0U) {
        SIM_ERROR << "gripper '" << info_.name << "' requires at least one finger actuator.";
        return false;
    }
    initialized_ = true;
    return true;
}

bool GripperComponent::validate_info() const {
    const bool limits_valid = info_.width_limits.min <= info_.width_limits.max &&
                              info_.velocity_limits.min <= info_.velocity_limits.max &&
                              info_.effort_limits.min <= info_.effort_limits.max;
    const bool control_valid = std::isfinite(info_.control.stiffness) &&
                               info_.control.stiffness >= 0.0 &&
                               std::isfinite(info_.control.damping) && info_.control.damping >= 0.0;
    const bool stall_valid =
        std::isfinite(info_.stall.width_tolerance) && info_.stall.width_tolerance >= 0.0 &&
        std::isfinite(info_.stall.velocity_threshold) && info_.stall.velocity_threshold >= 0.0 &&
        std::isfinite(info_.stall.effort_ratio) && info_.stall.effort_ratio > 0.0 &&
        info_.stall.effort_ratio <= 1.0 && std::isfinite(info_.stall.timeout) &&
        info_.stall.timeout >= 0.0;
    if (!limits_valid || !control_valid || !stall_valid) {
        SIM_ERROR << "gripper '" << info_.name
                  << "' has invalid limits, control gains or stall parameters.";
        return false;
    }
    if (info_.fingers[0].joint_name.empty() || info_.fingers[1].joint_name.empty() ||
        info_.fingers[0].joint_name == info_.fingers[1].joint_name) {
        SIM_ERROR << "gripper '" << info_.name << "' requires two distinct finger joint names.";
        return false;
    }
    if (info_.fingers[0].actuator_name.empty() && info_.fingers[1].actuator_name.empty()) {
        SIM_ERROR << "gripper '" << info_.name << "' requires at least one finger actuator.";
        return false;
    }
    return true;
}

bool GripperComponent::bind_finger(
    const SimulationContext& context, const GripperFingerInfo& finger_info,
    FingerBinding& binding) const {
    const mjModel& model = *context.model;
    binding.joint_id = mj_name2id(&model, mjOBJ_JOINT, finger_info.joint_name.c_str());
    if (binding.joint_id < 0) {
        SIM_ERROR << "gripper '" << info_.name << "' finger joint '" << finger_info.joint_name
                  << "' was not found in the model.";
        return false;
    }
    if (model.jnt_type[binding.joint_id] != mjJNT_SLIDE) {
        SIM_ERROR << "gripper '" << info_.name << "' finger joint '" << finger_info.joint_name
                  << "' has MuJoCo type " << model.jnt_type[binding.joint_id]
                  << "; only slide joints are supported.";
        return false;
    }
    binding.qpos_address = model.jnt_qposadr[binding.joint_id];
    binding.dof_address = model.jnt_dofadr[binding.joint_id];
    if (binding.qpos_address < 0 || binding.dof_address < 0) {
        SIM_ERROR << "gripper '" << info_.name << "' finger joint '" << finger_info.joint_name
                  << "' has an invalid qpos or dof address.";
        return false;
    }
    if (finger_info.actuator_name.empty()) {
        binding.actuator_id = -1;
        return true;
    }
    binding.actuator_id = mj_name2id(&model, mjOBJ_ACTUATOR, finger_info.actuator_name.c_str());
    if (binding.actuator_id < 0) {
        SIM_ERROR << "gripper '" << info_.name << "' finger actuator '" << finger_info.actuator_name
                  << "' was not found in the model.";
        return false;
    }
    return validate_actuator(context, finger_info, binding);
}

bool GripperComponent::validate_actuator(
    const SimulationContext& context, const GripperFingerInfo& finger_info,
    const FingerBinding& binding) const {
    const mjModel& model = *context.model;
    const int actuator_id = binding.actuator_id;
    if (model.actuator_trntype[actuator_id] != mjTRN_JOINT) {
        SIM_ERROR << "gripper '" << info_.name << "' finger actuator '" << finger_info.actuator_name
                  << "' has transmission type " << model.actuator_trntype[actuator_id]
                  << ", expected joint transmission.";
        return false;
    }
    if (model.actuator_trnid[2 * actuator_id] != binding.joint_id) {
        SIM_ERROR << "gripper '" << info_.name << "' finger actuator '" << finger_info.actuator_name
                  << "' is bound to joint id " << model.actuator_trnid[2 * actuator_id]
                  << ", expected id " << binding.joint_id << ".";
        return false;
    }
    if (model.actuator_dyntype[actuator_id] != mjDYN_NONE) {
        SIM_ERROR << "gripper '" << info_.name << "' finger actuator '" << finger_info.actuator_name
                  << "' has dynamics type " << model.actuator_dyntype[actuator_id]
                  << ", expected no dynamics.";
        return false;
    }
    if (model.actuator_gaintype[actuator_id] != mjGAIN_FIXED) {
        SIM_ERROR << "gripper '" << info_.name << "' finger actuator '" << finger_info.actuator_name
                  << "' has gain type " << model.actuator_gaintype[actuator_id]
                  << ", expected fixed gain.";
        return false;
    }
    const mjtNum* gain = model.actuator_gainprm + actuator_id * mjNGAIN;
    if (!math::equal(static_cast<double>(gain[0]), 1.0)) {
        SIM_ERROR << "gripper '" << info_.name << "' finger actuator '" << finger_info.actuator_name
                  << "' has gain " << gain[0] << ", expected 1.";
        return false;
    }
    if (model.actuator_biastype[actuator_id] != mjBIAS_NONE) {
        SIM_ERROR << "gripper '" << info_.name << "' finger actuator '" << finger_info.actuator_name
                  << "' has bias type " << model.actuator_biastype[actuator_id]
                  << ", expected no bias.";
        return false;
    }
    const mjtNum* gear = model.actuator_gear + actuator_id * 6;
    if (!math::equal(static_cast<double>(gear[0]), 1.0)) {
        SIM_ERROR << "gripper '" << info_.name << "' finger actuator '" << finger_info.actuator_name
                  << "' has gear " << gear[0] << ", expected 1.";
        return false;
    }
    return true;
}

bool GripperComponent::reset(const SimulationContext& context) {
    GripperCommand command;
    return reset(context, command);
}

bool GripperComponent::reset(const SimulationContext& context, GripperCommand& command) {
    if (!initialized_) {
        SIM_ERROR << "gripper '" << info_.name << "' is not initialized.";
        return false;
    }
    const double current_width = measured_width(context);
    target_width_ = current_width;
    reference_width_ = current_width;
    reference_velocity_ = 0.0;
    command_ = {};
    command_.id = info_.id;
    command_.width = current_width;
    command_.velocity = 0.0;
    command_.effort = 0.0;
    stalled_ = false;
    stall_timer_active_ = false;
    stall_start_time_ = 0.0;
    state_.reset();
    for (const FingerBinding& binding : fingers_)
        if (binding.actuator_id >= 0) context.data->ctrl[binding.actuator_id] = 0.0;
    command = command_;
    return true;
}

bool GripperComponent::write(const SimulationContext& context, const GripperCommand& command) {
    UNUSED(context);
    if (!initialized_) {
        SIM_ERROR << "gripper '" << info_.name << "' is not initialized.";
        return false;
    }
    if (command.id != info_.id) {
        SIM_ERROR << "gripper '" << info_.name << "' received a command for id " << command.id
                  << ".";
        return false;
    }
    if (!std::isfinite(command.width) || !std::isfinite(command.velocity) ||
        !std::isfinite(command.effort)) {
        SIM_ERROR << "gripper '" << info_.name << "' command values must be finite.";
        return false;
    }
    if (command.velocity < 0.0 || command.effort < 0.0) {
        SIM_ERROR << "gripper '" << info_.name
                  << "' command velocity and effort must be non-negative.";
        return false;
    }
    command_ = command;
    command_.width = std::clamp(command.width, info_.width_limits.min, info_.width_limits.max);
    command_.velocity = std::clamp(command.velocity, 0.0, info_.velocity_limits.max);
    command_.effort = std::clamp(command.effort, 0.0, info_.effort_limits.max);
    target_width_ = command_.width;
    return true;
}

bool GripperComponent::advance(const SimulationContext& context) {
    if (!initialized_) {
        SIM_ERROR << "gripper '" << info_.name << "' is not initialized.";
        return false;
    }
    const double timestep = context.model->opt.timestep;
    if (!std::isfinite(timestep) || timestep <= 0.0) {
        SIM_ERROR << "gripper '" << info_.name << "' requires a positive model timestep.";
        return false;
    }
    const double error = target_width_ - reference_width_;
    reference_velocity_ = std::clamp(error / timestep, -command_.velocity, command_.velocity);
    reference_width_ += reference_velocity_ * timestep;
    apply_control(context);
    return true;
}

void GripperComponent::apply_control(const SimulationContext& context) {
    // V1 is a symmetric parallel gripper, so both fingers follow the same
    // half-width reference; unactuated fingers are driven by the MJCF coupling.
    const double finger_position_reference = reference_width_ * 0.5;
    const double finger_velocity_reference = reference_velocity_ * 0.5;
    for (const FingerBinding& binding : fingers_) {
        if (binding.actuator_id < 0) continue;
        const double position = context.data->qpos[binding.qpos_address];
        const double velocity = context.data->qvel[binding.dof_address];
        double effort = info_.control.stiffness * (finger_position_reference - position) +
                        info_.control.damping * (finger_velocity_reference - velocity);
        effort = std::clamp(effort, -command_.effort, command_.effort);
        context.data->ctrl[binding.actuator_id] = effort;
    }
}

double GripperComponent::measured_width(const SimulationContext& context) const noexcept {
    return context.data->qpos[fingers_[0].qpos_address] +
           context.data->qpos[fingers_[1].qpos_address];
}

double GripperComponent::measured_width_velocity(const SimulationContext& context) const noexcept {
    return context.data->qvel[fingers_[0].dof_address] +
           context.data->qvel[fingers_[1].dof_address];
}

double GripperComponent::measured_effort(const SimulationContext& context) const noexcept {
    double effort = 0.0;
    for (const FingerBinding& binding : fingers_)
        if (binding.actuator_id >= 0)
            effort = std::max(effort, std::abs(context.data->actuator_force[binding.actuator_id]));
    return effort;
}

void GripperComponent::update_stall_state(
    const SimulationContext& context, const GripperState& state) {
    // A zero (or negligible) effort limit disables stall detection: without a
    // commanded force the gripper cannot be considered blocked by an object.
    const bool blocking = !math::equal(command_.effort, 0.0) &&
                          std::abs(target_width_ - state.width) > info_.stall.width_tolerance &&
                          std::abs(state.velocity) <= info_.stall.velocity_threshold &&
                          state.effort >= command_.effort * info_.stall.effort_ratio;
    if (!blocking) {
        stalled_ = false;
        stall_timer_active_ = false;
        return;
    }
    if (!stall_timer_active_) {
        stall_timer_active_ = true;
        stall_start_time_ = context.data->time;
    }
    const double elapsed = context.data->time - stall_start_time_;
    if (!math::less(elapsed, info_.stall.timeout)) stalled_ = true;
}

bool GripperComponent::update(const SimulationContext& context) {
    if (!initialized_) {
        SIM_ERROR << "gripper '" << info_.name << "' is not initialized.";
        return false;
    }
    auto state = std::make_shared<GripperState>();
    state->id = info_.id;
    state->timestamp = context.data->time;
    state->width = measured_width(context);
    state->velocity = measured_width_velocity(context);
    state->effort = measured_effort(context);
    update_stall_state(context, *state);
    state->stalled = stalled_;
    state_ = std::move(state);
    return true;
}

bool GripperComponent::read_state(std::shared_ptr<const GripperState>& state) const {
    if (!initialized_) {
        SIM_ERROR << "gripper '" << info_.name << "' is not initialized.";
        return false;
    }
    state = state_;
    return state != nullptr;
}

}  // namespace romujoco
