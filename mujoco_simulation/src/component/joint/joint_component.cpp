#include "component/joint/joint_component.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "common/compare.hpp"
#include "log/logging.hpp"
#include "common/macro.hpp"

namespace mujoco_simulation {

JointComponent::JointComponent(JointInfo info)
: SimulationComponent(info.joint_name, info.period), info_(std::move(info)) {}

bool JointComponent::init(const mjContext& context) {
    initialized_ = false;
    if (!configure(context) || !validate_info()) return false;

    const mjModel& model = *context.model;
    joint_.joint_id = mj_name2id(&model, mjOBJ_JOINT, info_.joint_name.c_str());
    if (joint_.joint_id < 0) {
        SIM_ERROR << "joint '" << info_.joint_name << "' was not found in the model.";
        return false;
    }
    const int mujoco_joint_type = model.jnt_type[joint_.joint_id];
    if (mujoco_joint_type == mjJNT_HINGE) {
        info_.joint_type = JointType::Revolute;
    } else if (mujoco_joint_type == mjJNT_SLIDE) {
        info_.joint_type = JointType::Prismatic;
    } else {
        SIM_ERROR << "joint '" << info_.joint_name << "' has MuJoCo type " << mujoco_joint_type
                  << "; only hinge and slide joints are supported.";
        return false;
    }
    joint_.qpos_address = model.jnt_qposadr[joint_.joint_id];
    if (joint_.qpos_address < 0) {
        SIM_ERROR << "joint '" << info_.joint_name << "' has an invalid qpos address.";
        return false;
    }
    joint_.dof_address = model.jnt_dofadr[joint_.joint_id];
    if (joint_.dof_address < 0) {
        SIM_ERROR << "joint '" << info_.joint_name << "' has an invalid dof address.";
        return false;
    }
    shortest_angular_distance_ =
        mujoco_joint_type == mjJNT_HINGE && model.jnt_limited[joint_.joint_id] == 0;
    if (!validate_actuator_uniqueness(context)) return false;
    if (is_passive_joint()) {
        joint_.actuator_id = -1;
        initialized_ = true;
        return true;
    }
    joint_.actuator_id = mj_name2id(&model, mjOBJ_ACTUATOR, info_.actuator_name.c_str());
    if (joint_.actuator_id < 0) {
        SIM_ERROR << "joint '" << info_.joint_name << "' actuator '" << info_.actuator_name
                  << "' was not found in the model.";
        return false;
    }
    if (!validate_actuator(context)) return false;
    if (info_.hybrid.gravity_compensation || info_.position.gravity_compensation ||
        info_.velocity.gravity_compensation || info_.effort.gravity_compensation) {
        gravity_data_.reset(mj_makeData(&model));
        if (gravity_data_ == nullptr) {
            SIM_ERROR << "joint '" << info_.joint_name
                      << "' failed to allocate gravity-compensation MuJoCo data.";
            return false;
        }
    }
    initialized_ = true;
    return true;
}

bool JointComponent::validate_actuator(const mjContext& context) const {
    const mjModel& model = *context.model;
    if (model.actuator_trntype[joint_.actuator_id] != mjTRN_JOINT) {
        SIM_ERROR << "joint '" << info_.joint_name << "' actuator '" << info_.actuator_name
                  << "' has transmission type " << model.actuator_trntype[joint_.actuator_id]
                  << ", expected joint transmission.";
        return false;
    }
    if (model.actuator_trnid[2 * joint_.actuator_id] != joint_.joint_id) {
        SIM_ERROR << "joint '" << info_.joint_name << "' actuator '" << info_.actuator_name
                  << "' is bound to joint id " << model.actuator_trnid[2 * joint_.actuator_id]
                  << ", expected id " << joint_.joint_id << ".";
        return false;
    }
    if (model.actuator_dyntype[joint_.actuator_id] != mjDYN_NONE) {
        SIM_ERROR << "joint '" << info_.joint_name << "' actuator '" << info_.actuator_name
                  << "' has dynamics type " << model.actuator_dyntype[joint_.actuator_id]
                  << ", expected no dynamics.";
        return false;
    }
    if (model.actuator_gaintype[joint_.actuator_id] != mjGAIN_FIXED) {
        SIM_ERROR << "joint '" << info_.joint_name << "' actuator '" << info_.actuator_name
                  << "' has gain type " << model.actuator_gaintype[joint_.actuator_id]
                  << ", expected fixed gain.";
        return false;
    }
    const mjtNum* gain = model.actuator_gainprm + joint_.actuator_id * mjNGAIN;
    if (!math::equal(static_cast<double>(gain[0]), 1.0)) {
        SIM_ERROR << "joint '" << info_.joint_name << "' actuator '" << info_.actuator_name
                  << "' has gain " << gain[0] << ", expected 1.";
        return false;
    }
    if (model.actuator_biastype[joint_.actuator_id] != mjBIAS_NONE) {
        SIM_ERROR << "joint '" << info_.joint_name << "' actuator '" << info_.actuator_name
                  << "' has bias type " << model.actuator_biastype[joint_.actuator_id]
                  << ", expected no bias.";
        return false;
    }
    const mjtNum* gear = model.actuator_gear + joint_.actuator_id * 6;
    if (!math::equal(static_cast<double>(gear[0]), 1.0)) {
        SIM_ERROR << "joint '" << info_.joint_name << "' actuator '" << info_.actuator_name
                  << "' has gear " << gear[0] << ", expected 1.";
        return false;
    }
    return true;
}

bool JointComponent::validate_actuator_uniqueness(const mjContext& context) const {
    const mjModel& model = *context.model;
    int count = 0;
    for (int actuator_id = 0; actuator_id < model.nu; ++actuator_id) {
        if (model.actuator_trntype[actuator_id] == mjTRN_JOINT &&
            model.actuator_trnid[2 * actuator_id] == joint_.joint_id)
            ++count;
    }
    const int expected = is_active_joint() ? 1 : 0;
    if (count != expected) {
        SIM_ERROR << "joint '" << info_.joint_name << "' has " << count
                  << " joint actuator(s), expected " << expected << ".";
        return false;
    }
    return true;
}

bool JointComponent::validate_info() const {
    const bool limits_valid = info_.position_limits.min <= info_.position_limits.max &&
                              info_.velocity_limits.min <= info_.velocity_limits.max &&
                              info_.effort_limits.min <= info_.effort_limits.max;
    const bool gains_valid =
        std::isfinite(info_.hybrid.stiffness) && info_.hybrid.stiffness >= 0.0 &&
        std::isfinite(info_.hybrid.damping) && info_.hybrid.damping >= 0.0 &&
        std::isfinite(info_.position.stiffness) && info_.position.stiffness >= 0.0 &&
        std::isfinite(info_.position.damping) && info_.position.damping >= 0.0 &&
        std::isfinite(info_.velocity.damping) && info_.velocity.damping >= 0.0;
    if (!limits_valid || !gains_valid) {
        SIM_ERROR << "joint '" << info_.joint_name << "' has invalid limits or controller gains.";
        return false;
    }
    if (is_active_joint()) {
        if (info_.actuator_name.empty() || info_.default_mode == JointMode::None ||
            info_.allowed_modes.empty() || !info_.allowed_modes.contains(info_.default_mode) ||
            info_.allowed_modes.contains(JointMode::None)) {
            SIM_ERROR << "active joint '" << info_.joint_name
                      << "' has invalid command configuration.";
            return false;
        }
        return true;
    }
    const bool passive_defaults =
        info_.actuator_name.empty() && info_.default_mode == JointMode::None &&
        info_.allowed_modes.empty() && info_.hybrid.stiffness == 0.0 &&
        info_.hybrid.damping == 0.0 && info_.position.stiffness == 0.0 &&
        info_.position.damping == 0.0 && info_.velocity.damping == 0.0 &&
        !info_.hybrid.gravity_compensation && !info_.position.gravity_compensation &&
        !info_.velocity.gravity_compensation && !info_.effort.gravity_compensation;
    if (!passive_defaults) {
        SIM_ERROR << "passive joint '" << info_.joint_name
                  << "' must not configure an actuator or controller.";
        return false;
    }
    return true;
}

bool JointComponent::reset(const mjContext& context) {
    JointCommand command;
    return reset(context, command);
}

bool JointComponent::reset(const mjContext& context, JointCommand& command) {
    if (!is_initialized()) {
        SIM_ERROR << "joint '" << info_.joint_name << "' is not initialized.";
        return false;
    }
    state_.reset();
    if (is_passive_joint()) {
        command = {};
        command.mode = static_cast<std::uint8_t>(JointMode::None);
        return true;
    }
    context.data->ctrl[joint_.actuator_id] = 0.0;
    return make_reset_command(context, command) && write(context, command);
}

bool JointComponent::supports_mode(std::uint8_t mode_value) const noexcept {
    const JointMode mode = static_cast<JointMode>(mode_value);
    return mode != JointMode::None && info_.allowed_modes.contains(mode);
}

bool JointComponent::make_reset_command(const mjContext& context, JointCommand& command) const {
    if (!is_initialized() || !context.valid()) return false;
    command = {};
    command.id = info_.id;
    command.mode = static_cast<std::uint8_t>(info_.default_mode);
    command.position = context.data->qpos[joint_.qpos_address];
    switch (info_.default_mode) {
        case JointMode::Hybrid:
            command.stiffness = info_.hybrid.stiffness;
            command.damping = info_.hybrid.damping;
            break;
        case JointMode::Position:
        case JointMode::Velocity:
        case JointMode::Effort:
        case JointMode::None:
            break;
    }
    return true;
}

bool JointComponent::advance(const mjContext& context) {
    UNUSED(context);
    return true;
}

bool JointComponent::update(const mjContext& context) {
    if (!is_initialized()) {
        SIM_ERROR << "joint '" << info_.joint_name << "' is not initialized.";
        return false;
    }

    auto state = std::make_shared<JointState>();
    state->id = info_.id;
    state->timestamp = context.data->time;
    state->position = context.data->qpos[joint_.qpos_address];
    state->velocity = context.data->qvel[joint_.dof_address];
    state->effort = context.data->qfrc_actuator[joint_.dof_address];
    state->mode = is_active_joint() ? command_.mode : static_cast<std::uint8_t>(JointMode::None);
    state_ = std::move(state);
    return true;
}

bool JointComponent::write(const mjContext& context, const JointCommand& command) {
    if (!is_initialized()) {
        SIM_ERROR << "joint '" << info_.joint_name << "' is not initialized.";
        return false;
    }
    if (is_passive_joint()) {
        SIM_ERROR << "joint '" << info_.joint_name << "' is passive and does not accept commands.";
        return false;
    }
    if (!supports_mode(command.mode)) {
        SIM_ERROR << "joint '" << info_.joint_name << "' does not support control mode "
                  << static_cast<int>(command.mode) << ".";
        return false;
    }

    bool result = false;
    switch (static_cast<JointMode>(command.mode)) {
        case JointMode::Hybrid:
            result = write_hybrid_command(context, command);
            break;
        case JointMode::Position:
            result = write_position_command(context, command);
            break;
        case JointMode::Velocity:
            result = write_velocity_command(context, command);
            break;
        case JointMode::Effort:
            result = write_effort_command(context, command);
            break;
        default:
            SIM_ERROR << "joint '" << info_.joint_name << "' received unsupported control mode "
                      << static_cast<int>(command.mode) << ".";
            return false;
    }
    if (!result) return false;
    command_ = command;
    return true;
}

bool JointComponent::read_state(std::shared_ptr<const JointState>& state) const {
    if (!is_initialized()) {
        SIM_ERROR << "joint '" << info_.joint_name << "' is not initialized.";
        return false;
    }
    state = state_;
    return state != nullptr;
}

bool JointComponent::read(const mjContext& context, JointState& state) const {
    UNUSED(context);
    std::shared_ptr<const JointState> snapshot;
    if (!read_state(snapshot)) {
        return false;
    }
    state = *snapshot;
    return true;
}

bool JointComponent::is_initialized() const noexcept { return initialized_; }

bool JointComponent::is_active_joint() const noexcept {
    return info_.actuation == JointActuation::Active;
}

bool JointComponent::is_passive_joint() const noexcept {
    return info_.actuation == JointActuation::Passive;
}

double JointComponent::position_error(double target, double current) const noexcept {
    if (!shortest_angular_distance_) return target - current;
    constexpr double kPi = 3.14159265358979323846;
    constexpr double kTwoPi = 6.28318530717958647692;
    double error = std::remainder(target - current, kTwoPi);
    if (error >= kPi) error -= kTwoPi;
    return error;
}

bool JointComponent::write_position_command(
    const mjContext& context, const JointCommand& command) const {
    if (!std::isfinite(command.position)) {
        SIM_ERROR << "joint '" << info_.joint_name << "' position command must be finite.";
        return false;
    }

    const double position_state = context.data->qpos[joint_.qpos_address];
    const double velocity_state = context.data->qvel[joint_.dof_address];
    double effort =
        info_.position.stiffness *
            position_error(clamp_limits(info_.position_limits, command.position), position_state) +
        info_.position.damping * (0.0 - velocity_state);
    if (info_.position.gravity_compensation) effort += gravity_compensation_effort(context);
    effort = clamp_limits(info_.effort_limits, effort);
    effort = clamp_force_limits(context, effort);
    context.data->ctrl[joint_.actuator_id] = clamp_ctrl_limits(context, effort);
    return true;
}

bool JointComponent::write_velocity_command(
    const mjContext& context, const JointCommand& command) const {
    if (!std::isfinite(command.velocity)) {
        SIM_ERROR << "joint '" << info_.joint_name << "' velocity command must be finite.";
        return false;
    }
    const double velocity_state = context.data->qvel[joint_.dof_address];
    double effort = info_.velocity.damping *
                    (clamp_limits(info_.velocity_limits, command.velocity) - velocity_state);
    if (info_.velocity.gravity_compensation) effort += gravity_compensation_effort(context);
    effort = clamp_limits(info_.effort_limits, effort);
    effort = clamp_force_limits(context, effort);
    context.data->ctrl[joint_.actuator_id] = clamp_ctrl_limits(context, effort);
    return true;
}

bool JointComponent::write_effort_command(
    const mjContext& context, const JointCommand& command) const {
    if (!std::isfinite(command.effort)) {
        SIM_ERROR << "joint '" << info_.joint_name << "' effort command must be finite.";
        return false;
    }
    double effort = command.effort;
    if (info_.effort.gravity_compensation) effort += gravity_compensation_effort(context);
    effort = clamp_limits(info_.effort_limits, effort);
    effort = clamp_force_limits(context, effort);
    context.data->ctrl[joint_.actuator_id] = clamp_ctrl_limits(context, effort);
    return true;
}

bool JointComponent::write_hybrid_command(
    const mjContext& context, const JointCommand& command) const {
    if (!std::isfinite(command.position) || !std::isfinite(command.velocity) ||
        !std::isfinite(command.effort) || !std::isfinite(command.stiffness) ||
        !std::isfinite(command.damping)) {
        SIM_ERROR << "joint '" << info_.joint_name << "' hybrid command values must be finite.";
        return false;
    }
    if (command.stiffness < 0.0) {
        SIM_ERROR << "joint '" << info_.joint_name << "' hybrid stiffness must be non-negative.";
        return false;
    }
    if (command.damping < 0.0) {
        SIM_ERROR << "joint '" << info_.joint_name << "' hybrid damping must be non-negative.";
        return false;
    }
    const double position_state = context.data->qpos[joint_.qpos_address];
    const double velocity_state = context.data->qvel[joint_.dof_address];
    double effort =
        command.effort +
        command.stiffness *
            position_error(clamp_limits(info_.position_limits, command.position), position_state) +
        command.damping * (clamp_limits(info_.velocity_limits, command.velocity) - velocity_state);
    if (info_.hybrid.gravity_compensation) effort += gravity_compensation_effort(context);
    effort = clamp_limits(info_.effort_limits, effort);
    effort = clamp_force_limits(context, effort);
    context.data->ctrl[joint_.actuator_id] = clamp_ctrl_limits(context, effort);
    return true;
}

double JointComponent::gravity_compensation_effort(const mjContext& context) const {
    const mjModel& model = *context.model;
    mjData& gravity_data = *gravity_data_;

    // Use only the current configuration.  Resetting the scratch state leaves its
    // velocity and acceleration at zero, so qfrc_bias is the pure gravity term.
    mj_resetData(&model, &gravity_data);
    std::copy_n(context.data->qpos, model.nq, gravity_data.qpos);
    mj_forward(&model, &gravity_data);
    return static_cast<double>(gravity_data.qfrc_bias[joint_.dof_address]);
}

double JointComponent::clamp_limits(const JointLimit& limits, double value) const {
    return std::clamp(value, limits.min, limits.max);
}

double JointComponent::clamp_ctrl_limits(const mjContext& context, double value) const {
    if (context.model->actuator_ctrllimited[joint_.actuator_id] == 0) {
        return value;
    }
    const mjtNum* range = context.model->actuator_ctrlrange + 2 * joint_.actuator_id;
    return std::clamp(value, static_cast<double>(range[0]), static_cast<double>(range[1]));
}

double JointComponent::clamp_force_limits(const mjContext& context, double value) const {
    if (context.model->actuator_forcelimited[joint_.actuator_id] == 0) {
        return value;
    }
    const mjtNum* range = context.model->actuator_forcerange + 2 * joint_.actuator_id;
    return std::clamp(value, static_cast<double>(range[0]), static_cast<double>(range[1]));
}

}  // namespace mujoco_simulation
