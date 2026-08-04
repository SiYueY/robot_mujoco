#include "component/mobile_base/mobile_base_component.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "common/logging.hpp"
#include "common/macro.hpp"
#include "common/compare.hpp"
#include "mujoco_simulation/common/math.hpp"

namespace mujoco_simulation {

MobileBaseComponent::MobileBaseComponent(MobileBaseInfo info)
: SimulationComponent(info.mobile_base_name, info.period), info_(std::move(info)) {}

bool MobileBaseComponent::init(const mjContext& context) {
    initialized_ = false;
    base_body_id_ = -1;
    mecanum_wheels_ = {};
    mecanum_kinematics_.reset();
    if (!configure(context)) {
        return false;
    }
    if (!configure_base_body(context)) {
        return false;
    }
    switch (info_.type) {
        case MobileBaseType::Mecanum:
            if (!init_mecanum(context)) {
                return false;
            }
            break;
        default:
            LOG_ERROR << "mobile base '" << info_.mobile_base_name << "' type is not supported.";
            return false;
    }

    command_ = {};
    working_state_ = {};
    working_state_.id = info_.id;
    working_state_.base_frame_id = info_.base_frame_id;
    working_state_.odom_frame_id = info_.odom_frame_id;
    state_ = std::make_shared<MobileBaseState>(working_state_);
    reset_odometry();
    initialized_ = true;
    return true;
}

bool MobileBaseComponent::reset(const mjContext& context) {
    if (!is_initialized()) {
        LOG_ERROR << "mobile base '" << info_.mobile_base_name << "' is not initialized.";
        return false;
    }
    switch (info_.type) {
        case MobileBaseType::Mecanum:
            if (!reset_mecanum(context)) {
                return false;
            }
            break;
        default:
            LOG_ERROR << "mobile base '" << info_.mobile_base_name << "' type is not supported.";
            return false;
    }
    command_ = {};
    working_state_.id = info_.id;
    reset_odometry();
    state_ = std::make_shared<MobileBaseState>(working_state_);
    return true;
}

bool MobileBaseComponent::update(const mjContext& context) {
    if (!is_initialized()) {
        LOG_ERROR << "mobile base '" << info_.mobile_base_name << "' is not initialized.";
        return false;
    }

    switch (info_.type) {
        case MobileBaseType::Mecanum:
            if (!update_mecanum_state(context)) {
                return false;
            }
            break;
        default:
            LOG_ERROR << "mobile base '" << info_.mobile_base_name << "' type is not supported.";
            return false;
    }
    if (!update_ground_truth_pose(*context.data)) {
        return false;
    }
    working_state_.timestamp = context.data->time;
    state_ = std::make_shared<MobileBaseState>(working_state_);
    return true;
}

bool MobileBaseComponent::write(const mjContext& context, const MobileBaseCommand& command) {
    if (!is_initialized()) {
        LOG_ERROR << "mobile base '" << info_.mobile_base_name << "' is not initialized.";
        return false;
    }

    bool result = false;
    switch (info_.type) {
        case MobileBaseType::Mecanum:
            result = write_mecanum_command(context, command);
            break;
        default:
            LOG_ERROR << "mobile base '" << info_.mobile_base_name << "' type is not supported.";
            return false;
    }
    if (!result) {
        return false;
    }
    command_ = command;
    return true;
}

bool MobileBaseComponent::read_state(std::shared_ptr<const MobileBaseState>& state) const {
    if (!is_initialized()) {
        LOG_ERROR << "mobile base '" << info_.mobile_base_name << "' is not initialized.";
        return false;
    }
    state = state_;
    return state != nullptr;
}

bool MobileBaseComponent::read(const mjContext& context, MobileBaseState& state) const {
    UNUSED(context);
    std::shared_ptr<const MobileBaseState> snapshot;
    if (!read_state(snapshot)) {
        return false;
    }
    state = *snapshot;
    return true;
}

bool MobileBaseComponent::is_initialized() const noexcept { return initialized_; }

bool MobileBaseComponent::configure_base_body(const mjContext& context) {
    if (info_.base_body_name.empty()) {
        LOG_ERROR << "mobile base '" << info_.mobile_base_name
                  << "' base_body_name is required for ground-truth odometry.";
        return false;
    }
    base_body_id_ = mj_name2id(context.model, mjOBJ_BODY, info_.base_body_name.c_str());
    if (base_body_id_ < 0) {
        LOG_ERROR << "mobile base '" << info_.mobile_base_name << "' body '" << info_.base_body_name
                  << "' was not found.";
        return false;
    }
    return true;
}

bool MobileBaseComponent::init_mecanum(const mjContext& context) {
    const MecanumInfo& info = info_.mecanum_info;
    if (!std::isfinite(info.wheel_radius) || info.wheel_radius <= 0.0) {
        LOG_ERROR << "mobile base '" << info_.mobile_base_name
                  << "' mecanum wheel_radius must be finite and positive.";
        return false;
    }
    if (!std::isfinite(info.wheel_base) || info.wheel_base <= 0.0) {
        LOG_ERROR << "mobile base '" << info_.mobile_base_name
                  << "' mecanum wheel_base must be finite and positive.";
        return false;
    }
    if (!std::isfinite(info.track_width) || info.track_width <= 0.0) {
        LOG_ERROR << "mobile base '" << info_.mobile_base_name
                  << "' mecanum track_width must be finite and positive.";
        return false;
    }
    const double rotation_coefficient = (info.wheel_base + info.track_width) * 0.5;
    if (!std::isfinite(rotation_coefficient) || rotation_coefficient <= 0.0) {
        LOG_ERROR << "mobile base '" << info_.mobile_base_name
                  << "' mecanum rotation coefficient must be finite and positive.";
        return false;
    }

    for (std::size_t wheel_index = 0; wheel_index < MecanumWheelCount; ++wheel_index) {
        if (!configure_wheel(
                context, info_.mecanum_wheels[wheel_index], mecanum_wheels_[wheel_index])) {
            return false;
        }
    }

    mecanum_kinematics_.emplace(info);
    return true;
}

bool MobileBaseComponent::reset_mecanum(const mjContext& context) {
    for (std::size_t wheel_index = 0; wheel_index < MecanumWheelCount; ++wheel_index) {
        context.data->ctrl[mecanum_wheels_[wheel_index].actuator_id] = 0.0;
    }
    return true;
}

bool MobileBaseComponent::update_mecanum_state(const mjContext& context) {
    if (!mecanum_kinematics_) {
        LOG_ERROR << "mobile base '" << info_.mobile_base_name
                  << "' mecanum kinematics are not initialized.";
        return false;
    }
    Vector4d wheel_angular{};
    for (std::size_t wheel_index = 0; wheel_index < MecanumWheelCount; ++wheel_index) {
        wheel_angular[wheel_index] = context.data->qvel[mecanum_wheels_[wheel_index].dof_address];
        working_state_.wheel_linear[wheel_index] =
            info_.mecanum_info.wheel_radius * wheel_angular[wheel_index];
    }
    mecanum_kinematics_->forward(
        wheel_angular, working_state_.base_linear, working_state_.base_angular);
    working_state_.wheel_angular = wheel_angular;
    return true;
}

bool MobileBaseComponent::write_mecanum_command(
    const mjContext& context, const MobileBaseCommand& command) {
    switch (command.mode) {
        case MobileBaseControlMode::Twist:
            return write_twist_command(context, command);
        case MobileBaseControlMode::WheelLinear:
            return write_wheel_linear_command(context, command);
        case MobileBaseControlMode::WheelAngular:
            return write_wheel_angular_command(context, command);
        default:
            LOG_ERROR << "mobile base '" << info_.mobile_base_name
                      << "' received unsupported mecanum control mode "
                      << static_cast<int>(command.mode) << ".";
            return false;
    }
}

bool MobileBaseComponent::configure_wheel(
    const mjContext& context, const WheelInfo& info, mjWheel& wheel) const {
    if (info.wheel_name.empty()) {
        LOG_ERROR << "mobile base '" << info_.mobile_base_name << "' wheel name must not be empty.";
        return false;
    }
    if (info.actuator_name.empty()) {
        LOG_ERROR << "mobile base '" << info_.mobile_base_name << "' wheel '" << info.wheel_name
                  << "' actuator name must not be empty.";
        return false;
    }
    if (!std::isfinite(info.damping) || info.damping <= 0.0) {
        LOG_ERROR << "mobile base '" << info_.mobile_base_name << "' wheel '" << info.wheel_name
                  << "' damping must be finite and positive.";
        return false;
    }

    mjWheel binding{};
    binding.wheel_id = mj_name2id(context.model, mjOBJ_JOINT, info.wheel_name.c_str());
    if (binding.wheel_id < 0) {
        LOG_ERROR << "mobile base '" << info_.mobile_base_name << "' wheel '" << info.wheel_name
                  << "' was not found.";
        return false;
    }
    if (context.model->jnt_type[binding.wheel_id] != mjJNT_HINGE) {
        LOG_ERROR << "mobile base '" << info_.mobile_base_name << "' wheel '" << info.wheel_name
                  << "' must be a MuJoCo hinge joint.";
        return false;
    }
    binding.dof_address = context.model->jnt_dofadr[binding.wheel_id];
    if (binding.dof_address < 0) {
        LOG_ERROR << "mobile base '" << info_.mobile_base_name << "' wheel '" << info.wheel_name
                  << "' has an invalid DOF address.";
        return false;
    }

    binding.actuator_id = mj_name2id(context.model, mjOBJ_ACTUATOR, info.actuator_name.c_str());
    if (binding.actuator_id < 0) {
        LOG_ERROR << "mobile base '" << info_.mobile_base_name << "' actuator '"
                  << info.actuator_name << "' was not found.";
        return false;
    }
    if (context.model->actuator_trntype[binding.actuator_id] != mjTRN_JOINT) {
        LOG_ERROR << "mobile base '" << info_.mobile_base_name << "' actuator '"
                  << info.actuator_name << "' must use joint transmission.";
        return false;
    }
    if (context.model->actuator_trnid[2 * binding.actuator_id] != binding.wheel_id) {
        LOG_ERROR << "mobile base '" << info_.mobile_base_name << "' actuator '"
                  << info.actuator_name << "' must drive wheel '" << info.wheel_name << "'.";
        return false;
    }
    if (context.model->actuator_dyntype[binding.actuator_id] != mjDYN_NONE) {
        LOG_ERROR << "mobile base '" << info_.mobile_base_name << "' actuator '"
                  << info.actuator_name << "' must not use internal dynamics.";
        return false;
    }
    if (context.model->actuator_gaintype[binding.actuator_id] != mjGAIN_FIXED) {
        LOG_ERROR << "mobile base '" << info_.mobile_base_name << "' actuator '"
                  << info.actuator_name << "' must use fixed gain.";
        return false;
    }
    const mjtNum* gain = context.model->actuator_gainprm + binding.actuator_id * mjNGAIN;
    if (!math::equal(static_cast<double>(gain[0]), 1.0)) {
        LOG_ERROR << "mobile base '" << info_.mobile_base_name << "' actuator '"
                  << info.actuator_name << "' gain must be 1.";
        return false;
    }
    if (context.model->actuator_biastype[binding.actuator_id] != mjBIAS_NONE) {
        LOG_ERROR << "mobile base '" << info_.mobile_base_name << "' actuator '"
                  << info.actuator_name << "' must not use bias.";
        return false;
    }
    const mjtNum* gear = context.model->actuator_gear + binding.actuator_id * 6;
    if (!math::equal(static_cast<double>(gear[0]), 1.0)) {
        LOG_ERROR << "mobile base '" << info_.mobile_base_name << "' actuator '"
                  << info.actuator_name << "' gear must be 1.";
        return false;
    }

    wheel = binding;
    return true;
}

bool MobileBaseComponent::write_twist_command(
    const mjContext& context, const MobileBaseCommand& command) {
    if (!std::isfinite(command.base_linear[0]) || !std::isfinite(command.base_linear[1]) ||
        !std::isfinite(command.base_angular[2])) {
        LOG_ERROR << "mobile base '" << info_.mobile_base_name
                  << "' twist command must contain finite x, y, and yaw values.";
        return false;
    }
    if (!mecanum_kinematics_) {
        LOG_ERROR << "mobile base '" << info_.mobile_base_name
                  << "' mecanum kinematics are not initialized.";
        return false;
    }
    Vector4d wheel_angular{};
    mecanum_kinematics_->inverse(command.base_linear, command.base_angular, wheel_angular);
    return write_wheel_commands(context, wheel_angular);
}

bool MobileBaseComponent::write_wheel_linear_command(
    const mjContext& context, const MobileBaseCommand& command) {
    Vector4d wheel_angular{};
    for (std::size_t wheel_index = 0; wheel_index < MecanumWheelCount; ++wheel_index) {
        if (!std::isfinite(command.wheel_linear[wheel_index])) {
            LOG_ERROR << "mobile base '" << info_.mobile_base_name
                      << "' wheel linear velocity must be finite.";
            return false;
        }
        wheel_angular[wheel_index] =
            command.wheel_linear[wheel_index] / info_.mecanum_info.wheel_radius;
    }
    return write_wheel_commands(context, wheel_angular);
}

bool MobileBaseComponent::write_wheel_angular_command(
    const mjContext& context, const MobileBaseCommand& command) {
    for (std::size_t wheel_index = 0; wheel_index < MecanumWheelCount; ++wheel_index) {
        if (!std::isfinite(command.wheel_angular[wheel_index])) {
            LOG_ERROR << "mobile base '" << info_.mobile_base_name
                      << "' wheel angular velocity must be finite.";
            return false;
        }
    }
    return write_wheel_commands(context, command.wheel_angular);
}

bool MobileBaseComponent::write_wheel_commands(
    const mjContext& context, const Vector4d& wheel_angular) {
    for (std::size_t wheel_index = 0; wheel_index < MecanumWheelCount; ++wheel_index) {
        const mjWheel& wheel = mecanum_wheels_[wheel_index];
        double effort = info_.mecanum_wheels[wheel_index].damping *
                        (wheel_angular[wheel_index] - context.data->qvel[wheel.dof_address]);
        effort = clamp_force_limits(context, wheel, effort);
        context.data->ctrl[wheel.actuator_id] = clamp_ctrl_limits(context, wheel, effort);
    }
    return true;
}

double MobileBaseComponent::clamp_ctrl_limits(
    const mjContext& context, const mjWheel& wheel, double value) const {
    if (context.model->actuator_ctrllimited[wheel.actuator_id] == 0) {
        return value;
    }
    const mjtNum* range = context.model->actuator_ctrlrange + 2 * wheel.actuator_id;
    return std::clamp(value, static_cast<double>(range[0]), static_cast<double>(range[1]));
}

double MobileBaseComponent::clamp_force_limits(
    const mjContext& context, const mjWheel& wheel, double value) const {
    if (context.model->actuator_forcelimited[wheel.actuator_id] == 0) {
        return value;
    }
    const mjtNum* range = context.model->actuator_forcerange + 2 * wheel.actuator_id;
    return std::clamp(value, static_cast<double>(range[0]), static_cast<double>(range[1]));
}

void MobileBaseComponent::reset_odometry() {
    working_state_.pose = {0.0, 0.0, 0.0};
    working_state_.base_linear = {0.0, 0.0, 0.0};
    working_state_.base_angular = {0.0, 0.0, 0.0};
    working_state_.wheel_linear = {};
    working_state_.wheel_angular = {};
    working_state_.timestamp = 0.0;
}

bool MobileBaseComponent::update_ground_truth_pose(const mjData& data) {
    if (base_body_id_ < 0) {
        LOG_ERROR << "mobile base '" << info_.mobile_base_name << "' body binding is missing.";
        return false;
    }
    const mjtNum* xpos = data.xpos + 3 * base_body_id_;
    const mjtNum* xmat = data.xmat + 9 * base_body_id_;
    working_state_.pose = {
        static_cast<double>(xpos[0]), static_cast<double>(xpos[1]),
        wrap_angle(std::atan2(static_cast<double>(xmat[3]), static_cast<double>(xmat[0])))};
    return true;
}

double MobileBaseComponent::wrap_angle(double angle) {
    while (math::greater(angle, Pi)) {
        angle -= 2.0 * Pi;
    }
    while (math::less(angle, -Pi)) {
        angle += 2.0 * Pi;
    }
    return angle;
}

}  // namespace mujoco_simulation
