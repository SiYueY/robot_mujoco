#include "component/mobile_base/mecanum/mecanum_mobile_base.hpp"

#include <cmath>
#include <unordered_set>

#include "log/logging.hpp"

namespace mujoco_simulation {

MecanumMobileBase::MecanumMobileBase(const MecanumInfo& info) : info_(info), kinematics_(info) {}

bool MecanumMobileBase::init(const mjContext& context, const MecanumWheelInfo& wheels) {
    initialized_ = false;
    wheels_ = {};
    std::unordered_set<int> joint_ids;
    for (std::size_t index = 0; index < MecanumWheelCount; ++index) {
        if (!std::isfinite(wheels[index].speed_response) || wheels[index].speed_response < 0.0) {
            SIM_ERROR << "mecanum wheel '" << wheels[index].wheel_name
                      << "' speed_response must be finite and non-negative.";
            return false;
        }
        if (!configure_wheel(context, wheels[index], wheels_[index])) return false;
        if (!joint_ids.insert(wheels_[index].joint_id).second) {
            SIM_ERROR << "mecanum wheel joints must be unique.";
            return false;
        }
    }
    return reset(context) && (initialized_ = true);
}

bool MecanumMobileBase::reset(const mjContext& context) {
    if (!context.valid()) return false;
    for (Wheel& wheel : wheels_) {
        wheel.target = 0.0;
        wheel.feedback = 0.0;
        wheel.position = context.model->qpos0[wheel.qpos_address];
        context.data->qpos[wheel.qpos_address] = wheel.position;
        context.data->qvel[wheel.dof_address] = 0.0;
    }
    base_linear_ = {};
    base_angular_ = {};
    wheel_angular_ = {};
    wheel_linear_ = {};
    return true;
}

bool MecanumMobileBase::write(const MobileBaseCommand& command) {
    if (!initialized_) return false;
    Vector4d target{};
    if (!target_from_command(command, target)) return false;
    for (std::size_t index = 0; index < MecanumWheelCount; ++index)
        wheels_[index].target = target[index];
    return true;
}

bool MecanumMobileBase::advance(const mjContext& context) {
    if (!initialized_ || !context.valid()) return false;
    const double dt = context.model->opt.timestep;
    if (!std::isfinite(dt) || dt <= 0.0) return false;
    for (std::size_t index = 0; index < MecanumWheelCount; ++index) {
        Wheel& wheel = wheels_[index];
        if (wheel.speed_response == 0.0)
            wheel.feedback = wheel.target;
        else
            wheel.feedback +=
                (1.0 - std::exp(-dt / wheel.speed_response)) * (wheel.target - wheel.feedback);
        wheel_angular_[index] = wheel.feedback;
        wheel_linear_[index] = wheel.radius * wheel.feedback;
        wheel.position += wheel.feedback * dt;
        context.data->qvel[wheel.dof_address] = wheel.feedback;
        context.data->qpos[wheel.qpos_address] = wheel.position;
    }
    Vector4d canonical_wheel_linear{};
    for (std::size_t index = 0; index < MecanumWheelCount; ++index)
        canonical_wheel_linear[index] = wheels_[index].direction * wheel_linear_[index];
    kinematics_.forward(canonical_wheel_linear, base_linear_, base_angular_);
    return true;
}

bool MecanumMobileBase::configure_wheel(
    const mjContext& context, const WheelInfo& info, Wheel& wheel) const {
    if (!std::isfinite(info.radius) || info.radius <= 0.0 ||
        (info.direction != -1.0 && info.direction != 1.0)) {
        SIM_ERROR << "mecanum wheel '" << info.wheel_name
                  << "' requires a positive radius and direction of -1 or 1.";
        return false;
    }
    wheel.joint_id = mj_name2id(context.model, mjOBJ_JOINT, info.wheel_name.c_str());
    if (wheel.joint_id < 0 || context.model->jnt_type[wheel.joint_id] != mjJNT_HINGE) {
        SIM_ERROR << "mecanum wheel '" << info.wheel_name << "' must be a MuJoCo hinge joint.";
        return false;
    }
    if (context.model->jnt_limited[wheel.joint_id] != 0) {
        SIM_ERROR << "mecanum wheel '" << info.wheel_name << "' must be an unlimited hinge joint.";
        return false;
    }
    wheel.qpos_address = context.model->jnt_qposadr[wheel.joint_id];
    wheel.dof_address = context.model->jnt_dofadr[wheel.joint_id];
    wheel.radius = info.radius;
    wheel.direction = info.direction;
    wheel.speed_response = info.speed_response;
    return wheel.qpos_address >= 0 && wheel.dof_address >= 0;
}

bool MecanumMobileBase::target_from_command(
    const MobileBaseCommand& command, Vector4d& target) const {
    switch (command.mode) {
        case MobileBaseControlMode::Twist:
            if (!std::isfinite(command.base_linear[0]) || !std::isfinite(command.base_linear[1]) ||
                !std::isfinite(command.base_angular[2]))
                return false;
            kinematics_.inverse(command.base_linear, command.base_angular, target);
            for (std::size_t index = 0; index < MecanumWheelCount; ++index)
                target[index] = wheels_[index].direction * target[index] / wheels_[index].radius;
            return true;
        case MobileBaseControlMode::WheelLinear:
            for (std::size_t index = 0; index < MecanumWheelCount; ++index) {
                if (!std::isfinite(command.wheel_linear[index])) return false;
                target[index] = command.wheel_linear[index] / wheels_[index].radius;
            }
            return true;
        case MobileBaseControlMode::WheelAngular:
            for (const double value : command.wheel_angular)
                if (!std::isfinite(value)) return false;
            target = command.wheel_angular;
            return true;
        default:
            return false;
    }
}

}  // namespace mujoco_simulation
