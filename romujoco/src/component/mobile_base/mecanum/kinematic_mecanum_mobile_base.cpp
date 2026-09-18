#include "component/mobile_base/mecanum/kinematic_mecanum_mobile_base.hpp"

#include <cmath>
#include <unordered_set>
#include <utility>

namespace romujoco {

KinematicMecanumMobileBase::KinematicMecanumMobileBase(MecanumMobileBaseInfo info)
: MobileBaseComponent(info.common.name, info.common.period),
  info_(std::move(info)),
  kinematics_(info_) {}

bool KinematicMecanumMobileBase::bind_base_joint(const SimulationContext& context) {
    const int body_id = mj_name2id(context.model, mjOBJ_BODY, info_.common.base_body_name.c_str());
    if (body_id < 0) return false;
    int free_joint = -1;
    for (int joint = 0; joint < context.model->njnt; ++joint) {
        if (context.model->jnt_bodyid[joint] != body_id ||
            context.model->jnt_type[joint] != mjJNT_FREE)
            continue;
        if (free_joint >= 0) return false;
        free_joint = joint;
    }
    if (free_joint < 0) return false;
    base_qpos_ = context.model->jnt_qposadr[free_joint];
    base_dof_ = context.model->jnt_dofadr[free_joint];
    return true;
}

bool KinematicMecanumMobileBase::bind_wheel(
    const SimulationContext& context, const MecanumWheelInfo& wheel_info, Wheel& wheel) {
    wheel.joint_id = mj_name2id(context.model, mjOBJ_JOINT, wheel_info.joint_name.c_str());
    if (wheel.joint_id < 0 || context.model->jnt_type[wheel.joint_id] != mjJNT_HINGE ||
        context.model->jnt_limited[wheel.joint_id] != 0 || !std::isfinite(wheel_info.radius) ||
        wheel_info.radius <= 0.0 || !std::isfinite(wheel_info.speed_response) ||
        wheel_info.speed_response < 0.0 ||
        (wheel_info.direction != -1.0 && wheel_info.direction != 1.0))
        return false;
    wheel.qpos_address = context.model->jnt_qposadr[wheel.joint_id];
    wheel.dof_address = context.model->jnt_dofadr[wheel.joint_id];
    wheel.radius = wheel_info.radius;
    wheel.direction = wheel_info.direction;
    wheel.response = wheel_info.speed_response;
    return true;
}

bool KinematicMecanumMobileBase::init(const SimulationContext& context) {
    if (info_.common.execution_mode != MobileBaseExecutionMode::Kinematic || !configure(context) ||
        !bind_base_joint(context))
        return false;
    std::unordered_set<int> joint_ids;
    for (std::size_t index = 0; index < wheels_.size(); ++index) {
        if (!bind_wheel(context, info_.wheels[index], wheels_[index]) ||
            !joint_ids.insert(wheels_[index].joint_id).second)
            return false;
    }
    ready_ = true;
    return reset(context);
}

bool KinematicMecanumMobileBase::reset(const SimulationContext& context) {
    if (!ready_) return false;
    const mjtNum* reference = context.model->qpos0 + base_qpos_;
    if (std::abs(reference[4]) > 1e-10 || std::abs(reference[5]) > 1e-10) return false;
    for (int index = 0; index < 7; ++index)
        context.data->qpos[base_qpos_ + index] = reference[index];
    origin_ = {reference[0], reference[1], reference[2]};
    origin_yaw_ = 2.0 * std::atan2(reference[6], reference[3]);
    local_x_ = 0.0;
    local_y_ = 0.0;
    local_yaw_ = 0.0;
    linear_velocity_ = {};
    angular_velocity_ = {};
    for (Wheel& wheel : wheels_) {
        wheel.target = 0.0;
        wheel.feedback = 0.0;
        wheel.position = context.model->qpos0[wheel.qpos_address];
        context.data->qpos[wheel.qpos_address] = wheel.position;
        context.data->qvel[wheel.dof_address] = 0.0;
    }
    publish(context);
    state_ = std::make_shared<MobileBaseState>(working_);
    return true;
}

bool KinematicMecanumMobileBase::write(const SimulationContext&, const MobileBaseCommand& command) {
    if (!ready_ || command.id != info_.common.id || !std::isfinite(command.velocity.linear_x) ||
        !std::isfinite(command.velocity.linear_y) || !std::isfinite(command.velocity.angular_z))
        return false;
    Vector4d wheel_linear{};
    kinematics_.inverse(command.velocity, wheel_linear);
    for (std::size_t index = 0; index < wheels_.size(); ++index)
        wheels_[index].target =
            wheels_[index].direction * wheel_linear[index] / wheels_[index].radius;
    return true;
}

bool KinematicMecanumMobileBase::advance(const SimulationContext& context) {
    if (!ready_) return false;
    const double timestep = context.model->opt.timestep;
    if (!std::isfinite(timestep) || timestep <= 0.0) return false;
    Vector4d wheel_linear{};
    for (std::size_t index = 0; index < wheels_.size(); ++index) {
        Wheel& wheel = wheels_[index];
        wheel.feedback = wheel.response == 0.0
                             ? wheel.target
                             : wheel.feedback + (1.0 - std::exp(-timestep / wheel.response)) *
                                                    (wheel.target - wheel.feedback);
        wheel.position += wheel.feedback * timestep;
        context.data->qpos[wheel.qpos_address] = wheel.position;
        context.data->qvel[wheel.dof_address] = wheel.feedback;
        wheel_linear[index] = wheel.direction * wheel.radius * wheel.feedback;
    }
    kinematics_.forward(wheel_linear, linear_velocity_, angular_velocity_);
    local_yaw_ = std::remainder(local_yaw_ + angular_velocity_[2] * timestep, 2.0 * kPi);
    local_x_ +=
        (std::cos(local_yaw_) * linear_velocity_[0] - std::sin(local_yaw_) * linear_velocity_[1]) *
        timestep;
    local_y_ +=
        (std::sin(local_yaw_) * linear_velocity_[0] + std::cos(local_yaw_) * linear_velocity_[1]) *
        timestep;
    const double yaw = origin_yaw_ + local_yaw_;
    const double origin_cos = std::cos(origin_yaw_);
    const double origin_sin = std::sin(origin_yaw_);
    context.data->qpos[base_qpos_] = origin_[0] + origin_cos * local_x_ - origin_sin * local_y_;
    context.data->qpos[base_qpos_ + 1] = origin_[1] + origin_sin * local_x_ + origin_cos * local_y_;
    context.data->qpos[base_qpos_ + 2] = origin_[2];
    context.data->qpos[base_qpos_ + 3] = std::cos(yaw / 2.0);
    context.data->qpos[base_qpos_ + 4] = 0.0;
    context.data->qpos[base_qpos_ + 5] = 0.0;
    context.data->qpos[base_qpos_ + 6] = std::sin(yaw / 2.0);
    context.data->qvel[base_dof_] =
        std::cos(yaw) * linear_velocity_[0] - std::sin(yaw) * linear_velocity_[1];
    context.data->qvel[base_dof_ + 1] =
        std::sin(yaw) * linear_velocity_[0] + std::cos(yaw) * linear_velocity_[1];
    context.data->qvel[base_dof_ + 2] = 0.0;
    context.data->qvel[base_dof_ + 3] = 0.0;
    context.data->qvel[base_dof_ + 4] = 0.0;
    context.data->qvel[base_dof_ + 5] = angular_velocity_[2];
    return true;
}

void KinematicMecanumMobileBase::publish(const SimulationContext& context) {
    const double yaw = origin_yaw_ + local_yaw_;
    working_.id = info_.common.id;
    working_.timestamp = context.data->time;
    working_.pose.position = {
        context.data->qpos[base_qpos_], context.data->qpos[base_qpos_ + 1],
        context.data->qpos[base_qpos_ + 2]};
    working_.pose.orientation = {std::cos(yaw / 2.0), 0.0, 0.0, std::sin(yaw / 2.0)};
    working_.twist.linear = linear_velocity_;
    working_.twist.angular = angular_velocity_;
}

bool KinematicMecanumMobileBase::update(const SimulationContext& context) {
    if (!ready_) return false;
    publish(context);
    state_ = std::make_shared<MobileBaseState>(working_);
    return true;
}

bool KinematicMecanumMobileBase::read_state(std::shared_ptr<const MobileBaseState>& state) const {
    state = state_;
    return ready_ && state != nullptr;
}

}  // namespace romujoco
