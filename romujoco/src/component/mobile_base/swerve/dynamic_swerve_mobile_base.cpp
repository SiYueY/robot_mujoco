#include "component/mobile_base/swerve/dynamic_swerve_mobile_base.hpp"

#include <cmath>
#include <utility>

namespace romujoco {
namespace {

bool is_joint_actuator(const mjModel& model, int actuator, int joint) {
    return actuator >= 0 && model.actuator_trntype[actuator] == mjTRN_JOINT &&
           model.actuator_trnid[2 * actuator] == joint &&
           model.actuator_dyntype[actuator] == mjDYN_NONE &&
           model.actuator_gaintype[actuator] == mjGAIN_FIXED;
}

bool has_position_target_semantics(const mjModel& model, int actuator) {
    const mjtNum* bias = model.actuator_biasprm + actuator * mjNBIAS;
    return model.actuator_biastype[actuator] == mjBIAS_AFFINE && bias[1] != 0.0;
}

bool has_velocity_target_semantics(const mjModel& model, int actuator) {
    const mjtNum* bias = model.actuator_biasprm + actuator * mjNBIAS;
    return model.actuator_biastype[actuator] == mjBIAS_AFFINE && bias[2] != 0.0;
}

Vector3d inverse_rotate(const Quaterniond& q, const Vector3d& value) {
    const double w = q[0], x = -q[1], y = -q[2], z = -q[3];
    const Vector3d t{
        2.0 * (y * value[2] - z * value[1]), 2.0 * (z * value[0] - x * value[2]),
        2.0 * (x * value[1] - y * value[0])};
    return {
        value[0] + w * t[0] + y * t[2] - z * t[1], value[1] + w * t[1] + z * t[0] - x * t[2],
        value[2] + w * t[2] + x * t[1] - y * t[0]};
}

}  // namespace

DynamicSwerveMobileBase::DynamicSwerveMobileBase(SwerveMobileBaseInfo info)
: MobileBaseComponent(info.common.name, info.common.period), info_(std::move(info)), ik_([&] {
      std::vector<SwerveModuleGeometry> geometry;
      geometry.reserve(info_.modules.size());
      for (const SwerveModuleInfo& module : info_.modules)
          geometry.push_back({module.position_x, module.position_y, module.wheel_radius});
      return geometry;
  }()) {}

bool DynamicSwerveMobileBase::init(const mjContext& context) {
    if (info_.common.execution_mode != MobileBaseExecutionMode::Dynamic || !configure(context))
        return false;
    const int body = mj_name2id(context.model, mjOBJ_BODY, info_.common.base_body_name.c_str());
    int free_joint = -1;
    for (int joint = 0; body >= 0 && joint < context.model->njnt; ++joint) {
        if (context.model->jnt_bodyid[joint] == body &&
            context.model->jnt_type[joint] == mjJNT_FREE) {
            if (free_joint >= 0) return false;
            free_joint = joint;
        }
    }
    if (free_joint < 0) return false;
    base_qpos_ = context.model->jnt_qposadr[free_joint];
    base_dof_ = context.model->jnt_dofadr[free_joint];
    modules_.clear();
    modules_.reserve(info_.modules.size());
    for (const SwerveModuleInfo& info : info_.modules) {
        Module module;
        module.steering_joint =
            mj_name2id(context.model, mjOBJ_JOINT, info.steering_joint_name.c_str());
        module.drive_joint = mj_name2id(context.model, mjOBJ_JOINT, info.drive_joint_name.c_str());
        module.steering_actuator =
            mj_name2id(context.model, mjOBJ_ACTUATOR, info.steering_actuator_name.c_str());
        module.drive_actuator =
            mj_name2id(context.model, mjOBJ_ACTUATOR, info.drive_actuator_name.c_str());
        if (module.steering_joint < 0 || module.drive_joint < 0 ||
            context.model->jnt_type[module.steering_joint] != mjJNT_HINGE ||
            context.model->jnt_type[module.drive_joint] != mjJNT_HINGE ||
            !is_joint_actuator(*context.model, module.steering_actuator, module.steering_joint) ||
            !is_joint_actuator(*context.model, module.drive_actuator, module.drive_joint) ||
            !has_position_target_semantics(*context.model, module.steering_actuator) ||
            !has_velocity_target_semantics(*context.model, module.drive_actuator))
            return false;
        module.steering_qpos = context.model->jnt_qposadr[module.steering_joint];
        module.steering_dof = context.model->jnt_dofadr[module.steering_joint];
        module.drive_qpos = context.model->jnt_qposadr[module.drive_joint];
        module.drive_dof = context.model->jnt_dofadr[module.drive_joint];
        modules_.push_back(module);
    }
    feedback_.assign(modules_.size(), 0.0);
    targets_.assign(modules_.size(), {});
    ready_ = true;
    return reset(context);
}

bool DynamicSwerveMobileBase::reset(const mjContext& context) { return ready_ && update(context); }

bool DynamicSwerveMobileBase::write(const mjContext& context, const MobileBaseCommand& command) {
    if (!ready_ || command.id != info_.common.id || !std::isfinite(command.velocity.linear_x) ||
        !std::isfinite(command.velocity.linear_y) || !std::isfinite(command.velocity.angular_z))
        return false;
    for (std::size_t index = 0; index < modules_.size(); ++index)
        feedback_[index] = context.data->qpos[modules_[index].steering_qpos];
    if (!ik_.inverse(command.velocity, feedback_, targets_)) return false;
    for (std::size_t index = 0; index < modules_.size(); ++index) {
        context.data->ctrl[modules_[index].steering_actuator] = targets_[index].steering_position;
        context.data->ctrl[modules_[index].drive_actuator] = targets_[index].drive_velocity;
    }
    return true;
}

bool DynamicSwerveMobileBase::update(const mjContext& context) {
    if (!ready_) return false;
    const mjtNum* qpos = context.data->qpos + base_qpos_;
    const mjtNum* qvel = context.data->qvel + base_dof_;
    working_.id = info_.common.id;
    working_.timestamp = context.data->time;
    working_.pose.position = {qpos[0], qpos[1], qpos[2]};
    working_.pose.orientation = {qpos[3], qpos[4], qpos[5], qpos[6]};
    working_.twist.linear = inverse_rotate(working_.pose.orientation, {qvel[0], qvel[1], qvel[2]});
    // MuJoCo stores free-joint angular velocity in the joint's local frame.
    working_.twist.angular = {qvel[3], qvel[4], qvel[5]};
    state_ = std::make_shared<MobileBaseState>(working_);
    return true;
}

bool DynamicSwerveMobileBase::read_state(std::shared_ptr<const MobileBaseState>& state) const {
    state = state_;
    return ready_ && state != nullptr;
}
}  // namespace romujoco
