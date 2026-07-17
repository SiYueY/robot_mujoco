#include "mujoco_simulation/component/joint/joint_component.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "mujoco_simulation/common/logging.hpp"
#include "mujoco_simulation/component/update_context.hpp"

namespace mujoco_simulation {
namespace {

bool is_limited(unsigned char limit_flag) { return limit_flag != 0; }

}  // namespace

JointComponent::JointComponent(JointInfo info) : info_(std::move(info)) { set_update_every_step(); }

std::string JointComponent::name() const noexcept { return info_.joint; }

bool JointComponent::bind(const mjModel& model) {
  if (info_.joint.empty() || info_.actuator.empty()) {
    LOG_ERROR << "JointComponent::bind"
              << ": "
              << "joint and actuator names must not be empty.";
    return false;
  }
  joint_id_ = mj_name2id(&model, mjOBJ_JOINT, info_.joint.c_str());
  if (joint_id_ < 0) {
    LOG_ERROR << "JointComponent::bind"
              << ": "
              << "joint was not found in model.";
    return false;
  }
  joint_type_ = model.jnt_type[joint_id_];
  qpos_address_ = model.jnt_qposadr[joint_id_];
  dof_address_ = model.jnt_dofadr[joint_id_];
  motor_id_ = find_motor_id(model);
  if (qpos_address_ < 0 || dof_address_ < 0 || motor_id_ < 0) {
    LOG_ERROR << "JointComponent::bind"
              << ": "
              << "failed to resolve MuJoCo joint or actuator addresses.";
    return false;
  }
  return validate_binding();
}

bool JointComponent::reset(const mjModel&, mjData& data) {
  if (joint_id_ < 0 || motor_id_ < 0) {
    LOG_ERROR << "JointComponent::reset"
              << ": "
              << "joint must be bound before reset.";
    return false;
  }
  data.ctrl[motor_id_] = 0.0;
  last_command_ = {};
  state_ = {};
  state_.joint = info_.joint;
  return true;
}

bool JointComponent::update(const UpdateContext& context) { return read(context.data, state_); }

bool JointComponent::write(const mjModel& model, mjData& data, const JointCommand& command) {
  if (joint_id_ < 0 || motor_id_ < 0 || command.joint != info_.joint) {
    LOG_ERROR << "JointComponent::write"
              << ": "
              << "joint is not bound or command target does not match.";
    return false;
  }
  double effort = 0.0;
  if (!calculate_effort(data, command, &effort)) {
    return false;
  }
  if (write_effort_output(model, data, effort)) {
    last_command_ = command;
    state_.mode = command.mode;
    return true;
  }
  return false;
}

bool JointComponent::read(const mjData& data, JointState& state) const {
  if (joint_id_ < 0 || qpos_address_ < 0 || dof_address_ < 0) {
    LOG_ERROR << "JointComponent::read"
              << ": "
              << "joint must be bound before read.";
    return false;
  }
  state.joint = info_.joint;
  state.mode = state_.mode;
  state.position = data.qpos[qpos_address_];
  state.velocity = data.qvel[dof_address_];
  state.effort = data.qfrc_actuator[dof_address_];
  return true;
}

JointType JointComponent::joint_type() const noexcept { return parse_joint_type(joint_type_); }

bool JointComponent::validate_binding() const {
  if (joint_type_ != mjJNT_HINGE && joint_type_ != mjJNT_SLIDE) {
    LOG_ERROR << "JointComponent::validate_binding"
              << ": "
              << "only hinge and slide joints are supported.";
    return false;
  }
  if (info_.position_limits.min > info_.position_limits.max ||
      info_.velocity_limits.min > info_.velocity_limits.max ||
      info_.effort_limits.min > info_.effort_limits.max || !finite(info_.position_stiffness) ||
      !finite(info_.position_damping) || !finite(info_.velocity_damping) ||
      info_.position_stiffness < 0.0 || info_.position_damping < 0.0 ||
      info_.velocity_damping < 0.0) {
    LOG_ERROR << "JointComponent::validate_binding"
              << ": "
              << "joint configuration contains invalid limits or gains.";
    return false;
  }
  return true;
}

bool JointComponent::calculate_effort(const mjData& data, const JointCommand& command,
                                      double* effort) const {
  if (effort == nullptr || !finite(command.position) || !finite(command.velocity) ||
      !finite(command.effort) || !finite(command.stiffness) || !finite(command.damping)) {
    LOG_ERROR << "JointComponent::calculate_effort"
              << ": "
              << "command values must be finite and effort pointer valid.";
    return false;
  }
  const double position = data.qpos[qpos_address_];
  const double velocity = data.qvel[dof_address_];
  switch (command.mode) {
    case ControlMode::Position:
      *effort = info_.position_stiffness *
                    (clamp_limits(info_.position_limits, command.position) - position) +
                info_.position_damping * (0.0 - velocity);
      break;
    case ControlMode::Velocity:
      *effort = info_.velocity_damping *
                (clamp_limits(info_.velocity_limits, command.velocity) - velocity);
      break;
    case ControlMode::Effort:
      *effort = clamp_limits(info_.effort_limits, command.effort);
      break;
    case ControlMode::Hybrid:
      if (command.stiffness < 0.0 || command.damping < 0.0) {
        LOG_ERROR << "JointComponent::calculate_effort"
                  << ": "
                  << "hybrid stiffness and damping must be non-negative.";
        return false;
      }
      *effort =
          clamp_limits(info_.effort_limits, command.effort) +
          command.stiffness * (clamp_limits(info_.position_limits, command.position) - position) +
          command.damping * (clamp_limits(info_.velocity_limits, command.velocity) - velocity);
      break;
  }
  *effort = clamp_limits(info_.effort_limits, *effort);
  return true;
}

bool JointComponent::write_effort_output(const mjModel& model, mjData& data, double effort) const {
  effort = clamp_motor_force_limits(model, effort);
  data.ctrl[motor_id_] = clamp_motor_control_limits(model, effort);
  return true;
}

double JointComponent::clamp_limits(const Limit& limits, double value) {
  return std::clamp(value, limits.min, limits.max);
}

double JointComponent::clamp_motor_control_limits(const mjModel& model, double value) const {
  if (!is_limited(model.actuator_ctrllimited[motor_id_])) {
    return value;
  }
  const mjtNum* range = model.actuator_ctrlrange + 2 * motor_id_;
  return std::clamp(value, static_cast<double>(range[0]), static_cast<double>(range[1]));
}

double JointComponent::clamp_motor_force_limits(const mjModel& model, double value) const {
  if (!is_limited(model.actuator_forcelimited[motor_id_])) {
    return value;
  }
  const mjtNum* range = model.actuator_forcerange + 2 * motor_id_;
  return std::clamp(value, static_cast<double>(range[0]), static_cast<double>(range[1]));
}

bool JointComponent::finite(double value) { return std::isfinite(value); }

int JointComponent::find_motor_id(const mjModel& model) const {
  const int actuator_id = mj_name2id(&model, mjOBJ_ACTUATOR, info_.actuator.c_str());
  if (actuator_id < 0 || model.actuator_trntype[actuator_id] != mjTRN_JOINT ||
      model.actuator_trnid[2 * actuator_id] != joint_id_ ||
      model.actuator_biastype[actuator_id] != mjBIAS_NONE) {
    return -1;
  }
  return actuator_id;
}

JointType JointComponent::parse_joint_type(int mujoco_joint_type) {
  if (mujoco_joint_type == mjJNT_HINGE) {
    return JointType::Revolute;
  }
  if (mujoco_joint_type == mjJNT_SLIDE) {
    return JointType::Prismatic;
  }
  return JointType::Revolute;
}

}  // namespace mujoco_simulation
