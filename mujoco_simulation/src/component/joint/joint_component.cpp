#include "component/joint/joint_component.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "common/logging.hpp"
#include "common/macro.hpp"
#include "mujoco_simulation/common/math.hpp"

namespace mujoco_simulation {

JointComponent::JointComponent(JointInfo info)
    : SimulationComponent(info.joint_name, info.period),
      info_(std::move(info)) {}

bool JointComponent::init(const mjContext &context) {
  initialized_ = false;
  if (!configure(context)) {
    return false;
  }

  if (info_.actuator_name.empty()) {
    LOG_ERROR << "joint '" << info_.joint_name
              << "' actuator name must not be empty.";
    return false;
  }
  if (info_.position_limits.min > info_.position_limits.max) {
    LOG_ERROR << "joint '" << info_.joint_name
              << "' position limits have min greater than max.";
    return false;
  }
  if (info_.velocity_limits.min > info_.velocity_limits.max) {
    LOG_ERROR << "joint '" << info_.joint_name
              << "' velocity limits have min greater than max.";
    return false;
  }
  if (info_.effort_limits.min > info_.effort_limits.max) {
    LOG_ERROR << "joint '" << info_.joint_name
              << "' effort limits have min greater than max.";
    return false;
  }
  if (!std::isfinite(info_.position_stiffness) ||
      info_.position_stiffness < 0.0) {
    LOG_ERROR << "joint '" << info_.joint_name
              << "' position stiffness must be finite and non-negative.";
    return false;
  }
  if (!std::isfinite(info_.position_damping) || info_.position_damping < 0.0) {
    LOG_ERROR << "joint '" << info_.joint_name
              << "' position damping must be finite and non-negative.";
    return false;
  }
  if (!std::isfinite(info_.velocity_damping) || info_.velocity_damping < 0.0) {
    LOG_ERROR << "joint '" << info_.joint_name
              << "' velocity damping must be finite and non-negative.";
    return false;
  }

  const mjModel &model = *context.model;
  joint_.joint_id = mj_name2id(&model, mjOBJ_JOINT, info_.joint_name.c_str());
  if (joint_.joint_id < 0) {
    LOG_ERROR << "joint '" << info_.joint_name
              << "' was not found in the model.";
    return false;
  }
  joint_.actuator_id =
      mj_name2id(&model, mjOBJ_ACTUATOR, info_.actuator_name.c_str());
  if (joint_.actuator_id < 0) {
    LOG_ERROR << "joint '" << info_.joint_name << "' actuator '"
              << info_.actuator_name << "' was not found in the model.";
    return false;
  }
  const int mujoco_joint_type = model.jnt_type[joint_.joint_id];
  if (mujoco_joint_type == mjJNT_HINGE) {
    info_.joint_type = JointType::Revolute;
  } else if (mujoco_joint_type == mjJNT_SLIDE) {
    info_.joint_type = JointType::Prismatic;
  } else {
    LOG_ERROR << "joint '" << info_.joint_name << "' has MuJoCo type "
              << mujoco_joint_type
              << "; only hinge and slide joints are supported.";
    return false;
  }
  joint_.qpos_address = model.jnt_qposadr[joint_.joint_id];
  if (joint_.qpos_address < 0) {
    LOG_ERROR << "joint '" << info_.joint_name
              << "' has an invalid qpos address.";
    return false;
  }
  joint_.dof_address = model.jnt_dofadr[joint_.joint_id];
  if (joint_.dof_address < 0) {
    LOG_ERROR << "joint '" << info_.joint_name
              << "' has an invalid dof address.";
    return false;
  }
  if (model.actuator_trntype[joint_.actuator_id] != mjTRN_JOINT) {
    LOG_ERROR << "joint '" << info_.joint_name << "' actuator '"
              << info_.actuator_name << "' has transmission type "
              << model.actuator_trntype[joint_.actuator_id]
              << ", expected joint transmission.";
    return false;
  }
  if (model.actuator_trnid[2 * joint_.actuator_id] != joint_.joint_id) {
    LOG_ERROR << "joint '" << info_.joint_name << "' actuator '"
              << info_.actuator_name << "' is bound to joint id "
              << model.actuator_trnid[2 * joint_.actuator_id]
              << ", expected id " << joint_.joint_id << ".";
    return false;
  }
  if (model.actuator_dyntype[joint_.actuator_id] != mjDYN_NONE) {
    LOG_ERROR << "joint '" << info_.joint_name << "' actuator '"
              << info_.actuator_name << "' has dynamics type "
              << model.actuator_dyntype[joint_.actuator_id]
              << ", expected no dynamics.";
    return false;
  }
  if (model.actuator_gaintype[joint_.actuator_id] != mjGAIN_FIXED) {
    LOG_ERROR << "joint '" << info_.joint_name << "' actuator '"
              << info_.actuator_name << "' has gain type "
              << model.actuator_gaintype[joint_.actuator_id]
              << ", expected fixed gain.";
    return false;
  }
  const mjtNum *gain = model.actuator_gainprm + joint_.actuator_id * mjNGAIN;
  if (!equal(static_cast<double>(gain[0]), 1.0)) {
    LOG_ERROR << "joint '" << info_.joint_name << "' actuator '"
              << info_.actuator_name << "' has gain " << gain[0]
              << ", expected 1.";
    return false;
  }
  if (model.actuator_biastype[joint_.actuator_id] != mjBIAS_NONE) {
    LOG_ERROR << "joint '" << info_.joint_name << "' actuator '"
              << info_.actuator_name << "' has bias type "
              << model.actuator_biastype[joint_.actuator_id]
              << ", expected no bias.";
    return false;
  }
  const mjtNum *gear = model.actuator_gear + joint_.actuator_id * 6;
  if (!equal(static_cast<double>(gear[0]), 1.0)) {
    LOG_ERROR << "joint '" << info_.joint_name << "' actuator '"
              << info_.actuator_name << "' has gear " << gear[0]
              << ", expected 1.";
    return false;
  }
  initialized_ = true;
  return true;
}

bool JointComponent::reset(const mjContext &context) {
  mjData &data = *context.data;
  if (!is_initialized()) {
    LOG_ERROR << "joint '" << info_.joint_name << "' is not initialized.";
    return false;
  }
  data.ctrl[joint_.actuator_id] = 0.0;
  command_ = {};
  state_.reset();
  return true;
}

bool JointComponent::update(const mjContext &context) {
  if (!is_initialized()) {
    LOG_ERROR << "joint '" << info_.joint_name << "' is not initialized.";
    return false;
  }

  auto state = std::make_shared<JointState>();
  state->timestamp = context.data->time;
  state->position = context.data->qpos[joint_.qpos_address];
  state->velocity = context.data->qvel[joint_.dof_address];
  state->effort = context.data->qfrc_actuator[joint_.dof_address];
  state->mode = command_.mode;
  state_ = std::move(state);
  return true;
}

bool JointComponent::write(const mjContext &context,
                           const JointCommand &command) {
  if (!is_initialized()) {
    LOG_ERROR << "joint '" << info_.joint_name << "' is not initialized.";
    return false;
  }

  bool result = false;
  switch (command.mode) {
  case JointControlMode::Position:
    result = write_position_command(context, command);
    break;
  case JointControlMode::Velocity:
    result = write_velocity_command(context, command);
    break;
  case JointControlMode::Effort:
    result = write_effort_command(context, command);
    break;
  case JointControlMode::Hybrid:
    result = write_hybrid_command(context, command);
    break;
  default:
    LOG_ERROR << "joint '" << info_.joint_name
              << "' received unsupported control mode "
              << static_cast<int>(command.mode) << ".";
    return false;
  }
  if (!result) {
    return false;
  }

  command_ = command;
  return true;
}

bool JointComponent::read_state(
    std::shared_ptr<const JointState> &state) const {
  if (!is_initialized()) {
    LOG_ERROR << "joint '" << info_.joint_name << "' is not initialized.";
    return false;
  }
  state = state_;
  return state != nullptr;
}

bool JointComponent::read(const mjContext &context, JointState &state) const {
  UNUSED(context);
  std::shared_ptr<const JointState> snapshot;
  if (!read_state(snapshot)) {
    return false;
  }
  state = *snapshot;
  return true;
}

bool JointComponent::is_initialized() const noexcept { return initialized_; }

bool JointComponent::write_position_command(const mjContext &context,
                                            const JointCommand &command) const {
  if (!std::isfinite(command.position)) {
    LOG_ERROR << "joint '" << info_.joint_name
              << "' position command must be finite.";
    return false;
  }

  const double position_state = context.data->qpos[joint_.qpos_address];
  const double velocity_state = context.data->qvel[joint_.dof_address];
  double effort = info_.position_stiffness *
                      (clamp_limits(info_.position_limits, command.position) -
                       position_state) +
                  info_.position_damping * (0.0 - velocity_state);
  effort = clamp_limits(info_.effort_limits, effort);
  effort = clamp_force_limits(context, effort);
  context.data->ctrl[joint_.actuator_id] = clamp_ctrl_limits(context, effort);
  return true;
}

bool JointComponent::write_velocity_command(const mjContext &context,
                                            const JointCommand &command) const {
  if (!std::isfinite(command.velocity)) {
    LOG_ERROR << "joint '" << info_.joint_name
              << "' velocity command must be finite.";
    return false;
  }
  const double velocity_state = context.data->qvel[joint_.dof_address];
  double effort =
      info_.velocity_damping *
      (clamp_limits(info_.velocity_limits, command.velocity) - velocity_state);
  effort = clamp_limits(info_.effort_limits, effort);
  effort = clamp_force_limits(context, effort);
  context.data->ctrl[joint_.actuator_id] = clamp_ctrl_limits(context, effort);
  return true;
}

bool JointComponent::write_effort_command(const mjContext &context,
                                          const JointCommand &command) const {
  if (!std::isfinite(command.effort)) {
    LOG_ERROR << "joint '" << info_.joint_name
              << "' effort command must be finite.";
    return false;
  }
  double effort = clamp_limits(info_.effort_limits, command.effort);
  effort = clamp_force_limits(context, effort);
  context.data->ctrl[joint_.actuator_id] = clamp_ctrl_limits(context, effort);
  return true;
}

bool JointComponent::write_hybrid_command(const mjContext &context,
                                          const JointCommand &command) const {
  if (!std::isfinite(command.position) || !std::isfinite(command.velocity) ||
      !std::isfinite(command.effort) || !std::isfinite(command.stiffness) ||
      !std::isfinite(command.damping)) {
    LOG_ERROR << "joint '" << info_.joint_name
              << "' hybrid command values must be finite.";
    return false;
  }
  if (command.stiffness < 0.0) {
    LOG_ERROR << "joint '" << info_.joint_name
              << "' hybrid stiffness must be non-negative.";
    return false;
  }
  if (command.damping < 0.0) {
    LOG_ERROR << "joint '" << info_.joint_name
              << "' hybrid damping must be non-negative.";
    return false;
  }
  const double position_state = context.data->qpos[joint_.qpos_address];
  const double velocity_state = context.data->qvel[joint_.dof_address];
  double effort =
      command.effort +
      command.stiffness *
          (clamp_limits(info_.position_limits, command.position) -
           position_state) +
      command.damping * (clamp_limits(info_.velocity_limits, command.velocity) -
                         velocity_state);
  effort = clamp_limits(info_.effort_limits, effort);
  effort = clamp_force_limits(context, effort);
  context.data->ctrl[joint_.actuator_id] = clamp_ctrl_limits(context, effort);
  return true;
}

double JointComponent::clamp_limits(const Limit &limits, double value) const {
  return std::clamp(value, limits.min, limits.max);
}

double JointComponent::clamp_ctrl_limits(const mjContext &context,
                                         double value) const {
  if (context.model->actuator_ctrllimited[joint_.actuator_id] == 0) {
    return value;
  }
  const mjtNum *range =
      context.model->actuator_ctrlrange + 2 * joint_.actuator_id;
  return std::clamp(value, static_cast<double>(range[0]),
                    static_cast<double>(range[1]));
}

double JointComponent::clamp_force_limits(const mjContext &context,
                                          double value) const {
  if (context.model->actuator_forcelimited[joint_.actuator_id] == 0) {
    return value;
  }
  const mjtNum *range =
      context.model->actuator_forcerange + 2 * joint_.actuator_id;
  return std::clamp(value, static_cast<double>(range[0]),
                    static_cast<double>(range[1]));
}

} // namespace mujoco_simulation
