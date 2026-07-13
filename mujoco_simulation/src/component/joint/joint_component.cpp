#include "mujoco_simulation/component/joint/joint_component.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "mujoco_simulation/component/update_context.hpp"

namespace mujoco_simulation {
namespace {

constexpr int kActuatorParamCount = 10;

bool is_limited(unsigned char limit_flag) { return limit_flag != 0; }

double clamp_if_needed(double min_value, double max_value, double value) {
  return std::clamp(value, min_value, max_value);
}

}  // namespace

JointComponent::JointComponent(JointConfig config) : config_(std::move(config)) {
  set_update_every_step();
}

std::string JointComponent::name() const noexcept { return config_.name; }

ResultCode JointComponent::bind(const mjModel& model) {
  if (config_.name.empty()) {
    return ResultCode::InvalidArgument;
  }

  binding_ = {};
  binding_.joint_id = mj_name2id(&model, mjOBJ_JOINT, config_.name.c_str());
  if (binding_.joint_id < 0) {
    return ResultCode::BindingFailed;
  }

  binding_.joint_type = model.jnt_type[binding_.joint_id];
  binding_.qpos_address = model.jnt_qposadr[binding_.joint_id];
  binding_.dof_address = model.jnt_dofadr[binding_.joint_id];
  if (binding_.qpos_address < 0 || binding_.dof_address < 0) {
    return ResultCode::BindingFailed;
  }

  binding_.actuator_id = find_actuator_id(model);
  if (!config_.actuator_name.empty() && binding_.actuator_id < 0) {
    return ResultCode::BindingFailed;
  }
  binding_.has_actuator = binding_.actuator_id >= 0;
  binding_.actuator_type = static_cast<int>(parse_actuator_type(model, binding_.actuator_id));

  ResultCode status = validate_binding();
  if (status != ResultCode::Ok) {
    return status;
  }
  return validate_command_configuration();
}

ResultCode JointComponent::reset(const mjModel& model, mjData& data) {
  (void)model;
  if (binding_.joint_id < 0 || binding_.dof_address < 0) {
    return ResultCode::FailedPrecondition;
  }

  last_command_ = {};
  if (binding_.has_actuator && binding_.actuator_id >= 0) {
    data.ctrl[binding_.actuator_id] = 0.0;
  }
  data.qfrc_applied[binding_.dof_address] = 0.0;
  state_ = {};
  state_.name = config_.name;
  return ResultCode::Ok;
}

ResultCode JointComponent::update(const UpdateContext& context) {
  return read(context.data, state_);
}

ResultCode JointComponent::write(const mjModel& model, mjData& data, const JointCommand& command) {
  if (binding_.joint_id < 0) {
    return ResultCode::FailedPrecondition;
  }

  last_command_ = command;
  switch (config_.command_mode) {
    case CommandInterfaceType::None:
      return ResultCode::Ok;
    case CommandInterfaceType::Position:
    case CommandInterfaceType::Velocity:
      if (config_.controller_type == JointControllerType::SoftwarePd) {
        return write_software_pd_command(model, data, command);
      }
      return write_direct_command(model, data, command);
    case CommandInterfaceType::Effort:
      return write_direct_command(model, data, command);
  }

  return ResultCode::Internal;
}

ResultCode JointComponent::read(const mjData& data, JointState& state) const {
  if (binding_.joint_id < 0 || binding_.qpos_address < 0 || binding_.dof_address < 0) {
    return ResultCode::FailedPrecondition;
  }

  state.name = config_.name;
  state.position = data.qpos[binding_.qpos_address];
  state.velocity = data.qvel[binding_.dof_address];
  state.effort = data.qfrc_actuator[binding_.dof_address];
  return ResultCode::Ok;
}

JointType JointComponent::joint_type() const noexcept {
  return parse_joint_type(binding_.joint_type);
}

ActuatorType JointComponent::actuator_type() const noexcept {
  return static_cast<ActuatorType>(binding_.actuator_type);
}

ResultCode JointComponent::validate_binding() const {
  const JointType joint_kind = parse_joint_type(binding_.joint_type);
  if (joint_kind != JointType::Hinge && joint_kind != JointType::Slide) {
    return ResultCode::ModelValidationFailed;
  }
  return ResultCode::Ok;
}

ResultCode JointComponent::validate_command_configuration() const {
  const ActuatorType actuator_kind = actuator_type();
  switch (config_.command_mode) {
    case CommandInterfaceType::None:
      return ResultCode::Ok;
    case CommandInterfaceType::Position:
      if (config_.controller_type == JointControllerType::MuJoCoActuator) {
        if (actuator_kind != ActuatorType::Position) {
          return ResultCode::ModelValidationFailed;
        }
      } else {
        if (actuator_kind == ActuatorType::Position || actuator_kind == ActuatorType::Velocity) {
          return ResultCode::ModelValidationFailed;
        }
        if (!finite(config_.position_kp) || config_.position_kp <= 0.0) {
          return ResultCode::InvalidArgument;
        }
        if (!finite(config_.velocity_kd) || config_.velocity_kd < 0.0) {
          return ResultCode::InvalidArgument;
        }
      }
      return ResultCode::Ok;
    case CommandInterfaceType::Velocity:
      if (config_.controller_type == JointControllerType::MuJoCoActuator) {
        if (actuator_kind != ActuatorType::Velocity) {
          return ResultCode::ModelValidationFailed;
        }
      } else {
        if (actuator_kind == ActuatorType::Position || actuator_kind == ActuatorType::Velocity) {
          return ResultCode::ModelValidationFailed;
        }
        if (!finite(config_.velocity_kd) || config_.velocity_kd <= 0.0) {
          return ResultCode::InvalidArgument;
        }
      }
      return ResultCode::Ok;
    case CommandInterfaceType::Effort:
      if (actuator_kind == ActuatorType::Position || actuator_kind == ActuatorType::Velocity) {
        return ResultCode::ModelValidationFailed;
      }
      return ResultCode::Ok;
  }

  return ResultCode::Internal;
}

ResultCode JointComponent::write_direct_command(const mjModel& model, mjData& data,
                                                const JointCommand& command) {
  double value = 0.0;
  switch (config_.command_mode) {
    case CommandInterfaceType::Position:
      if (!finite(command.position)) {
        return ResultCode::InvalidArgument;
      }
      if (!binding_.has_actuator) {
        return ResultCode::FailedPrecondition;
      }
      value = clamp_actuator_control_limits(model, clamp_command_limits(command.position));
      data.ctrl[binding_.actuator_id] = value;
      return ResultCode::Ok;
    case CommandInterfaceType::Velocity:
      if (!finite(command.velocity)) {
        return ResultCode::InvalidArgument;
      }
      if (!binding_.has_actuator) {
        return ResultCode::FailedPrecondition;
      }
      value = clamp_actuator_control_limits(model, clamp_command_limits(command.velocity));
      data.ctrl[binding_.actuator_id] = value;
      return ResultCode::Ok;
    case CommandInterfaceType::Effort:
      if (!finite(command.effort)) {
        return ResultCode::InvalidArgument;
      }
      value = clamp_actuator_force_limits(model, clamp_command_limits(command.effort));
      return write_effort_output(model, data, value);
    case CommandInterfaceType::None:
      return ResultCode::Ok;
  }

  return ResultCode::Internal;
}

ResultCode JointComponent::write_software_pd_command(const mjModel& model, mjData& data,
                                                     const JointCommand& command) {
  if (!finite(command.effort)) {
    return ResultCode::InvalidArgument;
  }

  const double q = data.qpos[binding_.qpos_address];
  const double dq = data.qvel[binding_.dof_address];
  double effort = command.effort;

  if (config_.command_mode == CommandInterfaceType::Position) {
    if (!finite(command.position) || !finite(command.velocity)) {
      return ResultCode::InvalidArgument;
    }
    effort += config_.position_kp * (command.position - q);
    effort += config_.velocity_kd * (command.velocity - dq);
  } else if (config_.command_mode == CommandInterfaceType::Velocity) {
    if (!finite(command.velocity)) {
      return ResultCode::InvalidArgument;
    }
    effort += config_.velocity_kd * (command.velocity - dq);
  } else {
    return ResultCode::ModelValidationFailed;
  }

  effort = clamp_actuator_force_limits(model, clamp_command_limits(effort));
  return write_effort_output(model, data, effort);
}

ResultCode JointComponent::write_effort_output(const mjModel& model, mjData& data,
                                               double effort) const {
  if (binding_.has_actuator) {
    const ActuatorType actuator_kind = actuator_type();
    if (actuator_kind != ActuatorType::Motor && actuator_kind != ActuatorType::Custom) {
      return ResultCode::FailedPrecondition;
    }
    data.ctrl[binding_.actuator_id] = clamp_actuator_control_limits(model, effort);
    data.qfrc_applied[binding_.dof_address] = 0.0;
    return ResultCode::Ok;
  }

  data.qfrc_applied[binding_.dof_address] = effort;
  return ResultCode::Ok;
}

double JointComponent::clamp_command_limits(double value) const {
  return clamp_if_needed(config_.command_min, config_.command_max, value);
}

double JointComponent::clamp_actuator_control_limits(const mjModel& model, double value) const {
  if (!binding_.has_actuator) {
    return value;
  }
  if (!is_limited(model.actuator_ctrllimited[binding_.actuator_id])) {
    return value;
  }
  const mjtNum* range = model.actuator_ctrlrange + 2 * binding_.actuator_id;
  return std::clamp(value, static_cast<double>(range[0]), static_cast<double>(range[1]));
}

double JointComponent::clamp_actuator_force_limits(const mjModel& model, double value) const {
  if (!binding_.has_actuator) {
    return value;
  }
  if (!is_limited(model.actuator_forcelimited[binding_.actuator_id])) {
    return value;
  }
  const mjtNum* range = model.actuator_forcerange + 2 * binding_.actuator_id;
  return std::clamp(value, static_cast<double>(range[0]), static_cast<double>(range[1]));
}

bool JointComponent::finite(double value) { return std::isfinite(value); }

int JointComponent::find_actuator_id(const mjModel& model) const {
  if (!config_.actuator_name.empty()) {
    return mj_name2id(&model, mjOBJ_ACTUATOR, config_.actuator_name.c_str());
  }

  for (int actuator_id = 0; actuator_id < model.nu; ++actuator_id) {
    if (model.actuator_trntype[actuator_id] != mjTRN_JOINT) {
      continue;
    }
    if (model.actuator_trnid[2 * actuator_id] == binding_.joint_id) {
      return actuator_id;
    }
  }
  return -1;
}

JointType JointComponent::parse_joint_type(int mujoco_joint_type) {
  if (mujoco_joint_type == mjJNT_HINGE) {
    return JointType::Hinge;
  }
  if (mujoco_joint_type == mjJNT_SLIDE) {
    return JointType::Slide;
  }
  if (mujoco_joint_type == mjJNT_BALL) {
    return JointType::Ball;
  }
  if (mujoco_joint_type == mjJNT_FREE) {
    return JointType::Free;
  }
  return JointType::Unknown;
}

ActuatorType JointComponent::parse_actuator_type(const mjModel& model, int actuator_id) {
  if (actuator_id < 0) {
    return ActuatorType::Passive;
  }

  const int bias_type = model.actuator_biastype[actuator_id];
  const mjtNum* biasprm = model.actuator_biasprm + actuator_id * kActuatorParamCount;
  if (bias_type == mjBIAS_NONE) {
    return ActuatorType::Motor;
  }
  if (bias_type == mjBIAS_AFFINE && biasprm[1] != 0) {
    return ActuatorType::Position;
  }
  if (bias_type == mjBIAS_AFFINE && biasprm[1] == 0 && biasprm[2] != 0) {
    return ActuatorType::Velocity;
  }
  return ActuatorType::Custom;
}

}  // namespace mujoco_simulation
