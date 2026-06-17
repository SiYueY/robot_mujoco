#include "mujoco_simulation/component/joint/joint_component.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace mujoco_simulation {
namespace {

constexpr int kActuatorParamCount = 10;

bool is_limited(unsigned char limit_flag) { return limit_flag != 0; }

double clamp_if_needed(const std::optional<double>& min_value,
                       const std::optional<double>& max_value, double value) {
  if (min_value.has_value()) {
    value = std::max(value, *min_value);
  }
  if (max_value.has_value()) {
    value = std::min(value, *max_value);
  }
  return value;
}

}  // namespace

JointComponent::JointComponent(JointConfig config) : config_(std::move(config)) {}

std::string_view JointComponent::name() const noexcept { return config_.name; }

Status JointComponent::bind(const mjModel& model) {
  if (config_.name.empty()) {
    return Status::invalid_argument("JointComponent name must not be empty.");
  }

  binding_ = {};
  binding_.joint_id = mj_name2id(&model, mjOBJ_JOINT, config_.name.c_str());
  if (binding_.joint_id < 0) {
    return Status::binding_failed("Joint '" + config_.name +
                                  "' could not bind to a MuJoCo joint with the same name.");
  }

  binding_.joint_type = model.jnt_type[binding_.joint_id];
  binding_.qpos_address = model.jnt_qposadr[binding_.joint_id];
  binding_.dof_address = model.jnt_dofadr[binding_.joint_id];
  if (binding_.qpos_address < 0 || binding_.dof_address < 0) {
    return Status::binding_failed("Joint '" + config_.name +
                                  "' has invalid MuJoCo qpos/dof addresses.");
  }

  binding_.actuator_id = find_actuator_id(model);
  if (!config_.actuator_name.empty() && binding_.actuator_id < 0) {
    return Status::binding_failed("Joint '" + config_.name +
                                  "' could not bind to MuJoCo actuator '" + config_.actuator_name +
                                  "'.");
  }
  binding_.has_actuator = binding_.actuator_id >= 0;
  binding_.actuator_type = static_cast<int>(parse_actuator_type(model, binding_.actuator_id));

  Status status = validate_binding();
  if (!status.ok()) {
    return status;
  }
  return validate_command_configuration();
}

Status JointComponent::reset(const mjModel& model, mjData& data) {
  (void)model;
  if (binding_.joint_id < 0 || binding_.dof_address < 0) {
    return Status::failed_precondition("JointComponent is not bound: " + config_.name);
  }

  last_command_ = {};
  if (binding_.has_actuator && binding_.actuator_id >= 0) {
    data.ctrl[binding_.actuator_id] = 0.0;
  }
  data.qfrc_applied[binding_.dof_address] = 0.0;
  return Status::Ok();
}

Status JointComponent::write(const mjModel& model, mjData& data, const JointCommand& command) {
  if (binding_.joint_id < 0) {
    return Status::failed_precondition("JointComponent is not bound: " + config_.name);
  }

  last_command_ = command;
  switch (config_.command_mode) {
    case CommandInterfaceType::None:
      return Status::Ok();
    case CommandInterfaceType::Position:
    case CommandInterfaceType::Velocity:
      if (config_.controller_type == JointControllerType::SoftwarePd) {
        return write_software_pd_command(model, data, command);
      }
      return write_direct_command(model, data, command);
    case CommandInterfaceType::Effort:
      return write_direct_command(model, data, command);
  }

  return Status::internal("Joint '" + config_.name +
                          "' uses an unsupported command mode during write().");
}

Status JointComponent::read(const mjData& data, JointState& state) const {
  if (binding_.joint_id < 0 || binding_.qpos_address < 0 || binding_.dof_address < 0) {
    return Status::failed_precondition("JointComponent is not bound: " + config_.name);
  }

  state.name = config_.name;
  state.position = data.qpos[binding_.qpos_address];
  state.velocity = data.qvel[binding_.dof_address];
  state.effort = data.qfrc_actuator[binding_.dof_address];
  return Status::Ok();
}

JointType JointComponent::joint_type() const noexcept {
  return parse_joint_type(binding_.joint_type);
}

ActuatorType JointComponent::actuator_type() const noexcept {
  return static_cast<ActuatorType>(binding_.actuator_type);
}

Status JointComponent::validate_binding() const {
  const JointType joint_kind = parse_joint_type(binding_.joint_type);
  if (joint_kind != JointType::Hinge && joint_kind != JointType::Slide) {
    return Status::model_validation_failed(
        "Joint '" + config_.name +
        "' uses an unsupported MuJoCo joint type; only 1-DoF hinge/slide joints are supported.");
  }
  return Status::Ok();
}

Status JointComponent::validate_command_configuration() const {
  const ActuatorType actuator_kind = actuator_type();
  switch (config_.command_mode) {
    case CommandInterfaceType::None:
      return Status::Ok();
    case CommandInterfaceType::Position:
      if (config_.controller_type == JointControllerType::MuJoCoActuator) {
        if (actuator_kind != ActuatorType::Position) {
          return Status::model_validation_failed(
              "Position command requires a MuJoCo position actuator for joint '" + config_.name +
              "'.");
        }
      } else {
        if (actuator_kind == ActuatorType::Position || actuator_kind == ActuatorType::Velocity) {
          return Status::model_validation_failed(
              "Software PD position control is only supported with motor/custom/passive actuators "
              "for joint '" +
              config_.name + "'.");
        }
        if (!finite(config_.position_kp) || config_.position_kp <= 0.0) {
          return Status::invalid_argument(
              "Software PD position control requires position_kp > 0 for joint '" + config_.name +
              "'.");
        }
        if (!finite(config_.velocity_kd) || config_.velocity_kd < 0.0) {
          return Status::invalid_argument(
              "Software PD position control requires velocity_kd >= 0 for joint '" + config_.name +
              "'.");
        }
      }
      return Status::Ok();
    case CommandInterfaceType::Velocity:
      if (config_.controller_type == JointControllerType::MuJoCoActuator) {
        if (actuator_kind != ActuatorType::Velocity) {
          return Status::model_validation_failed(
              "Velocity command requires a MuJoCo velocity actuator for joint '" + config_.name +
              "'.");
        }
      } else {
        if (actuator_kind == ActuatorType::Position || actuator_kind == ActuatorType::Velocity) {
          return Status::model_validation_failed(
              "Software PD velocity control is only supported with motor/custom/passive actuators "
              "for joint '" +
              config_.name + "'.");
        }
        if (!finite(config_.velocity_kd) || config_.velocity_kd <= 0.0) {
          return Status::invalid_argument(
              "Software PD velocity control requires velocity_kd > 0 for joint '" + config_.name +
              "'.");
        }
      }
      return Status::Ok();
    case CommandInterfaceType::Effort:
      if (actuator_kind == ActuatorType::Position || actuator_kind == ActuatorType::Velocity) {
        return Status::model_validation_failed(
            "Effort command is not supported for position/velocity actuators on joint '" +
            config_.name + "'.");
      }
      return Status::Ok();
  }

  return Status::internal("Joint '" + config_.name +
                          "' uses an unsupported command mode during binding validation.");
}

Status JointComponent::write_direct_command(const mjModel& model, mjData& data,
                                            const JointCommand& command) {
  double value = 0.0;
  switch (config_.command_mode) {
    case CommandInterfaceType::Position:
      if (!finite(command.position)) {
        return Status::invalid_argument("Position command must be finite for joint '" +
                                        config_.name + "'.");
      }
      if (!binding_.has_actuator) {
        return Status::failed_precondition("Position command requires an actuator for joint '" +
                                           config_.name + "'.");
      }
      value = clamp_actuator_control_limits(model, clamp_command_limits(command.position));
      data.ctrl[binding_.actuator_id] = value;
      return Status::Ok();
    case CommandInterfaceType::Velocity:
      if (!finite(command.velocity)) {
        return Status::invalid_argument("Velocity command must be finite for joint '" +
                                        config_.name + "'.");
      }
      if (!binding_.has_actuator) {
        return Status::failed_precondition("Velocity command requires an actuator for joint '" +
                                           config_.name + "'.");
      }
      value = clamp_actuator_control_limits(model, clamp_command_limits(command.velocity));
      data.ctrl[binding_.actuator_id] = value;
      return Status::Ok();
    case CommandInterfaceType::Effort:
      if (!finite(command.effort)) {
        return Status::invalid_argument("Effort command must be finite for joint '" + config_.name +
                                        "'.");
      }
      value = clamp_actuator_force_limits(model, clamp_command_limits(command.effort));
      return write_effort_output(model, data, value);
    case CommandInterfaceType::None:
      return Status::Ok();
  }

  return Status::internal("Joint '" + config_.name +
                          "' uses an unsupported command mode during direct command write.");
}

Status JointComponent::write_software_pd_command(const mjModel& model, mjData& data,
                                                 const JointCommand& command) {
  if (!finite(command.effort)) {
    return Status::invalid_argument("Effort feedforward must be finite for joint '" + config_.name +
                                    "'.");
  }

  const double q = data.qpos[binding_.qpos_address];
  const double dq = data.qvel[binding_.dof_address];
  double effort = command.effort;

  if (config_.command_mode == CommandInterfaceType::Position) {
    if (!finite(command.position) || !finite(command.velocity)) {
      return Status::invalid_argument(
          "Software PD position control requires finite position and velocity targets for joint '" +
          config_.name + "'.");
    }
    effort += config_.position_kp * (command.position - q);
    effort += config_.velocity_kd * (command.velocity - dq);
  } else if (config_.command_mode == CommandInterfaceType::Velocity) {
    if (!finite(command.velocity)) {
      return Status::invalid_argument(
          "Software PD velocity control requires a finite velocity target for joint '" +
          config_.name + "'.");
    }
    effort += config_.velocity_kd * (command.velocity - dq);
  } else {
    return Status::model_validation_failed(
        "Software PD is only supported for position/velocity command modes on joint '" +
        config_.name + "'.");
  }

  effort = clamp_actuator_force_limits(model, clamp_command_limits(effort));
  return write_effort_output(model, data, effort);
}

Status JointComponent::write_effort_output(const mjModel& model, mjData& data,
                                           double effort) const {
  if (binding_.has_actuator) {
    const ActuatorType actuator_kind = actuator_type();
    if (actuator_kind != ActuatorType::Motor && actuator_kind != ActuatorType::Custom) {
      return Status::failed_precondition(
          "Effort output requires motor/custom/passive actuation for joint '" + config_.name +
          "'.");
    }
    data.ctrl[binding_.actuator_id] = clamp_actuator_control_limits(model, effort);
    data.qfrc_applied[binding_.dof_address] = 0.0;
    return Status::Ok();
  }

  data.qfrc_applied[binding_.dof_address] = effort;
  return Status::Ok();
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
