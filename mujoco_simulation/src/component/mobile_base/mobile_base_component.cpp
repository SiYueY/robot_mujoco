#include "mujoco_simulation/component/mobile_base/mobile_base_component.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "mujoco_simulation/component/joint/joint_config.hpp"

namespace mujoco_simulation {
namespace {

constexpr double kPi = 3.14159265358979323846;

bool is_limited(unsigned char limit_flag) { return limit_flag != 0; }

double yaw_from_xmat(const mjtNum* xmat) {
  return std::atan2(static_cast<double>(xmat[3]), static_cast<double>(xmat[0]));
}

Status validate_wheel_binding(const std::string& mobile_base_name,
                              const MobileBaseWheelBinding& wheel) {
  if (wheel.joint_name.empty()) {
    return Status::binding_failed("Mobile base '" + mobile_base_name +
                                  "' has an empty wheel binding name.");
  }
  if (wheel.joint.joint_id < 0 || wheel.joint.dof_address < 0) {
    return Status::binding_failed("Mobile base '" + mobile_base_name + "' wheel '" +
                                  wheel.joint_name + "' has invalid MuJoCo joint/dof addresses.");
  }
  return Status::Ok();
}

}  // namespace

MobileBaseComponent::MobileBaseComponent(MobileBaseConfig config, MobileBaseBinding binding)
    : binding_(std::move(binding)), config_(std::move(config)) {}

std::string_view MobileBaseComponent::name() const noexcept { return config_.name; }

Status MobileBaseComponent::bind(const mjModel& model) {
  command_ = {};
  state_ = {};
  state_.base_frame_id = config_.base_frame_id;
  state_.odom_frame_id = config_.odom_frame_id;
  state_.wheel_velocities.assign(wheel_count(), 0.0);

  Status status = validate(model);
  if (!status.ok()) {
    return status;
  }
  status = initialize_bindings(model);
  if (!status.ok()) {
    return status;
  }

  clear_odometry();
  return Status::Ok();
}

Status MobileBaseComponent::reset(const mjModel& model, mjData& data) {
  (void)model;
  (void)data;
  command_ = {};
  clear_odometry();
  return Status::Ok();
}

Status MobileBaseComponent::write(const mjModel& model, mjData& data,
                                  const MobileBaseCommand& command) {
  command_ = command;
  if (config_.type == MobileBaseType::Differential) {
    return write_differential(model, data, command);
  }
  if (config_.type == MobileBaseType::Omnidirectional) {
    return write_omnidirectional(model, data, command);
  }
  return Status::invalid_argument("Mobile base '" + config_.name + "' has unsupported type.");
}

Status MobileBaseComponent::read(const mjData& data, MobileBaseState& state) {
  if (config_.type == MobileBaseType::Differential) {
    return read_differential(data, state);
  }
  if (config_.type == MobileBaseType::Omnidirectional) {
    return read_omnidirectional(data, state);
  }
  return Status::invalid_argument("Mobile base '" + config_.name + "' has unsupported type.");
}

Status MobileBaseComponent::validate(const mjModel& model) const {
  if (config_.name.empty()) {
    return Status::invalid_argument("Mobile base name must not be empty.");
  }
  if (config_.type != MobileBaseType::Differential &&
      config_.type != MobileBaseType::Omnidirectional) {
    return Status::invalid_argument("Mobile base '" + config_.name +
                                    "' must use a supported type.");
  }
  if (config_.wheel_radius <= 0.0) {
    return Status::invalid_argument("Mobile base '" + config_.name +
                                    "' requires wheel_radius > 0.");
  }

  if (config_.type == MobileBaseType::Differential) {
    if (!binding_.differential.has_value()) {
      return Status::binding_failed("Differential mobile base '" + config_.name +
                                    "' is missing differential wheel bindings.");
    }
    Status status = validate_wheel_binding(config_.name, binding_.differential->left_wheel);
    if (!status.ok()) {
      return status;
    }
    status = validate_wheel_binding(config_.name, binding_.differential->right_wheel);
    if (!status.ok()) {
      return status;
    }
    if (config_.track_width <= 0.0) {
      return Status::invalid_argument("Differential mobile base '" + config_.name +
                                      "' requires track_width > 0.");
    }
  }

  if (config_.type == MobileBaseType::Omnidirectional) {
    if (!binding_.omnidirectional.has_value()) {
      return Status::binding_failed("Omnidirectional mobile base '" + config_.name +
                                    "' is missing wheel bindings.");
    }
    const OmnidirectionalBinding& wheels = *binding_.omnidirectional;
    Status status = validate_wheel_binding(config_.name, wheels.front_left);
    if (!status.ok()) {
      return status;
    }
    status = validate_wheel_binding(config_.name, wheels.front_right);
    if (!status.ok()) {
      return status;
    }
    status = validate_wheel_binding(config_.name, wheels.rear_left);
    if (!status.ok()) {
      return status;
    }
    status = validate_wheel_binding(config_.name, wheels.rear_right);
    if (!status.ok()) {
      return status;
    }
    if (config_.track_width <= 0.0 || config_.wheel_base <= 0.0) {
      return Status::invalid_argument("Omnidirectional mobile base '" + config_.name +
                                      "' requires positive track_width and wheel_base.");
    }
  }

  if (config_.odometry_source == OdometrySource::GroundTruthBodyPose &&
      config_.base_body_name.empty()) {
    return Status::invalid_argument("Mobile base '" + config_.name +
                                    "' requires base_body_name when using GroundTruthBodyPose.");
  }

  (void)model;
  return Status::Ok();
}

Status MobileBaseComponent::initialize_bindings(const mjModel& model) {
  binding_.base_body_id = -1;

  if (config_.odometry_source == OdometrySource::GroundTruthBodyPose) {
    binding_.base_body_id = mj_name2id(&model, mjOBJ_BODY, config_.base_body_name.c_str());
    if (binding_.base_body_id < 0) {
      return Status::binding_failed("Mobile base '" + config_.name +
                                    "' could not bind ground-truth body '" +
                                    config_.base_body_name + "'.");
    }
  }

  return Status::Ok();
}

void MobileBaseComponent::clear_odometry() {
  state_.x = 0.0;
  state_.y = 0.0;
  state_.yaw = 0.0;
  state_.linear_x = 0.0;
  state_.linear_y = 0.0;
  state_.angular_z = 0.0;
  state_.linear = {0.0, 0.0, 0.0};
  state_.angular = {0.0, 0.0, 0.0};
  state_.timestamp_ns = 0;
  std::fill(state_.wheel_velocities.begin(), state_.wheel_velocities.end(), 0.0);
  last_simulation_time_ = std::numeric_limits<double>::quiet_NaN();
}

void MobileBaseComponent::update_state_fields(const mjData& data) {
  state_.linear = {state_.linear_x, state_.linear_y, 0.0};
  state_.angular = {0.0, 0.0, state_.angular_z};
  state_.timestamp_ns = data.time <= 0.0 ? 0 : static_cast<std::uint64_t>(data.time * 1.0e9);
}

void MobileBaseComponent::integrate_wheel_odometry(double simulation_time) {
  if (!std::isfinite(last_simulation_time_)) {
    last_simulation_time_ = simulation_time;
    return;
  }

  const double dt = simulation_time - last_simulation_time_;
  last_simulation_time_ = simulation_time;
  if (!(dt > 0.0)) {
    return;
  }

  const double heading = state_.yaw + state_.angular_z * 0.5 * dt;
  const double cos_heading = std::cos(heading);
  const double sin_heading = std::sin(heading);
  state_.x += (state_.linear_x * cos_heading - state_.linear_y * sin_heading) * dt;
  state_.y += (state_.linear_x * sin_heading + state_.linear_y * cos_heading) * dt;
  state_.yaw = normalized_yaw(state_.yaw + state_.angular_z * dt);
}

Status MobileBaseComponent::update_ground_truth_pose(const mjData& data) {
  if (binding_.base_body_id < 0) {
    return Status::failed_precondition(
        "Ground truth pose state is not available for mobile base '" + config_.name + "'.");
  }

  const mjtNum* xpos = data.xpos + 3 * binding_.base_body_id;
  const mjtNum* xmat = data.xmat + 9 * binding_.base_body_id;
  state_.x = static_cast<double>(xpos[0]);
  state_.y = static_cast<double>(xpos[1]);
  state_.yaw = normalized_yaw(yaw_from_xmat(xmat));
  last_simulation_time_ = data.time;
  return Status::Ok();
}

double MobileBaseComponent::normalized_yaw(double yaw) {
  while (yaw > kPi) {
    yaw -= 2.0 * kPi;
  }
  while (yaw < -kPi) {
    yaw += 2.0 * kPi;
  }
  return yaw;
}

double MobileBaseComponent::command_linear_x(const MobileBaseCommand& command) const {
  return command.linear_x != 0.0 ? command.linear_x : command.linear[0];
}

double MobileBaseComponent::command_linear_y(const MobileBaseCommand& command) const {
  return command.linear_y != 0.0 ? command.linear_y : command.linear[1];
}

double MobileBaseComponent::command_angular_z(const MobileBaseCommand& command) const {
  return command.angular_z != 0.0 ? command.angular_z : command.angular[2];
}

Status MobileBaseComponent::write_differential(const mjModel& model, mjData& data,
                                               const MobileBaseCommand& command) {
  const double linear_x = command_linear_x(command);
  const double linear_y = command_linear_y(command);
  const double angular_z = command_angular_z(command);
  if (std::abs(linear_y) > 1e-9) {
    return Status::command_rejected("Differential mobile base '" + config_.name +
                                    "' does not support lateral velocity.");
  }
  if (!binding_.differential.has_value()) {
    return Status::failed_precondition("Differential mobile base '" + config_.name +
                                       "' is not bound.");
  }

  const double left_velocity =
      (linear_x - angular_z * config_.track_width * 0.5) / config_.wheel_radius;
  const double right_velocity =
      (linear_x + angular_z * config_.track_width * 0.5) / config_.wheel_radius;

  Status status =
      write_wheel_velocity_command(model, data, binding_.differential->left_wheel, left_velocity);
  if (!status.ok()) {
    return status;
  }
  return write_wheel_velocity_command(model, data, binding_.differential->right_wheel,
                                      right_velocity);
}

Status MobileBaseComponent::write_omnidirectional(const mjModel& model, mjData& data,
                                                  const MobileBaseCommand& command) {
  if (!binding_.omnidirectional.has_value()) {
    return Status::failed_precondition("Omnidirectional mobile base '" + config_.name +
                                       "' is not bound.");
  }

  const double linear_x = command_linear_x(command);
  const double linear_y = command_linear_y(command);
  const double angular_z = command_angular_z(command);
  const double base_sum = config_.wheel_base + config_.track_width;
  const double fl = (linear_x - linear_y - base_sum * angular_z) / config_.wheel_radius;
  const double fr = (linear_x + linear_y + base_sum * angular_z) / config_.wheel_radius;
  const double rl = (linear_x + linear_y - base_sum * angular_z) / config_.wheel_radius;
  const double rr = (linear_x - linear_y + base_sum * angular_z) / config_.wheel_radius;

  const OmnidirectionalBinding& wheels = *binding_.omnidirectional;
  Status status = write_wheel_velocity_command(model, data, wheels.front_left, fl);
  if (!status.ok()) {
    return status;
  }
  status = write_wheel_velocity_command(model, data, wheels.front_right, fr);
  if (!status.ok()) {
    return status;
  }
  status = write_wheel_velocity_command(model, data, wheels.rear_left, rl);
  if (!status.ok()) {
    return status;
  }
  return write_wheel_velocity_command(model, data, wheels.rear_right, rr);
}

Status MobileBaseComponent::read_differential(const mjData& data, MobileBaseState& state) {
  if (!binding_.differential.has_value()) {
    return Status::failed_precondition("Differential mobile base '" + config_.name +
                                       "' is not bound.");
  }

  double left_velocity = 0.0;
  double right_velocity = 0.0;
  Status status = read_wheel_velocity(data, binding_.differential->left_wheel, left_velocity);
  if (!status.ok()) {
    return status;
  }
  status = read_wheel_velocity(data, binding_.differential->right_wheel, right_velocity);
  if (!status.ok()) {
    return status;
  }

  state_.wheel_velocities = {left_velocity, right_velocity};
  state_.linear_x = config_.wheel_radius * (left_velocity + right_velocity) * 0.5;
  state_.linear_y = 0.0;
  state_.angular_z = config_.wheel_radius * (right_velocity - left_velocity) / config_.track_width;

  if (config_.odometry_source == OdometrySource::WheelIntegration) {
    integrate_wheel_odometry(data.time);
  } else {
    status = update_ground_truth_pose(data);
    if (!status.ok()) {
      return status;
    }
  }

  update_state_fields(data);
  state = state_;
  return Status::Ok();
}

Status MobileBaseComponent::read_omnidirectional(const mjData& data, MobileBaseState& state) {
  if (!binding_.omnidirectional.has_value()) {
    return Status::failed_precondition("Omnidirectional mobile base '" + config_.name +
                                       "' is not bound.");
  }

  double fl = 0.0;
  double fr = 0.0;
  double rl = 0.0;
  double rr = 0.0;
  const OmnidirectionalBinding& wheels = *binding_.omnidirectional;
  Status status = read_wheel_velocity(data, wheels.front_left, fl);
  if (!status.ok()) {
    return status;
  }
  status = read_wheel_velocity(data, wheels.front_right, fr);
  if (!status.ok()) {
    return status;
  }
  status = read_wheel_velocity(data, wheels.rear_left, rl);
  if (!status.ok()) {
    return status;
  }
  status = read_wheel_velocity(data, wheels.rear_right, rr);
  if (!status.ok()) {
    return status;
  }

  state_.wheel_velocities = {fl, fr, rl, rr};
  const double base_sum = config_.wheel_base + config_.track_width;

  state_.linear_x = config_.wheel_radius * (fl + fr + rl + rr) * 0.25;
  state_.linear_y = config_.wheel_radius * (-fl + fr + rl - rr) * 0.25;
  state_.angular_z = config_.wheel_radius * (-fl + fr - rl + rr) / (4.0 * base_sum);

  if (config_.odometry_source == OdometrySource::WheelIntegration) {
    integrate_wheel_odometry(data.time);
  } else {
    status = update_ground_truth_pose(data);
    if (!status.ok()) {
      return status;
    }
  }

  update_state_fields(data);
  state = state_;
  return Status::Ok();
}

std::size_t MobileBaseComponent::wheel_count() const {
  if (binding_.differential.has_value()) {
    return 2U;
  }
  if (binding_.omnidirectional.has_value()) {
    return 4U;
  }
  return 0U;
}

double MobileBaseComponent::clamp_actuator_control_limits(const mjModel& model,
                                                          const JointBinding& binding,
                                                          double value) {
  if (!binding.has_actuator || binding.actuator_id < 0) {
    return value;
  }
  if (!is_limited(model.actuator_ctrllimited[binding.actuator_id])) {
    return value;
  }
  const mjtNum* range = model.actuator_ctrlrange + 2 * binding.actuator_id;
  return std::clamp(value, static_cast<double>(range[0]), static_cast<double>(range[1]));
}

Status MobileBaseComponent::write_wheel_velocity_command(const mjModel& model, mjData& data,
                                                         const MobileBaseWheelBinding& wheel,
                                                         double velocity) {
  if (!wheel.joint.has_actuator || wheel.joint.actuator_id < 0) {
    return Status::failed_precondition("Mobile base wheel '" + wheel.joint_name +
                                       "' requires a MuJoCo actuator for velocity commands.");
  }
  if (wheel.joint.actuator_type != static_cast<int>(ActuatorType::Velocity)) {
    return Status::failed_precondition("Mobile base wheel '" + wheel.joint_name +
                                       "' requires a MuJoCo velocity actuator.");
  }

  data.ctrl[wheel.joint.actuator_id] = clamp_actuator_control_limits(model, wheel.joint, velocity);
  if (wheel.joint.dof_address >= 0) {
    data.qfrc_applied[wheel.joint.dof_address] = 0.0;
  }
  return Status::Ok();
}

Status MobileBaseComponent::read_wheel_velocity(const mjData& data,
                                                const MobileBaseWheelBinding& wheel,
                                                double& velocity) {
  if (wheel.joint.dof_address < 0) {
    return Status::failed_precondition("Mobile base wheel '" + wheel.joint_name +
                                       "' is missing a valid MuJoCo dof address.");
  }
  velocity = data.qvel[wheel.joint.dof_address];
  return Status::Ok();
}

}  // namespace mujoco_simulation
