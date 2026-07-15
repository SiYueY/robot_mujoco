#include "mujoco_simulation/component/mobile_base/mobile_base_component.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "mujoco_simulation/component/update_context.hpp"

namespace mujoco_simulation {
namespace {

constexpr double kPi = 3.14159265358979323846;

struct MobileBaseWheelBinding {
  std::string joint_name;
  struct JointDriveHandle {
    int joint_id{-1};
    int dof_address{-1};
    int actuator_id{-1};
    int actuator_type{-1};
    bool has_actuator{false};
  } drive{};
};

struct DifferentialBinding {
  MobileBaseWheelBinding left_wheel;
  MobileBaseWheelBinding right_wheel;
};

struct OmnidirectionalBinding {
  MobileBaseWheelBinding front_left;
  MobileBaseWheelBinding front_right;
  MobileBaseWheelBinding rear_left;
  MobileBaseWheelBinding rear_right;
};

struct MobileBaseBinding {
  bool has_differential{false};
  bool has_omnidirectional{false};
  DifferentialBinding differential{};
  OmnidirectionalBinding omnidirectional{};
  int base_body_id{-1};
};

bool is_limited(unsigned char limit_flag) { return limit_flag != 0; }

double yaw_from_xmat(const mjtNum* xmat) {
  return std::atan2(static_cast<double>(xmat[3]), static_cast<double>(xmat[0]));
}

ResultCode validate_wheel_binding(const std::string& mobile_base_name,
                                  const MobileBaseWheelBinding& wheel) {
  if (wheel.joint_name.empty()) {
    return ResultCode::BindingFailed;
  }
  if (wheel.drive.joint_id < 0 || wheel.drive.dof_address < 0) {
    return ResultCode::BindingFailed;
  }
  return ResultCode::Ok;
}

MobileBaseWheelBinding::JointDriveHandle make_drive_handle(const JointComponent& joint) {
  MobileBaseWheelBinding::JointDriveHandle handle;
  handle.joint_id = joint.joint_id();
  handle.dof_address = joint.dof_address();
  handle.actuator_id = joint.actuator_id();
  handle.actuator_type = static_cast<int>(joint.actuator_type());
  handle.has_actuator = joint.has_actuator();
  return handle;
}

double clamp_actuator_control_limits(const mjModel& model,
                                     const MobileBaseWheelBinding::JointDriveHandle& drive,
                                     double value) {
  if (!drive.has_actuator || drive.actuator_id < 0) {
    return value;
  }
  if (!is_limited(model.actuator_ctrllimited[drive.actuator_id])) {
    return value;
  }
  const mjtNum* range = model.actuator_ctrlrange + 2 * drive.actuator_id;
  return std::clamp(value, static_cast<double>(range[0]), static_cast<double>(range[1]));
}

ResultCode write_wheel_velocity_command(const mjModel& model, mjData& data,
                                        const MobileBaseWheelBinding& wheel, double velocity) {
  if (!wheel.drive.has_actuator || wheel.drive.actuator_id < 0) {
    return ResultCode::FailedPrecondition;
  }
  if (wheel.drive.actuator_type != static_cast<int>(ActuatorType::Velocity)) {
    return ResultCode::FailedPrecondition;
  }

  data.ctrl[wheel.drive.actuator_id] = clamp_actuator_control_limits(model, wheel.drive, velocity);
  if (wheel.drive.dof_address >= 0) {
    data.qfrc_applied[wheel.drive.dof_address] = 0.0;
  }
  return ResultCode::Ok;
}

ResultCode read_wheel_velocity(const mjData& data, const MobileBaseWheelBinding& wheel,
                               double& velocity) {
  if (wheel.drive.dof_address < 0) {
    return ResultCode::FailedPrecondition;
  }
  velocity = data.qvel[wheel.drive.dof_address];
  return ResultCode::Ok;
}

}  // namespace

struct MobileBaseComponent::Impl {
  MobileBaseBinding binding;
};

MobileBaseComponent::MobileBaseComponent(MobileBaseConfig config)
    : impl_(std::make_unique<Impl>()), config_(std::move(config)) {
  set_update_every_step();
}

MobileBaseComponent::~MobileBaseComponent() = default;

MobileBaseComponent::MobileBaseComponent(MobileBaseComponent&&) noexcept = default;

MobileBaseComponent& MobileBaseComponent::operator=(MobileBaseComponent&&) noexcept = default;

ResultCode MobileBaseComponent::configure_differential_drive(const JointComponent& left_wheel,
                                                             const JointComponent& right_wheel) {
  impl_->binding = {};
  impl_->binding.has_differential = true;
  impl_->binding.differential.left_wheel.joint_name = config_.left_wheel_joint;
  impl_->binding.differential.left_wheel.drive = make_drive_handle(left_wheel);
  impl_->binding.differential.right_wheel.joint_name = config_.right_wheel_joint;
  impl_->binding.differential.right_wheel.drive = make_drive_handle(right_wheel);
  return ResultCode::Ok;
}

ResultCode MobileBaseComponent::configure_omnidirectional_drive(const JointComponent& front_left,
                                                                const JointComponent& front_right,
                                                                const JointComponent& rear_left,
                                                                const JointComponent& rear_right) {
  impl_->binding = {};
  impl_->binding.has_omnidirectional = true;
  impl_->binding.omnidirectional.front_left.joint_name = config_.front_left_joint;
  impl_->binding.omnidirectional.front_left.drive = make_drive_handle(front_left);
  impl_->binding.omnidirectional.front_right.joint_name = config_.front_right_joint;
  impl_->binding.omnidirectional.front_right.drive = make_drive_handle(front_right);
  impl_->binding.omnidirectional.rear_left.joint_name = config_.rear_left_joint;
  impl_->binding.omnidirectional.rear_left.drive = make_drive_handle(rear_left);
  impl_->binding.omnidirectional.rear_right.joint_name = config_.rear_right_joint;
  impl_->binding.omnidirectional.rear_right.drive = make_drive_handle(rear_right);
  return ResultCode::Ok;
}

std::string MobileBaseComponent::name() const noexcept { return config_.name; }

ResultCode MobileBaseComponent::bind(const mjModel& model) {
  command_ = {};
  state_ = {};
  state_.base_frame_id = config_.base_frame_id;
  state_.odom_frame_id = config_.odom_frame_id;
  state_.wheel_velocities.assign(wheel_count(), 0.0);

  ResultCode status = validate(model);
  if (status != ResultCode::Ok) {
    return status;
  }
  status = initialize_bindings(model);
  if (status != ResultCode::Ok) {
    return status;
  }

  clear_odometry();
  return ResultCode::Ok;
}

ResultCode MobileBaseComponent::reset(const mjModel& model, mjData& data) {
  (void)model;
  (void)data;
  command_ = {};
  clear_odometry();
  return ResultCode::Ok;
}

ResultCode MobileBaseComponent::update(const UpdateContext& context) {
  MobileBaseState state;
  return read(context.data, state);
}

ResultCode MobileBaseComponent::write(const mjModel& model, mjData& data,
                                      const MobileBaseCommand& command) {
  command_ = command;
  if (config_.type == MobileBaseType::Differential) {
    return write_differential(model, data, command);
  }
  if (config_.type == MobileBaseType::Omnidirectional) {
    return write_omnidirectional(model, data, command);
  }
  return ResultCode::InvalidArgument;
}

ResultCode MobileBaseComponent::read(const mjData& data, MobileBaseState& state) {
  if (config_.type == MobileBaseType::Differential) {
    return read_differential(data, state);
  }
  if (config_.type == MobileBaseType::Omnidirectional) {
    return read_omnidirectional(data, state);
  }
  return ResultCode::InvalidArgument;
}

ResultCode MobileBaseComponent::validate(const mjModel& model) const {
  if (config_.name.empty()) {
    return ResultCode::InvalidArgument;
  }
  if (config_.type != MobileBaseType::Differential &&
      config_.type != MobileBaseType::Omnidirectional) {
    return ResultCode::InvalidArgument;
  }
  if (config_.wheel_radius <= 0.0) {
    return ResultCode::InvalidArgument;
  }

  if (config_.type == MobileBaseType::Differential) {
    if (!impl_->binding.has_differential) {
      return ResultCode::BindingFailed;
    }
    ResultCode status =
        validate_wheel_binding(config_.name, impl_->binding.differential.left_wheel);
    if (status != ResultCode::Ok) {
      return status;
    }
    status = validate_wheel_binding(config_.name, impl_->binding.differential.right_wheel);
    if (status != ResultCode::Ok) {
      return status;
    }
    if (config_.track_width <= 0.0) {
      return ResultCode::InvalidArgument;
    }
  }

  if (config_.type == MobileBaseType::Omnidirectional) {
    if (!impl_->binding.has_omnidirectional) {
      return ResultCode::BindingFailed;
    }
    const OmnidirectionalBinding& wheels = impl_->binding.omnidirectional;
    ResultCode status = validate_wheel_binding(config_.name, wheels.front_left);
    if (status != ResultCode::Ok) {
      return status;
    }
    status = validate_wheel_binding(config_.name, wheels.front_right);
    if (status != ResultCode::Ok) {
      return status;
    }
    status = validate_wheel_binding(config_.name, wheels.rear_left);
    if (status != ResultCode::Ok) {
      return status;
    }
    status = validate_wheel_binding(config_.name, wheels.rear_right);
    if (status != ResultCode::Ok) {
      return status;
    }
    if (config_.track_width <= 0.0 || config_.wheel_base <= 0.0) {
      return ResultCode::InvalidArgument;
    }
  }

  if (config_.odometry_source == OdometrySource::GroundTruthBodyPose &&
      config_.base_body_name.empty()) {
    return ResultCode::InvalidArgument;
  }

  (void)model;
  return ResultCode::Ok;
}

ResultCode MobileBaseComponent::initialize_bindings(const mjModel& model) {
  impl_->binding.base_body_id = -1;

  if (config_.odometry_source == OdometrySource::GroundTruthBodyPose) {
    impl_->binding.base_body_id = mj_name2id(&model, mjOBJ_BODY, config_.base_body_name.c_str());
    if (impl_->binding.base_body_id < 0) {
      return ResultCode::BindingFailed;
    }
  }

  return ResultCode::Ok;
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

ResultCode MobileBaseComponent::update_ground_truth_pose(const mjData& data) {
  if (impl_->binding.base_body_id < 0) {
    return ResultCode::FailedPrecondition;
  }

  const mjtNum* xpos = data.xpos + 3 * impl_->binding.base_body_id;
  const mjtNum* xmat = data.xmat + 9 * impl_->binding.base_body_id;
  state_.x = static_cast<double>(xpos[0]);
  state_.y = static_cast<double>(xpos[1]);
  state_.yaw = normalized_yaw(yaw_from_xmat(xmat));
  last_simulation_time_ = data.time;
  return ResultCode::Ok;
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

ResultCode MobileBaseComponent::write_differential(const mjModel& model, mjData& data,
                                                   const MobileBaseCommand& command) {
  const double linear_x = command_linear_x(command);
  const double linear_y = command_linear_y(command);
  const double angular_z = command_angular_z(command);
  if (std::abs(linear_y) > 1e-9) {
    return ResultCode::CommandRejected;
  }
  if (!impl_->binding.has_differential) {
    return ResultCode::FailedPrecondition;
  }

  const double left_velocity =
      (linear_x - angular_z * config_.track_width * 0.5) / config_.wheel_radius;
  const double right_velocity =
      (linear_x + angular_z * config_.track_width * 0.5) / config_.wheel_radius;

  ResultCode status = write_wheel_velocity_command(
      model, data, impl_->binding.differential.left_wheel, left_velocity);
  if (status != ResultCode::Ok) {
    return status;
  }
  return write_wheel_velocity_command(model, data, impl_->binding.differential.right_wheel,
                                      right_velocity);
}

ResultCode MobileBaseComponent::write_omnidirectional(const mjModel& model, mjData& data,
                                                      const MobileBaseCommand& command) {
  if (!impl_->binding.has_omnidirectional) {
    return ResultCode::FailedPrecondition;
  }

  const double linear_x = command_linear_x(command);
  const double linear_y = command_linear_y(command);
  const double angular_z = command_angular_z(command);
  const double base_sum = config_.wheel_base + config_.track_width;
  const double fl = (linear_x - linear_y - base_sum * angular_z) / config_.wheel_radius;
  const double fr = (linear_x + linear_y + base_sum * angular_z) / config_.wheel_radius;
  const double rl = (linear_x + linear_y - base_sum * angular_z) / config_.wheel_radius;
  const double rr = (linear_x - linear_y + base_sum * angular_z) / config_.wheel_radius;

  const OmnidirectionalBinding& wheels = impl_->binding.omnidirectional;
  ResultCode status = write_wheel_velocity_command(model, data, wheels.front_left, fl);
  if (status != ResultCode::Ok) {
    return status;
  }
  status = write_wheel_velocity_command(model, data, wheels.front_right, fr);
  if (status != ResultCode::Ok) {
    return status;
  }
  status = write_wheel_velocity_command(model, data, wheels.rear_left, rl);
  if (status != ResultCode::Ok) {
    return status;
  }
  return write_wheel_velocity_command(model, data, wheels.rear_right, rr);
}

ResultCode MobileBaseComponent::read_differential(const mjData& data, MobileBaseState& state) {
  if (!impl_->binding.has_differential) {
    return ResultCode::FailedPrecondition;
  }

  double left_velocity = 0.0;
  double right_velocity = 0.0;
  ResultCode status =
      read_wheel_velocity(data, impl_->binding.differential.left_wheel, left_velocity);
  if (status != ResultCode::Ok) {
    return status;
  }
  status = read_wheel_velocity(data, impl_->binding.differential.right_wheel, right_velocity);
  if (status != ResultCode::Ok) {
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
    if (status != ResultCode::Ok) {
      return status;
    }
  }

  update_state_fields(data);
  state = state_;
  return ResultCode::Ok;
}

ResultCode MobileBaseComponent::read_omnidirectional(const mjData& data, MobileBaseState& state) {
  if (!impl_->binding.has_omnidirectional) {
    return ResultCode::FailedPrecondition;
  }

  double fl = 0.0;
  double fr = 0.0;
  double rl = 0.0;
  double rr = 0.0;
  const OmnidirectionalBinding& wheels = impl_->binding.omnidirectional;
  ResultCode status = read_wheel_velocity(data, wheels.front_left, fl);
  if (status != ResultCode::Ok) {
    return status;
  }
  status = read_wheel_velocity(data, wheels.front_right, fr);
  if (status != ResultCode::Ok) {
    return status;
  }
  status = read_wheel_velocity(data, wheels.rear_left, rl);
  if (status != ResultCode::Ok) {
    return status;
  }
  status = read_wheel_velocity(data, wheels.rear_right, rr);
  if (status != ResultCode::Ok) {
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
    if (status != ResultCode::Ok) {
      return status;
    }
  }

  update_state_fields(data);
  state = state_;
  return ResultCode::Ok;
}

std::size_t MobileBaseComponent::wheel_count() const {
  if (impl_->binding.has_differential) {
    return 2U;
  }
  if (impl_->binding.has_omnidirectional) {
    return 4U;
  }
  return 0U;
}

}  // namespace mujoco_simulation
