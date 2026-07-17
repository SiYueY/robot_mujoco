#include "mujoco_simulation/component/mobile_base/mobile_base_component.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "mujoco_simulation/common/logging.hpp"
#include "mujoco_simulation/component/update_context.hpp"

namespace mujoco_simulation {
namespace {

constexpr double kPi = 3.14159265358979323846;

struct MobileBaseWheelBinding {
  std::string joint_name;
  struct JointDriveHandle {
    int joint_id{-1};
    int dof_address{-1};
    int motor_id{-1};
    double velocity_damping{0.0};
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

bool validate_wheel_binding(const std::string& mobile_base_name,
                            const MobileBaseWheelBinding& wheel) {
  (void)mobile_base_name;
  if (wheel.joint_name.empty()) {
    LOG_ERROR << "MobileBaseComponent::validate_wheel_binding"
              << ": "
              << "wheel joint name must not be empty.";
    return false;
  }
  if (wheel.drive.joint_id < 0 || wheel.drive.dof_address < 0) {
    LOG_ERROR << "MobileBaseComponent::validate_wheel_binding"
              << ": "
              << "wheel drive handle is not bound.";
    return false;
  }
  return true;
}

MobileBaseWheelBinding::JointDriveHandle make_drive_handle(const JointComponent& joint) {
  MobileBaseWheelBinding::JointDriveHandle handle;
  handle.joint_id = joint.joint_id();
  handle.dof_address = joint.dof_address();
  handle.motor_id = joint.motor_id();
  handle.velocity_damping = joint.info().velocity_damping;
  return handle;
}

double clamp_actuator_control_limits(const mjModel& model,
                                     const MobileBaseWheelBinding::JointDriveHandle& drive,
                                     double value) {
  if (drive.motor_id < 0) {
    return value;
  }
  if (!is_limited(model.actuator_ctrllimited[drive.motor_id])) {
    return value;
  }
  const mjtNum* range = model.actuator_ctrlrange + 2 * drive.motor_id;
  return std::clamp(value, static_cast<double>(range[0]), static_cast<double>(range[1]));
}

bool write_wheel_velocity_command(const mjModel& model, mjData& data,
                                  const MobileBaseWheelBinding& wheel, double velocity) {
  if (wheel.drive.motor_id < 0 || wheel.drive.dof_address < 0 ||
      wheel.drive.velocity_damping <= 0.0) {
    LOG_ERROR << "MobileBaseComponent::write_wheel_velocity_command"
              << ": "
              << "wheel drive handle is incomplete.";
    return false;
  }
  const double effort =
      wheel.drive.velocity_damping * (velocity - data.qvel[wheel.drive.dof_address]);
  data.ctrl[wheel.drive.motor_id] = clamp_actuator_control_limits(model, wheel.drive, effort);
  return true;
}

bool read_wheel_velocity(const mjData& data, const MobileBaseWheelBinding& wheel,
                         double& velocity) {
  if (wheel.drive.dof_address < 0) {
    LOG_ERROR << "MobileBaseComponent::read_wheel_velocity"
              << ": "
              << "wheel drive handle is incomplete.";
    return false;
  }
  velocity = data.qvel[wheel.drive.dof_address];
  return true;
}

}  // namespace

struct MobileBaseComponent::Impl {
  MobileBaseBinding binding;
};

MobileBaseComponent::MobileBaseComponent(MobileBaseInfo info)
    : impl_(std::make_unique<Impl>()), info_(std::move(info)) {
  set_update_every_step();
}

MobileBaseComponent::~MobileBaseComponent() = default;

MobileBaseComponent::MobileBaseComponent(MobileBaseComponent&&) noexcept = default;

MobileBaseComponent& MobileBaseComponent::operator=(MobileBaseComponent&&) noexcept = default;

bool MobileBaseComponent::configure_differential_drive(const JointComponent& left_wheel,
                                                       const JointComponent& right_wheel) {
  impl_->binding = {};
  impl_->binding.has_differential = true;
  impl_->binding.differential.left_wheel.joint_name = info_.left_wheel_joint;
  impl_->binding.differential.left_wheel.drive = make_drive_handle(left_wheel);
  impl_->binding.differential.right_wheel.joint_name = info_.right_wheel_joint;
  impl_->binding.differential.right_wheel.drive = make_drive_handle(right_wheel);
  return true;
}

bool MobileBaseComponent::configure_omnidirectional_drive(const JointComponent& front_left,
                                                          const JointComponent& front_right,
                                                          const JointComponent& rear_left,
                                                          const JointComponent& rear_right) {
  impl_->binding = {};
  impl_->binding.has_omnidirectional = true;
  impl_->binding.omnidirectional.front_left.joint_name = info_.front_left_joint;
  impl_->binding.omnidirectional.front_left.drive = make_drive_handle(front_left);
  impl_->binding.omnidirectional.front_right.joint_name = info_.front_right_joint;
  impl_->binding.omnidirectional.front_right.drive = make_drive_handle(front_right);
  impl_->binding.omnidirectional.rear_left.joint_name = info_.rear_left_joint;
  impl_->binding.omnidirectional.rear_left.drive = make_drive_handle(rear_left);
  impl_->binding.omnidirectional.rear_right.joint_name = info_.rear_right_joint;
  impl_->binding.omnidirectional.rear_right.drive = make_drive_handle(rear_right);
  return true;
}

std::string MobileBaseComponent::name() const noexcept { return info_.name; }

bool MobileBaseComponent::bind(const mjModel& model) {
  command_ = {};
  state_ = {};
  state_.base_frame_id = info_.base_frame_id;
  state_.odom_frame_id = info_.odom_frame_id;
  state_.wheel_velocities.assign(wheel_count(), 0.0);

  if (!validate(model)) {
    return false;
  }
  if (!initialize_bindings(model)) {
    return false;
  }

  clear_odometry();
  return true;
}

bool MobileBaseComponent::reset(const mjModel& model, mjData& data) {
  (void)model;
  (void)data;
  command_ = {};
  clear_odometry();
  return true;
}

bool MobileBaseComponent::update(const UpdateContext& context) {
  MobileBaseState state;
  return read(context.data, state);
}

bool MobileBaseComponent::write(const mjModel& model, mjData& data,
                                const MobileBaseCommand& command) {
  command_ = command;
  if (info_.type == MobileBaseType::Differential) {
    return write_differential(model, data, command);
  }
  if (info_.type == MobileBaseType::Omnidirectional) {
    return write_omnidirectional(model, data, command);
  }
  LOG_ERROR << "MobileBaseComponent::write"
            << ": "
            << "unsupported mobile base type.";
  return false;
}

bool MobileBaseComponent::read(const mjData& data, MobileBaseState& state) {
  if (info_.type == MobileBaseType::Differential) {
    return read_differential(data, state);
  }
  if (info_.type == MobileBaseType::Omnidirectional) {
    return read_omnidirectional(data, state);
  }
  LOG_ERROR << "MobileBaseComponent::read"
            << ": "
            << "unsupported mobile base type.";
  return false;
}

bool MobileBaseComponent::validate(const mjModel& model) const {
  if (info_.name.empty()) {
    LOG_ERROR << "MobileBaseComponent::validate"
              << ": "
              << "mobile base name must not be empty.";
    return false;
  }
  if (info_.type != MobileBaseType::Differential && info_.type != MobileBaseType::Omnidirectional) {
    LOG_ERROR << "MobileBaseComponent::validate"
              << ": "
              << "mobile base type is invalid.";
    return false;
  }
  if (info_.wheel_radius <= 0.0) {
    LOG_ERROR << "MobileBaseComponent::validate"
              << ": "
              << "wheel_radius must be positive.";
    return false;
  }

  if (info_.type == MobileBaseType::Differential) {
    if (!impl_->binding.has_differential) {
      LOG_ERROR << "MobileBaseComponent::validate"
                << ": "
                << "differential drive bindings are missing.";
      return false;
    }
    if (!validate_wheel_binding(info_.name, impl_->binding.differential.left_wheel) ||
        !validate_wheel_binding(info_.name, impl_->binding.differential.right_wheel)) {
      return false;
    }
    if (info_.track_width <= 0.0) {
      LOG_ERROR << "MobileBaseComponent::validate"
                << ": "
                << "track_width must be positive for differential drive.";
      return false;
    }
  }

  if (info_.type == MobileBaseType::Omnidirectional) {
    if (!impl_->binding.has_omnidirectional) {
      LOG_ERROR << "MobileBaseComponent::validate"
                << ": "
                << "omnidirectional drive bindings are missing.";
      return false;
    }
    const OmnidirectionalBinding& wheels = impl_->binding.omnidirectional;
    if (!validate_wheel_binding(info_.name, wheels.front_left) ||
        !validate_wheel_binding(info_.name, wheels.front_right) ||
        !validate_wheel_binding(info_.name, wheels.rear_left) ||
        !validate_wheel_binding(info_.name, wheels.rear_right)) {
      return false;
    }
    if (info_.track_width <= 0.0 || info_.wheel_base <= 0.0) {
      LOG_ERROR << "MobileBaseComponent::validate"
                << ": "
                << "track_width and wheel_base must be positive for omni drive.";
      return false;
    }
  }

  if (info_.odometry_source == OdometrySource::GroundTruthBodyPose &&
      info_.base_body_name.empty()) {
    LOG_ERROR << "MobileBaseComponent::validate"
              << ": "
              << "base_body_name is required for ground-truth odometry.";
    return false;
  }

  (void)model;
  return true;
}

bool MobileBaseComponent::initialize_bindings(const mjModel& model) {
  impl_->binding.base_body_id = -1;

  if (info_.odometry_source == OdometrySource::GroundTruthBodyPose) {
    impl_->binding.base_body_id = mj_name2id(&model, mjOBJ_BODY, info_.base_body_name.c_str());
    if (impl_->binding.base_body_id < 0) {
      LOG_ERROR << "MobileBaseComponent::initialize_bindings"
                << ": "
                << "base body was not found in model.";
      return false;
    }
  }

  return true;
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

bool MobileBaseComponent::update_ground_truth_pose(const mjData& data) {
  if (impl_->binding.base_body_id < 0) {
    LOG_ERROR << "MobileBaseComponent::update_ground_truth_pose"
              << ": "
              << "base body binding is missing.";
    return false;
  }

  const mjtNum* xpos = data.xpos + 3 * impl_->binding.base_body_id;
  const mjtNum* xmat = data.xmat + 9 * impl_->binding.base_body_id;
  state_.x = static_cast<double>(xpos[0]);
  state_.y = static_cast<double>(xpos[1]);
  state_.yaw = normalized_yaw(yaw_from_xmat(xmat));
  last_simulation_time_ = data.time;
  return true;
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

bool MobileBaseComponent::write_differential(const mjModel& model, mjData& data,
                                             const MobileBaseCommand& command) {
  const double linear_x = command_linear_x(command);
  const double linear_y = command_linear_y(command);
  const double angular_z = command_angular_z(command);
  if (std::abs(linear_y) > 1e-9) {
    LOG_ERROR << "MobileBaseComponent::write_differential"
              << ": "
              << "differential drive does not support lateral velocity.";
    return false;
  }
  if (!impl_->binding.has_differential) {
    LOG_ERROR << "MobileBaseComponent::write_differential"
              << ": "
              << "differential drive bindings are missing.";
    return false;
  }

  const double left_velocity =
      (linear_x - angular_z * info_.track_width * 0.5) / info_.wheel_radius;
  const double right_velocity =
      (linear_x + angular_z * info_.track_width * 0.5) / info_.wheel_radius;

  if (!write_wheel_velocity_command(model, data, impl_->binding.differential.left_wheel,
                                    left_velocity)) {
    return false;
  }
  return write_wheel_velocity_command(model, data, impl_->binding.differential.right_wheel,
                                      right_velocity);
}

bool MobileBaseComponent::write_omnidirectional(const mjModel& model, mjData& data,
                                                const MobileBaseCommand& command) {
  if (!impl_->binding.has_omnidirectional) {
    LOG_ERROR << "MobileBaseComponent::write_omnidirectional"
              << ": "
              << "omnidirectional drive bindings are missing.";
    return false;
  }

  const double linear_x = command_linear_x(command);
  const double linear_y = command_linear_y(command);
  const double angular_z = command_angular_z(command);
  const double base_sum = info_.wheel_base + info_.track_width;
  const double fl = (linear_x - linear_y - base_sum * angular_z) / info_.wheel_radius;
  const double fr = (linear_x + linear_y + base_sum * angular_z) / info_.wheel_radius;
  const double rl = (linear_x + linear_y - base_sum * angular_z) / info_.wheel_radius;
  const double rr = (linear_x - linear_y + base_sum * angular_z) / info_.wheel_radius;

  const OmnidirectionalBinding& wheels = impl_->binding.omnidirectional;
  if (!write_wheel_velocity_command(model, data, wheels.front_left, fl) ||
      !write_wheel_velocity_command(model, data, wheels.front_right, fr) ||
      !write_wheel_velocity_command(model, data, wheels.rear_left, rl)) {
    return false;
  }
  return write_wheel_velocity_command(model, data, wheels.rear_right, rr);
}

bool MobileBaseComponent::read_differential(const mjData& data, MobileBaseState& state) {
  if (!impl_->binding.has_differential) {
    LOG_ERROR << "MobileBaseComponent::read_differential"
              << ": "
              << "differential drive bindings are missing.";
    return false;
  }

  double left_velocity = 0.0;
  double right_velocity = 0.0;
  if (!read_wheel_velocity(data, impl_->binding.differential.left_wheel, left_velocity) ||
      !read_wheel_velocity(data, impl_->binding.differential.right_wheel, right_velocity)) {
    return false;
  }

  state_.wheel_velocities = {left_velocity, right_velocity};
  state_.linear_x = info_.wheel_radius * (left_velocity + right_velocity) * 0.5;
  state_.linear_y = 0.0;
  state_.angular_z = info_.wheel_radius * (right_velocity - left_velocity) / info_.track_width;

  if (info_.odometry_source == OdometrySource::WheelIntegration) {
    integrate_wheel_odometry(data.time);
  } else {
    if (!update_ground_truth_pose(data)) {
      return false;
    }
  }

  update_state_fields(data);
  state = state_;
  return true;
}

bool MobileBaseComponent::read_omnidirectional(const mjData& data, MobileBaseState& state) {
  if (!impl_->binding.has_omnidirectional) {
    LOG_ERROR << "MobileBaseComponent::read_omnidirectional"
              << ": "
              << "omnidirectional drive bindings are missing.";
    return false;
  }

  double fl = 0.0;
  double fr = 0.0;
  double rl = 0.0;
  double rr = 0.0;
  const OmnidirectionalBinding& wheels = impl_->binding.omnidirectional;
  if (!read_wheel_velocity(data, wheels.front_left, fl) ||
      !read_wheel_velocity(data, wheels.front_right, fr) ||
      !read_wheel_velocity(data, wheels.rear_left, rl) ||
      !read_wheel_velocity(data, wheels.rear_right, rr)) {
    return false;
  }

  state_.wheel_velocities = {fl, fr, rl, rr};
  const double base_sum = info_.wheel_base + info_.track_width;

  state_.linear_x = info_.wheel_radius * (fl + fr + rl + rr) * 0.25;
  state_.linear_y = info_.wheel_radius * (-fl + fr + rl - rr) * 0.25;
  state_.angular_z = info_.wheel_radius * (-fl + fr - rl + rr) / (4.0 * base_sum);

  if (info_.odometry_source == OdometrySource::WheelIntegration) {
    integrate_wheel_odometry(data.time);
  } else {
    if (!update_ground_truth_pose(data)) {
      return false;
    }
  }

  update_state_fields(data);
  state = state_;
  return true;
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
