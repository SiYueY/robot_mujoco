#include "mujoco_simulation/hardware/mobile_base.hpp"

#include <cmath>

namespace mujoco_simulation {

MobileBase::MobileBase(const MjContext& context, const std::vector<Joint*>& traction_joints)
    : context_(context), traction_joints_(traction_joints) {}

bool MobileBase::init(const MobileBaseInfo& data) {
  data_ = data;
  command_ = {};
  state_ = {};
  state_.base_frame_id = data.base_frame_id;
  state_.odom_frame_id = data.odom_frame_id;
  state_.wheel_velocities.assign(traction_joints_.size(), 0.0);
  last_error_.clear();
  return validate();
}

bool MobileBase::reset() {
  last_error_.clear();
  command_ = {};
  state_.linear = {0.0, 0.0, 0.0};
  state_.angular = {0.0, 0.0, 0.0};
  std::fill(state_.wheel_velocities.begin(), state_.wheel_velocities.end(), 0.0);
  return true;
}

bool MobileBase::write(const MobileBaseCommand& command) {
  last_error_.clear();
  command_ = command;
  if (data_.type == MobileBaseType::Differential) {
    return write_differential(command);
  }
  if (data_.type == MobileBaseType::Omnidirectional) {
    return write_omnidirectional(command);
  }
  return set_error("Mobile base '" + data_.name + "' has unsupported type.");
}

bool MobileBase::read(MobileBaseState& state) {
  last_error_.clear();
  if (data_.type == MobileBaseType::Differential) {
    return read_differential(state);
  }
  if (data_.type == MobileBaseType::Omnidirectional) {
    return read_omnidirectional(state);
  }
  return set_error("Mobile base '" + data_.name + "' has unsupported type.");
}

bool MobileBase::set_error(const std::string& message) {
  last_error_ = message;
  return false;
}

bool MobileBase::validate() const {
  auto* self = const_cast<MobileBase*>(this);
  if (data_.name.empty()) {
    return self->set_error("Mobile base name must not be empty.");
  }
  if (data_.type != MobileBaseType::Differential && data_.type != MobileBaseType::Omnidirectional) {
    return self->set_error("Mobile base '" + data_.name + "' must use a supported type.");
  }
  if (data_.wheel_radius <= 0.0) {
    return self->set_error("Mobile base '" + data_.name + "' requires wheel_radius > 0.");
  }
  if (data_.type == MobileBaseType::Differential) {
    if (traction_joints_.size() != 2U) {
      return self->set_error("Differential mobile base '" + data_.name +
                             "' requires exactly 2 traction joints.");
    }
    if (data_.track_width <= 0.0) {
      return self->set_error("Differential mobile base '" + data_.name +
                             "' requires track_width > 0.");
    }
  }
  if (data_.type == MobileBaseType::Omnidirectional) {
    if (traction_joints_.size() != 4U) {
      return self->set_error("Omnidirectional mobile base '" + data_.name +
                             "' requires exactly 4 traction joints.");
    }
    if (data_.track_width <= 0.0 || data_.wheel_base <= 0.0) {
      return self->set_error("Omnidirectional mobile base '" + data_.name +
                             "' requires positive track_width and wheel_base.");
    }
  }
  for (Joint* joint : traction_joints_) {
    if (joint == nullptr) {
      return self->set_error("Mobile base '" + data_.name + "' references a null traction joint.");
    }
  }
  return true;
}

bool MobileBase::write_differential(const MobileBaseCommand& command) {
  if (std::abs(command.linear[1]) > 1e-9) {
    return set_error("Differential mobile base '" + data_.name +
                     "' does not support lateral velocity.");
  }

  const double left_velocity =
      (command.linear[0] - command.angular[2] * data_.track_width * 0.5) / data_.wheel_radius;
  const double right_velocity =
      (command.linear[0] + command.angular[2] * data_.track_width * 0.5) / data_.wheel_radius;

  if (!traction_joints_[0]->write({"", 0.0, left_velocity, 0.0, 0.0})) {
    return set_error(traction_joints_[0]->last_error());
  }
  if (!traction_joints_[1]->write({"", 0.0, right_velocity, 0.0, 0.0})) {
    return set_error(traction_joints_[1]->last_error());
  }
  return true;
}

bool MobileBase::write_omnidirectional(const MobileBaseCommand& command) {
  const double base_sum = data_.wheel_base + data_.track_width;
  const double fl =
      (command.linear[0] - command.linear[1] - base_sum * command.angular[2]) / data_.wheel_radius;
  const double fr =
      (command.linear[0] + command.linear[1] + base_sum * command.angular[2]) / data_.wheel_radius;
  const double rl =
      (command.linear[0] + command.linear[1] - base_sum * command.angular[2]) / data_.wheel_radius;
  const double rr =
      (command.linear[0] - command.linear[1] + base_sum * command.angular[2]) / data_.wheel_radius;

  const double wheel_commands[4] = {fl, fr, rl, rr};
  for (std::size_t i = 0; i < traction_joints_.size(); ++i) {
    if (!traction_joints_[i]->write({"", 0.0, wheel_commands[i], 0.0, 0.0})) {
      return set_error(traction_joints_[i]->last_error());
    }
  }
  return true;
}

bool MobileBase::read_differential(MobileBaseState& state) {
  JointState left_state;
  JointState right_state;
  if (!traction_joints_[0]->read(left_state)) {
    return set_error(traction_joints_[0]->last_error());
  }
  if (!traction_joints_[1]->read(right_state)) {
    return set_error(traction_joints_[1]->last_error());
  }

  state_.wheel_velocities = {left_state.velocity, right_state.velocity};
  state_.linear = {data_.wheel_radius * (left_state.velocity + right_state.velocity) * 0.5, 0.0,
                   0.0};
  state_.angular = {
      0.0, 0.0,
      data_.wheel_radius * (right_state.velocity - left_state.velocity) / data_.track_width};
  state = state_;
  return true;
}

bool MobileBase::read_omnidirectional(MobileBaseState& state) {
  JointState wheel_states[4];
  for (std::size_t i = 0; i < traction_joints_.size(); ++i) {
    if (!traction_joints_[i]->read(wheel_states[i])) {
      return set_error(traction_joints_[i]->last_error());
    }
    state_.wheel_velocities[i] = wheel_states[i].velocity;
  }

  const double fl = wheel_states[0].velocity;
  const double fr = wheel_states[1].velocity;
  const double rl = wheel_states[2].velocity;
  const double rr = wheel_states[3].velocity;
  const double base_sum = data_.wheel_base + data_.track_width;

  state_.linear = {data_.wheel_radius * (fl + fr + rl + rr) * 0.25,
                   data_.wheel_radius * (-fl + fr + rl - rr) * 0.25, 0.0};
  state_.angular = {0.0, 0.0, data_.wheel_radius * (-fl + fr - rl + rr) / (4.0 * base_sum)};
  state = state_;
  return true;
}

}  // namespace mujoco_simulation
