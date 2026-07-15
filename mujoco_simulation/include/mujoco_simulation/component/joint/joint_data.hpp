#pragma once

#include <limits>
#include <string>

namespace mujoco_simulation {

enum class JointType {
  Revolute,
  Prismatic,
};

enum class ControlMode {
  Position,
  Velocity,
  Effort,
  Hybrid,
};

struct Limit {
  double min{-std::numeric_limits<double>::infinity()};
  double max{std::numeric_limits<double>::infinity()};
};

struct JointInfo {
  std::string joint;
  std::string actuator;
  double position_stiffness{0.0};
  double position_damping{0.0};
  double velocity_damping{0.0};
  Limit position_limits;
  Limit velocity_limits;
  Limit effort_limits;
};

struct JointState {
  std::string joint;
  ControlMode mode{ControlMode::Effort};
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
};

struct JointCommand {
  std::string joint;
  ControlMode mode{ControlMode::Effort};
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
  double stiffness{0.0};
  double damping{0.0};
};

}  // namespace mujoco_simulation
