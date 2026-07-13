#pragma once

#include <limits>
#include <string>

namespace mujoco_simulation {

enum class JointType {
  Unknown,
  Hinge,
  Slide,
  Ball,
  Free,
};

enum class ActuatorType {
  Unknown,
  Passive,
  Motor,
  Position,
  Velocity,
  Custom,
};

enum class CommandInterfaceType {
  None,
  Position,
  Velocity,
  Effort,
};

enum class JointControllerType {
  MuJoCoActuator,
  SoftwarePd,
};

struct JointConfig {
  std::string name;
  std::string actuator_name;
  CommandInterfaceType command_mode{CommandInterfaceType::None};
  JointControllerType controller_type{JointControllerType::MuJoCoActuator};
  double position_kp{0.0};
  double velocity_kd{0.0};
  double command_min{-std::numeric_limits<double>::infinity()};
  double command_max{std::numeric_limits<double>::infinity()};
};

struct JointState {
  std::string name;
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
};

struct JointCommand {
  std::string name;
  double position{0.0};
  double velocity{0.0};
  double acceleration{0.0};
  double effort{0.0};
};

}  // namespace mujoco_simulation
