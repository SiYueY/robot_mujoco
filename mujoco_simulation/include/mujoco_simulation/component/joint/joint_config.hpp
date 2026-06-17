#pragma once

#include <optional>
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
  std::optional<double> command_min;
  std::optional<double> command_max;
};

}  // namespace mujoco_simulation
