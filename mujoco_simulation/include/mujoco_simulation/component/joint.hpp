#pragma once

#include <cstddef>
#include <limits>
#include <optional>
#include <string>
#include <vector>

namespace mujoco_simulation {

using JointId = std::size_t;
inline constexpr JointId kInvalidJointId = std::numeric_limits<JointId>::max();

enum class JointType { Revolute, Prismatic };
enum class JointControlMode { Position, Velocity, Effort, Hybrid };

struct JointLimit {
  double min{-std::numeric_limits<double>::infinity()};
  double max{std::numeric_limits<double>::infinity()};
};

// Retained as an internal-source compatibility spelling. New public code uses
// JointLimit, which is the single component-header contract.
using Limit = JointLimit;

struct JointConfig {
  JointId id{kInvalidJointId};
  std::string joint_name;
  std::string actuator_name;
  JointType joint_type{JointType::Revolute};
  double position_stiffness{0.0};
  double position_damping{0.0};
  double velocity_damping{0.0};
  JointLimit position_limits;
  JointLimit velocity_limits;
  JointLimit effort_limits;
  double period{0.0};
};
using JointInfo = JointConfig;

struct JointCommand {
  JointControlMode mode{JointControlMode::Effort};
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
  double stiffness{0.0};
  double damping{0.0};
};

using JointCommandBatch = std::vector<std::optional<JointCommand>>;

struct JointState {
  double timestamp{0.0};
  JointControlMode mode{JointControlMode::Effort};
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
};
} // namespace mujoco_simulation
