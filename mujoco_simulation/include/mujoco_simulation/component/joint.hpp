#pragma once

#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace mujoco_simulation {

using JointId = std::size_t;

enum class JointType { Revolute, Prismatic };

enum class JointControlMode {
    Position,  // 位置模式
    Velocity,  // 速度模式
    Effort,    // 力矩模式
    Hybrid     // 力位混控模式
};

struct JointLimit {
    double min{-std::numeric_limits<double>::infinity()};
    double max{std::numeric_limits<double>::infinity()};
};

struct JointInfo {
    JointId id{0};
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

struct JointCommand {
    JointId id{0};
    JointControlMode mode{JointControlMode::Effort};
    double position{0.0};
    double velocity{0.0};
    double effort{0.0};
    double stiffness{0.0};
    double damping{0.0};
};

using JointCommands = std::vector<JointCommand>;

struct JointState {
    JointId id{0};
    double timestamp{0.0};
    JointControlMode mode{JointControlMode::Effort};
    double position{0.0};
    double velocity{0.0};
    double effort{0.0};
};
}  // namespace mujoco_simulation
