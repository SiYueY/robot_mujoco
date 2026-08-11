#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "mujoco_simulation/common/bitmask.hpp"

namespace mujoco_simulation {

using JointId = std::size_t;

enum class JointType : uint8_t {
    Revolute = 0,   // 旋转关节
    Prismatic = 1,  // 平移关节
};

enum class JointMode : uint8_t {
    Hybrid = 0,    // 力位混控模式
    Position = 1,  // 位置模式
    Velocity = 2,  // 速度模式
    Effort = 3,    // 力矩模式
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
    JointMode default_mode{JointMode::Hybrid};
    EnumMask<JointMode> allowed_modes;
    double hybrid_stiffness{0.0};
    double hybrid_damping{0.0};
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
    std::uint8_t mode{static_cast<std::uint8_t>(JointMode::Hybrid)};
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
    std::uint8_t mode{static_cast<std::uint8_t>(JointMode::Hybrid)};
    double position{0.0};
    double velocity{0.0};
    double effort{0.0};
};
}  // namespace mujoco_simulation
