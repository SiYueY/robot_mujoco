#pragma once

#include <array>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include "romujoco/common/limit.hpp"

namespace romujoco {

using GripperId = std::size_t;

// Gripper V1 is a two-finger parallel gripper.  The finger count is part of the
// device semantics of width/velocity/effort, so it is expressed directly
// instead of being modelled as a variable-length finger list.
inline constexpr std::size_t kGripperFingerCount = 2;

struct GripperFingerInfo {
    // MuJoCo slide joint of the finger.  The joint coordinate must be
    // normalised in the model so that a larger value opens the gripper.
    std::string joint_name;
    // Empty when the finger is driven by an MJCF mechanical coupling instead of
    // its own actuator.
    std::string actuator_name;
};

struct GripperControl {
    double stiffness{0.0};
    double damping{0.0};
};

struct GripperStallDetection {
    double width_tolerance{0.001};
    double velocity_threshold{0.001};
    double effort_ratio{0.9};
    double timeout{0.25};
};

struct GripperInfo {
    GripperId id{0};
    std::string name;

    std::array<GripperFingerInfo, kGripperFingerCount> fingers;

    double period{0.001};

    GripperControl control;

    Limit width_limits{0.0, std::numeric_limits<double>::infinity()};
    Limit velocity_limits{0.0, std::numeric_limits<double>::infinity()};
    Limit effort_limits{0.0, std::numeric_limits<double>::infinity()};

    GripperStallDetection stall;
};

// Device-level command.  width is the total opening, velocity is a non-negative
// speed magnitude and effort is a non-negative per-actuator force magnitude.
// Direction is derived from (width - current width), never from the sign of
// velocity.
struct GripperCommand {
    GripperId id{0};
    double width{0.0};
    double velocity{0.0};
    double effort{0.0};
};

using GripperCommands = std::vector<GripperCommand>;

// Unlike the command, state velocity is signed: positive means opening.
struct GripperState {
    GripperId id{0};
    double timestamp{0.0};
    double width{0.0};
    double velocity{0.0};
    double effort{0.0};
    bool stalled{false};
};

}  // namespace romujoco
