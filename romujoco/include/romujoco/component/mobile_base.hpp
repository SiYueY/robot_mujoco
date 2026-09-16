#pragma once

#include <vector>
#include "romujoco/common/geometry.hpp"
#include "romujoco/component/component_id.hpp"
namespace romujoco {

struct PlanarTwist {
    double linear_x{0.0};
    double linear_y{0.0};
    double angular_z{0.0};
};

struct MobileBaseCommand {
    ComponentId id{kInvalidComponentId};
    PlanarTwist velocity{};
};

using MobileBaseCommands = std::vector<MobileBaseCommand>;

struct MobileBaseState {
    ComponentId id{kInvalidComponentId};
    double timestamp{0.0};
    Pose3d pose{};
    Twist3d twist{};
};
}  // namespace romujoco
