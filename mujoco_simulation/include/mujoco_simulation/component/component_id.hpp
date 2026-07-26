#pragma once

#include <cstddef>
#include <limits>

namespace mujoco_simulation {

using ComponentId = std::size_t;
using JointId = ComponentId;
using ImuId = ComponentId;
using CameraId = ComponentId;
using LidarId = ComponentId;
using MobileBaseId = ComponentId;

inline constexpr ComponentId kInvalidComponentId =
    std::numeric_limits<ComponentId>::max();

} // namespace mujoco_simulation
