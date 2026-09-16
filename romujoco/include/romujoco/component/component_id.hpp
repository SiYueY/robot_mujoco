#pragma once

#include <cstddef>
#include <limits>
namespace romujoco {
using ComponentId = std::size_t;
inline constexpr ComponentId kInvalidComponentId = std::numeric_limits<ComponentId>::max();
}  // namespace romujoco
