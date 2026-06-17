#pragma once

#include <cstdint>

#include "mujoco_simulation/hardware/data.hpp"

namespace mujoco_simulation {

struct MobileBaseCommand {
  Vector3d linear;
  Vector3d angular;
  double linear_x{0.0};
  double linear_y{0.0};
  double angular_z{0.0};
  std::uint64_t timestamp_ns{0};
};

}  // namespace mujoco_simulation
