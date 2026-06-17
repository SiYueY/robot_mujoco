#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "mujoco_simulation/hardware/data.hpp"

namespace mujoco_simulation {

struct MobileBaseState {
  std::string base_frame_id;
  std::string odom_frame_id;
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double linear_x{0.0};
  double linear_y{0.0};
  double angular_z{0.0};
  Vector3d linear;
  Vector3d angular;
  std::vector<double> wheel_velocities;
  std::uint64_t timestamp_ns{0};
};

}  // namespace mujoco_simulation
