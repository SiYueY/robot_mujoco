#pragma once

#include <cstdint>
#include <string>

#include "mujoco_simulation/hardware/data.hpp"

namespace mujoco_simulation {

struct ImuSample {
  std::uint64_t sequence{0};
  std::uint64_t timestamp_ns{0};
  std::string frame_id;
  Quaterniond orientation{0.0, 0.0, 0.0, 1.0};
  Vector9d orientation_covariance{};
  Vector3d angular_velocity{0.0, 0.0, 0.0};
  Vector9d angular_velocity_covariance{};
  Vector3d linear_acceleration{0.0, 0.0, 0.0};
  Vector9d linear_acceleration_covariance{};
};

}  // namespace mujoco_simulation
