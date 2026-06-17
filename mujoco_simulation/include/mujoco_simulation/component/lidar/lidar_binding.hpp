#pragma once

#include <cstddef>
#include <vector>

namespace mujoco_simulation {

struct LidarBeamBinding {
  std::size_t beam_index{0};
  int sensor_id{-1};
  int sensor_address{-1};
};

struct LidarBinding {
  std::vector<LidarBeamBinding> beams;
};

}  // namespace mujoco_simulation
