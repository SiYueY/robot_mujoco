#pragma once

#include <string>

namespace mujoco_simulation {

struct SensorCommonConfig {
  std::string name;
  std::string frame_id;
  double update_rate{0.0};
};

}  // namespace mujoco_simulation
