#pragma once

#include <string>

#include "mujoco_simulation/component/sensor_common_config.hpp"

namespace mujoco_simulation {

struct LidarConfig {
  SensorCommonConfig common{.update_rate = 10.0};
  std::string sensor_prefix;
  double angle_min{0.0};
  double angle_max{0.0};
  double angle_increment{0.0};
  double range_min{0.0};
  double range_max{0.0};
};

}  // namespace mujoco_simulation
