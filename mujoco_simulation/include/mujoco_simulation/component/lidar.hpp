#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "mujoco_simulation/component/component_id.hpp"

namespace mujoco_simulation {
struct LidarConfig {
  LidarId id{kInvalidComponentId};
  std::string name;
  std::string frame_id;
  double period{1.0 / 10.0};
  std::string sensor_prefix;
  double angle_min{0.0};
  double angle_max{0.0};
  double angle_increment{0.0};
  double range_min{0.0};
  double range_max{0.0};
};
using LidarInfo = LidarConfig;

struct LaserScan {
  std::uint64_t sequence{0};
  double timestamp{0.0};
  std::string frame_id;
  double angle_min{0.0};
  double angle_max{0.0};
  double angle_increment{0.0};
  double time_increment{0.0};
  double scan_time{0.0};
  double range_min{0.0};
  double range_max{0.0};
  std::vector<double> ranges;
  std::vector<double> intensities;
};
using LidarState = LaserScan;
} // namespace mujoco_simulation
