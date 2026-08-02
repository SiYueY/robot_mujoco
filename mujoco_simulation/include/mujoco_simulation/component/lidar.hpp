#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace mujoco_simulation {

using LidarId = std::size_t;
inline constexpr LidarId kInvalidLidarId = std::numeric_limits<LidarId>::max();

struct LidarConfig {
  LidarId id{kInvalidLidarId};
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
