#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace mujoco_simulation {

struct LidarInfo {
  std::string name;
  std::string frame_id;
  double update_rate{10.0};
  std::string sensor_prefix;
  double angle_min{0.0};
  double angle_max{0.0};
  double angle_increment{0.0};
  double range_min{0.0};
  double range_max{0.0};
};

struct LidarState {
  std::uint64_t sequence{0};
  double timestamp{0.0}; // MuJoCo simulation time in seconds.
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

} // namespace mujoco_simulation
