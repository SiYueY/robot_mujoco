#pragma once

#include <cstdint>
#include <string>

#include "mujoco_simulation/common/math.hpp"

namespace mujoco_simulation {

struct ImuInfo {
  std::string name;
  std::string frame_id;

  std::string framequat_sensor_name;
  std::string gyro_sensor_name;
  std::string accelerometer_sensor_name;

  Vector9d orientation_covariance{};
  Vector9d angular_velocity_covariance{};
  Vector9d linear_acceleration_covariance{};

  double update_rate{0.0};
};

// ROS2 Imu:
// https://github.com/ros2/common_interfaces/blob/humble/sensor_msgs/msg/Imu.msg
struct ImuState {
  std::uint64_t sequence{0};
  double timestamp{0.0}; // seconds
  std::string frame_id;
  Vector4d orientation{0.0, 0.0, 0.0, 1.0};
  Vector9d orientation_covariance{};
  Vector3d angular_velocity{0.0, 0.0, 0.0};
  Vector9d angular_velocity_covariance{};
  Vector3d linear_acceleration{0.0, 0.0, 0.0};
  Vector9d linear_acceleration_covariance{};
};

} // namespace mujoco_simulation
