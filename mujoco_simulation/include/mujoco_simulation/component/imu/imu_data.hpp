#pragma once

#include <cstdint>
#include <string>

#include "mujoco_simulation/common/math.hpp"
#include "mujoco_simulation/component/sensor_common_config.hpp"

namespace mujoco_simulation {

struct ImuConfig {
  SensorCommonConfig common{.update_rate = 200.0};

  std::string framequat_sensor_name;
  std::string gyro_sensor_name;
  std::string accelerometer_sensor_name;

  Vector9d orientation_covariance{};
  Vector9d angular_velocity_covariance{};
  Vector9d linear_acceleration_covariance{};
};

struct ImuState {
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
