#pragma once

#include <string>

#include "mujoco_simulation/component/sensor_common_config.hpp"
#include "mujoco_simulation/hardware/data.hpp"

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

}  // namespace mujoco_simulation
