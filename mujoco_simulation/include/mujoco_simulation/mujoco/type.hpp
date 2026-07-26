#pragma once

#include <cstdint>

namespace mujoco_simulation {

using mjTime = double;
using mjStep = std::uint64_t;

struct mjWheel {
  int wheel_id{-1};
  int actuator_id{-1};
  int dof_address{-1};
};

struct mjJoint {
  int joint_id{-1};
  int actuator_id{-1};
  int qpos_address{-1};
  int dof_address{-1};
};

struct mjImu {
  int framequat_sensor_id{-1};
  int framequat_address{-1};
  int gyro_sensor_id{-1};
  int gyro_address{-1};
  int accelerometer_sensor_id{-1};
  int accelerometer_address{-1};
};

}  // namespace mujoco_simulation
