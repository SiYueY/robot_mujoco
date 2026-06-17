#pragma once

namespace mujoco_simulation {

struct ImuBinding {
  int framequat_sensor_id{-1};
  int framequat_address{-1};
  int gyro_sensor_id{-1};
  int gyro_address{-1};
  int accelerometer_sensor_id{-1};
  int accelerometer_address{-1};
};

}  // namespace mujoco_simulation
