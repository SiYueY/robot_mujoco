#pragma once

#include <cstdint>

namespace romujoco {

using SimTime = double;
using SimStep = std::uint64_t;

// MuJoCo resources bound to one wheel of a mobile base.
struct WheelBinding {
    int wheel_id{-1};
    int actuator_id{-1};
    int dof_address{-1};
};

// MuJoCo resources bound to one scalar joint.
struct JointBinding {
    int joint_id{-1};
    int actuator_id{-1};
    int qpos_address{-1};
    int dof_address{-1};
};

// MuJoCo sensor addresses bound to one IMU.
struct ImuBinding {
    int framequat_sensor_id{-1};
    int framequat_address{-1};
    int gyro_sensor_id{-1};
    int gyro_address{-1};
    int accelerometer_sensor_id{-1};
    int accelerometer_address{-1};
};

}  // namespace romujoco
