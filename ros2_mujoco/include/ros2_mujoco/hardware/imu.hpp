#pragma once
#include "romujoco/component/imu.hpp"
#include "sensor_msgs/msg/imu.hpp"
namespace ros2_mujoco::hardware {
class Imu {
public:
    sensor_msgs::msg::Imu data(const romujoco::ImuState& state) const;
};
}  // namespace ros2_mujoco::hardware
