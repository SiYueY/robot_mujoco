#pragma once
#include "romujoco/component/lidar.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
namespace ros2_mujoco::hardware {
class Lidar {
public:
    sensor_msgs::msg::LaserScan scan(const romujoco::LidarState& value) const;
};
}  // namespace ros2_mujoco::hardware
