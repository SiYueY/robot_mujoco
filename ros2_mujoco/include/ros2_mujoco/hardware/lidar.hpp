#pragma once
#include "mujoco_simulation/component/lidar.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
namespace ros2_mujoco::hardware {
class Lidar {
public:
    sensor_msgs::msg::LaserScan scan(const mujoco_simulation::LidarState& value) const;
};
}  // namespace ros2_mujoco::hardware
