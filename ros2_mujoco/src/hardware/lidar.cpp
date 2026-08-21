#include "ros2_mujoco/hardware/lidar.hpp"

namespace ros2_mujoco::hardware {
sensor_msgs::msg::LaserScan Lidar::scan(const mujoco_simulation::LidarState& value) const {
    sensor_msgs::msg::LaserScan message;
    message.header.frame_id = value.frame_id;
    message.header.stamp.sec = static_cast<int32_t>(value.timestamp);
    message.header.stamp.nanosec =
        static_cast<uint32_t>((value.timestamp - message.header.stamp.sec) * 1e9);
    message.angle_min = value.angle_min;
    message.angle_max = value.angle_max;
    message.angle_increment = value.angle_increment;
    message.time_increment = value.time_increment;
    message.scan_time = value.scan_time;
    message.range_min = value.range_min;
    message.range_max = value.range_max;
    message.ranges.assign(value.ranges.begin(), value.ranges.end());
    message.intensities.assign(value.intensities.begin(), value.intensities.end());
    return message;
}
}  // namespace ros2_mujoco::hardware
