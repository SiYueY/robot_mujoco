#include "ros2_mujoco/hardware/lidar.hpp"

namespace ros2_mujoco::hardware {
sensor_msgs::msg::LaserScan Lidar::scan(const romujoco::LaserScanState& value) const {
    const romujoco::LaserScan& scan = value.scan;
    sensor_msgs::msg::LaserScan message;
    message.header.frame_id = scan.frame_id;
    message.header.stamp.sec = static_cast<int32_t>(scan.timestamp / 1000000000ULL);
    message.header.stamp.nanosec = static_cast<uint32_t>(scan.timestamp % 1000000000ULL);
    message.angle_min = scan.angle_min;
    message.angle_max = scan.angle_max;
    message.angle_increment = scan.angle_increment;
    message.time_increment = scan.time_increment;
    message.scan_time = scan.scan_time;
    message.range_min = scan.range_min;
    message.range_max = scan.range_max;
    message.ranges.assign(scan.ranges.begin(), scan.ranges.end());
    message.intensities.assign(scan.intensities.begin(), scan.intensities.end());
    return message;
}
}  // namespace ros2_mujoco::hardware
