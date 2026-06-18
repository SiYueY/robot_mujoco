#pragma once

#include <cstdint>
#include <vector>

#include "rclcpp/time.hpp"
#include "robot_mujoco_ros2/bridge_config.hpp"
#include "robot_mujoco_ros2/publish_channel.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace robot_mujoco_ros2::message_mapper {

rclcpp::Time select_stamp(const rclcpp::Time& fallback, std::uint64_t timestamp_ns);
rosgraph_msgs::msg::Clock make_clock_message(const rclcpp::Time& sim_time);
sensor_msgs::msg::Imu make_imu_message(const ImuPublisherConfig& config,
                                       const mujoco_simulation::ImuSample& sample,
                                       const rclcpp::Time& fallback_stamp);
sensor_msgs::msg::LaserScan make_lidar_message(const LidarPublisherConfig& config,
                                               const mujoco_simulation::LidarSample& sample,
                                               const rclcpp::Time& fallback_stamp);
sensor_msgs::msg::CameraInfo make_camera_info_message(const CameraPublisherConfig& config,
                                                      const CameraFrame& frame);
void fill_rgb_image_message(const CameraPublisherConfig& config, const CameraFrame& frame,
                            sensor_msgs::msg::Image* message);
void fill_depth_image_message(const CameraPublisherConfig& config, const CameraFrame& frame,
                              sensor_msgs::msg::Image* message);

}  // namespace robot_mujoco_ros2::message_mapper
