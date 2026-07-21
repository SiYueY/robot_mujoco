#include "robot_mujoco_ros2/message_mapper.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace robot_mujoco_ros2::message_mapper {

rclcpp::Time select_stamp(const rclcpp::Time& fallback, std::uint64_t timestamp) {
  return timestamp == 0 ? fallback : rclcpp::Time(static_cast<int64_t>(timestamp), RCL_ROS_TIME);
}

rclcpp::Time select_simulation_stamp(const rclcpp::Time& fallback, double time_seconds) {
  constexpr double kNanosecondsPerSecond = 1.0e9;
  constexpr double kMaxSeconds =
      static_cast<double>(std::numeric_limits<int64_t>::max()) / kNanosecondsPerSecond;
  if (!(time_seconds > 0.0) || !std::isfinite(time_seconds) || time_seconds >= kMaxSeconds) {
    return fallback;
  }
  return rclcpp::Time(static_cast<int64_t>(time_seconds * kNanosecondsPerSecond), RCL_ROS_TIME);
}

rosgraph_msgs::msg::Clock make_clock_message(const rclcpp::Time& sim_time) {
  rosgraph_msgs::msg::Clock message;
  message.clock = sim_time;
  return message;
}

sensor_msgs::msg::Imu make_imu_message(const ImuPublisherConfig& config,
                                       const mujoco_simulation::ImuState& state,
                                       const rclcpp::Time& fallback_stamp) {
  sensor_msgs::msg::Imu message;
  message.header.stamp = select_simulation_stamp(fallback_stamp, state.timestamp);
  message.header.frame_id = config.frame_id;
  message.orientation.x = state.orientation[0];
  message.orientation.y = state.orientation[1];
  message.orientation.z = state.orientation[2];
  message.orientation.w = state.orientation[3];
  message.orientation_covariance = state.orientation_covariance;
  message.angular_velocity.x = state.angular_velocity[0];
  message.angular_velocity.y = state.angular_velocity[1];
  message.angular_velocity.z = state.angular_velocity[2];
  message.angular_velocity_covariance = state.angular_velocity_covariance;
  message.linear_acceleration.x = state.linear_acceleration[0];
  message.linear_acceleration.y = state.linear_acceleration[1];
  message.linear_acceleration.z = state.linear_acceleration[2];
  message.linear_acceleration_covariance = state.linear_acceleration_covariance;
  return message;
}

sensor_msgs::msg::LaserScan make_lidar_message(const LidarPublisherConfig& config,
                                               const mujoco_simulation::LidarState& state,
                                               const rclcpp::Time& fallback_stamp) {
  sensor_msgs::msg::LaserScan message;
  message.header.stamp = select_simulation_stamp(fallback_stamp, state.timestamp);
  message.header.frame_id = config.frame_id;
  message.angle_min = static_cast<float>(state.angle_min);
  message.angle_max = static_cast<float>(state.angle_max);
  message.angle_increment = static_cast<float>(state.angle_increment);
  message.scan_time = static_cast<float>(state.scan_time);
  message.time_increment = static_cast<float>(state.time_increment);
  message.range_min = static_cast<float>(state.range_min);
  message.range_max = static_cast<float>(state.range_max);
  message.ranges.assign(state.ranges.begin(), state.ranges.end());
  if (state.intensities.empty()) {
    message.intensities.assign(message.ranges.size(), 0.0F);
  } else {
    message.intensities.assign(state.intensities.begin(), state.intensities.end());
  }
  return message;
}

sensor_msgs::msg::CameraInfo make_camera_info_message(const CameraPublisherConfig& config,
                                                      const CameraFrame& frame) {
  sensor_msgs::msg::CameraInfo message;
  message.header.stamp = frame.acquisition_stamp;
  message.header.frame_id = config.frame_id;
  message.width = frame.width == 0 ? config.width : frame.width;
  message.height = frame.height == 0 ? config.height : frame.height;
  message.distortion_model = "plumb_bob";
  message.d.assign(5, 0.0);
  message.k = frame.camera_info.k;
  message.r = frame.camera_info.r;
  message.p = frame.camera_info.p;
  return message;
}

void fill_rgb_image_message(const CameraPublisherConfig& config, const CameraFrame& frame,
                            sensor_msgs::msg::Image* message) {
  if (message == nullptr) {
    return;
  }
  message->header.stamp = frame.acquisition_stamp;
  message->header.frame_id = config.frame_id;
  message->height = frame.height;
  message->width = frame.width;
  message->encoding = "rgb8";
  message->is_bigendian = false;
  message->step = frame.rgb_step;
  const std::size_t bytes = static_cast<std::size_t>(frame.height) * frame.rgb_step;
  if (message->data.size() >= bytes) {
    std::memcpy(message->data.data(), frame.rgb_data.data(), bytes);
  }
}

void fill_depth_image_message(const CameraPublisherConfig& config, const CameraFrame& frame,
                              sensor_msgs::msg::Image* message) {
  if (message == nullptr) {
    return;
  }
  message->header.stamp = frame.acquisition_stamp;
  message->header.frame_id = config.frame_id;
  message->height = frame.height;
  message->width = frame.width;
  message->encoding = "32FC1";
  message->is_bigendian = false;
  message->step = frame.depth_step;
  const std::size_t bytes = static_cast<std::size_t>(frame.height) * frame.depth_step;
  if (message->data.size() >= bytes) {
    std::memcpy(message->data.data(), frame.depth_data.data(), bytes);
  }
}

}  // namespace robot_mujoco_ros2::message_mapper
