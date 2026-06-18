#include "robot_mujoco_ros2/message_mapper.hpp"

#include <algorithm>
#include <cstring>

namespace robot_mujoco_ros2::message_mapper {

rclcpp::Time select_stamp(const rclcpp::Time& fallback, std::uint64_t timestamp_ns) {
  return timestamp_ns == 0 ? fallback
                           : rclcpp::Time(static_cast<int64_t>(timestamp_ns), RCL_ROS_TIME);
}

rosgraph_msgs::msg::Clock make_clock_message(const rclcpp::Time& sim_time) {
  rosgraph_msgs::msg::Clock message;
  message.clock = sim_time;
  return message;
}

sensor_msgs::msg::Imu make_imu_message(const ImuPublisherConfig& config,
                                       const mujoco_simulation::ImuSample& sample,
                                       const rclcpp::Time& fallback_stamp) {
  sensor_msgs::msg::Imu message;
  message.header.stamp = select_stamp(fallback_stamp, sample.timestamp_ns);
  message.header.frame_id = config.frame_id;
  message.orientation.x = sample.orientation[0];
  message.orientation.y = sample.orientation[1];
  message.orientation.z = sample.orientation[2];
  message.orientation.w = sample.orientation[3];
  message.orientation_covariance = sample.orientation_covariance;
  message.angular_velocity.x = sample.angular_velocity[0];
  message.angular_velocity.y = sample.angular_velocity[1];
  message.angular_velocity.z = sample.angular_velocity[2];
  message.angular_velocity_covariance = sample.angular_velocity_covariance;
  message.linear_acceleration.x = sample.linear_acceleration[0];
  message.linear_acceleration.y = sample.linear_acceleration[1];
  message.linear_acceleration.z = sample.linear_acceleration[2];
  message.linear_acceleration_covariance = sample.linear_acceleration_covariance;
  return message;
}

sensor_msgs::msg::LaserScan make_lidar_message(const LidarPublisherConfig& config,
                                               const mujoco_simulation::LidarSample& sample,
                                               const rclcpp::Time& fallback_stamp) {
  sensor_msgs::msg::LaserScan message;
  message.header.stamp = select_stamp(fallback_stamp, sample.timestamp_ns);
  message.header.frame_id = config.frame_id;
  message.angle_min = static_cast<float>(sample.angle_min);
  message.angle_max = static_cast<float>(sample.angle_max);
  message.angle_increment = static_cast<float>(sample.angle_increment);
  message.scan_time = static_cast<float>(sample.scan_time);
  message.time_increment = static_cast<float>(sample.time_increment);
  message.range_min = static_cast<float>(sample.range_min);
  message.range_max = static_cast<float>(sample.range_max);
  message.ranges.assign(sample.ranges.begin(), sample.ranges.end());
  if (sample.intensities.empty()) {
    message.intensities.assign(message.ranges.size(), 0.0F);
  } else {
    message.intensities.assign(sample.intensities.begin(), sample.intensities.end());
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
  message.k = frame.intrinsics.k;
  message.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  message.p = frame.intrinsics.p;
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
