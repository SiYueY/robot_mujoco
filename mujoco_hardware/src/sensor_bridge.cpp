#include "mujoco_hardware/sensor_bridge.hpp"

#include "rclcpp/qos.hpp"

namespace mujoco_hardware {

// ============================================================================
//  ImuPublisher
// ============================================================================

ImuPublisher::ImuPublisher(rclcpp::Node& node, const std::string& topic)
    : publisher_(node.create_publisher<sensor_msgs::msg::Imu>(topic, rclcpp::SensorDataQoS())) {}

void ImuPublisher::publish(const ImuData& imu, const rclcpp::Time& stamp) {
  sensor_msgs::msg::Imu message;
  message.header.stamp = stamp;
  message.header.frame_id = imu.frame_id;
  message.orientation.x = imu.state.orientation[0];
  message.orientation.y = imu.state.orientation[1];
  message.orientation.z = imu.state.orientation[2];
  message.orientation.w = imu.state.orientation[3];
  message.orientation_covariance = imu.state.orientation_covariance;
  message.angular_velocity.x = imu.state.angular_velocity[0];
  message.angular_velocity.y = imu.state.angular_velocity[1];
  message.angular_velocity.z = imu.state.angular_velocity[2];
  message.angular_velocity_covariance = imu.state.angular_velocity_covariance;
  message.linear_acceleration.x = imu.state.linear_acceleration[0];
  message.linear_acceleration.y = imu.state.linear_acceleration[1];
  message.linear_acceleration.z = imu.state.linear_acceleration[2];
  message.linear_acceleration_covariance = imu.state.linear_acceleration_covariance;
  publisher_->publish(message);
}

// ============================================================================
//  CameraPublisher
// ============================================================================

CameraPublisher::CameraPublisher(rclcpp::Node& node, const std::string& rgb_topic,
                                 const std::string& depth_topic,
                                 const std::string& camera_info_topic, bool enable_rgb,
                                 bool enable_depth)
    : depth_enabled_(enable_depth) {
  if (enable_rgb) {
    rgb_ = node.create_publisher<sensor_msgs::msg::Image>(rgb_topic, rclcpp::SensorDataQoS());
  }
  if (enable_depth) {
    depth_ = node.create_publisher<sensor_msgs::msg::Image>(depth_topic, rclcpp::SensorDataQoS());
  }
  camera_info_ = node.create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic,
                                                                     rclcpp::SensorDataQoS());
}

void CameraPublisher::publish(const CameraData& camera, const rclcpp::Time& stamp) {
  if (rgb_ != nullptr) {
    sensor_msgs::msg::Image message;
    message.header.stamp = stamp;
    message.header.frame_id = camera.frame_id;
    message.height = camera.state.image.height;
    message.width = camera.state.image.width;
    message.encoding = camera.state.image.encoding;
    message.is_bigendian = camera.state.image.is_bigendian;
    message.step = camera.state.image.step;
    message.data = camera.state.image.data;
    rgb_->publish(message);
  }

  if (depth_ != nullptr) {
    sensor_msgs::msg::Image message;
    message.header.stamp = stamp;
    message.header.frame_id = camera.frame_id;
    message.height = camera.state.depth_image.height;
    message.width = camera.state.depth_image.width;
    message.encoding = camera.state.depth_image.encoding;
    message.is_bigendian = camera.state.depth_image.is_bigendian;
    message.step = camera.state.depth_image.step;
    message.data = camera.state.depth_image.data;
    depth_->publish(message);
  }

  sensor_msgs::msg::CameraInfo info;
  info.header.stamp = stamp;
  info.header.frame_id = camera.frame_id;
  info.width = static_cast<uint32_t>(camera.info.width);
  info.height = static_cast<uint32_t>(camera.info.height);
  info.distortion_model = camera.intrinsics.distortion_model;
  info.d = camera.intrinsics.distortion_coefficients;
  info.k = camera.intrinsics.intrinsic_matrix;
  info.r = camera.intrinsics.rectification_matrix;
  info.p = camera.intrinsics.projection_matrix;
  camera_info_->publish(info);
}

// ============================================================================
//  LidarPublisher
// ============================================================================

LidarPublisher::LidarPublisher(rclcpp::Node& node, const std::string& topic)
    : publisher_(
          node.create_publisher<sensor_msgs::msg::LaserScan>(topic, rclcpp::SensorDataQoS())) {}

void LidarPublisher::publish(const LidarData& lidar, const rclcpp::Time& stamp) {
  sensor_msgs::msg::LaserScan message;
  message.header.stamp = stamp;
  message.header.frame_id = lidar.frame_id;
  message.angle_min = static_cast<float>(lidar.state.laser_scan.angle_min);
  message.angle_max = static_cast<float>(lidar.state.laser_scan.angle_max);
  message.angle_increment = static_cast<float>(lidar.state.laser_scan.angle_increment);
  message.scan_time = static_cast<float>(lidar.state.laser_scan.scan_time);
  message.time_increment = static_cast<float>(lidar.state.laser_scan.time_increment);
  message.range_min = static_cast<float>(lidar.state.laser_scan.range_min);
  message.range_max = static_cast<float>(lidar.state.laser_scan.range_max);
  message.ranges.assign(lidar.state.laser_scan.ranges.begin(), lidar.state.laser_scan.ranges.end());
  if (lidar.state.laser_scan.intensities.empty()) {
    message.intensities.assign(message.ranges.size(), 0.0F);
  } else {
    message.intensities.assign(lidar.state.laser_scan.intensities.begin(),
                               lidar.state.laser_scan.intensities.end());
  }
  publisher_->publish(message);
}

// ============================================================================
//  SensorBridge
// ============================================================================

SensorBridge::SensorBridge(const std::string& node_name, const std::vector<ImuData>* imus,
                           const std::vector<CameraData>* cameras,
                           const std::vector<LidarData>* lidars)
    : node_(std::make_shared<rclcpp::Node>(node_name)) {
  if (imus != nullptr) {
    for (const auto& imu : *imus) {
      imus_.emplace(imu.name, ImuPublisher(*node_, imu.topic));
    }
  }
  if (cameras != nullptr) {
    for (const auto& camera : *cameras) {
      cameras_.emplace(
          camera.name,
          CameraPublisher(*node_, camera.rgb_topic, camera.depth_topic, camera.camera_info_topic,
                          camera.info.enable_rgb, camera.info.enable_depth));
    }
  }
  if (lidars != nullptr) {
    for (const auto& lidar : *lidars) {
      lidars_.emplace(lidar.name, LidarPublisher(*node_, lidar.topic));
    }
  }
}

void SensorBridge::set_time(const rclcpp::Time& sim_time) { sim_time_ = sim_time; }

bool SensorBridge::publish_imu(const ImuData& imu) {
  const auto it = imus_.find(imu.name);
  if (it == imus_.end()) {
    return false;
  }
  it->second.publish(imu, sim_time_);
  return true;
}

bool SensorBridge::publish_camera(const CameraData& camera) {
  const auto it = cameras_.find(camera.name);
  if (it == cameras_.end()) {
    return false;
  }
  it->second.publish(camera, sim_time_);
  return true;
}

bool SensorBridge::publish_lidar(const LidarData& lidar) {
  const auto it = lidars_.find(lidar.name);
  if (it == lidars_.end()) {
    return false;
  }
  it->second.publish(lidar, sim_time_);
  return true;
}

}  // namespace mujoco_hardware
