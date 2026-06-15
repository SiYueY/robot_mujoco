#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "mujoco_hardware/data.hpp"
#include "mujoco_simulation/hardware/camera.hpp"
#include "mujoco_simulation/hardware/imu.hpp"
#include "mujoco_simulation/hardware/lidar.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/time.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace mujoco_hardware {

// ---------------------------------------------------------------------------
// Per-sensor publisher classes — each encapsulates ROS publisher creation and
// message filling, making them independently testable.
// ---------------------------------------------------------------------------

class ImuPublisher {
 public:
  ImuPublisher(rclcpp::Node& node, const std::string& topic);

  void publish(const ImuData& imu, const rclcpp::Time& stamp);

 private:
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
};

class CameraPublisher {
 public:
  CameraPublisher(rclcpp::Node& node, const std::string& rgb_topic, const std::string& depth_topic,
                  const std::string& camera_info_topic, bool enable_rgb, bool enable_depth);

  void publish(const CameraData& camera, const rclcpp::Time& stamp);

 private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_;
  bool depth_enabled_{false};
};

class LidarPublisher {
 public:
  LidarPublisher(rclcpp::Node& node, const std::string& topic);

  void publish(const LidarData& lidar, const rclcpp::Time& stamp);

 private:
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
};

// ---------------------------------------------------------------------------
// SensorBridge — thin coordinator that owns per-sensor publishers.
// ---------------------------------------------------------------------------

class SensorBridge {
 public:
  SensorBridge(const std::string& node_name, const std::vector<ImuData>* imus,
               const std::vector<CameraData>* cameras, const std::vector<LidarData>* lidars);

  void set_time(const rclcpp::Time& sim_time);
  bool publish_imu(const ImuData& imu);
  bool publish_camera(const CameraData& camera);
  bool publish_lidar(const LidarData& lidar);

 private:
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Time sim_time_{0, 0, RCL_ROS_TIME};
  std::unordered_map<std::string, ImuPublisher> imus_;
  std::unordered_map<std::string, CameraPublisher> cameras_;
  std::unordered_map<std::string, LidarPublisher> lidars_;
};

}  // namespace mujoco_hardware
