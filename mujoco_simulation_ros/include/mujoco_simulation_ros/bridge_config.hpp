#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace mujoco_simulation_ros {

struct ImuPublisherConfig {
  std::string name;
  std::string frame_id;
  std::string topic;
};

struct CameraPublisherConfig {
  std::string name;
  std::string frame_id;
  std::string rgb_topic;
  std::string depth_topic;
  std::string camera_info_topic;
  std::uint32_t width{0};
  std::uint32_t height{0};
  bool enable_rgb{false};
  bool enable_depth{false};
};

struct LidarPublisherConfig {
  std::string name;
  std::string frame_id;
  std::string topic;
};

struct SimulationRosBridgeConfig {
  std::string node_name;
  std::vector<ImuPublisherConfig> imus;
  std::vector<CameraPublisherConfig> cameras;
  std::vector<LidarPublisherConfig> lidars;
};

}  // namespace mujoco_simulation_ros
