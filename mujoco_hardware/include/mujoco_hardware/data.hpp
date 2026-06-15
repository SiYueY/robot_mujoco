#pragma once

#include <array>
#include <string>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "mujoco_simulation/hardware/camera.hpp"
#include "mujoco_simulation/hardware/imu.hpp"
#include "mujoco_simulation/hardware/joint.hpp"
#include "mujoco_simulation/hardware/lidar.hpp"
#include "mujoco_simulation/hardware/mobile_base.hpp"
#include "mujoco_simulation/mujoco_simulation.hpp"

namespace mujoco_hardware {

struct JointData {
  std::string name;
  std::vector<std::string> command_interfaces;
  std::vector<std::string> state_interfaces;
  mujoco_simulation::JointInfo info;
  mujoco_simulation::JointCommand command;
  mujoco_simulation::JointState state;
};

struct ImuData {
  std::string name;
  std::vector<std::string> state_interfaces;
  std::string frame_id;
  std::string topic;
  mujoco_simulation::ImuInfo info;
  mujoco_simulation::ImuState state;
};

struct CameraIntrinsics {
  std::string distortion_model{"plumb_bob"};
  std::vector<double> distortion_coefficients;
  std::array<double, 9> intrinsic_matrix{};
  std::array<double, 9> rectification_matrix{};
  std::array<double, 12> projection_matrix{};
};

struct CameraData {
  std::string name;

  // ROS frame used by Image and CameraInfo headers.
  std::string frame_id;

  // ROS topics published for this camera.
  std::string rgb_topic;
  std::string depth_topic;
  std::string camera_info_topic;

  // Pinhole intrinsics derived from MuJoCo camera geometry and output resolution.
  CameraIntrinsics intrinsics;

  mujoco_simulation::CameraSpec info;
  mujoco_simulation::CameraState state;
};

struct LidarData {
  std::string name;
  std::string frame_id;
  std::string topic;
  mujoco_simulation::LidarInfo info;
  mujoco_simulation::LidarState state;
};

struct MobileBaseData {
  std::string name;
  mujoco_simulation::MobileBaseInfo info;
  mujoco_simulation::MobileBaseCommand command;
  mujoco_simulation::MobileBaseState state;
};

struct HardwareConfig {
  mujoco_simulation::SimulationConfig simulation;
  std::vector<JointData> joints;
  std::vector<ImuData> imus;
  std::vector<CameraData> cameras;
  std::vector<LidarData> lidars;
  std::vector<MobileBaseData> mobile_bases;
};

bool parse_hardware_config(const hardware_interface::HardwareInfo& hardware_info,
                           HardwareConfig* config, std::string& error_message);

mujoco_simulation::CommandInterfaceType to_joint_control_mode(const std::string& interface_name);
bool is_joint_command_interface(const std::string& interface_name);
bool is_joint_state_interface(const std::string& interface_name);
bool is_imu_state_interface(const std::string& interface_name);

}  // namespace mujoco_hardware
