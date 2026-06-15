#include "mujoco_hardware/mujoco_hardware_interface.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "mujoco_hardware/sensor_bridge.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

namespace mujoco_hardware {
namespace {

rclcpp::Time to_ros_time(double sim_seconds) {
  const auto nanoseconds = static_cast<int64_t>(sim_seconds * 1e9);
  return rclcpp::Time(nanoseconds, RCL_ROS_TIME);
}

std::string parameter_or(const std::unordered_map<std::string, std::string>& parameters,
                         const std::string& key, const std::string& fallback = std::string()) {
  const auto it = parameters.find(key);
  return it == parameters.end() ? fallback : it->second;
}

bool split_interface_key(const std::string& interface_key, std::string* joint_name,
                         std::string* interface_name) {
  const auto separator = interface_key.find('/');
  if (separator == std::string::npos || separator == 0 || separator + 1 >= interface_key.size()) {
    return false;
  }
  *joint_name = interface_key.substr(0, separator);
  *interface_name = interface_key.substr(separator + 1);
  return true;
}

void assign_camera_intrinsics(const sensor_msgs::msg::CameraInfo& source, CameraData* target) {
  if (target == nullptr) {
    return;
  }
  target->intrinsics.distortion_model = source.distortion_model;
  target->intrinsics.distortion_coefficients = source.d;
  target->intrinsics.intrinsic_matrix = source.k;
  target->intrinsics.rectification_matrix = source.r;
  target->intrinsics.projection_matrix = source.p;
}

}  // namespace

MuJoCoHardwareInterface::MuJoCoHardwareInterface() = default;

MuJoCoHardwareInterface::~MuJoCoHardwareInterface() = default;

JointData* MuJoCoHardwareInterface::find_joint(const std::string& joint_name) {
  const auto it = std::find_if(config_.joints.begin(), config_.joints.end(),
                               [&](const JointData& joint) { return joint.name == joint_name; });
  return it == config_.joints.end() ? nullptr : &(*it);
}

const JointData* MuJoCoHardwareInterface::find_joint(const std::string& joint_name) const {
  const auto it = std::find_if(config_.joints.begin(), config_.joints.end(),
                               [&](const JointData& joint) { return joint.name == joint_name; });
  return it == config_.joints.end() ? nullptr : &(*it);
}

void MuJoCoHardwareInterface::initialize_command_buffers() {
  for (auto& joint : config_.joints) {
    const auto it = active_joint_modes_.find(joint.name);
    const auto mode = it == active_joint_modes_.end()
                          ? mujoco_simulation::CommandInterfaceType::None
                          : it->second;
    if (mode == mujoco_simulation::CommandInterfaceType::Position) {
      joint.command.position = joint.state.position;
    } else if (mode == mujoco_simulation::CommandInterfaceType::Velocity) {
      joint.command.velocity = 0.0;
    } else if (mode == mujoco_simulation::CommandInterfaceType::Effort) {
      joint.command.effort = 0.0;
    }
  }
}

bool MuJoCoHardwareInterface::update_runtime_state() {
  std::string err;
  const auto stamp = to_ros_time(simulation_ == nullptr ? 0.0 : simulation_->simulation_time());
  sensor_bridge_->set_time(stamp);
  for (auto& joint : config_.joints) {
    if (!simulation_->read_joint(joint.name, &joint.state, err)) {
      return false;
    }
  }
  for (auto& imu : config_.imus) {
    if (!simulation_->read_imu(imu.name, &imu.state, err)) {
      return false;
    }
    if (!sensor_bridge_->publish_imu(imu)) {
      return false;
    }
  }
  for (auto& camera : config_.cameras) {
    if (!simulation_->read_camera(camera.name, &camera.state, err)) {
      return false;
    }
    if (!sensor_bridge_->publish_camera(camera)) {
      return false;
    }
  }
  for (auto& lidar : config_.lidars) {
    if (!simulation_->read_lidar(lidar.name, &lidar.state, err)) {
      return false;
    }
    if (!sensor_bridge_->publish_lidar(lidar)) {
      return false;
    }
  }
  for (auto& mobile_base : config_.mobile_bases) {
    if (!simulation_->read_mobile_base(mobile_base.name, &mobile_base.state, err)) {
      return false;
    }
  }
  return true;
}

hardware_interface::CallbackReturn MuJoCoHardwareInterface::on_init(
    const hardware_interface::HardwareInfo& hardware_info) {
  if (hardware_interface::SystemInterface::on_init(hardware_info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::string error_message;
  if (!parse_hardware_config(hardware_info, &config_, error_message)) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  sensor_node_name_ = parameter_or(hardware_info.hardware_parameters, "sensor_node_name",
                                   hardware_info.name + "_sensor_bridge");

  simulation_ = std::make_unique<mujoco_simulation::MuJoCoSimulation>();
  if (!simulation_->initialize(config_.simulation, error_message)) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  for (const auto& joint : config_.joints) {
    if (!simulation_->register_joint(joint.info, error_message)) {
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  for (const auto& imu : config_.imus) {
    if (!simulation_->register_imu(imu.info, error_message)) {
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  for (const auto& camera : config_.cameras) {
    if (!simulation_->register_camera(camera.info, error_message)) {
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  for (const auto& lidar : config_.lidars) {
    if (!simulation_->register_lidar(lidar.info, error_message)) {
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  for (const auto& mobile_base : config_.mobile_bases) {
    if (!simulation_->register_mobile_base(mobile_base.info, error_message)) {
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  for (auto& camera : config_.cameras) {
    sensor_msgs::msg::CameraInfo camera_info;
    if (!fill_camera_info(camera.info.camera_name, camera.info.width, camera.info.height,
                          &camera_info, error_message)) {
      return hardware_interface::CallbackReturn::ERROR;
    }
    assign_camera_intrinsics(camera_info, &camera);
  }

  sensor_bridge_ = std::make_unique<SensorBridge>(sensor_node_name_, &config_.imus,
                                                  &config_.cameras, &config_.lidars);

  active_joint_modes_.clear();
  pending_mode_switch_.next_modes.clear();
  pending_mode_switch_.valid = false;
  for (const auto& joint : config_.joints) {
    active_joint_modes_[joint.name] = mujoco_simulation::CommandInterfaceType::None;
    pending_mode_switch_.next_modes[joint.name] = mujoco_simulation::CommandInterfaceType::None;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MuJoCoHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto& joint : config_.joints) {
    for (const auto& interface_name : joint.state_interfaces) {
      if (interface_name == hardware_interface::HW_IF_POSITION) {
        state_interfaces.emplace_back(joint.name, interface_name, &joint.state.position);
      } else if (interface_name == hardware_interface::HW_IF_VELOCITY) {
        state_interfaces.emplace_back(joint.name, interface_name, &joint.state.velocity);
      } else if (interface_name == hardware_interface::HW_IF_EFFORT) {
        state_interfaces.emplace_back(joint.name, interface_name, &joint.state.effort);
      }
    }
  }

  for (auto& imu : config_.imus) {
    for (const auto& interface_name : imu.state_interfaces) {
      if (interface_name == "orientation.x") {
        state_interfaces.emplace_back(imu.name, interface_name, &imu.state.orientation[0]);
      } else if (interface_name == "orientation.y") {
        state_interfaces.emplace_back(imu.name, interface_name, &imu.state.orientation[1]);
      } else if (interface_name == "orientation.z") {
        state_interfaces.emplace_back(imu.name, interface_name, &imu.state.orientation[2]);
      } else if (interface_name == "orientation.w") {
        state_interfaces.emplace_back(imu.name, interface_name, &imu.state.orientation[3]);
      } else if (interface_name == "angular_velocity.x") {
        state_interfaces.emplace_back(imu.name, interface_name, &imu.state.angular_velocity[0]);
      } else if (interface_name == "angular_velocity.y") {
        state_interfaces.emplace_back(imu.name, interface_name, &imu.state.angular_velocity[1]);
      } else if (interface_name == "angular_velocity.z") {
        state_interfaces.emplace_back(imu.name, interface_name, &imu.state.angular_velocity[2]);
      } else if (interface_name == "linear_acceleration.x") {
        state_interfaces.emplace_back(imu.name, interface_name, &imu.state.linear_acceleration[0]);
      } else if (interface_name == "linear_acceleration.y") {
        state_interfaces.emplace_back(imu.name, interface_name, &imu.state.linear_acceleration[1]);
      } else if (interface_name == "linear_acceleration.z") {
        state_interfaces.emplace_back(imu.name, interface_name, &imu.state.linear_acceleration[2]);
      }
    }
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
MuJoCoHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto& joint : config_.joints) {
    for (const auto& interface_name : joint.command_interfaces) {
      if (interface_name == hardware_interface::HW_IF_POSITION) {
        command_interfaces.emplace_back(joint.name, interface_name, &joint.command.position);
      } else if (interface_name == hardware_interface::HW_IF_VELOCITY) {
        command_interfaces.emplace_back(joint.name, interface_name, &joint.command.velocity);
      } else if (interface_name == hardware_interface::HW_IF_EFFORT) {
        command_interfaces.emplace_back(joint.name, interface_name, &joint.command.effort);
      }
    }
  }
  return command_interfaces;
}

hardware_interface::return_type MuJoCoHardwareInterface::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) {
  pending_mode_switch_.next_modes = active_joint_modes_;
  pending_mode_switch_.valid = false;

  std::unordered_map<std::string, mujoco_simulation::CommandInterfaceType> start_modes_by_joint;
  std::unordered_map<std::string, mujoco_simulation::CommandInterfaceType> stop_modes_by_joint;
  std::map<mujoco_simulation::CommandInterfaceType, std::size_t> start_counts;
  std::map<mujoco_simulation::CommandInterfaceType, std::size_t> stop_counts;
  std::string joint_name;
  std::string interface_name;

  for (const auto& stop_interface : stop_interfaces) {
    if (!split_interface_key(stop_interface, &joint_name, &interface_name)) {
      return hardware_interface::return_type::ERROR;
    }
    const auto* joint = find_joint(joint_name);
    if (joint == nullptr || !is_joint_command_interface(interface_name)) {
      return hardware_interface::return_type::ERROR;
    }
    if (std::find(joint->command_interfaces.begin(), joint->command_interfaces.end(),
                  interface_name) == joint->command_interfaces.end()) {
      return hardware_interface::return_type::ERROR;
    }
    const auto mode = to_joint_control_mode(interface_name);
    stop_modes_by_joint[joint_name] = mode;
    ++stop_counts[mode];
  }

  for (const auto& start_interface : start_interfaces) {
    if (!split_interface_key(start_interface, &joint_name, &interface_name)) {
      return hardware_interface::return_type::ERROR;
    }
    const auto* joint = find_joint(joint_name);
    if (joint == nullptr || !is_joint_command_interface(interface_name)) {
      return hardware_interface::return_type::ERROR;
    }
    if (std::find(joint->command_interfaces.begin(), joint->command_interfaces.end(),
                  interface_name) == joint->command_interfaces.end()) {
      return hardware_interface::return_type::ERROR;
    }
    const auto mode = to_joint_control_mode(interface_name);
    const auto [it, inserted] = start_modes_by_joint.emplace(joint_name, mode);
    if (!inserted && it->second != mode) {
      return hardware_interface::return_type::ERROR;
    }
    ++start_counts[mode];
  }

  const auto joint_count = config_.joints.size();
  for (const auto& [mode, count] : start_counts) {
    if (mode != mujoco_simulation::CommandInterfaceType::None && count != joint_count) {
      return hardware_interface::return_type::ERROR;
    }
  }
  for (const auto& [mode, count] : stop_counts) {
    if (mode != mujoco_simulation::CommandInterfaceType::None && count != joint_count) {
      return hardware_interface::return_type::ERROR;
    }
  }
  if (start_counts.size() > 1U) {
    return hardware_interface::return_type::ERROR;
  }

  for (const auto& joint : config_.joints) {
    if (stop_modes_by_joint.find(joint.name) != stop_modes_by_joint.end()) {
      pending_mode_switch_.next_modes[joint.name] = mujoco_simulation::CommandInterfaceType::None;
    }
  }
  for (const auto& [name, mode] : start_modes_by_joint) {
    pending_mode_switch_.next_modes[name] = mode;
  }

  pending_mode_switch_.valid = true;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MuJoCoHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string>& /*start_interfaces*/,
    const std::vector<std::string>& /*stop_interfaces*/) {
  if (!pending_mode_switch_.valid) {
    return hardware_interface::return_type::ERROR;
  }

  std::string err;
  for (const auto& [joint_name, mode] : pending_mode_switch_.next_modes) {
    if (!simulation_->configure_joint_command_mode(joint_name, mode, err)) {
      return hardware_interface::return_type::ERROR;
    }
  }
  active_joint_modes_ = pending_mode_switch_.next_modes;
  initialize_command_buffers();
  pending_mode_switch_.valid = false;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MuJoCoHardwareInterface::read(const rclcpp::Time&,
                                                              const rclcpp::Duration&) {
  return update_runtime_state() ? hardware_interface::return_type::OK
                                : hardware_interface::return_type::ERROR;
}

hardware_interface::return_type MuJoCoHardwareInterface::write(const rclcpp::Time&,
                                                               const rclcpp::Duration&) {
  std::string err;
  for (const auto& joint : config_.joints) {
    const auto it = active_joint_modes_.find(joint.name);
    const auto mode = it == active_joint_modes_.end()
                          ? mujoco_simulation::CommandInterfaceType::None
                          : it->second;
    if (mode == mujoco_simulation::CommandInterfaceType::None) {
      continue;
    }
    if (!simulation_->write_joint(joint.command, err)) {
      return hardware_interface::return_type::ERROR;
    }
  }
  for (const auto& mobile_base : config_.mobile_bases) {
    if (!simulation_->write_mobile_base(mobile_base.name, mobile_base.command, err)) {
      return hardware_interface::return_type::ERROR;
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn MuJoCoHardwareInterface::on_activate(
    const rclcpp_lifecycle::State&) {
  simulation_->start();
  if (!update_runtime_state()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  initialize_command_buffers();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MuJoCoHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State&) {
  simulation_->stop();
  for (auto& [joint_name, mode] : active_joint_modes_) {
    mode = mujoco_simulation::CommandInterfaceType::None;
    pending_mode_switch_.next_modes[joint_name] = mujoco_simulation::CommandInterfaceType::None;
  }
  pending_mode_switch_.valid = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool MuJoCoHardwareInterface::fill_camera_info(const std::string& camera_name, int width,
                                               int height, sensor_msgs::msg::CameraInfo* info,
                                               std::string& error_message) const {
  if (info == nullptr) {
    error_message = "CameraInfo output pointer must not be null.";
    return false;
  }
  if (width <= 0 || height <= 0) {
    error_message = "Camera dimensions must be positive.";
    return false;
  }
  if (simulation_ == nullptr) {
    error_message = "MuJoCo simulation is not available.";
    return false;
  }

  double fovy_degrees = 0.0;
  if (!simulation_->camera_fovy(camera_name, &fovy_degrees, error_message)) {
    return false;
  }

  const double aspect = static_cast<double>(width) / static_cast<double>(height);
  const double fovy_radians = fovy_degrees * M_PI / 180.0;
  const double fy = static_cast<double>(height) / (2.0 * std::tan(fovy_radians / 2.0));
  const double fovx_radians = 2.0 * std::atan(aspect * std::tan(fovy_radians / 2.0));
  const double fx = static_cast<double>(width) / (2.0 * std::tan(fovx_radians / 2.0));
  const double cx = (static_cast<double>(width) - 1.0) / 2.0;
  const double cy = (static_cast<double>(height) - 1.0) / 2.0;
  info->width = static_cast<uint32_t>(width);
  info->height = static_cast<uint32_t>(height);
  info->distortion_model = "plumb_bob";
  info->d.assign(5, 0.0);
  info->k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
  info->r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  info->p = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};
  error_message.clear();
  return true;
}

}  // namespace mujoco_hardware
PLUGINLIB_EXPORT_CLASS(mujoco_hardware::MuJoCoHardwareInterface,
                       hardware_interface::SystemInterface)
