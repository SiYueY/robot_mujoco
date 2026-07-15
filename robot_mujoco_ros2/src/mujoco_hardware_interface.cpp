#include "robot_mujoco_ros2/mujoco_hardware_interface.hpp"

#include <algorithm>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <unordered_map>
#include <utility>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_mujoco_ros2/simulation_ros_bridge.hpp"

namespace robot_mujoco_ros2 {
namespace {

using mujoco_simulation::ResultCode;

const rclcpp::Logger& hardware_logger() {
  static const rclcpp::Logger logger = rclcpp::get_logger("robot_mujoco_ros2");
  return logger;
}

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

using JointInterfaceSet = std::set<std::string>;

std::optional<mujoco_simulation::ControlMode> control_mode_for(
    const JointInterfaceSet& interfaces) {
  if (interfaces == JointInterfaceSet{hardware_interface::HW_IF_POSITION}) {
    return mujoco_simulation::ControlMode::Position;
  }
  if (interfaces == JointInterfaceSet{hardware_interface::HW_IF_VELOCITY}) {
    return mujoco_simulation::ControlMode::Velocity;
  }
  if (interfaces == JointInterfaceSet{hardware_interface::HW_IF_EFFORT}) {
    return mujoco_simulation::ControlMode::Effort;
  }
  if (interfaces == JointInterfaceSet{hardware_interface::HW_IF_POSITION,
                                      hardware_interface::HW_IF_VELOCITY,
                                      hardware_interface::HW_IF_EFFORT, "stiffness", "damping"}) {
    return mujoco_simulation::ControlMode::Hybrid;
  }
  return std::nullopt;
}

const char* result_code_name(mujoco_simulation::ResultCode code) {
  switch (code) {
    case mujoco_simulation::ResultCode::Ok:
      return "Ok";
    case mujoco_simulation::ResultCode::InvalidArgument:
      return "InvalidArgument";
    case mujoco_simulation::ResultCode::AlreadyExists:
      return "AlreadyExists";
    case mujoco_simulation::ResultCode::InvalidState:
      return "InvalidState";
    case mujoco_simulation::ResultCode::FailedPrecondition:
      return "FailedPrecondition";
    case mujoco_simulation::ResultCode::NotFound:
      return "NotFound";
    case mujoco_simulation::ResultCode::ModelLoadFailed:
      return "ModelLoadFailed";
    case mujoco_simulation::ResultCode::ModelValidationFailed:
      return "ModelValidationFailed";
    case mujoco_simulation::ResultCode::BindingFailed:
      return "BindingFailed";
    case mujoco_simulation::ResultCode::CommandRejected:
      return "CommandRejected";
    case mujoco_simulation::ResultCode::RenderFailed:
      return "RenderFailed";
    case mujoco_simulation::ResultCode::ThreadFailed:
      return "ThreadFailed";
    case mujoco_simulation::ResultCode::Timeout:
      return "Timeout";
    case mujoco_simulation::ResultCode::Internal:
      return "Internal";
  }
  return "Unknown";
}

void copy_imu_state_rt(const mujoco_simulation::ImuState& source,
                       mujoco_simulation::ImuState* target) {
  if (target == nullptr) {
    return;
  }
  target->sequence = source.sequence;
  target->timestamp_ns = source.timestamp_ns;
  target->orientation = source.orientation;
  target->orientation_covariance = source.orientation_covariance;
  target->angular_velocity = source.angular_velocity;
  target->angular_velocity_covariance = source.angular_velocity_covariance;
  target->linear_acceleration = source.linear_acceleration;
  target->linear_acceleration_covariance = source.linear_acceleration_covariance;
}

bool copy_lidar_state_rt(const mujoco_simulation::LidarState& source,
                         mujoco_simulation::LidarState* target) {
  if (target == nullptr || target->ranges.size() < source.ranges.size() ||
      target->intensities.size() < source.ranges.size()) {
    return false;
  }
  target->sequence = source.sequence;
  target->timestamp_ns = source.timestamp_ns;
  target->angle_min = source.angle_min;
  target->angle_max = source.angle_max;
  target->angle_increment = source.angle_increment;
  target->time_increment = source.time_increment;
  target->scan_time = source.scan_time;
  target->range_min = source.range_min;
  target->range_max = source.range_max;
  std::copy(source.ranges.begin(), source.ranges.end(), target->ranges.begin());
  if (source.intensities.empty()) {
    std::fill(target->intensities.begin(),
              target->intensities.begin() + static_cast<std::ptrdiff_t>(source.ranges.size()), 0.0);
  } else {
    std::copy(source.intensities.begin(), source.intensities.end(), target->intensities.begin());
  }
  return true;
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
    const auto it = active_joint_interfaces_.find(joint.name);
    const auto mode =
        it == active_joint_interfaces_.end() ? std::nullopt : control_mode_for(it->second);
    if (mode == mujoco_simulation::ControlMode::Position ||
        mode == mujoco_simulation::ControlMode::Hybrid) {
      joint.command.position = joint.state.position;
    }
    if (mode == mujoco_simulation::ControlMode::Velocity ||
        mode == mujoco_simulation::ControlMode::Hybrid) {
      joint.command.velocity = 0.0;
    }
    if (mode == mujoco_simulation::ControlMode::Effort ||
        mode == mujoco_simulation::ControlMode::Hybrid) {
      joint.command.effort = 0.0;
    }
    joint.command.stiffness = joint.config.position_stiffness;
    joint.command.damping = joint.config.position_damping;
  }
}

mujoco_simulation::ResultCode MuJoCoHardwareInterface::request_start_status() {
  std::lock_guard<std::mutex> lock(simulation_control_mutex_);
  if (simulation_ == nullptr) {
    return mujoco_simulation::ResultCode::InvalidState;
  }
  if (!simulation_started_) {
    const mujoco_simulation::ResultCode status = simulation_->start();
    if (status != ResultCode::Ok) {
      return status;
    }
    simulation_started_ = true;
  }
  return mujoco_simulation::ResultCode::Ok;
}

mujoco_simulation::ResultCode MuJoCoHardwareInterface::request_stop_status() {
  std::lock_guard<std::mutex> lock(simulation_control_mutex_);
  if (simulation_ == nullptr) {
    return mujoco_simulation::ResultCode::InvalidState;
  }
  if (simulation_started_) {
    const mujoco_simulation::ResultCode status = simulation_->stop();
    if (status != ResultCode::Ok) {
      return status;
    }
    simulation_started_ = false;
  }
  return mujoco_simulation::ResultCode::Ok;
}

mujoco_simulation::ResultCode MuJoCoHardwareInterface::request_pause_status() {
  std::lock_guard<std::mutex> lock(simulation_control_mutex_);
  if (simulation_ == nullptr) {
    return mujoco_simulation::ResultCode::InvalidState;
  }
  if (!simulation_started_) {
    return mujoco_simulation::ResultCode::InvalidState;
  }
  return simulation_->pause();
}

mujoco_simulation::ResultCode MuJoCoHardwareInterface::request_resume_status() {
  std::lock_guard<std::mutex> lock(simulation_control_mutex_);
  if (simulation_ == nullptr) {
    return mujoco_simulation::ResultCode::InvalidState;
  }
  if (!simulation_started_) {
    return mujoco_simulation::ResultCode::InvalidState;
  }
  return simulation_->resume();
}

mujoco_simulation::ResultCode MuJoCoHardwareInterface::request_step_status(uint32_t steps) {
  std::lock_guard<std::mutex> lock(simulation_control_mutex_);
  if (simulation_ == nullptr) {
    return mujoco_simulation::ResultCode::InvalidState;
  }
  return simulation_->step(steps);
}

mujoco_simulation::ResultCode MuJoCoHardwareInterface::request_set_realtime_factor_status(
    double realtime_factor) {
  std::lock_guard<std::mutex> lock(simulation_control_mutex_);
  if (simulation_ == nullptr) {
    return mujoco_simulation::ResultCode::InvalidState;
  }
  return simulation_->set_realtime_factor(realtime_factor);
}

mujoco_simulation::ResultCode MuJoCoHardwareInterface::request_keyframe_reset_status(
    const std::string& keyframe) {
  std::lock_guard<std::mutex> lock(simulation_control_mutex_);
  if (simulation_ == nullptr) {
    return mujoco_simulation::ResultCode::InvalidState;
  }
  return simulation_->reset_to_keyframe_name(keyframe);
}

mujoco_simulation::ResultCode MuJoCoHardwareInterface::request_reset_status() {
  std::lock_guard<std::mutex> lock(simulation_control_mutex_);
  if (simulation_ == nullptr) {
    return mujoco_simulation::ResultCode::InvalidState;
  }
  return simulation_->request_reset();
}

mujoco_simulation::ResultCode MuJoCoHardwareInterface::update_runtime_state() {
  const std::shared_ptr<const mujoco_simulation::StateSnapshot> snapshot =
      simulation_ == nullptr ? nullptr : simulation_->state_snapshot();
  if (snapshot == nullptr) {
    return mujoco_simulation::ResultCode::InvalidState;
  }
  return update_runtime_state_from_snapshot(*snapshot);
}

mujoco_simulation::ResultCode MuJoCoHardwareInterface::update_runtime_state_from_snapshot(
    const mujoco_simulation::StateSnapshot& snapshot) {
  for (auto& joint : config_.joints) {
    const auto it = snapshot.joints.find(joint.name);
    if (it == snapshot.joints.end()) {
      return mujoco_simulation::ResultCode::NotFound;
    }
    joint.state = it->second;
  }
  for (auto& imu : config_.imus) {
    const auto it = snapshot.imus.find(imu.name);
    if (it == snapshot.imus.end()) {
      return mujoco_simulation::ResultCode::NotFound;
    }
    imu.state = it->second;
  }
  for (auto& lidar : config_.lidars) {
    const auto it = snapshot.lidars.find(lidar.name);
    if (it == snapshot.lidars.end()) {
      return mujoco_simulation::ResultCode::NotFound;
    }
    lidar.state = it->second;
  }
  for (auto& mobile_base : config_.mobile_bases) {
    const auto it = snapshot.mobile_bases.find(mobile_base.name);
    if (it == snapshot.mobile_bases.end()) {
      return mujoco_simulation::ResultCode::NotFound;
    }
    mobile_base.state = it->second;
  }
  return mujoco_simulation::ResultCode::Ok;
}

mujoco_simulation::ResultCode MuJoCoHardwareInterface::publish_snapshot_to_channel(
    const std::shared_ptr<const mujoco_simulation::StateSnapshot>& snapshot) {
  if (snapshot == nullptr) {
    return mujoco_simulation::ResultCode::InvalidArgument;
  }
  if (ros_bridge_ == nullptr) {
    return mujoco_simulation::ResultCode::InvalidState;
  }
  if (simulation_ == nullptr) {
    return mujoco_simulation::ResultCode::InvalidState;
  }
  if (system_state_ != SystemState::kActive) {
    return mujoco_simulation::ResultCode::Ok;
  }

  const auto stamp = to_ros_time(snapshot->simulation_time);
  publish_bundle_.sim_time = stamp;
  for (std::size_t index = 0; index < config_.imus.size(); ++index) {
    const auto& imu = config_.imus[index];
    const auto it = snapshot->imus.find(imu.name);
    if (it == snapshot->imus.end()) {
      return mujoco_simulation::ResultCode::NotFound;
    }
    copy_imu_state_rt(it->second, &publish_imu_states_[index]);
    publish_bundle_.imus[index].publisher_index = index;
    copy_imu_state_rt(publish_imu_states_[index], &publish_bundle_.imus[index].state);
  }
  for (std::size_t index = 0; index < config_.cameras.size(); ++index) {
    const auto& camera = config_.cameras[index];
    mujoco_simulation::CameraState state;
    if (!simulation_->camera_state(camera.name, &state)) {
      return mujoco_simulation::ResultCode::NotFound;
    }
    publish_camera_states_[index] = std::move(state);
    publish_bundle_.cameras[index].publisher_index = index;
    publish_bundle_.cameras[index].state = &publish_camera_states_[index];
  }
  for (std::size_t index = 0; index < config_.lidars.size(); ++index) {
    const auto& lidar = config_.lidars[index];
    const auto it = snapshot->lidars.find(lidar.name);
    if (it == snapshot->lidars.end()) {
      return mujoco_simulation::ResultCode::NotFound;
    }
    if (!copy_lidar_state_rt(it->second, &publish_lidar_states_[index]) ||
        !copy_lidar_state_rt(publish_lidar_states_[index], &publish_bundle_.lidars[index].state)) {
      return mujoco_simulation::ResultCode::FailedPrecondition;
    }
    publish_bundle_.lidars[index].publisher_index = index;
  }
  return ros_bridge_->enqueue_publish_bundle(publish_bundle_);
}

hardware_interface::CallbackReturn MuJoCoHardwareInterface::on_init(
    const hardware_interface::HardwareInfo& hardware_info) {
  if (hardware_interface::SystemInterface::on_init(hardware_info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::string error_message;
  AdapterConfigBundle adapter_config;
  if (!build_adapter_config(hardware_info, &adapter_config, error_message)) {
    RCLCPP_ERROR(hardware_logger(), "Failed to build MuJoCo hardware adapter config: %s",
                 error_message.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  config_ = std::move(adapter_config.runtime_config);
  hardware_mapping_config_ = std::move(adapter_config.hardware_mapping_config);

  sensor_node_name_ = parameter_or(hardware_info.hardware_parameters, "sensor_node_name",
                                   hardware_info.name + "_simulation_ros_bridge");
  system_state_ = SystemState::kInactive;

  simulation_ = std::make_unique<mujoco_simulation::Simulation>();
  const mujoco_simulation::ResultCode initialize_status =
      simulation_->initialize(config_.simulation);
  if (initialize_status != ResultCode::Ok) {
    RCLCPP_ERROR(hardware_logger(), "Failed to initialize MuJoCo simulation for hardware '%s': %s",
                 hardware_info.name.c_str(), result_code_name(initialize_status));
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (auto& camera : config_.cameras) {
    if (!simulation_->camera_state(camera.name, &camera.state)) {
      RCLCPP_ERROR(hardware_logger(),
                   "Failed to initialize camera state for hardware '%s', camera '%s'",
                   hardware_info.name.c_str(), camera.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  ros_bridge_ = std::make_unique<robot_mujoco_ros2::SimulationRosBridge>(
      [&]() {
        auto bridge_config = adapter_config.ros_interface_config;
        bridge_config.node_name = sensor_node_name_;
        return bridge_config;
      }(),
      nullptr, [this]() { return system_state_ == SystemState::kActive; },
      [this]() { return request_reset_status(); }, [this]() { return request_start_status(); },
      [this]() { return request_stop_status(); }, [this]() { return request_pause_status(); },
      [this]() { return request_resume_status(); },
      [this](uint32_t steps) { return request_step_status(steps); },
      [this](double realtime_factor) {
        return request_set_realtime_factor_status(realtime_factor);
      },
      [this](const std::string& keyframe) { return request_keyframe_reset_status(keyframe); });
  publish_imu_states_.resize(config_.imus.size());
  publish_lidar_states_.resize(config_.lidars.size());
  publish_camera_states_.resize(config_.cameras.size());
  publish_bundle_.imus.resize(config_.imus.size());
  publish_bundle_.lidars.resize(config_.lidars.size());
  publish_bundle_.cameras.resize(config_.cameras.size());
  for (std::size_t i = 0; i < config_.lidars.size(); ++i) {
    const std::size_t sample_count =
        i < adapter_config.ros_interface_config.lidars.size()
            ? adapter_config.ros_interface_config.lidars[i].sample_count
            : 0U;
    publish_lidar_states_[i].ranges.resize(sample_count, 0.0);
    publish_lidar_states_[i].intensities.resize(sample_count, 0.0);
    publish_bundle_.lidars[i].state.ranges.resize(sample_count, 0.0);
    publish_bundle_.lidars[i].state.intensities.resize(sample_count, 0.0);
  }
  simulation_->set_snapshot_observer(
      [this](std::shared_ptr<const mujoco_simulation::StateSnapshot> snapshot) {
        const mujoco_simulation::ResultCode status = publish_snapshot_to_channel(snapshot);
        if (status != ResultCode::Ok) {
          RCLCPP_ERROR(hardware_logger(), "publish snapshot failed: %s", result_code_name(status));
        }
      });

  active_joint_interfaces_.clear();
  pending_mode_switch_.next_interfaces.clear();
  pending_mode_switch_.valid = false;
  for (auto& joint : config_.joints) {
    active_joint_interfaces_[joint.name] = {};
    pending_mode_switch_.next_interfaces[joint.name] = {};
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
      } else if (interface_name == "stiffness") {
        command_interfaces.emplace_back(joint.name, interface_name, &joint.command.stiffness);
      } else if (interface_name == "damping") {
        command_interfaces.emplace_back(joint.name, interface_name, &joint.command.damping);
      }
    }
  }
  return command_interfaces;
}

hardware_interface::return_type MuJoCoHardwareInterface::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) {
  pending_mode_switch_.next_interfaces = active_joint_interfaces_;
  pending_mode_switch_.valid = false;
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
    pending_mode_switch_.next_interfaces[joint_name].erase(interface_name);
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
    pending_mode_switch_.next_interfaces[joint_name].insert(interface_name);
  }
  for (const auto& [name, interfaces] : pending_mode_switch_.next_interfaces) {
    if (!interfaces.empty() && !control_mode_for(interfaces).has_value()) {
      return hardware_interface::return_type::ERROR;
    }
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

  active_joint_interfaces_ = pending_mode_switch_.next_interfaces;
  initialize_command_buffers();
  pending_mode_switch_.valid = false;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MuJoCoHardwareInterface::read(const rclcpp::Time&,
                                                              const rclcpp::Duration&) {
  const mujoco_simulation::ResultCode status = update_runtime_state();
  if (status != ResultCode::Ok) {
    RCLCPP_ERROR(hardware_logger(), "read failed: %s", result_code_name(status));
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MuJoCoHardwareInterface::write(const rclcpp::Time&,
                                                               const rclcpp::Duration&) {
  for (auto& joint : config_.joints) {
    const auto it = active_joint_interfaces_.find(joint.name);
    const auto mode =
        it == active_joint_interfaces_.end() ? std::nullopt : control_mode_for(it->second);
    if (!mode.has_value()) {
      continue;
    }
    joint.command.mode = *mode;
    const mujoco_simulation::ResultCode status = simulation_->set_joint_command(joint.command);
    if (status != ResultCode::Ok) {
      RCLCPP_ERROR(hardware_logger(), "set_joint_command failed: %s", result_code_name(status));
      return hardware_interface::return_type::ERROR;
    }
  }
  for (const auto& mobile_base : config_.mobile_bases) {
    const mujoco_simulation::ResultCode status =
        simulation_->set_mobile_base_command(mobile_base.name, mobile_base.command);
    if (status != ResultCode::Ok) {
      RCLCPP_ERROR(hardware_logger(), "set_mobile_base_command failed: %s",
                   result_code_name(status));
      return hardware_interface::return_type::ERROR;
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn MuJoCoHardwareInterface::on_activate(
    const rclcpp_lifecycle::State&) {
  system_state_ = SystemState::kActivating;
  const mujoco_simulation::ResultCode start_status = request_start_status();
  if (start_status != ResultCode::Ok) {
    system_state_ = SystemState::kError;
    RCLCPP_ERROR(hardware_logger(), "start failed: %s", result_code_name(start_status));
    return hardware_interface::CallbackReturn::ERROR;
  }
  const mujoco_simulation::ResultCode bridge_start_status = ros_bridge_->start();
  if (bridge_start_status != ResultCode::Ok) {
    system_state_ = SystemState::kError;
    (void)request_stop_status();
    RCLCPP_ERROR(hardware_logger(), "bridge start failed: %s",
                 result_code_name(bridge_start_status));
    return hardware_interface::CallbackReturn::ERROR;
  }
  system_state_ = SystemState::kActive;
  const mujoco_simulation::ResultCode read_status = update_runtime_state();
  if (read_status != ResultCode::Ok) {
    system_state_ = SystemState::kError;
    (void)ros_bridge_->stop();
    (void)request_stop_status();
    RCLCPP_ERROR(hardware_logger(), "runtime state update failed: %s",
                 result_code_name(read_status));
    return hardware_interface::CallbackReturn::ERROR;
  }
  const std::shared_ptr<const mujoco_simulation::StateSnapshot> initial_snapshot =
      simulation_->state_snapshot();
  if (initial_snapshot != nullptr) {
    const mujoco_simulation::ResultCode publish_status =
        publish_snapshot_to_channel(initial_snapshot);
    if (publish_status != ResultCode::Ok) {
      system_state_ = SystemState::kError;
      (void)ros_bridge_->stop();
      (void)request_stop_status();
      RCLCPP_ERROR(hardware_logger(), "initial publish failed: %s",
                   result_code_name(publish_status));
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  initialize_command_buffers();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MuJoCoHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State&) {
  system_state_ = SystemState::kDeactivating;
  if (ros_bridge_ != nullptr) {
    const mujoco_simulation::ResultCode bridge_stop_status = ros_bridge_->stop();
    if (bridge_stop_status != ResultCode::Ok) {
      system_state_ = SystemState::kError;
      RCLCPP_ERROR(hardware_logger(), "bridge stop failed: %s",
                   result_code_name(bridge_stop_status));
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  const mujoco_simulation::ResultCode stop_status = request_stop_status();
  if (stop_status != ResultCode::Ok) {
    system_state_ = SystemState::kError;
    RCLCPP_ERROR(hardware_logger(), "stop failed: %s", result_code_name(stop_status));
    return hardware_interface::CallbackReturn::ERROR;
  }
  for (auto& [joint_name, interfaces] : active_joint_interfaces_) {
    interfaces.clear();
    pending_mode_switch_.next_interfaces[joint_name].clear();
  }
  pending_mode_switch_.valid = false;
  system_state_ = SystemState::kInactive;
  return hardware_interface::CallbackReturn::SUCCESS;
}

}  // namespace robot_mujoco_ros2
