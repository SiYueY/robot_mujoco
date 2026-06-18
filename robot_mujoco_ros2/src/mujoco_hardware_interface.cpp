#include "robot_mujoco_ros2/mujoco_hardware_interface.hpp"

#include <algorithm>
#include <map>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_mujoco_ros2/simulation_ros_bridge.hpp"

namespace robot_mujoco_ros2 {
namespace {

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

void copy_imu_sample_rt(const mujoco_simulation::ImuSample& source,
                        mujoco_simulation::ImuSample* target) {
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

bool copy_lidar_sample_rt(const mujoco_simulation::LidarSample& source,
                          mujoco_simulation::LidarSample* target) {
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

mujoco_simulation::Status MuJoCoHardwareInterface::request_start_status() {
  std::lock_guard<std::mutex> lock(simulation_control_mutex_);
  if (simulation_ == nullptr) {
    return mujoco_simulation::Status::invalid_state("Simulation is not initialized.");
  }
  if (!simulation_started_) {
    const mujoco_simulation::Status status = simulation_->start();
    if (!status.ok()) {
      return status;
    }
    simulation_started_ = true;
  }
  return mujoco_simulation::Status::Ok();
}

mujoco_simulation::Status MuJoCoHardwareInterface::request_stop_status() {
  std::lock_guard<std::mutex> lock(simulation_control_mutex_);
  if (simulation_ == nullptr) {
    return mujoco_simulation::Status::invalid_state("Simulation is not initialized.");
  }
  if (simulation_started_) {
    const mujoco_simulation::Status status = simulation_->stop();
    if (!status.ok()) {
      return status;
    }
    simulation_started_ = false;
  }
  return mujoco_simulation::Status::Ok();
}

mujoco_simulation::Status MuJoCoHardwareInterface::request_pause_status() {
  std::lock_guard<std::mutex> lock(simulation_control_mutex_);
  if (simulation_ == nullptr) {
    return mujoco_simulation::Status::invalid_state("Simulation is not initialized.");
  }
  if (!simulation_started_) {
    return mujoco_simulation::Status::invalid_state("Simulation is not running.");
  }
  return simulation_->pause();
}

mujoco_simulation::Status MuJoCoHardwareInterface::request_resume_status() {
  std::lock_guard<std::mutex> lock(simulation_control_mutex_);
  if (simulation_ == nullptr) {
    return mujoco_simulation::Status::invalid_state("Simulation is not initialized.");
  }
  if (!simulation_started_) {
    return mujoco_simulation::Status::invalid_state("Simulation is not running.");
  }
  return simulation_->resume();
}

mujoco_simulation::Status MuJoCoHardwareInterface::request_step_status(uint32_t steps) {
  std::lock_guard<std::mutex> lock(simulation_control_mutex_);
  if (simulation_ == nullptr) {
    return mujoco_simulation::Status::invalid_state("Simulation is not initialized.");
  }
  return simulation_->step(steps);
}

mujoco_simulation::Status MuJoCoHardwareInterface::request_set_realtime_factor_status(
    double realtime_factor) {
  std::lock_guard<std::mutex> lock(simulation_control_mutex_);
  if (simulation_ == nullptr) {
    return mujoco_simulation::Status::invalid_state("Simulation is not initialized.");
  }
  return simulation_->set_realtime_factor(realtime_factor);
}

mujoco_simulation::Status MuJoCoHardwareInterface::request_keyframe_reset_status(
    const std::string& keyframe) {
  std::lock_guard<std::mutex> lock(simulation_control_mutex_);
  if (simulation_ == nullptr) {
    return mujoco_simulation::Status::invalid_state("Simulation is not initialized.");
  }
  return simulation_->reset({.keyframe_name = keyframe});
}

mujoco_simulation::Status MuJoCoHardwareInterface::request_reset_status() {
  std::lock_guard<std::mutex> lock(simulation_control_mutex_);
  if (simulation_ == nullptr) {
    return mujoco_simulation::Status::invalid_state("Simulation is not initialized.");
  }
  return simulation_->request_reset();
}

mujoco_simulation::Status MuJoCoHardwareInterface::update_runtime_state() {
  const std::shared_ptr<const mujoco_simulation::SimulationStateSnapshot> snapshot =
      simulation_ == nullptr ? nullptr : simulation_->state_snapshot();
  if (snapshot == nullptr) {
    return mujoco_simulation::Status::invalid_state("Simulation state snapshot is not available.");
  }
  return update_runtime_state_from_snapshot(*snapshot);
}

mujoco_simulation::Status MuJoCoHardwareInterface::update_runtime_state_from_snapshot(
    const mujoco_simulation::SimulationStateSnapshot& snapshot) {
  for (auto& joint : config_.joints) {
    const auto it = snapshot.joints.find(joint.name);
    if (it == snapshot.joints.end()) {
      return mujoco_simulation::Status::not_found(
          "Joint state is missing from the simulation snapshot: " + joint.name);
    }
    joint.state = it->second;
  }
  for (auto& imu : config_.imus) {
    const auto it = snapshot.imus.find(imu.name);
    if (it == snapshot.imus.end()) {
      return mujoco_simulation::Status::not_found(
          "IMU sample is missing from the simulation snapshot: " + imu.name);
    }
    imu.sample = it->second;
  }
  for (auto& lidar : config_.lidars) {
    const auto it = snapshot.lidars.find(lidar.name);
    if (it == snapshot.lidars.end()) {
      return mujoco_simulation::Status::not_found(
          "Lidar sample is missing from the simulation snapshot: " + lidar.name);
    }
    lidar.sample = it->second;
  }
  for (auto& mobile_base : config_.mobile_bases) {
    const auto it = snapshot.mobile_bases.find(mobile_base.name);
    if (it == snapshot.mobile_bases.end()) {
      return mujoco_simulation::Status::not_found(
          "Mobile base state is missing from the simulation snapshot: " + mobile_base.name);
    }
    mobile_base.state = it->second;
  }
  return mujoco_simulation::Status::Ok();
}

mujoco_simulation::Status MuJoCoHardwareInterface::publish_snapshot_to_channel(
    const std::shared_ptr<const mujoco_simulation::SimulationStateSnapshot>& snapshot) {
  if (snapshot == nullptr) {
    return mujoco_simulation::Status::invalid_argument("Simulation snapshot is null.");
  }
  if (ros_bridge_ == nullptr) {
    return mujoco_simulation::Status::invalid_state(
        "ROS bridge is not initialized for robot_mujoco_ros2.");
  }
  if (simulation_ == nullptr) {
    return mujoco_simulation::Status::invalid_state("Simulation is not initialized.");
  }
  if (system_state_ != SystemState::kActive) {
    return mujoco_simulation::Status::Ok();
  }

  const auto stamp = to_ros_time(snapshot->simulation_time);
  publish_bundle_.sim_time = stamp;
  for (std::size_t index = 0; index < config_.imus.size(); ++index) {
    const auto& imu = config_.imus[index];
    const auto it = snapshot->imus.find(imu.name);
    if (it == snapshot->imus.end()) {
      return mujoco_simulation::Status::not_found(
          "IMU sample is missing from the simulation snapshot: " + imu.name);
    }
    copy_imu_sample_rt(it->second, &publish_imu_samples_[index]);
    publish_bundle_.imus[index].publisher_index = index;
    copy_imu_sample_rt(publish_imu_samples_[index], &publish_bundle_.imus[index].sample);
  }
  for (std::size_t index = 0; index < config_.cameras.size(); ++index) {
    const auto& camera = config_.cameras[index];
    const mujoco_simulation::Result<std::shared_ptr<const mujoco_simulation::CameraSample>> sample =
        simulation_->camera_sample(camera.name);
    if (!sample.ok() || sample.value() == nullptr) {
      if (!sample.ok()) {
        return sample.status();
      }
      return mujoco_simulation::Status::invalid_state("Camera sample is not available for '" +
                                                      camera.name + "'.");
    }
    publish_camera_samples_[index] = sample.value();
    publish_bundle_.cameras[index].publisher_index = index;
    publish_bundle_.cameras[index].sample = publish_camera_samples_[index].get();
  }
  for (std::size_t index = 0; index < config_.lidars.size(); ++index) {
    const auto& lidar = config_.lidars[index];
    const auto it = snapshot->lidars.find(lidar.name);
    if (it == snapshot->lidars.end()) {
      return mujoco_simulation::Status::not_found(
          "Lidar sample is missing from the simulation snapshot: " + lidar.name);
    }
    if (!copy_lidar_sample_rt(it->second, &publish_lidar_samples_[index]) ||
        !copy_lidar_sample_rt(publish_lidar_samples_[index],
                              &publish_bundle_.lidars[index].sample)) {
      return mujoco_simulation::Status::failed_precondition(
          "Preallocated lidar publish buffer is smaller than the incoming sample.");
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
  const mujoco_simulation::Status initialize_status = simulation_->initialize(config_.simulation);
  if (!initialize_status.ok()) {
    error_message = initialize_status.message();
    RCLCPP_ERROR(hardware_logger(), "Failed to initialize MuJoCo simulation for hardware '%s': %s",
                 hardware_info.name.c_str(), error_message.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (auto& camera : config_.cameras) {
    const auto sample = simulation_->camera_sample(camera.name);
    if (!sample.ok() || sample.value() == nullptr) {
      error_message = sample.ok() ? "Camera sample is not available." : sample.status().message();
      RCLCPP_ERROR(hardware_logger(),
                   "Failed to initialize camera sample for hardware '%s', camera '%s': %s",
                   hardware_info.name.c_str(), camera.name.c_str(), error_message.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    camera.sample = *sample.value();
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
  publish_imu_samples_.resize(config_.imus.size());
  publish_lidar_samples_.resize(config_.lidars.size());
  publish_camera_samples_.resize(config_.cameras.size());
  publish_bundle_.imus.resize(config_.imus.size());
  publish_bundle_.lidars.resize(config_.lidars.size());
  publish_bundle_.cameras.resize(config_.cameras.size());
  for (std::size_t i = 0; i < config_.lidars.size(); ++i) {
    const std::size_t sample_count =
        i < adapter_config.ros_interface_config.lidars.size()
            ? adapter_config.ros_interface_config.lidars[i].sample_count
            : 0U;
    publish_lidar_samples_[i].ranges.resize(sample_count, 0.0);
    publish_lidar_samples_[i].intensities.resize(sample_count, 0.0);
    publish_bundle_.lidars[i].sample.ranges.resize(sample_count, 0.0);
    publish_bundle_.lidars[i].sample.intensities.resize(sample_count, 0.0);
  }
  simulation_->set_snapshot_observer(
      [this](std::shared_ptr<const mujoco_simulation::SimulationStateSnapshot> snapshot) {
        const mujoco_simulation::Status status = publish_snapshot_to_channel(snapshot);
        if (!status.ok()) {
          RCLCPP_ERROR(hardware_logger(), "%s", status.message().c_str());
        }
      });

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
        state_interfaces.emplace_back(imu.name, interface_name, &imu.sample.orientation[0]);
      } else if (interface_name == "orientation.y") {
        state_interfaces.emplace_back(imu.name, interface_name, &imu.sample.orientation[1]);
      } else if (interface_name == "orientation.z") {
        state_interfaces.emplace_back(imu.name, interface_name, &imu.sample.orientation[2]);
      } else if (interface_name == "orientation.w") {
        state_interfaces.emplace_back(imu.name, interface_name, &imu.sample.orientation[3]);
      } else if (interface_name == "angular_velocity.x") {
        state_interfaces.emplace_back(imu.name, interface_name, &imu.sample.angular_velocity[0]);
      } else if (interface_name == "angular_velocity.y") {
        state_interfaces.emplace_back(imu.name, interface_name, &imu.sample.angular_velocity[1]);
      } else if (interface_name == "angular_velocity.z") {
        state_interfaces.emplace_back(imu.name, interface_name, &imu.sample.angular_velocity[2]);
      } else if (interface_name == "linear_acceleration.x") {
        state_interfaces.emplace_back(imu.name, interface_name, &imu.sample.linear_acceleration[0]);
      } else if (interface_name == "linear_acceleration.y") {
        state_interfaces.emplace_back(imu.name, interface_name, &imu.sample.linear_acceleration[1]);
      } else if (interface_name == "linear_acceleration.z") {
        state_interfaces.emplace_back(imu.name, interface_name, &imu.sample.linear_acceleration[2]);
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

  for (const auto& [joint_name, mode] : pending_mode_switch_.next_modes) {
    JointData* joint = find_joint(joint_name);
    if (joint == nullptr) {
      return hardware_interface::return_type::ERROR;
    }
    const auto previous_config = joint->config;
    joint->config.command_mode = mode;
    const mujoco_simulation::Status status =
        simulation_->reconfigure_component(mujoco_simulation::ComponentConfig{joint->config});
    if (!status.ok()) {
      joint->config = previous_config;
      RCLCPP_ERROR(hardware_logger(), "%s", status.message().c_str());
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
  const mujoco_simulation::Status status = update_runtime_state();
  if (!status.ok()) {
    RCLCPP_ERROR(hardware_logger(), "%s", status.message().c_str());
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MuJoCoHardwareInterface::write(const rclcpp::Time&,
                                                               const rclcpp::Duration&) {
  for (const auto& joint : config_.joints) {
    const auto it = active_joint_modes_.find(joint.name);
    const auto mode = it == active_joint_modes_.end()
                          ? mujoco_simulation::CommandInterfaceType::None
                          : it->second;
    if (mode == mujoco_simulation::CommandInterfaceType::None) {
      continue;
    }
    const mujoco_simulation::Status status = simulation_->set_joint_command(joint.command);
    if (!status.ok()) {
      RCLCPP_ERROR(hardware_logger(), "%s", status.message().c_str());
      return hardware_interface::return_type::ERROR;
    }
  }
  for (const auto& mobile_base : config_.mobile_bases) {
    const mujoco_simulation::Status status =
        simulation_->set_mobile_base_command(mobile_base.name, mobile_base.command);
    if (!status.ok()) {
      RCLCPP_ERROR(hardware_logger(), "%s", status.message().c_str());
      return hardware_interface::return_type::ERROR;
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::CallbackReturn MuJoCoHardwareInterface::on_activate(
    const rclcpp_lifecycle::State&) {
  system_state_ = SystemState::kActivating;
  const mujoco_simulation::Status start_status = request_start_status();
  if (!start_status.ok()) {
    system_state_ = SystemState::kError;
    RCLCPP_ERROR(hardware_logger(), "%s", start_status.message().c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  const mujoco_simulation::Status bridge_start_status = ros_bridge_->start();
  if (!bridge_start_status.ok()) {
    system_state_ = SystemState::kError;
    (void)request_stop_status();
    RCLCPP_ERROR(hardware_logger(), "%s", bridge_start_status.message().c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  system_state_ = SystemState::kActive;
  const mujoco_simulation::Status read_status = update_runtime_state();
  if (!read_status.ok()) {
    system_state_ = SystemState::kError;
    (void)ros_bridge_->stop();
    (void)request_stop_status();
    RCLCPP_ERROR(hardware_logger(), "%s", read_status.message().c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  const std::shared_ptr<const mujoco_simulation::SimulationStateSnapshot> initial_snapshot =
      simulation_->state_snapshot();
  if (initial_snapshot != nullptr) {
    const mujoco_simulation::Status publish_status = publish_snapshot_to_channel(initial_snapshot);
    if (!publish_status.ok()) {
      system_state_ = SystemState::kError;
      (void)ros_bridge_->stop();
      (void)request_stop_status();
      RCLCPP_ERROR(hardware_logger(), "%s", publish_status.message().c_str());
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
    const mujoco_simulation::Status bridge_stop_status = ros_bridge_->stop();
    if (!bridge_stop_status.ok()) {
      system_state_ = SystemState::kError;
      RCLCPP_ERROR(hardware_logger(), "%s", bridge_stop_status.message().c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  const mujoco_simulation::Status stop_status = request_stop_status();
  if (!stop_status.ok()) {
    system_state_ = SystemState::kError;
    RCLCPP_ERROR(hardware_logger(), "%s", stop_status.message().c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  for (auto& [joint_name, mode] : active_joint_modes_) {
    mode = mujoco_simulation::CommandInterfaceType::None;
    pending_mode_switch_.next_modes[joint_name] = mujoco_simulation::CommandInterfaceType::None;
  }
  pending_mode_switch_.valid = false;
  system_state_ = SystemState::kInactive;
  return hardware_interface::CallbackReturn::SUCCESS;
}

}  // namespace robot_mujoco_ros2
