#include <exception>
#include <limits>
#include <sstream>
#include <unordered_map>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "robot_mujoco_ros2/data.hpp"

namespace robot_mujoco_ros2 {
namespace {

std::string parameter_or(const std::unordered_map<std::string, std::string>& parameters,
                         const std::string& key, const std::string& fallback = std::string()) {
  const auto it = parameters.find(key);
  return it == parameters.end() ? fallback : it->second;
}

bool parameter_as_bool(const std::unordered_map<std::string, std::string>& parameters,
                       const std::string& key, bool fallback) {
  const auto value = parameter_or(parameters, key);
  if (value.empty()) {
    return fallback;
  }
  return value == "1" || value == "true" || value == "True";
}

bool parse_int_parameter(const std::unordered_map<std::string, std::string>& parameters,
                         const std::string& key, int fallback, int* value,
                         std::string& error_message) {
  if (value == nullptr) {
    return false;
  }
  const auto raw = parameter_or(parameters, key);
  if (raw.empty()) {
    *value = fallback;
    return true;
  }
  try {
    *value = std::stoi(raw);
  } catch (const std::exception&) {
    error_message = "Invalid integer parameter '" + key + "': " + raw;
    return false;
  }
  return true;
}

bool parse_double_parameter(const std::unordered_map<std::string, std::string>& parameters,
                            const std::string& key, double fallback, double* value,
                            std::string& error_message) {
  if (value == nullptr) {
    return false;
  }
  const auto raw = parameter_or(parameters, key);
  if (raw.empty()) {
    *value = fallback;
    return true;
  }
  try {
    *value = std::stod(raw);
  } catch (const std::exception&) {
    error_message = "Invalid floating-point parameter '" + key + "': " + raw;
    return false;
  }
  return true;
}

bool parse_covariance_parameter(const std::unordered_map<std::string, std::string>& parameters,
                                const std::string& key, mujoco_simulation::Vector9d* covariance,
                                std::string& error_message) {
  if (covariance == nullptr) {
    return false;
  }
  covariance->fill(-1.0);
  const auto raw = parameter_or(parameters, key);
  if (raw.empty()) {
    return true;
  }

  std::stringstream stream(raw);
  std::string item;
  std::size_t index = 0;
  while (std::getline(stream, item, ',')) {
    if (index >= covariance->size()) {
      error_message = "Covariance parameter '" + key + "' must contain exactly 9 values.";
      return false;
    }
    try {
      (*covariance)[index] = std::stod(item);
    } catch (const std::exception&) {
      error_message = "Invalid covariance value in '" + key + "': " + item;
      return false;
    }
    ++index;
  }

  if (index != covariance->size()) {
    error_message = "Covariance parameter '" + key + "' must contain exactly 9 values.";
    return false;
  }
  return true;
}

bool is_sensor_type(const hardware_interface::ComponentInfo& sensor, const std::string& type,
                    std::string& error_message) {
  const auto configured = parameter_or(sensor.parameters, "mujoco_type");
  if (configured.empty()) {
    if (type == "imu") {
      return true;
    }
    error_message = "Sensor '" + sensor.name + "' is missing the required 'mujoco_type' parameter.";
    return false;
  }
  return configured == type;
}

const char* mecanum_wheel_parameter_name(mujoco_simulation::MecanumWheelIndex index) {
  using mujoco_simulation::MecanumWheelIndex;
  switch (index) {
    case MecanumWheelIndex::FrontLeft:
      return "front_left";
    case MecanumWheelIndex::FrontRight:
      return "front_right";
    case MecanumWheelIndex::RearLeft:
      return "rear_left";
    case MecanumWheelIndex::RearRight:
      return "rear_right";
    default:
      return "";
  }
}

}  // namespace

bool is_joint_command_interface(const std::string& interface_name) {
  return interface_name == hardware_interface::HW_IF_POSITION ||
         interface_name == hardware_interface::HW_IF_VELOCITY ||
         interface_name == hardware_interface::HW_IF_EFFORT || interface_name == "stiffness" ||
         interface_name == "damping";
}

bool is_joint_state_interface(const std::string& interface_name) {
  return interface_name == hardware_interface::HW_IF_POSITION ||
         interface_name == hardware_interface::HW_IF_VELOCITY ||
         interface_name == hardware_interface::HW_IF_EFFORT;
}

bool is_imu_state_interface(const std::string& interface_name) {
  return interface_name == "orientation.x" || interface_name == "orientation.y" ||
         interface_name == "orientation.z" || interface_name == "orientation.w" ||
         interface_name == "angular_velocity.x" || interface_name == "angular_velocity.y" ||
         interface_name == "angular_velocity.z" || interface_name == "linear_acceleration.x" ||
         interface_name == "linear_acceleration.y" || interface_name == "linear_acceleration.z";
}

bool parse_hardware_config(const hardware_interface::HardwareInfo& hardware_info,
                           HardwareConfig* config, std::string& error_message) {
  if (config == nullptr) {
    error_message = "HardwareConfig output pointer must not be null.";
    return false;
  }

  HardwareConfig parsed;
  parsed.simulation.model.model_path =
      parameter_or(hardware_info.hardware_parameters, "mujoco_model_path");
  if (parsed.simulation.model.model_path.empty()) {
    error_message = "Missing required hardware parameter 'mujoco_model_path'.";
    return false;
  }

  parsed.simulation.model.initial_keyframe =
      parameter_or(hardware_info.hardware_parameters, "initial_keyframe");
  if (!parse_double_parameter(hardware_info.hardware_parameters, "viewer_update_rate", 60.0,
                              &parsed.simulation.scheduler.viewer_update_rate, error_message)) {
    return false;
  }
  try {
    parsed.simulation.render_mode = mujoco_simulation::parse_render_mode(
        parameter_or(hardware_info.hardware_parameters, "render_mode", "headless"));
  } catch (const std::exception& ex) {
    error_message = ex.what();
    return false;
  }

  for (const auto& joint : hardware_info.joints) {
    JointData joint_data;
    joint_data.name = joint.name;
    joint_data.config.joint_name = joint.name;
    joint_data.config.actuator_name = parameter_or(joint.parameters, "motor_name");
    if (!parse_double_parameter(joint.parameters, "position_stiffness", 0.0,
                                &joint_data.config.position_stiffness, error_message) ||
        !parse_double_parameter(joint.parameters, "position_damping", 0.0,
                                &joint_data.config.position_damping, error_message) ||
        !parse_double_parameter(joint.parameters, "velocity_damping", 0.0,
                                &joint_data.config.velocity_damping, error_message) ||
        !parse_double_parameter(joint.parameters, "position_min",
                                -std::numeric_limits<double>::infinity(),
                                &joint_data.config.position_limits.min, error_message) ||
        !parse_double_parameter(joint.parameters, "position_max",
                                std::numeric_limits<double>::infinity(),
                                &joint_data.config.position_limits.max, error_message) ||
        !parse_double_parameter(joint.parameters, "velocity_min",
                                -std::numeric_limits<double>::infinity(),
                                &joint_data.config.velocity_limits.min, error_message) ||
        !parse_double_parameter(joint.parameters, "velocity_max",
                                std::numeric_limits<double>::infinity(),
                                &joint_data.config.velocity_limits.max, error_message) ||
        !parse_double_parameter(joint.parameters, "effort_min",
                                -std::numeric_limits<double>::infinity(),
                                &joint_data.config.effort_limits.min, error_message) ||
        !parse_double_parameter(joint.parameters, "effort_max",
                                std::numeric_limits<double>::infinity(),
                                &joint_data.config.effort_limits.max, error_message)) {
      return false;
    }
    joint_data.command.joint_name = joint.name;
    joint_data.state.joint_name = joint.name;

    for (const auto& command_interface : joint.command_interfaces) {
      if (!is_joint_command_interface(command_interface.name)) {
        error_message = "Unsupported joint command interface '" + command_interface.name +
                        "' on joint '" + joint.name + "'.";
        return false;
      }
      joint_data.command_interfaces.push_back(command_interface.name);
    }
    for (const auto& state_interface : joint.state_interfaces) {
      if (!is_joint_state_interface(state_interface.name)) {
        error_message = "Unsupported joint state interface '" + state_interface.name +
                        "' on joint '" + joint.name + "'.";
        return false;
      }
      joint_data.state_interfaces.push_back(state_interface.name);
    }

    parsed.joints.push_back(std::move(joint_data));
    parsed.simulation.components.emplace_back(parsed.joints.back().config);
  }

  for (const auto& sensor : hardware_info.sensors) {
    if (is_sensor_type(sensor, "imu", error_message)) {
      ImuData imu_data;
      imu_data.name = sensor.name;
      imu_data.frame_id = parameter_or(sensor.parameters, "frame_id", sensor.name);
      imu_data.topic = parameter_or(sensor.parameters, "topic", sensor.name + "/imu");
      imu_data.config.name = sensor.name;
      imu_data.config.frame_id = imu_data.frame_id;
      imu_data.config.framequat_sensor_name =
          parameter_or(sensor.parameters, "mujoco_orientation_sensor");
      imu_data.config.gyro_sensor_name = parameter_or(sensor.parameters, "mujoco_gyro_sensor");
      imu_data.config.accelerometer_sensor_name =
          parameter_or(sensor.parameters, "mujoco_accel_sensor");
      if (!parse_double_parameter(sensor.parameters, "update_rate", 200.0,
                                  &imu_data.config.update_rate, error_message)) {
        return false;
      }

      if (imu_data.config.framequat_sensor_name.empty() ||
          imu_data.config.gyro_sensor_name.empty() ||
          imu_data.config.accelerometer_sensor_name.empty()) {
        error_message =
            "IMU '" + sensor.name +
            "' requires mujoco_orientation_sensor, mujoco_gyro_sensor, and mujoco_accel_sensor.";
        return false;
      }

      if (!parse_covariance_parameter(sensor.parameters, "orientation_covariance",
                                      &imu_data.state.orientation_covariance, error_message) ||
          !parse_covariance_parameter(sensor.parameters, "angular_velocity_covariance",
                                      &imu_data.state.angular_velocity_covariance, error_message) ||
          !parse_covariance_parameter(sensor.parameters, "linear_acceleration_covariance",
                                      &imu_data.state.linear_acceleration_covariance,
                                      error_message)) {
        return false;
      }
      imu_data.config.orientation_covariance = imu_data.state.orientation_covariance;
      imu_data.config.angular_velocity_covariance = imu_data.state.angular_velocity_covariance;
      imu_data.config.linear_acceleration_covariance =
          imu_data.state.linear_acceleration_covariance;

      for (const auto& state_interface : sensor.state_interfaces) {
        if (!is_imu_state_interface(state_interface.name)) {
          error_message = "Unsupported IMU state interface '" + state_interface.name +
                          "' on sensor '" + sensor.name + "'.";
          return false;
        }
        imu_data.state_interfaces.push_back(state_interface.name);
      }

      parsed.imus.push_back(std::move(imu_data));
      parsed.simulation.components.emplace_back(parsed.imus.back().config);
      continue;
    }

    if (is_sensor_type(sensor, "camera", error_message)) {
      CameraData camera_data;
      camera_data.name = sensor.name;
      camera_data.frame_id = parameter_or(sensor.parameters, "optical_frame_id",
                                          parameter_or(sensor.parameters, "frame_id", sensor.name));
      camera_data.rgb_topic =
          parameter_or(sensor.parameters, "image_topic", sensor.name + "/image_raw");
      camera_data.depth_topic =
          parameter_or(sensor.parameters, "depth_topic", sensor.name + "/depth/image_raw");
      camera_data.camera_info_topic =
          parameter_or(sensor.parameters, "camera_info_topic", sensor.name + "/camera_info");
      camera_data.config.name = sensor.name;
      camera_data.config.camera_name =
          parameter_or(sensor.parameters, "mujoco_camera_name", sensor.name);
      camera_data.config.frame_id = parameter_or(sensor.parameters, "frame_id", sensor.name);
      camera_data.config.optical_frame_id = camera_data.frame_id;
      camera_data.config.enable_rgb = parameter_as_bool(sensor.parameters, "enable_rgb", true);
      camera_data.config.enable_depth = parameter_as_bool(sensor.parameters, "enable_depth", false);
      if (!parse_double_parameter(sensor.parameters, "update_rate", 30.0,
                                  &camera_data.config.update_rate, error_message) ||
          !parse_int_parameter(sensor.parameters, "width", 640, &camera_data.config.width,
                               error_message) ||
          !parse_int_parameter(sensor.parameters, "height", 480, &camera_data.config.height,
                               error_message)) {
        return false;
      }

      parsed.cameras.push_back(std::move(camera_data));
      parsed.simulation.components.emplace_back(parsed.cameras.back().config);
      continue;
    }

    if (is_sensor_type(sensor, "lidar", error_message)) {
      LidarData lidar_data;
      lidar_data.name = sensor.name;
      lidar_data.frame_id = parameter_or(sensor.parameters, "frame_id", sensor.name);
      lidar_data.topic = parameter_or(sensor.parameters, "scan_topic", sensor.name + "/scan");
      lidar_data.config.name = sensor.name;
      lidar_data.config.frame_id = lidar_data.frame_id;
      lidar_data.config.sensor_prefix =
          parameter_or(sensor.parameters, "sensor_prefix", sensor.name);
      if (!parse_double_parameter(sensor.parameters, "update_rate", 10.0,
                                  &lidar_data.config.update_rate, error_message) ||
          !parse_double_parameter(sensor.parameters, "angle_min", 0.0, &lidar_data.config.angle_min,
                                  error_message) ||
          !parse_double_parameter(sensor.parameters, "angle_max", 0.0, &lidar_data.config.angle_max,
                                  error_message) ||
          !parse_double_parameter(sensor.parameters, "angle_increment", 0.0,
                                  &lidar_data.config.angle_increment, error_message) ||
          !parse_double_parameter(sensor.parameters, "range_min", 0.0, &lidar_data.config.range_min,
                                  error_message) ||
          !parse_double_parameter(sensor.parameters, "range_max", 0.0, &lidar_data.config.range_max,
                                  error_message)) {
        return false;
      }

      parsed.lidars.push_back(std::move(lidar_data));
      parsed.simulation.components.emplace_back(parsed.lidars.back().config);
    }
  }

  // Parse mobile bases from hardware parameters.
  // Convention: mobile_base_<N>_<field> where N is 0, 1, 2, ...
  for (int i = 0;; ++i) {
    const std::string name_key = "mobile_base_" + std::to_string(i) + "_name";
    const std::string raw_name = parameter_or(hardware_info.hardware_parameters, name_key);
    if (raw_name.empty()) {
      break;
    }

    MobileBaseData mb;
    mb.name = raw_name;
    mb.config.mobile_base_name = raw_name;
    mb.command = {};
    mb.state = {};

    const std::string type_str = parameter_or(hardware_info.hardware_parameters,
                                              "mobile_base_" + std::to_string(i) + "_type");
    if (type_str != "mecanum") {
      error_message =
          "Invalid mobile_base type '" + type_str + "' for '" + raw_name + "'. Must be 'mecanum'.";
      return false;
    }
    mb.config.type = mujoco_simulation::MobileBaseType::Mecanum;

    const std::string parameter_prefix = "mobile_base_" + std::to_string(i) + "_";
    for (std::size_t wheel_index = 0; wheel_index < mujoco_simulation::MecanumWheelCount;
         ++wheel_index) {
      const auto index = to_enum<mujoco_simulation::MecanumWheelIndex>(wheel_index);
      const std::string wheel_name = mecanum_wheel_parameter_name(index);
      auto& wheel = mb.config.mecanum_wheels[wheel_index];
      wheel.wheel_name =
          parameter_or(hardware_info.hardware_parameters, parameter_prefix + wheel_name + "_joint");
      wheel.actuator_name = parameter_or(hardware_info.hardware_parameters,
                                         parameter_prefix + wheel_name + "_actuator");
      if (wheel.wheel_name.empty() || wheel.actuator_name.empty()) {
        error_message = "Mecanum mobile base '" + raw_name + "' wheel '" + wheel_name +
                        "' requires joint and actuator parameters.";
        return false;
      }
      if (!parse_double_parameter(hardware_info.hardware_parameters,
                                  parameter_prefix + wheel_name + "_damping", 0.0, &wheel.damping,
                                  error_message)) {
        return false;
      }
    }

    if (!parse_double_parameter(hardware_info.hardware_parameters,
                                "mobile_base_" + std::to_string(i) + "_wheel_radius", 0.0,
                                &mb.config.mecanum_info.wheel_radius, error_message)) {
      return false;
    }
    if (!parse_double_parameter(hardware_info.hardware_parameters,
                                "mobile_base_" + std::to_string(i) + "_track_width", 0.0,
                                &mb.config.mecanum_info.track_width, error_message)) {
      return false;
    }
    if (!parse_double_parameter(hardware_info.hardware_parameters,
                                "mobile_base_" + std::to_string(i) + "_wheel_base", 0.0,
                                &mb.config.mecanum_info.wheel_base, error_message)) {
      return false;
    }

    mb.config.base_frame_id =
        parameter_or(hardware_info.hardware_parameters,
                     "mobile_base_" + std::to_string(i) + "_base_frame_id", "base_link");
    mb.config.odom_frame_id =
        parameter_or(hardware_info.hardware_parameters,
                     "mobile_base_" + std::to_string(i) + "_odom_frame_id", "odom");
    mb.config.base_body_name =
        parameter_or(hardware_info.hardware_parameters,
                     "mobile_base_" + std::to_string(i) + "_base_body_name", "");
    if (mb.config.base_body_name.empty()) {
      error_message = "Mecanum mobile base '" + raw_name + "' requires a base_body_name parameter.";
      return false;
    }

    parsed.mobile_bases.push_back(std::move(mb));
    parsed.simulation.components.emplace_back(parsed.mobile_bases.back().config);
  }

  *config = std::move(parsed);
  error_message.clear();
  return true;
}

}  // namespace robot_mujoco_ros2
