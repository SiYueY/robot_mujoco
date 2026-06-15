#include <exception>
#include <sstream>
#include <unordered_map>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "mujoco_hardware/data.hpp"

namespace mujoco_hardware {
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

}  // namespace

bool is_joint_command_interface(const std::string& interface_name) {
  return interface_name == hardware_interface::HW_IF_POSITION ||
         interface_name == hardware_interface::HW_IF_VELOCITY ||
         interface_name == hardware_interface::HW_IF_EFFORT;
}

bool is_joint_state_interface(const std::string& interface_name) {
  return is_joint_command_interface(interface_name);
}

bool is_imu_state_interface(const std::string& interface_name) {
  return interface_name == "orientation.x" || interface_name == "orientation.y" ||
         interface_name == "orientation.z" || interface_name == "orientation.w" ||
         interface_name == "angular_velocity.x" || interface_name == "angular_velocity.y" ||
         interface_name == "angular_velocity.z" || interface_name == "linear_acceleration.x" ||
         interface_name == "linear_acceleration.y" || interface_name == "linear_acceleration.z";
}

mujoco_simulation::CommandInterfaceType to_joint_control_mode(const std::string& interface_name) {
  if (interface_name == hardware_interface::HW_IF_POSITION) {
    return mujoco_simulation::CommandInterfaceType::Position;
  }
  if (interface_name == hardware_interface::HW_IF_VELOCITY) {
    return mujoco_simulation::CommandInterfaceType::Velocity;
  }
  if (interface_name == hardware_interface::HW_IF_EFFORT) {
    return mujoco_simulation::CommandInterfaceType::Effort;
  }
  return mujoco_simulation::CommandInterfaceType::None;
}

bool parse_hardware_config(const hardware_interface::HardwareInfo& hardware_info,
                           HardwareConfig* config, std::string& error_message) {
  if (config == nullptr) {
    error_message = "HardwareConfig output pointer must not be null.";
    return false;
  }

  HardwareConfig parsed;
  parsed.simulation.model_path =
      parameter_or(hardware_info.hardware_parameters, "mujoco_model_path");
  if (parsed.simulation.model_path.empty()) {
    error_message = "Missing required hardware parameter 'mujoco_model_path'.";
    return false;
  }

  parsed.simulation.initial_keyframe =
      parameter_or(hardware_info.hardware_parameters, "initial_keyframe");
  if (!parse_double_parameter(hardware_info.hardware_parameters, "sim_speed_factor", 1.0,
                              &parsed.simulation.sim_speed_factor, error_message)) {
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
    joint_data.info.name = joint.name;
    joint_data.info.actuator_name = parameter_or(joint.parameters, "actuator_name");
    joint_data.info.command_mode = mujoco_simulation::CommandInterfaceType::None;
    joint_data.command.name = joint.name;
    joint_data.state.name = joint.name;

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
  }

  for (const auto& sensor : hardware_info.sensors) {
    if (is_sensor_type(sensor, "imu", error_message)) {
      ImuData imu_data;
      imu_data.name = sensor.name;
      imu_data.frame_id = parameter_or(sensor.parameters, "frame_id", sensor.name);
      imu_data.topic = parameter_or(sensor.parameters, "topic", sensor.name + "/imu");
      imu_data.info.name = sensor.name;
      imu_data.info.framequat_sensor_name =
          parameter_or(sensor.parameters, "mujoco_orientation_sensor");
      imu_data.info.gyro_sensor_name = parameter_or(sensor.parameters, "mujoco_gyro_sensor");
      imu_data.info.accelerometer_sensor_name =
          parameter_or(sensor.parameters, "mujoco_accel_sensor");

      if (imu_data.info.framequat_sensor_name.empty() || imu_data.info.gyro_sensor_name.empty() ||
          imu_data.info.accelerometer_sensor_name.empty()) {
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

      for (const auto& state_interface : sensor.state_interfaces) {
        if (!is_imu_state_interface(state_interface.name)) {
          error_message = "Unsupported IMU state interface '" + state_interface.name +
                          "' on sensor '" + sensor.name + "'.";
          return false;
        }
        imu_data.state_interfaces.push_back(state_interface.name);
      }

      parsed.imus.push_back(std::move(imu_data));
      continue;
    }

    if (is_sensor_type(sensor, "camera", error_message)) {
      if (parsed.simulation.render_mode != mujoco_simulation::RenderMode::Viewer) {
        error_message = "Camera '" + sensor.name + "' requires render_mode=viewer.";
        return false;
      }

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
      camera_data.info.name = sensor.name;
      camera_data.info.camera_name =
          parameter_or(sensor.parameters, "mujoco_camera_name", sensor.name);
      camera_data.info.enable_rgb = parameter_as_bool(sensor.parameters, "enable_rgb", true);
      camera_data.info.enable_depth = parameter_as_bool(sensor.parameters, "enable_depth", false);
      if (!parse_int_parameter(sensor.parameters, "width", 640, &camera_data.info.width,
                               error_message) ||
          !parse_int_parameter(sensor.parameters, "height", 480, &camera_data.info.height,
                               error_message)) {
        return false;
      }

      parsed.cameras.push_back(std::move(camera_data));
      continue;
    }

    if (is_sensor_type(sensor, "lidar", error_message)) {
      LidarData lidar_data;
      lidar_data.name = sensor.name;
      lidar_data.frame_id = parameter_or(sensor.parameters, "frame_id", sensor.name);
      lidar_data.topic = parameter_or(sensor.parameters, "scan_topic", sensor.name + "/scan");
      lidar_data.info.name = sensor.name;
      lidar_data.info.frame_name = lidar_data.frame_id;
      lidar_data.info.sensor_prefix = parameter_or(sensor.parameters, "sensor_prefix", sensor.name);
      if (!parse_double_parameter(sensor.parameters, "angle_min", 0.0, &lidar_data.info.angle_min,
                                  error_message) ||
          !parse_double_parameter(sensor.parameters, "angle_max", 0.0, &lidar_data.info.angle_max,
                                  error_message) ||
          !parse_double_parameter(sensor.parameters, "angle_increment", 0.0,
                                  &lidar_data.info.angle_increment, error_message) ||
          !parse_double_parameter(sensor.parameters, "range_min", 0.0, &lidar_data.info.range_min,
                                  error_message) ||
          !parse_double_parameter(sensor.parameters, "range_max", 0.0, &lidar_data.info.range_max,
                                  error_message)) {
        return false;
      }

      parsed.lidars.push_back(std::move(lidar_data));
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
    mb.info.name = raw_name;
    mb.command = {};
    mb.state = {};

    const std::string type_str = parameter_or(hardware_info.hardware_parameters,
                                              "mobile_base_" + std::to_string(i) + "_type");
    if (type_str == "differential") {
      mb.info.type = mujoco_simulation::MobileBaseType::Differential;
    } else if (type_str == "omnidirectional") {
      mb.info.type = mujoco_simulation::MobileBaseType::Omnidirectional;
    } else {
      error_message = "Invalid mobile_base type '" + type_str + "' for '" + raw_name +
                      "'. Must be 'differential' or 'omnidirectional'.";
      return false;
    }

    const std::string joints_str = parameter_or(
        hardware_info.hardware_parameters, "mobile_base_" + std::to_string(i) + "_traction_joints");
    if (joints_str.empty()) {
      error_message = "Mobile base '" + raw_name + "' requires traction_joints parameter.";
      return false;
    }
    {
      std::stringstream joint_stream(joints_str);
      std::string joint_name;
      while (std::getline(joint_stream, joint_name, ',')) {
        mb.info.traction_joint_names.push_back(joint_name);
      }
    }

    if (!parse_double_parameter(hardware_info.hardware_parameters,
                                "mobile_base_" + std::to_string(i) + "_wheel_radius", 0.0,
                                &mb.info.wheel_radius, error_message)) {
      return false;
    }
    if (!parse_double_parameter(hardware_info.hardware_parameters,
                                "mobile_base_" + std::to_string(i) + "_track_width", 0.0,
                                &mb.info.track_width, error_message)) {
      return false;
    }
    if (mb.info.type == mujoco_simulation::MobileBaseType::Omnidirectional) {
      if (!parse_double_parameter(hardware_info.hardware_parameters,
                                  "mobile_base_" + std::to_string(i) + "_wheel_base", 0.0,
                                  &mb.info.wheel_base, error_message)) {
        return false;
      }
    }

    mb.info.base_frame_id =
        parameter_or(hardware_info.hardware_parameters,
                     "mobile_base_" + std::to_string(i) + "_base_frame_id", "base_link");
    mb.info.odom_frame_id =
        parameter_or(hardware_info.hardware_parameters,
                     "mobile_base_" + std::to_string(i) + "_odom_frame_id", "odom");

    parsed.mobile_bases.push_back(std::move(mb));
  }

  *config = std::move(parsed);
  error_message.clear();
  return true;
}

}  // namespace mujoco_hardware
