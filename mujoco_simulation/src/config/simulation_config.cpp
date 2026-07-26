#include "mujoco_simulation/config/simulation_config.hpp"

#include <cmath>
#include <exception>
#include <filesystem>
#include <optional>
#include <sstream>
#include <unordered_set>

#include "tinyxml2.h"

namespace mujoco_simulation {
namespace {
using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;
using tinyxml2::XMLNode;

std::string trim_copy(const std::string &value) {
  const auto first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos)
    return "";
  return value.substr(first, value.find_last_not_of(" \t\r\n") - first + 1U);
}
std::string text(const XMLElement &element) {
  return element.GetText() == nullptr ? "" : trim_copy(element.GetText());
}
bool allowed(const XMLElement &element,
             std::initializer_list<const char *> names) {
  for (const XMLNode *node = element.FirstChild(); node != nullptr;
       node = node->NextSibling()) {
    const XMLElement *child = node->ToElement();
    if (child == nullptr)
      continue;
    bool found = false;
    for (const char *name : names)
      if (std::string(child->Name()) == name)
        found = true;
    if (!found)
      return false;
  }
  return true;
}
bool required(const XMLElement &element, const char *attribute,
              std::string &out) {
  const char *raw = element.Attribute(attribute);
  if (raw == nullptr)
    return false;
  out = trim_copy(raw);
  return !out.empty();
}
bool number(const XMLElement &element, const char *attribute, double &out,
            bool mandatory = false) {
  const char *raw = element.Attribute(attribute);
  if (raw == nullptr)
    return !mandatory;
  std::string value = trim_copy(raw);
  std::size_t parsed = 0;
  try {
    out = std::stod(value, &parsed);
  } catch (const std::exception &) {
    return false;
  }
  return parsed == value.size() && std::isfinite(out);
}
template <std::size_t Size>
bool number_array(const XMLElement &element, const char *attribute,
                  std::array<double, Size> &out) {
  const char *raw = element.Attribute(attribute);
  if (raw == nullptr)
    return true;
  std::istringstream values(trim_copy(raw));
  for (std::size_t index = 0; index < Size; ++index) {
    std::string value;
    if (!std::getline(values, value, ','))
      return false;
    value = trim_copy(value);
    std::size_t parsed = 0;
    try {
      out[index] = std::stod(value, &parsed);
    } catch (const std::exception &) {
      return false;
    }
    if (parsed != value.size() || !std::isfinite(out[index]))
      return false;
  }
  std::string extra;
  return !std::getline(values, extra, ',');
}
bool id(const XMLElement &element, ComponentId maximum, ComponentId &out) {
  const char *raw = element.Attribute("id");
  if (raw == nullptr)
    return false;
  const std::string value = trim_copy(raw);
  if (value.empty() || value.front() == '+' || value.front() == '-')
    return false;
  std::size_t parsed = 0;
  unsigned long long numeric = 0;
  try {
    numeric = std::stoull(value, &parsed);
  } catch (const std::exception &) {
    return false;
  }
  if (parsed != value.size() ||
      numeric > std::numeric_limits<ComponentId>::max())
    return false;
  out = static_cast<ComponentId>(numeric);
  return out != kInvalidComponentId && out <= maximum;
}
bool parse_limit(const XMLElement *axis, Limit &limit) {
  if (axis == nullptr)
    return true;
  if (!allowed(*axis, {"min", "max"}))
    return false;
  const XMLElement *min = axis->FirstChildElement("min");
  const XMLElement *max = axis->FirstChildElement("max");
  if (min != nullptr) {
    std::string v = text(*min);
    try {
      limit.min = std::stod(v);
    } catch (...) {
      return false;
    }
  }
  if (max != nullptr) {
    std::string v = text(*max);
    try {
      limit.max = std::stod(v);
    } catch (...) {
      return false;
    }
  }
  return limit.min <= limit.max;
}
bool parse_joint(const XMLElement &element, ComponentId maximum,
                 JointInfo &info) {
  if (!allowed(element, {"position", "velocity", "limit"}) ||
      !id(element, maximum, info.id) ||
      !required(element, "name", info.joint_name) ||
      !number(element, "update_rate", info.update_rate))
    return false;
  info.actuator_name = info.joint_name;
  const char *actuator = element.Attribute("actuator");
  if (actuator != nullptr)
    info.actuator_name = trim_copy(actuator);
  const XMLElement *position = element.FirstChildElement("position");
  if (position != nullptr && (!allowed(*position, {"stiffness", "damping"})))
    return false;
  const XMLElement *velocity = element.FirstChildElement("velocity");
  if (velocity != nullptr && (!allowed(*velocity, {"damping"})))
    return false;
  if (position != nullptr &&
      position->FirstChildElement("stiffness") != nullptr)
    try {
      info.position_stiffness =
          std::stod(text(*position->FirstChildElement("stiffness")));
    } catch (...) {
      return false;
    }
  if (position != nullptr && position->FirstChildElement("damping") != nullptr)
    try {
      info.position_damping =
          std::stod(text(*position->FirstChildElement("damping")));
    } catch (...) {
      return false;
    }
  if (velocity != nullptr && velocity->FirstChildElement("damping") != nullptr)
    try {
      info.velocity_damping =
          std::stod(text(*velocity->FirstChildElement("damping")));
    } catch (...) {
      return false;
    }
  const XMLElement *limits = element.FirstChildElement("limit");
  return limits == nullptr ||
         (allowed(*limits, {"position", "velocity", "effort"}) &&
          parse_limit(limits->FirstChildElement("position"),
                      info.position_limits) &&
          parse_limit(limits->FirstChildElement("velocity"),
                      info.velocity_limits) &&
          parse_limit(limits->FirstChildElement("effort"), info.effort_limits));
}
template <typename Info>
bool unique_entry(const Info &info, std::unordered_set<ComponentId> &ids,
                  std::unordered_set<std::string> &names,
                  const std::string &name) {
  return ids.insert(info.id).second && names.insert(name).second;
}
bool parse_components(const XMLElement *robot, ComponentId maximum,
                      ComponentConfigList &out) {
  if (robot == nullptr)
    return true;
  if (!allowed(*robot, {"joint", "imu", "camera", "lidar", "mobile_base"}))
    return false;
  std::unordered_set<ComponentId> joint_ids, imu_ids, camera_ids, lidar_ids,
      base_ids;
  std::unordered_set<std::string> joint_names, imu_names, camera_names,
      lidar_names, base_names;
  for (const XMLElement *e = robot->FirstChildElement("joint"); e != nullptr;
       e = e->NextSiblingElement("joint")) {
    JointInfo v;
    if (!parse_joint(*e, maximum, v) ||
        !unique_entry(v, joint_ids, joint_names, v.joint_name))
      return false;
    out.emplace_back(std::move(v));
  }
  for (const XMLElement *e = robot->FirstChildElement("imu"); e != nullptr;
       e = e->NextSiblingElement("imu")) {
    ImuInfo v;
    if (!id(*e, maximum, v.id) || !required(*e, "name", v.name) ||
        !required(*e, "frame_id", v.frame_id) ||
        !required(*e, "framequat_sensor", v.framequat_sensor_name) ||
        !required(*e, "gyro_sensor", v.gyro_sensor_name) ||
        !required(*e, "accelerometer_sensor", v.accelerometer_sensor_name) ||
        !number(*e, "update_rate", v.update_rate) ||
        !number_array(*e, "orientation_covariance", v.orientation_covariance) ||
        !number_array(*e, "angular_velocity_covariance",
                      v.angular_velocity_covariance) ||
        !number_array(*e, "linear_acceleration_covariance",
                      v.linear_acceleration_covariance) ||
        !unique_entry(v, imu_ids, imu_names, v.name))
      return false;
    out.emplace_back(std::move(v));
  }
  for (const XMLElement *e = robot->FirstChildElement("camera"); e != nullptr;
       e = e->NextSiblingElement("camera")) {
    CameraConfig v;
    if (!id(*e, maximum, v.id) || !required(*e, "name", v.name) ||
        !required(*e, "frame_id", v.frame_id) ||
        !required(*e, "camera_name", v.camera_name) ||
        !required(*e, "optical_frame_id", v.optical_frame_id) ||
        !number(*e, "update_rate", v.update_rate) ||
        e->QueryIntAttribute("width", &v.width) != tinyxml2::XML_SUCCESS ||
        e->QueryIntAttribute("height", &v.height) != tinyxml2::XML_SUCCESS ||
        !unique_entry(v, camera_ids, camera_names, v.name))
      return false;
    e->QueryBoolAttribute("enable_rgb", &v.enable_rgb);
    e->QueryBoolAttribute("enable_depth", &v.enable_depth);
    out.emplace_back(std::move(v));
  }
  for (const XMLElement *e = robot->FirstChildElement("lidar"); e != nullptr;
       e = e->NextSiblingElement("lidar")) {
    LidarInfo v;
    if (!id(*e, maximum, v.id) || !required(*e, "name", v.name) ||
        !required(*e, "frame_id", v.frame_id) ||
        !required(*e, "sensor_prefix", v.sensor_prefix) ||
        !number(*e, "update_rate", v.update_rate) ||
        !number(*e, "angle_min", v.angle_min, true) ||
        !number(*e, "angle_max", v.angle_max, true) ||
        !number(*e, "angle_increment", v.angle_increment, true) ||
        !number(*e, "range_min", v.range_min, true) ||
        !number(*e, "range_max", v.range_max, true) ||
        !unique_entry(v, lidar_ids, lidar_names, v.name))
      return false;
    out.emplace_back(std::move(v));
  }
  for (const XMLElement *e = robot->FirstChildElement("mobile_base");
       e != nullptr; e = e->NextSiblingElement("mobile_base")) {
    MobileBaseInfo v;
    if (!allowed(*e, {"wheel"}) || !id(*e, maximum, v.id) ||
        !required(*e, "name", v.mobile_base_name) ||
        !required(*e, "base_body", v.base_body_name) ||
        !number(*e, "update_rate", v.update_rate) ||
        !unique_entry(v, base_ids, base_names, v.mobile_base_name))
      return false;
    const char *base = e->Attribute("base_frame_id");
    if (base != nullptr)
      v.base_frame_id = trim_copy(base);
    const char *odom = e->Attribute("odom_frame_id");
    if (odom != nullptr)
      v.odom_frame_id = trim_copy(odom);
    if (!number(*e, "wheel_radius", v.mecanum_info.wheel_radius, true) ||
        !number(*e, "wheel_base", v.mecanum_info.wheel_base, true) ||
        !number(*e, "track_width", v.mecanum_info.track_width, true))
      return false;
    std::size_t wheel_index = 0;
    for (const XMLElement *wheel = e->FirstChildElement("wheel");
         wheel != nullptr; wheel = wheel->NextSiblingElement("wheel")) {
      if (wheel_index >= MecanumWheelCount ||
          !required(*wheel, "name", v.mecanum_wheels[wheel_index].wheel_name) ||
          !required(*wheel, "actuator",
                    v.mecanum_wheels[wheel_index].actuator_name) ||
          !number(*wheel, "damping", v.mecanum_wheels[wheel_index].damping))
        return false;
      ++wheel_index;
    }
    if (wheel_index != MecanumWheelCount)
      return false;
    out.emplace_back(std::move(v));
  }
  return true;
}
std::optional<std::filesystem::path> resolve(const std::filesystem::path &file,
                                             const std::string &model) {
  if (model.empty())
    return std::nullopt;
  std::filesystem::path path(model);
  if (path.is_relative())
    path = file.parent_path() / path;
  return path.lexically_normal();
}
} // namespace

bool SimulationConfigParser::load_file(const std::string &path,
                                       SimulationConfig &config) const {
  XMLDocument document;
  if (path.empty() || document.LoadFile(path.c_str()) != tinyxml2::XML_SUCCESS)
    return false;
  const XMLElement *root = document.RootElement();
  if (root == nullptr || std::string(root->Name()) != "robot_mujoco" ||
      !allowed(*root, {"mujoco", "robot"}))
    return false;
  SimulationConfig parsed;
  if (const char *max = root->Attribute("max_component_id")) {
    std::string value = trim_copy(max);
    std::size_t count = 0;
    try {
      parsed.max_component_id =
          static_cast<ComponentId>(std::stoull(value, &count));
    } catch (...) {
      return false;
    }
    if (count != value.size() || parsed.max_component_id == kInvalidComponentId)
      return false;
  }
  const XMLElement *mujoco = root->FirstChildElement("mujoco");
  const XMLElement *mjcf =
      mujoco == nullptr ? nullptr : mujoco->FirstChildElement("mjcf");
  if (mujoco == nullptr || mjcf == nullptr || !allowed(*mujoco, {"mjcf"}))
    return false;
  const auto model = resolve(std::filesystem::path(path), text(*mjcf));
  if (!model)
    return false;
  parsed.model.model_path = model->string();
  if (!parse_components(root->FirstChildElement("robot"),
                        parsed.max_component_id, parsed.components))
    return false;
  config = std::move(parsed);
  return true;
}
} // namespace mujoco_simulation
