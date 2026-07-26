#include "mujoco_simulation/config/simulation_config.hpp"

#include <cmath>
#include <exception>
#include <filesystem>
#include <optional>
#include <unordered_set>
#include <utility>

#include "mujoco_simulation/component/joint/joint_data.hpp"
#include "tinyxml2.h"

namespace mujoco_simulation {

namespace {

using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;
using tinyxml2::XMLNode;

std::string trim_copy(const std::string &value) {
  const std::size_t first = value.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  const std::size_t last = value.find_last_not_of(" \t\r\n");
  return value.substr(first, last - first + 1);
}

std::string element_text(const XMLElement &element) {
  const char *text = element.GetText();
  return text == nullptr ? "" : trim_copy(text);
}

bool parse_required_double(const XMLElement &element, double &out) {
  const std::string text = element_text(element);
  if (text.empty()) {
    return false;
  }

  std::size_t parsed = 0;
  double value = 0.0;
  try {
    value = std::stod(text, &parsed);
  } catch (const std::exception &) {
    return false;
  }
  if (parsed != text.size() || !std::isfinite(value)) {
    return false;
  }

  out = value;
  return true;
}

bool reject_unknown_children(
    const XMLElement &element,
    const std::unordered_set<std::string> &allowed_names) {
  for (const XMLNode *child = element.FirstChild(); child != nullptr;
       child = child->NextSibling()) {
    const XMLElement *child_element = child->ToElement();
    if (child_element == nullptr) {
      continue;
    }
    if (allowed_names.count(child_element->Name()) == 0U) {
      return false;
    }
  }
  return true;
}

bool parse_limit_axis(const XMLElement *element, Limit &limits) {
  if (element == nullptr) {
    return true;
  }
  if (!reject_unknown_children(*element, {"min", "max"})) {
    return false;
  }

  const XMLElement *min = element->FirstChildElement("min");
  if (min != nullptr && !parse_required_double(*min, limits.min)) {
    return false;
  }
  const XMLElement *max = element->FirstChildElement("max");
  if (max != nullptr && !parse_required_double(*max, limits.max)) {
    return false;
  }

  return limits.min <= limits.max;
}

bool parse_position_config(const XMLElement *element, JointInfo &info) {
  if (element == nullptr) {
    return true;
  }
  if (!reject_unknown_children(*element, {"stiffness", "damping"})) {
    return false;
  }

  const XMLElement *stiffness = element->FirstChildElement("stiffness");
  if (stiffness != nullptr &&
      !parse_required_double(*stiffness, info.position_stiffness)) {
    return false;
  }
  const XMLElement *damping = element->FirstChildElement("damping");
  if (damping != nullptr &&
      !parse_required_double(*damping, info.position_damping)) {
    return false;
  }

  return true;
}

bool parse_velocity_config(const XMLElement *element, JointInfo &info) {
  if (element == nullptr) {
    return true;
  }
  if (!reject_unknown_children(*element, {"damping"})) {
    return false;
  }

  const XMLElement *damping = element->FirstChildElement("damping");
  if (damping != nullptr &&
      !parse_required_double(*damping, info.velocity_damping)) {
    return false;
  }

  return true;
}

bool parse_joint_config(const XMLElement &element, JointInfo &info) {
  if (!reject_unknown_children(element, {"position", "velocity", "limit"})) {
    return false;
  }

  const char *name = element.Attribute("name");
  const std::string trimmed_name = name == nullptr ? "" : trim_copy(name);
  if (trimmed_name.empty()) {
    return false;
  }

  info.joint_name = trimmed_name;
  info.actuator_name = trimmed_name;

  if (!parse_position_config(element.FirstChildElement("position"), info) ||
      !parse_velocity_config(element.FirstChildElement("velocity"), info)) {
    return false;
  }

  const XMLElement *limit = element.FirstChildElement("limit");
  if (limit == nullptr) {
    return true;
  }
  if (!reject_unknown_children(*limit, {"position", "velocity", "effort"})) {
    return false;
  }

  return parse_limit_axis(limit->FirstChildElement("position"),
                          info.position_limits) &&
         parse_limit_axis(limit->FirstChildElement("velocity"),
                          info.velocity_limits) &&
         parse_limit_axis(limit->FirstChildElement("effort"),
                          info.effort_limits);
}

bool parse_robot_section(const XMLElement *robot,
                         ComponentConfigList &components) {
  if (robot == nullptr) {
    return true;
  }
  if (!reject_unknown_children(*robot, {"joint"})) {
    return false;
  }

  std::unordered_set<std::string> seen_joint_names;
  for (const XMLElement *joint = robot->FirstChildElement("joint");
       joint != nullptr; joint = joint->NextSiblingElement("joint")) {
    JointInfo info;
    if (!parse_joint_config(*joint, info) ||
        !seen_joint_names.insert(info.joint_name).second) {
      return false;
    }
    components.emplace_back(info);
  }

  return true;
}

std::optional<std::filesystem::path>
resolve_model_path(const std::filesystem::path &config_path,
                   const std::string &mjcf_path) {
  if (mjcf_path.empty()) {
    return std::nullopt;
  }

  std::filesystem::path resolved(mjcf_path);
  if (resolved.is_relative()) {
    resolved = config_path.parent_path() / resolved;
  }
  return resolved.lexically_normal();
}

} // namespace

bool SimulationConfigParser::load_file(const std::string &path,
                                       SimulationConfig &config) const {
  if (path.empty()) {
    return false;
  }

  XMLDocument document;
  const tinyxml2::XMLError load_status = document.LoadFile(path.c_str());
  if (load_status != tinyxml2::XML_SUCCESS) {
    return false;
  }

  const XMLElement *root = document.RootElement();
  if (root == nullptr || std::string(root->Name()) != "robot_mujoco" ||
      !reject_unknown_children(*root, {"mujoco", "robot"})) {
    return false;
  }

  const XMLElement *mujoco = root->FirstChildElement("mujoco");
  if (mujoco == nullptr || !reject_unknown_children(*mujoco, {"mjcf"})) {
    return false;
  }

  const XMLElement *mjcf = mujoco->FirstChildElement("mjcf");
  if (mjcf == nullptr) {
    return false;
  }

  const std::optional<std::filesystem::path> resolved_model_path =
      resolve_model_path(std::filesystem::path(path), element_text(*mjcf));
  if (!resolved_model_path.has_value()) {
    return false;
  }

  SimulationConfig parsed;
  parsed.model.model_path = resolved_model_path->string();
  if (!parse_robot_section(root->FirstChildElement("robot"),
                           parsed.components)) {
    return false;
  }

  config = std::move(parsed);
  return true;
}

} // namespace mujoco_simulation
