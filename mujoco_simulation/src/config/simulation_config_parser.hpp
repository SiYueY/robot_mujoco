#pragma once

#include <array>
#include <filesystem>
#include <initializer_list>
#include <optional>
#include <string>

#include "config/simulation_config_validator.hpp"

namespace tinyxml2 {
class XMLElement;
}

namespace mujoco_simulation {

class SimulationConfigParser {
public:
    bool load_file(const std::string& path, SimulationConfig& config) const;

private:
    struct ParseFailure;

    static void log_error(
        const ParseFailure& failure, const tinyxml2::XMLElement* element,
        const std::string& attribute, const std::string& message);
    static std::string trim_copy(const std::string& value);
    static std::string text(const tinyxml2::XMLElement& element);
    static bool allowed(
        const tinyxml2::XMLElement& element, std::initializer_list<const char*> names);
    static bool required(
        const tinyxml2::XMLElement& element, const char* attribute, std::string& out);
    static bool parse_finite_double(const std::string& value, double& out);
    static bool number(
        const tinyxml2::XMLElement& element, const char* attribute, double& out,
        bool mandatory = false);
    static bool number_array(
        const tinyxml2::XMLElement& element, const char* attribute, std::array<double, 9>& out);
    static bool parse_component_id_value(
        const std::string& raw, ComponentId maximum, ComponentId& out);
    static bool id(const tinyxml2::XMLElement& element, ComponentId maximum, ComponentId& out);
    static bool parse_limit(const tinyxml2::XMLElement* axis, JointLimit& limit);
    static bool parse_joint_mode(const std::string& value, JointMode& out);
    static bool parse_joint(
        const tinyxml2::XMLElement& element, ComponentId maximum, JointInfo& info);
    static bool parse_components(
        const tinyxml2::XMLElement* robot, ComponentId maximum, ComponentConfigList& out,
        const ParseFailure& failure);
    static bool parse_simulation(
        const tinyxml2::XMLElement* simulation, SchedulerConfig& config, bool& viewer_enabled,
        const ParseFailure& failure);
    static std::optional<std::filesystem::path> resolve(
        const std::filesystem::path& file, const std::string& model);
};

}  // namespace mujoco_simulation
