#include "config/simulation_config_parser.hpp"
#include "config/simulation_config_validator.hpp"

#include "component/component.hpp"

#include <cmath>
#include <exception>
#include <filesystem>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <type_traits>
#include <unordered_set>
#include <utility>

#include "common/compare.hpp"
#include "tinyxml2.h"

namespace mujoco_simulation {
namespace {
using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;
using tinyxml2::XMLNode;

// Hard bounds shared by the XML validation paths.  These limits are compile-time
// constants and are intentionally not configurable through the config file.
constexpr std::size_t kMaximumComponentId{65535};
constexpr int kMaximumCameraDimension{8192};
constexpr std::size_t kMaximumCameraOutputBytes{256U * 1024U * 1024U};

void set_error(
    ConfigError* error, const XMLElement* element, std::string attribute, std::string message) {
    if (error == nullptr || !error->message.empty()) return;
    error->line = element == nullptr ? 0 : element->GetLineNum();
    error->element = element == nullptr || element->Name() == nullptr ? "" : element->Name();
    error->attribute = std::move(attribute);
    error->message = std::move(message);
}

bool validate_camera(const CameraConfig& camera, ConfigError* error) {
    if (camera.width <= 0 || camera.width > kMaximumCameraDimension) {
        set_error(error, nullptr, "width", "camera width must be in 1..8192");
        return false;
    }
    if (camera.height <= 0 || camera.height > kMaximumCameraDimension) {
        set_error(error, nullptr, "height", "camera height must be in 1..8192");
        return false;
    }
    if (!camera.enable_rgb && !camera.enable_depth) {
        set_error(error, nullptr, "enable_rgb", "camera must enable RGB or depth output");
        return false;
    }
    const std::size_t bytes_per_pixel =
        (camera.enable_rgb ? 3U : 0U) + (camera.enable_depth ? 4U : 0U);
    const std::size_t width = static_cast<std::size_t>(camera.width);
    const std::size_t height = static_cast<std::size_t>(camera.height);
    if (height > std::numeric_limits<std::size_t>::max() / width ||
        width * height > std::numeric_limits<std::size_t>::max() / bytes_per_pixel ||
        width * height * bytes_per_pixel > kMaximumCameraOutputBytes) {
        set_error(error, nullptr, "width", "camera output exceeds the 256 MiB limit");
        return false;
    }
    return true;
}

bool validate_camera_names(const CameraConfig& camera, ConfigError* error) {
    if (camera.frame_id.empty()) {
        set_error(error, nullptr, "frame_id", "camera frame_id must not be empty");
        return false;
    }
    if (camera.camera_name.empty()) {
        set_error(error, nullptr, "camera_name", "MuJoCo camera name must not be empty");
        return false;
    }
    if (camera.optical_frame_id.empty()) {
        set_error(error, nullptr, "optical_frame_id", "camera optical_frame_id must not be empty");
        return false;
    }
    return true;
}

bool validate_lidar_names(const LidarInfo& lidar, ConfigError* error) {
    if (lidar.frame_id.empty()) {
        set_error(error, nullptr, "frame_id", "lidar frame_id must not be empty");
        return false;
    }
    if (lidar.sensor_prefix.empty()) {
        set_error(error, nullptr, "sensor_prefix", "lidar sensor prefix must not be empty");
        return false;
    }
    return true;
}

bool validate_mobile_base_names(const MobileBaseInfo& base, ConfigError* error) {
    if (base.base_body_name.empty()) {
        set_error(error, nullptr, "base_body", "mobile-base body name must not be empty");
        return false;
    }
    return true;
}

bool validate_lidar(const LidarInfo& lidar, ConfigError* error) {
    if (!std::isfinite(lidar.angle_min) || !std::isfinite(lidar.angle_max) ||
        !std::isfinite(lidar.angle_increment) || !std::isfinite(lidar.range_min) ||
        !std::isfinite(lidar.range_max)) {
        set_error(error, nullptr, "angle_min", "lidar parameters must be finite");
        return false;
    }
    if (lidar.angle_increment <= 0.0) {
        set_error(error, nullptr, "angle_increment", "lidar angle_increment must be positive");
        return false;
    }
    if (!math::less(lidar.angle_min, lidar.angle_max)) {
        set_error(error, nullptr, "angle_min", "lidar angle_min must be less than angle_max");
        return false;
    }
    if (lidar.range_min < 0.0) {
        set_error(error, nullptr, "range_min", "lidar range_min must be non-negative");
        return false;
    }
    if (!math::less(lidar.range_min, lidar.range_max)) {
        set_error(error, nullptr, "range_min", "lidar range_min must be less than range_max");
        return false;
    }
    return true;
}

bool validate_mobile_base(const MobileBaseInfo& base, ConfigError* error) {
    const MecanumInfo& mecanum = base.mecanum_info;
    if (!std::isfinite(mecanum.wheel_radius) || mecanum.wheel_radius <= 0.0) {
        set_error(error, nullptr, "wheel_radius", "wheel_radius must be finite and positive");
        return false;
    }
    if (!std::isfinite(mecanum.wheel_base) || mecanum.wheel_base <= 0.0) {
        set_error(error, nullptr, "wheel_base", "wheel_base must be finite and positive");
        return false;
    }
    if (!std::isfinite(mecanum.track_width) || mecanum.track_width <= 0.0) {
        set_error(error, nullptr, "track_width", "track_width must be finite and positive");
        return false;
    }
    std::unordered_set<std::string> wheel_names;
    std::unordered_set<std::string> actuator_names;
    for (const WheelInfo& wheel : base.mecanum_wheels) {
        if (wheel.wheel_name.empty() || wheel.actuator_name.empty()) {
            set_error(error, nullptr, "wheel", "mobile-base wheel names are required");
            return false;
        }
        if (!std::isfinite(wheel.damping) || wheel.damping < 0.0) {
            set_error(
                error, nullptr, "damping",
                "mobile-base wheel damping must be finite and non-negative");
            return false;
        }
        if (!wheel_names.insert(wheel.wheel_name).second) {
            set_error(error, nullptr, "name", "mobile-base wheel names must be unique");
            return false;
        }
        if (!actuator_names.insert(wheel.actuator_name).second) {
            set_error(error, nullptr, "actuator", "mobile-base actuator names must be unique");
            return false;
        }
    }
    return true;
}

bool validate_limit(const JointLimit& limit, const char* name, ConfigError* error) {
    if (std::isnan(limit.min) || std::isnan(limit.max) || math::greater(limit.min, limit.max)) {
        set_error(error, nullptr, name, "joint limit bounds are invalid");
        return false;
    }
    return true;
}

template <typename Values>
bool values_are_finite(const Values& values) {
    for (const double value : values) {
        if (!std::isfinite(value)) return false;
    }
    return true;
}

bool validate_joint(const JointInfo& joint, ConfigError* error) {
    if (joint.joint_name.empty() || joint.actuator_name.empty()) {
        set_error(error, nullptr, "name", "joint and actuator names are required");
        return false;
    }
    if (!std::isfinite(joint.position_stiffness) || joint.position_stiffness < 0.0 ||
        !std::isfinite(joint.position_damping) || joint.position_damping < 0.0 ||
        !std::isfinite(joint.velocity_damping) || joint.velocity_damping < 0.0) {
        set_error(
            error, nullptr, "damping",
            "joint stiffness and damping must be finite and non-negative");
        return false;
    }
    return validate_limit(joint.position_limits, "position", error) &&
           validate_limit(joint.velocity_limits, "velocity", error) &&
           validate_limit(joint.effort_limits, "effort", error);
}

bool validate_imu(const ImuInfo& imu, ConfigError* error) {
    if (imu.name.empty() || imu.frame_id.empty() || imu.framequat_sensor_name.empty() ||
        imu.gyro_sensor_name.empty() || imu.accelerometer_sensor_name.empty()) {
        set_error(error, nullptr, "name", "IMU names are required");
        return false;
    }
    if (!values_are_finite(imu.orientation_covariance) ||
        !values_are_finite(imu.angular_velocity_covariance) ||
        !values_are_finite(imu.linear_acceleration_covariance)) {
        set_error(error, nullptr, "covariance", "IMU covariance must be finite");
        return false;
    }
    return true;
}

bool validate_component_identity(
    ComponentId id, const std::string& name, double period, std::unordered_set<ComponentId>& ids,
    std::unordered_set<std::string>& names, const char* kind, ConfigError* error) {
    if (id == kInvalidComponentId || id > kMaximumComponentId) {
        set_error(error, nullptr, "id", std::string(kind) + " ID is outside the configured range");
        return false;
    }
    if (name.empty()) {
        set_error(error, nullptr, "name", std::string(kind) + " name is required");
        return false;
    }
    if (!std::isfinite(period) || period <= 0.0) {
        set_error(
            error, nullptr, "period", std::string(kind) + " period must be finite and positive");
        return false;
    }
    if (!ids.insert(id).second || !names.insert(name).second) {
        set_error(error, nullptr, "id", std::string(kind) + " IDs and names must be unique");
        return false;
    }
    return true;
}

std::string trim_copy(const std::string& value) {
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return "";
    return value.substr(first, value.find_last_not_of(" \t\r\n") - first + 1U);
}
std::string text(const XMLElement& element) {
    return element.GetText() == nullptr ? "" : trim_copy(element.GetText());
}
bool allowed(const XMLElement& element, std::initializer_list<const char*> names) {
    for (const XMLNode* node = element.FirstChild(); node != nullptr; node = node->NextSibling()) {
        const XMLElement* child = node->ToElement();
        if (child == nullptr) continue;
        bool found = false;
        for (const char* name : names)
            if (std::string(child->Name()) == name) found = true;
        if (!found) return false;
    }
    return true;
}
bool required(const XMLElement& element, const char* attribute, std::string& out) {
    const char* raw = element.Attribute(attribute);
    if (raw == nullptr) return false;
    out = trim_copy(raw);
    return !out.empty();
}

bool parse_finite_double(const std::string& value, double& out) {
    const std::string trimmed = trim_copy(std::string(value));
    std::size_t parsed = 0;
    try {
        out = std::stod(trimmed, &parsed);
    } catch (const std::exception&) {
        return false;
    }
    return parsed == trimmed.size() && std::isfinite(out);
}

bool parse_finite_element_text(const XMLElement& element, double& out) {
    return parse_finite_double(text(element), out);
}

bool number(const XMLElement& element, const char* attribute, double& out, bool mandatory = false) {
    const char* raw = element.Attribute(attribute);
    if (raw == nullptr) return !mandatory;
    return parse_finite_double(raw, out);
}
template <std::size_t Size>
bool number_array(const XMLElement& element, const char* attribute, std::array<double, Size>& out) {
    const char* raw = element.Attribute(attribute);
    if (raw == nullptr) return true;
    std::istringstream values(trim_copy(raw));
    for (std::size_t index = 0; index < Size; ++index) {
        std::string value;
        if (!std::getline(values, value, ',')) return false;
        if (!parse_finite_double(value, out[index])) return false;
    }
    std::string extra;
    return !std::getline(values, extra, ',');
}
bool parse_component_id_value(const std::string& raw, ComponentId maximum, ComponentId& out) {
    const std::string value = trim_copy(std::string(raw));
    if (value.empty() || value.front() == '+' || value.front() == '-') return false;
    std::size_t parsed = 0;
    unsigned long long numeric = 0;
    try {
        numeric = std::stoull(value, &parsed);
    } catch (const std::exception&) {
        return false;
    }
    if (parsed != value.size() || numeric > std::numeric_limits<ComponentId>::max()) return false;
    out = static_cast<ComponentId>(numeric);
    return out != kInvalidComponentId && out <= kMaximumComponentId && out <= maximum;
}

bool id(const XMLElement& element, ComponentId maximum, ComponentId& out) {
    const char* raw = element.Attribute("id");
    return raw != nullptr && parse_component_id_value(raw, maximum, out);
}
bool parse_limit(const XMLElement* axis, JointLimit& limit) {
    if (axis == nullptr) return true;
    if (!allowed(*axis, {"min", "max"})) return false;
    const XMLElement* min = axis->FirstChildElement("min");
    const XMLElement* max = axis->FirstChildElement("max");
    if ((min != nullptr && !parse_finite_element_text(*min, limit.min)) ||
        (max != nullptr && !parse_finite_element_text(*max, limit.max)))
        return false;
    return true;
}
bool parse_joint(const XMLElement& element, ComponentId maximum, JointInfo& info) {
    if (!allowed(element, {"position", "velocity", "limit"}) || !id(element, maximum, info.id) ||
        !required(element, "name", info.joint_name) ||
        element.Attribute("update_rate") != nullptr || !number(element, "period", info.period))
        return false;
    info.actuator_name = info.joint_name;
    const char* actuator = element.Attribute("actuator");
    if (actuator != nullptr) info.actuator_name = trim_copy(actuator);
    const XMLElement* position = element.FirstChildElement("position");
    if (position != nullptr && (!allowed(*position, {"stiffness", "damping"}))) return false;
    const XMLElement* velocity = element.FirstChildElement("velocity");
    if (velocity != nullptr && (!allowed(*velocity, {"damping"}))) return false;
    const XMLElement* stiffness =
        position == nullptr ? nullptr : position->FirstChildElement("stiffness");
    const XMLElement* position_damping =
        position == nullptr ? nullptr : position->FirstChildElement("damping");
    const XMLElement* velocity_damping =
        velocity == nullptr ? nullptr : velocity->FirstChildElement("damping");
    if ((stiffness != nullptr && !parse_finite_element_text(*stiffness, info.position_stiffness)) ||
        (position_damping != nullptr &&
         !parse_finite_element_text(*position_damping, info.position_damping)) ||
        (velocity_damping != nullptr &&
         !parse_finite_element_text(*velocity_damping, info.velocity_damping))) {
        return false;
    }
    const XMLElement* limits = element.FirstChildElement("limit");
    return limits == nullptr ||
           (allowed(*limits, {"position", "velocity", "effort"}) &&
            parse_limit(limits->FirstChildElement("position"), info.position_limits) &&
            parse_limit(limits->FirstChildElement("velocity"), info.velocity_limits) &&
            parse_limit(limits->FirstChildElement("effort"), info.effort_limits));
}
bool parse_components(
    const XMLElement* robot, ComponentId maximum, ComponentConfigList& out,
    std::vector<const XMLElement*>& elements, ConfigError* error) {
    if (robot == nullptr) return true;
    if (!allowed(*robot, {"joint", "imu", "camera", "lidar", "mobile_base"})) {
        set_error(error, robot, "", "robot has an unknown component element");
        return false;
    }
    for (const XMLElement* e = robot->FirstChildElement("joint"); e != nullptr;
         e = e->NextSiblingElement("joint")) {
        JointInfo v;
        if (!parse_joint(*e, maximum, v)) {
            set_error(error, e, "", "invalid joint syntax or attribute value");
            return false;
        }
        out.emplace_back(std::move(v));
        elements.push_back(e);
    }
    for (const XMLElement* e = robot->FirstChildElement("imu"); e != nullptr;
         e = e->NextSiblingElement("imu")) {
        ImuInfo v;
        if (!id(*e, maximum, v.id) || !required(*e, "name", v.name) ||
            !required(*e, "frame_id", v.frame_id) ||
            !required(*e, "framequat_sensor", v.framequat_sensor_name) ||
            !required(*e, "gyro_sensor", v.gyro_sensor_name) ||
            !required(*e, "accelerometer_sensor", v.accelerometer_sensor_name) ||
            e->Attribute("update_rate") != nullptr || !number(*e, "period", v.period) ||
            !number_array(*e, "orientation_covariance", v.orientation_covariance) ||
            !number_array(*e, "angular_velocity_covariance", v.angular_velocity_covariance) ||
            !number_array(*e, "linear_acceleration_covariance", v.linear_acceleration_covariance)) {
            set_error(error, e, "", "invalid IMU syntax or attribute value");
            return false;
        }
        out.emplace_back(std::move(v));
        elements.push_back(e);
    }
    for (const XMLElement* e = robot->FirstChildElement("camera"); e != nullptr;
         e = e->NextSiblingElement("camera")) {
        CameraConfig v;
        if (!id(*e, maximum, v.id) || !required(*e, "name", v.name) ||
            !required(*e, "frame_id", v.frame_id) || !required(*e, "camera_name", v.camera_name) ||
            !required(*e, "optical_frame_id", v.optical_frame_id) ||
            e->Attribute("update_rate") != nullptr || !number(*e, "period", v.period)) {
            set_error(error, e, "", "invalid camera syntax or attribute value");
            return false;
        }
        if (e->QueryIntAttribute("width", &v.width) != tinyxml2::XML_SUCCESS) {
            set_error(error, e, "width", "camera width must be an integer");
            return false;
        }
        if (e->QueryIntAttribute("height", &v.height) != tinyxml2::XML_SUCCESS) {
            set_error(error, e, "height", "camera height must be an integer");
            return false;
        }
        if (e->Attribute("enable_rgb") != nullptr &&
            e->QueryBoolAttribute("enable_rgb", &v.enable_rgb) != tinyxml2::XML_SUCCESS) {
            set_error(error, e, "enable_rgb", "camera output enable attribute must be a boolean");
            return false;
        }
        if (e->Attribute("enable_depth") != nullptr &&
            e->QueryBoolAttribute("enable_depth", &v.enable_depth) != tinyxml2::XML_SUCCESS) {
            set_error(error, e, "enable_depth", "camera output enable attribute must be a boolean");
            return false;
        }
        out.emplace_back(std::move(v));
        elements.push_back(e);
    }
    for (const XMLElement* e = robot->FirstChildElement("lidar"); e != nullptr;
         e = e->NextSiblingElement("lidar")) {
        LidarInfo v;
        if (!id(*e, maximum, v.id) || !required(*e, "name", v.name) ||
            !required(*e, "frame_id", v.frame_id) ||
            !required(*e, "sensor_prefix", v.sensor_prefix) ||
            e->Attribute("update_rate") != nullptr || !number(*e, "period", v.period) ||
            !number(*e, "angle_min", v.angle_min, true) ||
            !number(*e, "angle_max", v.angle_max, true) ||
            !number(*e, "angle_increment", v.angle_increment, true) ||
            !number(*e, "range_min", v.range_min, true) ||
            !number(*e, "range_max", v.range_max, true)) {
            set_error(error, e, "", "invalid lidar syntax or attribute value");
            return false;
        }
        out.emplace_back(std::move(v));
        elements.push_back(e);
    }
    for (const XMLElement* e = robot->FirstChildElement("mobile_base"); e != nullptr;
         e = e->NextSiblingElement("mobile_base")) {
        MobileBaseInfo v;
        if (!allowed(*e, {"wheel"}) || !id(*e, maximum, v.id) ||
            !required(*e, "name", v.mobile_base_name) ||
            !required(*e, "base_body", v.base_body_name) ||
            e->Attribute("update_rate") != nullptr || !number(*e, "period", v.period)) {
            set_error(error, e, "", "invalid mobile-base syntax or attribute value");
            return false;
        }
        const char* base = e->Attribute("base_frame_id");
        if (base != nullptr) v.base_frame_id = trim_copy(base);
        const char* odom = e->Attribute("odom_frame_id");
        if (odom != nullptr) v.odom_frame_id = trim_copy(odom);
        if (!number(*e, "wheel_radius", v.mecanum_info.wheel_radius, true) ||
            !number(*e, "wheel_base", v.mecanum_info.wheel_base, true) ||
            !number(*e, "track_width", v.mecanum_info.track_width, true)) {
            set_error(error, e, "", "invalid mobile-base geometry attribute");
            return false;
        }
        std::size_t wheel_index = 0;
        for (const XMLElement* wheel = e->FirstChildElement("wheel"); wheel != nullptr;
             wheel = wheel->NextSiblingElement("wheel")) {
            if (wheel_index >= MecanumWheelCount ||
                !required(*wheel, "name", v.mecanum_wheels[wheel_index].wheel_name) ||
                !required(*wheel, "actuator", v.mecanum_wheels[wheel_index].actuator_name) ||
                !number(*wheel, "damping", v.mecanum_wheels[wheel_index].damping)) {
                set_error(error, wheel, "", "invalid mobile-base wheel attribute");
                return false;
            }
            ++wheel_index;
        }
        if (wheel_index != MecanumWheelCount) {
            set_error(error, e, "wheel", "mobile base requires exactly four wheels");
            return false;
        }
        out.emplace_back(std::move(v));
        elements.push_back(e);
    }
    return true;
}
bool parse_simulation(
    const XMLElement* simulation, SchedulerConfig& config, bool& viewer_enabled,
    ConfigError* error) {
    if (simulation == nullptr) {
        set_error(error, nullptr, "", "simulation timing configuration is missing");
        return false;
    }
    if (!allowed(*simulation, {"physics", "viewer"})) {
        set_error(error, simulation, "", "simulation has an unknown child element");
        return false;
    }
    const XMLElement* physics = simulation->FirstChildElement("physics");
    const XMLElement* viewer = simulation->FirstChildElement("viewer");
    if (physics == nullptr || physics->NextSiblingElement("physics") != nullptr) {
        set_error(error, simulation, "physics", "exactly one physics element is required");
        return false;
    }
    if (viewer == nullptr || viewer->NextSiblingElement("viewer") != nullptr) {
        set_error(error, simulation, "viewer", "exactly one viewer element is required");
        return false;
    }
    if (!number(*physics, "period", config.physics_period, true)) {
        set_error(error, physics, "period", "physics period must be a finite number");
        return false;
    }
    if (!number(*viewer, "period", config.viewer_period, true)) {
        set_error(error, viewer, "period", "viewer period must be a finite number");
        return false;
    }
    if (viewer->Attribute("enabled") != nullptr &&
        viewer->QueryBoolAttribute("enabled", &viewer_enabled) != tinyxml2::XML_SUCCESS) {
        set_error(error, viewer, "enabled", "viewer enabled must be a boolean");
        return false;
    }
    return true;
}
std::optional<std::filesystem::path> resolve(
    const std::filesystem::path& file, const std::string& model) {
    if (model.empty()) return std::nullopt;
    std::filesystem::path path(model);
    if (path.is_relative()) path = file.parent_path() / path;
    return path.lexically_normal();
}
}  // namespace

bool validate_simulation_config_impl(const SimulationConfig& config, ConfigError* error) {
    if (error != nullptr) *error = {};
    if (config.model.model_path.empty()) {
        set_error(error, nullptr, "model_path", "model path must not be empty");
        return false;
    }
    if (config.viewer_startup_timeout <= std::chrono::milliseconds::zero()) {
        set_error(
            error, nullptr, "viewer_startup_timeout", "viewer startup timeout must be positive");
        return false;
    }
    if (config.camera_renderer.max_scene_geometries <= 0) {
        set_error(
            error, nullptr, "max_scene_geometries",
            "camera renderer scene geometry limit must be positive");
        return false;
    }
    if (!config.camera_renderer.allow_glfw_backend && !config.camera_renderer.allow_egl_backend) {
        set_error(
            error, nullptr, "camera_renderer", "camera renderer requires a GLFW or EGL backend");
        return false;
    }
    if (config.camera_renderer.completed_ticket_history == 0U) {
        set_error(
            error, nullptr, "completed_ticket_history",
            "camera renderer ticket history must be positive");
        return false;
    }
    if (!std::isfinite(config.scheduler.physics_period) || config.scheduler.physics_period <= 0.0) {
        set_error(error, nullptr, "physics_period", "physics_period must be finite and positive");
        return false;
    }
    if (!std::isfinite(config.scheduler.viewer_period) || config.scheduler.viewer_period <= 0.0) {
        set_error(error, nullptr, "viewer_period", "viewer_period must be finite and positive");
        return false;
    }
    std::unordered_set<ComponentId> joint_ids, imu_ids, camera_ids, lidar_ids, mobile_base_ids;
    std::unordered_set<std::string> joint_names, imu_names, camera_names, lidar_names,
        mobile_base_names;
    for (std::size_t component_index = 0; component_index < config.components.size();
         ++component_index) {
        const ComponentConfig& component = config.components[component_index];
        ConfigError component_error;
        const bool valid = std::visit(
            [&config, &component_error, &joint_ids, &imu_ids, &camera_ids, &lidar_ids,
             &mobile_base_ids, &joint_names, &imu_names, &camera_names, &lidar_names,
             &mobile_base_names](const auto& info) {
                using Info = std::decay_t<decltype(info)>;
                if constexpr (std::is_same_v<Info, JointInfo>) {
                    return validate_component_identity(
                               info.id, info.joint_name, info.period, joint_ids, joint_names,
                               "joint", &component_error) &&
                           validate_joint(info, &component_error);
                } else if constexpr (std::is_same_v<Info, ImuInfo>) {
                    return validate_component_identity(
                               info.id, info.name, info.period, imu_ids, imu_names, "IMU",
                               &component_error) &&
                           validate_imu(info, &component_error);
                } else if constexpr (std::is_same_v<Info, CameraConfig>) {
                    return validate_camera(info, &component_error) &&
                           validate_component_identity(
                               info.id, info.name, info.period, camera_ids, camera_names, "camera",
                               &component_error) &&
                           validate_camera_names(info, &component_error);
                } else if constexpr (std::is_same_v<Info, LidarInfo>) {
                    return validate_component_identity(
                               info.id, info.name, info.period, lidar_ids, lidar_names, "lidar",
                               &component_error) &&
                           validate_lidar_names(info, &component_error) &&
                           validate_lidar(info, &component_error);
                } else {
                    return validate_component_identity(
                               info.id, info.mobile_base_name, info.period, mobile_base_ids,
                               mobile_base_names, "mobile base", &component_error) &&
                           validate_mobile_base_names(info, &component_error) &&
                           validate_mobile_base(info, &component_error);
                }
            },
            component);
        if (!valid) {
            if (error != nullptr) {
                *error = std::move(component_error);
                error->component_index = component_index;
            }
            return false;
        }
    }
    return true;
}

bool SimulationConfigParser::load_file(
    const std::string& path, SimulationConfig& config, ConfigError* error) const {
    if (error != nullptr) *error = {};
    XMLDocument document;
    if (path.empty()) {
        set_error(error, nullptr, "", "configuration path is empty");
        return false;
    }
    if (document.LoadFile(path.c_str()) != tinyxml2::XML_SUCCESS) {
        if (error != nullptr) {
            error->line = document.ErrorLineNum();
            error->message =
                document.ErrorStr() == nullptr ? "failed to load XML" : document.ErrorStr();
        }
        return false;
    }
    const XMLElement* root = document.RootElement();
    if (root == nullptr || std::string(root->Name()) != "robot_mujoco" ||
        !allowed(*root, {"mujoco", "robot", "simulation"})) {
        set_error(error, root, "", "expected a robot_mujoco root with known children");
        return false;
    }
    SimulationConfig parsed;
    const XMLElement* mujoco = root->FirstChildElement("mujoco");
    const XMLElement* mjcf = mujoco == nullptr ? nullptr : mujoco->FirstChildElement("mjcf");
    if (mujoco == nullptr || mjcf == nullptr || !allowed(*mujoco, {"mjcf"})) {
        set_error(error, mujoco == nullptr ? root : mujoco, "", "expected one mjcf element");
        return false;
    }
    const auto model = resolve(std::filesystem::path(path), text(*mjcf));
    if (!model) {
        set_error(error, mjcf, "", "MJCF model path must not be empty");
        return false;
    }
    parsed.model.model_path = model->string();
    const XMLElement* simulation = root->FirstChildElement("simulation");
    if (simulation == nullptr || simulation->NextSiblingElement("simulation") != nullptr) {
        set_error(
            error, simulation == nullptr ? root : simulation, "simulation",
            "exactly one simulation element is required");
        return false;
    }
    if (!parse_simulation(simulation, parsed.scheduler, parsed.viewer_enabled, error)) {
        return false;
    }
    std::vector<const XMLElement*> component_elements;
    if (!parse_components(
            root->FirstChildElement("robot"), kMaximumComponentId, parsed.components,
            component_elements, error)) {
        if (error != nullptr && error->message.empty())
            set_error(
                error, root->FirstChildElement("robot"), "",
                "invalid robot component configuration");
        return false;
    }
    // Resolve config-layer period defaults.  A missing (or zero) period on
    // Joint/IMU/MobileBase means "update every physics step", which here
    // becomes the parsed physics period so components never see a sentinel 0.
    const double physics_period = parsed.scheduler.physics_period;
    for (ComponentConfig& component : parsed.components) {
        std::visit(
            [physics_period](auto& info) {
                using Info = std::decay_t<decltype(info)>;
                if constexpr (
                    std::is_same_v<Info, JointInfo> || std::is_same_v<Info, ImuInfo> ||
                    std::is_same_v<Info, MobileBaseInfo>) {
                    if (info.period == 0.0) info.period = physics_period;
                }
            },
            component);
    }
    ConfigError validation_error;
    if (!SimulationConfigValidator::validate(parsed, &validation_error)) {
        if (error != nullptr) {
            *error = std::move(validation_error);
            if (error->component_index < component_elements.size()) {
                const XMLElement* element = component_elements[error->component_index];
                error->line = element->GetLineNum();
                error->element = element->Name() == nullptr ? "" : element->Name();
            } else if (
                error->attribute == "physics_period" || error->attribute == "viewer_period") {
                const XMLElement* element = simulation->FirstChildElement(
                    error->attribute == "physics_period" ? "physics" : "viewer");
                if (element != nullptr) {
                    error->line = element->GetLineNum();
                    error->element = element->Name() == nullptr ? "" : element->Name();
                    error->attribute = "period";
                }
            }
        }
        return false;
    }
    config = std::move(parsed);
    return true;
}
}  // namespace mujoco_simulation
