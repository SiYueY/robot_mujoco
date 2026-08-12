#include "config/simulation_config_parser.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <filesystem>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <type_traits>
#include <unordered_set>
#include <utility>

#include "tinyxml2.h"

#include "log/logging.hpp"
#include "component/component.hpp"
#include "config/simulation_config_data.hpp"
#include "config/simulation_config_validator.hpp"

namespace mujoco_simulation {
namespace {
constexpr std::size_t kMaximumComponentId{255};
}  // namespace

struct SimulationConfigParser::ParseFailure {
    const std::string& path;
};

void SimulationConfigParser::log_error(
    const ParseFailure& failure, const tinyxml2::XMLElement* element, const std::string& attribute,
    const std::string& message) {
    SIM_ERROR << "failed to parse simulation configuration '" << failure.path << "'"
              << (element == nullptr ? "" : " at line " + std::to_string(element->GetLineNum()))
              << (element == nullptr || element->Name() == nullptr
                      ? ""
                      : " element '" + std::string(element->Name()) + "'")
              << (attribute.empty() ? "" : " attribute '" + attribute + "'") << ": " << message;
}

std::string SimulationConfigParser::trim_copy(const std::string& value) {
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return "";
    return value.substr(first, value.find_last_not_of(" \t\r\n") - first + 1U);
}
std::string SimulationConfigParser::text(const tinyxml2::XMLElement& element) {
    return element.GetText() == nullptr ? "" : trim_copy(element.GetText());
}
bool SimulationConfigParser::allowed(
    const tinyxml2::XMLElement& element, std::initializer_list<const char*> names) {
    for (const tinyxml2::XMLNode* node = element.FirstChild(); node != nullptr;
         node = node->NextSibling()) {
        const tinyxml2::XMLElement* child = node->ToElement();
        if (child == nullptr) continue;
        bool found = false;
        for (const char* name : names)
            if (std::string(child->Name()) == name) found = true;
        if (!found) return false;
    }
    return true;
}
bool SimulationConfigParser::required(
    const tinyxml2::XMLElement& element, const char* attribute, std::string& out) {
    const char* raw = element.Attribute(attribute);
    if (raw == nullptr) return false;
    out = trim_copy(raw);
    return !out.empty();
}

bool SimulationConfigParser::parse_finite_double(const std::string& value, double& out) {
    const std::string trimmed = trim_copy(std::string(value));
    std::size_t parsed = 0;
    try {
        out = std::stod(trimmed, &parsed);
    } catch (const std::exception&) {
        return false;
    }
    return parsed == trimmed.size() && std::isfinite(out);
}

bool SimulationConfigParser::number(
    const tinyxml2::XMLElement& element, const char* attribute, double& out, bool mandatory) {
    const char* raw = element.Attribute(attribute);
    if (raw == nullptr) return !mandatory;
    return parse_finite_double(raw, out);
}
bool optional_bool(const tinyxml2::XMLElement& element, const char* attribute, bool& out) {
    return element.Attribute(attribute) == nullptr ||
           element.QueryBoolAttribute(attribute, &out) == tinyxml2::XML_SUCCESS;
}
bool SimulationConfigParser::number_array(
    const tinyxml2::XMLElement& element, const char* attribute, std::array<double, 9>& out) {
    const char* raw = element.Attribute(attribute);
    if (raw == nullptr) return true;
    std::istringstream values(trim_copy(raw));
    for (std::size_t index = 0; index < out.size(); ++index) {
        std::string value;
        if (!std::getline(values, value, ',')) return false;
        if (!parse_finite_double(value, out[index])) return false;
    }
    std::string extra;
    return !std::getline(values, extra, ',');
}
bool SimulationConfigParser::parse_component_id_value(
    const std::string& raw, ComponentId maximum, ComponentId& out) {
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

bool SimulationConfigParser::id(
    const tinyxml2::XMLElement& element, ComponentId maximum, ComponentId& out) {
    const char* raw = element.Attribute(config_names::kId);
    return raw != nullptr && parse_component_id_value(raw, maximum, out);
}
bool SimulationConfigParser::parse_limit(const tinyxml2::XMLElement* axis, JointLimit& limit) {
    if (axis == nullptr) return true;
    // Limit bounds are attributes so a configured axis remains compact.  Do
    // not accept the legacy <min>/<max> child-element form.
    return allowed(*axis, {}) && number(*axis, config_names::kMin, limit.min) &&
           number(*axis, config_names::kMax, limit.max);
}
bool SimulationConfigParser::parse_joint_mode(const std::string& value, JointMode& out) {
    if (value == config_names::kHybrid)
        out = JointMode::Hybrid;
    else if (value == config_names::kPosition)
        out = JointMode::Position;
    else if (value == config_names::kVelocity)
        out = JointMode::Velocity;
    else if (value == config_names::kEffort)
        out = JointMode::Effort;
    else
        return false;
    return true;
}
bool SimulationConfigParser::parse_joint(
    const tinyxml2::XMLElement& element, ComponentId maximum, JointInfo& info) {
    if (!allowed(element, {config_names::kControl, config_names::kLimit}) ||
        !id(element, maximum, info.id) ||
        !required(element, config_names::kName, info.joint_name) ||
        element.Attribute(config_names::kGravityCompensation) != nullptr ||
        element.Attribute(config_names::kUpdateRate) != nullptr ||
        !number(element, config_names::kPeriod, info.period))
        return false;
    info.actuator_name = info.joint_name;
    const char* actuator = element.Attribute(config_names::kActuator);
    if (actuator != nullptr) info.actuator_name = trim_copy(actuator);
    const char* mode = element.Attribute(config_names::kMode);
    if (mode == nullptr || !parse_joint_mode(trim_copy(mode), info.default_mode)) return false;
    const tinyxml2::XMLElement* control = element.FirstChildElement(config_names::kControl);
    if (control == nullptr) return false;
    {
        info.allowed_modes.clear();
        if (!allowed(
                *control, {config_names::kHybrid, config_names::kPosition, config_names::kVelocity,
                           config_names::kEffort}))
            return false;
        const tinyxml2::XMLElement* hybrid = control->FirstChildElement(config_names::kHybrid);
        const tinyxml2::XMLElement* position = control->FirstChildElement(config_names::kPosition);
        const tinyxml2::XMLElement* velocity = control->FirstChildElement(config_names::kVelocity);
        const tinyxml2::XMLElement* effort = control->FirstChildElement(config_names::kEffort);
        const auto unique = [control](const char* name) {
            const tinyxml2::XMLElement* first = control->FirstChildElement(name);
            return first == nullptr || first->NextSiblingElement(name) == nullptr;
        };
        if (!unique(config_names::kHybrid) || !unique(config_names::kPosition) ||
            !unique(config_names::kVelocity) || !unique(config_names::kEffort) ||
            (hybrid != nullptr &&
             (!allowed(*hybrid, {}) ||
              !number(*hybrid, config_names::kStiffness, info.hybrid.stiffness, true) ||
              !number(*hybrid, config_names::kDamping, info.hybrid.damping, true) ||
              !optional_bool(
                  *hybrid, config_names::kGravityCompensation,
                  info.hybrid.gravity_compensation))) ||
            (position != nullptr &&
             (!allowed(*position, {}) ||
              !number(*position, config_names::kStiffness, info.position.stiffness, true) ||
              !number(*position, config_names::kDamping, info.position.damping, true) ||
              !optional_bool(
                  *position, config_names::kGravityCompensation,
                  info.position.gravity_compensation))) ||
            (velocity != nullptr &&
             (!allowed(*velocity, {}) ||
              !number(*velocity, config_names::kDamping, info.velocity.damping, true) ||
              !optional_bool(
                  *velocity, config_names::kGravityCompensation,
                  info.velocity.gravity_compensation))) ||
            (effort != nullptr &&
             (!allowed(*effort, {}) || !optional_bool(
                                           *effort, config_names::kGravityCompensation,
                                           info.effort.gravity_compensation)))) {
            return false;
        }
        if (hybrid != nullptr) info.allowed_modes.set(JointMode::Hybrid);
        if (position != nullptr) info.allowed_modes.set(JointMode::Position);
        if (velocity != nullptr) info.allowed_modes.set(JointMode::Velocity);
        if (effort != nullptr) info.allowed_modes.set(JointMode::Effort);
    }
    const tinyxml2::XMLElement* limits = element.FirstChildElement(config_names::kLimit);
    return limits == nullptr ||
           (allowed(
                *limits,
                {config_names::kPosition, config_names::kVelocity, config_names::kEffort}) &&
            parse_limit(limits->FirstChildElement(config_names::kPosition), info.position_limits) &&
            parse_limit(limits->FirstChildElement(config_names::kVelocity), info.velocity_limits) &&
            parse_limit(limits->FirstChildElement(config_names::kEffort), info.effort_limits));
}
bool SimulationConfigParser::parse_components(
    const tinyxml2::XMLElement* robot, ComponentId maximum, ComponentConfigList& out,
    const ParseFailure& failure) {
    if (robot == nullptr) return true;
    if (!allowed(
            *robot, {config_names::kJoint, config_names::kImu, config_names::kCamera,
                     config_names::kLidar, config_names::kMobileBase})) {
        log_error(failure, robot, "", "robot has an unknown component element");
        return false;
    }
    for (const tinyxml2::XMLElement* e = robot->FirstChildElement(config_names::kJoint);
         e != nullptr; e = e->NextSiblingElement(config_names::kJoint)) {
        JointInfo v;
        if (!parse_joint(*e, maximum, v)) {
            log_error(failure, e, "", "invalid joint syntax or attribute value");
            return false;
        }
        out.emplace_back(std::move(v));
    }
    for (const tinyxml2::XMLElement* e = robot->FirstChildElement(config_names::kImu); e != nullptr;
         e = e->NextSiblingElement(config_names::kImu)) {
        ImuInfo v;
        if (!id(*e, maximum, v.id) || !required(*e, config_names::kName, v.name) ||
            !required(*e, config_names::kFrameId, v.frame_id) ||
            !required(*e, config_names::kFramequatSensor, v.framequat_sensor_name) ||
            !required(*e, config_names::kGyroSensor, v.gyro_sensor_name) ||
            !required(*e, config_names::kAccelerometerSensor, v.accelerometer_sensor_name) ||
            e->Attribute(config_names::kUpdateRate) != nullptr ||
            !number(*e, config_names::kPeriod, v.period) ||
            !number_array(*e, config_names::kOrientationCovariance, v.orientation_covariance) ||
            !number_array(
                *e, config_names::kAngularVelocityCovariance, v.angular_velocity_covariance) ||
            !number_array(
                *e, config_names::kLinearAccelerationCovariance,
                v.linear_acceleration_covariance)) {
            log_error(failure, e, "", "invalid IMU syntax or attribute value");
            return false;
        }
        out.emplace_back(std::move(v));
    }
    for (const tinyxml2::XMLElement* e = robot->FirstChildElement(config_names::kCamera);
         e != nullptr; e = e->NextSiblingElement(config_names::kCamera)) {
        CameraConfig v;
        if (!id(*e, maximum, v.id) || !required(*e, config_names::kName, v.name) ||
            !required(*e, config_names::kFrameId, v.frame_id) ||
            !required(*e, config_names::kCameraName, v.camera_name) ||
            !required(*e, config_names::kOpticalFrameId, v.optical_frame_id) ||
            e->Attribute(config_names::kUpdateRate) != nullptr ||
            !number(*e, config_names::kPeriod, v.period)) {
            log_error(failure, e, "", "invalid camera syntax or attribute value");
            return false;
        }
        if (e->QueryIntAttribute(config_names::kWidth, &v.width) != tinyxml2::XML_SUCCESS) {
            log_error(failure, e, config_names::kWidth, "camera width must be an integer");
            return false;
        }
        if (e->QueryIntAttribute(config_names::kHeight, &v.height) != tinyxml2::XML_SUCCESS) {
            log_error(failure, e, config_names::kHeight, "camera height must be an integer");
            return false;
        }
        if (e->Attribute(config_names::kEnableRgb) != nullptr &&
            e->QueryBoolAttribute(config_names::kEnableRgb, &v.enable_rgb) !=
                tinyxml2::XML_SUCCESS) {
            log_error(
                failure, e, config_names::kEnableRgb,
                "camera output enable attribute must be a boolean");
            return false;
        }
        if (e->Attribute(config_names::kEnableDepth) != nullptr &&
            e->QueryBoolAttribute(config_names::kEnableDepth, &v.enable_depth) !=
                tinyxml2::XML_SUCCESS) {
            log_error(
                failure, e, config_names::kEnableDepth,
                "camera output enable attribute must be a boolean");
            return false;
        }
        out.emplace_back(std::move(v));
    }
    for (const tinyxml2::XMLElement* e = robot->FirstChildElement(config_names::kLidar);
         e != nullptr; e = e->NextSiblingElement(config_names::kLidar)) {
        LidarInfo v;
        if (!id(*e, maximum, v.id) || !required(*e, config_names::kName, v.name) ||
            !required(*e, config_names::kFrameId, v.frame_id) ||
            !required(*e, config_names::kSensorPrefix, v.sensor_prefix) ||
            e->Attribute(config_names::kUpdateRate) != nullptr ||
            !number(*e, config_names::kPeriod, v.period) ||
            !number(*e, config_names::kAngleMin, v.angle_min, true) ||
            !number(*e, config_names::kAngleMax, v.angle_max, true) ||
            !number(*e, config_names::kAngleIncrement, v.angle_increment, true) ||
            !number(*e, config_names::kRangeMin, v.range_min, true) ||
            !number(*e, config_names::kRangeMax, v.range_max, true)) {
            log_error(failure, e, "", "invalid lidar syntax or attribute value");
            return false;
        }
        out.emplace_back(std::move(v));
    }
    for (const tinyxml2::XMLElement* e = robot->FirstChildElement(config_names::kMobileBase);
         e != nullptr; e = e->NextSiblingElement(config_names::kMobileBase)) {
        MobileBaseInfo v;
        if (!allowed(*e, {config_names::kWheel}) || !id(*e, maximum, v.id) ||
            !required(*e, config_names::kName, v.mobile_base_name) ||
            !required(*e, config_names::kBaseBody, v.base_body_name) ||
            !required(*e, config_names::kBaseJoint, v.base_joint_name) ||
            e->Attribute(config_names::kUpdateRate) != nullptr ||
            !number(*e, config_names::kPeriod, v.period)) {
            log_error(failure, e, "", "invalid mobile-base syntax or attribute value");
            return false;
        }
        const char* base = e->Attribute(config_names::kBaseFrameId);
        if (base != nullptr) v.base_frame_id = trim_copy(base);
        const char* odom = e->Attribute(config_names::kOdomFrameId);
        if (odom != nullptr) v.odom_frame_id = trim_copy(odom);
        const char* type = e->Attribute(config_names::kType);
        if (type != nullptr && trim_copy(type) != "mecanum") {
            log_error(failure, e, config_names::kType, "unsupported mobile-base type");
            return false;
        }
        if (e->Attribute(config_names::kRadius) != nullptr ||
            e->Attribute(config_names::kLegacyWheelRadius) != nullptr ||
            !number(*e, config_names::kWheelBase, v.mecanum_info.wheel_base, true) ||
            !number(*e, config_names::kTrackWidth, v.mecanum_info.track_width, true)) {
            log_error(failure, e, "", "invalid mobile-base geometry attribute");
            return false;
        }
        std::array<bool, MecanumWheelCount> seen_wheel_indices{};
        for (const tinyxml2::XMLElement* wheel = e->FirstChildElement(config_names::kWheel);
             wheel != nullptr; wheel = wheel->NextSiblingElement(config_names::kWheel)) {
            std::string index;
            if (!required(*wheel, config_names::kIndex, index)) {
                log_error(failure, wheel, "", "invalid mobile-base wheel attribute");
                return false;
            }
            const std::array<std::string_view, MecanumWheelCount> names{
                "front_left", "front_right", "rear_left", "rear_right"};
            const auto it = std::find(names.begin(), names.end(), index);
            if (it == names.end()) {
                log_error(failure, wheel, config_names::kIndex, "invalid mobile-base wheel index");
                return false;
            }
            const std::size_t wheel_index =
                static_cast<std::size_t>(std::distance(names.begin(), it));
            if (seen_wheel_indices[wheel_index] ||
                !required(*wheel, config_names::kName, v.mecanum_wheels[wheel_index].wheel_name) ||
                !number(
                    *wheel, config_names::kRadius, v.mecanum_wheels[wheel_index].radius, true) ||
                !number(
                    *wheel, config_names::kDirection, v.mecanum_wheels[wheel_index].direction,
                    true) ||
                !number(
                    *wheel, config_names::kSpeedResponse,
                    v.mecanum_wheels[wheel_index].speed_response)) {
                log_error(failure, wheel, "", "invalid mobile-base wheel attribute");
                return false;
            }
            seen_wheel_indices[wheel_index] = true;
        }
        if (std::find(seen_wheel_indices.begin(), seen_wheel_indices.end(), false) !=
            seen_wheel_indices.end()) {
            log_error(failure, e, config_names::kWheel, "mobile base requires exactly four wheels");
            return false;
        }
        out.emplace_back(std::move(v));
    }
    return true;
}
bool SimulationConfigParser::parse_simulation(
    const tinyxml2::XMLElement* simulation, SchedulerConfig& config, bool& viewer_enabled,
    const ParseFailure& failure) {
    if (simulation == nullptr) {
        log_error(failure, nullptr, "", "simulation timing configuration is missing");
        return false;
    }
    if (!allowed(*simulation, {config_names::kPhysics, config_names::kViewer})) {
        log_error(failure, simulation, "", "simulation has an unknown child element");
        return false;
    }
    const tinyxml2::XMLElement* physics = simulation->FirstChildElement(config_names::kPhysics);
    const tinyxml2::XMLElement* viewer = simulation->FirstChildElement(config_names::kViewer);
    if (physics == nullptr || physics->NextSiblingElement(config_names::kPhysics) != nullptr) {
        log_error(
            failure, simulation, config_names::kPhysics, "exactly one physics element is required");
        return false;
    }
    if (viewer == nullptr || viewer->NextSiblingElement(config_names::kViewer) != nullptr) {
        log_error(
            failure, simulation, config_names::kViewer, "exactly one viewer element is required");
        return false;
    }
    if (!number(*physics, config_names::kPeriod, config.physics_period, true)) {
        log_error(
            failure, physics, config_names::kPeriod, "physics period must be a finite number");
        return false;
    }
    if (!number(*viewer, config_names::kPeriod, config.viewer_period, true)) {
        log_error(failure, viewer, config_names::kPeriod, "viewer period must be a finite number");
        return false;
    }
    if (viewer->Attribute(config_names::kEnabled) != nullptr &&
        viewer->QueryBoolAttribute(config_names::kEnabled, &viewer_enabled) !=
            tinyxml2::XML_SUCCESS) {
        log_error(failure, viewer, config_names::kEnabled, "viewer enabled must be a boolean");
        return false;
    }
    return true;
}
std::optional<std::filesystem::path> SimulationConfigParser::resolve(
    const std::filesystem::path& file, const std::string& model) {
    if (model.empty()) return std::nullopt;
    std::filesystem::path path(model);
    if (path.is_relative()) path = file.parent_path() / path;
    return path.lexically_normal();
}
bool SimulationConfigParser::load_file(const std::string& path, SimulationConfig& config) const {
    ParseFailure failure{path};
    tinyxml2::XMLDocument document;
    if (path.empty()) {
        log_error(failure, nullptr, "", "configuration path is empty");
        return false;
    }
    if (document.LoadFile(path.c_str()) != tinyxml2::XML_SUCCESS) {
        SIM_ERROR << "failed to load simulation configuration '" << path << "' at line "
                  << document.ErrorLineNum() << ": "
                  << (document.ErrorStr() == nullptr ? "failed to load XML" : document.ErrorStr());
        return false;
    }
    const tinyxml2::XMLElement* root = document.RootElement();
    if (root == nullptr || std::string(root->Name()) != config_names::kRobotMujoco ||
        !allowed(*root, {config_names::kMujoco, config_names::kRobot, config_names::kSimulation})) {
        log_error(failure, root, "", "expected a robot_mujoco root with known children");
        return false;
    }
    SimulationConfig parsed;
    const tinyxml2::XMLElement* mujoco = root->FirstChildElement(config_names::kMujoco);
    const tinyxml2::XMLElement* mjcf =
        mujoco == nullptr ? nullptr : mujoco->FirstChildElement(config_names::kMjcf);
    if (mujoco == nullptr || mjcf == nullptr || !allowed(*mujoco, {config_names::kMjcf})) {
        log_error(failure, mujoco == nullptr ? root : mujoco, "", "expected one mjcf element");
        return false;
    }
    const auto model = resolve(std::filesystem::path(path), text(*mjcf));
    if (!model) {
        log_error(failure, mjcf, "", "MJCF model path must not be empty");
        return false;
    }
    parsed.model.model_path = model->string();
    const tinyxml2::XMLElement* simulation = root->FirstChildElement(config_names::kSimulation);
    if (simulation == nullptr ||
        simulation->NextSiblingElement(config_names::kSimulation) != nullptr) {
        log_error(
            failure, simulation == nullptr ? root : simulation, config_names::kSimulation,
            "exactly one simulation element is required");
        return false;
    }
    if (!parse_simulation(simulation, parsed.scheduler, parsed.viewer_enabled, failure)) {
        return false;
    }
    if (!parse_components(
            root->FirstChildElement(config_names::kRobot), kMaximumComponentId, parsed.components,
            failure)) {
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
    if (!SimulationConfigValidator::validate(parsed)) return false;
    config = std::move(parsed);
    return true;
}
}  // namespace mujoco_simulation
