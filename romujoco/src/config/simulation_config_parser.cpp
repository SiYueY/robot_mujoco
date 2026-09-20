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

namespace romujoco {
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
bool SimulationConfigParser::parse_limit(const tinyxml2::XMLElement* axis, Limit& limit) {
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
    if (const char* actuation = element.Attribute(config_names::kActuation); actuation != nullptr) {
        const std::string value = trim_copy(actuation);
        if (value == "active")
            info.actuation = JointActuation::Active;
        else if (value == "passive")
            info.actuation = JointActuation::Passive;
        else
            return false;
    }
    if (info.actuation == JointActuation::Passive) {
        if (element.Attribute(config_names::kActuator) != nullptr ||
            element.Attribute(config_names::kMode) != nullptr ||
            element.FirstChildElement(config_names::kControl) != nullptr ||
            element.FirstChildElement(config_names::kLimit) != nullptr)
            return false;
        info.actuator_name.clear();
        info.default_mode = JointMode::None;
        info.allowed_modes.clear();
        return true;
    }
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
bool SimulationConfigParser::parse_gripper(
    const tinyxml2::XMLElement& element, ComponentId maximum, GripperInfo& info) {
    if (!allowed(
            element, {config_names::kFinger, config_names::kControl, config_names::kLimit,
                      config_names::kStall}) ||
        !id(element, maximum, info.id) || !required(element, config_names::kName, info.name) ||
        element.Attribute(config_names::kUpdateRate) != nullptr)
        return false;
    // As for Joint, a missing period means "update every physics step" and is
    // resolved against the configured physics period after parsing.
    info.period = 0.0;
    if (!number(element, config_names::kPeriod, info.period)) return false;

    std::size_t finger_count = 0;
    for (const tinyxml2::XMLElement* finger = element.FirstChildElement(config_names::kFinger);
         finger != nullptr; finger = finger->NextSiblingElement(config_names::kFinger)) {
        if (finger_count >= kGripperFingerCount || !allowed(*finger, {}) ||
            !required(*finger, config_names::kJoint, info.fingers[finger_count].joint_name))
            return false;
        const char* actuator = finger->Attribute(config_names::kActuator);
        if (actuator != nullptr) info.fingers[finger_count].actuator_name = trim_copy(actuator);
        ++finger_count;
    }
    if (finger_count != kGripperFingerCount) return false;

    const tinyxml2::XMLElement* control = element.FirstChildElement(config_names::kControl);
    if (control != nullptr &&
        (!allowed(*control, {}) ||
         !number(*control, config_names::kStiffness, info.control.stiffness) ||
         !number(*control, config_names::kDamping, info.control.damping)))
        return false;

    const tinyxml2::XMLElement* limits = element.FirstChildElement(config_names::kLimit);
    if (limits != nullptr &&
        (!allowed(
             *limits, {config_names::kWidth, config_names::kVelocity, config_names::kEffort}) ||
         !parse_limit(limits->FirstChildElement(config_names::kWidth), info.width_limits) ||
         !parse_limit(limits->FirstChildElement(config_names::kVelocity), info.velocity_limits) ||
         !parse_limit(limits->FirstChildElement(config_names::kEffort), info.effort_limits)))
        return false;

    const tinyxml2::XMLElement* stall = element.FirstChildElement(config_names::kStall);
    if (stall != nullptr &&
        (!allowed(*stall, {}) ||
         !number(*stall, config_names::kWidthTolerance, info.stall.width_tolerance) ||
         !number(*stall, config_names::kVelocityThreshold, info.stall.velocity_threshold) ||
         !number(*stall, config_names::kEffortRatio, info.stall.effort_ratio) ||
         !number(*stall, config_names::kTimeout, info.stall.timeout)))
        return false;
    return true;
}

bool SimulationConfigParser::parse_components(
    const tinyxml2::XMLElement* robot, ComponentId maximum, ComponentConfigList& out,
    const ParseFailure& failure) {
    if (robot == nullptr) return true;
    if (!allowed(
            *robot, {config_names::kJoint, config_names::kGripper, config_names::kImu,
                     config_names::kCamera, config_names::kLidar, config_names::kMobileBase})) {
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
    for (const tinyxml2::XMLElement* e = robot->FirstChildElement(config_names::kGripper);
         e != nullptr; e = e->NextSiblingElement(config_names::kGripper)) {
        GripperInfo v;
        if (!parse_gripper(*e, maximum, v)) {
            log_error(failure, e, "", "invalid gripper syntax or attribute value");
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
        if (!allowed(*e, {config_names::kScan, config_names::kChannel, config_names::kRaycast}) ||
            !id(*e, maximum, v.id) || !required(*e, config_names::kName, v.name) ||
            !required(*e, config_names::kFrameId, v.frame_id) ||
            !required(*e, config_names::kSite, v.site_name) ||
            e->Attribute(config_names::kUpdateRate) != nullptr ||
            !number(*e, config_names::kPeriod, v.period) ||
            !number(*e, config_names::kRangeMin, v.range_min, true) ||
            !number(*e, config_names::kRangeMax, v.range_max, true)) {
            log_error(failure, e, "", "invalid lidar syntax or attribute value");
            return false;
        }
        const char* output = e->Attribute(config_names::kOutput);
        if (output == nullptr) return false;
        const std::string output_name = trim_copy(output);
        if (output_name == "laser_scan")
            v.output = LidarOutput::LaserScan;
        else if (output_name == "point_cloud2")
            v.output = LidarOutput::PointCloud2;
        else
            return false;
        if (e->Attribute(config_names::kGeomGroupMask) != nullptr &&
            e->QueryUnsignedAttribute(config_names::kGeomGroupMask, &v.geom_group_mask) !=
                tinyxml2::XML_SUCCESS) {
            log_error(failure, e, config_names::kGeomGroupMask, "must be an unsigned integer");
            return false;
        }
        const tinyxml2::XMLElement* raycast = e->FirstChildElement(config_names::kRaycast);
        if (raycast != nullptr) {
            if (raycast->NextSiblingElement(config_names::kRaycast) != nullptr ||
                !allowed(*raycast, {}) ||
                !optional_bool(*raycast, config_names::kExcludeParentBody, v.exclude_parent_body)) {
                log_error(failure, raycast, "", "invalid lidar raycast definition");
                return false;
            }
            const char* groups = raycast->Attribute(config_names::kGeomGroups);
            if (groups != nullptr) {
                std::istringstream values(trim_copy(groups));
                std::string value;
                while (std::getline(values, value, ',')) {
                    std::size_t parsed = 0;
                    unsigned long group = 0;
                    try {
                        group = std::stoul(trim_copy(value), &parsed);
                    } catch (const std::exception&) {
                        log_error(
                            failure, raycast, config_names::kGeomGroups,
                            "must be comma-separated group indices");
                        return false;
                    }
                    if (parsed != trim_copy(value).size() || group >= 32U) {
                        log_error(
                            failure, raycast, config_names::kGeomGroups,
                            "group index is out of range");
                        return false;
                    }
                    v.geom_group_mask |= 1U << static_cast<unsigned>(group);
                }
            }
        } else if (!optional_bool(*e, config_names::kExcludeParentBody, v.exclude_parent_body)) {
            log_error(failure, e, config_names::kExcludeParentBody, "must be boolean");
            return false;
        }
        const tinyxml2::XMLElement* scan = e->FirstChildElement(config_names::kScan);
        if (scan == nullptr || scan->NextSiblingElement(config_names::kScan) != nullptr ||
            !allowed(*scan, {}) ||
            !number(*scan, config_names::kAzimuthStart, v.azimuth_start, true) ||
            !number(*scan, config_names::kAzimuthIncrement, v.azimuth_increment, true) ||
            scan->QueryUnsignedAttribute(config_names::kAzimuthSamples, &v.azimuth_samples) !=
                tinyxml2::XML_SUCCESS) {
            log_error(failure, e, "", "invalid lidar scan definition");
            return false;
        }
        for (const tinyxml2::XMLElement* channel = e->FirstChildElement(config_names::kChannel);
             channel != nullptr; channel = channel->NextSiblingElement(config_names::kChannel)) {
            LidarChannel item;
            if (!allowed(*channel, {}) ||
                !number(*channel, config_names::kElevation, item.elevation, true) ||
                !number(*channel, config_names::kAzimuthOffset, item.azimuth_offset)) {
                log_error(failure, channel, "", "invalid lidar channel definition");
                return false;
            }
            v.channels.push_back(item);
        }
        out.emplace_back(std::move(v));
    }
    for (const tinyxml2::XMLElement* e = robot->FirstChildElement(config_names::kMobileBase);
         e != nullptr; e = e->NextSiblingElement(config_names::kMobileBase)) {
        const char* raw_type = e->Attribute(config_names::kType);
        const std::string type = raw_type == nullptr ? "mecanum" : trim_copy(raw_type);
        if (type == "swerve") {
            SwerveMobileBaseInfo v;
            if (!allowed(*e, {config_names::kModule}) || !id(*e, maximum, v.common.id) ||
                !required(*e, config_names::kName, v.common.name) ||
                !required(*e, config_names::kBaseBody, v.common.base_body_name) ||
                e->Attribute(config_names::kUpdateRate) != nullptr ||
                !number(*e, config_names::kPeriod, v.common.period)) {
                log_error(failure, e, "", "invalid swerve mobile-base syntax or attribute value");
                return false;
            }
            const char* execution = e->Attribute(config_names::kExecution);
            if (execution == nullptr || trim_copy(execution) != "dynamic") {
                log_error(
                    failure, e, config_names::kExecution, "swerve requires execution='dynamic'");
                return false;
            }
            v.common.execution_mode = MobileBaseExecutionMode::Dynamic;
            for (const tinyxml2::XMLElement* module = e->FirstChildElement(config_names::kModule);
                 module != nullptr; module = module->NextSiblingElement(config_names::kModule)) {
                SwerveModuleInfo item;
                if (!required(*module, config_names::kName, item.name) ||
                    !number(*module, config_names::kX, item.position_x, true) ||
                    !number(*module, config_names::kY, item.position_y, true) ||
                    !number(*module, config_names::kRadius, item.wheel_radius, true) ||
                    !required(*module, config_names::kSteeringJoint, item.steering_joint_name) ||
                    !required(
                        *module, config_names::kSteeringActuator, item.steering_actuator_name) ||
                    !required(*module, config_names::kDriveJoint, item.drive_joint_name) ||
                    !required(*module, config_names::kDriveActuator, item.drive_actuator_name)) {
                    log_error(failure, module, "", "invalid swerve module attribute");
                    return false;
                }
                v.modules.push_back(std::move(item));
            }
            out.emplace_back(std::move(v));
            continue;
        }
        if (type != "mecanum") {
            log_error(failure, e, config_names::kType, "unsupported mobile-base type");
            return false;
        }
        MecanumMobileBaseInfo v;
        if (!allowed(*e, {config_names::kWheel}) || !id(*e, maximum, v.common.id) ||
            !required(*e, config_names::kName, v.common.name) ||
            !required(*e, config_names::kBaseBody, v.common.base_body_name) ||
            e->Attribute(config_names::kUpdateRate) != nullptr ||
            !number(*e, config_names::kPeriod, v.common.period)) {
            log_error(failure, e, "", "invalid mobile-base syntax or attribute value");
            return false;
        }
        v.common.execution_mode = MobileBaseExecutionMode::Kinematic;
        const char* execution = e->Attribute(config_names::kExecution);
        if (execution != nullptr && trim_copy(execution) != "kinematic") {
            log_error(
                failure, e, config_names::kExecution, "mecanum requires execution='kinematic'");
            return false;
        }
        if (e->Attribute(config_names::kRadius) != nullptr ||
            e->Attribute(config_names::kLegacyWheelRadius) != nullptr ||
            !number(*e, config_names::kWheelBase, v.wheel_base, true) ||
            !number(*e, config_names::kTrackWidth, v.track_width, true)) {
            log_error(failure, e, "", "invalid mobile-base geometry attribute");
            return false;
        }
        std::array<bool, kMecanumWheelCount> seen_wheel_indices{};
        for (const tinyxml2::XMLElement* wheel = e->FirstChildElement(config_names::kWheel);
             wheel != nullptr; wheel = wheel->NextSiblingElement(config_names::kWheel)) {
            std::string index;
            if (!required(*wheel, config_names::kIndex, index)) {
                log_error(failure, wheel, "", "invalid mobile-base wheel attribute");
                return false;
            }
            const std::array<std::string_view, kMecanumWheelCount> names{
                "front_left", "front_right", "rear_left", "rear_right"};
            const auto it = std::find(names.begin(), names.end(), index);
            if (it == names.end()) {
                log_error(failure, wheel, config_names::kIndex, "invalid mobile-base wheel index");
                return false;
            }
            const std::size_t wheel_index =
                static_cast<std::size_t>(std::distance(names.begin(), it));
            if (seen_wheel_indices[wheel_index] ||
                !required(*wheel, config_names::kName, v.wheels[wheel_index].joint_name) ||
                !number(*wheel, config_names::kRadius, v.wheels[wheel_index].radius, true) ||
                !number(*wheel, config_names::kDirection, v.wheels[wheel_index].direction, true) ||
                !number(
                    *wheel, config_names::kSpeedResponse, v.wheels[wheel_index].speed_response)) {
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
    // Joint/Gripper/IMU/MobileBase means "update every physics step", which here
    // becomes the parsed physics period so components never see a sentinel 0.
    const double physics_period = parsed.scheduler.physics_period;
    for (ComponentConfig& component : parsed.components) {
        std::visit(
            [physics_period](auto& info) {
                using Info = std::decay_t<decltype(info)>;
                if constexpr (
                    std::is_same_v<Info, JointInfo> || std::is_same_v<Info, GripperInfo> ||
                    std::is_same_v<Info, ImuInfo> || std::is_same_v<Info, MecanumMobileBaseInfo> ||
                    std::is_same_v<Info, SwerveMobileBaseInfo>) {
                    if constexpr (
                        std::is_same_v<Info, MecanumMobileBaseInfo> ||
                        std::is_same_v<Info, SwerveMobileBaseInfo>) {
                        if (info.common.period == 0.0) info.common.period = physics_period;
                    } else if (info.period == 0.0)
                        info.period = physics_period;
                }
            },
            component);
    }
    if (!SimulationConfigValidator::validate(parsed)) return false;
    config = std::move(parsed);
    return true;
}
}  // namespace romujoco
