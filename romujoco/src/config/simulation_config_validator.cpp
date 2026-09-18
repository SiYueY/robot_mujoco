#include "config/simulation_config_validator.hpp"

#include <cmath>
#include <limits>
#include <type_traits>
#include <unordered_set>

#include "common/compare.hpp"
#include "log/logging.hpp"
#include "config/simulation_config_data.hpp"

namespace romujoco {
namespace {
constexpr std::size_t kMaximumComponentId{255};
constexpr int kMaximumCameraDimension{8192};
constexpr std::size_t kMaximumCameraOutputBytes{256U * 1024U * 1024U};

template <typename Values>
bool values_are_finite(const Values& values) {
    for (const double value : values)
        if (!std::isfinite(value)) return false;
    return true;
}
}  // namespace

void SimulationConfigValidator::log_error(const std::string& field, const std::string& message) {
    SIM_ERROR << "invalid simulation configuration" << (field.empty() ? "" : " (" + field + ")")
              << ": " << message;
}

bool SimulationConfigValidator::validate_camera(const CameraConfig& camera) {
    if (camera.width <= 0 || camera.width > kMaximumCameraDimension) {
        log_error(config_names::kWidth, "camera width must be in 1..8192");
        return false;
    }
    if (camera.height <= 0 || camera.height > kMaximumCameraDimension) {
        log_error(config_names::kHeight, "camera height must be in 1..8192");
        return false;
    }
    if (!camera.enable_rgb && !camera.enable_depth) {
        log_error(config_names::kEnableRgb, "camera must enable RGB or depth output");
        return false;
    }
    const std::size_t bytes_per_pixel =
        (camera.enable_rgb ? 3U : 0U) + (camera.enable_depth ? 4U : 0U);
    const std::size_t width = static_cast<std::size_t>(camera.width);
    const std::size_t height = static_cast<std::size_t>(camera.height);
    if (height > std::numeric_limits<std::size_t>::max() / width ||
        width * height > std::numeric_limits<std::size_t>::max() / bytes_per_pixel ||
        width * height * bytes_per_pixel > kMaximumCameraOutputBytes) {
        log_error(config_names::kWidth, "camera output exceeds the 256 MiB limit");
        return false;
    }
    return true;
}

bool SimulationConfigValidator::validate_camera_names(const CameraConfig& camera) {
    if (camera.frame_id.empty()) {
        log_error(config_names::kFrameId, "camera frame_id must not be empty");
        return false;
    }
    if (camera.camera_name.empty()) {
        log_error(config_names::kCameraName, "MuJoCo camera name must not be empty");
        return false;
    }
    if (camera.optical_frame_id.empty()) {
        log_error(config_names::kOpticalFrameId, "camera optical_frame_id must not be empty");
        return false;
    }
    return true;
}

bool SimulationConfigValidator::validate_lidar_names(const LidarInfo& lidar) {
    if (lidar.frame_id.empty()) {
        log_error(config_names::kFrameId, "lidar frame_id must not be empty");
        return false;
    }
    if (lidar.sensor_prefix.empty()) {
        log_error(config_names::kSensorPrefix, "lidar sensor prefix must not be empty");
        return false;
    }
    return true;
}

bool SimulationConfigValidator::validate_mobile_base_names(const MobileBaseCommonInfo& base) {
    if (base.base_body_name.empty() || base.name.empty()) {
        log_error(config_names::kBaseBody, "mobile-base body name must not be empty");
        return false;
    }
    return true;
}

bool SimulationConfigValidator::validate_lidar(const LidarInfo& lidar) {
    if (!std::isfinite(lidar.angle_min) || !std::isfinite(lidar.angle_max) ||
        !std::isfinite(lidar.angle_increment) || !std::isfinite(lidar.range_min) ||
        !std::isfinite(lidar.range_max)) {
        log_error(config_names::kAngleMin, "lidar parameters must be finite");
        return false;
    }
    if (lidar.angle_increment <= 0.0) {
        log_error(config_names::kAngleIncrement, "lidar angle_increment must be positive");
        return false;
    }
    if (!math::less(lidar.angle_min, lidar.angle_max)) {
        log_error(config_names::kAngleMin, "lidar angle_min must be less than angle_max");
        return false;
    }
    if (lidar.range_min < 0.0) {
        log_error(config_names::kRangeMin, "lidar range_min must be non-negative");
        return false;
    }
    if (!math::less(lidar.range_min, lidar.range_max)) {
        log_error(config_names::kRangeMin, "lidar range_min must be less than range_max");
        return false;
    }
    return true;
}

bool SimulationConfigValidator::validate_mobile_base(const MecanumMobileBaseInfo& base) {
    if (base.common.execution_mode != MobileBaseExecutionMode::Kinematic ||
        !std::isfinite(base.wheel_base) || base.wheel_base <= 0.0) {
        log_error(config_names::kWheelBase, "wheel_base must be finite and positive");
        return false;
    }
    if (!std::isfinite(base.track_width) || base.track_width <= 0.0) {
        log_error(config_names::kTrackWidth, "track_width must be finite and positive");
        return false;
    }
    std::unordered_set<std::string> wheel_names;
    for (const MecanumWheelInfo& wheel : base.wheels) {
        if (wheel.joint_name.empty()) {
            log_error(config_names::kWheel, "mobile-base wheel names are required");
            return false;
        }
        if (!std::isfinite(wheel.speed_response) || wheel.speed_response < 0.0) {
            log_error(
                config_names::kSpeedResponse,
                "mobile-base wheel speed_response must be finite and non-negative");
            return false;
        }
        if (!std::isfinite(wheel.radius) || wheel.radius <= 0.0) {
            log_error(
                config_names::kRadius, "mobile-base wheel radius must be finite and positive");
            return false;
        }
        if (wheel.direction != -1.0 && wheel.direction != 1.0) {
            log_error(config_names::kDirection, "mobile-base wheel direction must be -1 or 1");
            return false;
        }
        if (!wheel_names.insert(wheel.joint_name).second) {
            log_error(config_names::kName, "mobile-base wheel names must be unique");
            return false;
        }
    }
    return true;
}

bool SimulationConfigValidator::validate_mobile_base(const SwerveMobileBaseInfo& base) {
    if (base.common.execution_mode != MobileBaseExecutionMode::Dynamic || base.modules.size() < 2U)
        return false;
    std::unordered_set<std::string> names, joints, actuators;
    for (const auto& module : base.modules) {
        if (module.name.empty() || !std::isfinite(module.position_x) ||
            !std::isfinite(module.position_y) || !std::isfinite(module.wheel_radius) ||
            module.wheel_radius <= 0.0 || module.steering_joint_name.empty() ||
            module.drive_joint_name.empty() || module.steering_actuator_name.empty() ||
            module.drive_actuator_name.empty() ||
            module.steering_joint_name == module.drive_joint_name ||
            module.steering_actuator_name == module.drive_actuator_name ||
            !names.insert(module.name).second ||
            !joints.insert(module.steering_joint_name).second ||
            !joints.insert(module.drive_joint_name).second ||
            !actuators.insert(module.steering_actuator_name).second ||
            !actuators.insert(module.drive_actuator_name).second)
            return false;
    }
    return true;
}
bool SimulationConfigValidator::validate_limit(const Limit& limit, const char* name) {
    if (std::isnan(limit.min) || std::isnan(limit.max) || math::greater(limit.min, limit.max)) {
        log_error(name, "limit bounds are invalid");
        return false;
    }
    return true;
}

bool SimulationConfigValidator::validate_gripper(const GripperInfo& gripper) {
    const GripperFingerInfo& first = gripper.fingers[0];
    const GripperFingerInfo& second = gripper.fingers[1];
    if (first.joint_name.empty() || second.joint_name.empty()) {
        log_error(config_names::kJoint, "gripper finger joint names are required");
        return false;
    }
    if (first.joint_name == second.joint_name) {
        log_error(config_names::kJoint, "gripper fingers must use different joints");
        return false;
    }
    if (first.actuator_name.empty() && second.actuator_name.empty()) {
        log_error(config_names::kActuator, "gripper requires at least one finger actuator");
        return false;
    }
    if (!std::isfinite(gripper.control.stiffness) || gripper.control.stiffness < 0.0 ||
        !std::isfinite(gripper.control.damping) || gripper.control.damping < 0.0) {
        log_error(
            config_names::kDamping,
            "gripper stiffness and damping must be finite and non-negative");
        return false;
    }
    if (!validate_limit(gripper.width_limits, config_names::kWidth) ||
        math::less(gripper.width_limits.min, 0.0)) {
        log_error(config_names::kWidth, "gripper width limit must be non-negative");
        return false;
    }
    if (!validate_limit(gripper.velocity_limits, config_names::kVelocity) ||
        !math::equal(gripper.velocity_limits.min, 0.0) ||
        !math::greater(gripper.velocity_limits.max, 0.0)) {
        log_error(
            config_names::kVelocity,
            "gripper velocity limit must start at zero and have a positive maximum");
        return false;
    }
    if (!validate_limit(gripper.effort_limits, config_names::kEffort) ||
        !math::equal(gripper.effort_limits.min, 0.0) ||
        !math::greater(gripper.effort_limits.max, 0.0)) {
        log_error(
            config_names::kEffort,
            "gripper effort limit must start at zero and have a positive maximum");
        return false;
    }
    const GripperStallDetection& stall = gripper.stall;
    if (!std::isfinite(stall.width_tolerance) || math::less(stall.width_tolerance, 0.0) ||
        !std::isfinite(stall.velocity_threshold) || math::less(stall.velocity_threshold, 0.0)) {
        log_error(
            config_names::kWidthTolerance,
            "gripper stall tolerances must be finite and non-negative");
        return false;
    }
    if (!std::isfinite(stall.effort_ratio) || !math::greater(stall.effort_ratio, 0.0) ||
        math::greater(stall.effort_ratio, 1.0)) {
        log_error(config_names::kEffortRatio, "gripper stall effort_ratio must be in (0, 1]");
        return false;
    }
    if (!std::isfinite(stall.timeout) || math::less(stall.timeout, 0.0)) {
        log_error(config_names::kTimeout, "gripper stall timeout must be finite and non-negative");
        return false;
    }
    return true;
}

bool SimulationConfigValidator::claim_resource(
    ResourceOwnership& ownership, const std::string& name, const char* kind, const char* field) {
    if (name.empty()) return true;
    const auto inserted = ownership.emplace(name, kind);
    if (inserted.second) return true;
    log_error(
        field,
        "resource '" + name + "' is already owned by a " + inserted.first->second + " component");
    return false;
}

bool SimulationConfigValidator::claim_component_resources(
    const ComponentConfig& component, ResourceOwnership& joint_owners,
    ResourceOwnership& actuator_owners) {
    return std::visit(
        [&](const auto& info) {
            using Info = std::decay_t<decltype(info)>;
            if constexpr (std::is_same_v<Info, JointInfo>) {
                if (!claim_resource(
                        joint_owners, info.joint_name, config_names::kJointKind,
                        config_names::kJoint))
                    return false;
                if (info.actuation == JointActuation::Passive) return true;
                return claim_resource(
                    actuator_owners, info.actuator_name, config_names::kJointKind,
                    config_names::kActuator);
            } else if constexpr (std::is_same_v<Info, GripperInfo>) {
                for (const GripperFingerInfo& finger : info.fingers)
                    if (!claim_resource(
                            joint_owners, finger.joint_name, config_names::kGripperKind,
                            config_names::kJoint) ||
                        !claim_resource(
                            actuator_owners, finger.actuator_name, config_names::kGripperKind,
                            config_names::kActuator))
                        return false;
                return true;
            } else if constexpr (std::is_same_v<Info, MecanumMobileBaseInfo>) {
                for (const MecanumWheelInfo& wheel : info.wheels)
                    if (!claim_resource(
                            joint_owners, wheel.joint_name, config_names::kMobileBaseKind,
                            config_names::kWheel))
                        return false;
                return true;
            } else if constexpr (std::is_same_v<Info, SwerveMobileBaseInfo>) {
                for (const SwerveModuleInfo& module : info.modules)
                    if (!claim_resource(
                            joint_owners, module.steering_joint_name, config_names::kMobileBaseKind,
                            config_names::kSteeringJoint) ||
                        !claim_resource(
                            joint_owners, module.drive_joint_name, config_names::kMobileBaseKind,
                            config_names::kDriveJoint) ||
                        !claim_resource(
                            actuator_owners, module.steering_actuator_name,
                            config_names::kMobileBaseKind, config_names::kSteeringActuator) ||
                        !claim_resource(
                            actuator_owners, module.drive_actuator_name,
                            config_names::kMobileBaseKind, config_names::kDriveActuator))
                        return false;
                return true;
            } else {
                // Sensors own neither joints nor actuators.
                return true;
            }
        },
        component);
}

bool SimulationConfigValidator::validate_joint(const JointInfo& joint) {
    if (joint.joint_name.empty()) {
        log_error(config_names::kName, "joint and actuator names are required");
        return false;
    }
    if (joint.actuation == JointActuation::Passive) {
        const bool valid =
            joint.actuator_name.empty() && joint.default_mode == JointMode::None &&
            joint.allowed_modes.empty() && joint.hybrid.stiffness == 0.0 &&
            joint.hybrid.damping == 0.0 && joint.position.stiffness == 0.0 &&
            joint.position.damping == 0.0 && joint.velocity.damping == 0.0 &&
            !joint.hybrid.gravity_compensation && !joint.position.gravity_compensation &&
            !joint.velocity.gravity_compensation && !joint.effort.gravity_compensation;
        if (!valid) {
            log_error(config_names::kControl, "passive joint must not configure control");
            return false;
        }
        return validate_limit(joint.position_limits, config_names::kPosition) &&
               validate_limit(joint.velocity_limits, config_names::kVelocity) &&
               validate_limit(joint.effort_limits, config_names::kEffort);
    }
    if (joint.actuator_name.empty()) {
        log_error(config_names::kName, "active joint actuator name is required");
        return false;
    }
    if (joint.allowed_modes.empty() || !joint.allowed_modes.contains(joint.default_mode) ||
        !std::isfinite(joint.hybrid.stiffness) || joint.hybrid.stiffness < 0.0 ||
        !std::isfinite(joint.hybrid.damping) || joint.hybrid.damping < 0.0 ||
        !std::isfinite(joint.position.stiffness) || joint.position.stiffness < 0.0 ||
        !std::isfinite(joint.position.damping) || joint.position.damping < 0.0 ||
        !std::isfinite(joint.velocity.damping) || joint.velocity.damping < 0.0) {
        log_error(
            config_names::kDamping, "joint stiffness and damping must be finite and non-negative");
        return false;
    }
    return validate_limit(joint.position_limits, config_names::kPosition) &&
           validate_limit(joint.velocity_limits, config_names::kVelocity) &&
           validate_limit(joint.effort_limits, config_names::kEffort);
}

bool SimulationConfigValidator::validate_imu(const ImuInfo& imu) {
    if (imu.name.empty() || imu.frame_id.empty() || imu.framequat_sensor_name.empty() ||
        imu.gyro_sensor_name.empty() || imu.accelerometer_sensor_name.empty()) {
        log_error(config_names::kName, "IMU names are required");
        return false;
    }
    if (!values_are_finite(imu.orientation_covariance) ||
        !values_are_finite(imu.angular_velocity_covariance) ||
        !values_are_finite(imu.linear_acceleration_covariance)) {
        log_error(config_names::kCovariance, "IMU covariance must be finite");
        return false;
    }
    return true;
}

bool SimulationConfigValidator::validate_component_identity(
    ComponentId id, const std::string& name, double period, std::unordered_set<ComponentId>& ids,
    std::unordered_set<std::string>& names, const char* kind) {
    if (id == kInvalidComponentId || id > kMaximumComponentId) {
        log_error(config_names::kId, std::string(kind) + " ID is outside the configured range");
        return false;
    }
    if (name.empty()) {
        log_error(config_names::kName, std::string(kind) + " name is required");
        return false;
    }
    if (!std::isfinite(period) || period <= 0.0) {
        log_error(config_names::kPeriod, std::string(kind) + " period must be finite and positive");
        return false;
    }
    if (!ids.insert(id).second || !names.insert(name).second) {
        log_error(config_names::kId, std::string(kind) + " IDs and names must be unique");
        return false;
    }
    return true;
}

bool SimulationConfigValidator::validate(const SimulationConfig& config) {
    if (config.model.model_path.empty()) {
        log_error(config_names::kModelPath, "model path must not be empty");
        return false;
    }
    if (config.viewer_startup_timeout <= std::chrono::milliseconds::zero()) {
        log_error(config_names::kViewerStartupTimeout, "viewer startup timeout must be positive");
        return false;
    }
    if (config.camera_renderer.max_scene_geometries <= 0) {
        log_error(
            config_names::kMaxSceneGeometries,
            "camera renderer scene geometry limit must be positive");
        return false;
    }
    if (!config.camera_renderer.allow_glfw_backend && !config.camera_renderer.allow_egl_backend) {
        log_error(config_names::kCameraRenderer, "camera renderer requires a GLFW or EGL backend");
        return false;
    }
    if (config.camera_renderer.completed_ticket_history == 0U) {
        log_error(
            config_names::kCompletedTicketHistory,
            "camera renderer ticket history must be positive");
        return false;
    }
    if (!std::isfinite(config.scheduler.physics_period) || config.scheduler.physics_period <= 0.0) {
        log_error(config_names::kPhysicsPeriod, "physics_period must be finite and positive");
        return false;
    }
    if (!std::isfinite(config.scheduler.viewer_period) || config.scheduler.viewer_period <= 0.0) {
        log_error(config_names::kViewerPeriod, "viewer_period must be finite and positive");
        return false;
    }
    std::unordered_set<ComponentId> joint_ids, gripper_ids, imu_ids, camera_ids, lidar_ids,
        mobile_base_ids;
    std::unordered_set<std::string> joint_names, gripper_names, imu_names, camera_names,
        lidar_names, mobile_base_names;
    for (const ComponentConfig& component : config.components) {
        const bool valid = std::visit(
            [&](const auto& info) {
                using Info = std::decay_t<decltype(info)>;
                if constexpr (std::is_same_v<Info, JointInfo>)
                    return validate_component_identity(
                               info.id, info.joint_name, info.period, joint_ids, joint_names,
                               config_names::kJointKind) &&
                           validate_joint(info);
                else if constexpr (std::is_same_v<Info, GripperInfo>)
                    return validate_component_identity(
                               info.id, info.name, info.period, gripper_ids, gripper_names,
                               config_names::kGripperKind) &&
                           validate_gripper(info);
                else if constexpr (std::is_same_v<Info, ImuInfo>)
                    return validate_component_identity(
                               info.id, info.name, info.period, imu_ids, imu_names,
                               config_names::kImuKind) &&
                           validate_imu(info);
                else if constexpr (std::is_same_v<Info, CameraConfig>)
                    return validate_camera(info) &&
                           validate_component_identity(
                               info.id, info.name, info.period, camera_ids, camera_names,
                               config_names::kCameraKind) &&
                           validate_camera_names(info);
                else if constexpr (std::is_same_v<Info, LidarInfo>)
                    return validate_component_identity(
                               info.id, info.name, info.period, lidar_ids, lidar_names,
                               config_names::kLidarKind) &&
                           validate_lidar_names(info) && validate_lidar(info);
                else if constexpr (std::is_same_v<Info, MecanumMobileBaseInfo>)
                    return validate_component_identity(
                               info.common.id, info.common.name, info.common.period,
                               mobile_base_ids, mobile_base_names, config_names::kMobileBaseKind) &&
                           validate_mobile_base_names(info.common) && validate_mobile_base(info);
                else
                    return validate_component_identity(
                               info.common.id, info.common.name, info.common.period,
                               mobile_base_ids, mobile_base_names, config_names::kMobileBaseKind) &&
                           validate_mobile_base_names(info.common) && validate_mobile_base(info);
            },
            component);
        if (!valid) return false;
    }
    // Each MuJoCo joint and actuator has exactly one owning component.
    ResourceOwnership joint_owners, actuator_owners;
    for (const ComponentConfig& component : config.components)
        if (!claim_component_resources(component, joint_owners, actuator_owners)) return false;
    return true;
}
}  // namespace romujoco
