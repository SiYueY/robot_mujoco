#include "config/simulation_config_validator.hpp"

#include <cmath>
#include <limits>
#include <type_traits>
#include <unordered_set>

#include "common/compare.hpp"
#include "log/logging.hpp"
#include "config/simulation_config_data.hpp"

namespace mujoco_simulation {
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

bool SimulationConfigValidator::validate_mobile_base_names(const MobileBaseInfo& base) {
    if (base.base_body_name.empty()) {
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

bool SimulationConfigValidator::validate_mobile_base(const MobileBaseInfo& base) {
    const MecanumInfo& mecanum = base.mecanum_info;
    if (!std::isfinite(mecanum.wheel_radius) || mecanum.wheel_radius <= 0.0) {
        log_error(config_names::kWheelRadius, "wheel_radius must be finite and positive");
        return false;
    }
    if (!std::isfinite(mecanum.wheel_base) || mecanum.wheel_base <= 0.0) {
        log_error(config_names::kWheelBase, "wheel_base must be finite and positive");
        return false;
    }
    if (!std::isfinite(mecanum.track_width) || mecanum.track_width <= 0.0) {
        log_error(config_names::kTrackWidth, "track_width must be finite and positive");
        return false;
    }
    std::unordered_set<std::string> wheel_names, actuator_names;
    for (const WheelInfo& wheel : base.mecanum_wheels) {
        if (wheel.wheel_name.empty() || wheel.actuator_name.empty()) {
            log_error(config_names::kWheel, "mobile-base wheel names are required");
            return false;
        }
        if (!std::isfinite(wheel.damping) || wheel.damping < 0.0) {
            log_error(
                config_names::kDamping,
                "mobile-base wheel damping must be finite and non-negative");
            return false;
        }
        if (!wheel_names.insert(wheel.wheel_name).second) {
            log_error(config_names::kName, "mobile-base wheel names must be unique");
            return false;
        }
        if (!actuator_names.insert(wheel.actuator_name).second) {
            log_error(config_names::kActuator, "mobile-base actuator names must be unique");
            return false;
        }
    }
    return true;
}

bool SimulationConfigValidator::validate_limit(const JointLimit& limit, const char* name) {
    if (std::isnan(limit.min) || std::isnan(limit.max) || math::greater(limit.min, limit.max)) {
        log_error(name, "joint limit bounds are invalid");
        return false;
    }
    return true;
}

bool SimulationConfigValidator::validate_joint(const JointInfo& joint) {
    if (joint.joint_name.empty() || joint.actuator_name.empty()) {
        log_error(config_names::kName, "joint and actuator names are required");
        return false;
    }
    if (!std::isfinite(joint.position_stiffness) || joint.position_stiffness < 0.0 ||
        !std::isfinite(joint.position_damping) || joint.position_damping < 0.0 ||
        !std::isfinite(joint.velocity_damping) || joint.velocity_damping < 0.0) {
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
    std::unordered_set<ComponentId> joint_ids, imu_ids, camera_ids, lidar_ids, mobile_base_ids;
    std::unordered_set<std::string> joint_names, imu_names, camera_names, lidar_names,
        mobile_base_names;
    for (const ComponentConfig& component : config.components) {
        const bool valid = std::visit(
            [&](const auto& info) {
                using Info = std::decay_t<decltype(info)>;
                if constexpr (std::is_same_v<Info, JointInfo>)
                    return validate_component_identity(
                               info.id, info.joint_name, info.period, joint_ids, joint_names,
                               config_names::kJointKind) &&
                           validate_joint(info);
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
                else
                    return validate_component_identity(
                               info.id, info.mobile_base_name, info.period, mobile_base_ids,
                               mobile_base_names, config_names::kMobileBaseKind) &&
                           validate_mobile_base_names(info) && validate_mobile_base(info);
            },
            component);
        if (!valid) return false;
    }
    return true;
}
}  // namespace mujoco_simulation
