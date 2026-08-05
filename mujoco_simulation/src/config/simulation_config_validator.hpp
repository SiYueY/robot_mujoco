#pragma once

#include <string>
#include <unordered_set>

#include "component/component.hpp"
#include "mujoco_simulation/config/simulation_config.hpp"

namespace mujoco_simulation {

class SimulationConfigValidator {
public:
    static bool validate(const SimulationConfig& config);

private:
    static bool validate_camera(const CameraConfig& camera);
    static bool validate_camera_names(const CameraConfig& camera);
    static bool validate_lidar(const LidarInfo& lidar);
    static bool validate_lidar_names(const LidarInfo& lidar);
    static bool validate_mobile_base(const MobileBaseInfo& base);
    static bool validate_mobile_base_names(const MobileBaseInfo& base);
    static bool validate_joint(const JointInfo& joint);
    static bool validate_imu(const ImuInfo& imu);
    static bool validate_limit(const JointLimit& limit, const char* name);
    static bool validate_component_identity(
        ComponentId id, const std::string& name, double period,
        std::unordered_set<ComponentId>& ids, std::unordered_set<std::string>& names,
        const char* kind);
    static void log_error(const std::string& field, const std::string& message);
};

}  // namespace mujoco_simulation
