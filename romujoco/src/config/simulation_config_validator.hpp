#pragma once

#include <string>
#include <unordered_map>
#include <unordered_set>

#include "component/component.hpp"
#include "romujoco/config/simulation_config.hpp"

namespace romujoco {

class SimulationConfigValidator {
public:
    static bool validate(const SimulationConfig& config);

private:
    // Maps a MuJoCo resource name to the kind of component that owns it.
    using ResourceOwnership = std::unordered_map<std::string, std::string>;

    static bool validate_camera(const CameraConfig& camera);
    static bool validate_camera_names(const CameraConfig& camera);
    static bool validate_gripper(const GripperInfo& gripper);
    static bool validate_lidar(const LidarInfo& lidar);
    static bool validate_lidar_names(const LidarInfo& lidar);
    static bool validate_mobile_base(const MecanumMobileBaseInfo& base);
    static bool validate_mobile_base(const SwerveMobileBaseInfo& base);
    static bool validate_mobile_base_names(const MobileBaseCommonInfo& base);
    static bool validate_joint(const JointInfo& joint);
    static bool validate_imu(const ImuInfo& imu);
    static bool validate_limit(const Limit& limit, const char* name);
    static bool validate_component_identity(
        ComponentId id, const std::string& name, double period,
        std::unordered_set<ComponentId>& ids, std::unordered_set<std::string>& names,
        const char* kind);
    static bool claim_resource(
        ResourceOwnership& ownership, const std::string& name, const char* kind, const char* field);
    static bool claim_component_resources(
        const ComponentConfig& component, ResourceOwnership& joint_owners,
        ResourceOwnership& actuator_owners);
    static void log_error(const std::string& field, const std::string& message);
};

}  // namespace romujoco
