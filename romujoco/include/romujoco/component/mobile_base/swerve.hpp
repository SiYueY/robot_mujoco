#pragma once
#include <string>
#include <vector>
#include "romujoco/component/mobile_base/common.hpp"
namespace romujoco {
struct SwerveModuleInfo {
    std::string name;
    double position_x{0.0};
    double position_y{0.0};
    double wheel_radius{0.0};
    std::string steering_joint_name;
    std::string steering_actuator_name;
    std::string drive_joint_name;
    std::string drive_actuator_name;
};
struct SwerveMobileBaseInfo {
    MobileBaseCommonInfo common;
    std::vector<SwerveModuleInfo> modules;
};
}  // namespace romujoco
