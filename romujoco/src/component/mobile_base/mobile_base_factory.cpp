#include "component/mobile_base/mobile_base_factory.hpp"
#include "component/mobile_base/mecanum/kinematic_mecanum_mobile_base.hpp"
#include "component/mobile_base/swerve/dynamic_swerve_mobile_base.hpp"
namespace romujoco {
std::unique_ptr<MobileBaseComponent> create_mobile_base(const MecanumMobileBaseInfo& i) {
    if (i.common.execution_mode != MobileBaseExecutionMode::Kinematic) return {};
    return std::make_unique<KinematicMecanumMobileBase>(i);
}
std::unique_ptr<MobileBaseComponent> create_mobile_base(const SwerveMobileBaseInfo& i) {
    if (i.common.execution_mode != MobileBaseExecutionMode::Dynamic) return {};
    return std::make_unique<DynamicSwerveMobileBase>(i);
}
}  // namespace romujoco
