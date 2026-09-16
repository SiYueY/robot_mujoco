#pragma once
#include <memory>
#include "romujoco/component/mobile_base/mecanum.hpp"
#include "romujoco/component/mobile_base/swerve.hpp"
#include "component/mobile_base/mobile_base_component.hpp"
namespace romujoco {
std::unique_ptr<MobileBaseComponent> create_mobile_base(const MecanumMobileBaseInfo&);
std::unique_ptr<MobileBaseComponent> create_mobile_base(const SwerveMobileBaseInfo&);
}  // namespace romujoco
