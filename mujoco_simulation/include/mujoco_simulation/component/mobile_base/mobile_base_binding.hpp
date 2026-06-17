#pragma once

#include <optional>
#include <string>

#include "mujoco_simulation/component/joint/joint_binding.hpp"

namespace mujoco_simulation {

struct MobileBaseWheelBinding {
  std::string joint_name;
  JointBinding joint{};
};

struct DifferentialBinding {
  MobileBaseWheelBinding left_wheel;
  MobileBaseWheelBinding right_wheel;
};

struct OmnidirectionalBinding {
  MobileBaseWheelBinding front_left;
  MobileBaseWheelBinding front_right;
  MobileBaseWheelBinding rear_left;
  MobileBaseWheelBinding rear_right;
};

struct MobileBaseBinding {
  std::optional<DifferentialBinding> differential;
  std::optional<OmnidirectionalBinding> omnidirectional;
  int base_body_id{-1};
};

}  // namespace mujoco_simulation
