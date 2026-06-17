#pragma once

#include <string>

namespace mujoco_simulation {

struct JointCommand {
  std::string name;
  double position{0.0};
  double velocity{0.0};
  double acceleration{0.0};
  double effort{0.0};
};

}  // namespace mujoco_simulation
