#pragma once

#include <cstddef>

#include "mujoco_simulation/component/component_id.hpp"

namespace mujoco_simulation {

// Configuration limits are deliberately kept independent from the XML parser.
// They apply equally to configurations assembled in C++ and to standalone
// subsystems such as CameraRenderer.
struct SimulationConfigLimits {
  static constexpr ComponentId kMaximumComponentId{65535};
  static constexpr int kMaximumCameraDimension{8192};
  static constexpr std::size_t kMaximumCameraOutputBytes{256U * 1024U * 1024U};
};

} // namespace mujoco_simulation
