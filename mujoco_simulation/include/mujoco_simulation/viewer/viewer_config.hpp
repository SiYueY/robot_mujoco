#pragma once

#include <chrono>

namespace mujoco_simulation {

struct ViewerConfig {
  std::chrono::milliseconds startup_timeout{5000};
};

}  // namespace mujoco_simulation
