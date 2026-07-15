#pragma once

#include <iostream>
#include <string_view>

namespace mujoco_simulation {

inline bool log_component_error(std::string_view where, std::string_view message) {
  std::cerr << "[mujoco_simulation][component] " << where << ": " << message << '\n';
  return false;
}

}  // namespace mujoco_simulation
