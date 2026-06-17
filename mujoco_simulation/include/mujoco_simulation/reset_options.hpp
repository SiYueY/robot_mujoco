#pragma once

#include <optional>
#include <string>

namespace mujoco_simulation {

struct ResetOptions {
  std::optional<std::string> keyframe_name;
  std::optional<int> keyframe_id;
  bool clear_commands{true};
  bool reset_components{true};
  bool clear_state_buffer{true};
  bool clear_camera_buffer{true};
  bool reset_statistics{false};
};

}  // namespace mujoco_simulation
