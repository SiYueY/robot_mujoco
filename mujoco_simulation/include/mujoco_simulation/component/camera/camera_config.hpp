#pragma once

#include <string>

#include "mujoco_simulation/component/sensor_common_config.hpp"

namespace mujoco_simulation {

struct CameraConfig {
  SensorCommonConfig common{.update_rate = 30.0};
  std::string camera_name;
  std::string optical_frame_id;

  int height{0};
  int width{0};

  bool enable_rgb{true};
  bool enable_depth{false};
};

}  // namespace mujoco_simulation
