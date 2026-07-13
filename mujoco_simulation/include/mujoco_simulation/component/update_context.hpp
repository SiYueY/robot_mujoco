#pragma once

#include <mujoco/mujoco.h>

#include <cstdint>

namespace mujoco_simulation {

class CameraBuffer;
class CameraRenderer;

struct UpdateContext {
  const mjModel& model;
  const mjData& data;
  double simulation_time;
  std::uint64_t step_count;
  CameraRenderer* camera_renderer;
  CameraBuffer* camera_buffer;
};

}  // namespace mujoco_simulation
