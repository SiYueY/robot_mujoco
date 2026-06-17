#pragma once

#include <cstdint>

#include "mujoco_simulation/component/simulation_component.hpp"

namespace mujoco_simulation {

class CameraBuffer;
class CameraRenderer;

struct SensorSampleContext {
  const mjModel& model;
  const mjData& data;
  double simulation_time;
  std::uint64_t step_count;
  CameraRenderer* camera_renderer;
  CameraBuffer* camera_buffer;
};

class SensorComponent : public SimulationComponent {
 public:
  ~SensorComponent() override = default;

  virtual double update_rate() const noexcept = 0;
  virtual Status sample(const SensorSampleContext& context) = 0;
};

}  // namespace mujoco_simulation
