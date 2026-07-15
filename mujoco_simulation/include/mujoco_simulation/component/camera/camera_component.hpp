#pragma once

#include <mujoco/mujoco.h>

#include "mujoco_simulation/buffer/camera_buffer.hpp"
#include "mujoco_simulation/component/camera/camera_data.hpp"
#include "mujoco_simulation/component/camera/camera_renderer.hpp"
#include "mujoco_simulation/component/simulation_component.hpp"

namespace mujoco_simulation {

class CameraComponent : public SimulationComponent {
 public:
  explicit CameraComponent(CameraConfig config);

  std::string name() const noexcept override;
  bool bind(const mjModel& model) override;
  bool reset(const mjModel& model, mjData& data) override;
  bool update(const UpdateContext& context) override;

  const CameraConfig& config() const noexcept { return config_; }

 private:
  CameraConfig config_;
  int camera_id_{-1};
  double fovy_degrees_{0.0};
  std::uint64_t sample_sequence_{0};
};

}  // namespace mujoco_simulation
