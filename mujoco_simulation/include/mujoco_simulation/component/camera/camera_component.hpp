#pragma once

#include <mujoco/mujoco.h>

#include "mujoco_simulation/buffer/camera_buffer.hpp"
#include "mujoco_simulation/component/camera/camera_data.hpp"
#include "mujoco_simulation/component/camera/camera_renderer.hpp"
#include "mujoco_simulation/component/simulation_component.hpp"

namespace mujoco_simulation {

struct CameraBinding {
  int camera_id{-1};
  double fovy_degrees{0.0};
};

class CameraComponent : public SimulationComponent {
 public:
  explicit CameraComponent(CameraConfig config);

  std::string name() const noexcept override;
  ResultCode bind(const mjModel& model) override;
  ResultCode reset(const mjModel& model, mjData& data) override;
  ResultCode update(const UpdateContext& context) override;

  const CameraConfig& config() const noexcept { return config_; }
  const CameraBinding& binding() const noexcept { return binding_; }

 private:
  CameraConfig config_;
  CameraBinding binding_{};
  std::uint64_t sample_sequence_{0};
};

}  // namespace mujoco_simulation
