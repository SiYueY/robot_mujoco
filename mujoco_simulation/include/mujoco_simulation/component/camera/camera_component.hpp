#pragma once

#include <mujoco/mujoco.h>

#include "mujoco_simulation/buffer/camera_buffer.hpp"
#include "mujoco_simulation/component/camera/camera_binding.hpp"
#include "mujoco_simulation/component/camera/camera_config.hpp"
#include "mujoco_simulation/component/camera/camera_renderer.hpp"
#include "mujoco_simulation/component/camera/camera_sample.hpp"
#include "mujoco_simulation/component/sensor_component.hpp"

namespace mujoco_simulation {

class CameraComponent : public SensorComponent {
 public:
  explicit CameraComponent(CameraConfig config);

  std::string_view name() const noexcept override;
  Status bind(const mjModel& model) override;
  Status reset(const mjModel& model, mjData& data) override;
  double update_rate() const noexcept override;
  Status sample(const SensorSampleContext& context) override;

  const CameraConfig& config() const noexcept { return config_; }
  const CameraBinding& binding() const noexcept { return binding_; }

 private:
  CameraConfig config_;
  CameraBinding binding_{};
  std::uint64_t sample_sequence_{0};
};

}  // namespace mujoco_simulation
