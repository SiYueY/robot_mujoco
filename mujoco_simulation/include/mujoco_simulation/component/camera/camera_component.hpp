#pragma once

#include <mujoco/mujoco.h>

#include <memory>

#include "mujoco_simulation/component/camera/camera_data.hpp"
#include "mujoco_simulation/component/simulation_component.hpp"
#include "mujoco_simulation/mujoco/camera_renderer.hpp"

namespace mujoco_simulation {

class CameraComponent : public SimulationComponent {
 public:
  explicit CameraComponent(CameraConfig config);

  bool init(const mjContext& context) override;
  bool reset(const mjContext& context) override;
  bool update(const mjContext& context) override;
  bool configure_rendering(CameraRenderer& renderer);
  bool prepare_rendering(const mjContext& context);
  bool read_state(std::shared_ptr<const CameraState>& state) const;

  const CameraConfig& config() const noexcept { return config_; }

 public:
  using SharedPtr = std::shared_ptr<CameraComponent>;
  using UniquePtr = std::unique_ptr<CameraComponent>;
  using WeakPtr = std::weak_ptr<CameraComponent>;

 private:
  static CameraRenderIntrinsics compute_intrinsics(double fovy_degrees, std::uint32_t width,
                                                   std::uint32_t height);

  CameraConfig config_;
  int camera_id_{-1};
  double fovy_degrees_{0.0};
  std::uint64_t sample_sequence_{0};
  CameraRenderer* camera_renderer_{nullptr};
  std::shared_ptr<const CameraState> state_;
};

}  // namespace mujoco_simulation
