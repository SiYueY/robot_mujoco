#pragma once
// Internal OpenGL-backed CameraRenderService implementation.

#include "component/camera/camera_render_service.hpp"
#include "render/camera_renderer.hpp"

namespace mujoco_simulation {

class CameraRenderServiceImpl final : public CameraRenderService {
public:
  explicit CameraRenderServiceImpl(CameraRendererConfig config);

  bool initialize(const SimulationConfig &config,
                  const mjModel *model) override;
  CameraRenderSubmitResult submit(const CameraRenderBatchRequest &request,
                                  CameraRenderTicket &ticket) override;
  CameraRenderWaitStatus wait(const CameraRenderTicket &ticket,
                              std::chrono::milliseconds timeout) override;
  CameraRenderWaitStatus query(const CameraRenderTicket &ticket) const override;
  bool read_batch_result(const CameraRenderTicket &ticket,
                         CameraRenderBatchResult &result) override;
  bool reset() override;
  bool shutdown() override;

private:
  CameraRenderer renderer_;
};

} // namespace mujoco_simulation
