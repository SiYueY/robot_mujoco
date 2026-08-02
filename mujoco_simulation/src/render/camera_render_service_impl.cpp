#include "render/camera_render_service_impl.hpp"

#include <utility>

namespace mujoco_simulation {
CameraRenderServiceImpl::CameraRenderServiceImpl(CameraRendererConfig config)
    : renderer_(std::move(config)) {}

bool CameraRenderServiceImpl::initialize(const SimulationConfig &config,
                                         const mjModel *model) {
  (void)config;
  return renderer_.initialize(model);
}

CameraRenderSubmitResult
CameraRenderServiceImpl::submit(const CameraRenderBatchRequest &request,
                                CameraRenderTicket &ticket) {
  if (request.model == nullptr || request.data == nullptr)
    return CameraRenderSubmitResult::InvalidRequest;
  bool replaced_pending_batch = false;
  const auto submitted = renderer_.submit(request, &replaced_pending_batch);
  if (!submitted.has_value())
    return renderer_.is_initialized() ? CameraRenderSubmitResult::Failed
                                      : CameraRenderSubmitResult::Stopped;
  ticket = *submitted;
  return replaced_pending_batch ? CameraRenderSubmitResult::ReplacedPendingBatch
                                : CameraRenderSubmitResult::Accepted;
}

CameraRenderWaitStatus
CameraRenderServiceImpl::wait(const CameraRenderTicket &ticket,
                              std::chrono::milliseconds timeout) {
  return renderer_.wait_result(ticket, timeout);
}

CameraRenderWaitStatus
CameraRenderServiceImpl::query(const CameraRenderTicket &ticket) const {
  return renderer_.query(ticket);
}

bool CameraRenderServiceImpl::read_batch_result(
    const CameraRenderTicket &ticket, CameraRenderBatchResult &result) {
  const CameraRenderWaitStatus status = renderer_.query(ticket, &result);
  return status == CameraRenderWaitStatus::Completed ||
         status == CameraRenderWaitStatus::PartiallyFailed ||
         status == CameraRenderWaitStatus::Failed ||
         status == CameraRenderWaitStatus::Superseded;
}

bool CameraRenderServiceImpl::reset() { return renderer_.release(); }

bool CameraRenderServiceImpl::shutdown() { return renderer_.release(); }

} // namespace mujoco_simulation
