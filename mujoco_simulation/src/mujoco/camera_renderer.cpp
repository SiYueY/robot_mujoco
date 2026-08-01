#include "mujoco_simulation/mujoco/camera_renderer.hpp"

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLFW/glfw3.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <exception>
#include <limits>
#include <unordered_set>
#include <utility>

#include "common/logging.hpp"
#include "mujoco_simulation/common/macro.hpp"

namespace mujoco_simulation {
namespace {

constexpr int kHiddenContextWidth = 1;
constexpr int kHiddenContextHeight = 1;
constexpr int kColorChannelCount = 3;
constexpr int kEglColorBits = 8;
constexpr int kEglDepthBits = 24;
constexpr int kFontScale = mjFONTSCALE_150;
constexpr double kPi = 3.14159265358979323846;

template <typename Callback> class ScopeExit {
public:
  explicit ScopeExit(Callback callback) : callback_(std::move(callback)) {}
  ~ScopeExit() { callback_(); }

  ScopeExit(const ScopeExit &) = delete;
  ScopeExit &operator=(const ScopeExit &) = delete;

private:
  Callback callback_;
};

template <typename Callback> ScopeExit(Callback) -> ScopeExit<Callback>;

bool pixel_count_for(int width, int height, std::size_t &pixel_count) {
  if (width <= 0 || height <= 0) {
    return false;
  }
  const auto unsigned_width = static_cast<std::size_t>(width);
  const auto unsigned_height = static_cast<std::size_t>(height);
  if (unsigned_width >
      std::numeric_limits<std::size_t>::max() / unsigned_height) {
    return false;
  }
  pixel_count = unsigned_width * unsigned_height;
  return true;
}

} // namespace

CameraRenderer::CameraRenderer() : CameraRenderer(CameraRendererConfig{}) {}

CameraRenderer::CameraRenderer(CameraRendererConfig config) : config_(config) {}

CameraRenderer::~CameraRenderer() { UNUSED(release()); }

bool CameraRenderer::initialize(const mjContext &context) {
  std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
  if (initialized_.load() && running_.load()) {
    return true;
  }

  // A worker that failed after a successful initialize() has already released
  // its thread-local OpenGL resources, but its thread object and mjData
  // buffers still belong to this renderer. Reap that failed lifecycle before
  // allocating a replacement one; otherwise initialize() would overwrite the
  // stale pointers and report a healthy renderer without a worker.
  if (worker_thread_.joinable() || pending_data_ != nullptr ||
      render_data_ != nullptr || model_ != nullptr) {
    if (!release_locked()) {
      return false;
    }
  }
  if (!context.valid()) {
    LOG_ERROR << "camera renderer requires a valid MuJoCo context.";
    return false;
  }
  if (config_.max_scene_geometries <= 0) {
    LOG_ERROR << "max_scene_geometries must be positive.";
    return false;
  }
  if (!config_.allow_glfw_backend && !config_.allow_egl_backend) {
    LOG_ERROR << "camera renderer requires a GLFW or EGL backend.";
    return false;
  }
  if (config_.completed_ticket_history == 0U) {
    LOG_ERROR << "camera renderer ticket history must be positive.";
    return false;
  }

  model_ = context.model;
  pending_data_ = mj_makeData(model_);
  render_data_ = mj_makeData(model_);
  if (pending_data_ == nullptr || render_data_ == nullptr) {
    LOG_ERROR << "failed to allocate camera render data buffers.";
    UNUSED(release_locked());
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(job_mutex_);
    pending_tasks_.clear();
    pending_ready_ = false;
    pending_ticket_ = 0;
    submitted_ticket_ = 0;
    expired_through_ticket_ = {};
    completed_results_.clear();
    worker_ready_ = false;
    worker_initialization_failed_ = false;
    stopping_ = false;
  }
  {
    std::lock_guard<std::mutex> lock(result_mutex_);
    latest_results_.reset();
  }

  try {
    worker_thread_ = std::thread(&CameraRenderer::worker_loop, this);
  } catch (const std::exception &) {
    LOG_ERROR << "failed to start the camera render worker.";
    UNUSED(release_locked());
    return false;
  }

  std::unique_lock<std::mutex> lock(job_mutex_);
  const bool worker_reported =
      completion_condition_.wait_for(lock, kWorkerTimeout, [this] {
        return worker_ready_ || worker_initialization_failed_;
      });
  if (!worker_reported || worker_initialization_failed_) {
    LOG_ERROR << "camera render worker initialization timed out or failed.";
    lock.unlock();
    UNUSED(release_locked());
    return false;
  }
  const std::uint64_t prior_generation = ticket_epoch_.load();
  if (prior_generation == std::numeric_limits<std::uint64_t>::max()) {
    LOG_ERROR << "camera renderer ticket epoch overflowed.";
    lock.unlock();
    UNUSED(release_locked());
    return false;
  }
  ticket_epoch_.store(prior_generation + 1U);
  initialized_.store(true);
  return true;
}

std::optional<CameraRenderTicket>
CameraRenderer::submit(const mjContext &context,
                       std::vector<CameraRenderTask> tasks) {
  if (tasks.empty()) {
    return CameraRenderTicket{};
  }
  std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
  if (!context.valid()) {
    LOG_ERROR << "camera renderer requires a valid MuJoCo context.";
    return std::nullopt;
  }
  if (!initialized_.load() || !running_.load()) {
    LOG_ERROR << "camera renderer is not initialized.";
    return std::nullopt;
  }
  std::unordered_set<CameraId> ids;
  for (const CameraRenderTask &task : tasks) {
    if (task.config.id == kInvalidComponentId ||
        task.config.id > config_.max_camera_id ||
        !ids.insert(task.config.id).second) {
      LOG_ERROR
          << "camera render task id is invalid, outside range, or duplicated.";
      return std::nullopt;
    }
  }
  CameraRenderTicket ticket;
  {
    std::lock_guard<std::mutex> lock(job_mutex_);
    if (stopping_ || pending_data_ == nullptr) {
      LOG_WARNING << "camera renderer is stopping; dropped render request.";
      return std::nullopt;
    }
    if (mj_copyData(pending_data_, context.model, context.data) == nullptr) {
      LOG_ERROR
          << "failed to copy MuJoCo simulation data for camera rendering.";
      return std::nullopt;
    }
    if (pending_ready_) {
      CameraBatchResult superseded;
      superseded.ticket = {ticket_epoch_.load(), pending_ticket_};
      for (const CameraRenderTask &pending : pending_tasks_)
        superseded.statuses.push_back({pending.config.id,
                                       CameraRenderTaskResult::Superseded,
                                       "render request was superseded"});
      completed_results_[{superseded.ticket.generation,
                          superseded.ticket.sequence}] = std::move(superseded);
      while (completed_results_.size() > config_.completed_ticket_history) {
        const TicketKey expired_key = completed_results_.begin()->first;
        expired_through_ticket_ = {expired_key.first, expired_key.second};
        completed_results_.erase(completed_results_.begin());
      }
    }
    pending_tasks_ = std::move(tasks);
    pending_ticket_ = ++submitted_ticket_;
    ticket = {ticket_epoch_.load(), pending_ticket_};
    pending_ready_ = true;
  }
  completion_condition_.notify_all();
  job_condition_.notify_one();
  return ticket;
}

bool CameraRenderer::read_results(CameraRenderStates &states) const {
  std::lock_guard<std::mutex> lock(result_mutex_);
  states = latest_results_;
  return states != nullptr;
}

CameraWaitResult CameraRenderer::wait_result(CameraRenderTicket requested,
                                             CameraBatchResult *result) {
  std::unique_lock<std::mutex> lock(job_mutex_);
  if (requested.is_noop()) {
    return CameraWaitResult::Succeeded;
  }
  const std::uint64_t requested_generation = requested.generation;
  const TicketKey requested_key{requested.generation, requested.sequence};
  if (requested_generation != ticket_epoch_.load()) {
    LOG_ERROR << "camera render ticket belongs to another renderer lifecycle.";
    return CameraWaitResult::InvalidTicket;
  }
  if (requested.sequence > submitted_ticket_) {
    LOG_ERROR << "camera render ticket was not submitted by this renderer.";
    return CameraWaitResult::InvalidTicket;
  }
  if (requested.generation == expired_through_ticket_.generation &&
      requested.sequence <= expired_through_ticket_.sequence) {
    LOG_ERROR << "camera render ticket has expired.";
    return CameraWaitResult::Expired;
  }
  const bool completed = completion_condition_.wait_for(
      lock, kWorkerTimeout, [this, requested_generation, requested_key] {
        return ticket_epoch_.load() != requested_generation || stopping_ ||
               completed_results_.find(requested_key) !=
                   completed_results_.end() ||
               worker_initialization_failed_;
      });
  if (!completed) {
    LOG_ERROR << "camera render worker timed out for the submitted frame.";
    return CameraWaitResult::Timeout;
  }
  if (ticket_epoch_.load() != requested_generation) {
    LOG_ERROR << "camera render ticket belongs to another renderer lifecycle.";
    return CameraWaitResult::InvalidTicket;
  }
  if (stopping_ || worker_initialization_failed_) {
    LOG_ERROR << "camera render worker stopped before completing the frame.";
    return CameraWaitResult::RendererStopped;
  }
  const auto completed_batch = completed_results_.find(requested_key);
  if (completed_batch == completed_results_.end()) {
    LOG_ERROR << "camera render worker did not complete the submitted frame.";
    return CameraWaitResult::RendererStopped;
  }
  const CameraBatchResult &batch = completed_batch->second;
  if (result != nullptr)
    *result = batch;
  if (!batch.all_succeeded) {
    LOG_ERROR << "camera render worker failed to render the submitted frame.";
    const bool superseded =
        !batch.statuses.empty() &&
        std::all_of(batch.statuses.begin(), batch.statuses.end(),
                    [](const CameraRenderTaskStatus &status) {
                      return status.result ==
                             CameraRenderTaskResult::Superseded;
                    });
    return superseded ? CameraWaitResult::Superseded
                      : CameraWaitResult::RenderFailed;
  }
  return CameraWaitResult::Succeeded;
}

bool CameraRenderer::wait(CameraRenderTicket ticket,
                          CameraBatchResult *result) {
  return wait_result(ticket, result) == CameraWaitResult::Succeeded;
}

bool CameraRenderer::release() {
  std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
  return release_locked();
}

bool CameraRenderer::release_locked() {
  if (initialized_.load()) {
    const std::uint64_t prior_generation = ticket_epoch_.load();
    if (prior_generation != std::numeric_limits<std::uint64_t>::max()) {
      // Invalidate tickets before waking waiters. The next initialize()
      // advances the epoch again, so an old waiter cannot observe its
      // sequence number in a restarted lifecycle.
      ticket_epoch_.store(prior_generation + 1U);
    }
  }
  {
    std::lock_guard<std::mutex> lock(job_mutex_);
    stopping_ = true;
    pending_ready_ = false;
    pending_tasks_.clear();
  }
  job_condition_.notify_all();
  completion_condition_.notify_all();

  if (worker_thread_.joinable()) {
    if (worker_thread_.get_id() == std::this_thread::get_id()) {
      LOG_ERROR << "camera render worker cannot join itself.";
      return false;
    }
    worker_thread_.join();
  }

  if (pending_data_ != nullptr) {
    mj_deleteData(pending_data_);
    pending_data_ = nullptr;
  }
  if (render_data_ != nullptr) {
    mj_deleteData(render_data_);
    render_data_ = nullptr;
  }
  {
    std::lock_guard<std::mutex> lock(result_mutex_);
    latest_results_.reset();
  }
  {
    std::lock_guard<std::mutex> lock(job_mutex_);
    pending_tasks_.clear();
    pending_ready_ = false;
    pending_ticket_ = 0;
    submitted_ticket_ = 0;
    expired_through_ticket_ = {};
    completed_results_.clear();
    worker_ready_ = false;
    worker_initialization_failed_ = false;
    stopping_ = false;
  }
  model_ = nullptr;
  initialized_.store(false);
  running_.store(false);
  return true;
}

void CameraRenderer::worker_loop() {
  bool resources_ready = false;
  try {
    resources_ready = initialize_worker_resources();
  } catch (const std::exception &) {
    LOG_ERROR << "camera render worker threw during initialization.";
  } catch (...) {
    LOG_ERROR << "camera render worker threw an unknown initialization error.";
  }
  running_.store(resources_ready);
  {
    std::lock_guard<std::mutex> lock(job_mutex_);
    worker_ready_ = resources_ready;
    worker_initialization_failed_ = !resources_ready;
  }
  completion_condition_.notify_all();
  if (!resources_ready) {
    return;
  }

  try {
    while (true) {
      std::vector<CameraRenderTask> tasks;
      std::uint64_t ticket = 0;
      {
        std::unique_lock<std::mutex> lock(job_mutex_);
        job_condition_.wait(lock,
                            [this] { return stopping_ || pending_ready_; });
        if (stopping_) {
          break;
        }
        std::swap(render_data_, pending_data_);
        tasks = std::move(pending_tasks_);
        pending_tasks_.clear();
        ticket = pending_ticket_;
        pending_ready_ = false;
      }

      auto updates = std::make_shared<std::vector<CameraRenderStatePtr>>();
      CameraBatchResult batch;
      batch.ticket = {ticket_epoch_.load(), ticket};
      bool succeeded = activate_context();
      if (!succeeded) {
        LOG_ERROR << "failed to activate the camera render context.";
      }
      if (succeeded) {
        const ScopeExit deactivate([this] { deactivate_context(); });
        for (const CameraRenderTask &task : tasks) {
          CameraRenderStatePtr state;
          std::string error;
          if (!render_task(task, state, &error)) {
            succeeded = false;
            batch.statuses.push_back({task.config.id,
                                      CameraRenderTaskResult::RenderFailed,
                                      std::move(error)});
            LOG_WARNING
                << "camera render request failed; keeping the prior frame.";
            continue;
          }
          if (updates->size() <= task.config.id) {
            updates->resize(task.config.id + 1U);
          }
          (*updates)[task.config.id] = std::move(state);
          batch.statuses.push_back(
              {task.config.id, CameraRenderTaskResult::Succeeded, ""});
        }
      } else {
        for (const CameraRenderTask &task : tasks)
          batch.statuses.push_back({task.config.id,
                                    CameraRenderTaskResult::RenderFailed,
                                    "failed to activate render context"});
      }

      if (!updates->empty()) {
        std::lock_guard<std::mutex> lock(result_mutex_);
        auto next = std::make_shared<std::vector<CameraRenderStatePtr>>(
            latest_results_ == nullptr ? 0U : latest_results_->size());
        if (latest_results_ != nullptr) {
          *next = *latest_results_;
        }
        if (next->size() < updates->size()) {
          next->resize(updates->size());
        }
        for (CameraId id = 0; id < updates->size(); ++id) {
          if ((*updates)[id] != nullptr) {
            (*next)[id] = std::move((*updates)[id]);
          }
        }
        latest_results_ =
            std::static_pointer_cast<const std::vector<CameraRenderStatePtr>>(
                next);
      }
      {
        std::lock_guard<std::mutex> lock(job_mutex_);
        batch.all_succeeded =
            succeeded &&
            std::all_of(batch.statuses.begin(), batch.statuses.end(),
                        [](const CameraRenderTaskStatus &status) {
                          return status.result ==
                                 CameraRenderTaskResult::Succeeded;
                        });
        const TicketKey completed_key{batch.ticket.generation,
                                      batch.ticket.sequence};
        completed_results_[completed_key] = std::move(batch);
        while (completed_results_.size() > config_.completed_ticket_history) {
          const TicketKey expired_key = completed_results_.begin()->first;
          expired_through_ticket_ = {expired_key.first, expired_key.second};
          completed_results_.erase(completed_results_.begin());
        }
      }
      completion_condition_.notify_all();
    }
  } catch (const std::exception &) {
    LOG_ERROR << "camera render worker threw while rendering.";
    std::lock_guard<std::mutex> lock(job_mutex_);
    worker_initialization_failed_ = true;
    stopping_ = true;
  } catch (...) {
    LOG_ERROR << "camera render worker threw an unknown rendering error.";
    std::lock_guard<std::mutex> lock(job_mutex_);
    worker_initialization_failed_ = true;
    stopping_ = true;
  }

  release_worker_resources();
  // A stopped worker no longer represents an initialized lifecycle. Mark it
  // inactive only after its worker-owned resources are released, so a later
  // initialize() can synchronously reap and replace it.
  initialized_.store(false);
  running_.store(false);
  completion_condition_.notify_all();
}

bool CameraRenderer::initialize_worker_resources() {
  if (!create_context()) {
    return false;
  }
  if (!activate_context()) {
    destroy_context();
    return false;
  }
  const ScopeExit deactivate([this] { deactivate_context(); });
  if (!create_render_resources()) {
    destroy_render_resources();
    destroy_context();
    return false;
  }
  return true;
}

void CameraRenderer::release_worker_resources() {
  const bool context_active =
      gl_context_.backend != OffscreenGlBackend::None && activate_context();
  if (!context_active && gl_context_.backend != OffscreenGlBackend::None) {
    LOG_ERROR << "failed to activate offscreen context during worker release.";
  }
  destroy_render_resources();
  if (context_active) {
    deactivate_context();
  }
  destroy_context();
  offscreen_width_ = 0;
  offscreen_height_ = 0;
}

bool CameraRenderer::render_task(const CameraRenderTask &task,
                                 CameraRenderStatePtr &out,
                                 std::string *error) {
  const auto fail = [error](const char *message) {
    if (error != nullptr)
      *error = message;
    return false;
  };
  const CameraConfig &spec = task.config;
  if (model_ == nullptr || render_data_ == nullptr) {
    LOG_ERROR << "camera render resources are not initialized.";
    return fail("camera render resources are not initialized");
  }
  if (spec.name.empty() || spec.camera_name.empty()) {
    LOG_ERROR << "camera component and MuJoCo camera names must not be empty.";
    return fail("camera component and MuJoCo camera names must not be empty");
  }
  if (spec.width <= 0 || spec.height <= 0) {
    LOG_ERROR << "camera width and height must be positive.";
    return fail("camera width and height must be positive");
  }
  if (!spec.enable_rgb && !spec.enable_depth) {
    LOG_ERROR << "camera must enable rgb or depth output.";
    return fail("camera must enable RGB or depth output");
  }
  const int camera_id =
      mj_name2id(model_, mjOBJ_CAMERA, spec.camera_name.c_str());
  if (camera_id < 0) {
    LOG_ERROR << "camera was not found in model.";
    return fail("camera was not found in the MuJoCo model");
  }
  const double fovy_degrees = static_cast<double>(model_->cam_fovy[camera_id]);
  if (!std::isfinite(fovy_degrees) || fovy_degrees <= 0.0 ||
      fovy_degrees >= 180.0) {
    LOG_ERROR << "camera field of view must be in the range (0, 180) degrees.";
    return fail("camera field of view must be in (0, 180) degrees");
  }
  if (!resize_offscreen_buffer(spec.width, spec.height)) {
    return fail("failed to resize the offscreen render buffer");
  }

  std::size_t pixel_count = 0;
  if (!pixel_count_for(spec.width, spec.height, pixel_count) ||
      pixel_count >
          std::numeric_limits<std::size_t>::max() / kColorChannelCount) {
    LOG_ERROR << "camera image dimensions exceed the supported pixel capacity.";
    return fail("camera image dimensions exceed supported pixel capacity");
  }

  mjr_setBuffer(mjFB_OFFSCREEN, &render_context_);
  mjvCamera camera{};
  mjv_defaultCamera(&camera);
  camera.type = mjCAMERA_FIXED;
  camera.fixedcamid = camera_id;
  const mjrRect viewport{0, 0, spec.width, spec.height};
  mjv_updateScene(model_, render_data_, &option_, nullptr, &camera, mjCAT_ALL,
                  &scene_);
  mjr_render(viewport, &scene_, &render_context_);

  auto state = std::make_shared<CameraRenderState>();
  state->sequence = task.sequence;
  state->timestamp = task.timestamp;
  state->frame_id = spec.frame_id.empty() ? spec.name : spec.frame_id;
  state->optical_frame_id =
      spec.optical_frame_id.empty() ? state->frame_id : spec.optical_frame_id;
  state->intrinsics =
      compute_intrinsics(fovy_degrees, static_cast<std::uint32_t>(spec.width),
                         static_cast<std::uint32_t>(spec.height));

  std::vector<std::uint8_t> rgb_buffer;
  std::vector<float> depth_buffer;
  unsigned char *rgb_ptr = nullptr;
  float *depth_ptr = nullptr;
  if (spec.enable_rgb) {
    rgb_buffer.assign(pixel_count * kColorChannelCount, 0U);
    rgb_ptr = rgb_buffer.data();
  }
  if (spec.enable_depth) {
    depth_buffer.assign(pixel_count, 0.0F);
    depth_ptr = depth_buffer.data();
  }
  mjr_readPixels(rgb_ptr, depth_ptr, viewport, &render_context_);

  if (spec.enable_rgb) {
    state->color.width = static_cast<std::uint32_t>(spec.width);
    state->color.height = static_cast<std::uint32_t>(spec.height);
    state->color.step =
        static_cast<std::uint32_t>(spec.width * kColorChannelCount);
    state->color.data.resize(rgb_buffer.size());
    transform_rgb(rgb_buffer, state->color.width, state->color.height,
                  state->color.data);
  }
  if (spec.enable_depth) {
    state->depth.width = static_cast<std::uint32_t>(spec.width);
    state->depth.height = static_cast<std::uint32_t>(spec.height);
    state->depth.data.resize(depth_buffer.size());
    if (!transform_depth(depth_buffer, state->depth.width, state->depth.height,
                         state->depth.data)) {
      return fail("failed to transform the camera depth image");
    }
  }
  out = std::static_pointer_cast<const CameraRenderState>(state);
  return true;
}

bool CameraRenderer::create_context() {
  if (gl_context_.backend != OffscreenGlBackend::None) {
    return true;
  }
  if (config_.allow_glfw_backend && create_glfw_context()) {
    return true;
  }
  if (!config_.allow_egl_backend) {
    LOG_ERROR << "no usable offscreen rendering backend is available.";
    return false;
  }
  return create_egl_context();
}

bool CameraRenderer::create_glfw_context() {
  // GLFW is process-lifetime state. This module never terminates it because
  // the unmodified vendored viewer owns the process-exit termination hook.
  if (glfwInit() == GLFW_FALSE) {
    LOG_WARNING << "failed to initialize GLFW; trying EGL.";
    return false;
  }
  glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
  glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_API);
  gl_context_.window = glfwCreateWindow(
      kHiddenContextWidth, kHiddenContextHeight, "", nullptr, nullptr);
  if (gl_context_.window == nullptr) {
    LOG_WARNING << "failed to create hidden GLFW OpenGL context; trying EGL.";
    return false;
  }
  gl_context_.backend = OffscreenGlBackend::Glfw;
  return true;
}

bool CameraRenderer::activate_context() {
  if (gl_context_.backend == OffscreenGlBackend::Glfw &&
      gl_context_.window != nullptr) {
    glfwMakeContextCurrent(gl_context_.window);
    return true;
  }
  if (gl_context_.backend == OffscreenGlBackend::Egl &&
      gl_context_.egl_display != EGL_NO_DISPLAY &&
      gl_context_.egl_context != EGL_NO_CONTEXT &&
      gl_context_.egl_surface != EGL_NO_SURFACE) {
    return eglMakeCurrent(static_cast<::EGLDisplay>(gl_context_.egl_display),
                          static_cast<::EGLSurface>(gl_context_.egl_surface),
                          static_cast<::EGLSurface>(gl_context_.egl_surface),
                          static_cast<::EGLContext>(gl_context_.egl_context));
  }
  LOG_ERROR << "offscreen OpenGL context is not available.";
  return false;
}

bool CameraRenderer::resize_offscreen_buffer(int width, int height) {
  std::size_t pixel_count = 0;
  if (!pixel_count_for(width, height, pixel_count)) {
    LOG_ERROR << "offscreen image dimensions must be positive.";
    return false;
  }
  if (width <= offscreen_width_ && height <= offscreen_height_) {
    return true;
  }
  mjr_resizeOffscreen(std::max(offscreen_width_, width),
                      std::max(offscreen_height_, height), &render_context_);
  mjr_setBuffer(mjFB_OFFSCREEN, &render_context_);
  offscreen_width_ = std::max(offscreen_width_, width);
  offscreen_height_ = std::max(offscreen_height_, height);
  return true;
}

bool CameraRenderer::create_render_resources() {
  if (model_ == nullptr) {
    LOG_ERROR << "camera renderer model is not available.";
    return false;
  }
  mjv_defaultScene(&scene_);
  mjv_defaultOption(&option_);
  mjr_defaultContext(&render_context_);
  option_.flags[mjVIS_RANGEFINDER] = 0;
  for (int group = 0; group < mjNGROUP; ++group) {
    option_.sitegroup[group] = 0;
  }
  mjv_makeScene(model_, &scene_, config_.max_scene_geometries);
  mjr_makeContext(model_, &render_context_, kFontScale);
  mjr_setBuffer(mjFB_OFFSCREEN, &render_context_);
  return true;
}

CameraRenderIntrinsics
CameraRenderer::compute_intrinsics(double fovy_degrees, std::uint32_t width,
                                   std::uint32_t height) const {
  CameraRenderIntrinsics intrinsics;
  if (width == 0U || height == 0U) {
    return intrinsics;
  }
  const double aspect = static_cast<double>(width) / height;
  const double fovy_radians = fovy_degrees * kPi / 180.0;
  intrinsics.fy = height / (2.0 * std::tan(fovy_radians / 2.0));
  const double fovx_radians =
      2.0 * std::atan(aspect * std::tan(fovy_radians / 2.0));
  intrinsics.fx = width / (2.0 * std::tan(fovx_radians / 2.0));
  intrinsics.cx = (static_cast<double>(width) - 1.0) / 2.0;
  intrinsics.cy = (static_cast<double>(height) - 1.0) / 2.0;
  intrinsics.k = {intrinsics.fx, 0.0, intrinsics.cx, 0.0, intrinsics.fy,
                  intrinsics.cy, 0.0, 0.0,           1.0};
  intrinsics.p = {intrinsics.fx, 0.0, intrinsics.cx, 0.0, 0.0, intrinsics.fy,
                  intrinsics.cy, 0.0, 0.0,           0.0, 1.0, 0.0};
  return intrinsics;
}

void CameraRenderer::transform_rgb(const std::vector<std::uint8_t> &source,
                                   std::uint32_t width, std::uint32_t height,
                                   std::vector<std::uint8_t> &dest) const {
  const std::size_t row_size =
      static_cast<std::size_t>(width) * kColorChannelCount;
  for (std::uint32_t row = 0; row < height; ++row) {
    const std::size_t src_offset = static_cast<std::size_t>(row) * row_size;
    const std::size_t dst_offset =
        static_cast<std::size_t>(height - 1U - row) * row_size;
    std::memcpy(dest.data() + dst_offset, source.data() + src_offset, row_size);
  }
}

bool CameraRenderer::transform_depth(const std::vector<float> &source,
                                     std::uint32_t width, std::uint32_t height,
                                     std::vector<float> &dest) const {
  if (model_ == nullptr) {
    LOG_ERROR << "camera renderer model is not available.";
    return false;
  }
  const float near =
      static_cast<float>(model_->vis.map.znear * model_->stat.extent);
  const float far =
      static_cast<float>(model_->vis.map.zfar * model_->stat.extent);
  if (!std::isfinite(near) || !std::isfinite(far) || near <= 0.0F ||
      far <= near) {
    LOG_ERROR << "MuJoCo camera depth range must satisfy 0 < near < far.";
    return false;
  }
  if (source.size() != dest.size() ||
      source.size() != static_cast<std::size_t>(width) * height) {
    LOG_ERROR << "camera depth buffer has an unexpected size.";
    return false;
  }
  const float depth_scale = 1.0F - near / far;
  for (std::uint32_t row = 0; row < height; ++row) {
    for (std::uint32_t column = 0; column < width; ++column) {
      const std::size_t src_index =
          static_cast<std::size_t>(row) * width + column;
      const std::size_t dst_index =
          static_cast<std::size_t>(height - 1U - row) * width + column;
      const float denominator = 1.0F - source[src_index] * depth_scale;
      if (!std::isfinite(denominator) || denominator <= 0.0F) {
        LOG_ERROR << "camera depth buffer contains an invalid normalized depth "
                     "value.";
        return false;
      }
      dest[dst_index] = near / denominator;
    }
  }
  return true;
}

bool CameraRenderer::create_egl_context() {
  gl_context_.egl_display = eglGetPlatformDisplay(EGL_PLATFORM_SURFACELESS_MESA,
                                                  EGL_DEFAULT_DISPLAY, nullptr);
  if (gl_context_.egl_display == EGL_NO_DISPLAY) {
    gl_context_.egl_display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
  }
  if (gl_context_.egl_display == EGL_NO_DISPLAY) {
    LOG_ERROR << "failed to acquire EGL display.";
    return false;
  }
  EGLint major = 0;
  EGLint minor = 0;
  if (!eglInitialize(static_cast<::EGLDisplay>(gl_context_.egl_display), &major,
                     &minor)) {
    LOG_ERROR << "failed to initialize EGL.";
    destroy_egl_context();
    return false;
  }
  const EGLint config_attribs[] = {
      EGL_SURFACE_TYPE,    EGL_PBUFFER_BIT, EGL_RED_SIZE,   kEglColorBits,
      EGL_GREEN_SIZE,      kEglColorBits,   EGL_BLUE_SIZE,  kEglColorBits,
      EGL_ALPHA_SIZE,      kEglColorBits,   EGL_DEPTH_SIZE, kEglDepthBits,
      EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT,  EGL_NONE};
  EGLConfig egl_config = nullptr;
  EGLint num_configs = 0;
  if (!eglChooseConfig(static_cast<::EGLDisplay>(gl_context_.egl_display),
                       config_attribs, &egl_config, 1, &num_configs) ||
      num_configs == 0) {
    LOG_ERROR << "failed to choose EGL config.";
    destroy_egl_context();
    return false;
  }
  if (!eglBindAPI(EGL_OPENGL_API)) {
    LOG_ERROR << "failed to bind EGL OpenGL API.";
    destroy_egl_context();
    return false;
  }
  gl_context_.egl_context =
      eglCreateContext(static_cast<::EGLDisplay>(gl_context_.egl_display),
                       egl_config, EGL_NO_CONTEXT, nullptr);
  if (gl_context_.egl_context == EGL_NO_CONTEXT) {
    LOG_ERROR << "failed to create EGL context.";
    destroy_egl_context();
    return false;
  }
  const EGLint pbuffer_attribs[] = {EGL_WIDTH, kHiddenContextWidth, EGL_HEIGHT,
                                    kHiddenContextHeight, EGL_NONE};
  gl_context_.egl_surface = eglCreatePbufferSurface(
      static_cast<::EGLDisplay>(gl_context_.egl_display), egl_config,
      pbuffer_attribs);
  if (gl_context_.egl_surface == EGL_NO_SURFACE) {
    LOG_ERROR << "failed to create EGL pbuffer surface.";
    destroy_egl_context();
    return false;
  }
  gl_context_.backend = OffscreenGlBackend::Egl;
  return true;
}

void CameraRenderer::deactivate_context() {
  if (gl_context_.backend == OffscreenGlBackend::Glfw &&
      gl_context_.window != nullptr) {
    glfwMakeContextCurrent(nullptr);
  } else if (gl_context_.backend == OffscreenGlBackend::Egl &&
             gl_context_.egl_display != EGL_NO_DISPLAY) {
    UNUSED(eglMakeCurrent(static_cast<::EGLDisplay>(gl_context_.egl_display),
                          EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT));
  }
}

void CameraRenderer::destroy_context() {
  destroy_glfw_context();
  destroy_egl_context();
  gl_context_ = OffscreenGlContext{};
}

void CameraRenderer::destroy_render_resources() {
  mjv_freeScene(&scene_);
  mjr_freeContext(&render_context_);
  mjv_defaultScene(&scene_);
  mjv_defaultOption(&option_);
  mjr_defaultContext(&render_context_);
}

void CameraRenderer::destroy_glfw_context() {
  if (gl_context_.window != nullptr) {
    glfwDestroyWindow(gl_context_.window);
    gl_context_.window = nullptr;
  }
}

void CameraRenderer::destroy_egl_context() {
  if (gl_context_.egl_display == EGL_NO_DISPLAY) {
    return;
  }
  const auto display = static_cast<::EGLDisplay>(gl_context_.egl_display);
  UNUSED(
      eglMakeCurrent(display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT));
  if (gl_context_.egl_surface != EGL_NO_SURFACE) {
    UNUSED(eglDestroySurface(
        display, static_cast<::EGLSurface>(gl_context_.egl_surface)));
    gl_context_.egl_surface = EGL_NO_SURFACE;
  }
  if (gl_context_.egl_context != EGL_NO_CONTEXT) {
    UNUSED(eglDestroyContext(
        display, static_cast<::EGLContext>(gl_context_.egl_context)));
    gl_context_.egl_context = EGL_NO_CONTEXT;
  }
  UNUSED(eglTerminate(display));
  gl_context_.egl_display = EGL_NO_DISPLAY;
}

} // namespace mujoco_simulation
