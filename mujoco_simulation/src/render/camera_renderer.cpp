#include "render/camera_renderer.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <exception>
#include <limits>
#include <unordered_set>
#include <utility>

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLFW/glfw3.h>

#include "log/logging.hpp"
#include "common/macro.hpp"

namespace mujoco_simulation {
namespace {

constexpr int kHiddenContextWidth = 1;
constexpr int kHiddenContextHeight = 1;
constexpr int kColorChannelCount = 3;
constexpr int kEglColorBits = 8;
constexpr int kEglDepthBits = 24;
constexpr int kFontScale = mjFONTSCALE_150;
constexpr double kPi = 3.14159265358979323846;
constexpr CameraId kMaximumCameraId{255};

template <typename Callback>
class ScopeExit {
public:
    explicit ScopeExit(Callback callback) : callback_(std::move(callback)) {}
    ~ScopeExit() { callback_(); }

    ScopeExit(const ScopeExit&) = delete;
    ScopeExit& operator=(const ScopeExit&) = delete;

private:
    Callback callback_;
};

template <typename Callback>
ScopeExit(Callback) -> ScopeExit<Callback>;

bool pixel_count_for(int width, int height, std::size_t& pixel_count) {
    if (width <= 0 || height <= 0) {
        return false;
    }
    const auto unsigned_width = static_cast<std::size_t>(width);
    const auto unsigned_height = static_cast<std::size_t>(height);
    if (unsigned_width > std::numeric_limits<std::size_t>::max() / unsigned_height) {
        return false;
    }
    pixel_count = unsigned_width * unsigned_height;
    return true;
}

CameraFrame frame_from_render_state(const CameraRenderState& state) {
    CameraFrame frame;
    frame.image.timestamp = state.timestamp;
    frame.image.frame_id = state.frame_id;
    frame.image.width = state.color.width;
    frame.image.height = state.color.height;
    frame.image.encoding = "rgb8";
    frame.image.step = state.color.step;
    frame.image.data = state.color.data;
    frame.depth_image.timestamp = state.timestamp;
    frame.depth_image.frame_id = state.optical_frame_id;
    frame.depth_image.width = state.depth.width;
    frame.depth_image.height = state.depth.height;
    frame.depth_image.encoding = "32FC1";
    frame.depth_image.step = state.depth.width * sizeof(float);
    frame.depth_image.data.resize(state.depth.data.size() * sizeof(float));
    if (!state.depth.data.empty()) {
        std::memcpy(
            frame.depth_image.data.data(), state.depth.data.data(), frame.depth_image.data.size());
    }
    frame.camera_info.width = state.color.width;
    frame.camera_info.height = state.color.height;
    frame.camera_info.k = state.intrinsics.k;
    frame.camera_info.p = state.intrinsics.p;
    return frame;
}

}  // namespace

CameraRenderer::CameraRenderer() : CameraRenderer(CameraRendererConfig{}) {}

CameraRenderer::CameraRenderer(CameraRendererConfig config) : config_(config) {}

CameraRenderer::~CameraRenderer() { UNUSED(release()); }

bool CameraRenderer::initialize(const mjContext& context) {
    if (!context.valid()) {
        SIM_ERROR << "camera renderer requires a valid MuJoCo context.";
        return false;
    }
    return initialize(context.model);
}

bool CameraRenderer::initialize(const mjModel* model) {
    std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
    if (initialized_.load() && running_.load()) {
        return true;
    }

    // A worker that failed after a successful initialize() has already released
    // its thread-local OpenGL resources, but its thread object and mjData
    // buffers still belong to this renderer. Reap that failed lifecycle before
    // allocating a replacement one; otherwise initialize() would overwrite the
    // stale pointers and report a healthy renderer without a worker.
    if (worker_thread_.joinable() || pending_data_ != nullptr || render_data_ != nullptr ||
        model_ != nullptr) {
        if (!release_locked()) {
            return false;
        }
    }
    if (model == nullptr) {
        SIM_ERROR << "camera renderer requires a valid MuJoCo model.";
        return false;
    }
    if (config_.max_scene_geometries <= 0) {
        SIM_ERROR << "max_scene_geometries must be positive.";
        return false;
    }
    if (!config_.allow_glfw_backend && !config_.allow_egl_backend) {
        SIM_ERROR << "camera renderer requires a GLFW or EGL backend.";
        return false;
    }
    if (config_.completed_ticket_history == 0U) {
        SIM_ERROR << "camera renderer ticket history must be positive.";
        return false;
    }

    model_ = model;
    pending_data_ = mj_makeData(model_);
    render_data_ = mj_makeData(model_);
    if (pending_data_ == nullptr || render_data_ == nullptr) {
        SIM_ERROR << "failed to allocate camera render data buffers.";
        UNUSED(release_locked());
        return false;
    }

    {
        std::unique_lock<std::mutex> lock(job_mutex_);
        pending_tasks_.clear();
        pending_ready_ = false;
        pending_ticket_ = 0;
        submitted_ticket_ = 0;
        expired_through_ticket_ = {};
        completed_results_.clear();
        worker_ready_.store(false);
        worker_initialization_failed_.store(false);
        stopping_ = false;
        lock.unlock();
    }
    snapshot_copy_count_.store(0);
    active_ticket_sequence_.store(0);
    try {
        worker_thread_ = std::thread(&CameraRenderer::worker_loop, this);
    } catch (const std::exception&) {
        SIM_ERROR << "failed to start the camera render worker.";
        UNUSED(release_locked());
        return false;
    }

    const auto startup_deadline = std::chrono::steady_clock::now() + kWorkerTimeout;
    while (!worker_ready_.load() && !worker_initialization_failed_.load() &&
           std::chrono::steady_clock::now() < startup_deadline) {
        std::this_thread::yield();
    }
    if (!worker_ready_.load() || worker_initialization_failed_.load()) {
        SIM_ERROR << "camera render worker initialization timed out or failed.";
        UNUSED(release_locked());
        return false;
    }
    const std::uint64_t prior_generation = ticket_epoch_.load();
    if (prior_generation == std::numeric_limits<std::uint64_t>::max()) {
        SIM_ERROR << "camera renderer ticket epoch overflowed.";
        UNUSED(release_locked());
        return false;
    }
    ticket_epoch_.store(prior_generation + 1U);
    initialized_.store(true);
    return true;
}

std::optional<CameraRenderTicket> CameraRenderer::submit(
    const mjContext& context, std::vector<CameraRenderTask> tasks) {
    if (!context.valid()) {
        SIM_ERROR << "camera renderer requires a valid MuJoCo context.";
        return std::nullopt;
    }
    CameraRenderBatchRequest request;
    request.model = context.model;
    request.data = context.data;
    request.tasks = std::move(tasks);
    request.simulation_time = context.data->time;
    return submit(request);
}

std::optional<CameraRenderTicket> CameraRenderer::submit(
    const mjModel* model, const mjData* data, std::vector<CameraRenderTask> tasks) {
    CameraRenderBatchRequest request;
    request.model = model;
    request.data = data;
    request.tasks = std::move(tasks);
    request.simulation_time = data == nullptr ? 0.0 : data->time;
    return submit(request);
}

std::optional<CameraRenderTicket> CameraRenderer::submit(
    const CameraRenderBatchRequest& request, bool* replaced_pending_batch) {
    if (replaced_pending_batch != nullptr) *replaced_pending_batch = false;
    if (request.tasks.empty()) {
        return CameraRenderTicket{};
    }
    std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
    if (request.model == nullptr || request.data == nullptr) {
        SIM_ERROR << "camera renderer requires a valid MuJoCo context.";
        return std::nullopt;
    }
    if (!initialized_.load() || !running_.load()) {
        SIM_ERROR << "camera renderer is not initialized.";
        return std::nullopt;
    }
    std::unordered_set<CameraId> ids;
    for (const CameraRenderTask& task : request.tasks) {
        // camera_id is the batch protocol contract; CameraConfig remains the
        // legacy rendering payload and carries no id authority.
        const CameraId id = task.camera_id;
        if (id > kMaximumCameraId || !ids.insert(id).second) {
            SIM_ERROR << "camera render task id is invalid, outside range, or duplicated.";
            return std::nullopt;
        }
    }
    CameraRenderTicket ticket;
    {
        std::lock_guard<std::mutex> lock(job_mutex_);
        if (stopping_ || pending_data_ == nullptr) {
            SIM_WARN << "camera renderer is stopping; dropped render request.";
            return std::nullopt;
        }
        if (mj_copyData(pending_data_, request.model, request.data) == nullptr) {
            SIM_ERROR << "failed to copy MuJoCo simulation data for camera rendering.";
            return std::nullopt;
        }
        snapshot_copy_count_.fetch_add(1);
        if (pending_ready_) {
            if (replaced_pending_batch != nullptr) *replaced_pending_batch = true;
            CameraBatchResult superseded;
            superseded.ticket = {ticket_epoch_.load(), pending_ticket_};
            superseded.status = CameraBatchStatus::Superseded;
            superseded.simulation_step = pending_simulation_step_;
            superseded.simulation_time = pending_simulation_time_;
            for (const CameraRenderTask& pending : pending_tasks_) {
                superseded.cameras.push_back(
                    {pending.camera_id,
                     CameraTaskStatus::Superseded,
                     {},
                     superseded.ticket.generation,
                     superseded.ticket.sequence,
                     superseded.simulation_step,
                     superseded.simulation_time,
                     pending.sequence,
                     pending.timestamp,
                     "render request was superseded"});
            }
            completed_results_[{superseded.ticket.generation, superseded.ticket.sequence}] =
                std::move(superseded);
            while (completed_results_.size() > config_.completed_ticket_history) {
                const TicketKey expired_key = completed_results_.begin()->first;
                expired_through_ticket_ = {expired_key.first, expired_key.second};
                completed_results_.erase(completed_results_.begin());
            }
        }
        pending_tasks_ = request.tasks;
        pending_simulation_step_ = request.simulation_step;
        pending_simulation_time_ = request.simulation_time;
        pending_ticket_ = ++submitted_ticket_;
        ticket = {ticket_epoch_.load(), pending_ticket_};
        pending_ready_ = true;
    }
    completion_condition_.notify_all();
    job_condition_.notify_one();
    return ticket;
}

CameraRenderWaitStatus CameraRenderer::wait_result(
    CameraRenderTicket requested, CameraBatchResult* result) {
    return wait_result(
        requested, std::chrono::duration_cast<std::chrono::milliseconds>(kWorkerTimeout), result);
}

CameraRenderWaitStatus CameraRenderer::wait_result(
    CameraRenderTicket requested, std::chrono::milliseconds timeout, CameraBatchResult* result) {
    std::unique_lock<std::mutex> lock(job_mutex_);
    if (requested.is_noop()) {
        return CameraRenderWaitStatus::Completed;
    }
    const std::uint64_t requested_generation = requested.generation;
    const TicketKey requested_key{requested.generation, requested.sequence};
    if (requested_generation != ticket_epoch_.load()) {
        // release_locked() invalidates the epoch before joining the worker. A
        // waiter released during that boundary has a stopped lifecycle and must
        // receive the shutdown terminal status.
        if (stopping_ || !initialized_.load() || !running_.load()) {
            SIM_WARN << "camera render worker stopped before completing the "
                        "submitted frame.";
            return CameraRenderWaitStatus::Stopped;
        }
        SIM_ERROR << "camera render ticket belongs to another renderer lifecycle.";
        return CameraRenderWaitStatus::InvalidTicket;
    }
    if (requested.sequence > submitted_ticket_) {
        SIM_ERROR << "camera render ticket was not submitted by this renderer.";
        return CameraRenderWaitStatus::InvalidTicket;
    }
    if (requested.generation == expired_through_ticket_.generation &&
        requested.sequence <= expired_through_ticket_.sequence) {
        SIM_ERROR << "camera render ticket has expired.";
        return CameraRenderWaitStatus::Stale;
    }
    const bool completed =
        completion_condition_.wait_for(lock, timeout, [this, requested_generation, requested_key] {
            return ticket_epoch_.load() != requested_generation || stopping_ ||
                   completed_results_.find(requested_key) != completed_results_.end() ||
                   worker_initialization_failed_;
        });
    if (!completed) {
        SIM_ERROR << "camera render worker timed out for the submitted frame.";
        return CameraRenderWaitStatus::Timeout;
    }
    if (ticket_epoch_.load() != requested_generation) {
        SIM_ERROR << "camera render ticket belongs to another renderer lifecycle.";
        return CameraRenderWaitStatus::InvalidTicket;
    }
    if (stopping_ || worker_initialization_failed_) {
        SIM_ERROR << "camera render worker stopped before completing the frame.";
        return CameraRenderWaitStatus::Stopped;
    }
    const auto completed_batch = completed_results_.find(requested_key);
    if (completed_batch == completed_results_.end()) {
        SIM_ERROR << "camera render worker did not complete the submitted frame.";
        return CameraRenderWaitStatus::Stopped;
    }
    const CameraBatchResult& batch = completed_batch->second;
    if (result != nullptr) *result = batch;
    if (!batch.all_succeeded) {
        SIM_ERROR << "camera render worker failed to render the submitted frame.";
        const bool superseded =
            !batch.cameras.empty() && std::all_of(
                                          batch.cameras.begin(), batch.cameras.end(),
                                          [](const CameraRenderTaskResult& camera) {
                                              return camera.status == CameraTaskStatus::Superseded;
                                          });
        if (superseded) return CameraRenderWaitStatus::Superseded;
        return batch.status == CameraBatchStatus::PartiallyFailed
                   ? CameraRenderWaitStatus::PartiallyFailed
                   : CameraRenderWaitStatus::Failed;
    }
    return CameraRenderWaitStatus::Completed;
}

CameraRenderWaitStatus CameraRenderer::query(
    CameraRenderTicket requested, CameraBatchResult* result) const {
    std::lock_guard<std::mutex> lock(job_mutex_);
    if (requested.is_noop()) return CameraRenderWaitStatus::Completed;
    if (requested.generation != ticket_epoch_.load() || requested.sequence > submitted_ticket_) {
        return CameraRenderWaitStatus::InvalidTicket;
    }
    if (requested.generation == expired_through_ticket_.generation &&
        requested.sequence <= expired_through_ticket_.sequence) {
        return CameraRenderWaitStatus::Stale;
    }
    const auto completed = completed_results_.find({requested.generation, requested.sequence});
    if (completed == completed_results_.end())
        return stopping_ || worker_initialization_failed_ ? CameraRenderWaitStatus::Stopped
                                                          : CameraRenderWaitStatus::Timeout;
    if (result != nullptr) *result = completed->second;
    if (completed->second.all_succeeded) return CameraRenderWaitStatus::Completed;
    const bool superseded = !completed->second.cameras.empty() &&
                            std::all_of(
                                completed->second.cameras.begin(), completed->second.cameras.end(),
                                [](const CameraRenderTaskResult& camera) {
                                    return camera.status == CameraTaskStatus::Superseded;
                                });
    if (superseded) return CameraRenderWaitStatus::Superseded;
    return completed->second.status == CameraBatchStatus::PartiallyFailed
               ? CameraRenderWaitStatus::PartiallyFailed
               : CameraRenderWaitStatus::Failed;
}

bool CameraRenderer::wait(CameraRenderTicket ticket, CameraBatchResult* result) {
    return wait_result(ticket, result) == CameraRenderWaitStatus::Completed;
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
            SIM_ERROR << "camera render worker cannot join itself.";
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
        std::lock_guard<std::mutex> lock(job_mutex_);
        pending_tasks_.clear();
        pending_ready_ = false;
        pending_ticket_ = 0;
        submitted_ticket_ = 0;
        expired_through_ticket_ = {};
        completed_results_.clear();
        worker_ready_.store(false);
        worker_initialization_failed_.store(false);
        stopping_ = false;
    }
    model_ = nullptr;
    active_ticket_sequence_.store(0);
    initialized_.store(false);
    running_.store(false);
    return true;
}

void CameraRenderer::worker_loop() {
    bool resources_ready = false;
    try {
        resources_ready = initialize_worker_resources();
    } catch (const std::exception&) {
        SIM_ERROR << "camera render worker threw during initialization.";
    } catch (...) {
        SIM_ERROR << "camera render worker threw an unknown initialization error.";
    }
    running_.store(resources_ready);
    worker_ready_.store(resources_ready);
    worker_initialization_failed_.store(!resources_ready);
    completion_condition_.notify_all();
    if (!resources_ready) {
        return;
    }

    try {
        while (true) {
            std::vector<CameraRenderTask> tasks;
            std::uint64_t ticket = 0;
            std::uint64_t simulation_step = 0;
            double simulation_time = 0.0;
            {
                std::unique_lock<std::mutex> lock(job_mutex_);
                job_condition_.wait(lock, [this] { return stopping_ || pending_ready_; });
                if (stopping_) {
                    break;
                }
                std::swap(render_data_, pending_data_);
                tasks = std::move(pending_tasks_);
                pending_tasks_.clear();
                ticket = pending_ticket_;
                simulation_step = pending_simulation_step_;
                simulation_time = pending_simulation_time_;
                pending_ready_ = false;
            }
            active_ticket_sequence_.store(ticket);

            CameraBatchResult batch;
            batch.ticket = {ticket_epoch_.load(), ticket};
            batch.simulation_step = simulation_step;
            batch.simulation_time = simulation_time;
            bool succeeded = activate_context();
            if (!succeeded) {
                SIM_ERROR << "failed to activate the camera render context.";
            }
            if (succeeded) {
                const ScopeExit deactivate([this] { deactivate_context(); });
                for (const CameraRenderTask& task : tasks) {
                    CameraRenderStatePtr state;
                    std::string error;
                    if (!render_task(task, state, &error)) {
                        succeeded = false;
                        batch.cameras.push_back(
                            {task.camera_id,
                             CameraTaskStatus::Failed,
                             {},
                             batch.ticket.generation,
                             batch.ticket.sequence,
                             batch.simulation_step,
                             batch.simulation_time,
                             task.sequence,
                             task.timestamp,
                             std::move(error)});
                        SIM_WARN << "camera render request failed; keeping the prior frame.";
                        continue;
                    }
                    batch.cameras.push_back(
                        {task.camera_id, CameraTaskStatus::Completed,
                         frame_from_render_state(*state), batch.ticket.generation,
                         batch.ticket.sequence, batch.simulation_step, batch.simulation_time,
                         task.sequence, task.timestamp, ""});
                }
            } else {
                for (const CameraRenderTask& task : tasks) {
                    batch.cameras.push_back(
                        {task.camera_id,
                         CameraTaskStatus::Failed,
                         {},
                         batch.ticket.generation,
                         batch.ticket.sequence,
                         batch.simulation_step,
                         batch.simulation_time,
                         task.sequence,
                         task.timestamp,
                         "failed to activate render context"});
                }
            }

            {
                std::lock_guard<std::mutex> lock(job_mutex_);
                batch.all_succeeded =
                    succeeded && std::all_of(
                                     batch.cameras.begin(), batch.cameras.end(),
                                     [](const CameraRenderTaskResult& camera) {
                                         return camera.status == CameraTaskStatus::Completed;
                                     });
                if (batch.all_succeeded) {
                    batch.status = CameraBatchStatus::Completed;
                } else if (
                    !batch.cameras.empty() && std::all_of(
                                                  batch.cameras.begin(), batch.cameras.end(),
                                                  [](const CameraRenderTaskResult& camera) {
                                                      return camera.status ==
                                                             CameraTaskStatus::Superseded;
                                                  })) {
                    batch.status = CameraBatchStatus::Superseded;
                } else {
                    const bool any_success = std::any_of(
                        batch.cameras.begin(), batch.cameras.end(),
                        [](const CameraRenderTaskResult& camera) {
                            return camera.status == CameraTaskStatus::Completed;
                        });
                    batch.status = any_success ? CameraBatchStatus::PartiallyFailed
                                               : CameraBatchStatus::Failed;
                }
                const TicketKey completed_key{batch.ticket.generation, batch.ticket.sequence};
                completed_results_[completed_key] = std::move(batch);
                while (completed_results_.size() > config_.completed_ticket_history) {
                    const TicketKey expired_key = completed_results_.begin()->first;
                    expired_through_ticket_ = {expired_key.first, expired_key.second};
                    completed_results_.erase(completed_results_.begin());
                }
            }
            active_ticket_sequence_.store(0);
            completion_condition_.notify_all();
        }
    } catch (const std::exception&) {
        SIM_ERROR << "camera render worker threw while rendering.";
        std::lock_guard<std::mutex> lock(job_mutex_);
        worker_initialization_failed_.store(true);
        stopping_ = true;
    } catch (...) {
        SIM_ERROR << "camera render worker threw an unknown rendering error.";
        std::lock_guard<std::mutex> lock(job_mutex_);
        worker_initialization_failed_.store(true);
        stopping_ = true;
    }

    release_worker_resources();
    active_ticket_sequence_.store(0);
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
        SIM_ERROR << "failed to activate offscreen context during worker release.";
    }
    destroy_render_resources();
    if (context_active) {
        deactivate_context();
    }
    destroy_context();
    offscreen_width_ = 0;
    offscreen_height_ = 0;
}

bool CameraRenderer::render_task(
    const CameraRenderTask& task, CameraRenderStatePtr& out, std::string* error) {
    const auto fail = [error](const char* message) {
        if (error != nullptr) *error = message;
        return false;
    };
    const CameraConfig& spec = task.config;
    if (model_ == nullptr || render_data_ == nullptr) {
        SIM_ERROR << "camera render resources are not initialized.";
        return fail("camera render resources are not initialized");
    }
    if (spec.name.empty() || spec.camera_name.empty()) {
        SIM_ERROR << "camera component and MuJoCo camera names must not be empty.";
        return fail("camera component and MuJoCo camera names must not be empty");
    }
    if (spec.width <= 0 || spec.height <= 0) {
        SIM_ERROR << "camera width and height must be positive.";
        return fail("camera width and height must be positive");
    }
    if (!spec.enable_rgb && !spec.enable_depth) {
        SIM_ERROR << "camera must enable rgb or depth output.";
        return fail("camera must enable RGB or depth output");
    }
    const int camera_id = mj_name2id(model_, mjOBJ_CAMERA, spec.camera_name.c_str());
    if (camera_id < 0) {
        SIM_ERROR << "camera was not found in model.";
        return fail("camera was not found in the MuJoCo model");
    }
    const double fovy_degrees = static_cast<double>(model_->cam_fovy[camera_id]);
    if (!std::isfinite(fovy_degrees) || fovy_degrees <= 0.0 || fovy_degrees >= 180.0) {
        SIM_ERROR << "camera field of view must be in the range (0, 180) degrees.";
        return fail("camera field of view must be in (0, 180) degrees");
    }
    if (!resize_offscreen_buffer(spec.width, spec.height)) {
        return fail("failed to resize the offscreen render buffer");
    }

    std::size_t pixel_count = 0;
    if (!pixel_count_for(spec.width, spec.height, pixel_count) ||
        pixel_count > std::numeric_limits<std::size_t>::max() / kColorChannelCount) {
        SIM_ERROR << "camera image dimensions exceed the supported pixel capacity.";
        return fail("camera image dimensions exceed supported pixel capacity");
    }

    mjr_setBuffer(mjFB_OFFSCREEN, &render_context_);
    mjvCamera camera{};
    mjv_defaultCamera(&camera);
    camera.type = mjCAMERA_FIXED;
    camera.fixedcamid = camera_id;
    const mjrRect viewport{0, 0, spec.width, spec.height};
    mjv_updateScene(model_, render_data_, &option_, nullptr, &camera, mjCAT_ALL, &scene_);
    mjr_render(viewport, &scene_, &render_context_);

    auto state = std::make_shared<CameraRenderState>();
    state->sequence = task.sequence;
    state->timestamp = task.timestamp;
    state->frame_id = spec.frame_id.empty() ? spec.name : spec.frame_id;
    state->optical_frame_id =
        spec.optical_frame_id.empty() ? state->frame_id : spec.optical_frame_id;
    state->intrinsics = compute_intrinsics(
        fovy_degrees, static_cast<std::uint32_t>(spec.width),
        static_cast<std::uint32_t>(spec.height));

    std::vector<std::uint8_t> rgb_buffer;
    std::vector<float> depth_buffer;
    unsigned char* rgb_ptr = nullptr;
    float* depth_ptr = nullptr;
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
        state->color.step = static_cast<std::uint32_t>(spec.width * kColorChannelCount);
        state->color.data.resize(rgb_buffer.size());
        transform_rgb(rgb_buffer, state->color.width, state->color.height, state->color.data);
    }
    if (spec.enable_depth) {
        state->depth.width = static_cast<std::uint32_t>(spec.width);
        state->depth.height = static_cast<std::uint32_t>(spec.height);
        state->depth.data.resize(depth_buffer.size());
        if (!transform_depth(
                depth_buffer, state->depth.width, state->depth.height, state->depth.data)) {
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
        SIM_ERROR << "no usable offscreen rendering backend is available.";
        return false;
    }
    return create_egl_context();
}

bool CameraRenderer::create_glfw_context() {
    // GLFW is process-lifetime state. This module never terminates it because
    // the unmodified vendored viewer owns the process-exit termination hook.
    if (glfwInit() == GLFW_FALSE) {
        SIM_WARN << "failed to initialize GLFW; trying EGL.";
        return false;
    }
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_API);
    gl_context_.window =
        glfwCreateWindow(kHiddenContextWidth, kHiddenContextHeight, "", nullptr, nullptr);
    if (gl_context_.window == nullptr) {
        SIM_WARN << "failed to create hidden GLFW OpenGL context; trying EGL.";
        return false;
    }
    gl_context_.backend = OffscreenGlBackend::Glfw;
    return true;
}

bool CameraRenderer::activate_context() {
    if (gl_context_.backend == OffscreenGlBackend::Glfw && gl_context_.window != nullptr) {
        glfwMakeContextCurrent(gl_context_.window);
        return true;
    }
    if (gl_context_.backend == OffscreenGlBackend::Egl &&
        gl_context_.egl_display != EGL_NO_DISPLAY && gl_context_.egl_context != EGL_NO_CONTEXT &&
        gl_context_.egl_surface != EGL_NO_SURFACE) {
        return eglMakeCurrent(
            static_cast<::EGLDisplay>(gl_context_.egl_display),
            static_cast<::EGLSurface>(gl_context_.egl_surface),
            static_cast<::EGLSurface>(gl_context_.egl_surface),
            static_cast<::EGLContext>(gl_context_.egl_context));
    }
    SIM_ERROR << "offscreen OpenGL context is not available.";
    return false;
}

bool CameraRenderer::resize_offscreen_buffer(int width, int height) {
    std::size_t pixel_count = 0;
    if (!pixel_count_for(width, height, pixel_count)) {
        SIM_ERROR << "offscreen image dimensions must be positive.";
        return false;
    }
    if (width <= offscreen_width_ && height <= offscreen_height_) {
        return true;
    }
    mjr_resizeOffscreen(
        std::max(offscreen_width_, width), std::max(offscreen_height_, height), &render_context_);
    mjr_setBuffer(mjFB_OFFSCREEN, &render_context_);
    offscreen_width_ = std::max(offscreen_width_, width);
    offscreen_height_ = std::max(offscreen_height_, height);
    return true;
}

bool CameraRenderer::create_render_resources() {
    if (model_ == nullptr) {
        SIM_ERROR << "camera renderer model is not available.";
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

CameraRenderIntrinsics CameraRenderer::compute_intrinsics(
    double fovy_degrees, std::uint32_t width, std::uint32_t height) const {
    CameraRenderIntrinsics intrinsics;
    if (width == 0U || height == 0U) {
        return intrinsics;
    }
    const double aspect = static_cast<double>(width) / height;
    const double fovy_radians = fovy_degrees * kPi / 180.0;
    intrinsics.fy = height / (2.0 * std::tan(fovy_radians / 2.0));
    const double fovx_radians = 2.0 * std::atan(aspect * std::tan(fovy_radians / 2.0));
    intrinsics.fx = width / (2.0 * std::tan(fovx_radians / 2.0));
    intrinsics.cx = (static_cast<double>(width) - 1.0) / 2.0;
    intrinsics.cy = (static_cast<double>(height) - 1.0) / 2.0;
    intrinsics.k = {intrinsics.fx, 0.0, intrinsics.cx, 0.0, intrinsics.fy,
                    intrinsics.cy, 0.0, 0.0,           1.0};
    intrinsics.p = {intrinsics.fx, 0.0, intrinsics.cx, 0.0, 0.0, intrinsics.fy,
                    intrinsics.cy, 0.0, 0.0,           0.0, 1.0, 0.0};
    return intrinsics;
}

void CameraRenderer::transform_rgb(
    const std::vector<std::uint8_t>& source, std::uint32_t width, std::uint32_t height,
    std::vector<std::uint8_t>& dest) const {
    const std::size_t row_size = static_cast<std::size_t>(width) * kColorChannelCount;
    for (std::uint32_t row = 0; row < height; ++row) {
        const std::size_t src_offset = static_cast<std::size_t>(row) * row_size;
        const std::size_t dst_offset = static_cast<std::size_t>(height - 1U - row) * row_size;
        std::memcpy(dest.data() + dst_offset, source.data() + src_offset, row_size);
    }
}

bool CameraRenderer::transform_depth(
    const std::vector<float>& source, std::uint32_t width, std::uint32_t height,
    std::vector<float>& dest) const {
    if (model_ == nullptr) {
        SIM_ERROR << "camera renderer model is not available.";
        return false;
    }
    const float near = static_cast<float>(model_->vis.map.znear * model_->stat.extent);
    const float far = static_cast<float>(model_->vis.map.zfar * model_->stat.extent);
    if (!std::isfinite(near) || !std::isfinite(far) || near <= 0.0F || far <= near) {
        SIM_ERROR << "MuJoCo camera depth range must satisfy 0 < near < far.";
        return false;
    }
    if (source.size() != dest.size() || source.size() != static_cast<std::size_t>(width) * height) {
        SIM_ERROR << "camera depth buffer has an unexpected size.";
        return false;
    }
    const float depth_scale = 1.0F - near / far;
    for (std::uint32_t row = 0; row < height; ++row) {
        for (std::uint32_t column = 0; column < width; ++column) {
            const std::size_t src_index = static_cast<std::size_t>(row) * width + column;
            const std::size_t dst_index =
                static_cast<std::size_t>(height - 1U - row) * width + column;
            const float denominator = 1.0F - source[src_index] * depth_scale;
            if (!std::isfinite(denominator) || denominator <= 0.0F) {
                SIM_ERROR << "camera depth buffer contains an invalid normalized depth "
                             "value.";
                return false;
            }
            dest[dst_index] = near / denominator;
        }
    }
    return true;
}

bool CameraRenderer::create_egl_context() {
    gl_context_.egl_display =
        eglGetPlatformDisplay(EGL_PLATFORM_SURFACELESS_MESA, EGL_DEFAULT_DISPLAY, nullptr);
    if (gl_context_.egl_display == EGL_NO_DISPLAY) {
        gl_context_.egl_display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
    }
    if (gl_context_.egl_display == EGL_NO_DISPLAY) {
        SIM_ERROR << "failed to acquire EGL display.";
        return false;
    }
    EGLint major = 0;
    EGLint minor = 0;
    if (!eglInitialize(static_cast<::EGLDisplay>(gl_context_.egl_display), &major, &minor)) {
        SIM_ERROR << "failed to initialize EGL.";
        destroy_egl_context();
        return false;
    }
    const EGLint config_attribs[] = {
        EGL_SURFACE_TYPE, EGL_PBUFFER_BIT, EGL_RED_SIZE,        kEglColorBits,  EGL_GREEN_SIZE,
        kEglColorBits,    EGL_BLUE_SIZE,   kEglColorBits,       EGL_ALPHA_SIZE, kEglColorBits,
        EGL_DEPTH_SIZE,   kEglDepthBits,   EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT, EGL_NONE};
    EGLConfig egl_config = nullptr;
    EGLint num_configs = 0;
    if (!eglChooseConfig(
            static_cast<::EGLDisplay>(gl_context_.egl_display), config_attribs, &egl_config, 1,
            &num_configs) ||
        num_configs == 0) {
        SIM_ERROR << "failed to choose EGL config.";
        destroy_egl_context();
        return false;
    }
    if (!eglBindAPI(EGL_OPENGL_API)) {
        SIM_ERROR << "failed to bind EGL OpenGL API.";
        destroy_egl_context();
        return false;
    }
    gl_context_.egl_context = eglCreateContext(
        static_cast<::EGLDisplay>(gl_context_.egl_display), egl_config, EGL_NO_CONTEXT, nullptr);
    if (gl_context_.egl_context == EGL_NO_CONTEXT) {
        SIM_ERROR << "failed to create EGL context.";
        destroy_egl_context();
        return false;
    }
    const EGLint pbuffer_attribs[] = {
        EGL_WIDTH, kHiddenContextWidth, EGL_HEIGHT, kHiddenContextHeight, EGL_NONE};
    gl_context_.egl_surface = eglCreatePbufferSurface(
        static_cast<::EGLDisplay>(gl_context_.egl_display), egl_config, pbuffer_attribs);
    if (gl_context_.egl_surface == EGL_NO_SURFACE) {
        SIM_ERROR << "failed to create EGL pbuffer surface.";
        destroy_egl_context();
        return false;
    }
    gl_context_.backend = OffscreenGlBackend::Egl;
    return true;
}

void CameraRenderer::deactivate_context() {
    if (gl_context_.backend == OffscreenGlBackend::Glfw && gl_context_.window != nullptr) {
        glfwMakeContextCurrent(nullptr);
    } else if (
        gl_context_.backend == OffscreenGlBackend::Egl &&
        gl_context_.egl_display != EGL_NO_DISPLAY) {
        UNUSED(eglMakeCurrent(
            static_cast<::EGLDisplay>(gl_context_.egl_display), EGL_NO_SURFACE, EGL_NO_SURFACE,
            EGL_NO_CONTEXT));
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
    UNUSED(eglMakeCurrent(display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT));
    if (gl_context_.egl_surface != EGL_NO_SURFACE) {
        UNUSED(eglDestroySurface(display, static_cast<::EGLSurface>(gl_context_.egl_surface)));
        gl_context_.egl_surface = EGL_NO_SURFACE;
    }
    if (gl_context_.egl_context != EGL_NO_CONTEXT) {
        UNUSED(eglDestroyContext(display, static_cast<::EGLContext>(gl_context_.egl_context)));
        gl_context_.egl_context = EGL_NO_CONTEXT;
    }
    UNUSED(eglTerminate(display));
    gl_context_.egl_display = EGL_NO_DISPLAY;
}

}  // namespace mujoco_simulation
