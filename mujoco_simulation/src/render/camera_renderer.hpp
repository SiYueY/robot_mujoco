#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <mujoco/mujoco.h>

#include "mujoco_simulation/config/simulation_config.hpp"

#include "component/camera/camera_render_service.hpp"
#include "render/offscreen_gl_context.hpp"

namespace mujoco_simulation {

class CameraRenderer {
public:
    CameraRenderer();
    explicit CameraRenderer(CameraRendererConfig config);
    ~CameraRenderer();

    CameraRenderer(const CameraRenderer&) = delete;
    CameraRenderer& operator=(const CameraRenderer&) = delete;

    /// 创建双缓冲数据并启动专属渲染线程；成功后重复调用不执行额外操作。
    bool initialize(const mjContext& context);
    bool initialize(const mjModel* model);
    /// 提交最新相机渲染请求；调用方必须在保护主 mjData 时调用。
    /// Empty task lists return a zero-valued immediately-completed no-op ticket.
    /// A no-op does not access the context and is valid even while this renderer
    /// is uninitialized.
    std::optional<CameraRenderTicket> submit(
        const mjContext& context, std::vector<CameraRenderTask> tasks);
    std::optional<CameraRenderTicket> submit(
        const mjModel* model, const mjData* data, std::vector<CameraRenderTask> tasks);
    /// Batch-protocol entry point used by CameraRenderService.  It snapshots
    /// request.data before returning and retains only value metadata.
    std::optional<CameraRenderTicket> submit(
        const CameraRenderBatchRequest& request, bool* replaced_pending_batch = nullptr);
    /// Wait for the exact batch identified by ticket and report why it failed.
    CameraRenderWaitStatus wait_result(
        CameraRenderTicket ticket, CameraBatchResult* result = nullptr);
    CameraRenderWaitStatus wait_result(
        CameraRenderTicket ticket, std::chrono::milliseconds timeout,
        CameraBatchResult* result = nullptr);
    CameraRenderWaitStatus query(
        CameraRenderTicket ticket, CameraBatchResult* result = nullptr) const;
    /// Compatibility wrapper for callers that only need success/failure.
    bool wait(CameraRenderTicket ticket, CameraBatchResult* result = nullptr);
    /// 停止 worker 并释放全部资源；未初始化时重复调用不执行额外操作。
    bool release();

    bool is_initialized() const noexcept { return initialized_.load(); }
    bool is_running() const noexcept { return running_.load(); }
    // Internal test observability for the one-snapshot-per-batch contract.
    std::uint64_t snapshot_copy_count() const noexcept { return snapshot_copy_count_.load(); }
    // Internal test observability for deterministic batch-supersede coverage.
    std::uint64_t active_ticket_sequence_for_test() const noexcept {
        return active_ticket_sequence_.load();
    }

private:
    using TicketKey = std::pair<std::uint64_t, std::uint64_t>;

    static constexpr auto kWorkerTimeout = std::chrono::seconds(5);
    bool release_locked();
    void worker_loop();
    bool initialize_worker_resources();
    void release_worker_resources();
    bool render_task(
        const CameraRenderTask& task, CameraRenderStatePtr& out, std::string* error = nullptr);

    bool create_context();
    bool create_glfw_context();
    bool create_egl_context();
    void destroy_context();
    void destroy_glfw_context();
    void destroy_egl_context();

    bool activate_context();
    void deactivate_context();

    bool create_render_resources();
    void destroy_render_resources();
    bool resize_offscreen_buffer(int width, int height);

    CameraRenderIntrinsics compute_intrinsics(
        double fovy_degrees, std::uint32_t width, std::uint32_t height) const;
    void transform_rgb(
        const std::vector<std::uint8_t>& source, std::uint32_t width, std::uint32_t height,
        std::vector<std::uint8_t>& dest) const;
    bool transform_depth(
        const std::vector<float>& source, std::uint32_t width, std::uint32_t height,
        std::vector<float>& dest) const;

    CameraRendererConfig config_{};
    const mjModel* model_{nullptr};
    mutable std::mutex lifecycle_mutex_;
    // Advance at both initialize and release boundaries. This is an invalidation
    // epoch rather than a count of successful worker lifecycles, so waiters are
    // invalidated as soon as release begins.
    std::atomic<std::uint64_t> ticket_epoch_{0};
    std::atomic<std::uint64_t> snapshot_copy_count_{0};
    std::atomic<std::uint64_t> active_ticket_sequence_{0};

    mutable std::mutex job_mutex_;
    std::condition_variable job_condition_;
    std::condition_variable completion_condition_;
    std::vector<CameraRenderTask> pending_tasks_;
    bool pending_ready_{false};
    std::uint64_t pending_ticket_{0};
    std::uint64_t pending_simulation_step_{0};
    double pending_simulation_time_{0.0};
    std::uint64_t submitted_ticket_{0};
    CameraRenderTicket expired_through_ticket_{};
    std::map<TicketKey, CameraBatchResult> completed_results_;
    std::atomic<bool> worker_ready_{false};
    std::atomic<bool> worker_initialization_failed_{false};
    bool stopping_{false};

    std::thread worker_thread_;
    std::atomic<bool> initialized_{false};
    std::atomic<bool> running_{false};

    OffscreenGlContext gl_context_{};
    mjData* pending_data_{nullptr};
    mjData* render_data_{nullptr};
    mjvScene scene_{};
    mjvOption option_{};
    mjrContext render_context_{};
    int offscreen_width_{0};
    int offscreen_height_{0};
};

}  // namespace mujoco_simulation
