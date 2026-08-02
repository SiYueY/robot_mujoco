#pragma once

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "mujoco_simulation/component/camera.hpp"
#include "mujoco_simulation/config/simulation_config.hpp"
#include "runtime/context.hpp"

namespace mujoco_simulation {

struct CameraRenderImage {
    std::uint32_t width{0};
    std::uint32_t height{0};
    std::uint32_t step{0};
    std::vector<std::uint8_t> data;
};

struct CameraRenderDepthImage {
    std::uint32_t width{0};
    std::uint32_t height{0};
    std::vector<float> data;
};

struct CameraRenderIntrinsics {
    double fx{0.0};
    double fy{0.0};
    double cx{0.0};
    double cy{0.0};
    Vector9d k{};
    Vector12d p{};
};

struct CameraRenderState {
    std::uint64_t sequence{0};
    std::uint64_t timestamp{0};
    std::string frame_id;
    std::string optical_frame_id;
    CameraRenderImage color;
    CameraRenderDepthImage depth;
    CameraRenderIntrinsics intrinsics;
};

enum class CameraPixelFormat { Rgb8 };

struct CameraRenderTask {
    CameraId camera_id{0};
    int mujoco_camera_id{-1};
    std::uint32_t width{0};
    std::uint32_t height{0};
    CameraPixelFormat pixel_format{CameraPixelFormat::Rgb8};
    bool render_depth{false};

    // Component configuration and sampling metadata remain internal renderer
    // details, while the fields above are the batch protocol contract.
    CameraConfig config;
    std::uint64_t sequence{0};
    std::uint64_t timestamp{0};
};

struct CameraRenderBatchRequest {
    std::uint64_t generation{0};
    std::uint64_t sequence{0};
    std::uint64_t simulation_step{0};
    double simulation_time{0.0};
    const mjModel* model{nullptr};
    const mjData* data{nullptr};
    std::vector<CameraRenderTask> tasks;
};

using CameraRenderStatePtr = std::shared_ptr<const CameraRenderState>;

struct CameraRenderTicket {
    std::uint64_t generation{0};
    std::uint64_t sequence{0};

    bool is_noop() const noexcept { return generation == 0 && sequence == 0; }
    bool valid() const noexcept { return !is_noop(); }
};

enum class CameraTaskStatus {
    Pending,
    Rendering,
    Completed,
    Failed,
    Superseded,
    Stale,
    Cancelled,
};

enum class CameraBatchStatus {
    Pending,
    Rendering,
    Completed,
    PartiallyFailed,
    Failed,
    Superseded,
    Stale,
    Cancelled,
};

struct CameraRenderTaskResult {
    CameraId camera_id{0};
    CameraTaskStatus status{CameraTaskStatus::Pending};
    CameraFrame frame;
    std::uint64_t generation{0};
    std::uint64_t batch_sequence{0};
    std::uint64_t simulation_step{0};
    double simulation_time{0.0};
    std::uint64_t sequence{0};
    std::uint64_t timestamp{0};
    std::string message;
};

struct CameraRenderBatchResult {
    CameraRenderTicket ticket{};
    CameraBatchStatus status{CameraBatchStatus::Pending};
    std::uint64_t simulation_step{0};
    double simulation_time{0.0};
    std::vector<CameraRenderTaskResult> cameras;
    bool all_succeeded{false};
};

using CameraBatchResult = CameraRenderBatchResult;

enum class CameraRenderSubmitResult {
    Accepted,
    ReplacedPendingBatch,
    InvalidRequest,
    Stopped,
    Failed,
};

enum class CameraRenderWaitStatus {
    Completed,
    PartiallyFailed,
    Failed,
    Superseded,
    Stale,
    InvalidTicket,
    Timeout,
    Cancelled,
    Stopped,
};

// Runtime owns this contract; Render supplies the OpenGL-backed
// implementation.  The request is accepted only after a single private
// mjData snapshot has been made, so workers never touch live runtime data.
class CameraRenderService {
public:
    virtual ~CameraRenderService() = default;
    CameraRenderService(const CameraRenderService&) = delete;
    CameraRenderService& operator=(const CameraRenderService&) = delete;

    virtual bool initialize(const SimulationConfig& config, const mjModel* model) = 0;
    virtual CameraRenderSubmitResult submit(
        const CameraRenderBatchRequest& request, CameraRenderTicket& ticket) = 0;
    virtual CameraRenderWaitStatus wait(
        const CameraRenderTicket& ticket, std::chrono::milliseconds timeout) = 0;
    virtual CameraRenderWaitStatus query(const CameraRenderTicket& ticket) const = 0;
    virtual bool read_batch_result(
        const CameraRenderTicket& ticket, CameraRenderBatchResult& result) = 0;
    virtual bool reset() = 0;
    virtual bool shutdown() = 0;

protected:
    CameraRenderService() = default;
};

}  // namespace mujoco_simulation
#include <chrono>
