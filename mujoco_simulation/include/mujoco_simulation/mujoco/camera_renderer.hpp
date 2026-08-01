#pragma once

#include <mujoco/mujoco.h>

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

#include "mujoco_simulation/component/camera/camera_data.hpp"
#include "mujoco_simulation/config/config_limits.hpp"
#include "mujoco_simulation/mujoco/context.hpp"
#include "mujoco_simulation/visibility.hpp"

struct GLFWwindow;

namespace mujoco_simulation {

/// 共享的 MuJoCo 离屏相机渲染器配置。
struct CameraRendererConfig {
  /// MuJoCo 渲染场景可容纳的最大几何体数量。
  int max_scene_geometries{2000};
  /// 可用时优先使用隐藏的 GLFW OpenGL context。
  bool allow_glfw_backend{true};
  /// GLFW 无法创建 OpenGL context 时允许回退至 EGL。
  bool allow_egl_backend{true};
  /// Largest sparse Camera ID accepted by submit().
  ComponentId max_camera_id{SimulationConfigLimits::kMaximumComponentId};
  /// Completed ticket results retained for explicit waiters.
  std::size_t completed_ticket_history{8};
};

/// CameraRenderer 使用的内部离屏 OpenGL 后端。
enum class OffscreenGlBackend {
  None,
  Glfw,
  Egl,
};

/// 已选离屏 OpenGL context 的内部资源记录。
struct OffscreenGlContext {
  OffscreenGlBackend backend{OffscreenGlBackend::None};
  GLFWwindow *window{nullptr};
  void *egl_display{nullptr};
  void *egl_context{nullptr};
  void *egl_surface{nullptr};
};

/// CameraRenderer 输出的紧凑 RGB8 图像。
struct CameraRenderImage {
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::uint32_t step{0};
  std::vector<std::uint8_t> data;
};

/// CameraRenderer 输出的、单位为米的深度图像。
struct CameraRenderDepthImage {
  std::uint32_t width{0};
  std::uint32_t height{0};
  std::vector<float> data;
};

/// 由 MuJoCo 相机视场角推导出的针孔相机内参。
struct CameraRenderIntrinsics {
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};
  Vector9d k{};
  Vector12d p{};
};

/// 单次相机渲染生成的不可变结果。
struct CameraRenderState {
  std::uint64_t sequence{0};
  std::uint64_t timestamp{0};
  std::string frame_id;
  std::string optical_frame_id;
  CameraRenderImage color;
  CameraRenderDepthImage depth;
  CameraRenderIntrinsics intrinsics;
};

struct CameraRenderTask {
  CameraConfig config;
  std::uint64_t sequence{0};
  std::uint64_t timestamp{0};
};

using CameraRenderStatePtr = std::shared_ptr<const CameraRenderState>;
using CameraRenderStates =
    std::shared_ptr<const std::vector<CameraRenderStatePtr>>;

struct CameraRenderTicket {
  std::uint64_t generation{0};
  std::uint64_t sequence{0};

  bool is_noop() const noexcept { return generation == 0 && sequence == 0; }
};

enum class CameraRenderTaskResult {
  Succeeded,
  RenderFailed,
  Superseded,
};

struct CameraRenderTaskStatus {
  CameraId id{kInvalidComponentId};
  CameraRenderTaskResult result{CameraRenderTaskResult::RenderFailed};
  std::string message;
};

struct CameraBatchResult {
  CameraRenderTicket ticket{};
  std::vector<CameraRenderTaskStatus> statuses;
  bool all_succeeded{false};
};

enum class CameraWaitResult {
  Succeeded,
  RenderFailed,
  Superseded,
  Expired,
  InvalidTicket,
  Timeout,
  RendererStopped,
};

class MUJOCO_SIMULATION_PUBLIC CameraRenderer {
public:
  CameraRenderer();
  explicit CameraRenderer(CameraRendererConfig config);
  ~CameraRenderer();

  CameraRenderer(const CameraRenderer &) = delete;
  CameraRenderer &operator=(const CameraRenderer &) = delete;

  /// 创建双缓冲数据并启动专属渲染线程；成功后重复调用不执行额外操作。
  bool initialize(const mjContext &context);
  /// 提交最新相机渲染请求；调用方必须在保护主 mjData 时调用。
  /// Empty task lists return a zero-valued immediately-completed no-op ticket.
  /// A no-op does not access the context and is valid even while this renderer
  /// is uninitialized.
  std::optional<CameraRenderTicket> submit(const mjContext &context,
                                           std::vector<CameraRenderTask> tasks);
  /// 读取所有相机的最新完成结果。
  bool read_results(CameraRenderStates &states) const;
  /// Wait for the exact batch identified by ticket and report why it failed.
  CameraWaitResult wait_result(CameraRenderTicket ticket,
                               CameraBatchResult *result = nullptr);
  /// Compatibility wrapper for callers that only need success/failure.
  bool wait(CameraRenderTicket ticket, CameraBatchResult *result = nullptr);
  /// 停止 worker 并释放全部资源；未初始化时重复调用不执行额外操作。
  bool release();

  bool is_initialized() const noexcept { return initialized_.load(); }
  bool is_running() const noexcept { return running_.load(); }

private:
  using TicketKey = std::pair<std::uint64_t, std::uint64_t>;

  static constexpr auto kWorkerTimeout = std::chrono::seconds(5);
  bool release_locked();
  void worker_loop();
  bool initialize_worker_resources();
  void release_worker_resources();
  bool render_task(const CameraRenderTask &task, CameraRenderStatePtr &out,
                   std::string *error = nullptr);

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

  CameraRenderIntrinsics compute_intrinsics(double fovy_degrees,
                                            std::uint32_t width,
                                            std::uint32_t height) const;
  void transform_rgb(const std::vector<std::uint8_t> &source,
                     std::uint32_t width, std::uint32_t height,
                     std::vector<std::uint8_t> &dest) const;
  bool transform_depth(const std::vector<float> &source, std::uint32_t width,
                       std::uint32_t height, std::vector<float> &dest) const;

  CameraRendererConfig config_{};
  const mjModel *model_{nullptr};
  mutable std::mutex lifecycle_mutex_;
  // Advance at both initialize and release boundaries. This is an invalidation
  // epoch rather than a count of successful worker lifecycles, so waiters are
  // invalidated as soon as release begins.
  std::atomic<std::uint64_t> ticket_epoch_{0};

  mutable std::mutex job_mutex_;
  std::condition_variable job_condition_;
  std::condition_variable completion_condition_;
  std::vector<CameraRenderTask> pending_tasks_;
  bool pending_ready_{false};
  std::uint64_t pending_ticket_{0};
  std::uint64_t submitted_ticket_{0};
  CameraRenderTicket expired_through_ticket_{};
  std::map<TicketKey, CameraBatchResult> completed_results_;
  bool worker_ready_{false};
  bool worker_initialization_failed_{false};
  bool stopping_{false};

  mutable std::mutex result_mutex_;
  CameraRenderStates latest_results_;

  std::thread worker_thread_;
  std::atomic<bool> initialized_{false};
  std::atomic<bool> running_{false};

  OffscreenGlContext gl_context_{};
  mjData *pending_data_{nullptr};
  mjData *render_data_{nullptr};
  mjvScene scene_{};
  mjvOption option_{};
  mjrContext render_context_{};
  int offscreen_width_{0};
  int offscreen_height_{0};
};

} // namespace mujoco_simulation
