#pragma once

#include <mujoco/mujoco.h>

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "mujoco_simulation/mujoco/context.hpp"
#include "mujoco_simulation/visibility.hpp"

namespace mujoco {
class Simulate;
} // namespace mujoco

namespace mujoco_simulation {

// Passive MuJoCo viewer frontend. Simulation owns runtime state and drives
// simulation stepping.
class MUJOCO_SIMULATION_PUBLIC SimulationViewer {
public:
  SimulationViewer();
  explicit SimulationViewer(std::chrono::milliseconds timeout);
  ~SimulationViewer();

  SimulationViewer(const SimulationViewer &) = delete;
  SimulationViewer &operator=(const SimulationViewer &) = delete;
  SimulationViewer(SimulationViewer &&) = delete;
  SimulationViewer &operator=(SimulationViewer &&) = delete;

  bool start(const mjContext &context, const std::string &displayed_filename);
  void stop();
  // Copies the current simulation data into a capacity-one latest-only mailbox.
  // The caller must serialize access to context.data.
  bool submit(const mjContext &context);
  bool is_running() const;
  bool is_ready() const;

private:
  // Viewer 状态
  enum class ViewerState {
    Stopped,  // 已停止
    Starting, // 启动中
    Ready,    // 已就绪
    Stopping, // 停止中
    Failed,   // 失败
  };

  void set_ready();
  void set_failed();
  void finish_render_thread();
  void set_stop();
  void join_render_thread();
  void cleanup();
  void stop_viewer();
  void release_viewer_data();
  bool create_viewer_data(const mjContext &context);
  bool start_sync_worker();
  void stop_sync_worker();
  void sync_worker_loop();
  bool sync_render_data(const mjData &data);
  void render_task(const mjModel *model, mjData *data,
                   std::string displayed_filename);

  std::chrono::milliseconds startup_timeout_{5000};

  // Camera
  mjvCamera camera_{};
  mjvOption visual_options_{};
  mjvPerturb perturb_{};

  // Simulation
  std::unique_ptr<mujoco::Simulate> simulate_;
  // 渲染线程
  std::thread render_thread_;
  std::condition_variable cv_;
  mutable std::mutex lifecycle_mutex_;
  mutable std::mutex mutex_;

  std::mutex sync_mutex_;
  std::condition_variable sync_cv_;
  std::thread sync_thread_;
  bool sync_stopping_{false};
  bool sync_pending_{false};
  mjModel *viewer_model_{nullptr};
  mjData *viewer_data_{nullptr};
  mjData *pending_data_{nullptr};
  mjData *render_data_{nullptr};

  // Viewer state
  ViewerState state_{ViewerState::Stopped};
};

} // namespace mujoco_simulation
