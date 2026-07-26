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

  bool sync(bool state_only);
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
  void render_task(const mjContext &context, std::string displayed_filename);

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

  // Viewer state
  ViewerState state_{ViewerState::Stopped};
};

} // namespace mujoco_simulation
