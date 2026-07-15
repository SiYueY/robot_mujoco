#pragma once

#include <mujoco/mujoco.h>

#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "mujoco_simulation/result_code.hpp"
#include "mujoco_simulation/runtime/model_runtime.hpp"
#include "mujoco_simulation/simulation_status.hpp"

namespace mujoco {
class Simulate;
}  // namespace mujoco

namespace mujoco_simulation {

class MuJoCoViewerTestPeer;

// Passive MuJoCo viewer frontend. Simulation owns runtime state and
// drives simulation stepping; MuJoCoViewer only renders through a controlled runtime handle.
class MuJoCoViewer {
 public:
  MuJoCoViewer();
  explicit MuJoCoViewer(std::chrono::milliseconds startup_timeout);
  ~MuJoCoViewer();

  MuJoCoViewer(const MuJoCoViewer&) = delete;
  MuJoCoViewer& operator=(const MuJoCoViewer&) = delete;
  MuJoCoViewer(MuJoCoViewer&&) = delete;
  MuJoCoViewer& operator=(MuJoCoViewer&&) = delete;

  ResultCode start(const ViewerRuntimeHandle& runtime_handle,
                   const std::string& displayed_filename);
  void stop();

  ResultCode sync(bool state_only);
  bool is_running() const;
  bool is_ready() const;

 private:
  using SimulateHandle = std::unique_ptr<mujoco::Simulate, void (*)(mujoco::Simulate*)>;
  using RenderThreadEntry =
      std::function<void(MuJoCoViewer&, mjModel*, mjData*, const std::string&)>;

  friend class MuJoCoViewerTestPeer;

  void mark_ready();
  void record_async_failure(ResultCode status);

  std::chrono::milliseconds startup_timeout_{5000};
  ViewerRuntimeHandle runtime_handle_{};
  mjvCamera camera_{};
  mjvOption visual_options_{};
  mjvPerturb perturb_{};
  SimulateHandle simulate_;
  RenderThreadEntry render_thread_entry_;
  std::thread render_thread_;
  std::condition_variable cv_;
  mutable std::mutex mutex_;
  bool ready_{false};
  bool stop_requested_{false};
  std::optional<ResultCode> async_failure_;
};

}  // namespace mujoco_simulation
