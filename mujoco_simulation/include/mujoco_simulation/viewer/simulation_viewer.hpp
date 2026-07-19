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

#include "mujoco_simulation/common/mj_type.hpp"

namespace mujoco {
class Simulate;
}  // namespace mujoco

namespace mujoco_simulation {

class SimulationViewerTestPeer;

// Passive MuJoCo viewer frontend. Simulation owns runtime state and drives simulation stepping.
class SimulationViewer {
 public:
  SimulationViewer();
  explicit SimulationViewer(std::chrono::milliseconds timeout);
  ~SimulationViewer();

  SimulationViewer(const SimulationViewer&) = delete;
  SimulationViewer& operator=(const SimulationViewer&) = delete;
  SimulationViewer(SimulationViewer&&) = delete;
  SimulationViewer& operator=(SimulationViewer&&) = delete;

  bool start(const mjContext& context, const std::string& displayed_filename);
  void stop();

  bool sync(bool state_only);
  bool is_running() const;
  bool is_ready() const;

 private:
  using SimulateHandle = std::unique_ptr<mujoco::Simulate, void (*)(mujoco::Simulate*)>;
  using RenderThreadEntry =
      std::function<void(SimulationViewer&, mjModel*, mjData*, const std::string&)>;

  friend class SimulationViewerTestPeer;

  static void delete_simulate(mujoco::Simulate* simulate);
  void mark_ready();
  void record_async_failure();

  std::chrono::milliseconds startup_timeout_{5000};
  const mjModel* model_{nullptr};
  mjData* data_{nullptr};
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
  bool async_failed_{false};
};

}  // namespace mujoco_simulation
