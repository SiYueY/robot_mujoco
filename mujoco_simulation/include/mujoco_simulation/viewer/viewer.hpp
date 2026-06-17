#pragma once

#include <mujoco/mujoco.h>

#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "mujoco_simulation/status.hpp"
#include "mujoco_simulation/viewer/viewer_config.hpp"

namespace mujoco {
class Simulate;
}  // namespace mujoco

namespace mujoco_simulation {

class ViewerTestPeer;

// Passive MuJoCo viewer frontend. Simulation owns model/data and
// drives simulation stepping; Viewer only renders borrowed state.
class Viewer {
 public:
  Viewer();
  explicit Viewer(ViewerConfig config);
  ~Viewer();

  Viewer(const Viewer&) = delete;
  Viewer& operator=(const Viewer&) = delete;
  Viewer(Viewer&&) = delete;
  Viewer& operator=(Viewer&&) = delete;

  Status start(mjModel* model, mjData* data, const std::string& displayed_filename);
  void stop();

  Status sync(bool state_only);
  bool is_running() const;
  bool is_ready() const;

 private:
  using SimulateHandle = std::unique_ptr<mujoco::Simulate, void (*)(mujoco::Simulate*)>;
  using RenderThreadEntry = std::function<void(Viewer&, mjModel*, mjData*, const std::string&)>;

  friend class ViewerTestPeer;

  void mark_ready();
  void record_async_failure(Status status);

  ViewerConfig config_{};
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
  std::optional<Status> async_failure_;
};

}  // namespace mujoco_simulation
