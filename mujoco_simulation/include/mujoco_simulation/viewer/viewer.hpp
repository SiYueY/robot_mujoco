#pragma once

#include <mujoco/mujoco.h>

#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace mujoco {
class Simulate;
}  // namespace mujoco

namespace mujoco_simulation {

// Passive MuJoCo viewer frontend. MuJoCoSimulation owns model/data and
// drives simulation stepping; Viewer only renders borrowed state.
class Viewer {
 public:
  Viewer();
  ~Viewer();

  Viewer(const Viewer&) = delete;
  Viewer& operator=(const Viewer&) = delete;
  Viewer(Viewer&&) = delete;
  Viewer& operator=(Viewer&&) = delete;

  bool start(mjModel* model, mjData* data, const std::string& displayed_filename,
             std::string& error_message);
  void stop();

  bool sync(bool state_only, std::string& error_message);
  mjvScene* scene();
  mjrContext* render_context();
  bool is_running() const;

 private:
  using SimulateHandle = std::unique_ptr<mujoco::Simulate, void (*)(mujoco::Simulate*)>;

  mjvCamera camera_{};
  mjvOption visual_options_{};
  mjvPerturb perturb_{};
  SimulateHandle simulate_;
  std::thread render_thread_;
  mutable std::mutex mutex_;
};

}  // namespace mujoco_simulation
