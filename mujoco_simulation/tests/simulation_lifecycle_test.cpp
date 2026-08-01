#include <chrono>
#include <filesystem>
#include <iostream>

#include "mujoco_simulation/simulation.hpp"
#include "test_support.hpp"

namespace {

bool check(bool value, const char *message) {
  if (!value)
    std::cerr << message << '\n';
  return value;
}

bool wait_for_steps(mujoco_simulation::Simulation &simulation,
                    std::uint64_t minimum) {
  return mujoco_simulation_test::wait_until(
      [&] { return simulation.step_count() >= minimum; },
      std::chrono::seconds(1));
}

} // namespace

int main() {
  mujoco_simulation_test::TemporaryFile model(
      "mujoco_simulation_lifecycle_test.xml");
  if (!check(model.write(R"(<mujoco model="lifecycle_test">
  <worldbody><geom type="plane" size="1 1 .1"/></worldbody>
  <keyframe><key name="home"/></keyframe>
</mujoco>)"),
             "failed to write lifecycle model"))
    return 1;

  mujoco_simulation::SimulationConfig config;
  config.model.model_path = model.path().string();
  config.scheduler.physics_period = 0.001;
  config.scheduler.viewer_period = 0.02;
  config.viewer_enabled = false;
  config.viewer_startup_timeout = std::chrono::milliseconds(1);
  mujoco_simulation::Simulation simulation;

  if (!check(simulation.initialize(config),
             "simulation initialization failed") ||
      !check(simulation.status() ==
                 mujoco_simulation::SimulationStatus::Stopped,
             "initialized simulation was not stopped")) {
    return 1;
  }

  std::shared_ptr<const mujoco_simulation::RobotState> state;
  if (!check(simulation.read_state(state) && state != nullptr,
             "initial state was not readable") ||
      !check(simulation.start(), "simulation start failed") ||
      !check(wait_for_steps(simulation, 2), "simulation did not step") ||
      !check(simulation.pause(), "simulation pause failed") ||
      !check(simulation.status() == mujoco_simulation::SimulationStatus::Paused,
             "simulation was not paused") ||
      !check(simulation.reset(), "paused simulation reset failed") ||
      !check(simulation.status() == mujoco_simulation::SimulationStatus::Paused,
             "reset did not restore paused status") ||
      !check(simulation.resume(), "simulation resume failed") ||
      !check(simulation.stop(), "simulation stop failed") ||
      !check(simulation.status() ==
                 mujoco_simulation::SimulationStatus::Stopped,
             "simulation was not stopped") ||
      !check(simulation.read_state(state) && state != nullptr,
             "state was not readable after stop") ||
      !check(simulation.reset("home"), "stopped keyframe reset failed") ||
      !check(!simulation.reset("missing"), "unknown keyframe was accepted") ||
      !check(simulation.status() == mujoco_simulation::SimulationStatus::Error,
             "failed keyframe reset did not enter error state") ||
      !check(simulation.read_state(state) && state != nullptr,
             "failed keyframe reset did not retain diagnostic state") ||
      !check(simulation.stop(), "error-state simulation stop failed") ||
      !check(simulation.status() == mujoco_simulation::SimulationStatus::Error,
             "stop incorrectly cleared the simulation error state") ||
      !check(!simulation.start(),
             "error-state simulation started without a reset") ||
      !check(simulation.reset("home"), "error recovery reset failed") ||
      !check(simulation.status() ==
                 mujoco_simulation::SimulationStatus::Stopped,
             "error recovery reset did not restore stopped status") ||
      !check(simulation.read_state(state) && state != nullptr,
             "error recovery reset did not publish state") ||
      !check(simulation.start(), "simulation restart failed") ||
      !check(wait_for_steps(simulation, 3),
             "restarted simulation did not step") ||
      !check(simulation.stop(), "second simulation stop failed") ||
      !check(simulation.shutdown(), "simulation shutdown failed") ||
      !check(simulation.status() ==
                 mujoco_simulation::SimulationStatus::Uninitialized,
             "shutdown simulation was not uninitialized") ||
      !check(!simulation.read_state(state),
             "shutdown state was still readable")) {
    return 1;
  }
  return 0;
}
