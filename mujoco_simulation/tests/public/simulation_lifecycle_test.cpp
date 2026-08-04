#include <chrono>
#include <filesystem>
#include <iostream>

#include "mujoco_simulation/simulation.hpp"
#include "test_support.hpp"

namespace {

bool check(bool value, const char* message) {
    if (!value) std::cerr << message << '\n';
    return value;
}

bool wait_for_steps(mujoco_simulation::Simulation& simulation, std::uint64_t minimum) {
    return mujoco_simulation_test::wait_until(
        [&] { return simulation.step_count() >= minimum; }, std::chrono::seconds(1));
}

bool wait_for_camera_frame(
    mujoco_simulation::Simulation& simulation, std::uint64_t minimum_sequence,
    std::uint64_t* sequence = nullptr) {
    return mujoco_simulation_test::wait_until(
        [&] {
            std::shared_ptr<const mujoco_simulation::RobotState> state;
            if (!simulation.read_state(state) || state == nullptr || state->cameras == nullptr ||
                state->cameras->empty() || (*state->cameras)[0] == nullptr ||
                (*state->cameras)[0]->id != 2U) {
                return false;
            }
            const std::uint64_t observed = (*state->cameras)[0]->sequence;
            if (sequence != nullptr) *sequence = observed;
            return observed >= minimum_sequence;
        },
        std::chrono::seconds(2));
}

}  // namespace

int main() {
    mujoco_simulation_test::TemporaryFile model("mujoco_simulation_lifecycle_test.xml");
    if (!check(
            model.write(R"(<mujoco model="lifecycle_test">
  <worldbody>
    <light pos="0 0 3"/>
    <geom type="plane" size="1 1 .1"/>
    <camera name="test_camera" pos="0 -2 0.5" xyaxes="1 0 0 0 0 1"/>
  </worldbody>
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
    config.camera_renderer.allow_glfw_backend = false;
    config.camera_renderer.allow_egl_backend = true;
    mujoco_simulation::CameraConfig camera;
    camera.id = 2;
    camera.name = "camera";
    camera.frame_id = "camera_frame";
    camera.optical_frame_id = "camera_optical_frame";
    camera.camera_name = "test_camera";
    camera.width = 32;
    camera.height = 24;
    camera.period = 0.001;
    config.components.push_back(camera);
    mujoco_simulation::Simulation simulation;

    if (!check(simulation.initialize(config), "simulation initialization failed") ||
        !check(
            simulation.status() == mujoco_simulation::SimulationStatus::Stopped,
            "initialized simulation was not stopped") ||
        !check(
            simulation.write_command(mujoco_simulation::RobotCommand{}),
            "public empty robot command was rejected")) {
        return 1;
    }

    std::shared_ptr<const mujoco_simulation::RobotState> state;
    mujoco_simulation::CameraState camera_state;
    camera_state.id = camera.id;
    std::uint64_t before_reset_camera_sequence = 0;
    std::uint64_t after_reset_camera_sequence = 0;
    if (!check(
            simulation.read_state(state) && state != nullptr, "initial state was not readable") ||
        !check(simulation.step(), "public manual simulation step failed") ||
        !check(simulation.step_count() == 1, "public manual simulation step was not counted") ||
        !check(simulation.start(), "simulation start failed") ||
        !check(wait_for_steps(simulation, 2), "simulation did not step") ||
        !check(
            wait_for_camera_frame(simulation, 2, &before_reset_camera_sequence),
            "camera frame was not published before reset") ||
        !check(
            simulation.read_state(camera_state) && camera_state.id == camera.id &&
                camera_state.sequence >= 2U,
            "state lookup by embedded camera id failed") ||
        !check(simulation.pause(), "simulation pause failed") ||
        !check(
            simulation.status() == mujoco_simulation::SimulationStatus::Paused,
            "simulation was not paused") ||
        !check(simulation.reset(), "paused simulation reset failed") ||
        !check(
            simulation.status() == mujoco_simulation::SimulationStatus::Paused,
            "reset did not restore paused status") ||
        !check(
            wait_for_camera_frame(simulation, 1, &after_reset_camera_sequence) &&
                after_reset_camera_sequence == 1,
            "pre-reset camera batch leaked into reset state") ||
        !check(simulation.resume(), "simulation resume failed") ||
        !check(simulation.stop(), "simulation stop failed") ||
        !check(
            simulation.status() == mujoco_simulation::SimulationStatus::Stopped,
            "simulation was not stopped") ||
        !check(
            simulation.read_state(state) && state != nullptr,
            "state was not readable after stop") ||
        !check(simulation.reset("home"), "stopped keyframe reset failed") ||
        !check(!simulation.reset("missing"), "unknown keyframe was accepted") ||
        !check(
            simulation.status() == mujoco_simulation::SimulationStatus::Error,
            "failed keyframe reset did not enter error state") ||
        !check(
            simulation.read_state(state) && state != nullptr,
            "failed keyframe reset did not retain diagnostic state") ||
        !check(simulation.stop(), "error-state simulation stop failed") ||
        !check(
            simulation.status() == mujoco_simulation::SimulationStatus::Error,
            "stop incorrectly cleared the simulation error state") ||
        !check(!simulation.start(), "error-state simulation started without a reset") ||
        !check(simulation.reset("home"), "error recovery reset failed") ||
        !check(
            simulation.status() == mujoco_simulation::SimulationStatus::Stopped,
            "error recovery reset did not restore stopped status") ||
        !check(
            simulation.read_state(state) && state != nullptr,
            "error recovery reset did not publish state") ||
        !check(simulation.start(), "simulation restart failed") ||
        !check(wait_for_steps(simulation, 3), "restarted simulation did not step") ||
        !check(simulation.stop(), "second simulation stop failed") ||
        !check(simulation.shutdown(), "simulation shutdown failed") ||
        !check(
            simulation.status() == mujoco_simulation::SimulationStatus::Uninitialized,
            "shutdown simulation was not uninitialized") ||
        !check(!simulation.read_state(state), "shutdown state was still readable")) {
        return 1;
    }
    return 0;
}
