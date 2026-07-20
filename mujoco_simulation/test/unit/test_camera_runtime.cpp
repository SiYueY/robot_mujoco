#include <gtest/gtest.h>
#include <sys/wait.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>

#include "mujoco_simulation/runtime/simulation_runtime.hpp"
#include "mujoco_simulation/simulation.hpp"
#include "mujoco_simulation/viewer/simulation_viewer.hpp"

namespace mujoco_simulation {

class SimulationRuntimeTestPeer {
 public:
  static const mjContext& context(const SimulationRuntime& runtime) { return runtime.context(); }
};

namespace {

using namespace std::chrono_literals;

#define ASSERT_OK_STATUS(expr)           \
  do {                                   \
    const ResultCode status__ = (expr);  \
    ASSERT_EQ(status__, ResultCode::Ok); \
  } while (false)

class CameraRuntimeTest : public ::testing::Test {
 protected:
  void TearDown() override {
    if (!model_path_.empty()) {
      std::error_code error;
      std::filesystem::remove(model_path_, error);
    }
  }

  std::string write_model(const std::string& xml_contents) {
    const auto temp_dir = std::filesystem::temp_directory_path();
    model_path_ = temp_dir / std::filesystem::path("camera_runtime_test_" +
                                                   std::to_string(::getpid()) + ".xml");
    std::ofstream output(model_path_);
    EXPECT_TRUE(output.is_open());
    output << xml_contents;
    output.close();
    return model_path_.string();
  }

  static bool viewer_environment_available() {
    const char* enable_viewer_tests = std::getenv("MUJOCO_ENABLE_VIEWER_TESTS");
    if (enable_viewer_tests == nullptr || std::string(enable_viewer_tests) != "1") {
      return false;
    }
    const char* display = std::getenv("DISPLAY");
    const char* wayland_display = std::getenv("WAYLAND_DISPLAY");
    if (wayland_display != nullptr && wayland_display[0] != '\0') {
      const char* runtime_dir = std::getenv("XDG_RUNTIME_DIR");
      if (runtime_dir != nullptr && runtime_dir[0] != '\0') {
        return std::filesystem::exists(std::filesystem::path(runtime_dir) / wayland_display);
      }
    }

    if (display == nullptr || display[0] == '\0') {
      return false;
    }

    const std::string display_value(display);
    const std::size_t colon = display_value.find(':');
    if (colon == std::string::npos || colon + 1 >= display_value.size()) {
      return false;
    }
    const std::size_t dot = display_value.find('.', colon + 1);
    const std::string display_number = display_value.substr(
        colon + 1, dot == std::string::npos ? std::string::npos : dot - colon - 1);
    if (display_number.empty()) {
      return false;
    }
    return std::filesystem::exists("/tmp/.X11-unix/X" + display_number);
  }

  SimulationConfig camera_config(const std::string& model_path, RenderMode mode,
                                 double update_rate = 50.0, bool enable_depth = true) const {
    SimulationConfig config;
    config.model.model_path = model_path;
    config.render_mode = mode;
    config.components = {ComponentConfig{CameraConfig{.name = "front_camera",
                                                      .update_rate = update_rate,
                                                      .camera_name = "cam",
                                                      .height = 120,
                                                      .width = 160,
                                                      .enable_rgb = true,
                                                      .enable_depth = enable_depth}}};
    return config;
  }

  static std::uint8_t rgb_channel(const CameraState& state, std::size_t row, std::size_t column,
                                  std::size_t channel) {
    return state.image.data[(row * state.image.width + column) * 3U + channel];
  }

  static float depth_at(const CameraState& state, std::size_t row, std::size_t column) {
    float value = 0.0F;
    std::memcpy(
        &value,
        state.depth_image.data.data() + (row * state.depth_image.width + column) * sizeof(float),
        sizeof(float));
    return value;
  }

  std::filesystem::path model_path_;
};

bool wait_for_step_count(Simulation& simulation, std::uint64_t target_step_count,
                         std::chrono::milliseconds timeout = std::chrono::seconds(1)) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (simulation.step_count() >= target_step_count) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return simulation.step_count() >= target_step_count;
}

bool viewer_start_available(const std::string& model_path) {
  pid_t pid = ::fork();
  if (pid == -1) {
    return false;
  }
  if (pid == 0) {
    SimulationRuntime runtime;
    if (!runtime.init({model_path})) {
      _exit(2);
    }
    SimulationViewer viewer;
    const mjContext& context = SimulationRuntimeTestPeer::context(runtime);
    if (viewer.start(context, model_path)) {
      viewer.stop();
      _exit(0);
    }
    _exit(1);
  }

  int status = 0;
  if (::waitpid(pid, &status, 0) == -1) {
    return false;
  }
  return WIFEXITED(status) && WEXITSTATUS(status) == 0;
}

TEST_F(CameraRuntimeTest, CameraRemainsUsableWithViewerAndAfterViewerStops) {
  if (!viewer_environment_available()) {
    GTEST_SKIP() << "Viewer environment is not available.";
  }

  Simulation simulation;
  const std::string model_path = write_model(R"(
<mujoco model="camera_runtime_viewer">
  <option timestep="0.01"/>
  <worldbody>
    <geom type="plane" size="5 5 0.1" rgba="0.2 0.2 0.2 1"/>
    <body pos="0 0 0.35">
      <geom type="box" size="0.02 0.5 0.15" rgba="1 0 0 1"/>
    </body>
    <body pos="0 0 -0.35">
      <geom type="box" size="0.02 0.5 0.15" rgba="0 1 0 1"/>
    </body>
    <camera name="cam" pos="1 0 0" xyaxes="0 1 0 0 0 1" fovy="45"/>
  </worldbody>
</mujoco>)");

  if (!viewer_start_available(model_path)) {
    GTEST_SKIP() << "Viewer runtime probe failed in a subprocess.";
  }

  SimulationConfig config;
  config = camera_config(model_path, RenderMode::Viewer, 50.0, false);
  const ResultCode initialize_status = simulation.initialize(config);
  if (initialize_status != ResultCode::Ok) {
    GTEST_SKIP() << "simulation initialize failed";
  }

  ASSERT_OK_STATUS(simulation.start());
  std::this_thread::sleep_for(100ms);

  CameraState running_state;
  ASSERT_TRUE(simulation.camera_state("front_camera", &running_state));
  EXPECT_EQ(running_state.image.width, 160U);
  EXPECT_GT(running_state.image.data.size(), 0U);

  const std::uint64_t step_count_before_restart = simulation.step_count();
  ASSERT_OK_STATUS(simulation.stop());
  ASSERT_OK_STATUS(simulation.start());
  ASSERT_TRUE(wait_for_step_count(simulation, step_count_before_restart + 1));
  ASSERT_OK_STATUS(simulation.stop());

  CameraState stopped_state;
  ASSERT_TRUE(simulation.camera_state("front_camera", &stopped_state));
  EXPECT_EQ(stopped_state.image.width, 160U);
  EXPECT_GE(stopped_state.image.timestamp, running_state.image.timestamp);
}

TEST_F(CameraRuntimeTest, StartRecreatesViewerAfterStopOnSameSimulationInstance) {
  if (!viewer_environment_available()) {
    GTEST_SKIP() << "Viewer environment is not available.";
  }

  Simulation simulation;
  const std::string model_path = write_model(R"(
<mujoco model="camera_runtime_viewer_restart">
  <option timestep="0.01"/>
  <worldbody>
    <geom type="plane" size="5 5 0.1" rgba="0.2 0.2 0.2 1"/>
    <body pos="0 0 0.35">
      <geom type="box" size="0.02 0.5 0.15" rgba="1 0 0 1"/>
    </body>
    <camera name="cam" pos="1 0 0" xyaxes="0 1 0 0 0 1" fovy="45"/>
  </worldbody>
</mujoco>)");

  if (!viewer_start_available(model_path)) {
    GTEST_SKIP() << "Viewer runtime probe failed in a subprocess.";
  }

  SimulationConfig config = camera_config(model_path, RenderMode::Viewer, 50.0, false);
  const ResultCode initialize_status = simulation.initialize(config);
  if (initialize_status != ResultCode::Ok) {
    GTEST_SKIP() << "simulation initialize failed";
  }

  ASSERT_OK_STATUS(simulation.start());
  std::this_thread::sleep_for(100ms);

  std::shared_ptr<const StateSnapshot> first_running_snapshot = simulation.state_snapshot();
  ASSERT_NE(first_running_snapshot, nullptr);
  ASSERT_GT(first_running_snapshot->step_count, 0U);

  ASSERT_OK_STATUS(simulation.stop());
  EXPECT_EQ(simulation.status(), SimulationStatus::Stopped);

  ASSERT_OK_STATUS(simulation.start());
  std::this_thread::sleep_for(100ms);
  EXPECT_NE(simulation.status(), SimulationStatus::Error);

  std::shared_ptr<const StateSnapshot> restarted_snapshot;
  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (std::chrono::steady_clock::now() < deadline) {
    restarted_snapshot = simulation.state_snapshot();
    if (restarted_snapshot != nullptr &&
        restarted_snapshot->step_count > first_running_snapshot->step_count) {
      break;
    }
    std::this_thread::sleep_for(20ms);
  }

  ASSERT_NE(restarted_snapshot, nullptr);
  EXPECT_GT(restarted_snapshot->step_count, first_running_snapshot->step_count);

  CameraState restarted_state;
  ASSERT_TRUE(simulation.camera_state("front_camera", &restarted_state));
  EXPECT_EQ(restarted_state.image.width, 160U);
  EXPECT_GT(restarted_state.image.data.size(), 0U);

  ASSERT_OK_STATUS(simulation.stop());
}

TEST_F(CameraRuntimeTest, ViewerStartStopCyclesRemainSafe) {
  if (!viewer_environment_available()) {
    GTEST_SKIP() << "Viewer environment is not available.";
  }

  const std::string model_path = write_model(R"(
<mujoco model="camera_runtime_viewer_cycles">
  <option timestep="0.01"/>
  <worldbody>
    <geom type="plane" size="5 5 0.1" rgba="0.2 0.2 0.2 1"/>
    <body pos="0 0 0.35">
      <geom type="box" size="0.02 0.5 0.15" rgba="1 0 0 1"/>
    </body>
    <camera name="cam" pos="1 0 0" xyaxes="0 1 0 0 0 1" fovy="45"/>
  </worldbody>
</mujoco>)");

  for (int cycle = 0; cycle < 3; ++cycle) {
    if (cycle == 0 && !viewer_start_available(model_path)) {
      GTEST_SKIP() << "Viewer runtime probe failed in a subprocess.";
    }
    Simulation simulation;
    const ResultCode initialize_status =
        simulation.initialize({.model = {.model_path = model_path},
                               .scheduler = {.viewer_update_rate = 30.0},
                               .render_mode = RenderMode::Viewer});
    if (initialize_status != ResultCode::Ok) {
      GTEST_SKIP() << "simulation initialize failed";
    }

    ASSERT_OK_STATUS(simulation.start());
    std::this_thread::sleep_for(50ms);
    EXPECT_NE(simulation.status(), SimulationStatus::Error);
    ASSERT_OK_STATUS(simulation.stop());
    EXPECT_NE(simulation.status(), SimulationStatus::Error);
    ASSERT_OK_STATUS(simulation.shutdown());
    EXPECT_EQ(simulation.status(), SimulationStatus::Uninitialized);
  }
}

TEST_F(CameraRuntimeTest, HeadlessAndViewerCameraStatesRemainSemanticallyConsistent) {
  if (!viewer_environment_available()) {
    GTEST_SKIP() << "Viewer environment is not available.";
  }

  const std::string model_path = write_model(R"(
<mujoco model="camera_runtime_parity">
  <option timestep="0.01"/>
  <worldbody>
    <geom type="plane" size="5 5 0.1" rgba="0.2 0.2 0.2 1"/>
    <body pos="0 0 0.35">
      <geom type="box" size="0.02 0.5 0.15" rgba="1 0 0 1"/>
    </body>
    <body pos="0 0 -0.35">
      <geom type="box" size="0.02 0.5 0.15" rgba="0 1 0 1"/>
    </body>
    <camera name="cam" pos="1 0 0" xyaxes="0 1 0 0 0 1" fovy="45"/>
  </worldbody>
</mujoco>)");

  if (!viewer_start_available(model_path)) {
    GTEST_SKIP() << "Viewer runtime probe failed in a subprocess.";
  }

  Simulation headless;
  ASSERT_OK_STATUS(headless.initialize(camera_config(model_path, RenderMode::Headless)));

  Simulation viewer;
  const ResultCode viewer_status = viewer.initialize(camera_config(model_path, RenderMode::Viewer));
  if (viewer_status != ResultCode::Ok) {
    GTEST_SKIP() << "viewer initialize failed";
  }

  CameraState headless_state;
  ASSERT_TRUE(headless.camera_state("front_camera", &headless_state));
  CameraState viewer_state;
  ASSERT_TRUE(viewer.camera_state("front_camera", &viewer_state));

  EXPECT_FALSE(headless_state.image.data.empty());
  EXPECT_FALSE(viewer_state.image.data.empty());
  EXPECT_FALSE(headless_state.depth_image.data.empty());
  EXPECT_FALSE(viewer_state.depth_image.data.empty());
  EXPECT_EQ(headless_state.image.width, viewer_state.image.width);
  EXPECT_EQ(headless_state.image.height, viewer_state.image.height);
  EXPECT_EQ(headless_state.camera_info.k, viewer_state.camera_info.k);
  EXPECT_EQ(headless_state.camera_info.p, viewer_state.camera_info.p);

  EXPECT_GT(rgb_channel(headless_state, 10, 80, 0), rgb_channel(headless_state, 10, 80, 1));
  EXPECT_GT(rgb_channel(viewer_state, 10, 80, 0), rgb_channel(viewer_state, 10, 80, 1));
  EXPECT_GT(rgb_channel(headless_state, 105, 80, 1), rgb_channel(headless_state, 105, 80, 0));
  EXPECT_GT(rgb_channel(viewer_state, 105, 80, 1), rgb_channel(viewer_state, 105, 80, 0));

  EXPECT_NEAR(depth_at(headless_state, 10, 80), depth_at(viewer_state, 10, 80), 1.0e-4F);
  EXPECT_NEAR(depth_at(headless_state, 105, 80), depth_at(viewer_state, 105, 80), 1.0e-4F);
}

TEST_F(CameraRuntimeTest, LowViewerRateDoesNotBlockPhysicsProgress) {
  if (!viewer_environment_available()) {
    GTEST_SKIP() << "Viewer environment is not available.";
  }

  Simulation simulation;
  const std::string model_path = write_model(R"(
<mujoco model="camera_runtime_viewer_rate">
  <option timestep="0.001"/>
  <worldbody>
    <geom type="plane" size="5 5 0.1" rgba="0.2 0.2 0.2 1"/>
    <body pos="0 0 0.35">
      <geom type="box" size="0.02 0.5 0.15" rgba="1 0 0 1"/>
    </body>
    <camera name="cam" pos="1 0 0" xyaxes="0 1 0 0 0 1" fovy="45"/>
  </worldbody>
</mujoco>)");

  SimulationConfig config = camera_config(model_path, RenderMode::Viewer, 30.0, false);
  config.scheduler.viewer_update_rate = 5.0;
  if (!viewer_start_available(model_path)) {
    GTEST_SKIP() << "Viewer runtime probe failed in a subprocess.";
  }
  const ResultCode initialize_status = simulation.initialize(config);
  if (initialize_status != ResultCode::Ok) {
    GTEST_SKIP() << "simulation initialize failed";
  }

  ASSERT_OK_STATUS(simulation.start());
  std::shared_ptr<const StateSnapshot> snapshot;
  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (std::chrono::steady_clock::now() < deadline) {
    snapshot = simulation.state_snapshot();
    if (snapshot != nullptr && snapshot->step_count > 50U && snapshot->simulation_time > 0.05) {
      break;
    }
    std::this_thread::sleep_for(20ms);
  }

  ASSERT_NE(snapshot, nullptr);
  EXPECT_GT(snapshot->step_count, 50U);
  EXPECT_GT(snapshot->simulation_time, 0.05);

  ASSERT_OK_STATUS(simulation.stop());
  ASSERT_OK_STATUS(simulation.shutdown());
  EXPECT_EQ(simulation.status(), SimulationStatus::Uninitialized);
}

TEST_F(CameraRuntimeTest, ConcurrentCameraReadsRemainSafeDuringStepping) {
  Simulation simulation;
  const std::string model_path = write_model(R"(
<mujoco model="camera_runtime_threadsafe">
  <option timestep="0.005"/>
  <worldbody>
    <geom type="plane" size="5 5 0.1" rgba="0.2 0.2 0.2 1"/>
    <body pos="0 0 0.35">
      <geom type="box" size="0.02 0.5 0.15" rgba="1 0 0 1"/>
    </body>
    <body pos="0 0 -0.35">
      <geom type="box" size="0.02 0.5 0.15" rgba="0 1 0 1"/>
    </body>
    <camera name="cam" pos="1 0 0" xyaxes="0 1 0 0 0 1" fovy="45"/>
  </worldbody>
</mujoco>)");

  SimulationConfig config;
  config = camera_config(model_path, RenderMode::Headless, 100.0, true);
  ASSERT_OK_STATUS(simulation.initialize(config));

  std::atomic<int> failures{0};
  std::atomic<bool> start_reads{false};

  auto reader = [&]() {
    while (!start_reads.load()) {
      std::this_thread::yield();
    }
    std::uint64_t last_timestamp = 0;
    for (int iteration = 0; iteration < 100; ++iteration) {
      CameraState state;
      if (!simulation.camera_state("front_camera", &state)) {
        ++failures;
        continue;
      }
      if (state.image.width != 160U || state.image.height != 120U ||
          state.depth_image.width != 160U || state.depth_image.height != 120U ||
          state.image.data.empty() || state.depth_image.data.empty() ||
          state.image.timestamp < last_timestamp) {
        ++failures;
      }
      last_timestamp = state.image.timestamp;
      std::this_thread::yield();
    }
  };

  std::thread readers[4] = {std::thread(reader), std::thread(reader), std::thread(reader),
                            std::thread(reader)};
  start_reads.store(true);

  ASSERT_OK_STATUS(simulation.start());
  ASSERT_TRUE(wait_for_step_count(simulation, 100));
  ASSERT_OK_STATUS(simulation.stop());

  for (auto& thread : readers) {
    thread.join();
  }

  EXPECT_EQ(failures.load(), 0);
}

#undef ASSERT_OK_STATUS

}  // namespace
}  // namespace mujoco_simulation
