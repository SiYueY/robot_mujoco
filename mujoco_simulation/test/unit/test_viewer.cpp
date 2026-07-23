#include <gtest/gtest.h>
#include <sys/wait.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>

#include "mujoco_simulation/runtime/simulation_runtime.hpp"
#include "mujoco_simulation/viewer/simulation_viewer.hpp"

namespace mujoco_simulation {

using namespace std::chrono_literals;

#define ASSERT_OK_STATUS(expr) \
  do {                         \
    ASSERT_TRUE(expr);         \
  } while (false)

class SimulationViewerTestPeer {
 public:
  static void set_render_thread_entry(
      SimulationViewer& viewer,
      std::function<void(SimulationViewer&, mjModel*, mjData*, const std::string&)> entry) {
    viewer.render_thread_entry_ = std::move(entry);
  }

  static void mark_ready(SimulationViewer& viewer) { viewer.mark_ready(); }

  static void inject_async_failure(SimulationViewer& viewer) { viewer.record_async_failure(); }
};

class SimulationRuntimeTestPeer {
 public:
  static const mjContext& context(const SimulationRuntime& runtime) { return runtime.context(); }
};

namespace {

const mjContext& runtime_context(SimulationRuntime& runtime) {
  return SimulationRuntimeTestPeer::context(runtime);
}

class ViewerTest : public ::testing::Test {
 protected:
  void TearDown() override {
    if (!model_path_.empty()) {
      std::error_code error;
      std::filesystem::remove(model_path_, error);
    }
  }

  std::string write_model(const std::string& xml_contents) {
    const auto temp_dir = std::filesystem::temp_directory_path();
    model_path_ =
        temp_dir / std::filesystem::path("viewer_test_" + std::to_string(::getpid()) + ".xml");
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

  std::filesystem::path model_path_;
};

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
    if (viewer.start(runtime_context(runtime), model_path)) {
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

TEST_F(ViewerTest, RestartAndStopRemainSafeOnSingleViewerInstance) {
  if (!viewer_environment_available()) {
    GTEST_SKIP() << "Viewer environment is not available.";
  }

  const std::string model_path = write_model(R"(
<mujoco model="viewer_restart">
  <option timestep="0.01"/>
  <worldbody>
    <geom type="plane" size="5 5 0.1" rgba="0.2 0.2 0.2 1"/>
    <body pos="0 0 0.35">
      <geom type="box" size="0.02 0.5 0.15" rgba="1 0 0 1"/>
    </body>
  </worldbody>
</mujoco>)");

  if (!viewer_start_available(model_path)) {
    GTEST_SKIP() << "Viewer runtime probe failed in a subprocess.";
  }

  SimulationRuntime runtime;
  ASSERT_OK_STATUS(runtime.init({model_path}));

  SimulationViewer viewer;
  if (!viewer.start(runtime_context(runtime), model_path)) {
    GTEST_SKIP() << "viewer start failed";
  }

  EXPECT_TRUE(viewer.is_running());
  EXPECT_TRUE(viewer.is_ready());
  ASSERT_OK_STATUS(viewer.sync(true));

  viewer.stop();
  EXPECT_FALSE(viewer.is_running());
  EXPECT_FALSE(viewer.is_ready());

  EXPECT_FALSE(viewer.sync(false));

  ASSERT_OK_STATUS(viewer.start(runtime_context(runtime), model_path));
  EXPECT_TRUE(viewer.is_running());
  EXPECT_TRUE(viewer.is_ready());
  viewer.stop();
  EXPECT_FALSE(viewer.is_running());
}

TEST_F(ViewerTest, AsyncFailureInjectedAfterStartupIsReturnedBySync) {
  const std::string model_path = write_model(R"(
<mujoco model="viewer_async_failure">
  <option timestep="0.01"/>
  <worldbody>
    <geom type="plane" size="5 5 0.1" rgba="0.2 0.2 0.2 1"/>
  </worldbody>
</mujoco>)");

  SimulationRuntime runtime;
  ASSERT_OK_STATUS(runtime.init({model_path}));

  SimulationViewer viewer;
  SimulationViewerTestPeer::set_render_thread_entry(
      viewer, [](SimulationViewer& viewer, mjModel*, mjData*, const std::string&) {
        SimulationViewerTestPeer::mark_ready(viewer);
        std::this_thread::sleep_for(20ms);
        SimulationViewerTestPeer::inject_async_failure(viewer);
      });

  ASSERT_OK_STATUS(viewer.start(runtime_context(runtime), model_path));
  EXPECT_TRUE(viewer.is_running());
  EXPECT_TRUE(viewer.is_ready());

  bool sync_status = true;
  const auto deadline = std::chrono::steady_clock::now() + 1s;
  while (std::chrono::steady_clock::now() < deadline) {
    sync_status = viewer.sync(true);
    if (!sync_status) {
      break;
    }
    std::this_thread::sleep_for(10ms);
  }

  EXPECT_FALSE(sync_status);

  viewer.stop();
  EXPECT_FALSE(viewer.is_running());
  EXPECT_FALSE(viewer.is_ready());
}

TEST_F(ViewerTest, StartAutomaticallyRestartsAfterRenderThreadFailure) {
  const std::string model_path = write_model(R"(
<mujoco model="viewer_restart_after_failure">
  <worldbody><geom type="plane" size="1 1 0.1"/></worldbody>
</mujoco>)");

  SimulationRuntime runtime;
  ASSERT_OK_STATUS(runtime.init({model_path}));

  std::atomic<int> entries{0};
  std::atomic<bool> release_first{false};
  std::atomic<bool> release_second{false};
  SimulationViewer viewer;
  SimulationViewerTestPeer::set_render_thread_entry(
      viewer, [&](SimulationViewer& target, mjModel*, mjData*, const std::string&) {
        const int entry = ++entries;
        SimulationViewerTestPeer::mark_ready(target);
        const auto& release = entry == 1 ? release_first : release_second;
        while (!release.load()) {
          std::this_thread::sleep_for(1ms);
        }
        SimulationViewerTestPeer::inject_async_failure(target);
      });

  ASSERT_OK_STATUS(viewer.start(runtime_context(runtime), model_path));
  ASSERT_OK_STATUS(viewer.start(runtime_context(runtime), model_path));
  EXPECT_EQ(entries.load(), 1);

  release_first.store(true);
  const auto failure_deadline = std::chrono::steady_clock::now() + 1s;
  while (viewer.is_running() && std::chrono::steady_clock::now() < failure_deadline) {
    std::this_thread::sleep_for(1ms);
  }
  EXPECT_FALSE(viewer.is_running());
  EXPECT_FALSE(viewer.is_ready());

  ASSERT_OK_STATUS(viewer.start(runtime_context(runtime), model_path));
  EXPECT_EQ(entries.load(), 2);
  EXPECT_TRUE(viewer.is_running());
  EXPECT_TRUE(viewer.is_ready());

  release_second.store(true);
  viewer.stop();
  EXPECT_FALSE(viewer.is_running());
  EXPECT_FALSE(viewer.is_ready());
}

#undef ASSERT_OK_STATUS

}  // namespace
}  // namespace mujoco_simulation
