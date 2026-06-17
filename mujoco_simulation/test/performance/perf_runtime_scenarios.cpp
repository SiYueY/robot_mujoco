#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <numeric>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "mujoco_simulation/simulation.hpp"

namespace mujoco_simulation {
namespace {

using Clock = std::chrono::steady_clock;

struct TempModelFile {
  explicit TempModelFile(std::string xml_contents) {
    path = std::filesystem::temp_directory_path() /
           std::filesystem::path("mujoco_perf_" + std::to_string(::getpid()) + "_" +
                                 std::to_string(counter_++) + ".xml");
    std::ofstream output(path);
    output << xml_contents;
  }

  ~TempModelFile() {
    if (!path.empty()) {
      std::error_code error;
      std::filesystem::remove(path, error);
    }
  }

  std::filesystem::path path;

 private:
  inline static std::uint64_t counter_{0};
};

double mean(const std::vector<double>& values) {
  if (values.empty()) {
    return 0.0;
  }
  return std::accumulate(values.begin(), values.end(), 0.0) / static_cast<double>(values.size());
}

double p95(std::vector<double> values) {
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  const std::size_t index =
      std::min(values.size() - 1, static_cast<std::size_t>(std::ceil(values.size() * 0.95) - 1));
  return values[index];
}

int run_scheduler_1khz(std::uint64_t target_steps) {
  TempModelFile model(R"(
<mujoco model="perf_scheduler">
  <option timestep="0.001" gravity="0 0 0"/>
  <worldbody>
    <body>
      <joint name="hinge" type="hinge" axis="0 0 1" damping="1.0" limited="true" range="-3.14 3.14"/>
      <geom type="capsule" size="0.05 0.2" density="100"/>
    </body>
  </worldbody>
  <actuator>
    <velocity name="hinge_vel" joint="hinge"/>
  </actuator>
</mujoco>)");

  Simulation simulation;
  SimulationConfig config;
  config.model.model_path = model.path.string();
  config.scheduler.realtime_factor = 1.0;
  const Status initialize_status = simulation.initialize(config);
  if (!initialize_status.ok()) {
    std::cerr << initialize_status.message() << '\n';
    return 1;
  }
  const Status start_status = simulation.start();
  if (!start_status.ok()) {
    std::cerr << start_status.message() << '\n';
    return 1;
  }

  std::vector<double> per_step_wall_ms;
  std::optional<Clock::time_point> first_wall;
  std::optional<Clock::time_point> last_wall;
  double first_sim_time = 0.0;
  double last_sim_time = 0.0;
  std::uint64_t last_step = 0;

  while (last_step < target_steps) {
    const auto snapshot = simulation.state_snapshot();
    if (snapshot != nullptr && snapshot->step_count > last_step) {
      const auto now = Clock::now();
      if (!first_wall.has_value()) {
        first_wall = now;
        first_sim_time = snapshot->simulation_time;
      } else if (last_wall.has_value()) {
        const auto step_delta = snapshot->step_count - last_step;
        if (step_delta > 0) {
          const auto wall_delta_ms = std::chrono::duration<double, std::milli>(now - *last_wall);
          per_step_wall_ms.push_back(wall_delta_ms.count() / static_cast<double>(step_delta));
        }
      }
      last_wall = now;
      last_step = snapshot->step_count;
      last_sim_time = snapshot->simulation_time;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  const Status stop_status = simulation.stop();
  if (!stop_status.ok()) {
    std::cerr << stop_status.message() << '\n';
    return 1;
  }

  const double wall_seconds = first_wall.has_value() && last_wall.has_value()
                                  ? std::chrono::duration<double>(*last_wall - *first_wall).count()
                                  : 0.0;
  const double sim_seconds = last_sim_time - first_sim_time;
  const double step_throughput_hz =
      wall_seconds > 0.0 ? static_cast<double>(last_step) / wall_seconds : 0.0;
  const double realtime_factor = wall_seconds > 0.0 ? sim_seconds / wall_seconds : 0.0;

  std::cout << "scenario=scheduler_1khz\n";
  std::cout << "steps=" << last_step << '\n';
  std::cout << "wall_seconds=" << wall_seconds << '\n';
  std::cout << "sim_seconds=" << sim_seconds << '\n';
  std::cout << "throughput_hz=" << step_throughput_hz << '\n';
  std::cout << "realtime_factor=" << realtime_factor << '\n';
  std::cout << "jitter_ms_mean=" << mean(per_step_wall_ms) << '\n';
  std::cout << "jitter_ms_p95=" << p95(per_step_wall_ms) << '\n';
  return 0;
}

int run_headless_camera(std::uint64_t total_steps) {
  TempModelFile model(R"(
<mujoco model="perf_camera">
  <option timestep="0.005" gravity="0 0 0"/>
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

  Simulation simulation;
  SimulationConfig config;
  config.model.model_path = model.path.string();
  config.render_mode = RenderMode::Headless;
  config.components = {
      ComponentConfig{CameraConfig{.common = {.name = "front_camera", .update_rate = 50.0},
                                   .camera_name = "cam",
                                   .height = 120,
                                   .width = 160,
                                   .enable_rgb = true,
                                   .enable_depth = true}}};
  const Status initialize_status = simulation.initialize(config);
  if (!initialize_status.ok()) {
    std::cerr << initialize_status.message() << '\n';
    return 1;
  }

  std::vector<double> sample_latency_ms;
  std::uint64_t last_sequence = 0;
  std::uint64_t collected_samples = 0;
  std::uint64_t last_timestamp_ns = 0;
  auto wall_start = Clock::now();

  for (std::uint64_t step = 0; step < total_steps; ++step) {
    const auto before_step = Clock::now();
    const Status step_status = simulation.step(1);
    if (!step_status.ok()) {
      std::cerr << step_status.message() << '\n';
      return 1;
    }
    const auto sample = simulation.camera_sample("front_camera");
    if (!sample.ok() || sample.value() == nullptr) {
      std::cerr << (sample.ok() ? "camera sample unavailable" : sample.status().message()) << '\n';
      return 1;
    }
    if (sample.value()->sequence > last_sequence) {
      const auto after_read = Clock::now();
      sample_latency_ms.push_back(
          std::chrono::duration<double, std::milli>(after_read - before_step).count());
      last_sequence = sample.value()->sequence;
      last_timestamp_ns = sample.value()->timestamp_ns;
      ++collected_samples;
    }
  }
  const auto wall_end = Clock::now();
  const double wall_seconds = std::chrono::duration<double>(wall_end - wall_start).count();
  const double sim_seconds = static_cast<double>(last_timestamp_ns) / 1e9;
  const double sample_frequency_hz =
      sim_seconds > 0.0 ? static_cast<double>(collected_samples) / sim_seconds : 0.0;

  std::cout << "scenario=headless_camera\n";
  std::cout << "steps=" << total_steps << '\n';
  std::cout << "samples=" << collected_samples << '\n';
  std::cout << "wall_seconds=" << wall_seconds << '\n';
  std::cout << "sim_seconds=" << sim_seconds << '\n';
  std::cout << "sample_frequency_hz=" << sample_frequency_hz << '\n';
  std::cout << "sample_latency_ms_mean=" << mean(sample_latency_ms) << '\n';
  std::cout << "sample_latency_ms_p95=" << p95(sample_latency_ms) << '\n';
  return 0;
}

}  // namespace
}  // namespace mujoco_simulation

int main(int argc, char** argv) {
  std::string scenario = "scheduler_1khz";
  std::uint64_t steps = 2000;

  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--scenario" && index + 1 < argc) {
      scenario = argv[++index];
    } else if (argument == "--steps" && index + 1 < argc) {
      steps = static_cast<std::uint64_t>(std::stoull(argv[++index]));
    } else {
      std::cerr << "Unknown argument: " << argument << '\n';
      return 2;
    }
  }

  if (scenario == "scheduler_1khz") {
    return mujoco_simulation::run_scheduler_1khz(steps);
  }
  if (scenario == "headless_camera") {
    return mujoco_simulation::run_headless_camera(steps);
  }

  std::cerr << "Unknown scenario: " << scenario << '\n';
  return 2;
}
