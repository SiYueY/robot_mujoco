#include <mujoco/mujoco.h>

#include <atomic>
#include <iostream>
#include <mutex>
#include <thread>

#include "render/camera_renderer.hpp"
#include "test_support.hpp"

namespace {

bool check(bool value, const char *message) {
  if (!value) {
    std::cerr << message << '\n';
  }
  return value;
}

} // namespace

int main() {
  mujoco_simulation_test::TemporaryFile model_file(
      "mujoco_camera_snapshot_tsan_test.xml");
  if (!check(model_file.write(R"(<mujoco model="camera_snapshot_tsan_test">
  <worldbody>
    <light pos="0 0 3"/>
    <body><joint type="slide" axis="1 0 0"/>
      <geom type="box" size="0.2 0.2 0.2" rgba="0.8 0.2 0.2 1"/>
    </body>
    <camera name="test_camera" pos="0 -2 0.5" xyaxes="1 0 0 0 0 1"/>
  </worldbody>
</mujoco>)"),
             "failed to write MuJoCo model")) {
    return 1;
  }

  char error[1024] = {};
  mjModel *model =
      mj_loadXML(model_file.path().c_str(), nullptr, error, sizeof(error));
  if (!check(model != nullptr, error)) {
    return 1;
  }
  mjData *data = mj_makeData(model);
  if (!check(data != nullptr, "failed to allocate MuJoCo data")) {
    mj_deleteModel(model);
    return 1;
  }
  mj_forward(model, data);

  mujoco_simulation::mjContext context(model, data);
  mujoco_simulation::CameraRendererConfig renderer_config;
  renderer_config.allow_glfw_backend = false;
  renderer_config.allow_egl_backend = true;
  renderer_config.max_camera_id = 4;
  mujoco_simulation::CameraRenderer renderer(renderer_config);
  if (!check(renderer.initialize(context), "failed to initialize renderer")) {
    context.clear();
    return 1;
  }

  mujoco_simulation::CameraRenderTask task;
  task.camera_id = 2;
  task.config.id = 2;
  task.config.name = "camera";
  task.config.camera_name = "test_camera";
  task.config.width = 64;
  task.config.height = 48;
  task.config.enable_rgb = true;

  std::mutex live_data_mutex;
  std::atomic<bool> submit_failed{false};
  std::thread physics_writer([&] {
    for (int index = 0; index < 120; ++index) {
      std::lock_guard<std::mutex> lock(live_data_mutex);
      data->qpos[0] = (index & 1) == 0 ? -0.15 : 0.15;
      mj_forward(model, data);
    }
  });
  std::thread camera_submitter([&] {
    for (std::uint64_t index = 0; index < 48; ++index) {
      mujoco_simulation::CameraRenderBatchRequest request;
      request.generation = 1;
      request.sequence = index + 1U;
      request.simulation_step = index + 1U;
      request.model = model;
      task.sequence = index + 1U;
      request.tasks = {task};
      std::lock_guard<std::mutex> lock(live_data_mutex);
      request.data = data;
      request.simulation_time = data->time;
      if (!renderer.submit(request).has_value()) {
        submit_failed.store(true, std::memory_order_release);
      }
    }
  });
  physics_writer.join();
  camera_submitter.join();

  const bool success = !submit_failed.load(std::memory_order_acquire) &&
                       renderer.snapshot_copy_count() == 48U &&
                       renderer.release();
  context.clear();
  return success ? 0 : 1;
}
