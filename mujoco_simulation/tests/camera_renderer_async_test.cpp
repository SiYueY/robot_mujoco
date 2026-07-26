#include <mujoco/mujoco.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "mujoco_simulation/mujoco/camera_renderer.hpp"

namespace {

bool write_model(const std::filesystem::path &path) {
  std::ofstream output(path);
  if (!output.is_open()) {
    return false;
  }
  output << R"(<mujoco model="camera_async_test">
  <worldbody>
    <light pos="0 0 3"/>
    <geom type="box" size="0.2 0.2 0.2" rgba="0.8 0.2 0.2 1"/>
    <camera name="test_camera" pos="0 -2 0.5" xyaxes="1 0 0 0 0 1"/>
  </worldbody>
</mujoco>)";
  return output.good();
}

bool check(bool value, const char *message) {
  if (!value) {
    std::cerr << message << '\n';
  }
  return value;
}

} // namespace

int main() {
  const std::filesystem::path model_path =
      std::filesystem::temp_directory_path() / "mujoco_camera_async_test.xml";
  if (!check(write_model(model_path), "failed to write MuJoCo test model")) {
    return 1;
  }

  char error[1024] = {};
  mjModel *model =
      mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error));
  std::filesystem::remove(model_path);
  if (!check(model != nullptr, error)) {
    return 1;
  }
  mjData *data = mj_makeData(model);
  if (!check(data != nullptr, "failed to allocate MuJoCo test data")) {
    mj_deleteModel(model);
    return 1;
  }
  mj_forward(model, data);

  mujoco_simulation::mjContext context(model, data);
  bool released = false;
  {
    mujoco_simulation::CameraRendererConfig renderer_config;
    renderer_config.allow_glfw_backend = false;
    renderer_config.allow_egl_backend = true;
    mujoco_simulation::CameraRenderer renderer(renderer_config);
    if (!check(renderer.initialize(context),
               "failed to initialize camera worker")) {
      return 1;
    }

    mujoco_simulation::CameraRenderTask task;
    task.config.id = 2;
    task.config.name = "camera";
    task.config.camera_name = "test_camera";
    task.config.width = 32;
    task.config.height = 24;
    task.config.enable_rgb = true;
    task.sequence = 1;
    task.timestamp = 100;
    if (!check(renderer.submit(context, {task}),
               "failed to submit first frame") ||
        !check(renderer.wait_for_submitted_results(),
               "first frame did not finish")) {
      renderer.release();
      return 1;
    }

    task.sequence = 2;
    task.timestamp = 200;
    if (!check(renderer.submit(context, {task}),
               "failed to submit intermediate frame")) {
      renderer.release();
      return 1;
    }
    task.sequence = 3;
    task.timestamp = 300;
    if (!check(renderer.submit(context, {task}),
               "failed to submit latest frame") ||
        !check(renderer.wait_for_submitted_results(),
               "latest frame did not finish")) {
      renderer.release();
      return 1;
    }

    mujoco_simulation::CameraRenderStates results;
    if (!check(renderer.read_results(results) && results != nullptr,
               "camera worker did not publish a result")) {
      renderer.release();
      return 1;
    }
    if (!check(results->size() > task.config.id &&
                   (*results)[task.config.id] != nullptr,
               "camera result is missing") ||
        !check((*results)[task.config.id]->sequence == 3,
               "latest result sequence is wrong") ||
        !check((*results)[task.config.id]->timestamp == 300,
               "latest result timestamp is wrong") ||
        !check(!(*results)[task.config.id]->color.data.empty(),
               "RGB image is empty")) {
      renderer.release();
      return 1;
    }

    released = renderer.release();
  }
  context.clear();
  return released ? 0 : 1;
}
