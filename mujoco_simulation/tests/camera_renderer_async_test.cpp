#include <mujoco/mujoco.h>

#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include "mujoco_simulation/mujoco/camera_renderer.hpp"
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
      "mujoco_camera_async_test.xml");
  if (!check(model_file.write(R"(<mujoco model="camera_async_test">
  <worldbody>
    <light pos="0 0 3"/>
    <geom type="box" size="0.2 0.2 0.2" rgba="0.8 0.2 0.2 1"/>
    <camera name="test_camera" pos="0 -2 0.5" xyaxes="1 0 0 0 0 1"/>
  </worldbody>
</mujoco>)"),
             "failed to write MuJoCo test model")) {
    return 1;
  }

  char error[1024] = {};
  mjModel *model =
      mj_loadXML(model_file.path().c_str(), nullptr, error, sizeof(error));
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
    renderer_config.max_camera_id = 4;
    mujoco_simulation::CameraRenderer renderer(renderer_config);
    if (!check(!renderer.wait({1}),
               "renderer accepted a ticket it did not submit")) {
      return 1;
    }
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
    const auto first_ticket = renderer.submit(context, {task});
    if (!check(first_ticket.has_value(), "failed to submit first frame") ||
        !check(renderer.wait(*first_ticket), "first frame did not finish")) {
      renderer.release();
      return 1;
    }

    task.sequence = 2;
    task.timestamp = 200;
    const auto intermediate_ticket = renderer.submit(context, {task});
    if (!check(intermediate_ticket.has_value(),
               "failed to submit intermediate frame")) {
      renderer.release();
      return 1;
    }
    task.sequence = 3;
    task.timestamp = 300;
    const auto latest_ticket = renderer.submit(context, {task});
    if (!check(latest_ticket.has_value(), "failed to submit latest frame") ||
        !check(renderer.wait(*latest_ticket), "latest frame did not finish")) {
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

    // The intermediate request may either have rendered before the later
    // submit or have been superseded. In both cases wait() must return the
    // result belonging to that exact ticket, never the latest batch result.
    mujoco_simulation::CameraBatchResult intermediate_result;
    const bool intermediate_succeeded =
        renderer.wait(*intermediate_ticket, &intermediate_result);
    if (!check(intermediate_result.ticket == intermediate_ticket->value,
               "intermediate ticket returned a different batch result") ||
        !check(intermediate_succeeded ||
                   (!intermediate_result.all_succeeded &&
                    intermediate_result.statuses.size() == 1U &&
                    intermediate_result.statuses[0].message ==
                        "render request was superseded"),
               "intermediate ticket had an invalid completion result")) {
      renderer.release();
      return 1;
    }

    mujoco_simulation::CameraRenderTask out_of_range = task;
    out_of_range.config.id = 5;
    mujoco_simulation::CameraRenderTask duplicate = task;
    duplicate.sequence = 4;
    if (!check(!renderer.submit(context, {out_of_range}),
               "out-of-range camera ID was accepted") ||
        !check(!renderer.submit(context, {task, duplicate}),
               "duplicate camera ID was accepted")) {
      renderer.release();
      return 1;
    }

    mujoco_simulation::CameraRenderTask failed = task;
    failed.config.id = 3;
    failed.config.camera_name = "missing_camera";
    failed.sequence = 5;
    task.sequence = 6;
    mujoco_simulation::CameraBatchResult batch;
    const auto mixed_ticket = renderer.submit(context, {task, failed});
    if (!check(mixed_ticket.has_value(),
               "failed to submit mixed camera batch") ||
        !check(!renderer.wait(*mixed_ticket, &batch),
               "mixed camera batch was incorrectly successful") ||
        !check(batch.statuses.size() == 2U && !batch.all_succeeded &&
                   batch.statuses[0].succeeded &&
                   !batch.statuses[1].succeeded &&
                   !batch.statuses[1].message.empty(),
               "mixed camera batch did not report per-camera statuses")) {
      renderer.release();
      return 1;
    }

    released = renderer.release();
  }
  context.clear();
  return released ? 0 : 1;
}
