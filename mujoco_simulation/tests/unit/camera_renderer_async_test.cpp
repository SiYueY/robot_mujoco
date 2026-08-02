#include <mujoco/mujoco.h>

#include <algorithm>
#include <atomic>
#include <filesystem>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "render/camera_render_service_impl.hpp"
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
      "mujoco_camera_async_test.xml");
  if (!check(model_file.write(R"(<mujoco model="camera_async_test">
  <worldbody>
    <light pos="0 0 3"/>
    <body name="moving_box">
      <joint name="box_slide" type="slide" axis="1 0 0"/>
      <geom type="box" size="0.2 0.2 0.2" rgba="0.8 0.2 0.2 1"/>
    </body>
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
    if (!check(renderer.wait_result({0, 1}) ==
                   mujoco_simulation::CameraRenderWaitStatus::InvalidTicket,
               "renderer did not diagnose an invalid ticket") ||
        !check(!renderer.wait({0, 1}),
               "renderer accepted a ticket it did not submit")) {
      return 1;
    }
    if (!check(renderer.initialize(context),
               "failed to initialize camera worker")) {
      return 1;
    }
    const auto noop_ticket = renderer.submit(context, {});
    if (!check(noop_ticket.has_value() && noop_ticket->is_noop(),
               "empty camera batch did not return a no-op ticket") ||
        !check(renderer.wait_result(*noop_ticket) ==
                   mujoco_simulation::CameraRenderWaitStatus::Completed,
               "no-op camera ticket was not immediately successful")) {
      renderer.release();
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
    task.camera_id = task.config.id;
    mujoco_simulation::CameraRenderBatchRequest first_request;
    first_request.generation = 1;
    first_request.sequence = 1;
    first_request.simulation_step = 42;
    first_request.simulation_time = 0.7;
    first_request.model = model;
    first_request.data = data;
    first_request.tasks = {task};
    const auto first_ticket = renderer.submit(first_request);
    if (!check(first_ticket.has_value(), "failed to submit first frame") ||
        !check(renderer.wait(*first_ticket), "first frame did not finish") ||
        !check(renderer.query(*first_ticket) ==
                   mujoco_simulation::CameraRenderWaitStatus::Completed,
               "completed camera ticket was not queryable")) {
      renderer.release();
      return 1;
    }
    mujoco_simulation::CameraBatchResult first_result;
    if (!check(renderer.query(*first_ticket, &first_result) ==
                       mujoco_simulation::CameraRenderWaitStatus::Completed &&
                   first_result.status ==
                       mujoco_simulation::CameraBatchStatus::Completed &&
                   first_result.simulation_step == 42 &&
                   first_result.simulation_time == 0.7 &&
                   first_result.cameras.size() == 1U &&
                   first_result.cameras.front().camera_id == task.camera_id &&
                   first_result.cameras.front().status ==
                       mujoco_simulation::CameraTaskStatus::Completed &&
                   !first_result.cameras.front().frame.image.data.empty(),
               "batch result did not preserve submitted camera metadata")) {
      renderer.release();
      return 1;
    }

    // submit() must copy mjData before it returns. Queue a deliberately large
    // render first, submit at qpos=0, then move the live model before the
    // queued job can be consumed. The queued image must differ from a later
    // request made from the moved state.
    mujoco_simulation::CameraRenderTask blocker = task;
    blocker.config.width = 1024;
    blocker.config.height = 768;
    blocker.sequence = 20;
    if (!check(renderer.submit(context, {blocker}).has_value(),
               "failed to submit snapshot-isolation blocker")) {
      renderer.release();
      return 1;
    }
    mujoco_simulation::CameraRenderTask snapshot_task = task;
    snapshot_task.sequence = 21;
    mujoco_simulation::CameraRenderBatchRequest snapshot_request;
    snapshot_request.generation = 9;
    snapshot_request.sequence = 21;
    snapshot_request.simulation_step = 101;
    snapshot_request.simulation_time = data->time;
    snapshot_request.model = model;
    snapshot_request.data = data;
    snapshot_request.tasks = {snapshot_task};
    const auto snapshot_ticket = renderer.submit(snapshot_request);
    if (!check(snapshot_ticket.has_value() && model->nq > 0,
               "failed to submit snapshot-isolation request")) {
      renderer.release();
      return 1;
    }
    data->qpos[0] = 2.0;
    mj_forward(model, data);
    mujoco_simulation::CameraBatchResult snapshot_result;
    if (!check(renderer.wait(*snapshot_ticket, &snapshot_result) &&
                   snapshot_result.cameras.size() == 1U,
               "snapshot-isolation request did not finish")) {
      renderer.release();
      return 1;
    }
    mujoco_simulation::CameraRenderBatchRequest moved_request =
        snapshot_request;
    moved_request.sequence = 22;
    moved_request.simulation_step = 102;
    moved_request.data = data;
    const auto moved_ticket = renderer.submit(moved_request);
    mujoco_simulation::CameraBatchResult moved_result;
    if (!check(moved_ticket.has_value() &&
                   renderer.wait(*moved_ticket, &moved_result) &&
                   moved_result.cameras.size() == 1U &&
                   snapshot_result.cameras.front().frame.image.data !=
                       moved_result.cameras.front().frame.image.data,
               "camera render used live mjData after submit returned")) {
      renderer.release();
      return 1;
    }

    // Force the documented A-active / B-pending / C-submitted schedule.
    // B must become an entire superseded batch while A continues to finish.
    mujoco_simulation::CameraRenderTask active_task = task;
    active_task.config.width = 2048;
    active_task.config.height = 1536;
    active_task.sequence = 30;
    const auto active_ticket = renderer.submit(context, {active_task});
    if (!check(active_ticket.has_value(), "failed to submit active batch")) {
      renderer.release();
      return 1;
    }
    const auto active_deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (renderer.active_ticket_sequence_for_test() !=
               active_ticket->sequence &&
           std::chrono::steady_clock::now() < active_deadline) {
      std::this_thread::yield();
    }
    if (!check(renderer.active_ticket_sequence_for_test() ==
                   active_ticket->sequence,
               "camera worker did not begin the active batch")) {
      renderer.release();
      return 1;
    }
    mujoco_simulation::CameraRenderTask pending_task = task;
    pending_task.sequence = 31;
    const auto pending_ticket = renderer.submit(context, {pending_task});
    mujoco_simulation::CameraRenderTask replacement_task = task;
    replacement_task.sequence = 32;
    bool replacement_replaced_pending_batch = false;
    const auto replacement_batch_ticket = renderer.submit(
        mujoco_simulation::CameraRenderBatchRequest{
            0, 0, 0, data->time, model, data, {replacement_task}},
        &replacement_replaced_pending_batch);
    mujoco_simulation::CameraBatchResult pending_result;
    if (!check(pending_ticket.has_value() &&
                   replacement_batch_ticket.has_value() &&
                   replacement_replaced_pending_batch &&
                   renderer.wait_result(*pending_ticket, &pending_result) ==
                       mujoco_simulation::CameraRenderWaitStatus::Superseded &&
                   pending_result.status ==
                       mujoco_simulation::CameraBatchStatus::Superseded &&
                   pending_result.cameras.size() == 1U &&
                   pending_result.cameras.front().status ==
                       mujoco_simulation::CameraTaskStatus::Superseded &&
                   renderer.wait(*active_ticket) &&
                   renderer.wait(*replacement_batch_ticket),
               "batch supersede did not preserve active and replace pending")) {
      renderer.release();
      return 1;
    }

    mujoco_simulation::CameraRenderTask second_camera = task;
    second_camera.config.id = 3;
    second_camera.camera_id = 3;
    second_camera.sequence = 2;
    const std::uint64_t copies_before = renderer.snapshot_copy_count();
    const auto two_camera_ticket =
        renderer.submit(context, {task, second_camera});
    mujoco_simulation::CameraBatchResult two_camera_result;
    if (!check(
            two_camera_ticket.has_value() &&
                renderer.wait(*two_camera_ticket) &&
                renderer.snapshot_copy_count() == copies_before + 1U &&
                renderer.query(*two_camera_ticket, &two_camera_result) ==
                    mujoco_simulation::CameraRenderWaitStatus::Completed &&
                two_camera_result.cameras.size() == 2U &&
                std::all_of(two_camera_result.cameras.begin(),
                            two_camera_result.cameras.end(),
                            [&two_camera_result](const auto &camera) {
                              return camera.generation ==
                                         two_camera_result.ticket.generation &&
                                     camera.batch_sequence ==
                                         two_camera_result.ticket.sequence &&
                                     camera.simulation_step ==
                                         two_camera_result.simulation_step &&
                                     camera.simulation_time ==
                                         two_camera_result.simulation_time;
                            }),
            "two-camera batch did not preserve one shared snapshot contract")) {
      renderer.release();
      return 1;
    }

    const auto old_ticket = *first_ticket;
    if (!check(renderer.release(),
               "failed to release first renderer lifecycle") ||
        !check(renderer.wait_result(old_ticket) ==
                   mujoco_simulation::CameraRenderWaitStatus::Stopped,
               "stopped renderer did not report a stopped ticket") ||
        !check(renderer.initialize(context),
               "failed to initialize second renderer lifecycle")) {
      return 1;
    }
    task.sequence = 2;
    task.timestamp = 200;
    const auto restarted_ticket = renderer.submit(context, {task});
    if (!check(
            restarted_ticket.has_value() &&
                restarted_ticket->sequence == old_ticket.sequence &&
                restarted_ticket->generation != old_ticket.generation,
            "renderer restart did not create a distinct ticket generation") ||
        !check(renderer.wait_result(old_ticket) ==
                   mujoco_simulation::CameraRenderWaitStatus::InvalidTicket,
               "old renderer lifecycle ticket matched a restarted renderer") ||
        !check(renderer.wait(*restarted_ticket),
               "restarted renderer ticket did not complete")) {
      renderer.release();
      return 1;
    }

    // A waiter releases job_mutex_ while blocked. Releasing and immediately
    // restarting the renderer must invalidate that old ticket even when the
    // new lifecycle reuses sequence 1.
    if (!check(renderer.release(), "failed to prepare race-test lifecycle") ||
        !check(renderer.initialize(context),
               "failed to initialize race-test lifecycle")) {
      return 1;
    }
    mujoco_simulation::CameraRenderTask race_task = task;
    race_task.config.width = 1024;
    race_task.config.height = 768;
    race_task.sequence = 3;
    const auto race_ticket = renderer.submit(context, {race_task});
    if (!check(race_ticket.has_value(), "failed to submit race-test frame")) {
      renderer.release();
      return 1;
    }
    std::atomic<bool> waiter_started{false};
    mujoco_simulation::CameraRenderWaitStatus old_wait_result =
        mujoco_simulation::CameraRenderWaitStatus::Timeout;
    std::thread waiter([&] {
      waiter_started.store(true, std::memory_order_release);
      old_wait_result = renderer.wait_result(*race_ticket);
    });
    while (!waiter_started.load(std::memory_order_acquire)) {
      std::this_thread::yield();
    }
    if (!check(renderer.release(), "failed to release race-test lifecycle") ||
        !check(renderer.initialize(context),
               "failed to restart race-test lifecycle")) {
      waiter.join();
      return 1;
    }
    const auto replacement_ticket = renderer.submit(context, {task});
    if (!check(
            replacement_ticket.has_value() &&
                replacement_ticket->sequence == race_ticket->sequence &&
                replacement_ticket->generation != race_ticket->generation,
            "race-test restart did not reuse sequence in a new generation") ||
        !check(renderer.wait(*replacement_ticket),
               "race-test replacement frame did not finish")) {
      waiter.join();
      renderer.release();
      return 1;
    }
    waiter.join();
    if (!check(
            old_wait_result ==
                    mujoco_simulation::CameraRenderWaitStatus::InvalidTicket ||
                old_wait_result ==
                    mujoco_simulation::CameraRenderWaitStatus::Stopped,
            "waiter was not woken by the replaced renderer lifecycle")) {
      renderer.release();
      return 1;
    }

    task.sequence = 4;
    task.timestamp = 300;
    const auto intermediate_ticket = renderer.submit(context, {task});
    if (!check(intermediate_ticket.has_value(),
               "failed to submit intermediate frame")) {
      renderer.release();
      return 1;
    }
    task.sequence = 5;
    task.timestamp = 400;
    const auto latest_ticket = renderer.submit(context, {task});
    if (!check(latest_ticket.has_value(), "failed to submit latest frame") ||
        !check(renderer.wait(*latest_ticket), "latest frame did not finish")) {
      renderer.release();
      return 1;
    }

    mujoco_simulation::CameraBatchResult latest_result;
    if (!check(renderer.query(*latest_ticket, &latest_result) ==
                       mujoco_simulation::CameraRenderWaitStatus::Completed &&
                   latest_result.cameras.size() == 1U &&
                   latest_result.cameras.front().sequence == 5 &&
                   latest_result.cameras.front().timestamp == 400 &&
                   !latest_result.cameras.front().frame.image.data.empty(),
               "latest ticket did not retain its completed camera result")) {
      renderer.release();
      return 1;
    }

    // The intermediate request may either have rendered before the later
    // submit or have been superseded. In both cases wait() must return the
    // result belonging to that exact ticket, never the latest batch result.
    mujoco_simulation::CameraBatchResult intermediate_result;
    const bool intermediate_succeeded =
        renderer.wait(*intermediate_ticket, &intermediate_result);
    if (!check(intermediate_result.ticket.generation ==
                       intermediate_ticket->generation &&
                   intermediate_result.ticket.sequence ==
                       intermediate_ticket->sequence,
               "intermediate ticket returned a different batch result") ||
        !check(intermediate_succeeded ||
                   (!intermediate_result.all_succeeded &&
                    intermediate_result.cameras.size() == 1U &&
                    intermediate_result.cameras[0].status ==
                        mujoco_simulation::CameraTaskStatus::Superseded),
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
        !check(renderer.wait_result(*mixed_ticket, &batch) ==
                   mujoco_simulation::CameraRenderWaitStatus::PartiallyFailed,
               "mixed camera batch did not report PartiallyFailed") ||
        !check(renderer.query(*mixed_ticket, &batch) ==
                   mujoco_simulation::CameraRenderWaitStatus::PartiallyFailed,
               "mixed camera batch query did not report PartiallyFailed") ||
        !check(!renderer.wait(*mixed_ticket, &batch),
               "mixed camera batch was incorrectly successful") ||
        !check(batch.cameras.size() == 2U && !batch.all_succeeded &&
                   batch.cameras[0].status ==
                       mujoco_simulation::CameraTaskStatus::Completed &&
                   batch.cameras[1].status ==
                       mujoco_simulation::CameraTaskStatus::Failed &&
                   !batch.cameras[1].message.empty(),
               "mixed camera batch did not report per-camera statuses")) {
      renderer.release();
      return 1;
    }

    // Under TSan, exercise the actual contract: a physics-side writer and a
    // submitter serialize access to live mjData, while the renderer consumes
    // only its owned snapshot after submit() has returned.
    std::mutex live_data_mutex;
    std::atomic<bool> concurrent_submit_failed{false};
    std::thread physics_writer([&] {
      for (int index = 0; index < 80; ++index) {
        std::lock_guard<std::mutex> lock(live_data_mutex);
        data->qpos[0] = (index & 1) == 0 ? -0.15 : 0.15;
        mj_forward(model, data);
      }
    });
    std::thread camera_submitter([&] {
      for (std::uint64_t index = 0; index < 24; ++index) {
        mujoco_simulation::CameraRenderTask concurrent_task = task;
        concurrent_task.sequence = 1000 + index;
        mujoco_simulation::CameraRenderBatchRequest request;
        request.generation = 17;
        request.sequence = 1000 + index;
        request.simulation_step = 1000 + index;
        request.model = model;
        request.tasks = {concurrent_task};
        {
          std::lock_guard<std::mutex> lock(live_data_mutex);
          request.data = data;
          request.simulation_time = data->time;
          if (!renderer.submit(request).has_value()) {
            concurrent_submit_failed.store(true, std::memory_order_release);
          }
        }
      }
    });
    physics_writer.join();
    camera_submitter.join();
    if (!check(!concurrent_submit_failed.load(std::memory_order_acquire),
               "concurrent camera snapshot submission failed")) {
      renderer.release();
      return 1;
    }

    released = renderer.release();
  }
  {
    // Service-level reset/shutdown must wake a batch waiter. This verifies the
    // contract at the Runtime/Render boundary rather than only CameraRenderer.
    mujoco_simulation::CameraRendererConfig service_renderer_config;
    service_renderer_config.allow_glfw_backend = false;
    service_renderer_config.allow_egl_backend = true;
    service_renderer_config.max_camera_id = 4;
    mujoco_simulation::CameraRenderServiceImpl service(service_renderer_config);
    mujoco_simulation::SimulationConfig service_config;
    if (!check(service.initialize(service_config, model),
               "failed to initialize camera render service")) {
      context.clear();
      return 1;
    }
    mujoco_simulation::CameraRenderTask service_task;
    service_task.camera_id = 2;
    service_task.config.id = 2;
    service_task.config.name = "camera";
    service_task.config.camera_name = "test_camera";
    service_task.config.width = 2048;
    service_task.config.height = 1536;
    service_task.config.enable_rgb = true;
    service_task.sequence = 1;
    mujoco_simulation::CameraRenderBatchRequest service_request;
    service_request.generation = 1;
    service_request.sequence = 1;
    service_request.model = model;
    service_request.data = data;
    service_request.tasks = {service_task};
    mujoco_simulation::CameraRenderTicket service_ticket;
    if (!check(service.submit(service_request, service_ticket) ==
                   mujoco_simulation::CameraRenderSubmitResult::Accepted,
               "failed to submit service shutdown batch")) {
      service.shutdown();
      context.clear();
      return 1;
    }
    std::atomic<bool> service_waiter_started{false};
    mujoco_simulation::CameraRenderWaitStatus service_wait_status =
        mujoco_simulation::CameraRenderWaitStatus::Timeout;
    std::thread service_waiter([&] {
      service_waiter_started.store(true, std::memory_order_release);
      service_wait_status =
          service.wait(service_ticket, std::chrono::seconds(5));
    });
    while (!service_waiter_started.load(std::memory_order_acquire)) {
      std::this_thread::yield();
    }
    if (!check(service.shutdown(), "camera render service shutdown failed")) {
      service_waiter.join();
      context.clear();
      return 1;
    }
    service_waiter.join();
    if (!check(service_wait_status ==
                       mujoco_simulation::CameraRenderWaitStatus::Stopped ||
                   service_wait_status ==
                       mujoco_simulation::CameraRenderWaitStatus::Cancelled,
               "service shutdown did not wake the batch waiter")) {
      context.clear();
      return 1;
    }
    if (!check(service.initialize(service_config, model),
               "failed to restart camera render service for reset")) {
      context.clear();
      return 1;
    }
    service_task.sequence = 2;
    service_request.sequence = 2;
    service_request.tasks = {service_task};
    const bool reset_submit_accepted =
        service.submit(service_request, service_ticket) ==
        mujoco_simulation::CameraRenderSubmitResult::Accepted;
    const bool reset_succeeded = reset_submit_accepted && service.reset();
    const auto reset_wait_status =
        reset_succeeded
            ? service.wait(service_ticket, std::chrono::milliseconds(1))
            : mujoco_simulation::CameraRenderWaitStatus::Failed;
    if (!check(reset_succeeded &&
                   (reset_wait_status ==
                        mujoco_simulation::CameraRenderWaitStatus::Stopped ||
                    reset_wait_status ==
                        mujoco_simulation::CameraRenderWaitStatus::Cancelled),
               "service reset did not invalidate the pre-reset batch")) {
      service.shutdown();
      context.clear();
      return 1;
    }
    if (!check(service.shutdown(),
               "camera render service final shutdown failed")) {
      context.clear();
      return 1;
    }
  }
  mujoco_simulation::CameraRendererConfig concurrent_config;
  concurrent_config.allow_glfw_backend = false;
  concurrent_config.allow_egl_backend = true;
  mujoco_simulation::CameraRenderer concurrent_renderer(concurrent_config);
  std::atomic<bool> first_initialized{false};
  std::atomic<bool> second_initialized{false};
  std::thread first_initializer([&] {
    first_initialized.store(concurrent_renderer.initialize(context));
  });
  std::thread second_initializer([&] {
    second_initialized.store(concurrent_renderer.initialize(context));
  });
  first_initializer.join();
  second_initializer.join();
  if (!check(first_initialized.load() && second_initialized.load(),
             "concurrent renderer initialization was not idempotent")) {
    concurrent_renderer.release();
    context.clear();
    return 1;
  }
  std::atomic<bool> first_released{false};
  std::atomic<bool> second_released{false};
  std::thread first_releaser(
      [&] { first_released.store(concurrent_renderer.release()); });
  std::thread second_releaser(
      [&] { second_released.store(concurrent_renderer.release()); });
  first_releaser.join();
  second_releaser.join();
  if (!check(first_released.load() && second_released.load() &&
                 !concurrent_renderer.is_initialized(),
             "concurrent renderer release was not idempotent")) {
    context.clear();
    return 1;
  }

  mujoco_simulation::CameraRendererConfig history_config;
  history_config.allow_glfw_backend = false;
  history_config.allow_egl_backend = true;
  history_config.completed_ticket_history = 1U;
  mujoco_simulation::CameraRenderer history_renderer(history_config);
  if (!check(history_renderer.initialize(context),
             "failed to initialize ticket-history renderer")) {
    context.clear();
    return 1;
  }
  mujoco_simulation::CameraRenderTask history_task;
  history_task.config.id = 2;
  history_task.config.name = "camera";
  history_task.config.camera_name = "test_camera";
  history_task.config.width = 32;
  history_task.config.height = 24;
  history_task.config.enable_rgb = true;
  history_task.sequence = 9;
  const auto stale_ticket = history_renderer.submit(context, {history_task});
  if (!check(stale_ticket.has_value() && history_renderer.wait(*stale_ticket),
             "first ticket-history batch did not complete")) {
    history_renderer.release();
    context.clear();
    return 1;
  }
  ++history_task.sequence;
  const auto retained_ticket = history_renderer.submit(context, {history_task});
  if (!check(retained_ticket.has_value() &&
                 history_renderer.wait(*retained_ticket),
             "second ticket-history batch did not complete") ||
      !check(history_renderer.wait_result(*stale_ticket) ==
                 mujoco_simulation::CameraRenderWaitStatus::Stale,
             "stale ticket did not report Stale")) {
    history_renderer.release();
    context.clear();
    return 1;
  }
  if (!check(history_renderer.release(),
             "failed to release ticket-history renderer")) {
    context.clear();
    return 1;
  }
  context.clear();
  return released ? 0 : 1;
}
