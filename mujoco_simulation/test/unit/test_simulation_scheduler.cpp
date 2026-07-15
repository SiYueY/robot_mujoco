#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <future>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "mujoco_simulation/runtime/simulation_scheduler.hpp"

namespace mujoco_simulation {
namespace {

using namespace std::chrono_literals;

SchedulerCallbacks make_callbacks(std::atomic<int>& physics_steps,
                                  std::function<void()> after_step = {},
                                  std::function<ResultCode(const ResetRequest&)> reset = {},
                                  std::function<ResultCode()> write = {},
                                  std::function<ResultCode()> update = {},
                                  std::function<ResultCode()> write_snapshot = {},
                                  std::function<ResultCode()> sync_viewer = {}) {
  SchedulerCallbacks callbacks;
  callbacks.timestep_provider = []() { return 0.001; };
  callbacks.write_commands = std::move(write);
  callbacks.step_physics = [&physics_steps, after_step = std::move(after_step)]() {
    ++physics_steps;
    if (after_step) {
      after_step();
    }
    return ResultCode::Ok;
  };
  callbacks.update_components = std::move(update);
  callbacks.write_state_snapshot = std::move(write_snapshot);
  callbacks.sync_viewer_if_due = std::move(sync_viewer);
  callbacks.reset_runtime =
      reset ? std::move(reset) : [](const ResetRequest&) { return ResultCode::Ok; };
  return callbacks;
}

}  // namespace

TEST(SimulationSchedulerTest, RejectsStartBeforeInitialize) {
  SimulationScheduler scheduler;
  const ResultCode status = scheduler.start();

  EXPECT_EQ(status, ResultCode::FailedPrecondition);
}

TEST(SimulationSchedulerTest, ManualStepAllowedWhenStoppedAndRejectedWhenRunning) {
  SimulationScheduler scheduler;
  std::atomic<int> physics_steps{0};
  ASSERT_EQ(scheduler.initialize({}, make_callbacks(physics_steps)), ResultCode::Ok);

  ASSERT_EQ(scheduler.step(3), ResultCode::Ok);
  EXPECT_EQ(physics_steps.load(), 3);
  EXPECT_EQ(scheduler.statistics().manual_step_calls, 3u);

  ASSERT_EQ(scheduler.start(), ResultCode::Ok);
  std::this_thread::sleep_for(10ms);

  const ResultCode step_while_running = scheduler.step();
  EXPECT_EQ(step_while_running, ResultCode::FailedPrecondition);

  ASSERT_EQ(scheduler.stop(), ResultCode::Ok);
  ASSERT_EQ(scheduler.shutdown(), ResultCode::Ok);
}

TEST(SimulationSchedulerTest, PauseStopsPhysicsAndResumeRestartsIt) {
  SimulationScheduler scheduler;
  std::atomic<int> physics_steps{0};
  ASSERT_EQ(scheduler.initialize({}, make_callbacks(physics_steps)), ResultCode::Ok);
  ASSERT_EQ(scheduler.start(), ResultCode::Ok);

  std::this_thread::sleep_for(20ms);
  ASSERT_EQ(scheduler.pause(), ResultCode::Ok);

  const int paused_count = physics_steps.load();
  std::this_thread::sleep_for(20ms);
  EXPECT_EQ(physics_steps.load(), paused_count);
  EXPECT_EQ(scheduler.status(), SimulationStatus::Paused);

  ASSERT_EQ(scheduler.resume(), ResultCode::Ok);
  std::this_thread::sleep_for(20ms);
  EXPECT_GT(physics_steps.load(), paused_count);

  ASSERT_EQ(scheduler.stop(), ResultCode::Ok);
  ASSERT_EQ(scheduler.shutdown(), ResultCode::Ok);
}

TEST(SimulationSchedulerTest, ManualStepExecutesCallbacksInDocumentedOrder) {
  SimulationScheduler scheduler;
  std::atomic<int> physics_steps{0};
  std::mutex events_mutex;
  std::vector<std::string> events;

  auto record = [&events_mutex, &events](const char* name) {
    std::lock_guard<std::mutex> lock(events_mutex);
    events.emplace_back(name);
    return ResultCode::Ok;
  };

  SchedulerCallbacks callbacks = make_callbacks(
      physics_steps, {}, {}, [&record]() { return record("write"); },
      [&record]() { return record("update"); }, [&record]() { return record("write_snapshot"); },
      [&record]() { return record("sync"); });
  callbacks.step_physics = [&physics_steps, &record]() {
    ++physics_steps;
    return record("step");
  };

  ASSERT_EQ(scheduler.initialize({}, std::move(callbacks)), ResultCode::Ok);
  ASSERT_EQ(scheduler.step(), ResultCode::Ok);

  EXPECT_EQ(events.size(), 5u);
  ASSERT_EQ(events.size(), 5u);
  EXPECT_EQ(events[0], "write");
  EXPECT_EQ(events[1], "step");
  EXPECT_EQ(events[2], "update");
  EXPECT_EQ(events[3], "write_snapshot");
  EXPECT_EQ(events[4], "sync");

  const SchedulerStatistics statistics = scheduler.statistics();
  EXPECT_EQ(statistics.physics_steps, 1u);
  EXPECT_EQ(statistics.loop_iterations, 1u);
  EXPECT_EQ(statistics.manual_step_calls, 1u);

  ASSERT_EQ(scheduler.shutdown(), ResultCode::Ok);
}

TEST(SimulationSchedulerTest, ResetRequestRunsOnWorkerThread) {
  SimulationScheduler scheduler;
  std::atomic<int> physics_steps{0};
  std::promise<std::thread::id> reset_thread_promise;
  std::future<std::thread::id> reset_thread_future = reset_thread_promise.get_future();
  const std::thread::id caller_thread = std::this_thread::get_id();

  auto reset_callback = [&reset_thread_promise](const ResetRequest&) {
    reset_thread_promise.set_value(std::this_thread::get_id());
    return ResultCode::Ok;
  };

  ASSERT_EQ(scheduler.initialize({}, make_callbacks(physics_steps, {}, std::move(reset_callback))),
            ResultCode::Ok);
  ASSERT_EQ(scheduler.start(), ResultCode::Ok);
  ASSERT_EQ(scheduler.request_reset({.options = {.reset_statistics = true}}), ResultCode::Ok);

  ASSERT_EQ(reset_thread_future.wait_for(1s), std::future_status::ready);
  EXPECT_NE(reset_thread_future.get(), caller_thread);

  ASSERT_EQ(scheduler.stop(), ResultCode::Ok);
  ASSERT_EQ(scheduler.shutdown(), ResultCode::Ok);
}

TEST(SimulationSchedulerTest, WaitableResetReturnsExecutionFailure) {
  SimulationScheduler scheduler;
  std::atomic<int> physics_steps{0};
  auto reset_callback = [](const ResetRequest&) { return ResultCode::Internal; };

  ASSERT_EQ(scheduler.initialize({}, make_callbacks(physics_steps, {}, std::move(reset_callback))),
            ResultCode::Ok);
  ASSERT_EQ(scheduler.start(), ResultCode::Ok);

  std::future<ResultCode> completion =
      scheduler.request_reset_waitable({.options = {.reset_statistics = true}});
  ASSERT_EQ(completion.wait_for(1s), std::future_status::ready);

  const ResultCode status = completion.get();
  EXPECT_EQ(status, ResultCode::Internal);

  ASSERT_EQ(scheduler.stop(), ResultCode::Ok);
  ASSERT_EQ(scheduler.shutdown(), ResultCode::Ok);
}

TEST(SimulationSchedulerTest, WaitableResetReturnsThreadFailureWhenCallbackThrows) {
  SimulationScheduler scheduler;
  std::atomic<int> physics_steps{0};
  auto reset_callback = [](const ResetRequest&) -> ResultCode {
    throw std::runtime_error("reset callback boom");
  };

  ASSERT_EQ(scheduler.initialize({}, make_callbacks(physics_steps, {}, std::move(reset_callback))),
            ResultCode::Ok);
  ASSERT_EQ(scheduler.start(), ResultCode::Ok);

  std::future<ResultCode> completion =
      scheduler.request_reset_waitable({.options = {.reset_statistics = true}});
  ASSERT_EQ(completion.wait_for(1s), std::future_status::ready);

  const ResultCode status = completion.get();
  EXPECT_EQ(status, ResultCode::ThreadFailed);
  EXPECT_EQ(scheduler.status(), SimulationStatus::Error);

  ASSERT_EQ(scheduler.stop(), ResultCode::Ok);
  ASSERT_EQ(scheduler.shutdown(), ResultCode::Ok);
}

TEST(SimulationSchedulerTest, ResetStatisticsClearsCycleCountersAfterStoppedManualStep) {
  SimulationScheduler scheduler;
  std::atomic<int> physics_steps{0};
  ASSERT_EQ(scheduler.initialize({}, make_callbacks(physics_steps)), ResultCode::Ok);

  ASSERT_EQ(scheduler.step(3), ResultCode::Ok);
  SchedulerStatistics statistics = scheduler.statistics();
  EXPECT_EQ(statistics.physics_steps, 3u);
  EXPECT_EQ(statistics.loop_iterations, 3u);
  EXPECT_EQ(statistics.manual_step_calls, 3u);

  std::future<ResultCode> completion =
      scheduler.request_reset_waitable({.options = {.reset_statistics = true}});
  ASSERT_EQ(completion.wait_for(1s), std::future_status::ready);
  ASSERT_EQ(completion.get(), ResultCode::Ok);

  statistics = scheduler.statistics();
  EXPECT_EQ(statistics.reset_requests, 1u);
  EXPECT_EQ(statistics.physics_steps, 0u);
  EXPECT_EQ(statistics.loop_iterations, 0u);
  EXPECT_EQ(statistics.manual_step_calls, 0u);
  EXPECT_EQ(statistics.lag_recoveries, 0u);
  EXPECT_DOUBLE_EQ(statistics.last_loop_duration_sec, 0.0);
  EXPECT_DOUBLE_EQ(statistics.last_step_duration_sec, 0.0);
  EXPECT_DOUBLE_EQ(statistics.last_realtime_factor, 0.0);

  ASSERT_EQ(scheduler.shutdown(), ResultCode::Ok);
}

TEST(SimulationSchedulerTest, TracksLagRecoveryAcrossStartStopCycles) {
  SimulationScheduler scheduler;
  std::atomic<int> physics_steps{0};
  SchedulerConfig config;
  config.max_schedule_lag = 1ms;

  auto after_step = []() { std::this_thread::sleep_for(4ms); };
  ASSERT_EQ(scheduler.initialize(config, make_callbacks(physics_steps, after_step)),
            ResultCode::Ok);

  for (int i = 0; i < 3; ++i) {
    ASSERT_EQ(scheduler.start(), ResultCode::Ok);
    std::this_thread::sleep_for(25ms);
    ASSERT_EQ(scheduler.stop(), ResultCode::Ok);
  }

  const SchedulerStatistics statistics = scheduler.statistics();
  EXPECT_GT(statistics.physics_steps, 0u);
  EXPECT_GT(statistics.lag_recoveries, 0u);
  EXPECT_GT(statistics.last_realtime_factor, 0.0);
  EXPECT_EQ(scheduler.status(), SimulationStatus::Stopped);

  ASSERT_EQ(scheduler.shutdown(), ResultCode::Ok);
}

TEST(SimulationSchedulerTest, WorkerStepCallbackFailureTransitionsSchedulerToError) {
  SimulationScheduler scheduler;
  std::atomic<int> physics_steps{0};
  auto failing_step = [&physics_steps]() -> ResultCode {
    ++physics_steps;
    throw std::runtime_error("physics worker boom");
  };

  SchedulerCallbacks callbacks = make_callbacks(physics_steps);
  callbacks.step_physics = std::move(failing_step);
  ASSERT_EQ(scheduler.initialize({}, std::move(callbacks)), ResultCode::Ok);
  ASSERT_EQ(scheduler.start(), ResultCode::Ok);

  for (int attempt = 0; attempt < 100 && scheduler.status() != SimulationStatus::Error; ++attempt) {
    std::this_thread::sleep_for(2ms);
  }

  EXPECT_EQ(scheduler.status(), SimulationStatus::Error);
  EXPECT_GT(physics_steps.load(), 0);
  ASSERT_EQ(scheduler.stop(), ResultCode::Ok);
  ASSERT_EQ(scheduler.shutdown(), ResultCode::Ok);
}

TEST(SimulationSchedulerTest, RealtimeFactorCanBeUpdatedAfterInitialize) {
  SimulationScheduler scheduler;
  std::atomic<int> physics_steps{0};
  SchedulerConfig config;
  config.realtime_factor = 1.0;

  ASSERT_EQ(scheduler.initialize(config, make_callbacks(physics_steps)), ResultCode::Ok);
  EXPECT_DOUBLE_EQ(scheduler.realtime_factor(), 1.0);

  ASSERT_EQ(scheduler.set_realtime_factor(2.5), ResultCode::Ok);
  EXPECT_DOUBLE_EQ(scheduler.realtime_factor(), 2.5);

  ASSERT_EQ(scheduler.start(), ResultCode::Ok);
  std::this_thread::sleep_for(10ms);
  ASSERT_EQ(scheduler.set_realtime_factor(0.5), ResultCode::Ok);
  EXPECT_DOUBLE_EQ(scheduler.realtime_factor(), 0.5);

  ASSERT_EQ(scheduler.stop(), ResultCode::Ok);
  ASSERT_EQ(scheduler.shutdown(), ResultCode::Ok);
}

TEST(SimulationSchedulerTest, RejectsInvalidRealtimeFactorUpdates) {
  SimulationScheduler scheduler;
  std::atomic<int> physics_steps{0};

  const ResultCode before_initialize = scheduler.set_realtime_factor(0.0);
  EXPECT_EQ(before_initialize, ResultCode::InvalidArgument);

  ASSERT_EQ(scheduler.initialize({}, make_callbacks(physics_steps)), ResultCode::Ok);

  const ResultCode zero = scheduler.set_realtime_factor(0.0);
  EXPECT_EQ(zero, ResultCode::InvalidArgument);

  const ResultCode negative = scheduler.set_realtime_factor(-1.0);
  EXPECT_EQ(negative, ResultCode::InvalidArgument);

  ASSERT_EQ(scheduler.shutdown(), ResultCode::Ok);
}

}  // namespace mujoco_simulation
