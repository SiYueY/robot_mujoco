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
                                  std::function<Status(const ResetOptions&)> reset = {},
                                  std::function<Status()> write = {},
                                  std::function<Status()> read = {},
                                  std::function<Status()> publish = {},
                                  std::function<Status()> sync_viewer = {}) {
  SchedulerCallbacks callbacks;
  callbacks.timestep_provider = []() { return 0.001; };
  callbacks.write_commands = std::move(write);
  callbacks.step_physics = [&physics_steps, after_step = std::move(after_step)]() {
    ++physics_steps;
    if (after_step) {
      after_step();
    }
    return Status::Ok();
  };
  callbacks.read_components = std::move(read);
  callbacks.publish_state_snapshot = std::move(publish);
  callbacks.sync_viewer_if_due = std::move(sync_viewer);
  callbacks.reset_runtime =
      reset ? std::move(reset) : [](const ResetOptions&) { return Status::Ok(); };
  return callbacks;
}

}  // namespace

TEST(SimulationSchedulerTest, RejectsStartBeforeInitialize) {
  SimulationScheduler scheduler;
  const Status status = scheduler.start();

  EXPECT_FALSE(status.ok());
  EXPECT_EQ(status.code(), StatusCode::FailedPrecondition);
}

TEST(SimulationSchedulerTest, ManualStepAllowedWhenStoppedAndRejectedWhenRunning) {
  SimulationScheduler scheduler;
  std::atomic<int> physics_steps{0};
  ASSERT_TRUE(scheduler.initialize({}, make_callbacks(physics_steps)).ok());

  ASSERT_TRUE(scheduler.step(3).ok());
  EXPECT_EQ(physics_steps.load(), 3);
  EXPECT_EQ(scheduler.statistics().manual_step_calls, 3u);

  ASSERT_TRUE(scheduler.start().ok());
  std::this_thread::sleep_for(10ms);

  const Status step_while_running = scheduler.step();
  EXPECT_FALSE(step_while_running.ok());
  EXPECT_EQ(step_while_running.code(), StatusCode::FailedPrecondition);

  ASSERT_TRUE(scheduler.stop().ok());
  ASSERT_TRUE(scheduler.shutdown().ok());
}

TEST(SimulationSchedulerTest, PauseStopsPhysicsAndResumeRestartsIt) {
  SimulationScheduler scheduler;
  std::atomic<int> physics_steps{0};
  ASSERT_TRUE(scheduler.initialize({}, make_callbacks(physics_steps)).ok());
  ASSERT_TRUE(scheduler.start().ok());

  std::this_thread::sleep_for(20ms);
  ASSERT_TRUE(scheduler.pause().ok());

  const int paused_count = physics_steps.load();
  std::this_thread::sleep_for(20ms);
  EXPECT_EQ(physics_steps.load(), paused_count);
  EXPECT_EQ(scheduler.status(), SimulationStatus::Paused);

  ASSERT_TRUE(scheduler.resume().ok());
  std::this_thread::sleep_for(20ms);
  EXPECT_GT(physics_steps.load(), paused_count);

  ASSERT_TRUE(scheduler.stop().ok());
  ASSERT_TRUE(scheduler.shutdown().ok());
}

TEST(SimulationSchedulerTest, ManualStepExecutesCallbacksInDocumentedOrder) {
  SimulationScheduler scheduler;
  std::atomic<int> physics_steps{0};
  std::mutex events_mutex;
  std::vector<std::string> events;

  auto record = [&events_mutex, &events](const char* name) {
    std::lock_guard<std::mutex> lock(events_mutex);
    events.emplace_back(name);
    return Status::Ok();
  };

  SchedulerCallbacks callbacks = make_callbacks(
      physics_steps, {}, {}, [&record]() { return record("write"); },
      [&record]() { return record("read"); }, [&record]() { return record("publish"); },
      [&record]() { return record("sync"); });
  callbacks.step_physics = [&physics_steps, &record]() {
    ++physics_steps;
    return record("step");
  };

  ASSERT_TRUE(scheduler.initialize({}, std::move(callbacks)).ok());
  ASSERT_TRUE(scheduler.step().ok());

  EXPECT_EQ(events.size(), 5u);
  ASSERT_EQ(events.size(), 5u);
  EXPECT_EQ(events[0], "write");
  EXPECT_EQ(events[1], "step");
  EXPECT_EQ(events[2], "read");
  EXPECT_EQ(events[3], "publish");
  EXPECT_EQ(events[4], "sync");

  const SchedulerStatistics statistics = scheduler.statistics();
  EXPECT_EQ(statistics.physics_steps, 1u);
  EXPECT_EQ(statistics.loop_iterations, 1u);
  EXPECT_EQ(statistics.manual_step_calls, 1u);

  ASSERT_TRUE(scheduler.shutdown().ok());
}

TEST(SimulationSchedulerTest, ResetRequestRunsOnWorkerThread) {
  SimulationScheduler scheduler;
  std::atomic<int> physics_steps{0};
  std::promise<std::thread::id> reset_thread_promise;
  std::future<std::thread::id> reset_thread_future = reset_thread_promise.get_future();
  const std::thread::id caller_thread = std::this_thread::get_id();

  auto reset_callback = [&reset_thread_promise](const ResetOptions&) {
    reset_thread_promise.set_value(std::this_thread::get_id());
    return Status::Ok();
  };

  ASSERT_TRUE(
      scheduler.initialize({}, make_callbacks(physics_steps, {}, std::move(reset_callback))).ok());
  ASSERT_TRUE(scheduler.start().ok());
  ASSERT_TRUE(scheduler.request_reset({.options = {.reset_statistics = true}}).ok());

  ASSERT_EQ(reset_thread_future.wait_for(1s), std::future_status::ready);
  EXPECT_NE(reset_thread_future.get(), caller_thread);

  ASSERT_TRUE(scheduler.stop().ok());
  ASSERT_TRUE(scheduler.shutdown().ok());
}

TEST(SimulationSchedulerTest, WaitableResetReturnsExecutionFailure) {
  SimulationScheduler scheduler;
  std::atomic<int> physics_steps{0};
  auto reset_callback = [](const ResetOptions&) { return Status::internal("reset failed"); };

  ASSERT_TRUE(
      scheduler.initialize({}, make_callbacks(physics_steps, {}, std::move(reset_callback))).ok());
  ASSERT_TRUE(scheduler.start().ok());

  std::future<Status> completion =
      scheduler.request_reset_waitable({.options = {.reset_statistics = true}});
  ASSERT_EQ(completion.wait_for(1s), std::future_status::ready);

  const Status status = completion.get();
  EXPECT_FALSE(status.ok());
  EXPECT_EQ(status.code(), StatusCode::Internal);
  EXPECT_EQ(status.message(), "reset failed");

  ASSERT_TRUE(scheduler.stop().ok());
  ASSERT_TRUE(scheduler.shutdown().ok());
}

TEST(SimulationSchedulerTest, WaitableResetReturnsThreadFailureWhenCallbackThrows) {
  SimulationScheduler scheduler;
  std::atomic<int> physics_steps{0};
  auto reset_callback = [](const ResetOptions&) -> Status {
    throw std::runtime_error("reset callback boom");
  };

  ASSERT_TRUE(
      scheduler.initialize({}, make_callbacks(physics_steps, {}, std::move(reset_callback))).ok());
  ASSERT_TRUE(scheduler.start().ok());

  std::future<Status> completion =
      scheduler.request_reset_waitable({.options = {.reset_statistics = true}});
  ASSERT_EQ(completion.wait_for(1s), std::future_status::ready);

  const Status status = completion.get();
  EXPECT_FALSE(status.ok());
  EXPECT_EQ(status.code(), StatusCode::ThreadFailed);
  EXPECT_NE(status.message().find("reset callback boom"), std::string::npos);
  EXPECT_EQ(scheduler.status(), SimulationStatus::Error);

  ASSERT_TRUE(scheduler.stop().ok());
  ASSERT_TRUE(scheduler.shutdown().ok());
}

TEST(SimulationSchedulerTest, ResetStatisticsClearsCycleCountersAfterStoppedManualStep) {
  SimulationScheduler scheduler;
  std::atomic<int> physics_steps{0};
  ASSERT_TRUE(scheduler.initialize({}, make_callbacks(physics_steps)).ok());

  ASSERT_TRUE(scheduler.step(3).ok());
  SchedulerStatistics statistics = scheduler.statistics();
  EXPECT_EQ(statistics.physics_steps, 3u);
  EXPECT_EQ(statistics.loop_iterations, 3u);
  EXPECT_EQ(statistics.manual_step_calls, 3u);

  std::future<Status> completion =
      scheduler.request_reset_waitable({.options = {.reset_statistics = true}});
  ASSERT_EQ(completion.wait_for(1s), std::future_status::ready);
  ASSERT_TRUE(completion.get().ok());

  statistics = scheduler.statistics();
  EXPECT_EQ(statistics.reset_requests, 1u);
  EXPECT_EQ(statistics.physics_steps, 0u);
  EXPECT_EQ(statistics.loop_iterations, 0u);
  EXPECT_EQ(statistics.manual_step_calls, 0u);
  EXPECT_EQ(statistics.lag_recoveries, 0u);
  EXPECT_DOUBLE_EQ(statistics.last_loop_duration_sec, 0.0);
  EXPECT_DOUBLE_EQ(statistics.last_step_duration_sec, 0.0);
  EXPECT_DOUBLE_EQ(statistics.last_realtime_factor, 0.0);

  ASSERT_TRUE(scheduler.shutdown().ok());
}

TEST(SimulationSchedulerTest, TracksLagRecoveryAcrossStartStopCycles) {
  SimulationScheduler scheduler;
  std::atomic<int> physics_steps{0};
  SchedulerConfig config;
  config.max_schedule_lag = 1ms;

  auto after_step = []() { std::this_thread::sleep_for(4ms); };
  ASSERT_TRUE(scheduler.initialize(config, make_callbacks(physics_steps, after_step)).ok());

  for (int i = 0; i < 3; ++i) {
    ASSERT_TRUE(scheduler.start().ok());
    std::this_thread::sleep_for(25ms);
    ASSERT_TRUE(scheduler.stop().ok());
  }

  const SchedulerStatistics statistics = scheduler.statistics();
  EXPECT_GT(statistics.physics_steps, 0u);
  EXPECT_GT(statistics.lag_recoveries, 0u);
  EXPECT_GT(statistics.last_realtime_factor, 0.0);
  EXPECT_EQ(scheduler.status(), SimulationStatus::Stopped);

  ASSERT_TRUE(scheduler.shutdown().ok());
}

TEST(SimulationSchedulerTest, WorkerStepCallbackFailureTransitionsSchedulerToError) {
  SimulationScheduler scheduler;
  std::atomic<int> physics_steps{0};
  auto failing_step = [&physics_steps]() -> Status {
    ++physics_steps;
    throw std::runtime_error("physics worker boom");
  };

  SchedulerCallbacks callbacks = make_callbacks(physics_steps);
  callbacks.step_physics = std::move(failing_step);
  ASSERT_TRUE(scheduler.initialize({}, std::move(callbacks)).ok());
  ASSERT_TRUE(scheduler.start().ok());

  for (int attempt = 0; attempt < 100 && scheduler.status() != SimulationStatus::Error; ++attempt) {
    std::this_thread::sleep_for(2ms);
  }

  EXPECT_EQ(scheduler.status(), SimulationStatus::Error);
  EXPECT_GT(physics_steps.load(), 0);
  ASSERT_TRUE(scheduler.stop().ok());
  ASSERT_TRUE(scheduler.shutdown().ok());
}

TEST(SimulationSchedulerTest, RealtimeFactorCanBeUpdatedAfterInitialize) {
  SimulationScheduler scheduler;
  std::atomic<int> physics_steps{0};
  SchedulerConfig config;
  config.realtime_factor = 1.0;

  ASSERT_TRUE(scheduler.initialize(config, make_callbacks(physics_steps)).ok());
  EXPECT_DOUBLE_EQ(scheduler.realtime_factor(), 1.0);

  ASSERT_TRUE(scheduler.set_realtime_factor(2.5).ok());
  EXPECT_DOUBLE_EQ(scheduler.realtime_factor(), 2.5);

  ASSERT_TRUE(scheduler.start().ok());
  std::this_thread::sleep_for(10ms);
  ASSERT_TRUE(scheduler.set_realtime_factor(0.5).ok());
  EXPECT_DOUBLE_EQ(scheduler.realtime_factor(), 0.5);

  ASSERT_TRUE(scheduler.stop().ok());
  ASSERT_TRUE(scheduler.shutdown().ok());
}

TEST(SimulationSchedulerTest, RejectsInvalidRealtimeFactorUpdates) {
  SimulationScheduler scheduler;
  std::atomic<int> physics_steps{0};

  const Status before_initialize = scheduler.set_realtime_factor(0.0);
  EXPECT_FALSE(before_initialize.ok());
  EXPECT_EQ(before_initialize.code(), StatusCode::InvalidArgument);

  ASSERT_TRUE(scheduler.initialize({}, make_callbacks(physics_steps)).ok());

  const Status zero = scheduler.set_realtime_factor(0.0);
  EXPECT_FALSE(zero.ok());
  EXPECT_EQ(zero.code(), StatusCode::InvalidArgument);

  const Status negative = scheduler.set_realtime_factor(-1.0);
  EXPECT_FALSE(negative.ok());
  EXPECT_EQ(negative.code(), StatusCode::InvalidArgument);

  ASSERT_TRUE(scheduler.shutdown().ok());
}

}  // namespace mujoco_simulation
