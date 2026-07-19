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

struct FakeSchedulerOperations {
  double timestep{0.001};
  std::atomic<int> cycles{0};
  std::function<void()> after_cycle;
  std::function<bool()> run_cycle_callback;
  std::function<bool(const ResetRequest&)> reset_callback;
};

bool register_operations(SimulationScheduler& scheduler, FakeSchedulerOperations& ops) {
  if (!scheduler.register_timestep_provider([&ops]() { return ops.timestep; })) {
    return false;
  }
  if (!scheduler.register_cycle_runner([&ops]() {
        ++ops.cycles;
        if (ops.after_cycle) {
          ops.after_cycle();
        }
        if (ops.run_cycle_callback) {
          return ops.run_cycle_callback();
        }
        return true;
      })) {
    return false;
  }
  return scheduler.register_reset_handler([&ops](const ResetRequest& request) {
    if (ops.reset_callback) {
      return ops.reset_callback(request);
    }
    return true;
  });
}

}  // namespace

TEST(SimulationSchedulerTest, RejectsStartBeforeInitialize) {
  SimulationScheduler scheduler;
  EXPECT_FALSE(scheduler.start());
}

TEST(SimulationSchedulerTest, ManualStepAllowedWhenStoppedAndRejectedWhenRunning) {
  SimulationScheduler scheduler;
  FakeSchedulerOperations ops;
  ASSERT_TRUE(scheduler.initialize({}));
  ASSERT_TRUE(register_operations(scheduler, ops));

  ASSERT_TRUE(scheduler.step(3));
  EXPECT_EQ(ops.cycles.load(), 3);

  ASSERT_TRUE(scheduler.start());
  std::this_thread::sleep_for(10ms);

  EXPECT_FALSE(scheduler.step());

  ASSERT_TRUE(scheduler.stop());
  ASSERT_TRUE(scheduler.shutdown());
}

TEST(SimulationSchedulerTest, PauseStopsPhysicsAndResumeRestartsIt) {
  SimulationScheduler scheduler;
  FakeSchedulerOperations ops;
  ASSERT_TRUE(scheduler.initialize({}));
  ASSERT_TRUE(register_operations(scheduler, ops));
  ASSERT_TRUE(scheduler.start());

  std::this_thread::sleep_for(20ms);
  ASSERT_TRUE(scheduler.pause());

  const int paused_count = ops.cycles.load();
  std::this_thread::sleep_for(20ms);
  EXPECT_EQ(ops.cycles.load(), paused_count);
  EXPECT_EQ(scheduler.status(), SimulationStatus::Paused);

  ASSERT_TRUE(scheduler.resume());
  std::this_thread::sleep_for(20ms);
  EXPECT_GT(ops.cycles.load(), paused_count);

  ASSERT_TRUE(scheduler.stop());
  ASSERT_TRUE(scheduler.shutdown());
}

TEST(SimulationSchedulerTest, ManualStepExecutesSingleRunCyclePerStep) {
  SimulationScheduler scheduler;
  std::mutex events_mutex;
  std::vector<std::string> events;
  FakeSchedulerOperations ops;
  ops.run_cycle_callback = [&events_mutex, &events]() {
    std::lock_guard<std::mutex> lock(events_mutex);
    events.emplace_back("cycle");
    return true;
  };

  ASSERT_TRUE(scheduler.initialize({}));
  ASSERT_TRUE(register_operations(scheduler, ops));
  ASSERT_TRUE(scheduler.step(2));

  ASSERT_EQ(events.size(), 2u);
  EXPECT_EQ(events[0], "cycle");
  EXPECT_EQ(events[1], "cycle");
  EXPECT_EQ(ops.cycles.load(), 2);

  ASSERT_TRUE(scheduler.shutdown());
}

TEST(SimulationSchedulerTest, ResetRequestRunsOnWorkerThread) {
  SimulationScheduler scheduler;
  FakeSchedulerOperations ops;
  std::promise<std::thread::id> reset_thread_promise;
  std::future<std::thread::id> reset_thread_future = reset_thread_promise.get_future();
  const std::thread::id caller_thread = std::this_thread::get_id();

  ops.reset_callback = [&reset_thread_promise](const ResetRequest&) {
    reset_thread_promise.set_value(std::this_thread::get_id());
    return true;
  };

  ASSERT_TRUE(scheduler.initialize({}));
  ASSERT_TRUE(register_operations(scheduler, ops));
  ASSERT_TRUE(scheduler.start());
  ASSERT_TRUE(scheduler.request_reset());

  ASSERT_EQ(reset_thread_future.wait_for(1s), std::future_status::ready);
  EXPECT_NE(reset_thread_future.get(), caller_thread);

  ASSERT_TRUE(scheduler.stop());
  ASSERT_TRUE(scheduler.shutdown());
}

TEST(SimulationSchedulerTest, WaitableResetReturnsExecutionFailure) {
  SimulationScheduler scheduler;
  FakeSchedulerOperations ops;
  ops.reset_callback = [](const ResetRequest&) { return false; };

  ASSERT_TRUE(scheduler.initialize({}));
  ASSERT_TRUE(register_operations(scheduler, ops));
  ASSERT_TRUE(scheduler.start());

  std::future<bool> completion = scheduler.request_reset_waitable();
  ASSERT_EQ(completion.wait_for(1s), std::future_status::ready);

  EXPECT_FALSE(completion.get());

  ASSERT_TRUE(scheduler.stop());
  ASSERT_TRUE(scheduler.shutdown());
}

TEST(SimulationSchedulerTest, WaitableResetReturnsThreadFailureWhenCallbackThrows) {
  SimulationScheduler scheduler;
  FakeSchedulerOperations ops;
  ops.reset_callback = [](const ResetRequest&) -> bool {
    throw std::runtime_error("reset callback boom");
  };

  ASSERT_TRUE(scheduler.initialize({}));
  ASSERT_TRUE(register_operations(scheduler, ops));
  ASSERT_TRUE(scheduler.start());

  std::future<bool> completion = scheduler.request_reset_waitable();
  ASSERT_EQ(completion.wait_for(1s), std::future_status::ready);

  EXPECT_FALSE(completion.get());
  EXPECT_EQ(scheduler.status(), SimulationStatus::Error);

  ASSERT_TRUE(scheduler.stop());
  ASSERT_TRUE(scheduler.shutdown());
}

TEST(SimulationSchedulerTest, WorkerRunCycleFailureTransitionsSchedulerToError) {
  SimulationScheduler scheduler;
  FakeSchedulerOperations ops;
  ops.run_cycle_callback = []() -> bool { throw std::runtime_error("physics worker boom"); };

  ASSERT_TRUE(scheduler.initialize({}));
  ASSERT_TRUE(register_operations(scheduler, ops));
  ASSERT_TRUE(scheduler.start());

  for (int attempt = 0; attempt < 100 && scheduler.status() != SimulationStatus::Error; ++attempt) {
    std::this_thread::sleep_for(2ms);
  }

  EXPECT_EQ(scheduler.status(), SimulationStatus::Error);
  EXPECT_GT(ops.cycles.load(), 0);
  ASSERT_TRUE(scheduler.stop());
  ASSERT_TRUE(scheduler.shutdown());
}

TEST(SimulationSchedulerTest, RealtimeFactorCanBeUpdatedAfterInitialize) {
  SimulationScheduler scheduler;
  FakeSchedulerOperations ops;
  SchedulerConfig config;
  config.realtime_factor = 1.0;

  ASSERT_TRUE(scheduler.initialize(config));
  ASSERT_TRUE(register_operations(scheduler, ops));
  EXPECT_DOUBLE_EQ(scheduler.realtime_factor(), 1.0);

  ASSERT_TRUE(scheduler.set_realtime_factor(2.5));
  EXPECT_DOUBLE_EQ(scheduler.realtime_factor(), 2.5);

  ASSERT_TRUE(scheduler.start());
  std::this_thread::sleep_for(10ms);
  ASSERT_TRUE(scheduler.set_realtime_factor(0.5));
  EXPECT_DOUBLE_EQ(scheduler.realtime_factor(), 0.5);

  ASSERT_TRUE(scheduler.stop());
  ASSERT_TRUE(scheduler.shutdown());
}

TEST(SimulationSchedulerTest, RejectsInvalidRealtimeFactorUpdates) {
  SimulationScheduler scheduler;
  FakeSchedulerOperations ops;

  EXPECT_FALSE(scheduler.set_realtime_factor(0.0));

  ASSERT_TRUE(scheduler.initialize({}));
  ASSERT_TRUE(register_operations(scheduler, ops));

  EXPECT_FALSE(scheduler.set_realtime_factor(0.0));
  EXPECT_FALSE(scheduler.set_realtime_factor(-1.0));

  ASSERT_TRUE(scheduler.shutdown());
}

TEST(SimulationSchedulerTest, RejectsRegisteringOperationsBeforeInitializeOrAfterStart) {
  SimulationScheduler scheduler;
  FakeSchedulerOperations ops;

  EXPECT_FALSE(scheduler.register_timestep_provider([&ops]() { return ops.timestep; }));
  EXPECT_FALSE(scheduler.register_cycle_runner([]() { return true; }));
  EXPECT_FALSE(scheduler.register_reset_handler([](const ResetRequest&) { return true; }));

  ASSERT_TRUE(scheduler.initialize({}));
  ASSERT_TRUE(register_operations(scheduler, ops));
  ASSERT_TRUE(scheduler.start());

  EXPECT_FALSE(scheduler.register_timestep_provider([&ops]() { return ops.timestep; }));
  EXPECT_FALSE(scheduler.register_cycle_runner([]() { return true; }));
  EXPECT_FALSE(scheduler.register_reset_handler([](const ResetRequest&) { return true; }));

  ASSERT_TRUE(scheduler.stop());
  ASSERT_TRUE(scheduler.shutdown());
}

}  // namespace mujoco_simulation
