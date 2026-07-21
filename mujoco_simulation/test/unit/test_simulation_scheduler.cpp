#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
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
  std::atomic<int> tasks{0};
  std::function<void()> after_task;
  std::function<bool()> run_task_callback;
};

bool register_operations(SimulationScheduler& scheduler, FakeSchedulerOperations& ops) {
  if (!scheduler.register_task([&ops]() {
        ++ops.tasks;
        if (ops.after_task) {
          ops.after_task();
        }
        if (ops.run_task_callback) {
          return ops.run_task_callback();
        }
        return true;
      })) {
    return false;
  }
  return true;
}

}  // namespace

TEST(SimulationSchedulerTest, RejectsStartBeforeInitialize) {
  SimulationScheduler scheduler;
  EXPECT_FALSE(scheduler.start());
}

TEST(SimulationSchedulerTest, ManualStepAllowedWhenStoppedAndRejectedWhenRunning) {
  SimulationScheduler scheduler;
  FakeSchedulerOperations ops;
  ASSERT_TRUE(scheduler.initialize());
  ASSERT_TRUE(register_operations(scheduler, ops));

  ASSERT_TRUE(scheduler.step(3));
  EXPECT_EQ(ops.tasks.load(), 3);

  ASSERT_TRUE(scheduler.start());
  std::this_thread::sleep_for(10ms);

  EXPECT_FALSE(scheduler.step());

  ASSERT_TRUE(scheduler.stop());
  ASSERT_TRUE(scheduler.shutdown());
}

TEST(SimulationSchedulerTest, PauseStopsPhysicsAndResumeRestartsIt) {
  SimulationScheduler scheduler;
  FakeSchedulerOperations ops;
  ASSERT_TRUE(scheduler.initialize());
  ASSERT_TRUE(register_operations(scheduler, ops));
  ASSERT_TRUE(scheduler.start());

  std::this_thread::sleep_for(20ms);
  ASSERT_TRUE(scheduler.pause());

  const int paused_count = ops.tasks.load();
  std::this_thread::sleep_for(20ms);
  EXPECT_EQ(ops.tasks.load(), paused_count);
  EXPECT_EQ(scheduler.status(), SimulationStatus::Paused);

  ASSERT_TRUE(scheduler.resume());
  std::this_thread::sleep_for(20ms);
  EXPECT_GT(ops.tasks.load(), paused_count);

  ASSERT_TRUE(scheduler.stop());
  ASSERT_TRUE(scheduler.shutdown());
}

TEST(SimulationSchedulerTest, ManualStepExecutesSingleTaskPerStep) {
  SimulationScheduler scheduler;
  std::mutex events_mutex;
  std::vector<std::string> events;
  FakeSchedulerOperations ops;
  ops.run_task_callback = [&events_mutex, &events]() {
    std::lock_guard<std::mutex> lock(events_mutex);
    events.emplace_back("task");
    return true;
  };

  ASSERT_TRUE(scheduler.initialize());
  ASSERT_TRUE(register_operations(scheduler, ops));
  ASSERT_TRUE(scheduler.step(2));

  ASSERT_EQ(events.size(), 2u);
  EXPECT_EQ(events[0], "task");
  EXPECT_EQ(events[1], "task");
  EXPECT_EQ(ops.tasks.load(), 2);

  ASSERT_TRUE(scheduler.shutdown());
}

TEST(SimulationSchedulerTest, WorkerTaskFailureTransitionsSchedulerToError) {
  SimulationScheduler scheduler;
  FakeSchedulerOperations ops;
  ops.run_task_callback = []() -> bool { throw std::runtime_error("physics worker boom"); };

  ASSERT_TRUE(scheduler.initialize());
  ASSERT_TRUE(register_operations(scheduler, ops));
  ASSERT_TRUE(scheduler.start());

  for (int attempt = 0; attempt < 100 && scheduler.status() != SimulationStatus::Error; ++attempt) {
    std::this_thread::sleep_for(2ms);
  }

  EXPECT_EQ(scheduler.status(), SimulationStatus::Error);
  EXPECT_GT(ops.tasks.load(), 0);
  ASSERT_TRUE(scheduler.stop());
  ASSERT_TRUE(scheduler.shutdown());
}

TEST(SimulationSchedulerTest, RejectsRegisteringOperationsBeforeInitializeOrAfterStart) {
  SimulationScheduler scheduler;
  FakeSchedulerOperations ops;

  EXPECT_FALSE(scheduler.register_task([]() { return true; }));

  ASSERT_TRUE(scheduler.initialize());
  ASSERT_TRUE(register_operations(scheduler, ops));
  ASSERT_TRUE(scheduler.start());

  EXPECT_FALSE(scheduler.register_task([]() { return true; }));

  ASSERT_TRUE(scheduler.stop());
  ASSERT_TRUE(scheduler.shutdown());
}

}  // namespace mujoco_simulation
