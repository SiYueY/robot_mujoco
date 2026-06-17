#include <gtest/gtest.h>

#include "mujoco_simulation/scheduler/sensor_scheduler.hpp"

namespace mujoco_simulation {
namespace {

TEST(SensorSchedulerTest, RejectsUpdateRateAbovePhysicsRate) {
  SensorScheduler scheduler;
  const Status status = scheduler.register_sensor("imu", 2000.0, 1000.0);
  EXPECT_FALSE(status.ok());
}

TEST(SensorSchedulerTest, SamplesOnStableCadenceAcrossSteps) {
  SensorScheduler scheduler;
  ASSERT_TRUE(scheduler.register_sensor("imu", 200.0, 1000.0).ok());

  EXPECT_TRUE(scheduler.is_due("imu", 0.0));
  ASSERT_TRUE(scheduler.mark_sampled("imu", 0.0).ok());
  EXPECT_FALSE(scheduler.is_due("imu", 0.004));
  EXPECT_TRUE(scheduler.is_due("imu", 0.005));
  ASSERT_TRUE(scheduler.mark_sampled("imu", 0.005).ok());
  EXPECT_FALSE(scheduler.is_due("imu", 0.009));
  EXPECT_TRUE(scheduler.is_due("imu", 0.01));
}

TEST(SensorSchedulerTest, ResetMakesSensorsImmediatelyDueAgain) {
  SensorScheduler scheduler;
  ASSERT_TRUE(scheduler.register_sensor("lidar", 10.0, 1000.0).ok());
  ASSERT_TRUE(scheduler.mark_sampled("lidar", 0.0).ok());
  EXPECT_FALSE(scheduler.is_due("lidar", 0.05));

  scheduler.reset();
  EXPECT_TRUE(scheduler.is_due("lidar", 0.0));
}

TEST(SensorSchedulerTest, TracksMissedUpdatesWhenSamplingFallsBehind) {
  SensorScheduler scheduler;
  ASSERT_TRUE(scheduler.register_sensor("camera", 20.0, 1000.0).ok());

  ASSERT_TRUE(scheduler.mark_sampled("camera", 0.0).ok());
  EXPECT_EQ(scheduler.missed_updates("camera"), 0U);

  ASSERT_TRUE(scheduler.mark_sampled("camera", 0.16).ok());
  EXPECT_EQ(scheduler.missed_updates("camera"), 2U);

  scheduler.reset();
  EXPECT_EQ(scheduler.missed_updates("camera"), 0U);
  EXPECT_TRUE(scheduler.is_due("camera", 0.0));
}

}  // namespace
}  // namespace mujoco_simulation
