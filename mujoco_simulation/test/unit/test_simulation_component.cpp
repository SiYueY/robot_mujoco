#include <gtest/gtest.h>

#include "mujoco_simulation/component/simulation_component.hpp"
#include "mujoco_simulation/component/update_context.hpp"

namespace mujoco_simulation {
namespace {

class FakeComponent final : public SimulationComponent {
 public:
  ResultCode configure_periodic(double update_rate, double physics_rate) {
    return set_update_rate(update_rate, physics_rate);
  }

  void configure_stepwise() { set_update_every_step(); }

  std::string name() const noexcept override { return "fake"; }
  ResultCode bind(const mjModel& model) override {
    (void)model;
    return ResultCode::Ok;
  }
  ResultCode reset(const mjModel& model, mjData& data) override {
    (void)model;
    (void)data;
    return ResultCode::Ok;
  }
  ResultCode update(const UpdateContext& context) override {
    (void)context;
    return ResultCode::Ok;
  }
};

TEST(SimulationComponentTest, RejectsUpdateRateAbovePhysicsRate) {
  FakeComponent component;
  EXPECT_EQ(component.configure_periodic(2000.0, 1000.0), ResultCode::InvalidArgument);
}

TEST(SimulationComponentTest, UpdatesOnStableCadenceAcrossSteps) {
  FakeComponent component;
  ASSERT_EQ(component.configure_periodic(200.0, 1000.0), ResultCode::Ok);

  EXPECT_TRUE(component.should_update(0.0));
  EXPECT_FALSE(component.should_update(0.004));
  EXPECT_TRUE(component.should_update(0.005));
  EXPECT_FALSE(component.should_update(0.009));
  EXPECT_TRUE(component.should_update(0.01));
}

TEST(SimulationComponentTest, ResetMakesComponentsImmediatelyDueAgain) {
  FakeComponent component;
  ASSERT_EQ(component.configure_periodic(10.0, 1000.0), ResultCode::Ok);
  EXPECT_TRUE(component.should_update(0.0));
  EXPECT_FALSE(component.should_update(0.05));

  component.reset_update_schedule();
  EXPECT_TRUE(component.should_update(0.0));
}

TEST(SimulationComponentTest, TracksMissedUpdatesWhenUpdatingFallsBehind) {
  FakeComponent component;
  ASSERT_EQ(component.configure_periodic(20.0, 1000.0), ResultCode::Ok);

  EXPECT_TRUE(component.should_update(0.0));
  EXPECT_EQ(component.missed_updates(), 0U);

  EXPECT_TRUE(component.should_update(0.16));
  EXPECT_EQ(component.missed_updates(), 2U);

  component.reset_update_schedule();
  EXPECT_EQ(component.missed_updates(), 0U);
  EXPECT_TRUE(component.should_update(0.0));
}

TEST(SimulationComponentTest, StepwiseComponentsAlwaysUpdate) {
  FakeComponent component;
  component.configure_stepwise();

  EXPECT_TRUE(component.should_update(0.0));
  EXPECT_TRUE(component.should_update(0.001));
  EXPECT_TRUE(component.should_update(1.0));
}

}  // namespace
}  // namespace mujoco_simulation
