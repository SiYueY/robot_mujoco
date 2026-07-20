#include <gtest/gtest.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <limits>
#include <string>

#include "mujoco_simulation/component/simulation_component.hpp"
#include "mujoco_simulation/runtime/simulation_runtime.hpp"

namespace mujoco_simulation {

class SimulationRuntimeTestPeer {
 public:
  static const mjContext& context(const SimulationRuntime& runtime) { return runtime.context(); }
};

namespace {

class FakeComponent final : public SimulationComponent {
 public:
  FakeComponent(std::string name, double update_rate)
      : SimulationComponent(std::move(name), update_rate) {}

  bool init(const mjContext& context) override { return configure(context); }
  bool reset(const mjContext&) override { return true; }
  bool update(const mjContext&) override { return true; }
};

class SimulationComponentTest : public ::testing::Test {
 protected:
  void SetUp() override {
    model_path_ = std::filesystem::temp_directory_path() /
                  ("simulation_component_test_" + std::to_string(::getpid()) + ".xml");
    std::ofstream output(model_path_);
    ASSERT_TRUE(output.is_open());
    output
        << R"(<mujoco><option timestep=".001"/><worldbody><body><geom type="sphere" size=".1"/></body></worldbody></mujoco>)";
    output.close();

    ASSERT_TRUE(runtime_.init({model_path_}));
  }

  void TearDown() override {
    if (!model_path_.empty()) {
      std::error_code error;
      std::filesystem::remove(model_path_, error);
    }
  }

  const mjContext& context() const { return SimulationRuntimeTestPeer::context(runtime_); }

  std::filesystem::path model_path_;
  SimulationRuntime runtime_;
};

TEST_F(SimulationComponentTest, ValidatesNameAndUpdateRateDuringInitialization) {
  EXPECT_EQ(FakeComponent("fake", 0.0).name(), "fake");
  EXPECT_FALSE(mjContext{}.valid());
  EXPECT_FALSE(FakeComponent("fake", 0.0).init({}));
  EXPECT_FALSE(FakeComponent("", 0.0).init(context()));
  EXPECT_FALSE(FakeComponent("fake", -1.0).init(context()));
  EXPECT_FALSE(FakeComponent("fake", std::numeric_limits<double>::infinity()).init(context()));
  EXPECT_FALSE(FakeComponent("fake", 2000.0).init(context()));
  EXPECT_TRUE(FakeComponent("fake", 0.0).init(context()));
}

TEST_F(SimulationComponentTest, UpdatesOnStableCadenceAcrossSteps) {
  FakeComponent component("fake", 200.0);

  EXPECT_TRUE(component.poll_update(0.0));
  EXPECT_FALSE(component.poll_update(0.004));
  EXPECT_TRUE(component.poll_update(0.005));
  EXPECT_FALSE(component.poll_update(0.009));
  EXPECT_TRUE(component.poll_update(0.01));
}

TEST_F(SimulationComponentTest, ResetMakesComponentsImmediatelyDueAgain) {
  FakeComponent component("fake", 10.0);
  EXPECT_TRUE(component.poll_update(0.0));
  EXPECT_FALSE(component.poll_update(0.05));

  EXPECT_TRUE(component.reset_schedule());
  EXPECT_TRUE(component.poll_update(0.0));
}

TEST_F(SimulationComponentTest, AdvancesScheduleAfterTimeJump) {
  FakeComponent component("fake", 20.0);

  EXPECT_TRUE(component.poll_update(0.0));
  EXPECT_TRUE(component.poll_update(0.16));
  EXPECT_FALSE(component.poll_update(0.19));
  EXPECT_TRUE(component.poll_update(0.20));
}

TEST_F(SimulationComponentTest, DefaultScheduleUpdatesEveryStep) {
  FakeComponent component("fake", 0.0);

  EXPECT_TRUE(component.poll_update(0.0));
  EXPECT_TRUE(component.poll_update(0.001));
  EXPECT_TRUE(component.poll_update(1.0));
}

}  // namespace
}  // namespace mujoco_simulation
