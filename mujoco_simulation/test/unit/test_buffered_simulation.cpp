#include <gtest/gtest.h>
#include <unistd.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <thread>

#include "mujoco_simulation/simulation.hpp"

namespace mujoco_simulation {
namespace {

bool wait_for_step_count(Simulation& simulation, std::uint64_t target_step_count,
                         std::chrono::milliseconds timeout = std::chrono::seconds(1)) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (simulation.step_count() >= target_step_count) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return simulation.step_count() >= target_step_count;
}

TEST(BufferedSimulationTest, HybridCommandIsBufferedAndPublishedWithItsMode) {
  const auto path = std::filesystem::temp_directory_path() /
                    ("buffered_simulation_" + std::to_string(::getpid()) + ".xml");
  std::ofstream output(path);
  output
      << R"(<mujoco><option timestep=".001"/><worldbody><body><joint name="slide" type="slide"/><geom type="sphere" size=".1"/></body></worldbody><actuator><motor name="slide_motor" joint="slide"/></actuator></mujoco>)";
  output.close();
  Simulation simulation;
  SimulationConfig config;
  config.model.model_path = path;
  config.components = {JointInfo{.joint_name = "slide",
                                 .actuator_name = "slide_motor",
                                 .position_stiffness = 10.0,
                                 .position_damping = 1.0,
                                 .velocity_damping = 2.0}};
  ASSERT_EQ(simulation.initialize(config), ResultCode::Ok);
  ASSERT_EQ(simulation.set_joint_command({.joint_name = "slide",
                                          .mode = JointControlMode::Hybrid,
                                          .position = 1.0,
                                          .effort = 1.0,
                                          .stiffness = 2.0,
                                          .damping = 1.0}),
            ResultCode::Ok);
  ASSERT_EQ(simulation.start(), ResultCode::Ok);
  ASSERT_TRUE(wait_for_step_count(simulation, 1));
  ASSERT_EQ(simulation.stop(), ResultCode::Ok);
  JointState state;
  ASSERT_TRUE(simulation.joint_state("slide", &state));
  EXPECT_EQ(state.mode, JointControlMode::Hybrid);

  EXPECT_EQ(simulation.step_count(), 1u);
  const std::shared_ptr<const StateSnapshot> snapshot_after_step = simulation.state_snapshot();
  ASSERT_NE(snapshot_after_step, nullptr);
  EXPECT_EQ(snapshot_after_step->step_count, 1u);

  ASSERT_EQ(simulation.reset(), ResultCode::Unimplemented);

  ASSERT_EQ(simulation.start(), ResultCode::Ok);
  ASSERT_TRUE(wait_for_step_count(simulation, 2));
  ASSERT_EQ(simulation.stop(), ResultCode::Ok);
  EXPECT_EQ(simulation.step_count(), 2u);
  std::error_code remove_error;
  std::filesystem::remove(path, remove_error);
}

}  // namespace
}  // namespace mujoco_simulation
