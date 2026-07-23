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
    if (simulation.step() >= target_step_count) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return simulation.step() >= target_step_count;
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
  ASSERT_TRUE(simulation.initialize(config));
  RobotCommand command;
  command.joint_commands.emplace("slide", JointCommand{.joint_name = "ignored",
                                                       .mode = JointControlMode::Hybrid,
                                                       .position = 1.0,
                                                       .effort = 1.0,
                                                       .stiffness = 2.0,
                                                       .damping = 1.0});
  ASSERT_TRUE(simulation.write_command(command));
  ASSERT_TRUE(simulation.start());
  ASSERT_TRUE(wait_for_step_count(simulation, 1));
  ASSERT_TRUE(simulation.stop());
  JointState state;
  ASSERT_TRUE(simulation.read_state("slide", state));
  EXPECT_EQ(state.mode, JointControlMode::Hybrid);

  const std::uint64_t first_step = simulation.step();
  EXPECT_GE(first_step, 1U);
  std::shared_ptr<const RobotState> snapshot_after_step;
  ASSERT_TRUE(simulation.read_state(snapshot_after_step));
  EXPECT_EQ(snapshot_after_step->step, first_step);
  RobotState copied_snapshot;
  ASSERT_TRUE(simulation.read_state(copied_snapshot));
  EXPECT_EQ(copied_snapshot.sequence, snapshot_after_step->sequence);
  EXPECT_EQ(copied_snapshot.step, snapshot_after_step->step);

  ASSERT_TRUE(simulation.reset());

  ASSERT_TRUE(simulation.start());
  ASSERT_TRUE(wait_for_step_count(simulation, first_step + 1));
  ASSERT_TRUE(simulation.stop());
  EXPECT_GE(simulation.step(), first_step + 1);
  std::error_code remove_error;
  std::filesystem::remove(path, remove_error);
}

TEST(BufferedSimulationTest, FullStateReadsFailBeforeInitialization) {
  Simulation simulation;
  std::shared_ptr<const RobotState> shared_state = std::make_shared<RobotState>();
  EXPECT_FALSE(simulation.read_state(shared_state));
  EXPECT_EQ(shared_state, nullptr);

  RobotState copied_state{.sequence = 42};
  EXPECT_FALSE(simulation.read_state(copied_state));
  EXPECT_EQ(copied_state.sequence, 42U);
}

}  // namespace
}  // namespace mujoco_simulation
