#include <gtest/gtest.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>

#include "mujoco_simulation/simulation.hpp"

namespace mujoco_simulation {
namespace {

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
  config.components = {JointInfo{.joint = "slide",
                                 .actuator = "slide_motor",
                                 .position_stiffness = 10.0,
                                 .position_damping = 1.0,
                                 .velocity_damping = 2.0}};
  ASSERT_EQ(simulation.initialize(config), ResultCode::Ok);
  ASSERT_EQ(simulation.set_joint_command({.joint = "slide",
                                          .mode = ControlMode::Hybrid,
                                          .position = 1.0,
                                          .effort = 1.0,
                                          .stiffness = 2.0,
                                          .damping = 1.0}),
            ResultCode::Ok);
  ASSERT_EQ(simulation.step(1), ResultCode::Ok);
  JointState state;
  ASSERT_TRUE(simulation.joint_state("slide", &state));
  EXPECT_EQ(state.mode, ControlMode::Hybrid);
  std::error_code remove_error;
  std::filesystem::remove(path, remove_error);
}

}  // namespace
}  // namespace mujoco_simulation
