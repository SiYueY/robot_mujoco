#include <gtest/gtest.h>

#include <chrono>
#include <thread>

#include "mujoco_simulation/buffer/command_buffer.hpp"

namespace mujoco_simulation {
namespace {

using namespace std::chrono_literals;

TEST(CommandBufferTest, VelocityAndMobileBaseCommandsZeroAfterTimeout) {
  CommandBuffer buffer;
  buffer.set_timeout_config(
      {.enabled = true, .timeout_seconds = 0.001, .behavior = CommandTimeoutBehavior::ZeroCommand});

  ASSERT_TRUE(buffer.set_joint_command("joint", {"joint", 1.0, 2.0, 0.0, 3.0}).ok());
  ASSERT_TRUE(buffer
                  .set_mobile_base_command("base", {.linear = {1.0, 2.0, 0.0},
                                                    .angular = {0.0, 0.0, 3.0},
                                                    .linear_x = 1.0,
                                                    .linear_y = 2.0,
                                                    .angular_z = 3.0})
                  .ok());

  std::this_thread::sleep_for(5ms);
  const CommandSnapshot snapshot = buffer.snapshot(
      CommandBuffer::Clock::now(), [](std::string_view) { return CommandInterfaceType::Velocity; });

  const auto joint_it = snapshot.joint_commands.find("joint");
  ASSERT_NE(joint_it, snapshot.joint_commands.end());
  EXPECT_DOUBLE_EQ(joint_it->second.velocity, 0.0);
  EXPECT_DOUBLE_EQ(joint_it->second.effort, 3.0);
  const auto mobile_base_it = snapshot.mobile_base_commands.find("base");
  ASSERT_NE(mobile_base_it, snapshot.mobile_base_commands.end());
  EXPECT_DOUBLE_EQ(mobile_base_it->second.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(mobile_base_it->second.angular_z, 0.0);
}

TEST(CommandBufferTest, PositionCommandsHoldLastTargetOnTimeout) {
  CommandBuffer buffer;
  buffer.set_timeout_config({.enabled = true,
                             .timeout_seconds = 0.001,
                             .behavior = CommandTimeoutBehavior::HoldPosition});

  ASSERT_TRUE(buffer.set_joint_command("joint", {"joint", 1.5, 0.0, 0.0, 0.0}).ok());
  std::this_thread::sleep_for(5ms);
  const CommandSnapshot snapshot = buffer.snapshot(
      CommandBuffer::Clock::now(), [](std::string_view) { return CommandInterfaceType::Position; });

  const auto joint_it = snapshot.joint_commands.find("joint");
  ASSERT_NE(joint_it, snapshot.joint_commands.end());
  EXPECT_DOUBLE_EQ(joint_it->second.position, 1.5);
}

}  // namespace
}  // namespace mujoco_simulation
