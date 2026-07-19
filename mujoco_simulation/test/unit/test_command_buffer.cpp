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

  ASSERT_TRUE(buffer.write_joint_command("joint", {.joint_name = "joint",
                                                   .mode = JointControlMode::Velocity,
                                                   .position = 1.0,
                                                   .velocity = 2.0,
                                                   .effort = 3.0}));
  ASSERT_TRUE(buffer.write_mobile_base_command("base", {.mobile_base_name = "base",
                                                        .mode = MobileBaseControlMode::WheelLinear,
                                                        .wheel_linear = {1.0, 2.0, 3.0, 4.0}}));

  std::this_thread::sleep_for(5ms);
  const CommandSnapshot snapshot = buffer.read(CommandBuffer::Clock::now());

  const auto joint_it = snapshot.joint_commands.find("joint");
  ASSERT_NE(joint_it, snapshot.joint_commands.end());
  EXPECT_DOUBLE_EQ(joint_it->second.velocity, 0.0);
  EXPECT_DOUBLE_EQ(joint_it->second.effort, 3.0);
  const auto mobile_base_it = snapshot.mobile_base_commands.find("base");
  ASSERT_NE(mobile_base_it, snapshot.mobile_base_commands.end());
  EXPECT_EQ(mobile_base_it->second.mobile_base_name, "base");
  EXPECT_EQ(mobile_base_it->second.mode, MobileBaseControlMode::WheelLinear);
  EXPECT_DOUBLE_EQ(mobile_base_it->second.base_linear[0], 0.0);
  EXPECT_DOUBLE_EQ(mobile_base_it->second.base_angular[2], 0.0);
  EXPECT_EQ(mobile_base_it->second.wheel_linear, (Vector4d{0.0, 0.0, 0.0, 0.0}));
  EXPECT_EQ(mobile_base_it->second.wheel_angular, (Vector4d{0.0, 0.0, 0.0, 0.0}));
}

TEST(CommandBufferTest, PositionCommandsHoldLastTargetOnTimeout) {
  CommandBuffer buffer;
  buffer.set_timeout_config({.enabled = true,
                             .timeout_seconds = 0.001,
                             .behavior = CommandTimeoutBehavior::HoldPosition});

  ASSERT_TRUE(buffer.write_joint_command(
      "joint", {.joint_name = "joint", .mode = JointControlMode::Position, .position = 1.5}));
  std::this_thread::sleep_for(5ms);
  const CommandSnapshot snapshot = buffer.read(CommandBuffer::Clock::now());

  const auto joint_it = snapshot.joint_commands.find("joint");
  ASSERT_NE(joint_it, snapshot.joint_commands.end());
  EXPECT_DOUBLE_EQ(joint_it->second.position, 1.5);
}

}  // namespace
}  // namespace mujoco_simulation
