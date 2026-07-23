#include <gtest/gtest.h>

#include "mujoco_simulation/buffer/command_buffer.hpp"

namespace mujoco_simulation {
namespace {

TEST(CommandBufferTest, SingleComponentWritesPreserveLatestCommands) {
  CommandBuffer buffer;

  ASSERT_TRUE(buffer.write_joint_command("joint", {.joint_name = "joint",
                                                   .mode = JointControlMode::Velocity,
                                                   .position = 1.0,
                                                   .velocity = 2.0,
                                                   .effort = 3.0}));
  ASSERT_TRUE(buffer.write_mobile_base_command("base", {.mobile_base_name = "base",
                                                        .mode = MobileBaseControlMode::WheelLinear,
                                                        .wheel_linear = {1.0, 2.0, 3.0, 4.0}}));

  const RobotCommand snapshot = buffer.read();

  const auto joint_it = snapshot.joint_commands.find("joint");
  ASSERT_NE(joint_it, snapshot.joint_commands.end());
  EXPECT_DOUBLE_EQ(joint_it->second.velocity, 2.0);
  EXPECT_DOUBLE_EQ(joint_it->second.effort, 3.0);
  const auto mobile_base_it = snapshot.mobile_base_commands.find("base");
  ASSERT_NE(mobile_base_it, snapshot.mobile_base_commands.end());
  EXPECT_EQ(mobile_base_it->second.mobile_base_name, "base");
  EXPECT_EQ(mobile_base_it->second.mode, MobileBaseControlMode::WheelLinear);
  EXPECT_EQ(mobile_base_it->second.wheel_linear, (Vector4d{1.0, 2.0, 3.0, 4.0}));
}

TEST(CommandBufferTest, BatchWriteAtomicallyReplacesAndNormalizesCommands) {
  CommandBuffer buffer;
  ASSERT_TRUE(buffer.write_joint_command(
      "stale", {.joint_name = "stale", .mode = JointControlMode::Velocity, .velocity = 1.0}));
  const std::uint64_t sequence_before_batch = buffer.read().sequence;

  RobotCommand command;
  command.joint_commands.emplace(
      "joint",
      JointCommand{.joint_name = "ignored", .mode = JointControlMode::Position, .position = 1.5});
  command.mobile_base_commands.emplace("base",
                                       MobileBaseCommand{.mobile_base_name = "ignored",
                                                         .mode = MobileBaseControlMode::WheelLinear,
                                                         .wheel_linear = {1.0, 2.0, 3.0, 4.0}});
  ASSERT_TRUE(buffer.write_command(command));

  const RobotCommand snapshot = buffer.read();
  EXPECT_EQ(snapshot.sequence, sequence_before_batch + 1);
  EXPECT_EQ(snapshot.joint_commands.size(), 1U);
  EXPECT_EQ(snapshot.mobile_base_commands.size(), 1U);
  EXPECT_EQ(snapshot.joint_commands.at("joint").joint_name, "joint");
  EXPECT_EQ(snapshot.mobile_base_commands.at("base").mobile_base_name, "base");
  EXPECT_EQ(snapshot.joint_commands.count("stale"), 0U);
}

TEST(CommandBufferTest, BatchWriteRejectsEmptyNamesWithoutChangingCommands) {
  CommandBuffer buffer;
  ASSERT_TRUE(buffer.write_joint_command(
      "joint", {.joint_name = "joint", .mode = JointControlMode::Position, .position = 1.5}));
  const RobotCommand before = buffer.read();

  RobotCommand invalid;
  invalid.joint_commands.emplace("", JointCommand{});
  EXPECT_FALSE(buffer.write_command(invalid));

  const RobotCommand after = buffer.read();
  EXPECT_EQ(after.sequence, before.sequence);
  ASSERT_EQ(after.joint_commands.size(), 1U);
  EXPECT_DOUBLE_EQ(after.joint_commands.at("joint").position, 1.5);
}

TEST(CommandBufferTest, EmptyBatchClearsAllCommands) {
  CommandBuffer buffer;
  ASSERT_TRUE(buffer.write_joint_command(
      "joint", {.joint_name = "joint", .mode = JointControlMode::Position, .position = 1.5}));
  const std::uint64_t sequence_before_clear = buffer.read().sequence;

  ASSERT_TRUE(buffer.write_command({}));
  const RobotCommand snapshot = buffer.read();
  EXPECT_EQ(snapshot.sequence, sequence_before_clear + 1);
  EXPECT_TRUE(snapshot.joint_commands.empty());
  EXPECT_TRUE(snapshot.mobile_base_commands.empty());
}

}  // namespace
}  // namespace mujoco_simulation
