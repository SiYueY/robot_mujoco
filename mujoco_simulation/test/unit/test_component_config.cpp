#include <gtest/gtest.h>

#include "mujoco_simulation/component/component_config.hpp"

namespace mujoco_simulation {
namespace {

TEST(ComponentConfigTest, ReplaceJointConfigUpdatesMatchingEntryOnly) {
  JointConfig first{
      .name = "joint_a",
      .actuator_name = "joint_a_act",
      .command_mode = CommandInterfaceType::Position,
  };
  JointConfig second{
      .name = "joint_b",
      .actuator_name = "joint_b_act",
      .command_mode = CommandInterfaceType::Velocity,
  };

  ComponentConfigList components{first, second};
  JointConfig updated = second;
  updated.command_mode = CommandInterfaceType::Effort;

  EXPECT_TRUE(replace_joint_config(components, updated));
  ASSERT_EQ(std::get<JointConfig>(components[0]).command_mode, CommandInterfaceType::Position);
  ASSERT_EQ(std::get<JointConfig>(components[1]).command_mode, CommandInterfaceType::Effort);
}

TEST(ComponentConfigTest, ReplaceJointConfigReturnsFalseWhenMissing) {
  ComponentConfigList components{JointConfig{
      .name = "joint_a",
      .actuator_name = "joint_a_act",
      .command_mode = CommandInterfaceType::Position,
  }};

  EXPECT_FALSE(replace_joint_config(components, JointConfig{
                                                    .name = "joint_missing",
                                                    .actuator_name = "joint_missing_act",
                                                    .command_mode = CommandInterfaceType::Velocity,
                                                }));
}

TEST(ComponentConfigTest, ReplaceComponentConfigMatchesByTypeAndSemanticName) {
  ComponentConfigList components{
      CameraConfig{.common = {.name = "front_camera", .update_rate = 30.0},
                   .camera_name = "cam_front",
                   .height = 480,
                   .width = 640},
      ImuConfig{.common = {.name = "imu", .update_rate = 200.0},
                .framequat_sensor_name = "imu_quat",
                .gyro_sensor_name = "imu_gyro",
                .accelerometer_sensor_name = "imu_acc"},
  };

  ComponentConfig updated_camera =
      CameraConfig{.common = {.name = "front_camera", .update_rate = 60.0},
                   .camera_name = "cam_front",
                   .height = 240,
                   .width = 320};

  EXPECT_TRUE(replace_component_config(components, updated_camera));
  const auto* camera = std::get_if<CameraConfig>(&components.front());
  ASSERT_NE(camera, nullptr);
  EXPECT_EQ(camera->common.update_rate, 60.0);
  EXPECT_EQ(camera->width, 320);
  EXPECT_EQ(camera->height, 240);

  const auto* imu = std::get_if<ImuConfig>(&components.back());
  ASSERT_NE(imu, nullptr);
  EXPECT_EQ(imu->common.update_rate, 200.0);
}

TEST(ComponentConfigTest, ReplaceComponentConfigRejectsDifferentTypeWithSameName) {
  ComponentConfigList components{
      CameraConfig{.common = {.name = "shared_name", .update_rate = 30.0},
                   .camera_name = "cam_front",
                   .height = 480,
                   .width = 640}};

  EXPECT_FALSE(replace_component_config(
      components, ComponentConfig{ImuConfig{.common = {.name = "shared_name", .update_rate = 200.0},
                                            .framequat_sensor_name = "imu_quat",
                                            .gyro_sensor_name = "imu_gyro",
                                            .accelerometer_sensor_name = "imu_acc"}}));
}

}  // namespace
}  // namespace mujoco_simulation
