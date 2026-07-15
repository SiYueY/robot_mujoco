#include <gtest/gtest.h>

#include "mujoco_simulation/component/component_config.hpp"

namespace mujoco_simulation {
namespace {

TEST(ComponentConfigTest, ReplaceJointConfigUpdatesMatchingEntryOnly) {
  JointInfo first{
      .joint = "joint_a",
      .actuator = "joint_a_motor",
  };
  JointInfo second{
      .joint = "joint_b",
      .actuator = "joint_b_motor",
  };

  ComponentConfigList components{first, second};
  JointInfo updated = second;
  updated.effort_limits.max = 42.0;

  EXPECT_TRUE(replace_joint_info(components, updated));
  ASSERT_EQ(std::get<JointInfo>(components[0]).actuator, "joint_a_motor");
  ASSERT_EQ(std::get<JointInfo>(components[1]).effort_limits.max, 42.0);
}

TEST(ComponentConfigTest, ReplaceJointConfigReturnsFalseWhenMissing) {
  ComponentConfigList components{JointInfo{
      .joint = "joint_a",
      .actuator = "joint_a_motor",
  }};

  EXPECT_FALSE(replace_joint_info(components, JointInfo{
                                                  .joint = "joint_missing",
                                                  .actuator = "joint_missing_motor",
                                              }));
}

TEST(ComponentConfigTest, ReplaceComponentConfigMatchesByTypeAndSemanticName) {
  ComponentConfigList components{
      CameraConfig{.common = {.name = "front_camera", .update_rate = 30.0},
                   .camera_name = "cam_front",
                   .height = 480,
                   .width = 640},
      ImuInfo{.common = {.name = "imu", .update_rate = 200.0},
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

  const auto* imu = std::get_if<ImuInfo>(&components.back());
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
      components, ComponentConfig{ImuInfo{.common = {.name = "shared_name", .update_rate = 200.0},
                                          .framequat_sensor_name = "imu_quat",
                                          .gyro_sensor_name = "imu_gyro",
                                          .accelerometer_sensor_name = "imu_acc"}}));
}

}  // namespace
}  // namespace mujoco_simulation
