#include <gtest/gtest.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>

#include "mujoco_simulation/component/mobile_base/mobile_base_component.hpp"

namespace mujoco_simulation {
namespace {

TEST(MobileBaseTest, MecanumUsesWheelMotorEffortAndReportsWheelState) {
  const auto path = std::filesystem::temp_directory_path() /
                    ("mobile_base_" + std::to_string(::getpid()) + ".xml");
  std::ofstream output(path);
  output
      << R"(<mujoco><worldbody><body name="base"><joint name="front_left" type="hinge"/><geom type="sphere" size=".1"/></body><body><joint name="front_right" type="hinge"/><geom type="sphere" size=".1"/></body><body><joint name="rear_left" type="hinge"/><geom type="sphere" size=".1"/></body><body><joint name="rear_right" type="hinge"/><geom type="sphere" size=".1"/></body></worldbody><actuator><motor name="front_left_motor" joint="front_left"/><motor name="front_right_motor" joint="front_right"/><motor name="rear_left_motor" joint="rear_left"/><motor name="rear_right_motor" joint="rear_right"/></actuator></mujoco>)";
  output.close();
  char error[1024] = {};
  mjModel* model = mj_loadXML(path.c_str(), nullptr, error, sizeof(error));
  ASSERT_NE(model, nullptr) << error;
  mjData* data = mj_makeData(model);
  ASSERT_NE(data, nullptr);
  mjContext context(model, data);
  const int front_left_actuator = mj_name2id(model, mjOBJ_ACTUATOR, "front_left_motor");
  const int front_right_actuator = mj_name2id(model, mjOBJ_ACTUATOR, "front_right_motor");
  const int rear_left_actuator = mj_name2id(model, mjOBJ_ACTUATOR, "rear_left_motor");
  const int rear_right_actuator = mj_name2id(model, mjOBJ_ACTUATOR, "rear_right_motor");
  MobileBaseInfo info{.mobile_base_name = "base",
                      .type = MobileBaseType::Mecanum,
                      .base_body_name = "base",
                      .mecanum_info = {.wheel_radius = 0.2, .wheel_base = 0.4, .track_width = 0.6}};
  info.mecanum_wheels[to_integer(MecanumWheelIndex::FrontLeft)] = {
      .wheel_name = "front_left", .actuator_name = "front_left_motor", .damping = 2.0};
  info.mecanum_wheels[to_integer(MecanumWheelIndex::FrontRight)] = {
      .wheel_name = "front_right", .actuator_name = "front_right_motor", .damping = 2.0};
  info.mecanum_wheels[to_integer(MecanumWheelIndex::RearLeft)] = {
      .wheel_name = "rear_left", .actuator_name = "rear_left_motor", .damping = 2.0};
  info.mecanum_wheels[to_integer(MecanumWheelIndex::RearRight)] = {
      .wheel_name = "rear_right", .actuator_name = "rear_right_motor", .damping = 2.0};
  MobileBaseComponent base(info);
  ASSERT_TRUE(base.init(context));
  ASSERT_TRUE(base.write(context, {.mobile_base_name = "base", .base_linear = {1.0, 0.0, 0.0}}));
  EXPECT_GT(data->ctrl[front_left_actuator], 0.0);
  EXPECT_GT(data->ctrl[front_right_actuator], 0.0);
  EXPECT_GT(data->ctrl[rear_left_actuator], 0.0);
  EXPECT_GT(data->ctrl[rear_right_actuator], 0.0);

  ASSERT_TRUE(base.write(context, {.mobile_base_name = "base",
                                   .mode = MobileBaseControlMode::WheelLinear,
                                   .wheel_linear = {1.0, 1.0, 1.0, 1.0}}));
  EXPECT_NEAR(data->ctrl[front_left_actuator], 10.0, 1e-12);
  EXPECT_NEAR(data->ctrl[front_right_actuator], 10.0, 1e-12);
  EXPECT_NEAR(data->ctrl[rear_left_actuator], 10.0, 1e-12);
  EXPECT_NEAR(data->ctrl[rear_right_actuator], 10.0, 1e-12);

  ASSERT_TRUE(base.write(context, {.mobile_base_name = "base",
                                   .mode = MobileBaseControlMode::WheelAngular,
                                   .wheel_angular = {2.0, 2.0, 2.0, 2.0}}));
  EXPECT_NEAR(data->ctrl[front_left_actuator], 4.0, 1e-12);
  EXPECT_NEAR(data->ctrl[front_right_actuator], 4.0, 1e-12);
  EXPECT_NEAR(data->ctrl[rear_left_actuator], 4.0, 1e-12);
  EXPECT_NEAR(data->ctrl[rear_right_actuator], 4.0, 1e-12);

  data->qvel[model->jnt_dofadr[mj_name2id(model, mjOBJ_JOINT, "front_left")]] = 5.0;
  data->qvel[model->jnt_dofadr[mj_name2id(model, mjOBJ_JOINT, "front_right")]] = 5.0;
  data->qvel[model->jnt_dofadr[mj_name2id(model, mjOBJ_JOINT, "rear_left")]] = 5.0;
  data->qvel[model->jnt_dofadr[mj_name2id(model, mjOBJ_JOINT, "rear_right")]] = 5.0;
  data->time = 0.25;
  ASSERT_TRUE(base.update(context));
  MobileBaseState state;
  ASSERT_TRUE(base.read(context, state));
  EXPECT_EQ(state.mobile_base_name, "base");
  EXPECT_DOUBLE_EQ(state.timestamp, 0.25);
  EXPECT_EQ(state.pose, (Vector3d{0.0, 0.0, 0.0}));
  EXPECT_NEAR(state.base_linear[0], 1.0, 1e-12);
  EXPECT_NEAR(state.base_linear[1], 0.0, 1e-12);
  EXPECT_NEAR(state.base_angular[2], 0.0, 1e-12);
  EXPECT_EQ(state.wheel_angular, (Vector4d{5.0, 5.0, 5.0, 5.0}));
  EXPECT_EQ(state.wheel_linear, (Vector4d{1.0, 1.0, 1.0, 1.0}));

  model->actuator_forcelimited[front_left_actuator] = 1;
  model->actuator_forcerange[2 * front_left_actuator] = -3.0;
  model->actuator_forcerange[2 * front_left_actuator + 1] = 3.0;
  model->actuator_ctrllimited[front_left_actuator] = 1;
  model->actuator_ctrlrange[2 * front_left_actuator] = -2.0;
  model->actuator_ctrlrange[2 * front_left_actuator + 1] = 2.0;
  data->qvel[model->jnt_dofadr[mj_name2id(model, mjOBJ_JOINT, "front_left")]] = 0.0;
  ASSERT_TRUE(base.write(context, {.mobile_base_name = "base",
                                   .mode = MobileBaseControlMode::WheelAngular,
                                   .wheel_angular = {5.0, 0.0, 0.0, 0.0}}));
  EXPECT_DOUBLE_EQ(data->ctrl[front_left_actuator], 2.0);

  ASSERT_TRUE(base.reset(context));
  EXPECT_DOUBLE_EQ(data->ctrl[front_left_actuator], 0.0);
  EXPECT_DOUBLE_EQ(data->ctrl[front_right_actuator], 0.0);
  EXPECT_DOUBLE_EQ(data->ctrl[rear_left_actuator], 0.0);
  EXPECT_DOUBLE_EQ(data->ctrl[rear_right_actuator], 0.0);

  model->actuator_gainprm[front_left_actuator * mjNGAIN] = 2.0;
  EXPECT_FALSE(base.init(context));
  EXPECT_FALSE(base.is_initialized());
  EXPECT_FALSE(base.write(context, {.mobile_base_name = "base"}));
  std::error_code remove_error;
  std::filesystem::remove(path, remove_error);
}

}  // namespace
}  // namespace mujoco_simulation
