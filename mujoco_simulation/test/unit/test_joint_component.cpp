#include <gtest/gtest.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>

#include "mujoco_simulation/component/joint/joint_component.hpp"

namespace mujoco_simulation {
namespace {

class JointComponentTest : public ::testing::Test {
 protected:
  void TearDown() override {
    if (data_ != nullptr) mj_deleteData(data_);
    if (model_ != nullptr) mj_deleteModel(model_);
    std::error_code error;
    std::filesystem::remove(model_path_, error);
  }

  void load(const std::string& xml) {
    model_path_ = std::filesystem::temp_directory_path() /
                  ("joint_component_" + std::to_string(::getpid()) + ".xml");
    std::ofstream output(model_path_);
    output << xml;
    output.close();
    char error[1024] = {};
    model_ = mj_loadXML(model_path_.c_str(), nullptr, error, sizeof(error));
    ASSERT_NE(model_, nullptr) << error;
    data_ = mj_makeData(model_);
    ASSERT_NE(data_, nullptr);
    mj_forward(model_, data_);
  }

  mjModel* model_{nullptr};
  mjData* data_{nullptr};
  std::filesystem::path model_path_;
};

TEST_F(JointComponentTest, AllModesWriteEffortToMotor) {
  load(
      R"(<mujoco><worldbody><body><joint name="hinge" type="hinge"/><geom type="sphere" size=".1"/></body></worldbody><actuator><motor name="motor" joint="hinge" ctrlrange="-20 20"/></actuator></mujoco>)");
  JointComponent joint({.joint = "hinge",
                        .actuator = "motor",
                        .position_stiffness = 10.0,
                        .position_damping = 2.0,
                        .velocity_damping = 4.0});
  ASSERT_TRUE(joint.bind(*model_));
  ASSERT_TRUE(joint.write(*model_, *data_,
                          {.joint = "hinge", .mode = ControlMode::Position, .position = 1.0}));
  EXPECT_DOUBLE_EQ(data_->ctrl[joint.motor_id()], 10.0);
  ASSERT_TRUE(joint.write(*model_, *data_,
                          {.joint = "hinge", .mode = ControlMode::Velocity, .velocity = 2.0}));
  EXPECT_DOUBLE_EQ(data_->ctrl[joint.motor_id()], 8.0);
  ASSERT_TRUE(
      joint.write(*model_, *data_, {.joint = "hinge", .mode = ControlMode::Effort, .effort = 3.0}));
  EXPECT_DOUBLE_EQ(data_->ctrl[joint.motor_id()], 3.0);
  ASSERT_TRUE(joint.write(*model_, *data_,
                          {.joint = "hinge",
                           .mode = ControlMode::Hybrid,
                           .position = 1.0,
                           .velocity = 2.0,
                           .effort = 3.0,
                           .stiffness = 5.0,
                           .damping = 2.0}));
  EXPECT_DOUBLE_EQ(data_->ctrl[joint.motor_id()], 12.0);
}

TEST_F(JointComponentTest, PrismaticJointAndSafetyLimitsAreSupported) {
  load(
      R"(<mujoco><worldbody><body><joint name="slide" type="slide"/><geom type="sphere" size=".1"/></body></worldbody><actuator><motor name="motor" joint="slide" ctrlrange="-10 10"/></actuator></mujoco>)");
  JointComponent joint({.joint = "slide",
                        .actuator = "motor",
                        .velocity_damping = 4.0,
                        .velocity_limits = {.min = -1.0, .max = 1.0},
                        .effort_limits = {.min = -2.0, .max = 2.0}});
  ASSERT_TRUE(joint.bind(*model_));
  EXPECT_EQ(joint.joint_type(), JointType::Prismatic);
  ASSERT_TRUE(joint.write(*model_, *data_,
                          {.joint = "slide", .mode = ControlMode::Velocity, .velocity = 3.0}));
  EXPECT_DOUBLE_EQ(data_->ctrl[joint.motor_id()], 2.0);
  ASSERT_TRUE(
      joint.write(*model_, *data_, {.joint = "slide", .mode = ControlMode::Effort, .effort = 8.0}));
  EXPECT_DOUBLE_EQ(data_->ctrl[joint.motor_id()], 2.0);
}

TEST_F(JointComponentTest, RejectsNonMotorActuators) {
  load(
      R"(<mujoco><worldbody><body><joint name="hinge" type="hinge"/><geom type="sphere" size=".1"/></body></worldbody><actuator><position name="position" joint="hinge"/></actuator></mujoco>)");
  JointComponent joint({.joint = "hinge", .actuator = "position"});
  EXPECT_FALSE(joint.bind(*model_));
}

}  // namespace
}  // namespace mujoco_simulation
