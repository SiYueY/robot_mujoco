#include <gtest/gtest.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <limits>

#include "mujoco_simulation/component/joint/joint_component.hpp"

namespace mujoco_simulation {
namespace {

class JointComponentTest : public ::testing::Test {
 protected:
  void TearDown() override {
    context_.clear();
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
    mjModel* model = mj_loadXML(model_path_.c_str(), nullptr, error, sizeof(error));
    ASSERT_NE(model, nullptr) << error;
    mjData* data = mj_makeData(model);
    ASSERT_NE(data, nullptr);
    mj_forward(model, data);
    context_ = mjContext(model, data);
  }

  const mjContext& context() const { return context_; }
  mjModel* model() const { return const_cast<mjModel*>(context_.model); }
  mjData* data() const { return context_.data; }

  mjContext context_{};
  std::filesystem::path model_path_;
};

TEST_F(JointComponentTest, AllModesWriteEffortToMotor) {
  load(
      R"(<mujoco><worldbody><body><joint name="hinge" type="hinge"/><geom type="sphere" size=".1"/></body></worldbody><actuator><motor name="motor" joint="hinge" ctrlrange="-20 20"/></actuator></mujoco>)");
  JointComponent joint({.joint_name = "hinge",
                        .actuator_name = "motor",
                        .position_stiffness = 10.0,
                        .position_damping = 2.0,
                        .velocity_damping = 4.0});
  EXPECT_FALSE(joint.is_initialized());
  ASSERT_TRUE(joint.init(context()));
  EXPECT_TRUE(joint.is_initialized());
  ASSERT_TRUE(joint.write(
      context(), {.joint_name = "hinge", .mode = JointControlMode::Position, .position = 1.0}));
  EXPECT_DOUBLE_EQ(data()->ctrl[joint.actuator_id()], 10.0);
  ASSERT_TRUE(joint.write(
      context(), {.joint_name = "hinge", .mode = JointControlMode::Velocity, .velocity = 2.0}));
  EXPECT_DOUBLE_EQ(data()->ctrl[joint.actuator_id()], 8.0);
  ASSERT_TRUE(joint.write(
      context(), {.joint_name = "hinge", .mode = JointControlMode::Effort, .effort = 3.0}));
  EXPECT_DOUBLE_EQ(data()->ctrl[joint.actuator_id()], 3.0);
  ASSERT_TRUE(joint.write(context(), {.joint_name = "hinge",
                                      .mode = JointControlMode::Hybrid,
                                      .position = 1.0,
                                      .velocity = 2.0,
                                      .effort = 3.0,
                                      .stiffness = 5.0,
                                      .damping = 2.0}));
  EXPECT_DOUBLE_EQ(data()->ctrl[joint.actuator_id()], 12.0);
}

TEST_F(JointComponentTest, PrismaticJointAndSafetyLimitsAreSupported) {
  load(
      R"(<mujoco><worldbody><body><joint name="slide" type="slide"/><geom type="sphere" size=".1"/></body></worldbody><actuator><motor name="motor" joint="slide" ctrlrange="-10 10"/></actuator></mujoco>)");
  JointComponent joint({.joint_name = "slide",
                        .actuator_name = "motor",
                        .velocity_damping = 4.0,
                        .velocity_limits = {.min = -1.0, .max = 1.0},
                        .effort_limits = {.min = -2.0, .max = 2.0}});
  ASSERT_TRUE(joint.init(context()));
  EXPECT_EQ(joint.joint_type(), JointType::Prismatic);
  ASSERT_TRUE(joint.write(
      context(), {.joint_name = "slide", .mode = JointControlMode::Velocity, .velocity = 3.0}));
  EXPECT_DOUBLE_EQ(data()->ctrl[joint.actuator_id()], 2.0);
  ASSERT_TRUE(joint.write(
      context(), {.joint_name = "slide", .mode = JointControlMode::Effort, .effort = 8.0}));
  EXPECT_DOUBLE_EQ(data()->ctrl[joint.actuator_id()], 2.0);
}

TEST_F(JointComponentTest, ReadReturnsStateCachedByUpdate) {
  load(
      R"(<mujoco><worldbody><body><joint name="hinge" type="hinge"/><geom type="sphere" size=".1"/></body></worldbody><actuator><motor name="motor" joint="hinge"/></actuator></mujoco>)");
  JointComponent joint({.joint_name = "hinge", .actuator_name = "motor"});
  data()->time = 0.125;
  ASSERT_TRUE(joint.init(context()));

  const int qpos_address = model()->jnt_qposadr[joint.joint_id()];
  const int dof_address = model()->jnt_dofadr[joint.joint_id()];
  data()->qpos[qpos_address] = 1.0;
  data()->qvel[dof_address] = 2.0;
  data()->qfrc_actuator[dof_address] = 3.0;
  ASSERT_TRUE(joint.update(context()));

  data()->qpos[qpos_address] = 4.0;
  data()->qvel[dof_address] = 5.0;
  data()->qfrc_actuator[dof_address] = 6.0;
  JointState state;
  ASSERT_TRUE(joint.read(context(), state));
  EXPECT_DOUBLE_EQ(state.timestamp, 0.125);
  EXPECT_DOUBLE_EQ(state.position, 1.0);
  EXPECT_DOUBLE_EQ(state.velocity, 2.0);
  EXPECT_DOUBLE_EQ(state.effort, 3.0);
}

TEST_F(JointComponentTest, RejectsNonMotorActuators) {
  load(
      R"(<mujoco><worldbody><body><joint name="hinge" type="hinge"/><geom type="sphere" size=".1"/></body></worldbody><actuator><position name="position" joint="hinge"/></actuator></mujoco>)");
  JointComponent joint({.joint_name = "hinge", .actuator_name = "position"});
  EXPECT_FALSE(joint.init(context()));
  EXPECT_FALSE(joint.is_initialized());
}

TEST_F(JointComponentTest, RejectsNonJointTransmission) {
  load(
      R"(<mujoco><worldbody><body><joint name="hinge" type="hinge"/><geom type="sphere" size=".1"/></body></worldbody><actuator><motor name="motor" joint="hinge"/></actuator></mujoco>)");
  model()->actuator_trntype[0] = mjTRN_TENDON;
  JointComponent joint({.joint_name = "hinge", .actuator_name = "motor"});
  EXPECT_FALSE(joint.init(context()));
  EXPECT_FALSE(joint.is_initialized());
}

TEST_F(JointComponentTest, RejectsActuatorBoundToDifferentJoint) {
  load(
      R"(<mujoco><worldbody><body><joint name="hinge_1" type="hinge"/><joint name="hinge_2" type="hinge"/><geom type="sphere" size=".1"/></body></worldbody><actuator><motor name="motor" joint="hinge_2"/></actuator></mujoco>)");
  JointComponent joint({.joint_name = "hinge_1", .actuator_name = "motor"});
  EXPECT_FALSE(joint.init(context()));
  EXPECT_FALSE(joint.is_initialized());
}

TEST_F(JointComponentTest, RejectsUnsupportedActuatorDynamics) {
  load(
      R"(<mujoco><worldbody><body><joint name="hinge" type="hinge"/><geom type="sphere" size=".1"/></body></worldbody><actuator><general name="motor" joint="hinge" dyntype="filterexact" dynprm=".1" gainprm="1"/></actuator></mujoco>)");
  JointComponent joint({.joint_name = "hinge", .actuator_name = "motor"});
  EXPECT_FALSE(joint.init(context()));
  EXPECT_FALSE(joint.is_initialized());
}

TEST_F(JointComponentTest, RejectsNonFixedAndNonUnitActuatorGain) {
  load(
      R"(<mujoco><worldbody><body><joint name="hinge" type="hinge"/><geom type="sphere" size=".1"/></body></worldbody><actuator><general name="motor" joint="hinge" gaintype="affine" gainprm="1 0 0"/></actuator></mujoco>)");
  JointComponent affine_gain({.joint_name = "hinge", .actuator_name = "motor"});
  EXPECT_FALSE(affine_gain.init(context()));
  EXPECT_FALSE(affine_gain.is_initialized());

  model()->actuator_gaintype[0] = mjGAIN_FIXED;
  model()->actuator_gainprm[0] = 2.0;
  JointComponent non_unit_gain({.joint_name = "hinge", .actuator_name = "motor"});
  EXPECT_FALSE(non_unit_gain.init(context()));
  EXPECT_FALSE(non_unit_gain.is_initialized());
}

TEST_F(JointComponentTest, RejectsNonUnitActuatorGear) {
  load(
      R"(<mujoco><worldbody><body><joint name="hinge" type="hinge"/><geom type="sphere" size=".1"/></body></worldbody><actuator><motor name="motor" joint="hinge" gear="2"/></actuator></mujoco>)");
  JointComponent joint({.joint_name = "hinge", .actuator_name = "motor"});
  EXPECT_FALSE(joint.init(context()));
  EXPECT_FALSE(joint.is_initialized());
}

TEST_F(JointComponentTest, FailedReinitializationInvalidatesPreviousBinding) {
  load(
      R"(<mujoco><worldbody><body><joint name="hinge" type="hinge"/><geom type="sphere" size=".1"/></body></worldbody><actuator><motor name="motor" joint="hinge" ctrlrange="-20 20"/></actuator></mujoco>)");
  JointComponent joint({.joint_name = "hinge", .actuator_name = "motor"});
  ASSERT_TRUE(joint.init(context()));
  ASSERT_TRUE(joint.write(
      context(), {.joint_name = "hinge", .mode = JointControlMode::Effort, .effort = 3.0}));

  model()->actuator_gainprm[joint.actuator_id() * mjNGAIN] = 2.0;
  EXPECT_FALSE(joint.init(context()));
  EXPECT_FALSE(joint.is_initialized());
  EXPECT_FALSE(joint.write(
      context(), {.joint_name = "hinge", .mode = JointControlMode::Effort, .effort = 4.0}));
  EXPECT_DOUBLE_EQ(data()->ctrl[joint.actuator_id()], 3.0);
}

TEST_F(JointComponentTest, RejectsInvalidHybridCommandWithoutOverwritingControl) {
  load(
      R"(<mujoco><worldbody><body><joint name="hinge" type="hinge"/><geom type="sphere" size=".1"/></body></worldbody><actuator><motor name="motor" joint="hinge" ctrlrange="-20 20"/></actuator></mujoco>)");
  JointComponent joint({.joint_name = "hinge", .actuator_name = "motor"});
  ASSERT_TRUE(joint.init(context()));
  ASSERT_TRUE(joint.write(
      context(), {.joint_name = "hinge", .mode = JointControlMode::Effort, .effort = 3.0}));

  EXPECT_FALSE(joint.write(context(), {.joint_name = "hinge",
                                       .mode = JointControlMode::Hybrid,
                                       .stiffness = -1.0,
                                       .damping = 0.0}));
  EXPECT_DOUBLE_EQ(data()->ctrl[joint.actuator_id()], 3.0);
}

TEST_F(JointComponentTest, HybridLimitsOnlyTheCombinedEffort) {
  load(
      R"(<mujoco><worldbody><body><joint name="hinge" type="hinge"/><geom type="sphere" size=".1"/></body></worldbody><actuator><motor name="motor" joint="hinge" ctrlrange="-20 20"/></actuator></mujoco>)");
  JointComponent joint({.joint_name = "hinge",
                        .actuator_name = "motor",
                        .effort_limits = {.min = -10.0, .max = 10.0}});
  ASSERT_TRUE(joint.init(context()));

  ASSERT_TRUE(joint.write(context(), {.joint_name = "hinge",
                                      .mode = JointControlMode::Hybrid,
                                      .velocity = -5.0,
                                      .effort = 12.0,
                                      .damping = 1.0}));
  EXPECT_DOUBLE_EQ(data()->ctrl[joint.actuator_id()], 7.0);
}

TEST_F(JointComponentTest, ValidatesOnlyModeSpecificCommandValues) {
  load(
      R"(<mujoco><worldbody><body><joint name="hinge" type="hinge"/><geom type="sphere" size=".1"/></body></worldbody><actuator><motor name="motor" joint="hinge" ctrlrange="-20 20"/></actuator></mujoco>)");
  JointComponent joint(
      {.joint_name = "hinge", .actuator_name = "motor", .position_stiffness = 10.0});
  const double nan = std::numeric_limits<double>::quiet_NaN();
  ASSERT_TRUE(joint.init(context()));

  EXPECT_FALSE(joint.write(
      context(), {.joint_name = "hinge", .mode = JointControlMode::Position, .position = nan}));
  EXPECT_FALSE(joint.write(
      context(), {.joint_name = "hinge", .mode = JointControlMode::Velocity, .velocity = nan}));
  EXPECT_FALSE(joint.write(
      context(), {.joint_name = "hinge", .mode = JointControlMode::Effort, .effort = nan}));
  EXPECT_FALSE(joint.write(context(), {.joint_name = "hinge",
                                       .mode = JointControlMode::Hybrid,
                                       .position = 1.0,
                                       .stiffness = nan}));
  EXPECT_DOUBLE_EQ(data()->ctrl[joint.actuator_id()], 0.0);

  EXPECT_TRUE(joint.write(
      context(),
      {.joint_name = "hinge", .mode = JointControlMode::Position, .position = 1.0, .effort = nan}));
  EXPECT_DOUBLE_EQ(data()->ctrl[joint.actuator_id()], 10.0);
}

TEST_F(JointComponentTest, RejectsInvalidJointInfo) {
  load(
      R"(<mujoco><worldbody><body><joint name="hinge" type="hinge"/><geom type="sphere" size=".1"/></body></worldbody><actuator><motor name="motor" joint="hinge"/></actuator></mujoco>)");
  JointInfo invalid{.joint_name = "hinge", .actuator_name = "motor"};
  invalid.position_limits = {.min = 1.0, .max = -1.0};
  EXPECT_FALSE(JointComponent(invalid).init(context()));

  invalid.position_limits = {};
  invalid.velocity_limits = {.min = 1.0, .max = -1.0};
  EXPECT_FALSE(JointComponent(invalid).init(context()));

  invalid.velocity_limits = {};
  invalid.effort_limits = {.min = 1.0, .max = -1.0};
  EXPECT_FALSE(JointComponent(invalid).init(context()));

  invalid.effort_limits = {};
  invalid.position_stiffness = -1.0;
  EXPECT_FALSE(JointComponent(invalid).init(context()));

  invalid.position_stiffness = 0.0;
  invalid.position_damping = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(JointComponent(invalid).init(context()));

  invalid.position_damping = 0.0;
  invalid.velocity_damping = -1.0;
  EXPECT_FALSE(JointComponent(invalid).init(context()));
}

}  // namespace
}  // namespace mujoco_simulation
