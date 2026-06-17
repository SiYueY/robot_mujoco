#include <gtest/gtest.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <string>

#include "mujoco_simulation/component/component_manager.hpp"
#include "mujoco_simulation/component/mobile_base/mobile_base_component.hpp"

namespace mujoco_simulation {
namespace {

class MobileBaseTest : public ::testing::Test {
 protected:
  void TearDown() override {
    if (data_ != nullptr) {
      mj_deleteData(data_);
      data_ = nullptr;
    }
    if (model_ != nullptr) {
      mj_deleteModel(model_);
      model_ = nullptr;
    }
    if (!model_path_.empty()) {
      std::error_code error;
      std::filesystem::remove(model_path_, error);
    }
  }

  void load_model(const std::string& xml_contents) {
    const auto temp_dir = std::filesystem::temp_directory_path();
    model_path_ =
        temp_dir / std::filesystem::path("mobile_base_test_" + std::to_string(::getpid()) + ".xml");

    std::ofstream output(model_path_);
    ASSERT_TRUE(output.is_open());
    output << xml_contents;
    output.close();

    char error[1024] = {0};
    model_ = mj_loadXML(model_path_.c_str(), nullptr, error, sizeof(error));
    ASSERT_NE(model_, nullptr) << error;

    data_ = mj_makeData(model_);
    ASSERT_NE(data_, nullptr);
    mj_forward(model_, data_);
  }

  Status register_joint(ComponentManager& manager, const JointConfig& info) {
    return manager.register_joint(*model_, std::make_unique<JointComponent>(info));
  }

  MobileBaseBinding differential_binding(ComponentManager& manager, const std::string& left,
                                         const std::string& right) {
    MobileBaseBinding binding;
    binding.differential.emplace();
    const JointComponent* left_joint = manager.joint(left);
    const JointComponent* right_joint = manager.joint(right);
    EXPECT_NE(left_joint, nullptr);
    EXPECT_NE(right_joint, nullptr);
    if (left_joint != nullptr) {
      binding.differential->left_wheel.joint_name = left;
      binding.differential->left_wheel.joint = left_joint->binding();
    }
    if (right_joint != nullptr) {
      binding.differential->right_wheel.joint_name = right;
      binding.differential->right_wheel.joint = right_joint->binding();
    }
    return binding;
  }

  MobileBaseBinding omnidirectional_binding(ComponentManager& manager, const std::string& fl,
                                            const std::string& fr, const std::string& rl,
                                            const std::string& rr) {
    MobileBaseBinding binding;
    binding.omnidirectional.emplace();
    const JointComponent* front_left = manager.joint(fl);
    const JointComponent* front_right = manager.joint(fr);
    const JointComponent* rear_left = manager.joint(rl);
    const JointComponent* rear_right = manager.joint(rr);
    EXPECT_NE(front_left, nullptr);
    EXPECT_NE(front_right, nullptr);
    EXPECT_NE(rear_left, nullptr);
    EXPECT_NE(rear_right, nullptr);
    if (front_left != nullptr) {
      binding.omnidirectional->front_left.joint_name = fl;
      binding.omnidirectional->front_left.joint = front_left->binding();
    }
    if (front_right != nullptr) {
      binding.omnidirectional->front_right.joint_name = fr;
      binding.omnidirectional->front_right.joint = front_right->binding();
    }
    if (rear_left != nullptr) {
      binding.omnidirectional->rear_left.joint_name = rl;
      binding.omnidirectional->rear_left.joint = rear_left->binding();
    }
    if (rear_right != nullptr) {
      binding.omnidirectional->rear_right.joint_name = rr;
      binding.omnidirectional->rear_right.joint = rear_right->binding();
    }
    return binding;
  }

  mjModel* model_{nullptr};
  mjData* data_{nullptr};
  std::filesystem::path model_path_;
};

TEST_F(MobileBaseTest, DifferentialWriteMapsTwistToWheelVelocityCommands) {
  load_model(R"(
<mujoco model="mobile_base">
  <worldbody>
    <body>
      <joint name="left_wheel" type="hinge"/>
      <geom type="capsule" size="0.05 0.2"/>
    </body>
    <body pos="0 0.2 0">
      <joint name="right_wheel" type="hinge"/>
      <geom type="capsule" size="0.05 0.2"/>
    </body>
  </worldbody>
  <actuator>
    <velocity name="left_act" joint="left_wheel"/>
    <velocity name="right_act" joint="right_wheel"/>
  </actuator>
</mujoco>)");

  ComponentManager manager;
  ASSERT_TRUE(register_joint(manager, {.name = "left_wheel",
                                       .actuator_name = "left_act",
                                       .command_mode = CommandInterfaceType::Velocity})
                  .ok());
  ASSERT_TRUE(register_joint(manager, {.name = "right_wheel",
                                       .actuator_name = "right_act",
                                       .command_mode = CommandInterfaceType::Velocity})
                  .ok());

  MobileBaseComponent base({.name = "base",
                            .type = MobileBaseType::Differential,
                            .left_wheel_joint = "left_wheel",
                            .right_wheel_joint = "right_wheel",
                            .wheel_radius = 0.2,
                            .track_width = 0.6},
                           differential_binding(manager, "left_wheel", "right_wheel"));
  ASSERT_TRUE(base.bind(*model_).ok());

  ASSERT_TRUE(
      base.write(*model_, *data_, {.linear = {1.0, 0.0, 0.0}, .angular = {0.0, 0.0, 0.5}}).ok());
  const int left_id = mj_name2id(model_, mjOBJ_ACTUATOR, "left_act");
  const int right_id = mj_name2id(model_, mjOBJ_ACTUATOR, "right_act");
  ASSERT_GE(left_id, 0);
  ASSERT_GE(right_id, 0);
  EXPECT_DOUBLE_EQ(data_->ctrl[left_id], 4.25);
  EXPECT_DOUBLE_EQ(data_->ctrl[right_id], 5.75);
}

TEST_F(MobileBaseTest, DifferentialReadComputesTwistFromWheelVelocities) {
  load_model(R"(
<mujoco model="mobile_base">
  <worldbody>
    <body>
      <joint name="left_wheel" type="hinge"/>
      <geom type="capsule" size="0.05 0.2"/>
    </body>
    <body pos="0 0.2 0">
      <joint name="right_wheel" type="hinge"/>
      <geom type="capsule" size="0.05 0.2"/>
    </body>
  </worldbody>
</mujoco>)");

  ComponentManager manager;
  ASSERT_TRUE(register_joint(manager, {.name = "left_wheel"}).ok());
  ASSERT_TRUE(register_joint(manager, {.name = "right_wheel"}).ok());

  data_->qvel[model_->jnt_dofadr[mj_name2id(model_, mjOBJ_JOINT, "left_wheel")]] = 2.0;
  data_->qvel[model_->jnt_dofadr[mj_name2id(model_, mjOBJ_JOINT, "right_wheel")]] = 4.0;

  MobileBaseComponent base({.name = "base",
                            .type = MobileBaseType::Differential,
                            .left_wheel_joint = "left_wheel",
                            .right_wheel_joint = "right_wheel",
                            .wheel_radius = 0.1,
                            .track_width = 0.5},
                           differential_binding(manager, "left_wheel", "right_wheel"));
  ASSERT_TRUE(base.bind(*model_).ok());

  MobileBaseState state;
  ASSERT_TRUE(base.read(*data_, state).ok());
  EXPECT_DOUBLE_EQ(state.linear[0], 0.3);
  EXPECT_DOUBLE_EQ(state.angular[2], 0.4);
  EXPECT_DOUBLE_EQ(state.linear_x, 0.3);
  EXPECT_DOUBLE_EQ(state.angular_z, 0.4);
}

TEST_F(MobileBaseTest, OmnidirectionalReadComputesTwistFromWheelVelocities) {
  load_model(R"(
<mujoco model="mobile_base">
  <worldbody>
    <body><joint name="fl" type="hinge"/><geom type="capsule" size="0.05 0.2"/></body>
    <body><joint name="fr" type="hinge"/><geom type="capsule" size="0.05 0.2"/></body>
    <body><joint name="rl" type="hinge"/><geom type="capsule" size="0.05 0.2"/></body>
    <body><joint name="rr" type="hinge"/><geom type="capsule" size="0.05 0.2"/></body>
  </worldbody>
</mujoco>)");

  ComponentManager manager;
  for (const auto& name : {"fl", "fr", "rl", "rr"}) {
    ASSERT_TRUE(register_joint(manager, {.name = name}).ok());
  }

  const int fl_id = mj_name2id(model_, mjOBJ_JOINT, "fl");
  const int fr_id = mj_name2id(model_, mjOBJ_JOINT, "fr");
  const int rl_id = mj_name2id(model_, mjOBJ_JOINT, "rl");
  const int rr_id = mj_name2id(model_, mjOBJ_JOINT, "rr");
  data_->qvel[model_->jnt_dofadr[fl_id]] = 2.0;
  data_->qvel[model_->jnt_dofadr[fr_id]] = 6.0;
  data_->qvel[model_->jnt_dofadr[rl_id]] = 4.0;
  data_->qvel[model_->jnt_dofadr[rr_id]] = 8.0;

  MobileBaseComponent base({.name = "base",
                            .type = MobileBaseType::Omnidirectional,
                            .front_left_joint = "fl",
                            .front_right_joint = "fr",
                            .rear_left_joint = "rl",
                            .rear_right_joint = "rr",
                            .wheel_radius = 0.1,
                            .track_width = 0.3,
                            .wheel_base = 0.5},
                           omnidirectional_binding(manager, "fl", "fr", "rl", "rr"));
  ASSERT_TRUE(base.bind(*model_).ok());

  MobileBaseState state;
  ASSERT_TRUE(base.read(*data_, state).ok());
  EXPECT_DOUBLE_EQ(state.linear[0], 0.5);
  EXPECT_DOUBLE_EQ(state.linear[1], 0.0);
  EXPECT_DOUBLE_EQ(state.angular[2], 0.25);
  EXPECT_DOUBLE_EQ(state.linear_x, 0.5);
  EXPECT_DOUBLE_EQ(state.linear_y, 0.0);
  EXPECT_DOUBLE_EQ(state.angular_z, 0.25);
}

TEST_F(MobileBaseTest, OmnidirectionalWriteMapsTwistToWheelVelocityCommands) {
  load_model(R"(
<mujoco model="mobile_base">
  <worldbody>
    <body><joint name="fl" type="hinge"/><geom type="capsule" size="0.05 0.2"/></body>
    <body><joint name="fr" type="hinge"/><geom type="capsule" size="0.05 0.2"/></body>
    <body><joint name="rl" type="hinge"/><geom type="capsule" size="0.05 0.2"/></body>
    <body><joint name="rr" type="hinge"/><geom type="capsule" size="0.05 0.2"/></body>
  </worldbody>
  <actuator>
    <velocity name="fl_act" joint="fl"/>
    <velocity name="fr_act" joint="fr"/>
    <velocity name="rl_act" joint="rl"/>
    <velocity name="rr_act" joint="rr"/>
  </actuator>
</mujoco>)");

  ComponentManager manager;
  ASSERT_TRUE(register_joint(manager, {.name = "fl",
                                       .actuator_name = "fl_act",
                                       .command_mode = CommandInterfaceType::Velocity})
                  .ok());
  ASSERT_TRUE(register_joint(manager, {.name = "fr",
                                       .actuator_name = "fr_act",
                                       .command_mode = CommandInterfaceType::Velocity})
                  .ok());
  ASSERT_TRUE(register_joint(manager, {.name = "rl",
                                       .actuator_name = "rl_act",
                                       .command_mode = CommandInterfaceType::Velocity})
                  .ok());
  ASSERT_TRUE(register_joint(manager, {.name = "rr",
                                       .actuator_name = "rr_act",
                                       .command_mode = CommandInterfaceType::Velocity})
                  .ok());

  MobileBaseComponent base({.name = "base",
                            .type = MobileBaseType::Omnidirectional,
                            .front_left_joint = "fl",
                            .front_right_joint = "fr",
                            .rear_left_joint = "rl",
                            .rear_right_joint = "rr",
                            .wheel_radius = 0.1,
                            .track_width = 0.3,
                            .wheel_base = 0.5},
                           omnidirectional_binding(manager, "fl", "fr", "rl", "rr"));
  ASSERT_TRUE(base.bind(*model_).ok());

  ASSERT_TRUE(
      base.write(*model_, *data_, {.linear = {1.0, 0.2, 0.0}, .angular = {0.0, 0.0, 0.5}}).ok());
  EXPECT_DOUBLE_EQ(data_->ctrl[mj_name2id(model_, mjOBJ_ACTUATOR, "fl_act")], 4.0);
  EXPECT_DOUBLE_EQ(data_->ctrl[mj_name2id(model_, mjOBJ_ACTUATOR, "fr_act")], 16.0);
  EXPECT_DOUBLE_EQ(data_->ctrl[mj_name2id(model_, mjOBJ_ACTUATOR, "rl_act")], 8.0);
  EXPECT_DOUBLE_EQ(data_->ctrl[mj_name2id(model_, mjOBJ_ACTUATOR, "rr_act")], 12.0);
}

TEST_F(MobileBaseTest, WheelIntegrationAccumulatesOdometryAndResetClearsIt) {
  load_model(R"(
<mujoco model="mobile_base">
  <worldbody>
    <body>
      <joint name="left_wheel" type="hinge"/>
      <geom type="capsule" size="0.05 0.2"/>
    </body>
    <body>
      <joint name="right_wheel" type="hinge"/>
      <geom type="capsule" size="0.05 0.2"/>
    </body>
  </worldbody>
</mujoco>)");

  ComponentManager manager;
  ASSERT_TRUE(register_joint(manager, {.name = "left_wheel"}).ok());
  ASSERT_TRUE(register_joint(manager, {.name = "right_wheel"}).ok());

  MobileBaseComponent base({.name = "base",
                            .type = MobileBaseType::Differential,
                            .left_wheel_joint = "left_wheel",
                            .right_wheel_joint = "right_wheel",
                            .wheel_radius = 0.1,
                            .track_width = 0.5,
                            .odometry_source = OdometrySource::WheelIntegration},
                           differential_binding(manager, "left_wheel", "right_wheel"));
  ASSERT_TRUE(base.bind(*model_).ok());

  const int left_id = mj_name2id(model_, mjOBJ_JOINT, "left_wheel");
  const int right_id = mj_name2id(model_, mjOBJ_JOINT, "right_wheel");
  data_->qvel[model_->jnt_dofadr[left_id]] = 1.0;
  data_->qvel[model_->jnt_dofadr[right_id]] = 1.0;
  data_->time = 0.0;
  MobileBaseState state;
  ASSERT_TRUE(base.read(*data_, state).ok());
  EXPECT_DOUBLE_EQ(state.x, 0.0);
  EXPECT_DOUBLE_EQ(state.y, 0.0);

  data_->time = 1.0;
  ASSERT_TRUE(base.read(*data_, state).ok());
  EXPECT_NEAR(state.x, 0.1, 1e-9);
  EXPECT_NEAR(state.y, 0.0, 1e-9);
  EXPECT_NEAR(state.yaw, 0.0, 1e-9);

  ASSERT_TRUE(base.reset(*model_, *data_).ok());
  ASSERT_TRUE(base.read(*data_, state).ok());
  EXPECT_DOUBLE_EQ(state.x, 0.0);
  EXPECT_DOUBLE_EQ(state.y, 0.0);
  EXPECT_DOUBLE_EQ(state.yaw, 0.0);
}

TEST_F(MobileBaseTest, OmnidirectionalWheelIntegrationAndResetClearOdometry) {
  load_model(R"(
<mujoco model="mobile_base">
  <worldbody>
    <body><joint name="fl" type="hinge"/><geom type="capsule" size="0.05 0.2"/></body>
    <body><joint name="fr" type="hinge"/><geom type="capsule" size="0.05 0.2"/></body>
    <body><joint name="rl" type="hinge"/><geom type="capsule" size="0.05 0.2"/></body>
    <body><joint name="rr" type="hinge"/><geom type="capsule" size="0.05 0.2"/></body>
  </worldbody>
</mujoco>)");

  ComponentManager manager;
  for (const auto& name : {"fl", "fr", "rl", "rr"}) {
    ASSERT_TRUE(register_joint(manager, {.name = name}).ok());
  }

  MobileBaseComponent base({.name = "base",
                            .type = MobileBaseType::Omnidirectional,
                            .front_left_joint = "fl",
                            .front_right_joint = "fr",
                            .rear_left_joint = "rl",
                            .rear_right_joint = "rr",
                            .wheel_radius = 0.1,
                            .track_width = 0.3,
                            .wheel_base = 0.5,
                            .odometry_source = OdometrySource::WheelIntegration},
                           omnidirectional_binding(manager, "fl", "fr", "rl", "rr"));
  ASSERT_TRUE(base.bind(*model_).ok());

  const int fl_id = mj_name2id(model_, mjOBJ_JOINT, "fl");
  const int fr_id = mj_name2id(model_, mjOBJ_JOINT, "fr");
  const int rl_id = mj_name2id(model_, mjOBJ_JOINT, "rl");
  const int rr_id = mj_name2id(model_, mjOBJ_JOINT, "rr");

  data_->qvel[model_->jnt_dofadr[fl_id]] = 1.0;
  data_->qvel[model_->jnt_dofadr[fr_id]] = 1.0;
  data_->qvel[model_->jnt_dofadr[rl_id]] = 1.0;
  data_->qvel[model_->jnt_dofadr[rr_id]] = 1.0;
  data_->time = 0.0;

  MobileBaseState state;
  ASSERT_TRUE(base.read(*data_, state).ok());
  EXPECT_DOUBLE_EQ(state.x, 0.0);
  EXPECT_DOUBLE_EQ(state.y, 0.0);
  EXPECT_DOUBLE_EQ(state.yaw, 0.0);

  data_->time = 1.0;
  ASSERT_TRUE(base.read(*data_, state).ok());
  EXPECT_NEAR(state.x, 0.1, 1e-9);
  EXPECT_NEAR(state.y, 0.0, 1e-9);
  EXPECT_NEAR(state.yaw, 0.0, 1e-9);

  ASSERT_TRUE(base.reset(*model_, *data_).ok());
  ASSERT_TRUE(base.read(*data_, state).ok());
  EXPECT_DOUBLE_EQ(state.x, 0.0);
  EXPECT_DOUBLE_EQ(state.y, 0.0);
  EXPECT_DOUBLE_EQ(state.yaw, 0.0);
}

TEST_F(MobileBaseTest, GroundTruthPoseOverridesIntegratedOdometry) {
  load_model(R"(
<mujoco model="mobile_base">
  <compiler angle="radian"/>
  <worldbody>
    <body name="base_link" pos="1 2 0" euler="0 0 0.5">
      <joint name="left_wheel" type="hinge"/>
      <geom type="capsule" size="0.05 0.2"/>
    </body>
    <body>
      <joint name="right_wheel" type="hinge"/>
      <geom type="capsule" size="0.05 0.2"/>
    </body>
  </worldbody>
</mujoco>)");

  ComponentManager manager;
  ASSERT_TRUE(register_joint(manager, {.name = "left_wheel"}).ok());
  ASSERT_TRUE(register_joint(manager, {.name = "right_wheel"}).ok());

  MobileBaseComponent base({.name = "base",
                            .type = MobileBaseType::Differential,
                            .left_wheel_joint = "left_wheel",
                            .right_wheel_joint = "right_wheel",
                            .wheel_radius = 0.1,
                            .track_width = 0.5,
                            .odometry_source = OdometrySource::GroundTruthBodyPose,
                            .base_body_name = "base_link"},
                           differential_binding(manager, "left_wheel", "right_wheel"));
  ASSERT_TRUE(base.bind(*model_).ok());

  const int left_id = mj_name2id(model_, mjOBJ_JOINT, "left_wheel");
  const int right_id = mj_name2id(model_, mjOBJ_JOINT, "right_wheel");
  data_->qvel[model_->jnt_dofadr[left_id]] = 1.0;
  data_->qvel[model_->jnt_dofadr[right_id]] = 2.0;
  mj_forward(model_, data_);

  MobileBaseState state;
  ASSERT_TRUE(base.read(*data_, state).ok());
  EXPECT_NEAR(state.x, 1.0, 1e-9);
  EXPECT_NEAR(state.y, 2.0, 1e-9);
  EXPECT_NEAR(state.yaw, 0.5, 1e-6);
  EXPECT_NEAR(state.linear_x, 0.15, 1e-9);
}

TEST_F(MobileBaseTest, DifferentialBaseRejectsLateralVelocityCommand) {
  load_model(R"(
<mujoco model="mobile_base">
  <worldbody>
    <body>
      <joint name="left_wheel" type="hinge"/>
      <geom type="capsule" size="0.05 0.2"/>
    </body>
    <body pos="0 0.2 0">
      <joint name="right_wheel" type="hinge"/>
      <geom type="capsule" size="0.05 0.2"/>
    </body>
  </worldbody>
  <actuator>
    <velocity name="left_act" joint="left_wheel"/>
    <velocity name="right_act" joint="right_wheel"/>
  </actuator>
</mujoco>)");

  ComponentManager manager;
  ASSERT_TRUE(register_joint(manager, {.name = "left_wheel",
                                       .actuator_name = "left_act",
                                       .command_mode = CommandInterfaceType::Velocity})
                  .ok());
  ASSERT_TRUE(register_joint(manager, {.name = "right_wheel",
                                       .actuator_name = "right_act",
                                       .command_mode = CommandInterfaceType::Velocity})
                  .ok());

  MobileBaseComponent base({.name = "base",
                            .type = MobileBaseType::Differential,
                            .left_wheel_joint = "left_wheel",
                            .right_wheel_joint = "right_wheel",
                            .wheel_radius = 0.2,
                            .track_width = 0.6},
                           differential_binding(manager, "left_wheel", "right_wheel"));
  ASSERT_TRUE(base.bind(*model_).ok());

  const Status status =
      base.write(*model_, *data_, {.linear = {0.5, 0.1, 0.0}, .angular = {0.0, 0.0, 0.0}});
  EXPECT_FALSE(status.ok());
  EXPECT_EQ(status.code(), StatusCode::CommandRejected);
}

TEST_F(MobileBaseTest, MissingGroundTruthBodyFailsWithBindingError) {
  load_model(R"(
<mujoco model="mobile_base">
  <worldbody>
    <body>
      <joint name="left_wheel" type="hinge"/>
      <geom type="capsule" size="0.05 0.2"/>
    </body>
    <body>
      <joint name="right_wheel" type="hinge"/>
      <geom type="capsule" size="0.05 0.2"/>
    </body>
  </worldbody>
</mujoco>)");

  ComponentManager manager;
  ASSERT_TRUE(register_joint(manager, {.name = "left_wheel"}).ok());
  ASSERT_TRUE(register_joint(manager, {.name = "right_wheel"}).ok());

  MobileBaseComponent base({.name = "base",
                            .type = MobileBaseType::Differential,
                            .left_wheel_joint = "left_wheel",
                            .right_wheel_joint = "right_wheel",
                            .wheel_radius = 0.1,
                            .track_width = 0.5,
                            .odometry_source = OdometrySource::GroundTruthBodyPose,
                            .base_body_name = "missing_body"},
                           differential_binding(manager, "left_wheel", "right_wheel"));
  const Status status = base.bind(*model_);
  EXPECT_FALSE(status.ok());
  EXPECT_EQ(status.code(), StatusCode::BindingFailed);
}

TEST_F(MobileBaseTest, JointReconfigurationDoesNotInvalidateResolvedBindings) {
  load_model(R"(
<mujoco model="mobile_base">
  <worldbody>
    <body>
      <joint name="left_wheel" type="hinge"/>
      <geom type="capsule" size="0.05 0.2"/>
    </body>
    <body pos="0 0.2 0">
      <joint name="right_wheel" type="hinge"/>
      <geom type="capsule" size="0.05 0.2"/>
    </body>
  </worldbody>
  <actuator>
    <velocity name="left_act" joint="left_wheel"/>
    <velocity name="right_act" joint="right_wheel"/>
  </actuator>
</mujoco>)");

  ComponentManager manager;
  const JointConfig config_left{.name = "left_wheel",
                                .actuator_name = "left_act",
                                .command_mode = CommandInterfaceType::Velocity};
  const JointConfig config_right{.name = "right_wheel",
                                 .actuator_name = "right_act",
                                 .command_mode = CommandInterfaceType::Velocity};
  ASSERT_TRUE(register_joint(manager, config_left).ok());
  ASSERT_TRUE(register_joint(manager, config_right).ok());

  MobileBaseComponent base({.name = "base",
                            .type = MobileBaseType::Differential,
                            .left_wheel_joint = "left_wheel",
                            .right_wheel_joint = "right_wheel",
                            .wheel_radius = 0.2,
                            .track_width = 0.6},
                           differential_binding(manager, "left_wheel", "right_wheel"));
  ASSERT_TRUE(base.bind(*model_).ok());

  ASSERT_TRUE(
      base.write(*model_, *data_, {.linear = {0.4, 0.0, 0.0}, .angular = {0.0, 0.0, 0.0}}).ok());
  ASSERT_TRUE(manager.unregister_joint("left_wheel").ok());
  ASSERT_TRUE(manager.unregister_joint("right_wheel").ok());

  ASSERT_TRUE(
      base.write(*model_, *data_, {.linear = {0.4, 0.0, 0.0}, .angular = {0.0, 0.0, 0.0}}).ok());

  ASSERT_TRUE(register_joint(manager, config_left).ok());
  ASSERT_TRUE(register_joint(manager, config_right).ok());
  ASSERT_TRUE(
      base.write(*model_, *data_, {.linear = {0.4, 0.0, 0.0}, .angular = {0.0, 0.0, 0.0}}).ok());

  const int left_id = mj_name2id(model_, mjOBJ_ACTUATOR, "left_act");
  const int right_id = mj_name2id(model_, mjOBJ_ACTUATOR, "right_act");
  ASSERT_GE(left_id, 0);
  ASSERT_GE(right_id, 0);
  EXPECT_DOUBLE_EQ(data_->ctrl[left_id], 2.0);
  EXPECT_DOUBLE_EQ(data_->ctrl[right_id], 2.0);
}

}  // namespace
}  // namespace mujoco_simulation
