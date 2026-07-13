#include <gtest/gtest.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <memory>
#include <string>

#include "mujoco_simulation/component/joint/joint_component.hpp"
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

  std::unique_ptr<JointComponent> bind_joint(const JointConfig& info) {
    auto joint = std::make_unique<JointComponent>(info);
    EXPECT_EQ(joint->bind(*model_), ResultCode::Ok);
    return joint;
  }

  MobileBaseBinding differential_binding(const JointComponent& left_joint,
                                         const JointComponent& right_joint) {
    MobileBaseBinding binding;
    binding.differential.emplace();
    binding.differential->left_wheel.joint_name = std::string(left_joint.name());
    binding.differential->left_wheel.joint = left_joint.binding();
    binding.differential->right_wheel.joint_name = std::string(right_joint.name());
    binding.differential->right_wheel.joint = right_joint.binding();
    return binding;
  }

  MobileBaseBinding omnidirectional_binding(const JointComponent& front_left,
                                            const JointComponent& front_right,
                                            const JointComponent& rear_left,
                                            const JointComponent& rear_right) {
    MobileBaseBinding binding;
    binding.omnidirectional.emplace();
    binding.omnidirectional->front_left.joint_name = std::string(front_left.name());
    binding.omnidirectional->front_left.joint = front_left.binding();
    binding.omnidirectional->front_right.joint_name = std::string(front_right.name());
    binding.omnidirectional->front_right.joint = front_right.binding();
    binding.omnidirectional->rear_left.joint_name = std::string(rear_left.name());
    binding.omnidirectional->rear_left.joint = rear_left.binding();
    binding.omnidirectional->rear_right.joint_name = std::string(rear_right.name());
    binding.omnidirectional->rear_right.joint = rear_right.binding();
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

  auto left_joint = bind_joint({.name = "left_wheel",
                                .actuator_name = "left_act",
                                .command_mode = CommandInterfaceType::Velocity});
  auto right_joint = bind_joint({.name = "right_wheel",
                                 .actuator_name = "right_act",
                                 .command_mode = CommandInterfaceType::Velocity});

  MobileBaseComponent base({.name = "base",
                            .type = MobileBaseType::Differential,
                            .left_wheel_joint = "left_wheel",
                            .right_wheel_joint = "right_wheel",
                            .wheel_radius = 0.2,
                            .track_width = 0.6},
                           differential_binding(*left_joint, *right_joint));
  ASSERT_EQ(base.bind(*model_), ResultCode::Ok);

  ASSERT_EQ(base.write(*model_, *data_, {.linear = {1.0, 0.0, 0.0}, .angular = {0.0, 0.0, 0.5}}),
            ResultCode::Ok);
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

  auto left_joint = bind_joint({.name = "left_wheel"});
  auto right_joint = bind_joint({.name = "right_wheel"});

  data_->qvel[model_->jnt_dofadr[mj_name2id(model_, mjOBJ_JOINT, "left_wheel")]] = 2.0;
  data_->qvel[model_->jnt_dofadr[mj_name2id(model_, mjOBJ_JOINT, "right_wheel")]] = 4.0;

  MobileBaseComponent base({.name = "base",
                            .type = MobileBaseType::Differential,
                            .left_wheel_joint = "left_wheel",
                            .right_wheel_joint = "right_wheel",
                            .wheel_radius = 0.1,
                            .track_width = 0.5},
                           differential_binding(*left_joint, *right_joint));
  ASSERT_EQ(base.bind(*model_), ResultCode::Ok);

  MobileBaseState state;
  ASSERT_EQ(base.read(*data_, state), ResultCode::Ok);
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

  auto front_left = bind_joint({.name = "fl"});
  auto front_right = bind_joint({.name = "fr"});
  auto rear_left = bind_joint({.name = "rl"});
  auto rear_right = bind_joint({.name = "rr"});

  const int fl_id = mj_name2id(model_, mjOBJ_JOINT, "fl");
  const int fr_id = mj_name2id(model_, mjOBJ_JOINT, "fr");
  const int rl_id = mj_name2id(model_, mjOBJ_JOINT, "rl");
  const int rr_id = mj_name2id(model_, mjOBJ_JOINT, "rr");
  data_->qvel[model_->jnt_dofadr[fl_id]] = 2.0;
  data_->qvel[model_->jnt_dofadr[fr_id]] = 6.0;
  data_->qvel[model_->jnt_dofadr[rl_id]] = 4.0;
  data_->qvel[model_->jnt_dofadr[rr_id]] = 8.0;

  MobileBaseComponent base(
      {.name = "base",
       .type = MobileBaseType::Omnidirectional,
       .front_left_joint = "fl",
       .front_right_joint = "fr",
       .rear_left_joint = "rl",
       .rear_right_joint = "rr",
       .wheel_radius = 0.1,
       .track_width = 0.3,
       .wheel_base = 0.5},
      omnidirectional_binding(*front_left, *front_right, *rear_left, *rear_right));
  ASSERT_EQ(base.bind(*model_), ResultCode::Ok);

  MobileBaseState state;
  ASSERT_EQ(base.read(*data_, state), ResultCode::Ok);
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

  auto front_left = bind_joint(
      {.name = "fl", .actuator_name = "fl_act", .command_mode = CommandInterfaceType::Velocity});
  auto front_right = bind_joint(
      {.name = "fr", .actuator_name = "fr_act", .command_mode = CommandInterfaceType::Velocity});
  auto rear_left = bind_joint(
      {.name = "rl", .actuator_name = "rl_act", .command_mode = CommandInterfaceType::Velocity});
  auto rear_right = bind_joint(
      {.name = "rr", .actuator_name = "rr_act", .command_mode = CommandInterfaceType::Velocity});

  MobileBaseComponent base(
      {.name = "base",
       .type = MobileBaseType::Omnidirectional,
       .front_left_joint = "fl",
       .front_right_joint = "fr",
       .rear_left_joint = "rl",
       .rear_right_joint = "rr",
       .wheel_radius = 0.1,
       .track_width = 0.3,
       .wheel_base = 0.5},
      omnidirectional_binding(*front_left, *front_right, *rear_left, *rear_right));
  ASSERT_EQ(base.bind(*model_), ResultCode::Ok);

  ASSERT_EQ(base.write(*model_, *data_, {.linear = {1.0, 0.2, 0.0}, .angular = {0.0, 0.0, 0.5}}),
            ResultCode::Ok);
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

  auto left_joint = bind_joint({.name = "left_wheel"});
  auto right_joint = bind_joint({.name = "right_wheel"});

  MobileBaseComponent base({.name = "base",
                            .type = MobileBaseType::Differential,
                            .left_wheel_joint = "left_wheel",
                            .right_wheel_joint = "right_wheel",
                            .wheel_radius = 0.1,
                            .track_width = 0.5,
                            .odometry_source = OdometrySource::WheelIntegration},
                           differential_binding(*left_joint, *right_joint));
  ASSERT_EQ(base.bind(*model_), ResultCode::Ok);

  const int left_id = mj_name2id(model_, mjOBJ_JOINT, "left_wheel");
  const int right_id = mj_name2id(model_, mjOBJ_JOINT, "right_wheel");
  data_->qvel[model_->jnt_dofadr[left_id]] = 1.0;
  data_->qvel[model_->jnt_dofadr[right_id]] = 1.0;
  data_->time = 0.0;
  MobileBaseState state;
  ASSERT_EQ(base.read(*data_, state), ResultCode::Ok);
  EXPECT_DOUBLE_EQ(state.x, 0.0);
  EXPECT_DOUBLE_EQ(state.y, 0.0);

  data_->time = 1.0;
  ASSERT_EQ(base.read(*data_, state), ResultCode::Ok);
  EXPECT_NEAR(state.x, 0.1, 1e-9);
  EXPECT_NEAR(state.y, 0.0, 1e-9);
  EXPECT_NEAR(state.yaw, 0.0, 1e-9);

  ASSERT_EQ(base.reset(*model_, *data_), ResultCode::Ok);
  ASSERT_EQ(base.read(*data_, state), ResultCode::Ok);
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

  auto front_left = bind_joint({.name = "fl"});
  auto front_right = bind_joint({.name = "fr"});
  auto rear_left = bind_joint({.name = "rl"});
  auto rear_right = bind_joint({.name = "rr"});

  MobileBaseComponent base(
      {.name = "base",
       .type = MobileBaseType::Omnidirectional,
       .front_left_joint = "fl",
       .front_right_joint = "fr",
       .rear_left_joint = "rl",
       .rear_right_joint = "rr",
       .wheel_radius = 0.1,
       .track_width = 0.3,
       .wheel_base = 0.5,
       .odometry_source = OdometrySource::WheelIntegration},
      omnidirectional_binding(*front_left, *front_right, *rear_left, *rear_right));
  ASSERT_EQ(base.bind(*model_), ResultCode::Ok);

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
  ASSERT_EQ(base.read(*data_, state), ResultCode::Ok);
  EXPECT_DOUBLE_EQ(state.x, 0.0);
  EXPECT_DOUBLE_EQ(state.y, 0.0);
  EXPECT_DOUBLE_EQ(state.yaw, 0.0);

  data_->time = 1.0;
  ASSERT_EQ(base.read(*data_, state), ResultCode::Ok);
  EXPECT_NEAR(state.x, 0.1, 1e-9);
  EXPECT_NEAR(state.y, 0.0, 1e-9);
  EXPECT_NEAR(state.yaw, 0.0, 1e-9);

  ASSERT_EQ(base.reset(*model_, *data_), ResultCode::Ok);
  ASSERT_EQ(base.read(*data_, state), ResultCode::Ok);
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

  auto left_joint = bind_joint({.name = "left_wheel"});
  auto right_joint = bind_joint({.name = "right_wheel"});

  MobileBaseComponent base({.name = "base",
                            .type = MobileBaseType::Differential,
                            .left_wheel_joint = "left_wheel",
                            .right_wheel_joint = "right_wheel",
                            .wheel_radius = 0.1,
                            .track_width = 0.5,
                            .odometry_source = OdometrySource::GroundTruthBodyPose,
                            .base_body_name = "base_link"},
                           differential_binding(*left_joint, *right_joint));
  ASSERT_EQ(base.bind(*model_), ResultCode::Ok);

  const int left_id = mj_name2id(model_, mjOBJ_JOINT, "left_wheel");
  const int right_id = mj_name2id(model_, mjOBJ_JOINT, "right_wheel");
  data_->qvel[model_->jnt_dofadr[left_id]] = 1.0;
  data_->qvel[model_->jnt_dofadr[right_id]] = 2.0;
  mj_forward(model_, data_);

  MobileBaseState state;
  ASSERT_EQ(base.read(*data_, state), ResultCode::Ok);
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

  auto left_joint = bind_joint({.name = "left_wheel",
                                .actuator_name = "left_act",
                                .command_mode = CommandInterfaceType::Velocity});
  auto right_joint = bind_joint({.name = "right_wheel",
                                 .actuator_name = "right_act",
                                 .command_mode = CommandInterfaceType::Velocity});

  MobileBaseComponent base({.name = "base",
                            .type = MobileBaseType::Differential,
                            .left_wheel_joint = "left_wheel",
                            .right_wheel_joint = "right_wheel",
                            .wheel_radius = 0.2,
                            .track_width = 0.6},
                           differential_binding(*left_joint, *right_joint));
  ASSERT_EQ(base.bind(*model_), ResultCode::Ok);

  const ResultCode status =
      base.write(*model_, *data_, {.linear = {0.5, 0.1, 0.0}, .angular = {0.0, 0.0, 0.0}});
  EXPECT_EQ(status, ResultCode::CommandRejected);
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

  auto left_joint = bind_joint({.name = "left_wheel"});
  auto right_joint = bind_joint({.name = "right_wheel"});

  MobileBaseComponent base({.name = "base",
                            .type = MobileBaseType::Differential,
                            .left_wheel_joint = "left_wheel",
                            .right_wheel_joint = "right_wheel",
                            .wheel_radius = 0.1,
                            .track_width = 0.5,
                            .odometry_source = OdometrySource::GroundTruthBodyPose,
                            .base_body_name = "missing_body"},
                           differential_binding(*left_joint, *right_joint));
  const ResultCode status = base.bind(*model_);
  EXPECT_EQ(status, ResultCode::BindingFailed);
}

TEST_F(MobileBaseTest, RebindingNewJointComponentsDoesNotInvalidateResolvedBindings) {
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

  const JointConfig config_left{.name = "left_wheel",
                                .actuator_name = "left_act",
                                .command_mode = CommandInterfaceType::Velocity};
  const JointConfig config_right{.name = "right_wheel",
                                 .actuator_name = "right_act",
                                 .command_mode = CommandInterfaceType::Velocity};
  auto left_joint = bind_joint(config_left);
  auto right_joint = bind_joint(config_right);

  MobileBaseComponent base({.name = "base",
                            .type = MobileBaseType::Differential,
                            .left_wheel_joint = "left_wheel",
                            .right_wheel_joint = "right_wheel",
                            .wheel_radius = 0.2,
                            .track_width = 0.6},
                           differential_binding(*left_joint, *right_joint));
  ASSERT_EQ(base.bind(*model_), ResultCode::Ok);

  ASSERT_EQ(base.write(*model_, *data_, {.linear = {0.4, 0.0, 0.0}, .angular = {0.0, 0.0, 0.0}}),
            ResultCode::Ok);

  ASSERT_EQ(base.write(*model_, *data_, {.linear = {0.4, 0.0, 0.0}, .angular = {0.0, 0.0, 0.0}}),
            ResultCode::Ok);

  auto rebound_left_joint = bind_joint(config_left);
  auto rebound_right_joint = bind_joint(config_right);
  (void)rebound_left_joint;
  (void)rebound_right_joint;
  ASSERT_EQ(base.write(*model_, *data_, {.linear = {0.4, 0.0, 0.0}, .angular = {0.0, 0.0, 0.0}}),
            ResultCode::Ok);

  const int left_id = mj_name2id(model_, mjOBJ_ACTUATOR, "left_act");
  const int right_id = mj_name2id(model_, mjOBJ_ACTUATOR, "right_act");
  ASSERT_GE(left_id, 0);
  ASSERT_GE(right_id, 0);
  EXPECT_DOUBLE_EQ(data_->ctrl[left_id], 2.0);
  EXPECT_DOUBLE_EQ(data_->ctrl[right_id], 2.0);
}

}  // namespace
}  // namespace mujoco_simulation
