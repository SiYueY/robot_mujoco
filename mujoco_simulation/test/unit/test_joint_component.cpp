#include <gtest/gtest.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <string>

#include "mujoco_simulation/component/joint/joint_component.hpp"

namespace mujoco_simulation {
namespace {

class JointComponentTest : public ::testing::Test {
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
    model_path_ = temp_dir / std::filesystem::path("joint_component_test_" +
                                                   std::to_string(::getpid()) + ".xml");

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

  mjModel* model_{nullptr};
  mjData* data_{nullptr};
  std::filesystem::path model_path_;
};

TEST_F(JointComponentTest, DirectPositionActuatorWritesControl) {
  load_model(R"(
<mujoco model="joint_component">
  <worldbody>
    <body>
      <joint name="hinge" type="hinge"/>
      <geom type="capsule" size="0.05 0.2"/>
    </body>
  </worldbody>
  <actuator>
    <position name="hinge_pos" joint="hinge" ctrlrange="-1 1"/>
  </actuator>
</mujoco>)");

  JointComponent joint({.name = "hinge",
                        .actuator_name = "hinge_pos",
                        .command_mode = CommandInterfaceType::Position});
  ASSERT_TRUE(joint.bind(*model_).ok());

  ASSERT_TRUE(joint.write(*model_, *data_, {"hinge", 0.4, 0.0, 0.0, 0.0}).ok());
  const int actuator_id = mj_name2id(model_, mjOBJ_ACTUATOR, "hinge_pos");
  ASSERT_GE(actuator_id, 0);
  EXPECT_DOUBLE_EQ(data_->ctrl[actuator_id], 0.4);
}

TEST_F(JointComponentTest, InvalidActuatorModeFailsBinding) {
  load_model(R"(
<mujoco model="joint_component">
  <worldbody>
    <body>
      <joint name="hinge" type="hinge"/>
      <geom type="capsule" size="0.05 0.2"/>
    </body>
  </worldbody>
  <actuator>
    <position name="hinge_pos" joint="hinge"/>
  </actuator>
</mujoco>)");

  JointComponent joint({.name = "hinge",
                        .actuator_name = "hinge_pos",
                        .command_mode = CommandInterfaceType::Velocity});
  const Status status = joint.bind(*model_);
  EXPECT_FALSE(status.ok());
  EXPECT_EQ(status.code(), StatusCode::ModelValidationFailed);
}

TEST_F(JointComponentTest, MissingActuatorFailsWithBindingError) {
  load_model(R"(
<mujoco model="joint_component">
  <worldbody>
    <body>
      <joint name="hinge" type="hinge"/>
      <geom type="capsule" size="0.05 0.2"/>
    </body>
  </worldbody>
</mujoco>)");

  JointComponent joint({.name = "hinge",
                        .actuator_name = "missing_actuator",
                        .command_mode = CommandInterfaceType::Velocity});
  const Status status = joint.bind(*model_);
  EXPECT_FALSE(status.ok());
  EXPECT_EQ(status.code(), StatusCode::BindingFailed);
}

TEST_F(JointComponentTest, SoftwarePdOnPassiveJointWritesAppliedForceAndResetClearsIt) {
  load_model(R"(
<mujoco model="joint_component">
  <worldbody>
    <body>
      <joint name="hinge" type="hinge"/>
      <geom type="capsule" size="0.05 0.2"/>
    </body>
  </worldbody>
</mujoco>)");

  JointComponent joint({.name = "hinge",
                        .command_mode = CommandInterfaceType::Position,
                        .controller_type = JointControllerType::SoftwarePd,
                        .position_kp = 10.0,
                        .velocity_kd = 1.0,
                        .command_min = -5.0,
                        .command_max = 5.0});
  ASSERT_TRUE(joint.bind(*model_).ok());

  ASSERT_TRUE(joint.write(*model_, *data_, {"hinge", 1.0, 0.0, 0.0, 0.0}).ok());
  EXPECT_DOUBLE_EQ(data_->qfrc_applied[model_->jnt_dofadr[0]], 5.0);

  ASSERT_TRUE(joint.reset(*model_, *data_).ok());
  EXPECT_DOUBLE_EQ(data_->qfrc_applied[model_->jnt_dofadr[0]], 0.0);
}

TEST_F(JointComponentTest, SoftwarePdOnMotorActuatorClampsToActuatorLimits) {
  load_model(R"(
<mujoco model="joint_component">
  <worldbody>
    <body>
      <joint name="hinge" type="hinge"/>
      <geom type="capsule" size="0.05 0.2"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="hinge_motor" joint="hinge" gear="1" ctrlrange="-2 2" forcerange="-1.5 1.5"/>
  </actuator>
</mujoco>)");

  JointComponent joint({.name = "hinge",
                        .actuator_name = "hinge_motor",
                        .command_mode = CommandInterfaceType::Velocity,
                        .controller_type = JointControllerType::SoftwarePd,
                        .velocity_kd = 10.0});
  ASSERT_TRUE(joint.bind(*model_).ok());

  ASSERT_TRUE(joint.write(*model_, *data_, {"hinge", 0.0, 1.0, 0.0, 0.0}).ok());
  const int actuator_id = mj_name2id(model_, mjOBJ_ACTUATOR, "hinge_motor");
  ASSERT_GE(actuator_id, 0);
  EXPECT_DOUBLE_EQ(data_->ctrl[actuator_id], 1.5);

  ASSERT_TRUE(joint.reset(*model_, *data_).ok());
  EXPECT_DOUBLE_EQ(data_->ctrl[actuator_id], 0.0);
}

}  // namespace
}  // namespace mujoco_simulation
