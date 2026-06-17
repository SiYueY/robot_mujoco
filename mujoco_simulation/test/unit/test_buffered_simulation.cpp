#include <gtest/gtest.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <string>

#include "mujoco_simulation/simulation.hpp"

namespace mujoco_simulation {
namespace {

#define ASSERT_OK_STATUS(expr)                        \
  do {                                                \
    const Status status__ = (expr);                   \
    ASSERT_TRUE(status__.ok()) << status__.message(); \
  } while (false)

class BufferedSimulationTest : public ::testing::Test {
 protected:
  void TearDown() override {
    if (!model_path_.empty()) {
      std::error_code error;
      std::filesystem::remove(model_path_, error);
    }
  }

  std::string write_model(const std::string& xml_contents) {
    const auto temp_dir = std::filesystem::temp_directory_path();
    model_path_ = temp_dir / std::filesystem::path("buffered_simulation_test_" +
                                                   std::to_string(::getpid()) + ".xml");
    std::ofstream output(model_path_);
    EXPECT_TRUE(output.is_open());
    output << xml_contents;
    output.close();
    return model_path_.string();
  }

  std::filesystem::path model_path_;
};

TEST_F(BufferedSimulationTest, JointCommandsAreAppliedDuringStepNotAtSubmissionTime) {
  Simulation simulation;
  const std::string model_path = write_model(R"(
<mujoco model="buffered_simulation">
  <worldbody>
    <body>
      <joint name="hinge" type="hinge"/>
      <geom type="capsule" size="0.05 0.2"/>
    </body>
  </worldbody>
  <actuator>
    <velocity name="hinge_vel" joint="hinge"/>
  </actuator>
</mujoco>)");

  SimulationConfig config;
  config.model.model_path = model_path;
  config.components = {
      ComponentConfig{JointConfig{.name = "hinge",
                                  .actuator_name = "hinge_vel",
                                  .command_mode = CommandInterfaceType::Velocity}}};
  ASSERT_OK_STATUS(simulation.initialize(config));

  const auto before_command = simulation.joint_state("hinge");
  ASSERT_TRUE(before_command.ok()) << before_command.status().message();
  EXPECT_DOUBLE_EQ(before_command.value().velocity, 0.0);

  ASSERT_OK_STATUS(simulation.set_joint_command({"hinge", 0.0, 0.75, 0.0, 0.0}));

  const auto before_step = simulation.joint_state("hinge");
  ASSERT_TRUE(before_step.ok()) << before_step.status().message();
  EXPECT_DOUBLE_EQ(before_step.value().velocity, 0.0);

  ASSERT_OK_STATUS(simulation.step(1));

  const auto after_step = simulation.joint_state("hinge");
  ASSERT_TRUE(after_step.ok()) << after_step.status().message();
  EXPECT_EQ(after_step.value().name, "hinge");
  EXPECT_GT(after_step.value().velocity, 0.0);
}

TEST_F(BufferedSimulationTest, ImuSamplesAreReadFromPublishedSnapshot) {
  Simulation simulation;
  const std::string model_path = write_model(R"(
<mujoco model="buffered_simulation">
  <worldbody>
    <body name="imu_body">
      <freejoint/>
      <site name="imu_site" pos="0 0 0"/>
      <geom type="sphere" size="0.05"/>
    </body>
  </worldbody>
  <sensor>
    <framequat name="imu_quat" objtype="site" objname="imu_site"/>
    <gyro name="imu_gyro" site="imu_site"/>
    <accelerometer name="imu_acc" site="imu_site"/>
  </sensor>
</mujoco>)");

  SimulationConfig config;
  config.model.model_path = model_path;
  config.components = {ComponentConfig{ImuConfig{.common = {.name = "imu", .update_rate = 200.0},
                                                 .framequat_sensor_name = "imu_quat",
                                                 .gyro_sensor_name = "imu_gyro",
                                                 .accelerometer_sensor_name = "imu_acc"}}};
  ASSERT_OK_STATUS(simulation.initialize(config));
  ASSERT_OK_STATUS(simulation.step(1));

  const auto sample = simulation.imu_sample("imu");
  ASSERT_TRUE(sample.ok()) << sample.status().message();
  EXPECT_DOUBLE_EQ(sample.value().orientation[0], 0.0);
  EXPECT_DOUBLE_EQ(sample.value().orientation[1], 0.0);
  EXPECT_DOUBLE_EQ(sample.value().orientation[2], 0.0);
  EXPECT_DOUBLE_EQ(sample.value().orientation[3], 1.0);
}

TEST_F(BufferedSimulationTest, ConfiguredDevicesAppearInSnapshotBeforeStepping) {
  Simulation simulation;
  const std::string model_path = write_model(R"(
<mujoco model="buffered_simulation">
  <worldbody>
    <body>
      <joint name="hinge" type="hinge"/>
      <site name="imu_site" pos="0 0 0"/>
      <geom type="capsule" size="0.05 0.2"/>
    </body>
  </worldbody>
  <actuator>
    <velocity name="hinge_vel" joint="hinge"/>
  </actuator>
  <sensor>
    <framequat name="imu_quat" objtype="site" objname="imu_site"/>
    <gyro name="imu_gyro" site="imu_site"/>
    <accelerometer name="imu_acc" site="imu_site"/>
  </sensor>
</mujoco>)");

  SimulationConfig config;
  config.model.model_path = model_path;
  config.components = {
      ComponentConfig{JointConfig{.name = "hinge",
                                  .actuator_name = "hinge_vel",
                                  .command_mode = CommandInterfaceType::Velocity}},
      ComponentConfig{ImuConfig{.common = {.name = "imu", .update_rate = 200.0},
                                .framequat_sensor_name = "imu_quat",
                                .gyro_sensor_name = "imu_gyro",
                                .accelerometer_sensor_name = "imu_acc"}},
  };
  ASSERT_OK_STATUS(simulation.initialize(config));

  const std::shared_ptr<const SimulationStateSnapshot> snapshot = simulation.state_snapshot();
  ASSERT_NE(snapshot, nullptr);
  EXPECT_NE(snapshot->joints.find("hinge"), snapshot->joints.end());
  EXPECT_NE(snapshot->imus.find("imu"), snapshot->imus.end());
}

TEST_F(BufferedSimulationTest, InitializeAppliesConfiguredInitialKeyframeThroughModelLoad) {
  Simulation simulation;
  const std::string model_path = write_model(R"(
<mujoco model="buffered_simulation">
  <worldbody>
    <body>
      <joint name="hinge" type="hinge"/>
      <geom type="capsule" size="0.05 0.2"/>
    </body>
  </worldbody>
  <keyframe>
    <key name="home" qpos="0.75"/>
  </keyframe>
</mujoco>)");

  SimulationConfig config;
  config.model.model_path = model_path;
  config.model.initial_keyframe = "home";
  config.components = {
      ComponentConfig{JointConfig{.name = "hinge", .command_mode = CommandInterfaceType::None}}};
  ASSERT_OK_STATUS(simulation.initialize(config));

  const auto state = simulation.joint_state("hinge");
  ASSERT_TRUE(state.ok()) << state.status().message();
  EXPECT_NEAR(state.value().position, 0.75, 1.0e-9);
}

TEST_F(BufferedSimulationTest, ResetReturnsRuntimeFailureWhileRunning) {
  Simulation simulation;
  const std::string model_path = write_model(R"(
<mujoco model="buffered_simulation">
  <worldbody>
    <body>
      <joint name="hinge" type="hinge"/>
      <geom type="capsule" size="0.05 0.2"/>
    </body>
  </worldbody>
  <actuator>
    <velocity name="hinge_vel" joint="hinge"/>
  </actuator>
</mujoco>)");

  ASSERT_OK_STATUS(simulation.initialize({.model = {.model_path = model_path}}));
  ASSERT_OK_STATUS(simulation.start());

  const Status reset_status = simulation.reset({.keyframe_name = "missing_keyframe"});
  EXPECT_FALSE(reset_status.ok());
  EXPECT_NE(reset_status.message().find("missing_keyframe"), std::string::npos);

  ASSERT_OK_STATUS(simulation.stop());
}

TEST_F(BufferedSimulationTest, RealtimeFactorCanBeConfiguredThroughSimulationApi) {
  Simulation simulation;
  const std::string model_path = write_model(R"(
<mujoco model="buffered_simulation">
  <worldbody>
    <body>
      <joint name="hinge" type="hinge"/>
      <geom type="capsule" size="0.05 0.2"/>
    </body>
  </worldbody>
</mujoco>)");

  ASSERT_OK_STATUS(simulation.initialize(
      {.model = {.model_path = model_path}, .scheduler = {.realtime_factor = 1.0}}));
  ASSERT_OK_STATUS(simulation.set_realtime_factor(3.0));

  const Status invalid_status = simulation.set_realtime_factor(0.0);
  EXPECT_FALSE(invalid_status.ok());
  EXPECT_NE(invalid_status.message().find("greater than zero"), std::string::npos);
}

TEST_F(BufferedSimulationTest, ResetCanPreserveBufferedCommandsWhenRequested) {
  Simulation simulation;
  const std::string model_path = write_model(R"(
<mujoco model="buffered_simulation">
  <option timestep="0.01"/>
  <worldbody>
    <body>
      <joint name="hinge" type="hinge"/>
      <geom type="capsule" size="0.05 0.2"/>
    </body>
  </worldbody>
  <actuator>
    <velocity name="hinge_vel" joint="hinge"/>
  </actuator>
</mujoco>)");

  SimulationConfig config;
  config.model.model_path = model_path;
  config.components = {
      ComponentConfig{JointConfig{.name = "hinge",
                                  .actuator_name = "hinge_vel",
                                  .command_mode = CommandInterfaceType::Velocity}}};
  ASSERT_OK_STATUS(simulation.initialize(config));

  ASSERT_OK_STATUS(simulation.set_joint_command({"hinge", 0.0, 0.75, 0.0, 0.0}));
  ASSERT_OK_STATUS(simulation.reset({.clear_commands = false}));
  ASSERT_OK_STATUS(simulation.step(1));

  const auto state = simulation.joint_state("hinge");
  ASSERT_TRUE(state.ok()) << state.status().message();
  EXPECT_GT(state.value().velocity, 0.0);
}

TEST_F(BufferedSimulationTest, JointReconfigureUpdatesComponentManagerState) {
  Simulation simulation;
  const std::string model_path = write_model(R"(
<mujoco model="buffered_simulation">
  <worldbody>
    <body>
      <joint name="hinge" type="hinge"/>
      <geom type="capsule" size="0.05 0.2"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="hinge_motor" joint="hinge"/>
  </actuator>
</mujoco>)");

  SimulationConfig config;
  config.model.model_path = model_path;
  config.components = {
      ComponentConfig{JointConfig{.name = "hinge",
                                  .actuator_name = "hinge_motor",
                                  .command_mode = CommandInterfaceType::Effort,
                                  .controller_type = JointControllerType::SoftwarePd,
                                  .velocity_kd = 8.0}}};
  ASSERT_OK_STATUS(simulation.initialize(config));

  ASSERT_OK_STATUS(simulation.reconfigure_component(ComponentConfig{JointConfig{
      .name = "hinge",
      .actuator_name = "hinge_motor",
      .command_mode = CommandInterfaceType::Velocity,
      .controller_type = JointControllerType::SoftwarePd,
      .velocity_kd = 8.0,
  }}));
  ASSERT_OK_STATUS(simulation.set_joint_command({"hinge", 0.0, 0.75, 0.0, 0.0}));
  ASSERT_OK_STATUS(simulation.step(1));

  const auto state = simulation.joint_state("hinge");
  ASSERT_TRUE(state.ok()) << state.status().message();
  EXPECT_GT(state.value().velocity, 0.0);
}

TEST_F(BufferedSimulationTest, FailedJointReconfigureKeepsExistingJointComponent) {
  Simulation simulation;
  const std::string model_path = write_model(R"(
<mujoco model="buffered_simulation">
  <worldbody>
    <body>
      <joint name="hinge" type="hinge"/>
      <geom type="capsule" size="0.05 0.2"/>
    </body>
  </worldbody>
  <actuator>
    <velocity name="hinge_vel" joint="hinge"/>
  </actuator>
</mujoco>)");

  SimulationConfig config;
  config.model.model_path = model_path;
  config.components = {
      ComponentConfig{JointConfig{.name = "hinge",
                                  .actuator_name = "hinge_vel",
                                  .command_mode = CommandInterfaceType::Velocity}}};
  ASSERT_OK_STATUS(simulation.initialize(config));

  const Status switch_status = simulation.reconfigure_component(ComponentConfig{JointConfig{
      .name = "hinge",
      .actuator_name = "hinge_vel",
      .command_mode = CommandInterfaceType::Position,
  }});
  EXPECT_FALSE(switch_status.ok());
  EXPECT_NE(switch_status.message().find("MuJoCo position actuator"), std::string::npos);

  ASSERT_OK_STATUS(simulation.set_joint_command({"hinge", 0.0, 0.75, 0.0, 0.0}));
  ASSERT_OK_STATUS(simulation.step(1));

  const auto state = simulation.joint_state("hinge");
  ASSERT_TRUE(state.ok()) << state.status().message();
  EXPECT_GT(state.value().velocity, 0.0);
}

TEST_F(BufferedSimulationTest, InitializeBuildsConfiguredComponentsInDependencyOrder) {
  Simulation simulation;
  const std::string model_path = write_model(R"(
<mujoco model="buffered_simulation">
  <option timestep="0.01"/>
  <worldbody>
    <body>
      <joint name="left_wheel" type="hinge"/>
      <geom type="capsule" size="0.05 0.2"/>
      <site name="imu_site" pos="0 0 0"/>
    </body>
    <body pos="0 0.2 0">
      <joint name="right_wheel" type="hinge"/>
      <geom type="capsule" size="0.05 0.2"/>
    </body>
    <camera name="cam" pos="1 0 0" xyaxes="0 1 0 0 0 1" fovy="45"/>
  </worldbody>
  <actuator>
    <velocity name="left_act" joint="left_wheel"/>
    <velocity name="right_act" joint="right_wheel"/>
  </actuator>
  <sensor>
    <framequat name="imu_quat" objtype="site" objname="imu_site"/>
    <gyro name="imu_gyro" site="imu_site"/>
    <accelerometer name="imu_acc" site="imu_site"/>
  </sensor>
</mujoco>)");

  SimulationConfig config;
  config.model.model_path = model_path;
  config.components = {
      ComponentConfig{CameraConfig{
          .common = {.name = "front_camera", .frame_id = "camera_link", .update_rate = 30.0},
          .camera_name = "cam",
          .optical_frame_id = "camera_optical_frame",
          .height = 120,
          .width = 160,
          .enable_rgb = true,
          .enable_depth = false}},
      ComponentConfig{MobileBaseConfig{.name = "base",
                                       .type = MobileBaseType::Differential,
                                       .left_wheel_joint = "left_wheel",
                                       .right_wheel_joint = "right_wheel",
                                       .wheel_radius = 0.2,
                                       .track_width = 0.6}},
      ComponentConfig{
          ImuConfig{.common = {.name = "imu", .frame_id = "imu_link", .update_rate = 100.0},
                    .framequat_sensor_name = "imu_quat",
                    .gyro_sensor_name = "imu_gyro",
                    .accelerometer_sensor_name = "imu_acc"}},
      ComponentConfig{JointConfig{.name = "right_wheel",
                                  .actuator_name = "right_act",
                                  .command_mode = CommandInterfaceType::Velocity}},
      ComponentConfig{JointConfig{.name = "left_wheel",
                                  .actuator_name = "left_act",
                                  .command_mode = CommandInterfaceType::Velocity}},
  };

  ASSERT_OK_STATUS(simulation.initialize(config));

  const std::shared_ptr<const SimulationStateSnapshot> snapshot = simulation.state_snapshot();
  ASSERT_NE(snapshot, nullptr);
  EXPECT_NE(snapshot->joints.find("left_wheel"), snapshot->joints.end());
  EXPECT_NE(snapshot->joints.find("right_wheel"), snapshot->joints.end());
  EXPECT_NE(snapshot->imus.find("imu"), snapshot->imus.end());
  EXPECT_NE(snapshot->mobile_bases.find("base"), snapshot->mobile_bases.end());

  const auto camera_sample = simulation.camera_sample("front_camera");
  ASSERT_TRUE(camera_sample.ok()) << camera_sample.status().message();
  ASSERT_NE(camera_sample.value(), nullptr);
  const CameraState camera_state = camera_state_from_sample(*camera_sample.value());
  EXPECT_EQ(camera_state.image.width, 160U);
  EXPECT_EQ(camera_state.image.height, 120U);
  EXPECT_EQ(camera_state.image.frame_id, "camera_optical_frame");
}

#undef ASSERT_OK_STATUS

}  // namespace
}  // namespace mujoco_simulation
