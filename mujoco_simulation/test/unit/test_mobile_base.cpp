#include <gtest/gtest.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>

#include "mujoco_simulation/component/joint/joint_component.hpp"
#include "mujoco_simulation/component/mobile_base/mobile_base_component.hpp"

namespace mujoco_simulation {
namespace {

TEST(MobileBaseTest, DifferentialDriveUsesWheelMotorEffort) {
  const auto path = std::filesystem::temp_directory_path() /
                    ("mobile_base_" + std::to_string(::getpid()) + ".xml");
  std::ofstream output(path);
  output
      << R"(<mujoco><worldbody><body><joint name="left" type="hinge"/><geom type="sphere" size=".1"/></body><body><joint name="right" type="hinge"/><geom type="sphere" size=".1"/></body></worldbody><actuator><motor name="left_motor" joint="left"/><motor name="right_motor" joint="right"/></actuator></mujoco>)";
  output.close();
  char error[1024] = {};
  mjModel* model = mj_loadXML(path.c_str(), nullptr, error, sizeof(error));
  ASSERT_NE(model, nullptr) << error;
  mjData* data = mj_makeData(model);
  ASSERT_NE(data, nullptr);
  JointComponent left({.joint = "left", .actuator = "left_motor", .velocity_damping = 2.0});
  JointComponent right({.joint = "right", .actuator = "right_motor", .velocity_damping = 2.0});
  ASSERT_TRUE(left.bind(*model));
  ASSERT_TRUE(right.bind(*model));
  MobileBaseComponent base({.name = "base",
                            .type = MobileBaseType::Differential,
                            .left_wheel_joint = "left",
                            .right_wheel_joint = "right",
                            .wheel_radius = 0.2,
                            .track_width = 0.6});
  ASSERT_TRUE(base.configure_differential_drive(left, right));
  ASSERT_TRUE(base.bind(*model));
  ASSERT_TRUE(base.write(*model, *data, {.linear_x = 1.0}));
  EXPECT_GT(data->ctrl[left.motor_id()], 0.0);
  EXPECT_GT(data->ctrl[right.motor_id()], 0.0);
  mj_deleteData(data);
  mj_deleteModel(model);
  std::error_code remove_error;
  std::filesystem::remove(path, remove_error);
}

}  // namespace
}  // namespace mujoco_simulation
