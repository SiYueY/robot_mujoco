#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>

#include "mujoco_simulation/component/joint/joint_data.hpp"
#include "mujoco_simulation/simulation_config.hpp"

namespace mujoco_simulation {
namespace {

class RobotMujocoConfigTest : public ::testing::Test {
 protected:
  std::filesystem::path write_config(const std::string& name, const std::string& xml) {
    const std::filesystem::path path =
        std::filesystem::temp_directory_path() / std::filesystem::path(name);
    std::ofstream output(path);
    output << xml;
    output.close();
    return path;
  }
};

SimulationConfig sentinel_config() {
  SimulationConfig config;
  config.model.model_path = "sentinel_model";
  config.model.initial_keyframe = "sentinel_keyframe";
  config.scheduler.realtime_factor = 2.0;
  config.components.emplace_back(JointInfo{
      .joint = "sentinel_joint",
      .actuator = "sentinel_motor",
  });
  return config;
}

const JointInfo& require_joint(const ComponentConfig& component) {
  const auto* joint = std::get_if<JointInfo>(&component);
  EXPECT_NE(joint, nullptr);
  return *joint;
}

TEST_F(RobotMujocoConfigTest, LoadsMinimalJointConfiguration) {
  SimulationConfigParser parser;
  const std::filesystem::path config_path = write_config("robot_mujoco_minimal.xml",
                                                         R"(<robot_mujoco>
           <mujoco>
             <mjcf>robot.xml</mjcf>
           </mujoco>
           <robot>
             <joint name="joint_a">
               <position>
                 <stiffness>300</stiffness>
                 <damping>10</damping>
               </position>
               <velocity>
                 <damping>20</damping>
               </velocity>
               <limit>
                 <position>
                   <min>-2.8</min>
                   <max>2.8</max>
                 </position>
                 <velocity>
                   <min>-5</min>
                   <max>5</max>
                 </velocity>
                 <effort>
                   <min>-100</min>
                   <max>100</max>
                 </effort>
               </limit>
             </joint>
           </robot>
         </robot_mujoco>)");

  SimulationConfig config;
  ASSERT_TRUE(parser.load_file(config_path.string(), &config));
  EXPECT_EQ(config.model.model_path,
            (config_path.parent_path() / "robot.xml").lexically_normal().string());
  ASSERT_EQ(config.components.size(), 1U);

  const JointInfo& joint = require_joint(config.components.front());
  EXPECT_EQ(joint.joint, "joint_a");
  EXPECT_EQ(joint.actuator, "joint_a");
  EXPECT_DOUBLE_EQ(joint.position_stiffness, 300.0);
  EXPECT_DOUBLE_EQ(joint.position_damping, 10.0);
  EXPECT_DOUBLE_EQ(joint.velocity_damping, 20.0);
  EXPECT_DOUBLE_EQ(joint.position_limits.min, -2.8);
  EXPECT_DOUBLE_EQ(joint.position_limits.max, 2.8);
  EXPECT_DOUBLE_EQ(joint.velocity_limits.min, -5.0);
  EXPECT_DOUBLE_EQ(joint.velocity_limits.max, 5.0);
  EXPECT_DOUBLE_EQ(joint.effort_limits.min, -100.0);
  EXPECT_DOUBLE_EQ(joint.effort_limits.max, 100.0);
}

TEST_F(RobotMujocoConfigTest, ResolvesRelativeMjcfPathFromConfigDirectory) {
  SimulationConfigParser parser;
  const std::filesystem::path nested_dir =
      std::filesystem::temp_directory_path() / "robot_mujoco_parser" / "configs";
  std::filesystem::create_directories(nested_dir);
  const std::filesystem::path config_path = nested_dir / "robot_mujoco.xml";
  std::ofstream output(config_path);
  output << R"(<robot_mujoco>
                <mujoco>
                  <mjcf>../models/robot.xml</mjcf>
                </mujoco>
              </robot_mujoco>)";
  output.close();

  SimulationConfig config;
  ASSERT_TRUE(parser.load_file(config_path.string(), &config));
  EXPECT_EQ(config.model.model_path,
            (nested_dir / "../models/robot.xml").lexically_normal().string());
}

TEST_F(RobotMujocoConfigTest, LoadsMultipleJointsIntoComponentList) {
  SimulationConfigParser parser;
  const std::filesystem::path config_path = write_config("robot_mujoco_multiple.xml",
                                                         R"(<robot_mujoco>
           <mujoco><mjcf>robot.xml</mjcf></mujoco>
           <robot>
             <joint name="joint_a" />
             <joint name="joint_b">
               <velocity><damping>3</damping></velocity>
             </joint>
           </robot>
         </robot_mujoco>)");

  SimulationConfig config;
  ASSERT_TRUE(parser.load_file(config_path.string(), &config));
  ASSERT_EQ(config.components.size(), 2U);
  EXPECT_EQ(require_joint(config.components[0]).joint, "joint_a");
  EXPECT_EQ(require_joint(config.components[1]).joint, "joint_b");
  EXPECT_DOUBLE_EQ(require_joint(config.components[1]).velocity_damping, 3.0);
}

TEST_F(RobotMujocoConfigTest, RejectsMissingRootElement) {
  SimulationConfigParser parser;
  const std::filesystem::path config_path =
      write_config("robot_mujoco_missing_root.xml", "<mujoco></mujoco>");

  SimulationConfig config = sentinel_config();
  const SimulationConfig before = config;
  EXPECT_FALSE(parser.load_file(config_path.string(), &config));
  EXPECT_EQ(config.model.model_path, before.model.model_path);
  EXPECT_EQ(config.components.size(), before.components.size());
}

TEST_F(RobotMujocoConfigTest, RejectsMissingMjcf) {
  SimulationConfigParser parser;
  const std::filesystem::path config_path = write_config(
      "robot_mujoco_missing_mjcf.xml", R"(<robot_mujoco><mujoco></mujoco></robot_mujoco>)");

  SimulationConfig config = sentinel_config();
  const SimulationConfig before = config;
  EXPECT_FALSE(parser.load_file(config_path.string(), &config));
  EXPECT_EQ(config.model.model_path, before.model.model_path);
  EXPECT_EQ(config.components.size(), before.components.size());
}

TEST_F(RobotMujocoConfigTest, RejectsJointWithoutName) {
  SimulationConfigParser parser;
  const std::filesystem::path config_path = write_config("robot_mujoco_missing_joint_name.xml",
                                                         R"(<robot_mujoco>
           <mujoco><mjcf>robot.xml</mjcf></mujoco>
           <robot><joint /></robot>
         </robot_mujoco>)");

  SimulationConfig config = sentinel_config();
  const SimulationConfig before = config;
  EXPECT_FALSE(parser.load_file(config_path.string(), &config));
  EXPECT_EQ(config.model.model_path, before.model.model_path);
  EXPECT_EQ(config.components.size(), before.components.size());
}

TEST_F(RobotMujocoConfigTest, RejectsInvalidNumericValue) {
  SimulationConfigParser parser;
  const std::filesystem::path config_path = write_config("robot_mujoco_invalid_numeric.xml",
                                                         R"(<robot_mujoco>
           <mujoco><mjcf>robot.xml</mjcf></mujoco>
           <robot>
             <joint name="joint_a">
               <position><stiffness>abc</stiffness></position>
             </joint>
           </robot>
         </robot_mujoco>)");

  SimulationConfig config = sentinel_config();
  const SimulationConfig before = config;
  EXPECT_FALSE(parser.load_file(config_path.string(), &config));
  EXPECT_EQ(config.model.model_path, before.model.model_path);
  EXPECT_EQ(config.components.size(), before.components.size());
}

TEST_F(RobotMujocoConfigTest, RejectsInvalidLimitRange) {
  SimulationConfigParser parser;
  const std::filesystem::path config_path = write_config("robot_mujoco_invalid_limit.xml",
                                                         R"(<robot_mujoco>
           <mujoco><mjcf>robot.xml</mjcf></mujoco>
           <robot>
             <joint name="joint_a">
               <limit>
                 <position>
                   <min>2</min>
                   <max>1</max>
                 </position>
               </limit>
             </joint>
           </robot>
         </robot_mujoco>)");

  SimulationConfig config = sentinel_config();
  const SimulationConfig before = config;
  EXPECT_FALSE(parser.load_file(config_path.string(), &config));
  EXPECT_EQ(config.model.model_path, before.model.model_path);
  EXPECT_EQ(config.components.size(), before.components.size());
}

TEST_F(RobotMujocoConfigTest, RejectsDuplicateJointNames) {
  SimulationConfigParser parser;
  const std::filesystem::path config_path = write_config("robot_mujoco_duplicate_joint.xml",
                                                         R"(<robot_mujoco>
           <mujoco><mjcf>robot.xml</mjcf></mujoco>
           <robot>
             <joint name="joint_a" />
             <joint name="joint_a" />
           </robot>
         </robot_mujoco>)");

  SimulationConfig config = sentinel_config();
  const SimulationConfig before = config;
  EXPECT_FALSE(parser.load_file(config_path.string(), &config));
  EXPECT_EQ(config.model.model_path, before.model.model_path);
  EXPECT_EQ(config.components.size(), before.components.size());
}

TEST_F(RobotMujocoConfigTest, RejectsUnknownTag) {
  SimulationConfigParser parser;
  const std::filesystem::path config_path = write_config("robot_mujoco_unknown_tag.xml",
                                                         R"(<robot_mujoco>
           <mujoco><mjcf>robot.xml</mjcf></mujoco>
           <robot>
             <joint name="joint_a">
               <foo>1</foo>
             </joint>
           </robot>
         </robot_mujoco>)");

  SimulationConfig config = sentinel_config();
  const SimulationConfig before = config;
  EXPECT_FALSE(parser.load_file(config_path.string(), &config));
  EXPECT_EQ(config.model.model_path, before.model.model_path);
  EXPECT_EQ(config.components.size(), before.components.size());
}

}  // namespace
}  // namespace mujoco_simulation
