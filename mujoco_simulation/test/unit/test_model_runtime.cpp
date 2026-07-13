#include <mujoco/mujoco.h>

#include <filesystem>
#include <fstream>
#include <memory>
#include <string>

#include "gtest/gtest.h"
#include "mujoco_simulation/runtime/model_runtime.hpp"

namespace {

constexpr char kFallingBodyXml[] = R"(
<mujoco model="model_runtime_test">
  <option timestep="0.01" gravity="0 0 -9.81"/>
  <worldbody>
    <geom type="plane" size="1 1 0.1"/>
    <body name="box" pos="0 0 0.5">
      <freejoint/>
      <geom type="box" size="0.05 0.05 0.05" mass="1"/>
    </body>
  </worldbody>
  <keyframe>
    <key name="raised" qpos="0 0 1 1 0 0 0"/>
  </keyframe>
</mujoco>
)";

class ModelRuntimeTest : public ::testing::Test {
 protected:
  std::filesystem::path write_model_file(const std::string& name, const std::string& contents) {
    const auto dir = std::filesystem::temp_directory_path() / "mujoco_simulation_tests";
    std::filesystem::create_directories(dir);
    const auto path = dir / name;
    std::ofstream out(path);
    out << contents;
    out.close();
    return path;
  }
};

}  // namespace

TEST_F(ModelRuntimeTest, LoadsStepsAndResetsMinimalMjcf) {
  mujoco_simulation::ModelRuntime runtime;
  const auto model_path = write_model_file("model_runtime_test.xml", kFallingBodyXml);

  const auto load_status = runtime.load({model_path.string()});
  ASSERT_EQ(load_status, mujoco_simulation::ResultCode::Ok);
  ASSERT_TRUE(runtime.is_loaded());
  EXPECT_GT(runtime.timestep(), 0.0);

  const double initial_z = runtime.data().qpos[2];
  const auto step_status = runtime.step(10);
  ASSERT_EQ(step_status, mujoco_simulation::ResultCode::Ok);
  EXPECT_GT(runtime.simulation_time(), 0.0);
  EXPECT_LT(runtime.data().qpos[2], initial_z);

  const auto reset_status = runtime.reset();
  ASSERT_EQ(reset_status, mujoco_simulation::ResultCode::Ok);
  EXPECT_NEAR(runtime.data().qpos[2], initial_z, 1e-9);
}

TEST_F(ModelRuntimeTest, ResetToNamedKeyframe) {
  mujoco_simulation::ModelRuntime runtime;
  const auto model_path = write_model_file("model_runtime_keyframe.xml", kFallingBodyXml);

  ASSERT_EQ(runtime.load({model_path.string()}), mujoco_simulation::ResultCode::Ok);
  ASSERT_EQ(runtime.reset({.keyframe_name = "raised"}), mujoco_simulation::ResultCode::Ok);
  EXPECT_NEAR(runtime.data().qpos[2], 1.0, 1e-9);
}

TEST_F(ModelRuntimeTest, LoadAppliesConfiguredInitialKeyframe) {
  mujoco_simulation::ModelRuntime runtime;
  const auto model_path = write_model_file("model_runtime_initial_keyframe.xml", kFallingBodyXml);

  const mujoco_simulation::ResultCode load_status =
      runtime.load({.model_path = model_path.string(), .initial_keyframe = "raised"});
  ASSERT_EQ(load_status, mujoco_simulation::ResultCode::Ok);
  EXPECT_NEAR(runtime.data().qpos[2], 1.0, 1e-9);
}

TEST_F(ModelRuntimeTest, RejectsMissingConfiguredInitialKeyframeAsModelValidationFailure) {
  mujoco_simulation::ModelRuntime runtime;
  const auto model_path = write_model_file("model_runtime_missing_keyframe.xml", kFallingBodyXml);

  const mujoco_simulation::ResultCode load_status =
      runtime.load({.model_path = model_path.string(), .initial_keyframe = "missing"});
  EXPECT_EQ(load_status, mujoco_simulation::ResultCode::ModelValidationFailed);
  EXPECT_FALSE(runtime.is_loaded());
}

TEST_F(ModelRuntimeTest, RejectsMalformedXmlAsModelLoadFailure) {
  mujoco_simulation::ModelRuntime runtime;
  const auto model_path = write_model_file("model_runtime_bad_xml.xml", "<mujoco><worldbody>");

  const mujoco_simulation::ResultCode load_status = runtime.load({model_path.string()});
  EXPECT_EQ(load_status, mujoco_simulation::ResultCode::ModelLoadFailed);
  EXPECT_FALSE(runtime.is_loaded());
}

TEST_F(ModelRuntimeTest, RejectsNonPositiveTimestepAsModelValidationFailure) {
  mujoco_simulation::ModelRuntime runtime;
  const auto model_path = write_model_file("model_runtime_bad_timestep.xml",
                                           R"(
<mujoco model="model_runtime_bad_timestep">
  <option timestep="0"/>
  <worldbody>
    <body>
      <joint name="hinge" type="hinge"/>
      <geom type="capsule" size="0.05 0.2"/>
    </body>
  </worldbody>
</mujoco>
)");

  const mujoco_simulation::ResultCode load_status = runtime.load({model_path.string()});
  EXPECT_EQ(load_status, mujoco_simulation::ResultCode::ModelValidationFailed);
  EXPECT_FALSE(runtime.is_loaded());
}

TEST_F(ModelRuntimeTest, ForwardRefreshesDerivedBodyState) {
  mujoco_simulation::ModelRuntime runtime;
  const auto model_path = write_model_file("model_runtime_forward.xml", kFallingBodyXml);

  ASSERT_EQ(runtime.load({model_path.string()}), mujoco_simulation::ResultCode::Ok);

  mjData& data = runtime.mutable_data();
  data.qpos[2] = 1.25;
  const int body_id = mj_name2id(&runtime.model(), mjOBJ_BODY, "box");
  ASSERT_GE(body_id, 0);

  const double stale_z = data.xpos[3 * body_id + 2];
  ASSERT_EQ(runtime.forward(), mujoco_simulation::ResultCode::Ok);
  const double fresh_z = data.xpos[3 * body_id + 2];

  EXPECT_NE(stale_z, fresh_z);
  EXPECT_NEAR(fresh_z, 1.25, 1e-6);
}

TEST_F(ModelRuntimeTest, ResetRejectsMissingRuntimeKeyframeWithoutUnloadingModel) {
  mujoco_simulation::ModelRuntime runtime;
  const auto model_path = write_model_file("model_runtime_runtime_keyframe.xml", kFallingBodyXml);

  ASSERT_EQ(runtime.load({model_path.string()}), mujoco_simulation::ResultCode::Ok);
  const mujoco_simulation::ResultCode reset_status = runtime.reset({.keyframe_name = "missing"});
  EXPECT_EQ(reset_status, mujoco_simulation::ResultCode::NotFound);
  EXPECT_TRUE(runtime.is_loaded());
}

TEST_F(ModelRuntimeTest, UnloadAndReloadRemainStableAcrossRepeatedCycles) {
  mujoco_simulation::ModelRuntime runtime;
  const auto model_path = write_model_file("model_runtime_reload.xml", kFallingBodyXml);

  for (int cycle = 0; cycle < 3; ++cycle) {
    const mujoco_simulation::ResultCode load_status = runtime.load({model_path.string()});
    ASSERT_EQ(load_status, mujoco_simulation::ResultCode::Ok);
    EXPECT_TRUE(runtime.is_loaded());

    const mujoco_simulation::ResultCode step_status = runtime.step(2);
    ASSERT_EQ(step_status, mujoco_simulation::ResultCode::Ok);
    EXPECT_GT(runtime.simulation_time(), 0.0);

    runtime.unload();
    EXPECT_FALSE(runtime.is_loaded());
    EXPECT_DOUBLE_EQ(runtime.simulation_time(), 0.0);
    EXPECT_DOUBLE_EQ(runtime.timestep(), 0.0);
  }
}

TEST_F(ModelRuntimeTest, RejectsInvalidInput) {
  mujoco_simulation::ModelRuntime runtime;

  const auto load_status = runtime.load({"/tmp/does_not_exist.xml"});
  EXPECT_EQ(load_status, mujoco_simulation::ResultCode::ModelLoadFailed);

  const auto step_status = runtime.step();
  EXPECT_EQ(step_status, mujoco_simulation::ResultCode::FailedPrecondition);

  const auto reset_status = runtime.reset();
  EXPECT_EQ(reset_status, mujoco_simulation::ResultCode::FailedPrecondition);
}
