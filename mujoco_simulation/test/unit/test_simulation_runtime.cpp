#include <mujoco/mujoco.h>

#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <utility>

#include "gtest/gtest.h"
#include "mujoco_simulation/runtime/simulation_runtime.hpp"

namespace mujoco_simulation {

class SimulationRuntimeTestPeer {
 public:
  static const mjContext& context(const SimulationRuntime& runtime) { return runtime.context(); }
};

}  // namespace mujoco_simulation

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

class SimulationRuntimeTest : public ::testing::Test {
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

TEST_F(SimulationRuntimeTest, LoadsStepsAndResetsMinimalMjcf) {
  mujoco_simulation::SimulationRuntime runtime;
  const auto model_path = write_model_file("model_runtime_test.xml", kFallingBodyXml);

  ASSERT_TRUE(runtime.init({model_path.string()}));
  ASSERT_TRUE(runtime.is_initialized());
  EXPECT_GT(runtime.timestep(), 0.0);

  const mujoco_simulation::mjContext& context =
      mujoco_simulation::SimulationRuntimeTestPeer::context(runtime);
  const double initial_z = context.data->qpos[2];
  ASSERT_TRUE(runtime.step(10));
  EXPECT_GT(runtime.simulation_time(), 0.0);
  EXPECT_LT(context.data->qpos[2], initial_z);

  ASSERT_TRUE(runtime.reset());
  EXPECT_NEAR(context.data->qpos[2], initial_z, 1e-9);
}

TEST_F(SimulationRuntimeTest, ResetToNamedKeyframe) {
  mujoco_simulation::SimulationRuntime runtime;
  const auto model_path = write_model_file("model_runtime_keyframe.xml", kFallingBodyXml);

  ASSERT_TRUE(runtime.init({model_path.string()}));
  ASSERT_TRUE(runtime.reset_to_keyframe_name("raised"));
  EXPECT_NEAR(mujoco_simulation::SimulationRuntimeTestPeer::context(runtime).data->qpos[2], 1.0,
              1e-9);
}

TEST_F(SimulationRuntimeTest, LoadAppliesConfiguredInitialKeyframe) {
  mujoco_simulation::SimulationRuntime runtime;
  const auto model_path = write_model_file("model_runtime_initial_keyframe.xml", kFallingBodyXml);

  ASSERT_TRUE(runtime.init({.model_path = model_path.string(), .initial_keyframe = "raised"}));
  EXPECT_NEAR(mujoco_simulation::SimulationRuntimeTestPeer::context(runtime).data->qpos[2], 1.0,
              1e-9);
}

TEST_F(SimulationRuntimeTest, RejectsMissingConfiguredInitialKeyframeAsModelValidationFailure) {
  mujoco_simulation::SimulationRuntime runtime;
  const auto model_path = write_model_file("model_runtime_missing_keyframe.xml", kFallingBodyXml);

  EXPECT_FALSE(runtime.init({.model_path = model_path.string(), .initial_keyframe = "missing"}));
  EXPECT_FALSE(runtime.is_initialized());
}

TEST_F(SimulationRuntimeTest, RejectsMalformedXmlAsModelLoadFailure) {
  mujoco_simulation::SimulationRuntime runtime;
  const auto model_path = write_model_file("model_runtime_bad_xml.xml", "<mujoco><worldbody>");

  EXPECT_FALSE(runtime.init({model_path.string()}));
  EXPECT_FALSE(runtime.is_initialized());
}

TEST_F(SimulationRuntimeTest, RejectsBinaryModelFiles) {
  mujoco_simulation::SimulationRuntime runtime;
  const auto model_path = write_model_file("model_runtime_binary.mjb", "not a binary model");

  EXPECT_FALSE(runtime.init({model_path.string()}));
  EXPECT_FALSE(runtime.is_initialized());
}

TEST_F(SimulationRuntimeTest, RejectsNonPositiveTimestepAsModelValidationFailure) {
  mujoco_simulation::SimulationRuntime runtime;
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

  EXPECT_FALSE(runtime.init({model_path.string()}));
  EXPECT_FALSE(runtime.is_initialized());
}

TEST_F(SimulationRuntimeTest, ForwardRefreshesDerivedBodyState) {
  mujoco_simulation::SimulationRuntime runtime;
  const auto model_path = write_model_file("model_runtime_forward.xml", kFallingBodyXml);

  ASSERT_TRUE(runtime.init({model_path.string()}));

  const mujoco_simulation::mjContext& context =
      mujoco_simulation::SimulationRuntimeTestPeer::context(runtime);
  context.data->qpos[2] = 1.25;
  const int body_id = mj_name2id(context.model, mjOBJ_BODY, "box");
  ASSERT_GE(body_id, 0);

  const double stale_z = context.data->xpos[3 * body_id + 2];
  ASSERT_TRUE(runtime.forward());
  const double fresh_z = context.data->xpos[3 * body_id + 2];

  EXPECT_NE(stale_z, fresh_z);
  EXPECT_NEAR(fresh_z, 1.25, 1e-6);
}

TEST_F(SimulationRuntimeTest, ResetRejectsMissingRuntimeKeyframeWithoutUnloadingModel) {
  mujoco_simulation::SimulationRuntime runtime;
  const auto model_path = write_model_file("model_runtime_runtime_keyframe.xml", kFallingBodyXml);

  ASSERT_TRUE(runtime.init({model_path.string()}));
  EXPECT_FALSE(runtime.reset_to_keyframe_name("missing"));
  EXPECT_TRUE(runtime.is_initialized());
}

TEST_F(SimulationRuntimeTest, RepeatedLoadReplacesPreviousRuntimeAndRemainsStable) {
  mujoco_simulation::SimulationRuntime runtime;
  const auto model_path = write_model_file("model_runtime_reload.xml", kFallingBodyXml);

  for (int cycle = 0; cycle < 3; ++cycle) {
    ASSERT_TRUE(runtime.init({model_path.string()}));
    EXPECT_TRUE(runtime.is_initialized());

    ASSERT_TRUE(runtime.step(2));
    EXPECT_GT(runtime.simulation_time(), 0.0);
  }

  ASSERT_TRUE(runtime.init({model_path.string()}));
  EXPECT_TRUE(runtime.is_initialized());
  EXPECT_DOUBLE_EQ(runtime.simulation_time(), 0.0);
  EXPECT_GT(runtime.timestep(), 0.0);
}

TEST_F(SimulationRuntimeTest, FailedReloadInvalidatesPreviousRuntime) {
  mujoco_simulation::SimulationRuntime runtime;
  const auto valid_model_path = write_model_file("model_runtime_valid_reload.xml", kFallingBodyXml);

  ASSERT_TRUE(runtime.init({valid_model_path.string()}));
  ASSERT_TRUE(runtime.is_initialized());
  ASSERT_TRUE(runtime.step());
  EXPECT_GT(runtime.simulation_time(), 0.0);

  EXPECT_FALSE(runtime.init({"/tmp/does_not_exist.xml"}));
  EXPECT_FALSE(runtime.is_initialized());
  EXPECT_DOUBLE_EQ(runtime.simulation_time(), 0.0);
  EXPECT_DOUBLE_EQ(runtime.timestep(), 0.0);
  EXPECT_FALSE(runtime.step());
}

TEST_F(SimulationRuntimeTest, MoveTransfersLoadedResources) {
  const auto model_path = write_model_file("model_runtime_move.xml", kFallingBodyXml);

  mujoco_simulation::SimulationRuntime source;
  ASSERT_TRUE(source.init({model_path.string()}));

  mujoco_simulation::SimulationRuntime moved(std::move(source));
  EXPECT_FALSE(source.is_initialized());
  ASSERT_TRUE(moved.is_initialized());
  EXPECT_TRUE(moved.step());

  mujoco_simulation::SimulationRuntime target;
  ASSERT_TRUE(target.init({model_path.string()}));
  target = std::move(moved);
  EXPECT_FALSE(moved.is_initialized());
  ASSERT_TRUE(target.is_initialized());
  EXPECT_TRUE(target.step());
}

TEST_F(SimulationRuntimeTest, RejectsInvalidInput) {
  mujoco_simulation::SimulationRuntime runtime;

  EXPECT_FALSE(runtime.init({"/tmp/does_not_exist.xml"}));

  EXPECT_FALSE(runtime.step());

  EXPECT_FALSE(runtime.reset());
}
