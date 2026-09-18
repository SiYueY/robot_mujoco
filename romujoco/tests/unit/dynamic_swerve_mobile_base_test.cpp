#include <cmath>
#include <iostream>
#include <memory>

#include <mujoco/mujoco.h>

#include "component/mobile_base/swerve/dynamic_swerve_mobile_base.hpp"
#include "test_support.hpp"

namespace {
bool check(bool value, const char* message) {
    if (!value) std::cerr << message << '\n';
    return value;
}

romujoco::SwerveMobileBaseInfo make_info() {
    romujoco::SwerveMobileBaseInfo info;
    info.common.id = 7;
    info.common.name = "base";
    info.common.base_body_name = "base";
    info.common.period = 0.001;
    info.common.execution_mode = romujoco::MobileBaseExecutionMode::Dynamic;
    info.modules = {
        {"front", 0.3, -0.2, 0.05, "front_steer", "front_position", "front_drive",
         "front_velocity"},
        {"rear", -0.3, 0.2, 0.05, "rear_steer", "rear_position", "rear_drive", "rear_velocity"}};
    return info;
}
}  // namespace

int main() {
    romujoco_test::TemporaryFile model_file("mujoco_dynamic_swerve_mobile_base_test.xml");
    if (!check(
            model_file.write(R"(<mujoco><option timestep="0.001"/><worldbody>
 <body name="base"><freejoint/><geom type="sphere" size=".05" mass="1"/>
  <body><joint name="front_steer" type="hinge"/><geom type="sphere" size=".01" mass=".01"/><body><joint name="front_drive" type="hinge"/><geom type="sphere" size=".01" mass=".01"/></body></body>
  <body><joint name="rear_steer" type="hinge"/><geom type="sphere" size=".01" mass=".01"/><body><joint name="rear_drive" type="hinge"/><geom type="sphere" size=".01" mass=".01"/></body></body>
 </body></worldbody><actuator>
 <position name="front_position" joint="front_steer" kp="1"/><velocity name="front_velocity" joint="front_drive" kv="1"/>
 <position name="rear_position" joint="rear_steer" kp="1"/><velocity name="rear_velocity" joint="rear_drive" kv="1"/>
 </actuator></mujoco>)"),
            "failed to write model"))
        return 1;
    char error[1024] = {};
    mjModel* model = mj_loadXML(model_file.path().c_str(), nullptr, error, sizeof(error));
    if (!check(model != nullptr, error)) return 1;
    mjData* data = mj_makeData(model);
    if (!check(data != nullptr, "failed to allocate data")) {
        mj_deleteModel(model);
        return 1;
    }
    romujoco::SimulationContext context(model, data);
    romujoco::DynamicSwerveMobileBase base(make_info());
    romujoco::MobileBaseCommand command;
    command.id = 7;
    command.velocity.linear_x = 1.0;
    bool success =
        check(base.init(context), "dynamic swerve initialization failed") &&
        check(base.write(context, command), "swerve command rejected") &&
        check(
            std::abs(data->ctrl[0]) < 1e-12 && std::abs(data->ctrl[1] - 20.0) < 1e-12 &&
                std::abs(data->ctrl[2]) < 1e-12 && std::abs(data->ctrl[3] - 20.0) < 1e-12,
            "swerve IK targets are incorrect");
    data->qpos[7] = romujoco::kPi;
    success = success && check(base.write(context, command), "reverse command rejected") &&
              check(
                  std::abs(std::abs(data->ctrl[0]) - romujoco::kPi) < 1e-12 &&
                      std::abs(data->ctrl[1] + 20.0) < 1e-12,
                  "swerve shortest-path reversal is incorrect");
    data->qpos[3] = std::sqrt(0.5);
    data->qpos[6] = std::sqrt(0.5);
    data->qvel[0] = 1.0;
    success = success && check(base.update(context), "state update failed");
    std::shared_ptr<const romujoco::MobileBaseState> state;
    success = success && check(
                             base.read_state(state) && state != nullptr &&
                                 std::abs(state->twist.linear[0]) < 1e-12 &&
                                 std::abs(state->twist.linear[1] + 1.0) < 1e-12,
                             "world velocity was not converted to the base frame");
    context.clear();
    return success ? 0 : 1;
}
