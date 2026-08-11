#include <cmath>
#include <iostream>

#include <mujoco/mujoco.h>

#include "component/mobile_base/mobile_base_component.hpp"
#include "test_support.hpp"

namespace {
bool check(bool value, const char* message) {
    if (!value) std::cerr << message << '\n';
    return value;
}

mujoco_simulation::MobileBaseInfo make_info() {
    mujoco_simulation::MobileBaseInfo info;
    info.mobile_base_name = "base";
    info.base_body_name = "base";
    info.base_joint_name = "base_free";
    info.period = 0.001;
    info.mecanum_info = {0.4, 0.3};
    const char* names[] = {"fl", "fr", "rl", "rr"};
    constexpr double radii[] = {0.1, 0.2, 0.1, 0.2};
    constexpr double directions[] = {-1.0, 1.0, -1.0, 1.0};
    for (std::size_t i = 0; i < mujoco_simulation::MecanumWheelCount; ++i)
        info.mecanum_wheels[i] = {names[i], radii[i], directions[i], 0.0};
    return info;
}
}  // namespace

int main() {
    mujoco_simulation_test::TemporaryFile model_file("mujoco_mobile_base_component_test.xml");
    if (!check(
            model_file.write(R"(<mujoco><option timestep="0.001"/><worldbody>
  <body name="base" pos="1 2 0.3"><freejoint name="base_free"/><geom type="sphere" size=".05" mass="1"/>
    <body><joint name="fl" type="hinge" axis="0 1 0"/><geom type="sphere" size=".01" mass=".01"/></body>
    <body><joint name="fr" type="hinge" axis="0 1 0"/><geom type="sphere" size=".01" mass=".01"/></body>
    <body><joint name="rl" type="hinge" axis="0 1 0"/><geom type="sphere" size=".01" mass=".01"/></body>
    <body><joint name="rr" type="hinge" axis="0 1 0"/><geom type="sphere" size=".01" mass=".01"/></body>
  </body></worldbody></mujoco>)"),
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
    mujoco_simulation::mjContext context(model, data);
    mujoco_simulation::MobileBaseComponent base(make_info());
    mujoco_simulation::MobileBaseCommand command;
    command.mode = mujoco_simulation::MobileBaseControlMode::Twist;
    command.base_linear[0] = 1.0;
    bool success = check(base.init(context), "initialization failed") &&
                   check(base.write(context, command), "twist command was rejected") &&
                   check(base.advance(context), "advance failed");
    mj_forward(model, data);
    success = success && check(base.update(context), "state publication failed");
    std::shared_ptr<const mujoco_simulation::MobileBaseState> state;
    success = success && check(base.read_state(state) && state != nullptr, "state unavailable") &&
              check(std::abs(state->pose[0] - 0.001) < 1.0e-12, "odom was not integrated") &&
              check(std::abs(data->qpos[0] - 1.001) < 1.0e-12, "world pose was not written") &&
              check(
                  std::abs(state->wheel_angular[0] + 10.0) < 1.0e-12,
                  "wheel joint direction was not applied") &&
              check(
                  std::abs(state->wheel_angular[1] - 5.0) < 1.0e-12,
                  "per-wheel radius was not applied") &&
              check(
                  std::abs(state->wheel_linear[0] + 1.0) < 1.0e-12,
                  "wheel linear state is not in joint coordinates");
    mj_step(model, data);
    success = success && check(base.advance(context), "second advance failed") &&
              check(std::abs(data->qpos[7] + 0.02) < 1.0e-12, "wheel angle was double-integrated");
    success = success && check(base.reset(context), "reset failed") &&
              check(base.update(context), "reset publication failed") &&
              check(
                  base.read_state(state) && std::abs(state->pose[0]) < 1.0e-12 &&
                      std::abs(state->wheel_angular[0]) < 1.0e-12 &&
                      std::abs(data->qpos[0] - 1.0) < 1.0e-12 && std::abs(data->qpos[7]) < 1.0e-12,
                  "reset did not clear kinematic state");
    mujoco_simulation::MobileBaseInfo invalid_response = make_info();
    invalid_response.mecanum_wheels[0].speed_response = -1.0;
    mujoco_simulation::MobileBaseComponent invalid_response_base(std::move(invalid_response));
    success =
        success && check(
                       !invalid_response_base.init(context),
                       "negative wheel speed_response was accepted without config validation");
    mujoco_simulation::MobileBaseInfo invalid_direction = make_info();
    invalid_direction.mecanum_wheels[0].direction = 0.0;
    mujoco_simulation::MobileBaseComponent invalid_direction_base(std::move(invalid_direction));
    success = success && check(
                             !invalid_direction_base.init(context),
                             "invalid wheel direction was accepted without validation");
    mujoco_simulation::MobileBaseInfo invalid_radius = make_info();
    invalid_radius.mecanum_wheels[0].radius = 0.0;
    mujoco_simulation::MobileBaseComponent invalid_radius_base(std::move(invalid_radius));
    success = success && check(
                             !invalid_radius_base.init(context),
                             "non-positive wheel radius was accepted without validation");
    mujoco_simulation::MobileBaseInfo duplicate_wheel = make_info();
    duplicate_wheel.mecanum_wheels[1].wheel_name = duplicate_wheel.mecanum_wheels[0].wheel_name;
    mujoco_simulation::MobileBaseComponent duplicate_wheel_base(std::move(duplicate_wheel));
    success = success && check(
                             !duplicate_wheel_base.init(context),
                             "duplicate MuJoCo wheel joint was accepted without config validation");
    context.clear();
    return success ? 0 : 1;
}
