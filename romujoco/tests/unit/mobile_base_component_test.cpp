#include <cmath>
#include <iostream>

#include <mujoco/mujoco.h>

#include "component/mobile_base/mecanum/kinematic_mecanum_mobile_base.hpp"
#include "test_support.hpp"

namespace {
bool check(bool value, const char* message) {
    if (!value) std::cerr << message << '\n';
    return value;
}

romujoco::MecanumMobileBaseInfo make_info() {
    romujoco::MecanumMobileBaseInfo info;
    info.common.id = 1;
    info.common.name = "base";
    info.common.base_body_name = "base";
    info.common.period = 0.001;
    info.common.execution_mode = romujoco::MobileBaseExecutionMode::Kinematic;
    info.wheel_base = 0.4;
    info.track_width = 0.3;
    const char* names[] = {"fl", "fr", "rl", "rr"};
    constexpr double radii[] = {0.1, 0.2, 0.1, 0.2};
    constexpr double directions[] = {-1.0, 1.0, -1.0, 1.0};
    for (std::size_t i = 0; i < romujoco::MecanumWheelCount; ++i)
        info.wheels[i] = {names[i], radii[i], directions[i], 0.0};
    return info;
}
}  // namespace

int main() {
    romujoco_test::TemporaryFile model_file("mujoco_mobile_base_component_test.xml");
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
    romujoco::mjContext context(model, data);
    romujoco::KinematicMecanumMobileBase base(make_info());
    romujoco::MobileBaseCommand command;
    command.id = 1;
    command.velocity.linear_x = 1.0;
    bool success = check(base.init(context), "initialization failed") &&
                   check(base.write(context, command), "twist command was rejected") &&
                   check(base.advance(context), "advance failed");
    mj_forward(model, data);
    success = success && check(base.update(context), "state publication failed");
    std::shared_ptr<const romujoco::MobileBaseState> state;
    success =
        success && check(base.read_state(state) && state != nullptr, "state unavailable") &&
        check(std::abs(state->pose.position[0] - 1.001) < 1.0e-12, "pose was not integrated") &&
        check(std::abs(data->qpos[0] - 1.001) < 1.0e-12, "world pose was not written") &&
        check(std::abs(state->twist.linear[0] - 1.0) < 1.0e-12, "base twist was not published");
    mj_step(model, data);
    success = success && check(base.advance(context), "second advance failed") &&
              check(std::abs(data->qpos[7] + 0.02) < 1.0e-12, "wheel angle was double-integrated");
    success = success && check(base.reset(context), "reset failed") &&
              check(base.update(context), "reset publication failed") &&
              check(
                  base.read_state(state) && std::abs(state->pose.position[0] - 1.0) < 1.0e-12 &&
                      std::abs(data->qpos[0] - 1.0) < 1.0e-12 && std::abs(data->qpos[7]) < 1.0e-12,
                  "reset did not clear kinematic state");
    romujoco::MecanumMobileBaseInfo invalid_response = make_info();
    invalid_response.wheels[0].speed_response = -1.0;
    romujoco::KinematicMecanumMobileBase invalid_response_base(std::move(invalid_response));
    success =
        success && check(
                       !invalid_response_base.init(context),
                       "negative wheel speed_response was accepted without config validation");
    romujoco::MecanumMobileBaseInfo invalid_direction = make_info();
    invalid_direction.wheels[0].direction = 0.0;
    romujoco::KinematicMecanumMobileBase invalid_direction_base(std::move(invalid_direction));
    success = success && check(
                             !invalid_direction_base.init(context),
                             "invalid wheel direction was accepted without validation");
    romujoco::MecanumMobileBaseInfo invalid_radius = make_info();
    invalid_radius.wheels[0].radius = 0.0;
    romujoco::KinematicMecanumMobileBase invalid_radius_base(std::move(invalid_radius));
    success = success && check(
                             !invalid_radius_base.init(context),
                             "non-positive wheel radius was accepted without validation");
    romujoco::MecanumMobileBaseInfo duplicate_wheel = make_info();
    duplicate_wheel.wheels[1].joint_name = duplicate_wheel.wheels[0].joint_name;
    romujoco::KinematicMecanumMobileBase duplicate_wheel_base(std::move(duplicate_wheel));
    success = success && check(
                             !duplicate_wheel_base.init(context),
                             "duplicate MuJoCo wheel joint was accepted without config validation");
    context.clear();
    return success ? 0 : 1;
}
