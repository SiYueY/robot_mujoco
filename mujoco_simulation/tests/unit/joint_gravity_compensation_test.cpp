#include <mujoco/mujoco.h>

#include <algorithm>
#include <cmath>
#include <iostream>

#include "component/joint/joint_component.hpp"
#include "runtime/context.hpp"
#include "test_support.hpp"

namespace {

bool check(bool value, const char* message) {
    if (!value) std::cerr << message << '\n';
    return value;
}

bool close(double actual, double expected) { return std::abs(actual - expected) < 1e-10; }

mujoco_simulation::JointInfo joint_info(bool gravity_compensation) {
    mujoco_simulation::JointInfo info;
    info.id = 0;
    info.joint_name = "joint";
    info.actuator_name = "motor";
    info.default_mode = mujoco_simulation::JointMode::Effort;
    info.allowed_modes = {
        mujoco_simulation::JointMode::Hybrid, mujoco_simulation::JointMode::Position,
        mujoco_simulation::JointMode::Velocity, mujoco_simulation::JointMode::Effort};
    info.period = 0.001;
    info.hybrid.gravity_compensation = gravity_compensation;
    info.position.gravity_compensation = gravity_compensation;
    info.velocity.gravity_compensation = gravity_compensation;
    info.effort.gravity_compensation = gravity_compensation;
    return info;
}

}  // namespace

int main() {
    mujoco_simulation_test::TemporaryFile model_file("mujoco_joint_gravity_compensation_test.xml");
    if (!check(
            model_file.write(R"(<mujoco><option timestep="0.001" gravity="0 0 -9.81"/>
<worldbody><body name="pendulum"><joint name="joint" type="hinge" axis="0 1 0"/>
  <inertial pos="0 0 -0.5" mass="1" diaginertia="0.1 0.1 0.1"/>
</body></worldbody><actuator><motor name="motor" joint="joint" ctrlrange="-20 20"/></actuator>
</mujoco>)"),
            "failed to write MuJoCo model")) {
        return 1;
    }

    char error[1024] = {};
    mjModel* model = mj_loadXML(model_file.path().c_str(), nullptr, error, sizeof(error));
    if (!check(model != nullptr, error)) return 1;
    mjData* data = mj_makeData(model);
    if (!check(data != nullptr, "failed to allocate MuJoCo data")) {
        mj_deleteModel(model);
        return 1;
    }
    mujoco_simulation::mjContext context(model, data);
    data->qpos[0] = 1.5707963267948966;
    data->qvel[0] = 2.0;
    data->qacc[0] = 3.0;
    mj_forward(model, data);

    mjData* expected_data = mj_makeData(model);
    if (!check(expected_data != nullptr, "failed to allocate expected MuJoCo data")) {
        context.clear();
        return 1;
    }
    expected_data->qpos[0] = data->qpos[0];
    mj_forward(model, expected_data);
    const double gravity_effort = expected_data->qfrc_bias[0];
    mj_deleteData(expected_data);
    if (!check(std::abs(gravity_effort) > 1.0, "test model did not produce gravity effort")) {
        context.clear();
        return 1;
    }

    const double qpos_before = data->qpos[0];
    const double qvel_before = data->qvel[0];
    const double qacc_before = data->qacc[0];
    mujoco_simulation::JointComponent component(joint_info(true));
    if (!check(component.init(context), "failed to initialize gravity-compensated joint")) {
        context.clear();
        return 1;
    }

    mujoco_simulation::JointCommand command;
    command.id = 0;
    command.mode = static_cast<std::uint8_t>(mujoco_simulation::JointMode::Position);
    command.position = qpos_before;
    if (!check(component.write(context, command), "position command was rejected") ||
        !check(
            close(data->ctrl[0], gravity_effort), "position gravity compensation was incorrect")) {
        context.clear();
        return 1;
    }

    command.mode = static_cast<std::uint8_t>(mujoco_simulation::JointMode::Velocity);
    command.velocity = qvel_before;
    if (!check(component.write(context, command), "velocity command was rejected") ||
        !check(
            close(data->ctrl[0], gravity_effort), "velocity gravity compensation was incorrect")) {
        context.clear();
        return 1;
    }

    command.mode = static_cast<std::uint8_t>(mujoco_simulation::JointMode::Hybrid);
    command.position = qpos_before;
    command.velocity = qvel_before;
    command.effort = 0.0;
    command.stiffness = 0.0;
    command.damping = 0.0;
    if (!check(component.write(context, command), "hybrid command was rejected") ||
        !check(close(data->ctrl[0], gravity_effort), "hybrid gravity compensation was incorrect")) {
        context.clear();
        return 1;
    }

    command.mode = static_cast<std::uint8_t>(mujoco_simulation::JointMode::Effort);
    command.effort = 0.0;
    if (!check(component.write(context, command), "effort command was rejected") ||
        !check(close(data->ctrl[0], gravity_effort), "effort gravity compensation was incorrect") ||
        !check(
            close(data->qpos[0], qpos_before) && close(data->qvel[0], qvel_before) &&
                close(data->qacc[0], qacc_before),
            "gravity calculation modified live MuJoCo state")) {
        context.clear();
        return 1;
    }

    data->qpos[0] = 0.0;
    mj_forward(model, data);
    command.effort = 0.0;
    if (!check(component.write(context, command), "pose-change effort command was rejected") ||
        !check(
            std::abs(data->ctrl[0]) < 1e-10,
            "gravity compensation did not follow the current pose")) {
        context.clear();
        return 1;
    }

    data->qpos[0] = qpos_before;
    mj_forward(model, data);
    mujoco_simulation::JointComponent disabled_component(joint_info(false));
    if (!check(disabled_component.init(context), "failed to initialize uncompensated joint") ||
        !check(
            disabled_component.write(context, command),
            "uncompensated effort command was rejected") ||
        !check(close(data->ctrl[0], 0.0), "unconfigured effort mode added gravity compensation")) {
        context.clear();
        return 1;
    }

    auto limited_info = joint_info(true);
    limited_info.effort_limits = {-0.25, 0.25};
    mujoco_simulation::JointComponent limited_component(std::move(limited_info));
    if (!check(limited_component.init(context), "failed to initialize limited joint") ||
        !check(limited_component.write(context, command), "limited effort command was rejected") ||
        !check(
            close(data->ctrl[0], std::clamp(gravity_effort, -0.25, 0.25)),
            "gravity compensation was not subject to effort limits")) {
        context.clear();
        return 1;
    }

    auto continuous_info = joint_info(false);
    continuous_info.position.stiffness = 1.0;
    continuous_info.position.damping = 0.0;
    mujoco_simulation::JointComponent continuous_component(std::move(continuous_info));
    data->qpos[0] = 3.13;
    data->qvel[0] = 0.0;
    mj_forward(model, data);
    command.mode = static_cast<std::uint8_t>(mujoco_simulation::JointMode::Position);
    command.position = -3.13;
    if (!check(continuous_component.init(context), "failed to initialize continuous joint") ||
        !check(
            continuous_component.write(context, command), "continuous position command rejected") ||
        !check(
            data->ctrl[0] > 0.0 && data->ctrl[0] < 0.1,
            "continuous position command did not use shortest angular distance")) {
        context.clear();
        return 1;
    }

    mujoco_simulation::JointInfo invalid_passive;
    invalid_passive.id = 1;
    invalid_passive.joint_name = "joint";
    invalid_passive.actuation = mujoco_simulation::JointActuation::Passive;
    invalid_passive.default_mode = mujoco_simulation::JointMode::None;
    invalid_passive.period = 0.001;
    mujoco_simulation::JointComponent passive_with_actuator(std::move(invalid_passive));
    if (!check(
            !passive_with_actuator.init(context),
            "passive joint with a MuJoCo actuator was accepted")) {
        context.clear();
        return 1;
    }

    context.clear();
    return 0;
}
