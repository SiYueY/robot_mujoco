#include <mujoco/mujoco.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>

#include "component/component_manager.hpp"
#include "test_support.hpp"

namespace {

bool check(bool value, const char* message) {
    if (!value) std::cerr << message << '\n';
    return value;
}

class NoopCameraRenderService final : public mujoco_simulation::CameraRenderService {
public:
    bool initialize(const mujoco_simulation::SimulationConfig&, const mjModel*) override {
        return true;
    }
    mujoco_simulation::CameraRenderSubmitResult submit(
        const mujoco_simulation::CameraRenderBatchRequest&,
        mujoco_simulation::CameraRenderTicket&) override {
        return mujoco_simulation::CameraRenderSubmitResult::InvalidRequest;
    }
    mujoco_simulation::CameraRenderWaitStatus wait(
        const mujoco_simulation::CameraRenderTicket&, std::chrono::milliseconds) override {
        return mujoco_simulation::CameraRenderWaitStatus::InvalidTicket;
    }
    mujoco_simulation::CameraRenderWaitStatus query(
        const mujoco_simulation::CameraRenderTicket&) const override {
        return mujoco_simulation::CameraRenderWaitStatus::InvalidTicket;
    }
    bool read_batch_result(
        const mujoco_simulation::CameraRenderTicket&,
        mujoco_simulation::CameraRenderBatchResult&) override {
        return false;
    }
    bool reset() override { return true; }
    bool shutdown() override { return true; }
};

mujoco_simulation::JointInfo joint_info(
    mujoco_simulation::JointId id, const char* joint_name, const char* actuator_name) {
    mujoco_simulation::JointInfo info;
    info.id = id;
    info.joint_name = joint_name;
    info.actuator_name = actuator_name;
    info.default_mode = mujoco_simulation::JointMode::Effort;
    info.allowed_modes = {mujoco_simulation::JointMode::Effort};
    info.period = 0.002;
    return info;
}

mujoco_simulation::JointInfo passive_joint_info(
    mujoco_simulation::JointId id, const char* joint_name) {
    mujoco_simulation::JointInfo info;
    info.id = id;
    info.joint_name = joint_name;
    info.actuation = mujoco_simulation::JointActuation::Passive;
    info.default_mode = mujoco_simulation::JointMode::None;
    info.period = 0.002;
    return info;
}

}  // namespace

int main() {
    mujoco_simulation_test::TemporaryFile model_file(
        "mujoco_component_manager_sparse_command_test.xml");
    if (!check(
            model_file.write(R"(<mujoco><worldbody><body name="body">
  <joint name="joint_0" type="hinge"/>
  <joint name="joint_passive" type="hinge"/>
  <joint name="joint_2" type="hinge"/>
  <geom type="sphere" size="0.01" mass="1"/>
</body></worldbody><actuator>
  <motor name="actuator_0" joint="joint_0"/>
  <motor name="actuator_2" joint="joint_2"/>
</actuator></mujoco>)"),
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

    mujoco_simulation::ComponentConfigList components{
        joint_info(0, "joint_0", "actuator_0"), passive_joint_info(1, "joint_passive"),
        joint_info(2, "joint_2", "actuator_2")};
    NoopCameraRenderService camera_service;
    mujoco_simulation::ComponentManager manager;
    if (!check(
            manager.init(context, components, camera_service),
            "failed to initialize sparse joint components")) {
        context.clear();
        return 1;
    }
    if (!check(manager.update(context), "failed to publish sparse joint states")) {
        context.clear();
        return 1;
    }
    mujoco_simulation::RobotState state;
    if (!check(
            manager.read_state(context, state) && state.joints != nullptr &&
                state.joints->size() == 3U && (*state.joints)[0]->id == 0U &&
                (*state.joints)[1]->id == 1U &&
                (*state.joints)[1]->mode ==
                    static_cast<std::uint8_t>(mujoco_simulation::JointMode::None) &&
                (*state.joints)[2]->id == 2U,
            "joint states were not published as a compact ID-sorted list")) {
        context.clear();
        return 1;
    }
    mujoco_simulation::JointCommands reset_commands;
    if (!check(manager.reset(context, reset_commands), "component reset failed") ||
        !check(
            reset_commands.size() == 2U && reset_commands[0].id == 0U && reset_commands[1].id == 2U,
            "passive joint generated a reset command")) {
        context.clear();
        return 1;
    }

    mujoco_simulation::RobotCommand command;
    command.joints.resize(2);
    command.joints[0].id = 2;
    command.joints[0].mode = static_cast<std::uint8_t>(mujoco_simulation::JointMode::Effort);
    command.joints[0].effort = 2.5;
    command.joints[1].id = 0;
    command.joints[1].mode = static_cast<std::uint8_t>(mujoco_simulation::JointMode::Effort);
    command.joints[1].effort = 1.5;
    if (!check(manager.write_command(context, command), "sparse command snapshot was rejected")) {
        context.clear();
        return 1;
    }

    const int actuator_0 = mj_name2id(model, mjOBJ_ACTUATOR, "actuator_0");
    const int actuator_2 = mj_name2id(model, mjOBJ_ACTUATOR, "actuator_2");
    const bool applied = actuator_0 >= 0 && actuator_2 >= 0 &&
                         std::abs(data->ctrl[actuator_0] - 1.5) < 1e-12 &&
                         std::abs(data->ctrl[actuator_2] - 2.5) < 1e-12;
    mujoco_simulation::RobotCommand invalid_batch;
    invalid_batch.joints.resize(2);
    invalid_batch.joints[0].id = 0;
    invalid_batch.joints[0].mode = static_cast<std::uint8_t>(mujoco_simulation::JointMode::Effort);
    invalid_batch.joints[0].effort = 9.0;
    invalid_batch.joints[1].id = 2;
    invalid_batch.joints[1].mode = static_cast<std::uint8_t>(mujoco_simulation::JointMode::Effort);
    invalid_batch.joints[1].effort = std::numeric_limits<double>::quiet_NaN();
    const bool partial_apply = !manager.write_command(context, invalid_batch) && actuator_0 >= 0 &&
                               actuator_2 >= 0 && std::abs(data->ctrl[actuator_0] - 9.0) < 1e-12 &&
                               std::abs(data->ctrl[actuator_2] - 2.5) < 1e-12;
    mujoco_simulation::RobotCommand passive_command;
    passive_command.joints.push_back(
        {1, static_cast<std::uint8_t>(mujoco_simulation::JointMode::Effort)});
    const bool passive_rejected = !manager.write_command(context, passive_command);
    context.clear();
    return check(applied, "sparse command slots were not applied correctly") &&
                   check(
                       partial_apply,
                       "invalid batch did not preserve the preceding command effect") &&
                   check(passive_rejected, "passive joint command was accepted")
               ? 0
               : 1;
}
