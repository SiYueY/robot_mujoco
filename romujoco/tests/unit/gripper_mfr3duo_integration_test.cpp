#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <utility>

#include <mujoco/mujoco.h>

#include "component/gripper/gripper_component.hpp"

namespace {

// Integration test against the real MFR3Duo MJCF model.  The model is not part
// of this repository, so the test reports a skip unless ROMUJOCO_MFR3DUO_MODEL
// points at mfr3duo.xml.
// Full-stack coverage (XML configuration -> Simulation -> buffers) lives in
// romujoco.gripper_runtime and romujoco.gripper_config.

bool check(bool value, const char* message) {
    if (!value) std::cerr << message << '\n';
    return value;
}

// Finger position at which the fixed object starts to resist a closing motion.
constexpr double kObjectFingerStop = 0.012;

romujoco::GripperInfo make_gripper_info(
    romujoco::GripperId id, std::string name, romujoco::GripperFingerInfo first_finger,
    romujoco::GripperFingerInfo second_finger) {
    romujoco::GripperInfo info;
    info.id = id;
    info.name = std::move(name);
    info.period = 0.001;
    info.fingers = {std::move(first_finger), std::move(second_finger)};
    info.control.stiffness = 200.0;
    info.control.damping = 8.0;
    info.width_limits = {0.0, 0.08};
    info.velocity_limits = {0.0, 0.5};
    info.effort_limits = {0.0, 20.0};
    info.stall.width_tolerance = 0.001;
    info.stall.velocity_threshold = 0.01;
    info.stall.effort_ratio = 0.9;
    info.stall.timeout = 0.05;
    return info;
}

int qpos_address(const mjModel& model, const std::string& joint_name) {
    const int joint = mj_name2id(&model, mjOBJ_JOINT, joint_name.c_str());
    return joint < 0 ? -1 : model.jnt_qposadr[joint];
}

int dof_address(const mjModel& model, const std::string& joint_name) {
    const int joint = mj_name2id(&model, mjOBJ_JOINT, joint_name.c_str());
    return joint < 0 ? -1 : model.jnt_dofadr[joint];
}

bool send_command(
    romujoco::GripperComponent& gripper, const romujoco::SimulationContext& context, double width,
    double velocity, double effort) {
    romujoco::GripperCommand command;
    command.id = gripper.info().id;
    command.width = width;
    command.velocity = velocity;
    command.effort = effort;
    return gripper.write(context, command);
}

bool read_state(const romujoco::GripperComponent& gripper, romujoco::GripperState& state) {
    std::shared_ptr<const romujoco::GripperState> snapshot;
    if (!gripper.read_state(snapshot) || snapshot == nullptr) return false;
    state = *snapshot;
    return true;
}

// Steps the physics together with both grippers for the given number of steps.
bool simulate(
    const romujoco::SimulationContext& context, int steps, romujoco::GripperComponent& left,
    romujoco::GripperComponent& right) {
    for (int step = 0; step < steps; ++step) {
        if (!left.advance(context) || !right.advance(context)) return false;
        mj_step(context.model, context.data);
        if (!left.update(context) || !right.update(context)) return false;
    }
    return true;
}

// Steps the physics while a fixed object blocks the left fingers: the object is
// modelled as a one-sided spring that only acts when a finger closes past its
// width, because the external model contains no graspable body.
bool simulate_blocked(
    const romujoco::SimulationContext& context, int steps, romujoco::GripperComponent& left,
    romujoco::GripperComponent& right, double object_finger_width) {
    const mjModel& model = *context.model;
    const int qpos1 = qpos_address(model, "left_fr3v2_1_finger_joint1");
    const int qpos2 = qpos_address(model, "left_fr3v2_1_finger_joint2");
    const int dof1 = dof_address(model, "left_fr3v2_1_finger_joint1");
    const int dof2 = dof_address(model, "left_fr3v2_1_finger_joint2");
    if (qpos1 < 0 || qpos2 < 0 || dof1 < 0 || dof2 < 0) return false;
    constexpr double kObjectStiffness = 4000.0;
    for (int step = 0; step < steps; ++step) {
        const double penetration1 = object_finger_width - context.data->qpos[qpos1];
        const double penetration2 = object_finger_width - context.data->qpos[qpos2];
        context.data->qfrc_applied[dof1] =
            penetration1 > 0.0 ? kObjectStiffness * penetration1 : 0.0;
        context.data->qfrc_applied[dof2] =
            penetration2 > 0.0 ? kObjectStiffness * penetration2 : 0.0;
        if (!left.advance(context) || !right.advance(context)) return false;
        mj_step(context.model, context.data);
        if (!left.update(context) || !right.update(context)) return false;
    }
    return true;
}

}  // namespace

int main() {
    const char* path = std::getenv("ROMUJOCO_MFR3DUO_MODEL");
    if (path == nullptr || !std::filesystem::exists(path)) {
        std::cout << "skipped: set ROMUJOCO_MFR3DUO_MODEL to mfr3duo.xml to run the MFR3Duo "
                     "gripper integration test\n";
        return 0;
    }

    char error[1024] = {};
    mjModel* model = mj_loadXML(path, nullptr, error, sizeof(error));
    if (!check(model != nullptr, error)) return 1;
    mjData* data = mj_makeData(model);
    if (!check(data != nullptr, "failed to allocate MuJoCo data")) {
        mj_deleteModel(model);
        return 1;
    }
    romujoco::SimulationContext context(model, data);
    mj_forward(model, data);

    romujoco::GripperComponent left(make_gripper_info(
        0, "left_gripper", {"left_fr3v2_1_finger_joint1", "left_fr3v2_1_finger_motor"},
        {"left_fr3v2_1_finger_joint2", ""}));
    romujoco::GripperComponent right(make_gripper_info(
        1, "right_gripper", {"right_fr3v2_1_finger_joint1", "right_fr3v2_1_finger_motor"},
        {"right_fr3v2_1_finger_joint2", ""}));

    bool success = check(left.init(context), "left gripper initialization failed") &&
                   check(right.init(context), "right gripper initialization failed") &&
                   check(left.reset(context), "left gripper reset failed") &&
                   check(right.reset(context), "right gripper reset failed");

    romujoco::GripperState left_state;
    romujoco::GripperState right_state;
    success = success && check(simulate(context, 1, left, right), "initial step failed") &&
              check(read_state(left, left_state), "left state was not published") &&
              check(read_state(right, right_state), "right state was not published") &&
              check(
                  std::abs(left_state.width) < 1.0e-9 && std::abs(right_state.width) < 1.0e-9,
                  "grippers did not start closed");

    // An unactuated finger joint does not hold position under gravity, so both
    // grippers are actively held closed before independence is checked.
    success = success &&
              check(send_command(left, context, 0.0, 0.5, 5.0), "left hold was rejected") &&
              check(send_command(right, context, 0.0, 0.5, 5.0), "right hold was rejected") &&
              check(simulate(context, 600, left, right), "holding the grippers failed") &&
              check(read_state(left, left_state), "left state was not published") &&
              check(read_state(right, right_state), "right state was not published") &&
              check(
                  std::abs(left_state.width) < 3.0e-3 && std::abs(right_state.width) < 3.0e-3,
                  "grippers did not hold the closed width");

    // Only the left gripper is commanded: the held right gripper must not react.
    success =
        success && check(send_command(left, context, 0.06, 0.5, 5.0), "left open was rejected") &&
        check(simulate(context, 600, left, right), "left opening failed") &&
        check(read_state(left, left_state), "left state was not published") &&
        check(read_state(right, right_state), "right state was not published") &&
        check(
            std::abs(left_state.width - 0.06) < 3.0e-3,
            "left gripper did not reach the commanded width") &&
        check(std::abs(right_state.width) < 3.0e-3, "right gripper followed the left gripper");

    // The unactuated finger follows through the MJCF equality coupling.
    const int left_qpos1 = qpos_address(*context.model, "left_fr3v2_1_finger_joint1");
    const int left_qpos2 = qpos_address(*context.model, "left_fr3v2_1_finger_joint2");
    const double first_finger = context.data->qpos[left_qpos1];
    const double second_finger = context.data->qpos[left_qpos2];
    // The MFR3Duo coupling uses polycoef "0 1 -1 0 0", that is q1 = q2 - q2^2.
    const double coupling_error = first_finger - (second_finger - second_finger * second_finger);
    success = success &&
              check(
                  first_finger > 0.02 && second_finger > 0.02,
                  "coupled finger did not follow the actuated finger") &&
              check(std::abs(coupling_error) < 1.0e-3, "finger coupling was not satisfied");

    // Independent control of the second gripper.
    success = success &&
              check(send_command(right, context, 0.03, 0.5, 5.0), "right open was rejected") &&
              check(simulate(context, 600, left, right), "right opening failed") &&
              check(read_state(left, left_state), "left state was not published") &&
              check(read_state(right, right_state), "right state was not published") &&
              check(
                  std::abs(right_state.width - 0.03) < 3.0e-3,
                  "right gripper did not reach the commanded width") &&
              check(
                  std::abs(left_state.width - 0.06) < 3.0e-3,
                  "left gripper moved while the right gripper was commanded");

    // Both grippers commanded in the same step.
    success = success &&
              check(send_command(left, context, 0.02, 0.5, 5.0), "left close was rejected") &&
              check(send_command(right, context, 0.02, 0.5, 5.0), "right close was rejected") &&
              check(simulate(context, 600, left, right), "closing both grippers failed") &&
              check(read_state(left, left_state), "left state was not published") &&
              check(read_state(right, right_state), "right state was not published") &&
              check(
                  std::abs(left_state.width - 0.02) < 3.0e-3 &&
                      std::abs(right_state.width - 0.02) < 3.0e-3,
                  "simultaneous gripper commands were not applied");

    // A zero command effort must not produce actuator output.
    success =
        success && check(send_command(left, context, 0.06, 0.5, 0.0), "zero effort was rejected") &&
        check(simulate(context, 300, left, right), "zero effort run failed") &&
        check(read_state(left, left_state), "left state was not published") &&
        check(left_state.effort < 1.0e-12, "zero command effort still produced actuator output");

    // The command effort bounds the actuator output.
    success = success &&
              check(send_command(left, context, 0.06, 0.5, 0.5), "limited open was rejected") &&
              check(simulate(context, 600, left, right), "limited open failed") &&
              check(read_state(left, left_state), "left state was not published") &&
              check(left_state.effort <= 0.5 + 1.0e-9, "effort limit was exceeded");

    // A zero command velocity freezes the width reference.
    success = success &&
              check(send_command(left, context, 0.0, 0.0, 5.0), "frozen close was rejected") &&
              check(simulate(context, 300, left, right), "frozen close failed") &&
              check(read_state(left, left_state), "left state was not published") &&
              check(
                  std::abs(left_state.width - 0.06) < 3.0e-3,
                  "zero command velocity still advanced the gripper");

    // A fixed object blocks the closing motion and produces a stall.
    success = success &&
              check(send_command(left, context, 0.07, 0.5, 5.0), "left open was rejected") &&
              check(simulate(context, 600, left, right), "left opening failed") &&
              check(send_command(left, context, 0.0, 0.2, 0.5), "blocked close was rejected");
    success = success &&
              check(
                  simulate_blocked(context, 600, left, right, kObjectFingerStop),
                  "blocked closing failed") &&
              check(read_state(left, left_state), "blocked state was not published") &&
              check(left_state.width > 0.01, "gripper closed through the object") &&
              check(left_state.stalled, "stall was not detected while blocked");

    // Reset restores the model state and clears the command side effects.
    data->qfrc_applied[dof_address(*context.model, "left_fr3v2_1_finger_joint1")] = 0.0;
    data->qfrc_applied[dof_address(*context.model, "left_fr3v2_1_finger_joint2")] = 0.0;
    romujoco::GripperCommand reset_command;
    success = success && check(left.reset(context, reset_command), "left reset failed") &&
              check(
                  reset_command.velocity == 0.0 && reset_command.effort == 0.0 &&
                      std::abs(reset_command.width - left_state.width) < 1.0e-9,
                  "left reset command did not capture the current state") &&
              check(simulate(context, 200, left, right), "run after reset failed") &&
              check(read_state(left, left_state), "reset state was not published") &&
              check(
                  !left_state.stalled && left_state.effort < 1.0e-9,
                  "reset did not clear the stall state");

    context.clear();
    return success ? 0 : 1;
}
