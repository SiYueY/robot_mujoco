#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <string>

#include <mujoco/mujoco.h>

#include "component/gripper/gripper_component.hpp"
#include "test_support.hpp"

namespace {

bool check(bool value, const char* message) {
    if (!value) std::cerr << message << '\n';
    return value;
}

struct LoadedModel {
    mjModel* model{nullptr};
    mjData* data{nullptr};
};

// Franka-like model: one motor on finger1, finger2 follows through an MJCF
// equality coupling.  The right finger body is rotated by 180 degrees so a
// positive joint coordinate opens the gripper for both fingers.  A fixed box
// between the fingers blocks a closing motion.
std::string coupled_model() {
    std::string xml = R"(<mujoco model="gripper_coupled">
  <option timestep="0.001" integrator="implicitfast"/>
  <worldbody>
    <body name="hand" pos="0 0 0">
      <inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/>
      <body name="left_finger" pos="0 0.03 0">
        <joint name="finger1" type="slide" axis="0 1 0" limited="true" range="0 0.04"
               damping="0.3"/>
        <geom name="left_pad" type="box" size="0.01 0.005 0.01" mass="0.05"/>
      </body>
      <body name="right_finger" pos="0 -0.03 0" quat="0 0 0 1">
        <joint name="finger2" type="slide" axis="0 1 0" limited="true" range="0 0.04"
               damping="0.3"/>
        <geom name="right_pad" type="box" size="0.01 0.005 0.01" mass="0.05"/>
      </body>
      <body name="auxiliary" pos="0.05 0 0">
        <joint name="hinge_joint" type="hinge" axis="0 0 1"/>
        <geom type="box" size="0.005 0.005 0.005" mass="0.01"/>
      </body>
    </body>)";
    xml += R"(
    <geom name="obstacle" type="box" pos="0 0 0" size="0.02 0.03 0.02"/>)";
    xml += R"(
  </worldbody>
  <!-- 1:1 finger coupling: MuJoCo equality joints couple the displacement from
       qpos0, so "0 1 0 0 0" reproduces the parallel linkage exactly. -->
  <equality>
    <joint joint1="finger1" joint2="finger2" polycoef="0 1 0 0 0"/>
  </equality>
  <actuator>
    <motor name="finger1_motor" joint="finger1" gear="1" ctrlrange="-20 20"/>
    <motor name="finger2_motor" joint="finger2" gear="1" ctrlrange="-20 20"/>
    <motor name="geared_motor" joint="finger1" gear="2" ctrlrange="-20 20"/>
  </actuator>
</mujoco>)";
    return xml;
}

// Robotiq-like model: two independently actuated fingers without coupling.
std::string independent_model() {
    return R"(<mujoco model="gripper_independent">
  <option timestep="0.001" integrator="implicitfast"/>
  <worldbody>
    <body name="hand" pos="0 0 0">
      <inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/>
      <body name="left_finger" pos="0 0.03 0">
        <joint name="finger1" type="slide" axis="0 1 0" limited="true" range="0 0.04"
               damping="0.3"/>
        <geom name="left_pad" type="box" size="0.01 0.005 0.01" mass="0.05"/>
      </body>
      <body name="right_finger" pos="0 -0.03 0" quat="0 0 0 1">
        <joint name="finger2" type="slide" axis="0 1 0" limited="true" range="0 0.04"
               damping="0.3"/>
        <geom name="right_pad" type="box" size="0.01 0.005 0.01" mass="0.05"/>
      </body>
    </body>
    </worldbody>
  <actuator>
    <motor name="finger1_motor" joint="finger1" gear="1" ctrlrange="-20 20"/>
    <motor name="finger2_motor" joint="finger2" gear="1" ctrlrange="-20 20"/>
  </actuator>
</mujoco>)";
}

bool load_model(const char* filename, const std::string& xml, LoadedModel& out) {
    romujoco_test::TemporaryFile file(filename);
    if (!file.write(xml)) {
        std::cerr << "failed to write " << filename << '\n';
        return false;
    }
    char error[1024] = {};
    out.model = mj_loadXML(file.path().c_str(), nullptr, error, sizeof(error));
    if (out.model == nullptr) {
        std::cerr << error << '\n';
        return false;
    }
    out.data = mj_makeData(out.model);
    if (out.data == nullptr) {
        mj_deleteModel(out.model);
        out.model = nullptr;
        return false;
    }
    return true;
}

int qpos_address(const mjModel& model, const char* joint_name) {
    const int joint = mj_name2id(&model, mjOBJ_JOINT, joint_name);
    return joint < 0 ? -1 : model.jnt_qposadr[joint];
}

int actuator_id(const mjModel& model, const char* actuator_name) {
    return mj_name2id(&model, mjOBJ_ACTUATOR, actuator_name);
}

int dof_address(const mjModel& model, const char* joint_name) {
    const int joint = mj_name2id(&model, mjOBJ_JOINT, joint_name);
    return joint < 0 ? -1 : model.jnt_dofadr[joint];
}

// Puts both fingers at the given half opening, at rest.  The finger joints have
// no gravity load along their axis, so this is a genuine equilibrium state.
void set_rest_state(const romujoco::SimulationContext& context, double half_opening) {
    for (const char* name : {"finger1", "finger2"}) {
        context.data->qpos[qpos_address(*context.model, name)] = half_opening;
        context.data->qvel[dof_address(*context.model, name)] = 0.0;
    }
    mj_forward(context.model, context.data);
}

romujoco::GripperInfo make_gripper_info(bool dual_actuator = false) {
    romujoco::GripperInfo info;
    info.id = 3;
    info.name = "gripper";
    info.period = 0.001;
    info.fingers[0] = {"finger1", "finger1_motor"};
    info.fingers[1] = {"finger2", dual_actuator ? "finger2_motor" : ""};
    info.control.stiffness = 200.0;
    info.control.damping = 8.0;
    info.width_limits = {0.0, 0.08};
    info.velocity_limits = {0.0, 0.5};
    info.effort_limits = {0.0, 2.0};
    info.stall.width_tolerance = 0.001;
    info.stall.velocity_threshold = 0.01;
    info.stall.effort_ratio = 0.9;
    info.stall.timeout = 0.05;
    return info;
}

bool simulate(
    romujoco::GripperComponent& gripper, const romujoco::SimulationContext& context, int steps) {
    for (int step = 0; step < steps; ++step) {
        if (!gripper.advance(context)) return false;
        mj_step(context.model, context.data);
        if (!gripper.update(context)) return false;
    }
    return true;
}

bool read_state(const romujoco::GripperComponent& gripper, romujoco::GripperState& state) {
    std::shared_ptr<const romujoco::GripperState> snapshot;
    if (!gripper.read_state(snapshot) || snapshot == nullptr) return false;
    state = *snapshot;
    return true;
}

double measured_width(const romujoco::SimulationContext& context) {
    return context.data->qpos[qpos_address(*context.model, "finger1")] +
           context.data->qpos[qpos_address(*context.model, "finger2")];
}

romujoco::GripperCommand make_command(
    const romujoco::GripperComponent& gripper, double target_width, double velocity,
    double effort) {
    romujoco::GripperCommand command;
    command.id = gripper.info().id;
    command.width = target_width;
    command.velocity = velocity;
    command.effort = effort;
    return command;
}

bool test_initialization(const romujoco::SimulationContext& context) {
    bool success = true;

    romujoco::GripperComponent valid(make_gripper_info());
    success = success && check(valid.init(context), "valid single-actuator gripper was rejected");

    romujoco::GripperComponent dual(make_gripper_info(true));
    success = success && check(dual.init(context), "valid dual-actuator gripper was rejected");

    romujoco::GripperInfo unknown_joint = make_gripper_info();
    unknown_joint.fingers[1].joint_name = "missing_joint";
    romujoco::GripperComponent missing(unknown_joint);
    success = success && check(!missing.init(context), "unknown finger joint was accepted");

    romujoco::GripperInfo hinge = make_gripper_info();
    hinge.fingers[1].joint_name = "hinge_joint";
    romujoco::GripperComponent wrong_type(hinge);
    success = success && check(!wrong_type.init(context), "non-slide finger joint was accepted");

    romujoco::GripperInfo unknown_actuator = make_gripper_info();
    unknown_actuator.fingers[0].actuator_name = "missing_motor";
    romujoco::GripperComponent missing_motor(unknown_actuator);
    success =
        success && check(!missing_motor.init(context), "unknown finger actuator was accepted");

    romujoco::GripperInfo mismatched = make_gripper_info();
    mismatched.fingers[0].actuator_name = "finger2_motor";
    romujoco::GripperComponent wrong_binding(mismatched);
    success = success &&
              check(!wrong_binding.init(context), "actuator bound to another joint was accepted");

    romujoco::GripperInfo geared = make_gripper_info();
    geared.fingers[0].actuator_name = "geared_motor";
    romujoco::GripperComponent wrong_gear(geared);
    success = success && check(!wrong_gear.init(context), "actuator with gear != 1 was accepted");

    romujoco::GripperInfo same_joint = make_gripper_info();
    same_joint.fingers[1].joint_name = same_joint.fingers[0].joint_name;
    romujoco::GripperComponent duplicate(same_joint);
    success = success && check(!duplicate.init(context), "duplicate finger joint was accepted");

    romujoco::GripperInfo unactuated = make_gripper_info();
    unactuated.fingers[0].actuator_name.clear();
    romujoco::GripperComponent passive(unactuated);
    success = success && check(!passive.init(context), "gripper without any actuator was accepted");

    romujoco::GripperInfo inverted = make_gripper_info();
    inverted.width_limits = {0.08, 0.0};
    romujoco::GripperComponent invalid_limits(inverted);
    success = success && check(!invalid_limits.init(context), "inverted width limit was accepted");

    romujoco::GripperInfo negative_gain = make_gripper_info();
    negative_gain.control.stiffness = -1.0;
    romujoco::GripperComponent invalid_control(negative_gain);
    success = success && check(!invalid_control.init(context), "negative stiffness was accepted");

    romujoco::GripperInfo invalid_stall = make_gripper_info();
    invalid_stall.stall.effort_ratio = 1.5;
    romujoco::GripperComponent stall(invalid_stall);
    success = success && check(!stall.init(context), "invalid stall ratio was accepted");
    return success;
}

bool test_commands(const romujoco::SimulationContext& context) {
    romujoco::GripperComponent gripper(make_gripper_info());
    if (!check(gripper.init(context), "gripper initialization failed") ||
        !check(gripper.reset(context), "gripper reset failed"))
        return false;
    bool success = true;

    const romujoco::GripperCommand valid = make_command(gripper, 0.04, 0.1, 1.0);
    romujoco::GripperCommand negative_velocity = valid;
    negative_velocity.velocity = -0.1;
    success =
        success &&
        check(!gripper.write(context, negative_velocity), "negative command velocity was accepted");
    romujoco::GripperCommand negative_effort = valid;
    negative_effort.effort = -1.0;
    success =
        success &&
        check(!gripper.write(context, negative_effort), "negative command effort was accepted");
    romujoco::GripperCommand infinite = valid;
    infinite.width = std::numeric_limits<double>::infinity();
    success =
        success && check(!gripper.write(context, infinite), "non-finite command was accepted");
    romujoco::GripperCommand nan = valid;
    nan.width = std::numeric_limits<double>::quiet_NaN();
    success = success && check(!gripper.write(context, nan), "NaN command was accepted");
    romujoco::GripperCommand wrong_id = valid;
    wrong_id.id = gripper.info().id + 1U;
    success =
        success && check(!gripper.write(context, wrong_id), "command for another id was accepted");

    // Width limit: the configured maximum is smaller than the mechanical range.
    romujoco::GripperInfo limited = make_gripper_info();
    limited.width_limits = {0.0, 0.06};
    romujoco::GripperComponent limited_gripper(limited);
    success = success && check(limited_gripper.init(context), "limited gripper init failed") &&
              check(limited_gripper.reset(context), "limited gripper reset failed") &&
              check(
                  limited_gripper.write(context, make_command(limited_gripper, 0.5, 0.5, 2.0)),
                  "oversized width command was rejected") &&
              check(simulate(limited_gripper, context, 500), "limited gripper run failed") &&
              check(
                  std::abs(measured_width(context) - 0.06) < 1.0e-3,
                  "command width was not clamped to the configured limit");

    success =
        success &&
        check(
            gripper.write(context, make_command(gripper, 0.07, 0.5, 2.0)),
            "open command was rejected") &&
        check(simulate(gripper, context, 500), "gripper failed while opening") &&
        check(
            std::abs(measured_width(context) - 0.07) < 1.0e-3, "gripper did not reach the target");

    // command.velocity is a real motion constraint: once the ramp settles, the
    // realised width rate must equal the commanded magnitude.
    success = success &&
              check(
                  gripper.write(context, make_command(gripper, 0.0, 0.02, 2.0)),
                  "slow close was rejected") &&
              check(simulate(gripper, context, 200), "gripper failed while closing") &&
              check(measured_width(context) > 0.06, "gripper jumped while closing");
    const double before_ramp = measured_width(context);
    success = success && check(simulate(gripper, context, 100), "gripper failed while closing");
    const double ramp_delta = before_ramp - measured_width(context);
    success = success && check(
                             std::abs(ramp_delta - 0.002) < 0.0005,
                             "command velocity did not limit the reference width rate");

    // velocity == 0 freezes the reference: the fingers settle and stop.
    success =
        success &&
        check(gripper.write(context, make_command(gripper, 0.0, 0.0, 2.0)), "stop was rejected") &&
        check(simulate(gripper, context, 1500), "gripper failed while stopping");
    const double stopped_width = measured_width(context);
    success = success && check(simulate(gripper, context, 200), "gripper failed while stopped") &&
              check(
                  std::abs(measured_width(context) - stopped_width) < 1.0e-5,
                  "zero command velocity still advanced the reference");

    // effort == 0 must not command any actuator force.
    success = success &&
              check(
                  gripper.write(context, make_command(gripper, 0.0, 0.5, 0.0)),
                  "zero effort was rejected") &&
              check(simulate(gripper, context, 50), "gripper failed with zero effort");
    for (const char* name : {"finger1_motor", "finger2_motor"}) {
        const int id = actuator_id(*context.model, name);
        success = success && check(
                                 id >= 0 && context.data->ctrl[id] == 0.0,
                                 "zero command effort still produced actuator force");
    }

    // The configured effort limit bounds the actuator output.
    success = success &&
              check(
                  gripper.write(context, make_command(gripper, 0.07, 0.5, 10.0)),
                  "oversized effort was rejected") &&
              check(simulate(gripper, context, 200), "gripper failed while pressing");
    const int finger1 = actuator_id(*context.model, "finger1_motor");
    success = success && check(
                             finger1 >= 0 && std::abs(context.data->ctrl[finger1]) <= 2.0 + 1.0e-12,
                             "effort limit did not clamp the actuator output");
    return success;
}

bool test_state_and_stall(const romujoco::SimulationContext& context) {
    romujoco::GripperComponent gripper(make_gripper_info());
    if (!check(gripper.init(context), "gripper initialization failed") ||
        !check(gripper.reset(context), "gripper reset failed"))
        return false;
    bool success = true;
    set_rest_state(context, 0.03);

    romujoco::GripperState state;
    success = success && check(simulate(gripper, context, 1), "gripper update failed") &&
              check(read_state(gripper, state), "gripper state was not published") &&
              check(
                  std::abs(state.width - measured_width(context)) < 1.0e-12,
                  "state width does not match the finger positions") &&
              check(
                  std::abs(state.timestamp - context.data->time) < 1.0e-12,
                  "state timestamp does not match the simulation time");

    // Reset keeps the current opening, clears the actuator output and the stall
    // state, and never commands motion.
    romujoco::GripperCommand reset_command;
    success = success && check(gripper.reset(context, reset_command), "gripper reset failed") &&
              check(
                  std::abs(reset_command.width - measured_width(context)) < 1.0e-12,
                  "reset command did not capture the current width") &&
              check(
                  reset_command.velocity == 0.0 && reset_command.effort == 0.0,
                  "reset command was not passive");
    for (const char* name : {"finger1_motor", "finger2_motor"}) {
        const int id = actuator_id(*context.model, name);
        success = success && check(
                                 id >= 0 && context.data->ctrl[id] == 0.0,
                                 "reset did not clear the actuator output");
    }
    const double reset_width = measured_width(context);
    success =
        success && check(simulate(gripper, context, 200), "gripper failed after reset") &&
        check(
            std::abs(measured_width(context) - reset_width) < 1.0e-9,
            "reset commanded unexpected motion") &&
        check(read_state(gripper, state), "reset state was not published") &&
        check(!state.stalled && state.effort < 1.0e-12, "reset did not clear the stall state");

    // Pressing on the obstacle keeps the target unreachable while the actuator
    // saturates at the commanded effort and the fingers stop moving.
    romujoco::GripperCommand blocked_close = make_command(gripper, 0.0, 0.2, 0.5);
    success = success &&
              check(
                  gripper.write(context, make_command(gripper, 0.07, 0.5, 2.0)),
                  "open command was rejected") &&
              check(simulate(gripper, context, 500), "gripper failed while opening") &&
              check(gripper.write(context, blocked_close), "blocked close was rejected") &&
              check(simulate(gripper, context, 500), "gripper failed while blocked") &&
              check(read_state(gripper, state), "blocked state was not published") &&
              check(state.width > 0.005, "gripper closed through the obstacle");
    const int finger1 = actuator_id(*context.model, "finger1_motor");
    success = success &&
              check(
                  finger1 >= 0 && std::abs(std::abs(context.data->ctrl[finger1]) - 0.5) < 1.0e-9,
                  "command effort did not limit the actuator output") &&
              check(state.stalled, "stall was not detected while blocked") &&
              check(state.effort >= 0.5 * 0.9, "stall effort did not reach the commanded effort");

    // Releasing the target clears the stall condition.
    success =
        success &&
        check(
            gripper.write(context, make_command(gripper, 0.07, 0.5, 2.0)), "reopen was rejected") &&
        check(simulate(gripper, context, 500), "gripper failed while reopening") &&
        check(read_state(gripper, state), "released state was not published") &&
        check(!state.stalled, "stall was not cleared after the gripper reopened");

    // A zero effort limit disables stall detection and actuator output.
    romujoco::GripperCommand zero_effort = blocked_close;
    zero_effort.effort = 0.0;
    success = success &&
              check(gripper.write(context, zero_effort), "zero effort close was rejected") &&
              check(simulate(gripper, context, 500), "gripper failed with zero effort") &&
              check(read_state(gripper, state), "zero effort state was not published") &&
              check(!state.stalled, "stall was reported with a zero effort limit");
    return success;
}

bool test_dual_actuator(const romujoco::SimulationContext& context) {
    romujoco::GripperComponent gripper(make_gripper_info(true));
    if (!check(gripper.init(context), "dual-actuator gripper initialization failed") ||
        !check(gripper.reset(context), "dual-actuator gripper reset failed"))
        return false;
    bool success = true;

    success = success &&
              check(
                  gripper.write(context, make_command(gripper, 0.06, 0.5, 2.0)),
                  "dual open was rejected") &&
              check(simulate(gripper, context, 500), "dual gripper failed while opening");
    const int finger1 = qpos_address(*context.model, "finger1");
    const int finger2 = qpos_address(*context.model, "finger2");
    success = success && check(
                             std::abs(context.data->qpos[finger1] - 0.03) < 1.0e-3 &&
                                 std::abs(context.data->qpos[finger2] - 0.03) < 1.0e-3,
                             "dual-actuator fingers did not share the symmetric reference");

    success = success &&
              check(
                  gripper.write(context, make_command(gripper, 0.02, 0.5, 1.0)),
                  "dual close was rejected") &&
              check(simulate(gripper, context, 500), "dual gripper failed while closing");
    romujoco::GripperState state;
    success = success &&
              check(
                  std::abs(measured_width(context) - 0.02) < 1.0e-3,
                  "dual-actuator gripper did not reach the target width") &&
              check(read_state(gripper, state), "dual state was not published") &&
              check(state.effort <= 1.0 + 1.0e-9, "dual-actuator effort limit was not applied");
    return success;
}

}  // namespace

int main() {
    LoadedModel coupled;
    if (!check(load_model("romujoco_gripper_coupled.xml", coupled_model(), coupled), "")) return 1;
    romujoco::SimulationContext context(coupled.model, coupled.data);
    // Start with an open gripper so the obstacle can block a closing motion.
    for (const char* name : {"finger1", "finger2"})
        context.data->qpos[qpos_address(*context.model, name)] = 0.03;
    mj_forward(context.model, context.data);

    LoadedModel independent;
    if (!check(
            load_model("romujoco_gripper_independent.xml", independent_model(), independent), ""))
        return 1;
    romujoco::SimulationContext independent_context(independent.model, independent.data);
    for (const char* name : {"finger1", "finger2"})
        independent_context.data->qpos[qpos_address(*independent.model, name)] = 0.03;
    mj_forward(independent.model, independent.data);

    bool success = test_initialization(context);
    success = success && test_commands(context);
    success = success && test_state_and_stall(context);
    success = success && test_dual_actuator(independent_context);
    context.clear();
    independent_context.clear();
    return success ? 0 : 1;
}
