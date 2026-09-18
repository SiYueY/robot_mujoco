#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>

#include "romujoco/simulation.hpp"
#include "test_support.hpp"

namespace {

bool check(bool value, const char* message) {
    if (!value) std::cerr << message << '\n';
    return value;
}

constexpr romujoco::GripperId kGripperId = 7;

const char* kModel = R"(<mujoco model="gripper_runtime">
  <option timestep="0.001" integrator="implicitfast"/>
  <worldbody>
    <body name="hand" pos="0 0 0">
      <inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/>
      <body name="left_finger" pos="0 0.03 0">
        <joint name="finger1" type="slide" axis="0 1 0" limited="true" range="0 0.04"
               ref="0.03" damping="0.3"/>
        <geom name="left_pad" type="box" size="0.01 0.005 0.01" mass="0.05"/>
      </body>
      <body name="right_finger" pos="0 -0.03 0" quat="0 0 0 1">
        <joint name="finger2" type="slide" axis="0 1 0" limited="true" range="0 0.04"
               ref="0.03" damping="0.3"/>
        <geom name="right_pad" type="box" size="0.01 0.005 0.01" mass="0.05"/>
      </body>
    </body>
  </worldbody>
  <equality>
    <joint joint1="finger1" joint2="finger2" polycoef="0 1 0 0 0"/>
  </equality>
  <actuator>
    <motor name="finger_motor" joint="finger1" gear="1" ctrlrange="-20 20"/>
  </actuator>
</mujoco>)";

const char* kConfig = R"(<robot_mujoco>
  <mujoco><mjcf>gripper_runtime_model.xml</mjcf></mujoco>
  <simulation>
    <physics period="0.001"/>
    <viewer period="0.02" enabled="false"/>
  </simulation>
  <robot>
    <gripper id="7" name="hand" period="0.001">
      <finger joint="finger1" actuator="finger_motor"/>
      <finger joint="finger2"/>
      <control stiffness="200" damping="8"/>
      <limit>
        <width min="0" max="0.08"/>
        <velocity min="0" max="0.5"/>
        <effort min="0" max="10"/>
      </limit>
      <stall width_tolerance="0.001" velocity_threshold="0.01"
             effort_ratio="0.9" timeout="0.05"/>
    </gripper>
  </robot>
</robot_mujoco>)";

bool read_gripper(romujoco::Simulation& simulation, romujoco::GripperState& state) {
    state.id = kGripperId;
    return simulation.read_state(state);
}

bool wait_for_width(romujoco::Simulation& simulation, double target, double tolerance) {
    romujoco::GripperState state;
    return romujoco_test::wait_until(
        [&] {
            return read_gripper(simulation, state) && std::abs(state.width - target) < tolerance;
        },
        std::chrono::seconds(5));
}

bool wait_for_steps(romujoco::Simulation& simulation, std::uint64_t minimum) {
    return romujoco_test::wait_until(
        [&] { return simulation.step_count() >= minimum; }, std::chrono::seconds(5));
}

}  // namespace

int main() {
    romujoco_test::TemporaryFile model("gripper_runtime_model.xml");
    romujoco_test::TemporaryFile config("gripper_runtime_config.xml");
    if (!check(model.write(kModel), "failed to write the model") ||
        !check(config.write(kConfig), "failed to write the configuration"))
        return 1;

    romujoco::Simulation simulation;
    bool success = check(simulation.initialize(config.path().string()), "initialization failed") &&
                   check(
                       simulation.status() == romujoco::SimulationStatus::Stopped,
                       "initialized simulation was not stopped");

    romujoco::GripperState state;
    success = success &&
              check(read_gripper(simulation, state), "initial state was not published") &&
              check(std::abs(state.width - 0.06) < 1.0e-9, "initial width was not published");

    romujoco::GripperCommand unknown;
    unknown.id = kGripperId + 1U;
    success =
        success &&
        check(!simulation.write_command(unknown), "command for an unknown gripper was accepted");

    romujoco::GripperCommand open;
    open.id = kGripperId;
    open.width = 0.07;
    open.velocity = 0.5;
    open.effort = 2.0;
    success = success && check(simulation.write_command(open), "gripper command was rejected") &&
              check(simulation.start(), "failed to start the simulation") &&
              check(wait_for_width(simulation, 0.07, 2.0e-3), "gripper did not reach the target");

    success = success && check(read_gripper(simulation, state), "state was not published") &&
              check(state.timestamp > 0.0, "state timestamp was not updated") &&
              check(state.effort <= 10.0, "state effort exceeded the configured limit") &&
              check(!state.stalled, "stall was reported for a free gripper");

    romujoco::GripperStates states;
    success = success &&
              check(simulation.read_state(states), "gripper states were not published") &&
              check(
                  states != nullptr && states->size() == 1U && (*states)[0] != nullptr &&
                      (*states)[0]->id == kGripperId,
                  "gripper state list is inconsistent");
    std::shared_ptr<const romujoco::RobotState> robot_state;
    success = success &&
              check(simulation.read_state(robot_state), "robot state was not published") &&
              check(
                  robot_state != nullptr && robot_state->grippers != nullptr &&
                      robot_state->grippers->size() == 1U,
                  "robot state does not contain the gripper");

    romujoco::GripperCommand close = open;
    close.width = 0.02;
    success = success && check(simulation.write_command(close), "closing command was rejected") &&
              check(wait_for_width(simulation, 0.02, 2.0e-3), "gripper did not close") &&
              check(simulation.stop(), "failed to stop the simulation");

    success = success &&
              check(read_gripper(simulation, state), "stopped state was not published") &&
              check(std::abs(state.width - 0.02) < 2.0e-3, "stopping changed the gripper width");

    // Reset restores the model reference state and clears the command side
    // effects, so the gripper holds its opening instead of moving again.
    success = success && check(simulation.reset(), "reset failed");
    romujoco::GripperState reset_state;
    success = success &&
              check(read_gripper(simulation, reset_state), "reset state was not published") &&
              check(
                  std::abs(reset_state.width - 0.06) < 1.0e-9,
                  "reset did not restore the gripper reference state") &&
              check(
                  reset_state.effort < 1.0e-9 && !reset_state.stalled,
                  "reset did not clear the gripper effort");

    success = success && check(simulation.start(), "restart after reset failed") &&
              check(wait_for_steps(simulation, 100), "simulation did not run after reset");
    romujoco::GripperState idle_state;
    success = success &&
              check(read_gripper(simulation, idle_state), "idle state was not published") &&
              check(
                  std::abs(idle_state.width - reset_state.width) < 1.0e-6,
                  "reset commanded unexpected motion") &&
              check(simulation.stop(), "failed to stop after reset");

    success = success && check(simulation.shutdown(), "shutdown failed");
    return success ? 0 : 1;
}
