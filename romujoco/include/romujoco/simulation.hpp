#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "romujoco/component/camera.hpp"
#include "romujoco/component/imu.hpp"
#include "romujoco/component/joint.hpp"
#include "romujoco/component/lidar.hpp"
#include "romujoco/component/mobile_base.hpp"
#include "romujoco/config/simulation_config.hpp"
#include "romujoco/data/robot_command.hpp"
#include "romujoco/data/robot_state.hpp"
#include "romujoco/export.hpp"
#include "romujoco/simulation_status.hpp"

namespace romujoco {

class Simulation {
public:
    ROMUJOCO_PUBLIC Simulation();
    ROMUJOCO_PUBLIC ~Simulation();

    Simulation(const Simulation&) = delete;
    Simulation& operator=(const Simulation&) = delete;
    Simulation(Simulation&&) = delete;
    Simulation& operator=(Simulation&&) = delete;

    ROMUJOCO_PUBLIC bool initialize(const SimulationConfig& config);
    ROMUJOCO_PUBLIC bool initialize(const std::string& config_path);
    ROMUJOCO_PUBLIC bool shutdown();

    ROMUJOCO_PUBLIC bool start();
    ROMUJOCO_PUBLIC bool stop();
    ROMUJOCO_PUBLIC bool pause();
    ROMUJOCO_PUBLIC bool resume();
    ROMUJOCO_PUBLIC bool reset();
    ROMUJOCO_PUBLIC bool reset(std::string keyframe_name);

    ROMUJOCO_PUBLIC bool write_command(const JointCommand& command);
    ROMUJOCO_PUBLIC bool write_command(const MobileBaseCommand& command);
    ROMUJOCO_PUBLIC bool write_command(const RobotCommand& command);
    ROMUJOCO_PUBLIC bool write_commands(const JointCommands& commands);
    ROMUJOCO_PUBLIC bool write_commands(const MobileBaseCommands& commands);

    ROMUJOCO_PUBLIC bool read_state(std::shared_ptr<const RobotState>& state) const;
    ROMUJOCO_PUBLIC bool read_state(RobotState& state) const;
    ROMUJOCO_PUBLIC bool read_state(JointState& state) const;
    ROMUJOCO_PUBLIC bool read_state(ImuState& state) const;
    ROMUJOCO_PUBLIC bool read_state(CameraState& state) const;
    ROMUJOCO_PUBLIC bool read_state(LidarState& state) const;
    ROMUJOCO_PUBLIC bool read_state(MobileBaseState& state) const;
    ROMUJOCO_PUBLIC bool read_state(JointStates& states) const;
    ROMUJOCO_PUBLIC bool read_state(ImuStates& states) const;
    ROMUJOCO_PUBLIC bool read_state(CameraStates& states) const;
    ROMUJOCO_PUBLIC bool read_state(LidarStates& states) const;
    ROMUJOCO_PUBLIC bool read_state(MobileBaseStates& states) const;
    // Thread-safe copy of contacts captured in the most recently published
    // RobotState.  An initialized simulation with no contacts returns true and
    // an empty vector.
    ROMUJOCO_PUBLIC bool read_contacts(ContactStates& contacts) const;

    ROMUJOCO_PUBLIC bool step(std::size_t count = 1);
    ROMUJOCO_PUBLIC uint64_t step_count() const;
    ROMUJOCO_PUBLIC SimulationStatus status() const;
    ROMUJOCO_PUBLIC double time() const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace romujoco
