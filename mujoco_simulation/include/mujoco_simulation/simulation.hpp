#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

#include "mujoco_simulation/component/camera.hpp"
#include "mujoco_simulation/component/imu.hpp"
#include "mujoco_simulation/component/joint.hpp"
#include "mujoco_simulation/component/lidar.hpp"
#include "mujoco_simulation/component/mobile_base.hpp"
#include "mujoco_simulation/config/simulation_config.hpp"
#include "mujoco_simulation/data/robot_command.hpp"
#include "mujoco_simulation/data/robot_state.hpp"
#include "mujoco_simulation/export.hpp"
#include "mujoco_simulation/simulation_status.hpp"

namespace mujoco_simulation {

class Simulation {
public:
  MUJOCO_SIMULATION_PUBLIC Simulation();
  MUJOCO_SIMULATION_PUBLIC ~Simulation();

  Simulation(const Simulation &) = delete;
  Simulation &operator=(const Simulation &) = delete;
  Simulation(Simulation &&) = delete;
  Simulation &operator=(Simulation &&) = delete;

  MUJOCO_SIMULATION_PUBLIC bool initialize(const SimulationConfig &config);
  MUJOCO_SIMULATION_PUBLIC bool initialize(const std::string &config_path);
  MUJOCO_SIMULATION_PUBLIC bool shutdown();

  MUJOCO_SIMULATION_PUBLIC bool start();
  MUJOCO_SIMULATION_PUBLIC bool stop();
  MUJOCO_SIMULATION_PUBLIC bool pause();
  MUJOCO_SIMULATION_PUBLIC bool resume();
  MUJOCO_SIMULATION_PUBLIC bool reset();
  MUJOCO_SIMULATION_PUBLIC bool reset(std::string keyframe_name);

  MUJOCO_SIMULATION_PUBLIC bool write_command(JointId id,
                                              const JointCommand &command);
  MUJOCO_SIMULATION_PUBLIC bool write_command(MobileBaseId id,
                                              const MobileBaseCommand &command);
  MUJOCO_SIMULATION_PUBLIC bool write_command(const RobotCommand &command);
  MUJOCO_SIMULATION_PUBLIC bool
  write_commands(const JointCommandBatch &commands);
  MUJOCO_SIMULATION_PUBLIC bool
  write_commands(const MobileBaseCommandBatch &commands);

  MUJOCO_SIMULATION_PUBLIC bool
  read_state(std::shared_ptr<const RobotState> &state) const;
  MUJOCO_SIMULATION_PUBLIC bool read_state(RobotState &state) const;
  MUJOCO_SIMULATION_PUBLIC bool read_state(JointId id, JointState &state) const;
  MUJOCO_SIMULATION_PUBLIC bool read_state(ImuId id, ImuState &state) const;
  MUJOCO_SIMULATION_PUBLIC bool read_state(CameraId id,
                                           CameraState &state) const;
  MUJOCO_SIMULATION_PUBLIC bool read_state(LidarId id, LidarState &state) const;
  MUJOCO_SIMULATION_PUBLIC bool read_state(MobileBaseId id,
                                           MobileBaseState &state) const;
  MUJOCO_SIMULATION_PUBLIC bool read_state(JointStates &states) const;
  MUJOCO_SIMULATION_PUBLIC bool read_state(ImuStates &states) const;
  MUJOCO_SIMULATION_PUBLIC bool read_state(CameraStates &states) const;
  MUJOCO_SIMULATION_PUBLIC bool read_state(LidarStates &states) const;
  MUJOCO_SIMULATION_PUBLIC bool read_state(MobileBaseStates &states) const;

  MUJOCO_SIMULATION_PUBLIC bool step(std::size_t count = 1);
  MUJOCO_SIMULATION_PUBLIC uint64_t step_count() const;
  MUJOCO_SIMULATION_PUBLIC SimulationStatus status() const;
  MUJOCO_SIMULATION_PUBLIC double time() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace mujoco_simulation
