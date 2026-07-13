#pragma once

#include <mujoco/mujoco.h>

#include <string>

#include "mujoco_simulation/component/joint/joint_data.hpp"
#include "mujoco_simulation/component/simulation_component.hpp"

namespace mujoco_simulation {

struct JointBinding {
  int joint_id{-1};
  int qpos_address{-1};
  int dof_address{-1};
  int joint_type{-1};
  int actuator_id{-1};
  int actuator_type{-1};
  bool has_actuator{false};
};

class JointComponent : public SimulationComponent {
 public:
  explicit JointComponent(JointConfig config);

  std::string name() const noexcept override;
  ResultCode bind(const mjModel& model) override;
  ResultCode reset(const mjModel& model, mjData& data) override;
  ResultCode update(const UpdateContext& context) override;

  ResultCode write(const mjModel& model, mjData& data, const JointCommand& command);
  ResultCode read(const mjData& data, JointState& state) const;

  const JointConfig& config() const noexcept { return config_; }
  const JointBinding& binding() const noexcept { return binding_; }
  JointType joint_type() const noexcept;
  ActuatorType actuator_type() const noexcept;

 private:
  ResultCode validate_binding() const;
  ResultCode validate_command_configuration() const;
  ResultCode write_direct_command(const mjModel& model, mjData& data, const JointCommand& command);
  ResultCode write_software_pd_command(const mjModel& model, mjData& data,
                                       const JointCommand& command);
  ResultCode write_effort_output(const mjModel& model, mjData& data, double effort) const;

  double clamp_command_limits(double value) const;
  double clamp_actuator_control_limits(const mjModel& model, double value) const;
  double clamp_actuator_force_limits(const mjModel& model, double value) const;
  static bool finite(double value);
  int find_actuator_id(const mjModel& model) const;
  static JointType parse_joint_type(int mujoco_joint_type);
  static ActuatorType parse_actuator_type(const mjModel& model, int actuator_id);

  JointConfig config_;
  JointBinding binding_{};
  JointCommand last_command_{};
  JointState state_{};
};

}  // namespace mujoco_simulation
