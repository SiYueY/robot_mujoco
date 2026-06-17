#pragma once

#include <mujoco/mujoco.h>

#include <string_view>

#include "mujoco_simulation/component/joint/joint_binding.hpp"
#include "mujoco_simulation/component/joint/joint_command.hpp"
#include "mujoco_simulation/component/joint/joint_config.hpp"
#include "mujoco_simulation/component/joint/joint_state.hpp"
#include "mujoco_simulation/component/simulation_component.hpp"

namespace mujoco_simulation {

class JointComponent : public SimulationComponent {
 public:
  explicit JointComponent(JointConfig config);

  std::string_view name() const noexcept override;
  Status bind(const mjModel& model) override;
  Status reset(const mjModel& model, mjData& data) override;

  Status write(const mjModel& model, mjData& data, const JointCommand& command);
  Status read(const mjData& data, JointState& state) const;

  const JointConfig& config() const noexcept { return config_; }
  const JointBinding& binding() const noexcept { return binding_; }
  JointType joint_type() const noexcept;
  ActuatorType actuator_type() const noexcept;

 private:
  Status validate_binding() const;
  Status validate_command_configuration() const;
  Status write_direct_command(const mjModel& model, mjData& data, const JointCommand& command);
  Status write_software_pd_command(const mjModel& model, mjData& data, const JointCommand& command);
  Status write_effort_output(const mjModel& model, mjData& data, double effort) const;

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
};

}  // namespace mujoco_simulation
