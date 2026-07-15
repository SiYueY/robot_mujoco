#pragma once

#include <mujoco/mujoco.h>

#include <string>

#include "mujoco_simulation/component/joint/joint_data.hpp"
#include "mujoco_simulation/component/simulation_component.hpp"

namespace mujoco_simulation {

class JointComponent : public SimulationComponent {
 public:
  explicit JointComponent(JointInfo info);

  std::string name() const noexcept override;
  bool bind(const mjModel& model) override;
  bool reset(const mjModel& model, mjData& data) override;
  bool update(const UpdateContext& context) override;

  bool write(const mjModel& model, mjData& data, const JointCommand& command);
  bool read(const mjData& data, JointState& state) const;

  const JointInfo& info() const noexcept { return info_; }
  int joint_id() const noexcept { return joint_id_; }
  int dof_address() const noexcept { return dof_address_; }
  int motor_id() const noexcept { return motor_id_; }
  JointType joint_type() const noexcept;

 private:
  bool validate_binding() const;
  bool calculate_effort(const mjData& data, const JointCommand& command, double* effort) const;
  bool write_effort_output(const mjModel& model, mjData& data, double effort) const;

  static double clamp_limits(const Limit& limits, double value);
  double clamp_motor_control_limits(const mjModel& model, double value) const;
  double clamp_motor_force_limits(const mjModel& model, double value) const;
  static bool finite(double value);
  int find_motor_id(const mjModel& model) const;
  static JointType parse_joint_type(int mujoco_joint_type);

  JointInfo info_;
  int joint_id_{-1};
  int qpos_address_{-1};
  int dof_address_{-1};
  int joint_type_{-1};
  int motor_id_{-1};
  JointCommand last_command_{};
  JointState state_{};
};

}  // namespace mujoco_simulation
