#pragma once

#include <limits>

#include "mujoco_simulation/component/mobile_base/mobile_base_binding.hpp"
#include "mujoco_simulation/component/mobile_base/mobile_base_command.hpp"
#include "mujoco_simulation/component/mobile_base/mobile_base_config.hpp"
#include "mujoco_simulation/component/mobile_base/mobile_base_state.hpp"
#include "mujoco_simulation/component/simulation_component.hpp"

namespace mujoco_simulation {

class MobileBaseComponent : public SimulationComponent {
 public:
  MobileBaseComponent(MobileBaseConfig config, MobileBaseBinding binding);

  std::string_view name() const noexcept override;
  Status bind(const mjModel& model) override;
  Status reset(const mjModel& model, mjData& data) override;

  Status write(const mjModel& model, mjData& data, const MobileBaseCommand& command);
  Status read(const mjData& data, MobileBaseState& state);

  const MobileBaseConfig& config() const noexcept { return config_; }

 private:
  Status validate(const mjModel& model) const;
  Status initialize_bindings(const mjModel& model);
  void clear_odometry();
  void update_state_fields(const mjData& data);
  void integrate_wheel_odometry(double simulation_time);
  Status update_ground_truth_pose(const mjData& data);
  static double normalized_yaw(double yaw);
  double command_linear_x(const MobileBaseCommand& command) const;
  double command_linear_y(const MobileBaseCommand& command) const;
  double command_angular_z(const MobileBaseCommand& command) const;
  Status write_differential(const mjModel& model, mjData& data, const MobileBaseCommand& command);
  Status write_omnidirectional(const mjModel& model, mjData& data,
                               const MobileBaseCommand& command);
  Status read_differential(const mjData& data, MobileBaseState& state);
  Status read_omnidirectional(const mjData& data, MobileBaseState& state);
  std::size_t wheel_count() const;
  static double clamp_actuator_control_limits(const mjModel& model, const JointBinding& binding,
                                              double value);
  static Status write_wheel_velocity_command(const mjModel& model, mjData& data,
                                             const MobileBaseWheelBinding& wheel, double velocity);
  static Status read_wheel_velocity(const mjData& data, const MobileBaseWheelBinding& wheel,
                                    double& velocity);

  MobileBaseBinding binding_{};
  double last_simulation_time_{std::numeric_limits<double>::quiet_NaN()};
  MobileBaseConfig config_;
  MobileBaseCommand command_;
  MobileBaseState state_;
};

}  // namespace mujoco_simulation
