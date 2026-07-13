#pragma once

#include <limits>
#include <optional>
#include <string>

#include "mujoco_simulation/component/joint/joint_component.hpp"
#include "mujoco_simulation/component/mobile_base/mobile_base_data.hpp"
#include "mujoco_simulation/component/simulation_component.hpp"

namespace mujoco_simulation {

struct MobileBaseWheelBinding {
  std::string joint_name;
  JointBinding joint{};
};

struct DifferentialBinding {
  MobileBaseWheelBinding left_wheel;
  MobileBaseWheelBinding right_wheel;
};

struct OmnidirectionalBinding {
  MobileBaseWheelBinding front_left;
  MobileBaseWheelBinding front_right;
  MobileBaseWheelBinding rear_left;
  MobileBaseWheelBinding rear_right;
};

struct MobileBaseBinding {
  std::optional<DifferentialBinding> differential;
  std::optional<OmnidirectionalBinding> omnidirectional;
  int base_body_id{-1};
};

class MobileBaseComponent : public SimulationComponent {
 public:
  MobileBaseComponent(MobileBaseConfig config, MobileBaseBinding binding);

  std::string name() const noexcept override;
  ResultCode bind(const mjModel& model) override;
  ResultCode reset(const mjModel& model, mjData& data) override;
  ResultCode update(const UpdateContext& context) override;

  ResultCode write(const mjModel& model, mjData& data, const MobileBaseCommand& command);
  ResultCode read(const mjData& data, MobileBaseState& state);

  const MobileBaseConfig& config() const noexcept { return config_; }

 private:
  ResultCode validate(const mjModel& model) const;
  ResultCode initialize_bindings(const mjModel& model);
  void clear_odometry();
  void update_state_fields(const mjData& data);
  void integrate_wheel_odometry(double simulation_time);
  ResultCode update_ground_truth_pose(const mjData& data);
  static double normalized_yaw(double yaw);
  double command_linear_x(const MobileBaseCommand& command) const;
  double command_linear_y(const MobileBaseCommand& command) const;
  double command_angular_z(const MobileBaseCommand& command) const;
  ResultCode write_differential(const mjModel& model, mjData& data,
                                const MobileBaseCommand& command);
  ResultCode write_omnidirectional(const mjModel& model, mjData& data,
                                   const MobileBaseCommand& command);
  ResultCode read_differential(const mjData& data, MobileBaseState& state);
  ResultCode read_omnidirectional(const mjData& data, MobileBaseState& state);
  std::size_t wheel_count() const;
  static double clamp_actuator_control_limits(const mjModel& model, const JointBinding& binding,
                                              double value);
  static ResultCode write_wheel_velocity_command(const mjModel& model, mjData& data,
                                                 const MobileBaseWheelBinding& wheel,
                                                 double velocity);
  static ResultCode read_wheel_velocity(const mjData& data, const MobileBaseWheelBinding& wheel,
                                        double& velocity);

  MobileBaseBinding binding_{};
  double last_simulation_time_{std::numeric_limits<double>::quiet_NaN()};
  MobileBaseConfig config_;
  MobileBaseCommand command_;
  MobileBaseState state_;
};

}  // namespace mujoco_simulation
