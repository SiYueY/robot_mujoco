#pragma once

#include <limits>
#include <memory>
#include <string>

#include "mujoco_simulation/component/joint/joint_component.hpp"
#include "mujoco_simulation/component/mobile_base/mobile_base_data.hpp"
#include "mujoco_simulation/component/simulation_component.hpp"

namespace mujoco_simulation {

class MobileBaseComponent : public SimulationComponent {
 public:
  explicit MobileBaseComponent(MobileBaseInfo info);
  ~MobileBaseComponent();

  MobileBaseComponent(const MobileBaseComponent&) = delete;
  MobileBaseComponent& operator=(const MobileBaseComponent&) = delete;
  MobileBaseComponent(MobileBaseComponent&&) noexcept;
  MobileBaseComponent& operator=(MobileBaseComponent&&) noexcept;

  bool configure_differential_drive(const JointComponent& left_wheel,
                                    const JointComponent& right_wheel);
  bool configure_omnidirectional_drive(const JointComponent& front_left,
                                       const JointComponent& front_right,
                                       const JointComponent& rear_left,
                                       const JointComponent& rear_right);

  std::string name() const noexcept override;
  bool bind(const mjModel& model) override;
  bool reset(const mjModel& model, mjData& data) override;
  bool update(const UpdateContext& context) override;

  bool write(const mjModel& model, mjData& data, const MobileBaseCommand& command);
  bool read(const mjData& data, MobileBaseState& state);

  const MobileBaseInfo& info() const noexcept { return info_; }

 private:
  bool validate(const mjModel& model) const;
  bool initialize_bindings(const mjModel& model);
  void clear_odometry();
  void update_state_fields(const mjData& data);
  void integrate_wheel_odometry(double simulation_time);
  bool update_ground_truth_pose(const mjData& data);
  static double normalized_yaw(double yaw);
  double command_linear_x(const MobileBaseCommand& command) const;
  double command_linear_y(const MobileBaseCommand& command) const;
  double command_angular_z(const MobileBaseCommand& command) const;
  bool write_differential(const mjModel& model, mjData& data, const MobileBaseCommand& command);
  bool write_omnidirectional(const mjModel& model, mjData& data, const MobileBaseCommand& command);
  bool read_differential(const mjData& data, MobileBaseState& state);
  bool read_omnidirectional(const mjData& data, MobileBaseState& state);
  std::size_t wheel_count() const;

  struct Impl;
  std::unique_ptr<Impl> impl_;
  double last_simulation_time_{std::numeric_limits<double>::quiet_NaN()};
  MobileBaseInfo info_;
  MobileBaseCommand command_;
  MobileBaseState state_;
};

}  // namespace mujoco_simulation
