#pragma once

#include <mujoco/mujoco.h>

#include <memory>
#include <string>

#include "mujoco_simulation/component/joint/joint_data.hpp"
#include "mujoco_simulation/component/simulation_component.hpp"

namespace mujoco_simulation {

class JointComponent : public SimulationComponent {
public:
  explicit JointComponent(JointInfo info);

  bool init(const mjContext &context) override;
  bool reset(const mjContext &context) override;
  bool update(const mjContext &context) override;

  bool write(const mjContext &context, const JointCommand &command);
  bool read_state(std::shared_ptr<const JointState> &state) const;
  bool read(const mjContext &context, JointState &state) const;

  const JointInfo &info() const noexcept { return info_; }
  std::string joint_name() const noexcept { return info_.joint_name; }
  std::string actuator_name() const noexcept { return info_.actuator_name; }
  int joint_id() const noexcept { return joint_.joint_id; }
  int actuator_id() const noexcept { return joint_.actuator_id; }
  JointType joint_type() const noexcept { return info_.joint_type; }
  bool is_initialized() const noexcept;

public:
  using SharedPtr = std::shared_ptr<JointComponent>;
  using UniquePtr = std::unique_ptr<JointComponent>;
  using WeakPtr = std::weak_ptr<JointComponent>;

private:
  // command
  bool write_position_command(const mjContext &context,
                              const JointCommand &command) const;
  bool write_velocity_command(const mjContext &context,
                              const JointCommand &command) const;
  bool write_effort_command(const mjContext &context,
                            const JointCommand &command) const;
  bool write_hybrid_command(const mjContext &context,
                            const JointCommand &command) const;

  // limit
  double clamp_limits(const Limit &limits, double value) const;
  double clamp_ctrl_limits(const mjContext &context, double value) const;
  double clamp_force_limits(const mjContext &context, double value) const;

  // 仿真信息
  mjJoint joint_{};
  // 关节信息
  JointInfo info_;
  // 最新关节指令
  JointCommand command_{};
  // 最新关节状态
  std::shared_ptr<const JointState> state_;
  // 初始化标志
  bool initialized_{false};
};

} // namespace mujoco_simulation
