#pragma once

#include <mujoco/mujoco.h>

#include <array>
#include <memory>
#include <optional>
#include <string>

#include "mujoco_simulation/component/mobile_base/mobile_base_data.hpp"
#include "mujoco_simulation/component/simulation_component.hpp"

namespace mujoco_simulation {

class MobileBaseComponent : public SimulationComponent {
public:
  explicit MobileBaseComponent(MobileBaseInfo info);
  MobileBaseComponent(const MobileBaseComponent &) = delete;
  MobileBaseComponent &operator=(const MobileBaseComponent &) = delete;

  bool init(const mjContext &context) override;
  bool reset(const mjContext &context) override;
  bool update(const mjContext &context) override;

  bool write(const mjContext &context, const MobileBaseCommand &command);
  bool read_state(std::shared_ptr<const MobileBaseState> &state) const;
  bool read(const mjContext &context, MobileBaseState &state) const;

  const MobileBaseInfo &info() const noexcept { return info_; }
  bool is_initialized() const noexcept;

public:
  using SharedPtr = std::shared_ptr<MobileBaseComponent>;
  using UniquePtr = std::unique_ptr<MobileBaseComponent>;
  using WeakPtr = std::weak_ptr<MobileBaseComponent>;

private:
  bool configure_wheel(const mjContext &context, const WheelInfo &info,
                       mjWheel &wheel) const;
  bool configure_base_body(const mjContext &context);

  bool init_mecanum(const mjContext &context);
  bool reset_mecanum(const mjContext &context);
  bool update_mecanum_state(const mjContext &context);
  bool write_mecanum_command(const mjContext &context,
                             const MobileBaseCommand &command);

  bool write_twist_command(const mjContext &context,
                           const MobileBaseCommand &command);
  bool write_wheel_linear_command(const mjContext &context,
                                  const MobileBaseCommand &command);
  bool write_wheel_angular_command(const mjContext &context,
                                   const MobileBaseCommand &command);
  bool write_wheel_commands(const mjContext &context,
                            const Vector4d &wheel_angular);

  double clamp_ctrl_limits(const mjContext &context, const mjWheel &wheel,
                           double value) const;
  double clamp_force_limits(const mjContext &context, const mjWheel &wheel,
                            double value) const;

  void reset_odometry();
  bool update_ground_truth_pose(const mjData &data);
  static double wrap_angle(double angle);

  int base_body_id_{-1};

  // 移动底盘信息
  MobileBaseInfo info_;
  // 最新移动底盘指令
  MobileBaseCommand command_;
  // 供本次更新构建的移动底盘状态，以及已发布的不可变快照。
  MobileBaseState working_state_;
  std::shared_ptr<const MobileBaseState> state_;
  // 初始化标志
  bool initialized_{false};

  // Mechanum 移动底盘
  using MecanumWheels = std::array<mjWheel, MecanumWheelCount>;
  MecanumWheels mecanum_wheels_{};
  std::optional<MecanumKinematics> mecanum_kinematics_;
};

} // namespace mujoco_simulation
