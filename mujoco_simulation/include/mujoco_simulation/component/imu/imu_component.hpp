#pragma once

#include <mujoco/mujoco.h>

#include <memory>
#include <string>

#include "mujoco_simulation/component/imu/imu_data.hpp"
#include "mujoco_simulation/component/simulation_component.hpp"
#include "mujoco_simulation/mujoco/context.hpp"

namespace mujoco_simulation {

class ImuComponent : public SimulationComponent {
public:
  explicit ImuComponent(ImuInfo info);

  bool init(const mjContext &context) override;
  bool reset(const mjContext &context) override;
  bool update(const mjContext &context) override;

  bool read_state(std::shared_ptr<const ImuState> &state) const;
  bool read(const mjContext &context, ImuState &state) const;

  const ImuInfo &info() const noexcept { return info_; }
  bool is_initialized() const noexcept { return initialized_; }

public:
  using SharedPtr = std::shared_ptr<ImuComponent>;
  using UniquePtr = std::unique_ptr<ImuComponent>;
  using WeakPtr = std::weak_ptr<ImuComponent>;

private:
  // Imu 信息
  ImuInfo info_;
  // 仿真信息
  mjImu imu_{};
  std::uint64_t sequence_{0};
  // Imu 状态
  std::shared_ptr<const ImuState> state_;
  // 初始化标志
  bool initialized_{false};
};

} // namespace mujoco_simulation
