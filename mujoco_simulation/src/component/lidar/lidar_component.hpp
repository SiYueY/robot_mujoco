#pragma once
// Internal lidar component contract.

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "component/component.hpp"
#include "mujoco_simulation/component/lidar.hpp"

namespace mujoco_simulation {

class LidarComponent : public SimulationComponent {
public:
  explicit LidarComponent(LidarInfo info);

  bool init(const mjContext &context) override;
  bool reset(const mjContext &context) override;
  bool update(const mjContext &context) override;

  bool read_state(std::shared_ptr<const LidarState> &state) const;
  bool read(const mjContext &context, LidarState &state) const;

  const LidarInfo &info() const noexcept { return info_; }
  bool is_initialized() const noexcept { return initialized_; }

public:
  using SharedPtr = std::shared_ptr<LidarComponent>;
  using UniquePtr = std::unique_ptr<LidarComponent>;
  using WeakPtr = std::weak_ptr<LidarComponent>;

private:
  LidarInfo info_;
  std::vector<int> beam_addresses_;
  std::shared_ptr<const LidarState> state_;
  std::uint64_t sequence_{0};
  bool initialized_{false};
};

} // namespace mujoco_simulation
