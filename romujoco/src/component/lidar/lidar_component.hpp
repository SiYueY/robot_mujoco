#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "romujoco/component/lidar.hpp"

#include "component/component.hpp"

namespace romujoco {

class LidarComponent : public SimulationComponent {
public:
    explicit LidarComponent(LidarInfo info);

    bool init(const SimulationContext& context) override;
    bool reset(const SimulationContext& context) override;
    bool advance(const SimulationContext& context) override;
    bool update(const SimulationContext& context) override;

    bool read_state(std::shared_ptr<const LidarState>& state) const;
    bool read(const SimulationContext& context, LidarState& state) const;

    const LidarInfo& info() const noexcept { return info_; }
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

}  // namespace romujoco
