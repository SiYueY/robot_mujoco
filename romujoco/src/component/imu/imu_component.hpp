#pragma once

#include <memory>
#include <string>

#include <mujoco/mujoco.h>

#include "romujoco/component/imu.hpp"

#include "component/component.hpp"
#include "runtime/simulation_context.hpp"

namespace romujoco {

class ImuComponent : public SimulationComponent {
public:
    explicit ImuComponent(ImuInfo info);

    bool init(const SimulationContext& context) override;
    bool reset(const SimulationContext& context) override;
    bool advance(const SimulationContext& context) override;
    bool update(const SimulationContext& context) override;

    bool read_state(std::shared_ptr<const ImuState>& state) const;
    bool read(const SimulationContext& context, ImuState& state) const;

    const ImuInfo& info() const noexcept { return info_; }
    bool is_initialized() const noexcept { return initialized_; }

public:
    using SharedPtr = std::shared_ptr<ImuComponent>;
    using UniquePtr = std::unique_ptr<ImuComponent>;
    using WeakPtr = std::weak_ptr<ImuComponent>;

private:
    // Imu 信息
    ImuInfo info_;
    // 仿真信息
    ImuBinding imu_{};
    std::uint64_t sequence_{0};
    // Imu 状态
    std::shared_ptr<const ImuState> state_;
    // 初始化标志
    bool initialized_{false};
};

}  // namespace romujoco
