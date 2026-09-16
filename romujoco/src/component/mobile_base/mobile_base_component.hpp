#pragma once

#include <memory>

#include <mujoco/mujoco.h>

#include "romujoco/component/mobile_base.hpp"

#include "component/component.hpp"
namespace romujoco {

class MobileBaseComponent : public SimulationComponent {
public:
    using SimulationComponent::SimulationComponent;
    ~MobileBaseComponent() override = default;
    MobileBaseComponent(const MobileBaseComponent&) = delete;
    MobileBaseComponent& operator=(const MobileBaseComponent&) = delete;

    virtual bool write(const mjContext& context, const MobileBaseCommand& command) = 0;
    virtual bool read_state(std::shared_ptr<const MobileBaseState>& state) const = 0;

public:
    using SharedPtr = std::shared_ptr<MobileBaseComponent>;
    using UniquePtr = std::unique_ptr<MobileBaseComponent>;
    using WeakPtr = std::weak_ptr<MobileBaseComponent>;
};

}  // namespace romujoco
