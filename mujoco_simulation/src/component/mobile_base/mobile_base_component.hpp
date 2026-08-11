#pragma once

#include <memory>

#include <mujoco/mujoco.h>

#include "mujoco_simulation/component/mobile_base.hpp"

#include "component/component.hpp"
namespace mujoco_simulation {

class MecanumMobileBase;

class MobileBaseComponent : public SimulationComponent {
public:
    explicit MobileBaseComponent(MobileBaseInfo info);
    ~MobileBaseComponent() override;
    MobileBaseComponent(const MobileBaseComponent&) = delete;
    MobileBaseComponent& operator=(const MobileBaseComponent&) = delete;

    bool init(const mjContext& context) override;
    bool reset(const mjContext& context) override;
    bool advance(const mjContext& context) override;
    bool update(const mjContext& context) override;

    bool write(const mjContext& context, const MobileBaseCommand& command);
    bool read_state(std::shared_ptr<const MobileBaseState>& state) const;
    bool read(const mjContext& context, MobileBaseState& state) const;

    const MobileBaseInfo& info() const noexcept { return info_; }
    bool is_initialized() const noexcept;

public:
    using SharedPtr = std::shared_ptr<MobileBaseComponent>;
    using UniquePtr = std::unique_ptr<MobileBaseComponent>;
    using WeakPtr = std::weak_ptr<MobileBaseComponent>;

private:
    bool configure_base(const mjContext& context);
    bool reset_planar_base(const mjContext& context);
    bool advance_planar_base(const mjContext& context);
    void publish_state(const mjContext& context);
    static double wrap_angle(double angle);

    int base_qpos_address_{-1};
    int base_dof_address_{-1};
    Vector3d world_odom_position_{};
    double world_odom_yaw_{0.0};
    Vector3d odom_pose_{};

    MobileBaseInfo info_;
    MobileBaseState working_state_;
    std::shared_ptr<const MobileBaseState> state_;
    bool initialized_{false};
    std::unique_ptr<MecanumMobileBase> mecanum_;
};

}  // namespace mujoco_simulation
