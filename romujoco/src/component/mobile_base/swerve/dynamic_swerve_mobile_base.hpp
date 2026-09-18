#pragma once

#include <memory>
#include <vector>

#include "romujoco/component/mobile_base/swerve.hpp"

#include "component/mobile_base/mobile_base_component.hpp"
#include "component/mobile_base/swerve/swerve_kinematics.hpp"

namespace romujoco {

// Dynamic swerve chassis: MuJoCo owns the base pose and the wheel-ground
// contact, while this component only commands the module actuators.
class DynamicSwerveMobileBase final : public MobileBaseComponent {
public:
    explicit DynamicSwerveMobileBase(SwerveMobileBaseInfo info);

    bool init(const SimulationContext& context) override;
    bool reset(const SimulationContext& context) override;
    bool advance(const SimulationContext& context) override { return ready_; }
    bool update(const SimulationContext& context) override;
    bool write(const SimulationContext& context, const MobileBaseCommand& command) override;
    bool read_state(std::shared_ptr<const MobileBaseState>& state) const override;

private:
    // MuJoCo resources of one swerve module.
    struct Module {
        int steering_joint{-1};
        int steering_actuator{-1};
        int steering_qpos{-1};
        int steering_dof{-1};
        int drive_joint{-1};
        int drive_actuator{-1};
        int drive_qpos{-1};
        int drive_dof{-1};
    };
    SwerveMobileBaseInfo info_;
    std::vector<Module> modules_;
    SwerveKinematics kinematics_;
    int base_qpos_{-1};
    int base_dof_{-1};
    std::vector<double> feedback_;
    std::vector<SwerveModuleTarget> targets_;
    MobileBaseState working_{};
    std::shared_ptr<const MobileBaseState> state_;
    bool ready_{false};
};
}  // namespace romujoco
