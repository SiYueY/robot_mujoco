#pragma once

#include <array>
#include <cstddef>
#include <memory>

#include <mujoco/mujoco.h>

#include "romujoco/component/gripper.hpp"

#include "component/component.hpp"

namespace romujoco {

// Device-level controller for a two-finger parallel gripper.
//
// The component owns a velocity-limited width reference.  Every physics step
// `advance()` moves that reference towards the commanded width and writes a PD
// generalized force to each actuated finger; `update()` samples the device
// state and evaluates stall detection.  Fingers without an actuator are left to
// the MJCF mechanical coupling.
class GripperComponent : public SimulationComponent {
public:
    explicit GripperComponent(GripperInfo info);

    bool init(const SimulationContext& context) override;
    bool reset(const SimulationContext& context) override;
    bool reset(const SimulationContext& context, GripperCommand& command);
    bool advance(const SimulationContext& context) override;
    bool update(const SimulationContext& context) override;

    bool write(const SimulationContext& context, const GripperCommand& command);
    bool read_state(std::shared_ptr<const GripperState>& state) const;

    const GripperInfo& info() const noexcept { return info_; }

    using SharedPtr = std::shared_ptr<GripperComponent>;
    using UniquePtr = std::unique_ptr<GripperComponent>;
    using WeakPtr = std::weak_ptr<GripperComponent>;

private:
    // MuJoCo resources bound to one finger of the gripper.
    struct FingerBinding {
        int joint_id{-1};
        int actuator_id{-1};
        int qpos_address{-1};
        int dof_address{-1};
    };

    bool validate_info() const;
    bool bind_finger(
        const SimulationContext& context, const GripperFingerInfo& finger_info,
        FingerBinding& binding) const;
    bool validate_actuator(
        const SimulationContext& context, const GripperFingerInfo& finger_info,
        const FingerBinding& binding) const;
    void apply_control(const SimulationContext& context);
    double measured_width(const SimulationContext& context) const noexcept;
    double measured_width_velocity(const SimulationContext& context) const noexcept;
    double measured_effort(const SimulationContext& context) const noexcept;
    void update_stall_state(const SimulationContext& context, const GripperState& state);

    GripperInfo info_;
    std::array<FingerBinding, kGripperFingerCount> fingers_{};
    double target_width_{0.0};
    double reference_width_{0.0};
    double reference_velocity_{0.0};
    GripperCommand command_{};
    std::shared_ptr<const GripperState> state_;
    bool stalled_{false};
    bool stall_timer_active_{false};
    double stall_start_time_{0.0};
    bool initialized_{false};
};

}  // namespace romujoco
