#pragma once

#include <memory>
#include <string>

#include <mujoco/mujoco.h>

#include "mujoco_simulation/component/joint.hpp"

#include "component/component.hpp"

namespace mujoco_simulation {

class JointComponent : public SimulationComponent {
public:
    explicit JointComponent(JointInfo info);

    bool init(const mjContext& context) override;
    bool reset(const mjContext& context) override;
    bool reset(const mjContext& context, JointCommand& command);
    bool advance(const mjContext& context) override;
    bool update(const mjContext& context) override;

    bool write(const mjContext& context, const JointCommand& command);
    bool read_state(std::shared_ptr<const JointState>& state) const;
    bool read(const mjContext& context, JointState& state) const;

    const JointInfo& info() const noexcept { return info_; }
    std::string joint_name() const noexcept { return info_.joint_name; }
    std::string actuator_name() const noexcept { return info_.actuator_name; }
    int joint_id() const noexcept { return joint_.joint_id; }
    int actuator_id() const noexcept { return joint_.actuator_id; }
    JointType joint_type() const noexcept { return info_.joint_type; }
    bool is_initialized() const noexcept;
    bool supports_mode(uint8_t mode) const noexcept;

public:
    using SharedPtr = std::shared_ptr<JointComponent>;
    using UniquePtr = std::unique_ptr<JointComponent>;
    using WeakPtr = std::weak_ptr<JointComponent>;

private:
    bool make_reset_command(const mjContext& context, JointCommand& command) const;

    // command
    bool write_position_command(const mjContext& context, const JointCommand& command) const;
    bool write_velocity_command(const mjContext& context, const JointCommand& command) const;
    bool write_effort_command(const mjContext& context, const JointCommand& command) const;
    bool write_hybrid_command(const mjContext& context, const JointCommand& command) const;

    // limit
    double clamp_limits(const JointLimit& limits, double value) const;
    double clamp_ctrl_limits(const mjContext& context, double value) const;
    double clamp_force_limits(const mjContext& context, double value) const;

    // gravity compensation
    double gravity_compensation_effort(const mjContext& context) const;

private:
    bool initialized_{false};
    mjJoint joint_{};
    JointInfo info_;
    JointCommand command_{};
    std::shared_ptr<const JointState> state_;
    mutable std::unique_ptr<mjData, MjDataDeleter> gravity_data_;
};

}  // namespace mujoco_simulation
