#pragma once

#include <memory>
#include <string>

#include <mujoco/mujoco.h>

#include "romujoco/component/joint.hpp"

#include "component/component.hpp"

namespace romujoco {

class JointComponent : public SimulationComponent {
public:
    explicit JointComponent(JointInfo info);

    bool init(const SimulationContext& context) override;
    bool reset(const SimulationContext& context) override;
    bool reset(const SimulationContext& context, JointCommand& command);
    bool advance(const SimulationContext& context) override;
    bool update(const SimulationContext& context) override;

    bool write(const SimulationContext& context, const JointCommand& command);
    bool read_state(std::shared_ptr<const JointState>& state) const;
    bool read(const SimulationContext& context, JointState& state) const;

    const JointInfo& info() const noexcept { return info_; }
    std::string joint_name() const noexcept { return info_.joint_name; }
    std::string actuator_name() const noexcept { return info_.actuator_name; }
    int joint_id() const noexcept { return joint_.joint_id; }
    int actuator_id() const noexcept { return joint_.actuator_id; }
    JointType joint_type() const noexcept { return info_.joint_type; }
    JointActuation actuation() const noexcept { return info_.actuation; }
    bool is_active_joint() const noexcept;
    bool is_passive_joint() const noexcept;
    bool is_initialized() const noexcept;
    bool supports_mode(std::uint8_t mode) const noexcept;

public:
    using SharedPtr = std::shared_ptr<JointComponent>;
    using UniquePtr = std::unique_ptr<JointComponent>;
    using WeakPtr = std::weak_ptr<JointComponent>;

private:
    bool validate_info() const;
    bool validate_actuator(const SimulationContext& context) const;
    bool validate_actuator_uniqueness(const SimulationContext& context) const;
    bool make_reset_command(const SimulationContext& context, JointCommand& command) const;
    double position_error(double target, double current) const noexcept;

    // command
    bool write_position_command(
        const SimulationContext& context, const JointCommand& command) const;
    bool write_velocity_command(
        const SimulationContext& context, const JointCommand& command) const;
    bool write_effort_command(const SimulationContext& context, const JointCommand& command) const;
    bool write_hybrid_command(const SimulationContext& context, const JointCommand& command) const;

    // limit
    double clamp_limits(const JointLimit& limits, double value) const;
    double clamp_ctrl_limits(const SimulationContext& context, double value) const;
    double clamp_force_limits(const SimulationContext& context, double value) const;

    // gravity compensation
    double gravity_compensation_effort(const SimulationContext& context) const;

private:
    bool initialized_{false};
    bool shortest_angular_distance_{false};
    JointBinding joint_{};
    JointInfo info_;
    JointCommand command_{};
    std::shared_ptr<const JointState> state_;
    mutable std::unique_ptr<mjData, MjDataDeleter> gravity_data_;
};

}  // namespace romujoco
