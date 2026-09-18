#pragma once

#include <array>
#include <memory>

#include <mujoco/mujoco.h>

#include "romujoco/component/mobile_base/mecanum.hpp"

#include "component/mobile_base/mobile_base_component.hpp"
#include "component/mobile_base/mecanum/mecanum_kinematics.hpp"

namespace romujoco {

// Kinematic mecanum chassis: the base free joint and the wheel joints are
// prescribed directly, so the chassis pose is owned by this component instead
// of MuJoCo dynamics.
class KinematicMecanumMobileBase final : public MobileBaseComponent {
public:
    explicit KinematicMecanumMobileBase(MecanumMobileBaseInfo info);

    bool init(const SimulationContext& context) override;
    bool reset(const SimulationContext& context) override;
    bool advance(const SimulationContext& context) override;
    bool update(const SimulationContext& context) override;
    bool write(const SimulationContext& context, const MobileBaseCommand& command) override;
    bool read_state(std::shared_ptr<const MobileBaseState>& state) const override;

private:
    // MuJoCo resources and runtime values of one mecanum wheel.
    struct Wheel {
        int joint_id{-1};
        int qpos_address{-1};
        int dof_address{-1};
        double radius{0.0};
        double direction{1.0};
        double response{0.0};
        double target{0.0};
        double feedback{0.0};
        double position{0.0};
    };

    bool bind_base_joint(const SimulationContext& context);
    bool bind_wheel(
        const SimulationContext& context, const MecanumWheelInfo& wheel_info, Wheel& wheel);
    void publish(const SimulationContext& context);

    MecanumMobileBaseInfo info_;
    MecanumKinematics kinematics_;
    std::array<Wheel, kMecanumWheelCount> wheels_{};
    int base_qpos_{-1};
    int base_dof_{-1};
    Vector3d origin_{};
    double origin_yaw_{0.0};
    // Planar pose accumulated since the last reset, relative to `origin_`.
    double local_x_{0.0};
    double local_y_{0.0};
    double local_yaw_{0.0};
    Vector3d linear_velocity_{};
    Vector3d angular_velocity_{};
    MobileBaseState working_{};
    std::shared_ptr<const MobileBaseState> state_;
    bool ready_{false};
};

}  // namespace romujoco
