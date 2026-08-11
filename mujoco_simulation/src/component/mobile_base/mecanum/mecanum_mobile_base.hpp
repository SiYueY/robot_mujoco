#pragma once

#include <array>

#include <mujoco/mujoco.h>

#include "mujoco_simulation/component/mobile_base.hpp"

#include "component/mobile_base/mecanum/mecanum_kinematics.hpp"
#include "runtime/context.hpp"

namespace mujoco_simulation {

class MecanumMobileBase {
public:
    explicit MecanumMobileBase(const MecanumInfo& info);
    MecanumMobileBase(const MecanumMobileBase&) = delete;
    MecanumMobileBase& operator=(const MecanumMobileBase&) = delete;

    bool init(const mjContext& context, const MecanumWheelInfo& wheels);
    bool reset(const mjContext& context);
    bool write(const MobileBaseCommand& command);
    bool advance(const mjContext& context);

    const Vector3d& base_linear() const noexcept { return base_linear_; }
    const Vector3d& base_angular() const noexcept { return base_angular_; }
    const Vector4d& wheel_angular() const noexcept { return wheel_angular_; }
    const Vector4d& wheel_linear() const noexcept { return wheel_linear_; }

private:
    struct Wheel {
        int joint_id{-1};
        int qpos_address{-1};
        int dof_address{-1};
        double radius{0.0};
        double direction{1.0};
        double speed_response{0.0};
        double target{0.0};
        double feedback{0.0};
        double position{0.0};
    };

    bool configure_wheel(const mjContext& context, const WheelInfo& info, Wheel& wheel) const;
    bool target_from_command(const MobileBaseCommand& command, Vector4d& target) const;

    MecanumInfo info_;
    MecanumKinematics kinematics_;
    std::array<Wheel, MecanumWheelCount> wheels_{};
    Vector3d base_linear_{};
    Vector3d base_angular_{};
    Vector4d wheel_angular_{};
    Vector4d wheel_linear_{};
    bool initialized_{false};
};

}  // namespace mujoco_simulation
