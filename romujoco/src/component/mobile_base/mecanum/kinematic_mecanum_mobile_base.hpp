#pragma once
#include <array>
#include <memory>
#include <mujoco/mujoco.h>
#include "romujoco/component/mobile_base/mecanum.hpp"
#include "component/mobile_base/mobile_base_component.hpp"
#include "component/mobile_base/mecanum/mecanum_kinematics.hpp"
namespace romujoco {
class KinematicMecanumMobileBase final : public MobileBaseComponent {
public:
    explicit KinematicMecanumMobileBase(MecanumMobileBaseInfo info);
    bool init(const mjContext&) override;
    bool reset(const mjContext&) override;
    bool advance(const mjContext&) override;
    bool update(const mjContext&) override;
    bool write(const mjContext&, const MobileBaseCommand&) override;
    bool read_state(std::shared_ptr<const MobileBaseState>&) const override;

private:
    struct Wheel {
        int joint{-1}, qpos{-1}, dof{-1};
        double radius{}, direction{}, response{}, target{}, feedback{}, position{};
    };
    bool base(const mjContext&);
    bool wheel(const mjContext&, const MecanumWheelInfo&, Wheel&);
    void publish(const mjContext&);
    MecanumMobileBaseInfo info_;
    MecanumKinematics ik_;
    std::array<Wheel, MecanumWheelCount> wheels_{};
    int q_{-1}, d_{-1};
    Vector3d origin_{};
    double origin_yaw_{}, x_{}, y_{}, yaw_{};
    Vector3d linear_{}, angular_{};
    MobileBaseState working_{};
    std::shared_ptr<const MobileBaseState> state_;
    bool ready_{};
};
}  // namespace romujoco
