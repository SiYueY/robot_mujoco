#pragma once
#include <array>
#include <cstddef>
#include <string>
#include "romujoco/component/mobile_base/common.hpp"
namespace romujoco {
enum class MecanumWheelIndex : std::size_t { FrontLeft, FrontRight, RearLeft, RearRight, Count };
inline constexpr std::size_t kMecanumWheelCount =
    static_cast<std::size_t>(MecanumWheelIndex::Count);
struct MecanumWheelInfo {
    std::string joint_name;
    double radius{0.0};
    double direction{1.0};
    double speed_response{0.0};
};
struct MecanumMobileBaseInfo {
    MobileBaseCommonInfo common;
    double wheel_base{0.0};
    double track_width{0.0};
    std::array<MecanumWheelInfo, kMecanumWheelCount> wheels;
};
}  // namespace romujoco
