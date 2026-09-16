#pragma once
#include <cstdint>
#include <string>
#include "romujoco/component/component_id.hpp"
namespace romujoco {
enum class MobileBaseExecutionMode : std::uint8_t { Kinematic, Dynamic };
struct MobileBaseCommonInfo {
    ComponentId id{kInvalidComponentId};
    std::string name;
    std::string base_body_name;
    MobileBaseExecutionMode execution_mode{MobileBaseExecutionMode::Dynamic};
    double period{0.0};
};
}  // namespace romujoco
