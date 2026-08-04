#include <cstring>
#include <type_traits>

#include "mujoco_simulation/version.hpp"

static_assert(std::is_same_v<decltype(mujoco_simulation::version::major), const int>);
static_assert(std::is_same_v<decltype(mujoco_simulation::version::minor), const int>);
static_assert(std::is_same_v<decltype(mujoco_simulation::version::patch), const int>);
static_assert(std::is_array_v<decltype(mujoco_simulation::version::string)>);
static_assert(
    std::is_same_v<std::remove_extent_t<decltype(mujoco_simulation::version::string)>, const char>);
static_assert(mujoco_simulation::version::major == 0);
static_assert(mujoco_simulation::version::minor == 1);
static_assert(mujoco_simulation::version::patch == 0);

int main() { return std::strcmp(mujoco_simulation::version::string, "0.1.0") == 0 ? 0 : 1; }
