#include <cstring>
#include <type_traits>

#include "romujoco/version.hpp"

static_assert(std::is_same_v<decltype(romujoco::version::major), const int>);
static_assert(std::is_same_v<decltype(romujoco::version::minor), const int>);
static_assert(std::is_same_v<decltype(romujoco::version::patch), const int>);
static_assert(std::is_array_v<decltype(romujoco::version::string)>);
static_assert(
    std::is_same_v<std::remove_extent_t<decltype(romujoco::version::string)>, const char>);
static_assert(romujoco::version::major == 0);
static_assert(romujoco::version::minor == 1);
static_assert(romujoco::version::patch == 0);

int main() { return std::strcmp(romujoco::version::string, "0.1.0") == 0 ? 0 : 1; }
