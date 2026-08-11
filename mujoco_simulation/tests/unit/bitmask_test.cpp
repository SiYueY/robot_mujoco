#include <cstdint>
#include <iostream>
#include <limits>
#include <type_traits>

#include "mujoco_simulation/common/bitmask.hpp"

namespace {

enum class TestMode : int {
    Hybrid = 0,
    Velocity = 3,
    HighestByteBit = 7,
    Negative = -1,
    OutOfRange = 8,
};

bool check(bool condition, const char* message) {
    if (!condition) std::cerr << message << '\n';
    return condition;
}

}  // namespace

int main() {
    using mujoco_simulation::BitMask;
    using mujoco_simulation::EnumMask;

    BitMask<std::uint8_t> bits;
    if (!check(bits.empty(), "new integer bitmask is not empty") ||
        !check(bits.set(0), "failed to set lowest bit") ||
        !check(bits.set(7), "failed to set highest uint8 bit") ||
        !check(bits.contains(0) && bits.contains(7), "set integer bits are absent") ||
        !check(bits.value() == 0x81U, "integer bitmask value is incorrect") ||
        !check(
            bits.reset(0) && !bits.contains(0) && bits.contains(7), "integer reset is incorrect") ||
        !check(
            !bits.set(8) && !bits.reset(8) && !bits.contains(8),
            "out-of-range integer bit was accepted") ||
        !check(bits.value() == 0x80U, "invalid integer bit changed the bitmask")) {
        return 1;
    }
    bits.clear();
    if (!check(bits.empty() && bits.value() == 0U, "integer clear is incorrect")) return 1;

    BitMask<std::uint64_t> wide_bits;
    constexpr std::size_t kHighestUint64Bit = std::numeric_limits<std::uint64_t>::digits - 1U;
    if (!check(
            wide_bits.set(kHighestUint64Bit) && wide_bits.contains(kHighestUint64Bit),
            "failed to set highest uint64 bit") ||
        !check(!wide_bits.set(kHighestUint64Bit + 1U), "uint64 overflow bit was accepted")) {
        return 1;
    }

    static_assert(
        std::is_same<decltype(EnumMask<TestMode>{}.value()), unsigned int>::value,
        "EnumMask must default to the enum's unsigned underlying type");
    EnumMask<TestMode, std::uint8_t> enum_bits;
    if (!check(enum_bits.empty(), "new enum bitmask is not empty") ||
        !check(
            enum_bits.set(TestMode::Hybrid) && enum_bits.set(TestMode::Velocity) &&
                enum_bits.set(TestMode::HighestByteBit),
            "failed to set enum bits") ||
        !check(
            enum_bits.contains(TestMode::Hybrid) && enum_bits.contains(TestMode::Velocity) &&
                enum_bits.contains(TestMode::HighestByteBit),
            "set enum bits are absent") ||
        !check(enum_bits.value() == 0x89U, "enum bitmask value is incorrect") ||
        !check(
            enum_bits.reset(TestMode::Velocity) && !enum_bits.contains(TestMode::Velocity),
            "enum reset is incorrect")) {
        return 1;
    }
    const std::uint8_t value_before_invalid = enum_bits.value();
    if (!check(
            !enum_bits.set(TestMode::Negative) && !enum_bits.reset(TestMode::Negative) &&
                !enum_bits.contains(TestMode::Negative),
            "negative enum bit was accepted") ||
        !check(
            !enum_bits.set(TestMode::OutOfRange) && !enum_bits.reset(TestMode::OutOfRange) &&
                !enum_bits.contains(TestMode::OutOfRange),
            "out-of-range enum bit was accepted") ||
        !check(enum_bits.value() == value_before_invalid, "invalid enum bit changed the bitmask")) {
        return 1;
    }
    enum_bits.clear();
    return check(enum_bits.empty() && enum_bits.value() == 0U, "enum clear is incorrect") ? 0 : 1;
}
