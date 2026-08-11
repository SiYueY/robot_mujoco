#pragma once

#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <limits>
#include <type_traits>

namespace mujoco_simulation {

template <typename Underlying = std::uint64_t>
class BitMask final {
    static_assert(
        std::is_integral<Underlying>::value && std::is_unsigned<Underlying>::value,
        "BitMask requires an unsigned integral underlying type");

public:
    constexpr bool set(std::size_t bit) noexcept {
        if (!valid(bit)) return false;
        value_ |= bit_value(bit);
        return true;
    }

    constexpr bool reset(std::size_t bit) noexcept {
        if (!valid(bit)) return false;
        value_ &= static_cast<Underlying>(~bit_value(bit));
        return true;
    }

    constexpr bool contains(std::size_t bit) const noexcept {
        return valid(bit) && (value_ & bit_value(bit)) != 0;
    }

    constexpr bool empty() const noexcept { return value_ == 0; }
    constexpr void clear() noexcept { value_ = 0; }
    constexpr Underlying value() const noexcept { return value_; }

private:
    static constexpr bool valid(std::size_t bit) noexcept {
        return bit < std::numeric_limits<Underlying>::digits;
    }

    static constexpr Underlying bit_value(std::size_t bit) noexcept {
        return static_cast<Underlying>(Underlying{1} << bit);
    }

    Underlying value_{0};
};

template <typename Enum, typename Underlying = std::make_unsigned_t<std::underlying_type_t<Enum>>>
class EnumMask final {
    static_assert(std::is_enum<Enum>::value, "EnumMask requires an enum type");

public:
    constexpr EnumMask() noexcept = default;

    EnumMask(std::initializer_list<Enum> values) noexcept {
        for (const Enum value : values) set(value);
    }

    constexpr bool set(Enum value) noexcept {
        std::size_t bit = 0;
        return to_bit(value, bit) && bits_.set(bit);
    }

    constexpr bool reset(Enum value) noexcept {
        std::size_t bit = 0;
        return to_bit(value, bit) && bits_.reset(bit);
    }

    constexpr bool contains(Enum value) const noexcept {
        std::size_t bit = 0;
        return to_bit(value, bit) && bits_.contains(bit);
    }

    constexpr bool empty() const noexcept { return bits_.empty(); }
    constexpr void clear() noexcept { bits_.clear(); }
    constexpr Underlying value() const noexcept { return bits_.value(); }

private:
    static constexpr bool to_bit(Enum value, std::size_t& bit) noexcept {
        using Raw = typename std::underlying_type<Enum>::type;
        using UnsignedRaw = typename std::make_unsigned<Raw>::type;
        const Raw raw = static_cast<Raw>(value);
        if (std::is_signed<Raw>::value && raw < 0) return false;
        const UnsignedRaw unsigned_raw = static_cast<UnsignedRaw>(raw);
        if (std::numeric_limits<UnsignedRaw>::digits > std::numeric_limits<std::size_t>::digits &&
            unsigned_raw > static_cast<UnsignedRaw>(std::numeric_limits<std::size_t>::max()))
            return false;
        bit = static_cast<std::size_t>(unsigned_raw);
        return true;
    }

    BitMask<Underlying> bits_;
};

}  // namespace mujoco_simulation
