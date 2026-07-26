#pragma once

#include <type_traits>

template <typename Enum>
constexpr std::underlying_type_t<Enum> to_integer(Enum value) noexcept {
  return static_cast<std::underlying_type_t<Enum>>(value);
}

template <typename Enum, typename Integer>
constexpr Enum to_enum(Integer integer) noexcept {
  static_assert(std::is_integral_v<Integer>, "Integer must be integral");
  return static_cast<Enum>(integer);
}
