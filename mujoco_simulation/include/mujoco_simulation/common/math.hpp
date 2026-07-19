#pragma once

#include <array>
#include <cmath>
#include <limits>
#include <type_traits>
#include <vector>

namespace mujoco_simulation {

inline constexpr double Pi = 3.14159265358979323846;

// Vector
using Vector3d = std::array<double, 3>;
using Vector4d = std::array<double, 4>;
using Vector9d = std::array<double, 9>;
using Vector12d = std::array<double, 12>;
using Vector36d = std::array<double, 36>;
using VectorXd = std::vector<double>;

// Matrix
using Matrix3d = std::array<std::array<double, 3>, 3>;
using Matrix4d = std::array<std::array<double, 4>, 4>;
using Matrix9d = std::array<std::array<double, 9>, 9>;
using Matrix12d = std::array<std::array<double, 12>, 12>;
using MatrixXd = std::vector<std::vector<double>>;

// Quaternion
using Quaterniond = Vector4d;

// 默认浮点误差
template <typename T>
inline constexpr T kDefaultEpsilon = static_cast<T>(1e-6);

// 自定义 constexpr 绝对值
template <typename T>
constexpr T abs(T x) {
  static_assert(std::is_signed<T>::value, "abs only supports signed arithmetic types");
  return x < 0 ? -x : x;
}

// Equal
template <typename T>
constexpr bool equal(T lhs, T rhs) {
  if constexpr (std::is_floating_point_v<T>) {
    return abs(lhs - rhs) <= kDefaultEpsilon<T>;
  } else {
    return lhs == rhs;
  }
}

template <typename T>
constexpr bool equal(T lhs, T rhs, T epsilon) {
  static_assert(std::is_floating_point_v<T>, "epsilon version only supports floating point");
  return abs(lhs - rhs) <= epsilon;
}

// Less
template <typename T>
constexpr bool less(T lhs, T rhs) {
  if constexpr (std::is_floating_point_v<T>) {
    // 若近似相等，则不小于
    if (equal(lhs, rhs)) return false;
    return lhs < rhs;
  } else {
    return lhs < rhs;
  }
}

template <typename T>
constexpr bool less(T lhs, T rhs, T epsilon) {
  static_assert(std::is_floating_point_v<T>, "epsilon only supports floating point");
  if (equal(lhs, rhs, epsilon)) return false;
  return lhs < rhs;
}

// Greater
template <typename T>
constexpr bool greater(T lhs, T rhs) {
  if constexpr (std::is_floating_point_v<T>) {
    // 若近似相等，则不大于
    if (equal(lhs, rhs)) return false;
    return lhs > rhs;
  } else {
    return lhs > rhs;
  }
}

template <typename T>
constexpr bool greater(T lhs, T rhs, T epsilon) {
  static_assert(std::is_floating_point_v<T>, "epsilon only supports floating point");
  if (equal(lhs, rhs, epsilon)) return false;
  return lhs > rhs;
}

}  // namespace mujoco_simulation
