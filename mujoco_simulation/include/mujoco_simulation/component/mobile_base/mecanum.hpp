#pragma once

#include <cmath>
#include <cstddef>
#include <stdexcept>

#include "mujoco_simulation/common/enum.hpp"
#include "mujoco_simulation/common/math.hpp"

/**
 * @file mecanum.hpp
 * @brief 标准四轮麦克纳姆底盘运动学。
 *
 * @details 四个 45 度滚子麦克纳姆轮通过轮速差组合出底盘前进、横移和原地旋转
 * 运动，因此可在平面内实现 x、y 和 yaw 三个自由度的全向运动。本文件提供
 * 标准四轮麦克纳姆正、逆运动学：逆运动学将底盘速度映射为目标轮角速度，正运动学
 * 则由实际轮角速度恢复底盘速度。
 *
 * 几何模型假定四个轮中心位于矩形四角，wheel_base 和 track_width 分别为前后、
 * 左右轮中心距离。轮序固定为左前、右前、左后、右后；轮关节正方向必须由 MJCF
 * 统一为“四轮正转时底盘沿 x 轴正方向前进”。
 *
 * @note 设 r 为 wheel_radius，k = wheel_base / 2 + track_width / 2，底盘速度为
 * [vx, vy, wz]，四轮角速度为 [w_fl, w_fr, w_rl, w_rr]，逆运动学为：
 * @code
 * [w_fl]   1 / r [ 1 -1 -k ] [vx]
 * [w_fr] =       [ 1  1  k ] [vy]
 * [w_rl]         [ 1  1 -k ] [wz]
 * [w_rr]         [ 1 -1  k ]
 * @endcode
 * 正运动学为：
 * @code
 * vx = r / 4 * (w_fl + w_fr + w_rl + w_rr)
 * vy = r / 4 * (-w_fl + w_fr + w_rl - w_rr)
 * wz = r / (4 * k) * (-w_fl + w_fr - w_rl + w_rr)
 * @endcode
 */
namespace mujoco_simulation {

enum class MecanumWheelIndex : std::size_t {
  FrontLeft = 0,
  FrontRight,
  RearLeft,
  RearRight,
  Count,
};

inline constexpr std::size_t MecanumWheelCount = static_cast<std::size_t>(MecanumWheelIndex::Count);

/** @brief 麦克纳姆底盘的几何配置。 */
struct MecanumInfo {
  double wheel_radius{0.0};  ///< 车轮半径，单位：m。
  double wheel_base{0.0};    ///< 轴距，即前、后轮中心的纵向距离，单位：m。
  double track_width{0.0};   ///< 轮距，即左、右轮中心的横向距离，单位：m。
};

/**
 * @brief 标准四轮麦克纳姆底盘正、逆运动学。
 *
 * @details 坐标系遵循 ROS REP-103：x 轴向前、y 轴向左、z 轴向上；角速度正方向
 * 遵循右手定则，因此 base_angular.z 为正表示底盘逆时针旋转。
 *
 * @note 所有 Vector4d 轮角速度均固定采用左前、右前、左后、右后的顺序。MJCF
 * 必须将四个轮关节正方向统一，使四轮正角速度对应底盘沿 x 轴正方向前进。
 */
class MecanumKinematics {
 public:
  /**
   * @brief 根据几何配置构造麦克纳姆运动学模型。
   * @param info 底盘几何参数。
   * @throws std::invalid_argument 任一几何参数不是有限正数时抛出。
   */
  explicit MecanumKinematics(const MecanumInfo& info)
      : wheel_radius_(info.wheel_radius),
        rotation_coefficient_((info.wheel_base + info.track_width) * 0.5) {
    if (!std::isfinite(wheel_radius_) || wheel_radius_ <= 0.0) {
      throw std::invalid_argument("mecanum wheel_radius must be finite and positive");
    }
    if (!std::isfinite(info.wheel_base) || info.wheel_base <= 0.0) {
      throw std::invalid_argument("mecanum wheel_base must be finite and positive");
    }
    if (!std::isfinite(info.track_width) || info.track_width <= 0.0) {
      throw std::invalid_argument("mecanum track_width must be finite and positive");
    }
    if (!std::isfinite(rotation_coefficient_) || rotation_coefficient_ <= 0.0) {
      throw std::invalid_argument("mecanum rotation coefficient must be finite and positive");
    }
  }

  /**
   * @brief 逆运动学：将底盘平面速度转换为四轮目标角速度。
   * @param base_linear 底盘线速度，仅使用 x、y 分量，单位：m/s。
   * @param base_angular 底盘角速度，仅使用 z 分量，单位：rad/s。
   * @param[out] wheel_angular 左前、右前、左后、右后轮角速度，单位：rad/s。
   */
  void inverse(const Vector3d& base_linear, const Vector3d& base_angular,
               Vector4d& wheel_angular) const noexcept {
    const double forward = base_linear[0];
    const double lateral = base_linear[1];
    const double yaw = rotation_coefficient_ * base_angular[2];

    const double front_left = (forward - lateral - yaw) / wheel_radius_;
    const double front_right = (forward + lateral + yaw) / wheel_radius_;
    const double rear_left = (forward + lateral - yaw) / wheel_radius_;
    const double rear_right = (forward - lateral + yaw) / wheel_radius_;
    wheel_angular = {front_left, front_right, rear_left, rear_right};
  }

  /**
   * @brief 正运动学：由四轮角速度恢复底盘平面速度。
   * @param wheel_angular 左前、右前、左后、右后轮角速度，单位：rad/s。
   * @param[out] base_linear 底盘线速度，x、y 分量单位为 m/s，其余分量为零。
   * @param[out] base_angular 底盘角速度，z 分量单位为 rad/s，其余分量为零。
   */
  void forward(const Vector4d& wheel_angular, Vector3d& base_linear,
               Vector3d& base_angular) const noexcept {
    const double front_left = wheel_angular[to_integer(MecanumWheelIndex::FrontLeft)];
    const double front_right = wheel_angular[to_integer(MecanumWheelIndex::FrontRight)];
    const double rear_left = wheel_angular[to_integer(MecanumWheelIndex::RearLeft)];
    const double rear_right = wheel_angular[to_integer(MecanumWheelIndex::RearRight)];

    constexpr double kQuarter = 0.25;
    const double forward =
        wheel_radius_ * (front_left + front_right + rear_left + rear_right) * kQuarter;
    const double lateral =
        wheel_radius_ * (-front_left + front_right + rear_left - rear_right) * kQuarter;
    const double yaw = wheel_radius_ * (-front_left + front_right - rear_left + rear_right) /
                       (4.0 * rotation_coefficient_);

    base_linear = {forward, lateral, 0.0};
    base_angular = {0.0, 0.0, yaw};
  }

 private:
  double wheel_radius_;  ///< 车轮半径，单位：m。
  /// 麦克纳姆公式的 yaw 几何系数：(wheel_base + track_width) / 2，单位：m。
  double rotation_coefficient_;
};

}  // namespace mujoco_simulation
