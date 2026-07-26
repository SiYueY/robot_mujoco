#pragma once

#include <limits>
#include <string>
namespace mujoco_simulation {

// 关节类型
enum class JointType {
  Revolute,  // 旋转
  Prismatic, // 平移
};

// 控制模式
enum class JointControlMode {
  Position, // 位置
  Velocity, // 速度
  Effort,   // 力矩
  Hybrid,   // 力位混合
};

// 限制
struct Limit {
  double min{-std::numeric_limits<double>::infinity()}; // 最小值
  double max{std::numeric_limits<double>::infinity()};  // 最大值
};

struct JointInfo {
  std::string joint_name;                    // 关节名称
  std::string actuator_name;                 // 执行器名称
  JointType joint_type{JointType::Revolute}; // 关节类型
  double position_stiffness{0.0};            // 位置刚度
  double position_damping{0.0};              // 位置阻尼
  double velocity_damping{0.0};              // 速度阻尼
  Limit position_limits;                     // 位置限制
  Limit velocity_limits;                     // 速度限制
  Limit effort_limits;                       // 力矩限制
  double update_rate{0.0};                   // 更新频率
};

// 关节指令
struct JointCommand {
  std::string joint_name;                          // 关节名称
  JointControlMode mode{JointControlMode::Effort}; // 控制模式
  double position{0.0};                            // 位置
  double velocity{0.0};                            // 速度
  double effort{0.0};                              // 力矩
  double stiffness{0.0};                           // 刚度
  double damping{0.0};                             // 阻尼
};

// 关节状态
struct JointState {
  std::string joint_name;                          // 关节名称
  double timestamp{0.0};                           // seconds
  JointControlMode mode{JointControlMode::Effort}; // 控制模式
  double position{0.0};                            // 位置
  double velocity{0.0};                            // 速度
  double effort{0.0};                              // 力矩
};

} // namespace mujoco_simulation
