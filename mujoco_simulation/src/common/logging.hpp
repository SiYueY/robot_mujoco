#pragma once

#include "easylogging++.h"

// 统一日志前缀和函数定位；ELPP_FUNC 在 GCC/Clang 下包含类方法签名。
// 以下宏保持 Easylogging++ 的流式用法：
//   LOG_INFO << "simulation initialized";
//   LOG_WARNING << "fallback configuration is active";
//   LOG_ERROR << "failed to bind joint " << joint_name;
//   LOG_DEBUG << "step=" << step;
#define MUJOCO_SIMULATION_LOG_CONTEXT                                          \
  "[mujoco_simulation] [" << ELPP_FUNC << "] "
#define LOG_INFO LOG(INFO) << MUJOCO_SIMULATION_LOG_CONTEXT
#define LOG_WARNING LOG(WARNING) << MUJOCO_SIMULATION_LOG_CONTEXT
#define LOG_ERROR LOG(ERROR) << MUJOCO_SIMULATION_LOG_CONTEXT
#define LOG_DEBUG LOG(DEBUG) << MUJOCO_SIMULATION_LOG_CONTEXT

// FATAL 日志写出后会终止进程，仅用于不可恢复的错误。
#define LOG_FATAL LOG(FATAL) << MUJOCO_SIMULATION_LOG_CONTEXT
