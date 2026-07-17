#pragma once

// This header is only available to mujoco_simulation build targets. The
// Easylogging++ sources are intentionally not installed as part of the public
// package interface.
#include "easylogging++.h"

// 统一日志前缀；以下宏保持 Easylogging++ 的流式用法：
//   LOG_INFO << "simulation initialized";
//   LOG_WARNING << "fallback configuration is active";
//   LOG_ERROR << "failed to bind joint " << joint_name;
//   LOG_DEBUG << "step=" << step_count;
#define LOG_INFO LOG(INFO) << "[mujoco_simulation] "
#define LOG_WARNING LOG(WARNING) << "[mujoco_simulation] "
#define LOG_ERROR LOG(ERROR) << "[mujoco_simulation] "
#define LOG_DEBUG LOG(DEBUG) << "[mujoco_simulation] "

// FATAL 日志写出后会终止进程，仅用于不可恢复的错误。
#define LOG_FATAL LOG(FATAL) << "[mujoco_simulation] "
