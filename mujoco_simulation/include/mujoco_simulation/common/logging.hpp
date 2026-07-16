#pragma once

#include <string>
#include <string_view>

// This header is only available to mujoco_simulation build targets. The
// Easylogging++ sources are intentionally not installed as part of the public
// package interface.
#include "easylogging++.h"

namespace mujoco_simulation {

/**
 * @brief 常用 Easylogging++ 日志 API。
 *
 * - `LOG(INFO) << message`：记录一般运行信息。
 * - `LOG(WARNING) << message`：记录可恢复的异常或风险。
 * - `LOG(ERROR) << message`：记录当前操作失败，但不终止进程。
 * - `LOG(DEBUG) << message`：记录调试信息。
 * - `LOG_IF(condition, LEVEL) << message`：仅在条件为真时记录指定等级日志。
 * - `VLOG(level) << message`：记录可按 verbose level 控制的诊断信息。
 *
 * `LOG(FATAL)` 会在写入日志后终止进程，不应用于可通过返回值报告的运行时错误。
 * Easylogging++ 已由 mujoco_simulation 的内部实现统一初始化；调用方无需调用
 * `INITIALIZE_EASYLOGGINGPP` 或 `START_EASYLOGGINGPP`。
 */

/**
 * @brief 记录组件错误并返回 false，适用于直接返回 bool 的错误分支。
 *
 * @param where 失败的类和函数名。
 * @param message 面向日志的错误说明。
 * @return 始终为 false。
 */
inline bool log_component_error(std::string_view where, std::string_view message) {
  LOG(ERROR) << "[mujoco_simulation][component] " << std::string(where) << ": "
             << std::string(message);
  return false;
}

}  // namespace mujoco_simulation
