#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>

#include "log/logging.hpp"

#include "log/impl/easylogging_adapter.hpp"

namespace mujoco_simulation::logging::impl {

class Logger final {
public:
    Logger() = default;

    bool configure(const Policy& policy) noexcept;
    bool set_level(Level level) noexcept;
    Level level() noexcept;
    bool is_enabled(Level level) noexcept;
    void flush() noexcept;
    void commit(
        Level level, SourceLocation location, const char* message, std::size_t size) noexcept;

private:
    bool activate_default_policy() noexcept;
    bool activate_policy(const Policy& policy) noexcept;
    bool activate_prepared_policy(
        Policy&& policy, SinkMask sinks, const std::string& file_path) noexcept;
    void update_runtime_policy(SinkMask sinks) noexcept;
    void write_failure_message() noexcept;
    bool is_valid_threshold(Level level) const noexcept;
    bool is_valid_severity(Level level) const noexcept;
    bool resolve_file_path(const Policy& policy, std::string* path) noexcept;
    std::string format_record(
        Level level, SourceLocation location, const char* message, std::size_t size) noexcept;

    std::mutex policy_mutex_;
    std::mutex adapter_mutex_;
    Policy policy_{};
    EasyloggingAdapter adapter_{};
    SinkMask active_sinks_{SinkMask::None};
    std::atomic<std::uint32_t> runtime_policy_{0};
    std::atomic<bool> allow_default_configuration_{true};
};

Logger& logger() noexcept;

}  // namespace mujoco_simulation::logging::impl
