#include "log/impl/logger.hpp"

#include <chrono>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <type_traits>

#include <unistd.h>

namespace mujoco_simulation::logging::impl {
namespace {
constexpr std::uint32_t kLevelMask = 0xff;
constexpr std::uint32_t kConsoleBit = 1u << 8;
constexpr std::uint32_t kFileBit = 1u << 9;
constexpr std::uint32_t kSourceBit = 1u << 10;
constexpr std::size_t kMaximumLocationLength = 4096;
constexpr auto kFallbackInterval = std::chrono::seconds{1};

static_assert(std::is_nothrow_move_assignable_v<Policy>);

std::uint32_t make_runtime_policy(
    const Level level, const SinkMask sinks, const bool show_source) noexcept {
    std::uint32_t policy = static_cast<std::uint32_t>(level);
    if (has_sink(sinks, SinkMask::Console)) policy |= kConsoleBit;
    if (has_sink(sinks, SinkMask::File)) policy |= kFileBit;
    if (show_source) policy |= kSourceBit;
    return policy;
}

Level runtime_level(const std::uint32_t policy) noexcept {
    return static_cast<Level>(policy & kLevelMask);
}

SinkMask runtime_sinks(const std::uint32_t policy) noexcept {
    SinkMask sinks = SinkMask::None;
    if ((policy & kConsoleBit) != 0) sinks = sinks | SinkMask::Console;
    if ((policy & kFileBit) != 0) sinks = sinks | SinkMask::File;
    return sinks;
}

bool runtime_shows_source(const std::uint32_t policy) noexcept {
    return (policy & kSourceBit) != 0;
}

std::string sanitize(const char* bytes, const std::size_t size) {
    static constexpr char hex[] = "0123456789ABCDEF";
    std::string result;
    result.reserve(size);
    for (std::size_t index = 0; index < size; ++index) {
        const auto value = static_cast<unsigned char>(bytes[index]);
        switch (value) {
            case '\0':
                result += "\\0";
                break;
            case '\t':
                result += "\\t";
                break;
            case '\n':
                result += "\\n";
                break;
            case '\r':
                result += "\\r";
                break;
            case 0x1b:
                result += "\\x1B";
                break;
            default:
                if (value < 0x20 || value == 0x7f) {
                    result += "\\x";
                    result += hex[value >> 4];
                    result += hex[value & 0x0f];
                } else {
                    result += static_cast<char>(value);
                }
        }
    }
    return result;
}

std::string bounded_c_string(const char* value) {
    if (value == nullptr) return {};
    std::size_t size = 0;
    while (size < kMaximumLocationLength && value[size] != '\0') ++size;
    if (size == kMaximumLocationLength) throw std::length_error("source location is too long");
    return std::string(value, size);
}

const char* severity_name(const Level level) noexcept {
    switch (level) {
        case Level::Debug:
            return "DEBUG";
        case Level::Info:
            return "INFO";
        case Level::Warn:
            return "WARN";
        case Level::Error:
            return "ERROR";
        case Level::Fatal:
            return "FATAL";
        case Level::Off:
            break;
    }
    return "UNKNOWN";
}

std::int64_t monotonic_nanoseconds() noexcept {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}
}  // namespace

Logger& logger() noexcept {
    static auto* instance = new Logger();
    return *instance;
}

bool Logger::is_valid_threshold(const Level value) const noexcept {
    return value == Level::Debug || value == Level::Info || value == Level::Warn ||
           value == Level::Error || value == Level::Fatal || value == Level::Off;
}

bool Logger::is_valid_severity(const Level value) const noexcept {
    return value == Level::Debug || value == Level::Info || value == Level::Warn ||
           value == Level::Error || value == Level::Fatal;
}

void Logger::update_runtime_policy(const SinkMask sinks) noexcept {
    active_sinks_ = sinks;
    runtime_policy_.store(
        make_runtime_policy(policy_.level, sinks, policy_.show_source_location),
        std::memory_order_release);
}

// Resolves and verifies an appendable regular-file target.
bool Logger::resolve_file_path(const Policy& policy, std::string* path) noexcept {
    if (!policy.file_enabled) {
        path->clear();
        return true;
    }
    try {
        if (policy.file_path.empty() || policy.file_path.find('\0') != std::string::npos)
            return false;
        const std::filesystem::path candidate = std::filesystem::absolute(policy.file_path);
        std::error_code error;
        if (!std::filesystem::is_directory(candidate.parent_path(), error) || error) return false;
        const auto link_status = std::filesystem::symlink_status(candidate, error);
        if (error && error != std::errc::no_such_file_or_directory) return false;
        if (!error && std::filesystem::is_symlink(link_status)) {
            const auto target_status = std::filesystem::status(candidate, error);
            if (error || !std::filesystem::is_regular_file(target_status)) return false;
        } else if (
            !error && std::filesystem::exists(link_status) &&
            !std::filesystem::is_regular_file(link_status)) {
            return false;
        }
        std::FILE* probe = std::fopen(candidate.c_str(), "ab");
        if (probe == nullptr) return false;
        std::fclose(probe);
        *path = candidate.string();
        return true;
    } catch (...) {
        return false;
    }
}

bool Logger::activate_prepared_policy(
    Policy&& policy, const SinkMask sinks, const std::string& file_path) noexcept {
    std::lock_guard<std::mutex> adapter_lock(adapter_mutex_);
    if (!adapter_.configure(sinks, file_path, policy.colored_console)) return false;
    policy_ = std::move(policy);
    update_runtime_policy(sinks);
    return true;
}

bool Logger::activate_policy(const Policy& policy) noexcept {
    if (!is_valid_threshold(policy.level)) return false;
    std::string file_path;
    Policy prepared_policy;
    try {
        prepared_policy = policy;
    } catch (...) {
        return false;
    }
    if (!resolve_file_path(prepared_policy, &file_path)) return false;
    SinkMask sinks = SinkMask::None;
    if (prepared_policy.console_enabled) sinks = sinks | SinkMask::Console;
    if (prepared_policy.file_enabled) sinks = sinks | SinkMask::File;
    try {
        std::lock_guard<std::mutex> policy_lock(policy_mutex_);
        return activate_prepared_policy(std::move(prepared_policy), sinks, file_path);
    } catch (...) {
        return false;
    }
}

bool Logger::configure(const Policy& policy) noexcept {
    const bool activated = activate_policy(policy);
    if (activated) {
        allow_default_configuration_.store(false, std::memory_order_release);
    }
    return activated;
}

bool Logger::activate_default_policy() noexcept {
    if (runtime_sinks(runtime_policy_.load(std::memory_order_acquire)) != SinkMask::None)
        return true;
    bool expected = true;
    if (!allow_default_configuration_.compare_exchange_strong(
            expected, false, std::memory_order_acq_rel)) {
        return false;
    }
    try {
        std::lock_guard<std::mutex> lock(policy_mutex_);
        if (active_sinks_ != SinkMask::None) return true;
        Policy prepared_policy = policy_;
        std::string file_path;
        if (!resolve_file_path(prepared_policy, &file_path)) return false;
        SinkMask sinks = SinkMask::None;
        if (prepared_policy.console_enabled) sinks = sinks | SinkMask::Console;
        if (prepared_policy.file_enabled) sinks = sinks | SinkMask::File;
        return activate_prepared_policy(std::move(prepared_policy), sinks, file_path);
    } catch (...) {
        return false;
    }
}

bool Logger::set_level(const Level value) noexcept {
    if (!is_valid_threshold(value)) return false;
    try {
        std::lock_guard<std::mutex> lock(policy_mutex_);
        policy_.level = value;
        update_runtime_policy(active_sinks_);
        return true;
    } catch (...) {
        return false;
    }
}

Level Logger::level() noexcept {
    return runtime_level(runtime_policy_.load(std::memory_order_acquire));
}

bool Logger::is_enabled(const Level value) noexcept {
    if (!is_valid_severity(value)) return false;
    const std::uint32_t policy = runtime_policy_.load(std::memory_order_acquire);
    if (value < runtime_level(policy)) return false;
    if (runtime_sinks(policy) != SinkMask::None) return true;
    return allow_default_configuration_.load(std::memory_order_acquire);
}

std::string Logger::format_record(
    const Level value, const SourceLocation location, const char* message,
    const std::size_t size) noexcept {
    try {
        const auto now = std::chrono::system_clock::now();
        const auto millis =
            std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
        const std::time_t time = std::chrono::system_clock::to_time_t(now);
        std::tm local{};
        localtime_r(&time, &local);
        std::ostringstream output;
        output << std::put_time(&local, "%Y-%m-%d %H:%M:%S") << '.' << std::setw(3)
               << std::setfill('0') << millis.count() << " [" << severity_name(value) << "] ["
               << std::this_thread::get_id() << ']';
        if (runtime_shows_source(runtime_policy_.load(std::memory_order_acquire))) {
            const std::filesystem::path file_path(bounded_c_string(location.file));
            const std::string file_name = file_path.filename().string();
            const std::string function_name = bounded_c_string(location.function);
            output << " [" << sanitize(file_name.c_str(), file_name.size()) << ':' << location.line
                   << ' ' << sanitize(function_name.c_str(), function_name.size()) << ']';
        }
        output << ' ' << sanitize(message == nullptr ? "" : message, size);
        return output.str();
    } catch (...) {
        return {};
    }
}

void Logger::commit(
    const Level value, const SourceLocation location, const char* message,
    const std::size_t size) noexcept {
    if (!is_valid_severity(value) || (message == nullptr && size != 0)) return;
    if (!is_enabled(value)) return;
    if (runtime_sinks(runtime_policy_.load(std::memory_order_acquire)) == SinkMask::None &&
        !activate_default_policy()) {
        return;
    }
    SinkMask requested = SinkMask::None;
    SinkMask written = SinkMask::None;
    try {
        std::lock_guard<std::mutex> lock(adapter_mutex_);
        const std::uint32_t policy = runtime_policy_.load(std::memory_order_acquire);
        requested = runtime_sinks(policy);
        if (value < runtime_level(policy) || requested == SinkMask::None) return;
        const std::string record = format_record(value, location, message, size);
        if (record.empty()) return;
        written = adapter_.write(value, record.data(), record.size());
        if (value == Level::Fatal && written != SinkMask::None) {
            written = adapter_.flush(written);
        }
    } catch (...) {
    }
    if (written != requested) {
        write_failure_message();
    }
}

void Logger::flush() noexcept {
    SinkMask requested = SinkMask::None;
    SinkMask flushed = SinkMask::None;
    try {
        std::lock_guard<std::mutex> lock(adapter_mutex_);
        requested = runtime_sinks(runtime_policy_.load(std::memory_order_acquire));
        if (requested == SinkMask::None) return;
        flushed = adapter_.flush(requested);
    } catch (...) {
    }
    if (flushed != requested) write_failure_message();
}

void Logger::write_failure_message() noexcept {
    static std::atomic<std::int64_t> last_report{std::numeric_limits<std::int64_t>::min()};
    const std::int64_t now = monotonic_nanoseconds();
    std::int64_t previous = last_report.load(std::memory_order_relaxed);
    if ((previous != std::numeric_limits<std::int64_t>::min() && now >= previous &&
         now - previous <
             std::chrono::duration_cast<std::chrono::nanoseconds>(kFallbackInterval).count()) ||
        !last_report.compare_exchange_strong(previous, now, std::memory_order_relaxed))
        return;
    static constexpr char message[] = "mujoco_simulation logging: adapter sink failure\n";
    (void)::write(STDERR_FILENO, message, sizeof(message) - 1);
}

}  // namespace mujoco_simulation::logging::impl
