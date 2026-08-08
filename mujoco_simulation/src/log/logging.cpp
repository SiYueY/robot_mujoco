#include "log/logging.hpp"

#include "log/impl/logger.hpp"

namespace mujoco_simulation::logging {

bool configure(const Policy& policy) noexcept { return impl::logger().configure(policy); }
bool set_level(const Level value) noexcept { return impl::logger().set_level(value); }
Level level() noexcept { return impl::logger().level(); }
bool is_enabled(const Level value) noexcept { return impl::logger().is_enabled(value); }
void flush() noexcept { impl::logger().flush(); }

const char* to_string(const Level value) noexcept {
    switch (value) {
        case Level::Debug:
            return "Debug";
        case Level::Info:
            return "Info";
        case Level::Warn:
            return "Warn";
        case Level::Error:
            return "Error";
        case Level::Fatal:
            return "Fatal";
        case Level::Off:
            return "Off";
    }
    return "Unknown";
}

namespace impl {
void commit_message(
    const Level value, const SourceLocation location, const char* message,
    const std::size_t size) noexcept {
    logger().commit(value, location, message, size);
}
}  // namespace impl
}  // namespace mujoco_simulation::logging
