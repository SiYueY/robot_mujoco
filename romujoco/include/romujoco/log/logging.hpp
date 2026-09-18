#pragma once

#include <cstdint>
#include <exception>
#include <ostream>
#include <sstream>
#include <string>

#include <romujoco/export.hpp>

// These names are reserved by this header.  SIM_LOG_COMPILED_LEVEL is the
// single supported pre-include customization point.
#if defined(SIM_LOG) || defined(SIM_DEBUG) || defined(SIM_INFO) || defined(SIM_WARN) ||           \
    defined(SIM_ERROR) || defined(SIM_FATAL) || defined(SIM_LOG_LEVEL_DEBUG) ||                   \
    defined(SIM_LOG_LEVEL_INFO) || defined(SIM_LOG_LEVEL_WARN) || defined(SIM_LOG_LEVEL_ERROR) || \
    defined(SIM_LOG_LEVEL_FATAL) || defined(SIM_LOG_LEVEL_NONE) || defined(SIMULATE_LOG) ||       \
    defined(SIMULATE_LOG_COMPILE_DISABLED) || defined(SIMULATE_LOG_DEBUG) ||                      \
    defined(SIMULATE_LOG_INFO) || defined(SIMULATE_LOG_WARN) || defined(SIMULATE_LOG_ERROR) ||    \
    defined(SIMULATE_LOG_FATAL)
#error "romujoco logging macro name collision"
#endif

#define SIM_LOG_LEVEL_DEBUG 1
#define SIM_LOG_LEVEL_INFO 2
#define SIM_LOG_LEVEL_WARN 3
#define SIM_LOG_LEVEL_ERROR 4
#define SIM_LOG_LEVEL_FATAL 5
#define SIM_LOG_LEVEL_NONE 6

#ifndef SIM_LOG_COMPILED_LEVEL
#ifdef NDEBUG
#define SIM_LOG_COMPILED_LEVEL SIM_LOG_LEVEL_INFO
#else
#define SIM_LOG_COMPILED_LEVEL SIM_LOG_LEVEL_DEBUG
#endif
#endif

#if SIM_LOG_COMPILED_LEVEL < SIM_LOG_LEVEL_DEBUG || SIM_LOG_COMPILED_LEVEL > SIM_LOG_LEVEL_NONE
#error "invalid SIM_LOG_COMPILED_LEVEL"
#endif

namespace romujoco::logging {

enum class Level : std::uint8_t {
    Debug = 10,
    Info = 20,
    Warn = 30,
    Error = 40,
    Fatal = 50,
    Off = 255,
};

struct SourceLocation {
    const char* file;
    const char* function;
    std::int32_t line;
};

struct Policy {
    Level level{Level::Info};
    bool console_enabled{true};
    bool file_enabled{true};
    std::string file_path{"simulate.log"};
    bool colored_console{true};
    bool show_source_location{true};
};

ROMUJOCO_PUBLIC bool configure(const Policy& policy) noexcept;
ROMUJOCO_PUBLIC bool set_level(Level level) noexcept;
ROMUJOCO_PUBLIC Level level() noexcept;
ROMUJOCO_PUBLIC bool is_enabled(Level level) noexcept;
ROMUJOCO_PUBLIC void flush() noexcept;
ROMUJOCO_PUBLIC const char* to_string(Level level) noexcept;

namespace impl {

// Macro-support implementation. These names are not a direct-use API.
ROMUJOCO_PUBLIC void commit_message(
    Level level, SourceLocation location, const char* message, std::size_t message_size) noexcept;

class LogMessageStream final {
public:
    class Expression final {
    public:
        // Adapts the stream expression to void for conditional log macros.
        void operator&(std::ostream&) const noexcept {}
    };

    LogMessageStream(Level level, SourceLocation location)
    : level_(level), location_(location), uncaught_exceptions_(std::uncaught_exceptions()) {}

    ~LogMessageStream() noexcept {
        if (std::uncaught_exceptions() != uncaught_exceptions_ || !stream_) return;
        try {
            const std::string message = stream_.str();
            commit_message(level_, location_, message.data(), message.size());
        } catch (...) {
        }
    }

    std::ostream& stream() noexcept { return stream_; }

private:
    Level level_;
    SourceLocation location_;
    int uncaught_exceptions_;
    std::ostringstream stream_;
};

}  // namespace impl
}  // namespace romujoco::logging

#define SIMULATE_LOG(level)                                                         \
    !::romujoco::logging::is_enabled(level)                                         \
        ? (void)0                                                                   \
        : ::romujoco::logging::impl::LogMessageStream::Expression{} &               \
              ::romujoco::logging::impl::LogMessageStream(                          \
                  level, {__FILE__, __func__, static_cast<std::int32_t>(__LINE__)}) \
                  .stream()

// The false branch is type-checked but never evaluated. Reusing
// LogMessageStream preserves stream syntax without a separate discard stream.
#define SIMULATE_LOG_COMPILE_DISABLED(level)                                         \
    true ? (void)0                                                                   \
         : ::romujoco::logging::impl::LogMessageStream::Expression{} &               \
               ::romujoco::logging::impl::LogMessageStream(                          \
                   level, {__FILE__, __func__, static_cast<std::int32_t>(__LINE__)}) \
                   .stream()

#if SIM_LOG_COMPILED_LEVEL <= SIM_LOG_LEVEL_DEBUG
#define SIMULATE_LOG_DEBUG SIMULATE_LOG(::romujoco::logging::Level::Debug)
#else
#define SIMULATE_LOG_DEBUG SIMULATE_LOG_COMPILE_DISABLED(::romujoco::logging::Level::Debug)
#endif

#if SIM_LOG_COMPILED_LEVEL <= SIM_LOG_LEVEL_INFO
#define SIMULATE_LOG_INFO SIMULATE_LOG(::romujoco::logging::Level::Info)
#else
#define SIMULATE_LOG_INFO SIMULATE_LOG_COMPILE_DISABLED(::romujoco::logging::Level::Info)
#endif

#if SIM_LOG_COMPILED_LEVEL <= SIM_LOG_LEVEL_WARN
#define SIMULATE_LOG_WARN SIMULATE_LOG(::romujoco::logging::Level::Warn)
#else
#define SIMULATE_LOG_WARN SIMULATE_LOG_COMPILE_DISABLED(::romujoco::logging::Level::Warn)
#endif

#if SIM_LOG_COMPILED_LEVEL <= SIM_LOG_LEVEL_ERROR
#define SIMULATE_LOG_ERROR SIMULATE_LOG(::romujoco::logging::Level::Error)
#else
#define SIMULATE_LOG_ERROR SIMULATE_LOG_COMPILE_DISABLED(::romujoco::logging::Level::Error)
#endif

#if SIM_LOG_COMPILED_LEVEL <= SIM_LOG_LEVEL_FATAL
#define SIMULATE_LOG_FATAL SIMULATE_LOG(::romujoco::logging::Level::Fatal)
#else
#define SIMULATE_LOG_FATAL SIMULATE_LOG_COMPILE_DISABLED(::romujoco::logging::Level::Fatal)
#endif

// Do not add an indirection before token pasting: it protects these tokens
// from hostile ERROR/DEBUG/INFO macro definitions in consumer code.
#define SIM_LOG(level_token) SIMULATE_LOG_##level_token
#define SIM_DEBUG SIMULATE_LOG_DEBUG
#define SIM_INFO SIMULATE_LOG_INFO
#define SIM_WARN SIMULATE_LOG_WARN
#define SIM_ERROR SIMULATE_LOG_ERROR
#define SIM_FATAL SIMULATE_LOG_FATAL
