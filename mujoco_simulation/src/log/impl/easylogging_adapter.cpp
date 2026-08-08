#include "log/impl/easylogging_adapter.hpp"

#include <atomic>
#include <cstdio>
#include <string>

#include <unistd.h>

#include "easylogging++.h"

INITIALIZE_EASYLOGGINGPP

namespace mujoco_simulation::logging::impl {
namespace {
constexpr char kLoggerIdPrefix[] = "mujoco_simulation.logging.";
std::atomic<std::uint64_t> next_logger_number{0};

std::string make_logger_id() {
    return std::string{kLoggerIdPrefix} +
           std::to_string(next_logger_number.fetch_add(1, std::memory_order_relaxed));
}

void unregister_logger(const std::string& logger_id) noexcept {
    if (logger_id.empty()) return;
    try {
        (void)el::Loggers::unregisterLogger(logger_id);
    } catch (...) {
    }
}

el::Level to_easylogging_level(const Level level) noexcept {
    switch (level) {
        case Level::Debug:
            return el::Level::Debug;
        case Level::Info:
            return el::Level::Info;
        case Level::Warn:
            return el::Level::Warning;
        case Level::Error:
            return el::Level::Error;
        case Level::Fatal:
            return el::Level::Fatal;
        case Level::Off:
            break;
    }
    return el::Level::Info;
}
}  // namespace

bool EasyloggingAdapter::configure(
    const SinkMask sinks, const std::string& file_path, const bool colored_console) noexcept {
    std::string candidate_id;
    try {
        candidate_id = make_logger_id();
        el::Configurations configurations;
        configurations.setToDefault();
        configurations.setGlobally(el::ConfigurationType::Enabled, "true");
        configurations.setGlobally(el::ConfigurationType::ToStandardOutput, "false");
        configurations.setGlobally(
            el::ConfigurationType::ToFile, has_sink(sinks, SinkMask::File) ? "true" : "false");
        configurations.setGlobally(el::ConfigurationType::Format, "%msg");
        configurations.setGlobally(el::ConfigurationType::LogFlushThreshold, "1");
        if (has_sink(sinks, SinkMask::File)) {
            configurations.setGlobally(el::ConfigurationType::Filename, file_path);
        }
        el::Loggers::addFlag(el::LoggingFlag::DisableApplicationAbortOnFatalLog);
        auto* logger = el::Loggers::reconfigureLogger(candidate_id, configurations);
        if (logger == nullptr) {
            unregister_logger(candidate_id);
            return false;
        }
        if (has_sink(sinks, SinkMask::File)) {
            auto* stream = logger == nullptr
                               ? nullptr
                               : logger->typedConfigurations()->fileStream(el::Level::Info);
            if (stream == nullptr || stream->fail()) {
                unregister_logger(candidate_id);
                return false;
            }
        }
        active_logger_id_.swap(candidate_id);
        active_sinks_ = sinks;
        colored_console_ = colored_console;
        unregister_logger(candidate_id);
        return true;
    } catch (...) {
        unregister_logger(candidate_id);
        return false;
    }
}

SinkMask EasyloggingAdapter::write(
    const Level level, const char* record, const std::size_t size) noexcept {
    SinkMask written = SinkMask::None;
    try {
        if (has_sink(active_sinks_, SinkMask::Console)) {
            const bool color = colored_console_ && ::isatty(STDERR_FILENO) == 1;
            const char* color_code = level == Level::Fatal   ? "\033[1;31m"
                                     : level == Level::Error ? "\033[31m"
                                     : level == Level::Warn  ? "\033[33m"
                                     : level == Level::Info  ? "\033[32m"
                                                             : "\033[36m";
            const bool console_ok =
                color ? std::fputs(color_code, stderr) != EOF &&
                            std::fwrite(record, 1, size, stderr) == size &&
                            std::fputs("\033[0m\n", stderr) != EOF && std::ferror(stderr) == 0
                      : std::fwrite(record, 1, size, stderr) == size &&
                            std::fputc('\n', stderr) != EOF && std::ferror(stderr) == 0;
            if (console_ok) written = written | SinkMask::Console;
        }
        if (has_sink(active_sinks_, SinkMask::File)) {
            const std::string text(record, size);
            el::base::Writer(
                to_easylogging_level(level), __FILE__, __LINE__, __func__,
                el::base::DispatchAction::NormalLog)
                    .construct(1, active_logger_id_.c_str())
                << text;
            auto* logger = el::Loggers::getLogger(active_logger_id_, false);
            auto* stream =
                logger == nullptr
                    ? nullptr
                    : logger->typedConfigurations()->fileStream(to_easylogging_level(level));
            if (stream != nullptr && !stream->fail()) written = written | SinkMask::File;
        }
    } catch (...) {
    }
    return written;
}

SinkMask EasyloggingAdapter::flush(const SinkMask sinks) noexcept {
    SinkMask flushed = SinkMask::None;
    try {
        if (has_sink(sinks, SinkMask::Console) && std::fflush(stderr) == 0) {
            flushed = flushed | SinkMask::Console;
        }
        if (has_sink(sinks, SinkMask::File)) {
            if (auto* logger = el::Loggers::getLogger(active_logger_id_, false)) {
                logger->flush();
                auto* stream = logger->typedConfigurations()->fileStream(el::Level::Info);
                if (stream != nullptr && !stream->fail()) flushed = flushed | SinkMask::File;
            }
        }
    } catch (...) {
    }
    return flushed;
}

}  // namespace mujoco_simulation::logging::impl
