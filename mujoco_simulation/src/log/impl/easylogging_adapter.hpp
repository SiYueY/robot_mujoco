#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include "log/logging.hpp"

namespace mujoco_simulation::logging::impl {

enum class SinkMask : std::uint8_t {
    None = 0,
    Console = 1 << 0,
    File = 1 << 1,
};

constexpr SinkMask operator|(const SinkMask lhs, const SinkMask rhs) noexcept {
    return static_cast<SinkMask>(static_cast<std::uint8_t>(lhs) | static_cast<std::uint8_t>(rhs));
}
constexpr SinkMask operator&(const SinkMask lhs, const SinkMask rhs) noexcept {
    return static_cast<SinkMask>(static_cast<std::uint8_t>(lhs) & static_cast<std::uint8_t>(rhs));
}
constexpr bool has_sink(const SinkMask sinks, const SinkMask sink) noexcept {
    return (sinks & sink) != SinkMask::None;
}

class EasyloggingAdapter final {
public:
    bool configure(SinkMask sinks, const std::string& file_path, bool colored_console) noexcept;
    SinkMask write(Level level, const char* record, std::size_t size) noexcept;
    SinkMask flush(SinkMask sinks) noexcept;

private:
    SinkMask active_sinks_{SinkMask::None};
    bool colored_console_{true};
    std::string active_logger_id_;
};

}  // namespace mujoco_simulation::logging::impl
