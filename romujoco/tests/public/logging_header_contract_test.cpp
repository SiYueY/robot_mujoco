#define ERROR 0
#define DEBUG 1
#define INFO 2

#include <cstddef>
#include <cstdio>
#include <fstream>
#include <string>
#include <type_traits>

#include <romujoco/log/logging.hpp>

int main() {
    using romujoco::logging::Level;
    using romujoco::logging::Policy;

    static_assert(std::is_same_v<std::underlying_type_t<Level>, std::uint8_t>);
    static_assert(static_cast<std::uint8_t>(Level::Debug) == 10);
    static_assert(static_cast<std::uint8_t>(Level::Info) == 20);
    static_assert(static_cast<std::uint8_t>(Level::Warn) == 30);
    static_assert(static_cast<std::uint8_t>(Level::Error) == 40);
    static_assert(static_cast<std::uint8_t>(Level::Fatal) == 50);
    static_assert(static_cast<std::uint8_t>(Level::Off) == 255);
    static_assert(sizeof(romujoco::logging::SourceLocation) == 24);
    static_assert(alignof(romujoco::logging::SourceLocation) == 8);
    static_assert(offsetof(romujoco::logging::SourceLocation, file) == 0);
    static_assert(offsetof(romujoco::logging::SourceLocation, function) == 8);
    static_assert(offsetof(romujoco::logging::SourceLocation, line) == 16);
    static_assert(sizeof(Policy) == 48);
    static_assert(alignof(Policy) == 8);
    static_assert(offsetof(Policy, level) == 0);
    static_assert(offsetof(Policy, console_enabled) == 1);
    static_assert(offsetof(Policy, file_enabled) == 2);
    static_assert(offsetof(Policy, file_path) == 8);
    static_assert(offsetof(Policy, colored_console) == 40);
    static_assert(offsetof(Policy, show_source_location) == 41);

    Policy policy;
    policy.level = Level::Debug;
    policy.console_enabled = false;
    policy.file_enabled = false;
    if (!romujoco::logging::configure(policy)) return 1;

    int evaluated = 0;
    SIM_DEBUG << ++evaluated;
    SIM_LOG(DEBUG) << ++evaluated;
    SIM_INFO << ++evaluated;
    SIM_LOG(INFO) << ++evaluated;
    SIM_WARN << ++evaluated;
    SIM_LOG(WARN) << ++evaluated;
    SIM_ERROR << ++evaluated;
    SIM_LOG(ERROR) << ++evaluated;
    SIM_FATAL << ++evaluated;
    SIM_LOG(FATAL) << ++evaluated;
    if (true)
        SIM_INFO << ++evaluated;
    else
        return 9;
    if (evaluated != 0) return 2;

    if (!romujoco::logging::set_level(Level::Off)) return 3;
    if (romujoco::logging::is_enabled(Level::Info)) return 4;
    if (romujoco::logging::is_enabled(Level::Off)) return 5;
    if (romujoco::logging::to_string(Level::Warn) != std::string("Warn")) return 6;

    const char* const file_name = "logging_header_contract.log";
    std::remove(file_name);
    policy.level = Level::Info;
    policy.file_enabled = true;
    policy.file_path = file_name;
    if (!romujoco::logging::configure(policy)) return 7;
    SIM_INFO;
    SIM_LOG(INFO) << "embedded\nline";
    romujoco::logging::flush();
    std::ifstream log(file_name);
    std::string empty_line;
    std::string line;
    std::getline(log, empty_line);
    std::getline(log, line);
    std::string extra_line;
    const bool has_extra_line = static_cast<bool>(std::getline(log, extra_line));
    std::remove(file_name);
    if (empty_line.find("[INFO]") == std::string::npos ||
        line.find("[INFO]") == std::string::npos ||
        line.find("embedded\\nline") == std::string::npos || has_extra_line)
        return 8;
    return 0;
}
