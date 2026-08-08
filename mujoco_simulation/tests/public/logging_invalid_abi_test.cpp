#include <cstdio>
#include <filesystem>

#include <mujoco_simulation/log/logging.hpp>

int main() {
    namespace logging = mujoco_simulation::logging;
    const char* const default_path = "simulate.log";
    std::remove(default_path);

    logging::Policy policy;
    policy.level = static_cast<logging::Level>(123);
    policy.console_enabled = false;
    policy.file_enabled = false;
    if (logging::configure(policy)) return 1;
    if (logging::set_level(static_cast<logging::Level>(123))) return 2;
    if (logging::is_enabled(logging::Level::Off) ||
        logging::is_enabled(static_cast<logging::Level>(123)))
        return 3;
    return std::filesystem::exists(default_path) ? 4 : 0;
}
