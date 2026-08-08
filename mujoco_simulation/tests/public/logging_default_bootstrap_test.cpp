#include <array>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

#include <mujoco_simulation/log/logging.hpp>

namespace {
int bootstrap_child() {
    SIM_INFO << "invalid default target";
    mujoco_simulation::logging::flush();
    return 0;
}

bool run_invalid_target_case(const char* executable, const char* target_kind) {
    const std::string executable_path = std::filesystem::absolute(executable).string();
    std::array<char, 64> pattern{};
    std::snprintf(pattern.data(), pattern.size(), "/tmp/mujoco_logging_default_XXXXXX");
    char* const directory = ::mkdtemp(pattern.data());
    if (directory == nullptr) return false;
    const pid_t child = ::fork();
    if (child == 0) {
        if (::chdir(directory) != 0) _exit(127);
        if (std::string(target_kind) == "directory") {
            if (::mkdir("simulate.log", 0700) != 0) _exit(127);
        } else if (std::string(target_kind) == "fifo") {
            if (::mkfifo("simulate.log", 0600) != 0) _exit(127);
        } else if (::symlink("missing.log", "simulate.log") != 0) {
            _exit(127);
        }
        ::execl(
            executable_path.c_str(), executable_path.c_str(), "--invalid-target-child", nullptr);
        _exit(127);
    }
    int status = 0;
    const bool passed = child > 0 && ::waitpid(child, &status, 0) == child && WIFEXITED(status) &&
                        WEXITSTATUS(status) == 0;
    std::error_code ignored;
    std::filesystem::remove_all(directory, ignored);
    return passed;
}
}  // namespace

int main(int argc, char** argv) {
    if (argc == 2 && std::string(argv[1]) == "--invalid-target-child") return bootstrap_child();
    const char* const file_name = "simulate.log";
    std::remove(file_name);
    SIM_INFO << "default bootstrap";
    mujoco_simulation::logging::flush();

    std::ifstream file(file_name);
    std::string line;
    std::getline(file, line);
    std::string extra;
    const bool has_extra = static_cast<bool>(std::getline(file, extra));
    std::remove(file_name);
    if (line.find("[INFO]") == std::string::npos ||
        line.find("default bootstrap") == std::string::npos || has_extra)
        return 1;
    return run_invalid_target_case(argv[0], "directory") &&
                   run_invalid_target_case(argv[0], "fifo") &&
                   run_invalid_target_case(argv[0], "dangling")
               ? 0
               : 2;
}
