#include <array>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits.h>
#include <string>

#include <sys/wait.h>
#include <unistd.h>

#include <mujoco_simulation/log/logging.hpp>

namespace {
int write_default_record() {
    SIM_INFO << "multi-process-record";
    mujoco_simulation::logging::flush();
    return 0;
}

bool run_writer(const char* executable) {
    const pid_t child = ::fork();
    if (child == 0) {
        ::execl(executable, executable, "--writer", nullptr);
        _exit(127);
    }
    if (child < 0) return false;
    int status = 0;
    return ::waitpid(child, &status, 0) == child && WIFEXITED(status) && WEXITSTATUS(status) == 0;
}
}  // namespace

int main(int argc, char** argv) {
    if (argc == 2 && std::string(argv[1]) == "--writer") return write_default_record();

    const char* const default_path = "simulate.log";
    std::remove(default_path);
    if (!run_writer(argv[0]) || !run_writer(argv[0])) return 1;
    std::ifstream default_file(default_path);
    std::string line;
    int records = 0;
    while (std::getline(default_file, line)) {
        if (line.find("multi-process-record") != std::string::npos) ++records;
    }
    std::remove(default_path);
    if (records != 2) return 2;

    const char* const copytruncate_path = "logging_copytruncate.log";
    std::remove(copytruncate_path);
    mujoco_simulation::logging::Policy policy;
    policy.console_enabled = false;
    policy.file_enabled = true;
    policy.file_path = copytruncate_path;
    if (!mujoco_simulation::logging::configure(policy)) return 3;
    SIM_INFO << "before-copytruncate";
    mujoco_simulation::logging::flush();
    if (::truncate(copytruncate_path, 0) != 0) return 4;
    SIM_INFO << "after-copytruncate";
    mujoco_simulation::logging::flush();
    std::ifstream truncated_file(copytruncate_path);
    std::string truncated;
    std::getline(truncated_file, truncated);
    std::remove(copytruncate_path);
    if (truncated.find("after-copytruncate") == std::string::npos) return 5;

    std::array<char, PATH_MAX> original_cwd{};
    if (::getcwd(original_cwd.data(), original_cwd.size()) == nullptr) return 6;
    std::array<char, 64> temporary_pattern{};
    std::snprintf(
        temporary_pattern.data(), temporary_pattern.size(), "/tmp/mujoco_logging_cwd_XXXXXX");
    const char* temporary_directory = ::mkdtemp(temporary_pattern.data());
    if (temporary_directory == nullptr) return 7;
    const std::filesystem::path first_directory =
        std::filesystem::path(temporary_directory) / "first";
    const std::filesystem::path second_directory =
        std::filesystem::path(temporary_directory) / "second";
    std::filesystem::create_directory(first_directory);
    std::filesystem::create_directory(second_directory);
    if (::chdir(first_directory.c_str()) != 0) return 8;
    policy.file_path = "relative.log";
    if (!mujoco_simulation::logging::configure(policy)) return 9;
    SIM_INFO << "first-cwd";
    if (::chdir(second_directory.c_str()) != 0) return 10;
    SIM_INFO << "after-chdir";
    mujoco_simulation::logging::flush();
    std::ifstream first_file(first_directory / "relative.log");
    std::string first_contents(
        (std::istreambuf_iterator<char>(first_file)), std::istreambuf_iterator<char>());
    if (first_contents.find("first-cwd") == std::string::npos ||
        first_contents.find("after-chdir") == std::string::npos ||
        std::filesystem::exists(second_directory / "relative.log"))
        return 11;
    if (!mujoco_simulation::logging::configure(policy)) return 12;
    SIM_INFO << "second-cwd";
    mujoco_simulation::logging::flush();
    std::ifstream second_file(second_directory / "relative.log");
    std::string second_contents(
        (std::istreambuf_iterator<char>(second_file)), std::istreambuf_iterator<char>());
    const bool correct_second_file = second_contents.find("second-cwd") != std::string::npos;
    (void)::chdir(original_cwd.data());

    const std::filesystem::path stable_path =
        std::filesystem::path(temporary_directory) / "stable.log";
    mujoco_simulation::logging::Policy stable_policy;
    stable_policy.console_enabled = false;
    stable_policy.file_enabled = true;
    stable_policy.file_path = stable_path.string();
    if (!correct_second_file || !mujoco_simulation::logging::configure(stable_policy)) return 13;
    SIM_INFO << "before-failed-configure";
    mujoco_simulation::logging::Policy invalid_policy = stable_policy;
    invalid_policy.console_enabled = true;
    invalid_policy.file_path =
        (std::filesystem::path(temporary_directory) / "missing" / "log").string();
    if (mujoco_simulation::logging::configure(invalid_policy)) return 14;
    SIM_INFO << "after-failed-configure";
    mujoco_simulation::logging::flush();
    std::ifstream stable_file(stable_path);
    std::string stable_contents(
        (std::istreambuf_iterator<char>(stable_file)), std::istreambuf_iterator<char>());
    const bool preserved_configuration =
        stable_contents.find("before-failed-configure") != std::string::npos &&
        stable_contents.find("after-failed-configure") != std::string::npos;
    std::error_code remove_error;
    std::filesystem::remove_all(temporary_directory, remove_error);
    return preserved_configuration ? 0 : 15;
}
