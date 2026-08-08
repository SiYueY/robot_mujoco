#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include <mujoco_simulation/log/logging.hpp>

namespace logging = mujoco_simulation::logging;

namespace {
struct ThrowingInsertion {};

std::ostream& operator<<(std::ostream&, const ThrowingInsertion&) { throw 7; }

std::vector<std::string> read_lines(const char* path) {
    std::ifstream input(path);
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(input, line)) lines.push_back(line);
    return lines;
}
}  // namespace

int main() {
    const char* const path = "logging_frontend_contract.log";
    std::remove(path);
    logging::Policy policy;
    policy.console_enabled = false;
    policy.file_enabled = true;
    policy.file_path = path;
    if (!logging::configure(policy)) return 1;

    SIM_INFO << "kept";
    SIM_INFO << "manipulated=" << std::hex << 42;
    try {
        SIM_INFO << "partial-throw" << ThrowingInsertion{};
        return 2;
    } catch (int value) {
        if (value != 7) return 3;
    }
    logging::flush();

    const std::vector<std::string> lines = read_lines(path);
    std::remove(path);
    if (lines.size() != 2) return 4;
    if (lines[0].find("kept") == std::string::npos ||
        lines[1].find("manipulated=2a") == std::string::npos)
        return 5;
    for (const std::string& line : lines) {
        if (line.find("partial-throw") != std::string::npos) {
            return 6;
        }
    }
    return 0;
}
