#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include <mujoco_simulation/log/logging.hpp>

int main() {
    using mujoco_simulation::logging::Policy;

    const char* const file_name = "logging_record_contract.log";
    std::remove(file_name);
    Policy policy;
    policy.console_enabled = false;
    policy.file_enabled = true;
    policy.file_path = file_name;
    policy.show_source_location = true;
    if (!mujoco_simulation::logging::configure(policy)) return 1;

    constexpr char business[] = {'a', '\0', '\t', '\n', '\r', '\x1b', 'z'};
    SIM_INFO << std::string(business, sizeof(business));

    policy.show_source_location = false;
    if (!mujoco_simulation::logging::configure(policy)) return 2;
    SIM_INFO << "plain";
    mujoco_simulation::logging::flush();

    std::ifstream file(file_name);
    std::vector<std::string> lines;
    for (std::string line; std::getline(file, line);) lines.push_back(line);
    std::remove(file_name);
    if (lines.size() != 2) return 3;
    for (const auto& line : lines) {
        if (line.find("\033") != std::string::npos) return 7;
    }
    if (lines[0].find("[logging_record_contract_test.cpp:") == std::string::npos ||
        lines[0].find(" main]") == std::string::npos ||
        lines[0].find("a\\0\\t\\n\\r\\x1Bz") == std::string::npos)
        return 4;
    if (lines[1].find("[logging_record_contract_test.cpp:") != std::string::npos ||
        lines[1].find("plain") == std::string::npos)
        return 5;
    return 0;
}
