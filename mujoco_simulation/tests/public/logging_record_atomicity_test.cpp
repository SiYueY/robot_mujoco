#include <atomic>
#include <cstdio>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

#include <mujoco_simulation/log/logging.hpp>

int main() {
    const char* const path = "logging_record_atomicity.log";
    std::remove(path);
    mujoco_simulation::logging::Policy policy;
    policy.console_enabled = false;
    policy.file_enabled = true;
    policy.file_path = path;
    if (!mujoco_simulation::logging::configure(policy)) return 1;

    constexpr int kThreads = 8;
    constexpr int kRecordsPerThread = 100;
    std::vector<std::thread> threads;
    for (int thread = 0; thread < kThreads; ++thread) {
        threads.emplace_back([thread] {
            for (int sequence = 0; sequence < kRecordsPerThread; ++sequence) {
                SIM_INFO << "atomic-record:" << thread << ':' << sequence;
            }
        });
    }
    for (std::thread& thread : threads) thread.join();
    mujoco_simulation::logging::flush();

    std::ifstream input(path);
    std::string line;
    int lines = 0;
    while (std::getline(input, line)) {
        if (line.find("atomic-record:") == std::string::npos ||
            line.find("atomic-record:", line.find("atomic-record:") + 1) != std::string::npos) {
            std::remove(path);
            return 2;
        }
        ++lines;
    }
    std::remove(path);
    return lines == kThreads * kRecordsPerThread ? 0 : 3;
}
