#include <array>
#include <atomic>
#include <cerrno>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>

#include <fcntl.h>
#include <sys/wait.h>
#include <unistd.h>

#include <mujoco_simulation/log/logging.hpp>

namespace logging = mujoco_simulation::logging;

namespace {
struct TempDirectory final {
    TempDirectory() {
        std::array<char, 64> pattern{};
        std::snprintf(pattern.data(), pattern.size(), "/tmp/mujoco_logging_concurrency_XXXXXX");
        path = ::mkdtemp(pattern.data());
    }
    ~TempDirectory() {
        std::error_code ignored;
        std::filesystem::remove_all(path, ignored);
    }
    std::filesystem::path path;
};

void wait_for_start(std::atomic<int>& ready, const std::atomic<bool>& start) {
    ready.fetch_add(1, std::memory_order_release);
    while (!start.load(std::memory_order_acquire)) std::this_thread::yield();
}

int run_default_race(const bool configure_policy) {
    TempDirectory temporary;
    if (temporary.path.empty() || ::chdir(temporary.path.c_str()) != 0) return 1;

    std::atomic<int> ready{0};
    std::atomic<bool> start{false};
    std::thread bootstrap([&] {
        wait_for_start(ready, start);
        SIM_INFO << "concurrent-default-bootstrap";
    });
    std::thread update([&] {
        wait_for_start(ready, start);
        if (configure_policy) {
            logging::Policy policy;
            policy.level = logging::Level::Off;
            policy.console_enabled = false;
            policy.file_enabled = true;
            policy.file_path = (temporary.path / "explicit.log").string();
            (void)logging::configure(policy);
        } else {
            (void)logging::set_level(logging::Level::Off);
        }
    });
    while (ready.load(std::memory_order_acquire) != 2) std::this_thread::yield();
    start.store(true, std::memory_order_release);
    bootstrap.join();
    update.join();
    return logging::level() == logging::Level::Off ? 0 : 2;
}

bool run_child(const bool configure_policy) {
    const pid_t child = ::fork();
    if (child == 0) _exit(run_default_race(configure_policy));
    int status = 0;
    return child > 0 && ::waitpid(child, &status, 0) == child && WIFEXITED(status) &&
           WEXITSTATUS(status) == 0;
}

bool no_flush_failure_notice() {
    TempDirectory temporary;
    if (temporary.path.empty()) return false;
    logging::Policy policy;
    policy.console_enabled = false;
    policy.file_enabled = true;
    policy.file_path = (temporary.path / "active.log").string();
    if (!logging::configure(policy)) return false;

    int pipe_fds[2]{};
    if (::pipe(pipe_fds) != 0) return false;
    const int saved_stderr = ::dup(STDERR_FILENO);
    if (saved_stderr < 0 || ::dup2(pipe_fds[1], STDERR_FILENO) < 0) {
        if (saved_stderr >= 0) ::close(saved_stderr);
        ::close(pipe_fds[0]);
        ::close(pipe_fds[1]);
        return false;
    }
    ::close(pipe_fds[1]);

    std::atomic<bool> start{false};
    std::thread configurator([&] {
        while (!start.load(std::memory_order_acquire)) std::this_thread::yield();
        for (int index = 0; index < 2000; ++index) {
            policy.file_enabled = (index % 2) == 0;
            (void)logging::configure(policy);
        }
    });
    std::thread flusher([&] {
        while (!start.load(std::memory_order_acquire)) std::this_thread::yield();
        for (int index = 0; index < 2000; ++index) logging::flush();
    });
    start.store(true, std::memory_order_release);
    configurator.join();
    flusher.join();
    (void)::dup2(saved_stderr, STDERR_FILENO);
    ::close(saved_stderr);

    (void)::fcntl(pipe_fds[0], F_SETFL, ::fcntl(pipe_fds[0], F_GETFL) | O_NONBLOCK);
    std::string output;
    std::array<char, 256> buffer{};
    for (;;) {
        const ssize_t count = ::read(pipe_fds[0], buffer.data(), buffer.size());
        if (count > 0) {
            output.append(buffer.data(), static_cast<std::size_t>(count));
            continue;
        }
        if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) break;
        break;
    }
    ::close(pipe_fds[0]);
    return output.find("adapter sink failure") == std::string::npos;
}
}  // namespace

int main() {
    for (int iteration = 0; iteration < 64; ++iteration) {
        if (!run_child(true) || !run_child(false)) return 1;
    }
    return no_flush_failure_notice() ? 0 : 2;
}
