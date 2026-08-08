#include <array>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <string>

#include <fcntl.h>
#include <pty.h>
#include <unistd.h>

#include <mujoco_simulation/log/logging.hpp>

namespace logging = mujoco_simulation::logging;

namespace {
std::string write_to_pty(const bool colored, const char* message) {
    int master = -1;
    int slave = -1;
    if (::openpty(&master, &slave, nullptr, nullptr, nullptr) != 0) return {};
    const int saved_stderr = ::dup(STDERR_FILENO);
    if (saved_stderr < 0 || ::dup2(slave, STDERR_FILENO) < 0) {
        if (saved_stderr >= 0) ::close(saved_stderr);
        ::close(master);
        ::close(slave);
        return {};
    }
    ::close(slave);

    logging::Policy policy;
    policy.console_enabled = true;
    policy.file_enabled = false;
    policy.colored_console = colored;
    const bool configured = logging::configure(policy);
    if (configured) {
        SIM_INFO << message;
        logging::flush();
    }
    (void)::dup2(saved_stderr, STDERR_FILENO);
    ::close(saved_stderr);

    (void)::fcntl(master, F_SETFL, ::fcntl(master, F_GETFL) | O_NONBLOCK);
    std::string result;
    std::array<char, 256> buffer{};
    for (;;) {
        const ssize_t count = ::read(master, buffer.data(), buffer.size());
        if (count > 0) {
            result.append(buffer.data(), static_cast<std::size_t>(count));
            continue;
        }
        if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) break;
        break;
    }
    ::close(master);
    return configured ? result : std::string{};
}
}  // namespace

int main() {
    const std::string plain = write_to_pty(false, "plain-console");
    if (plain.find("plain-console") == std::string::npos ||
        plain.find("\033[") != std::string::npos)
        return 1;

    const std::string colored = write_to_pty(true, "colored-console");
    if (colored.find("colored-console") == std::string::npos ||
        colored.find("\033[") == std::string::npos)
        return 2;
    return 0;
}
