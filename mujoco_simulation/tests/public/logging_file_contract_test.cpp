#include <array>
#include <cerrno>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>

#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <mujoco_simulation/log/logging.hpp>

namespace {
struct TempDirectory final {
    TempDirectory() {
        std::array<char, 64> pattern{};
        std::snprintf(pattern.data(), pattern.size(), "/tmp/mujoco_logging_XXXXXX");
        path = ::mkdtemp(pattern.data());
    }
    ~TempDirectory() {
        std::error_code ignored;
        std::filesystem::remove_all(path, ignored);
    }
    std::filesystem::path path;
};
}  // namespace

int main() {
    using mujoco_simulation::logging::Policy;

    TempDirectory temporary;
    if (temporary.path.empty()) return 1;

    Policy policy;
    policy.console_enabled = false;
    policy.file_enabled = false;
    policy.file_path = std::string("ignored\0path", 12);
    if (!mujoco_simulation::logging::configure(policy)) return 2;

    policy.file_enabled = true;
    policy.file_path.clear();
    if (mujoco_simulation::logging::configure(policy)) return 3;

    policy.file_path = (temporary.path / "missing" / "file.log").string();
    if (mujoco_simulation::logging::configure(policy)) return 4;

    policy.file_path = temporary.path.string();
    if (mujoco_simulation::logging::configure(policy)) return 5;

    const auto fifo = temporary.path / "log.fifo";
    if (::mkfifo(fifo.c_str(), 0600) != 0) return 6;
    policy.file_path = fifo.string();
    if (mujoco_simulation::logging::configure(policy)) return 7;

    policy.file_path = "/dev/null";
    if (mujoco_simulation::logging::configure(policy)) return 8;

    const auto socket = temporary.path / "log.sock";
    const int socket_fd = ::socket(AF_UNIX, SOCK_STREAM, 0);
    if (socket_fd < 0) return 9;
    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    if (socket.string().size() >= sizeof(address.sun_path)) return 10;
    std::snprintf(address.sun_path, sizeof(address.sun_path), "%s", socket.c_str());
    const auto address_size =
        static_cast<socklen_t>(offsetof(sockaddr_un, sun_path) + std::strlen(address.sun_path) + 1);
    const bool socket_available =
        ::bind(socket_fd, reinterpret_cast<const sockaddr*>(&address), address_size) == 0;
    if (!socket_available && errno != EPERM) return 11;
    policy.file_path = socket.string();
    const bool accepted_socket = socket_available && mujoco_simulation::logging::configure(policy);
    ::close(socket_fd);
    if (accepted_socket) return 12;

    const auto regular = temporary.path / "regular.log";
    std::ofstream output{regular};
    output << "existing-record\n";
    output.close();
    policy.file_path = regular.string();
    if (!mujoco_simulation::logging::configure(policy)) return 13;
    SIM_INFO << "appended-record";
    mujoco_simulation::logging::flush();
    std::ifstream input{regular};
    std::string first_line;
    std::string second_line;
    if (!std::getline(input, first_line) || !std::getline(input, second_line) ||
        first_line != "existing-record" || second_line.find("appended-record") == std::string::npos)
        return 14;

    const auto symlink = temporary.path / "regular-link.log";
    std::error_code error;
    std::filesystem::create_symlink(regular, symlink, error);
    if (error) return 15;
    policy.file_path = symlink.string();
    if (!mujoco_simulation::logging::configure(policy)) return 16;

    const auto dangling = temporary.path / "dangling-link.log";
    std::filesystem::create_symlink(temporary.path / "absent.log", dangling, error);
    if (error) return 17;
    policy.file_path = dangling.string();
    return mujoco_simulation::logging::configure(policy) ? 18 : 0;
}
