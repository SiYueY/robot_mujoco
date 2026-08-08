#include <array>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

#include "log/impl/easylogging_adapter.hpp"

namespace {
struct TempDirectory final {
    TempDirectory() {
        std::array<char, 64> pattern{};
        std::snprintf(pattern.data(), pattern.size(), "/tmp/mujoco_logging_adapter_XXXXXX");
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
    using mujoco_simulation::logging::Level;
    using mujoco_simulation::logging::impl::EasyloggingAdapter;
    using mujoco_simulation::logging::impl::SinkMask;

    TempDirectory temporary;
    if (temporary.path.empty()) return 1;
    const std::string file_path = (temporary.path / "active.log").string();
    EasyloggingAdapter adapter;
    if (!adapter.configure(SinkMask::File, file_path, false)) return 2;
    if (adapter.write(Level::Info, "before-failed-candidate", 23) != SinkMask::File ||
        adapter.flush(SinkMask::File) != SinkMask::File) {
        return 3;
    }
    if (adapter.configure(SinkMask::File, temporary.path.string(), false)) return 4;
    if (adapter.write(Level::Info, "after-failed-candidate", 22) != SinkMask::File ||
        adapter.flush(SinkMask::File) != SinkMask::File) {
        return 5;
    }

    std::ifstream input(file_path);
    std::string contents((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    return contents.find("before-failed-candidate") != std::string::npos &&
                   contents.find("after-failed-candidate") != std::string::npos
               ? 0
               : 6;
}
