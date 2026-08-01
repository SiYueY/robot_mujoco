#pragma once

#include <chrono>
#include <filesystem>
#include <fstream>
#include <string_view>
#include <thread>

namespace mujoco_simulation_test {

class TemporaryFile {
public:
  explicit TemporaryFile(std::string_view filename)
      : path_(std::filesystem::temp_directory_path() / filename) {}
  ~TemporaryFile() { std::filesystem::remove(path_); }

  TemporaryFile(const TemporaryFile &) = delete;
  TemporaryFile &operator=(const TemporaryFile &) = delete;

  const std::filesystem::path &path() const noexcept { return path_; }

  bool write(std::string_view content) const {
    std::ofstream output(path_);
    output << content;
    return output.good();
  }

private:
  std::filesystem::path path_;
};

template <typename Predicate>
bool wait_until(Predicate predicate, std::chrono::milliseconds timeout,
                std::chrono::milliseconds interval = std::chrono::milliseconds{
                    1}) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!predicate() && std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(interval);
  return predicate();
}

} // namespace mujoco_simulation_test
