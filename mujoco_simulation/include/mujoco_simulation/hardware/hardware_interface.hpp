#pragma once

namespace mujoco_simulation {

template <typename HardwareData, typename Command, typename State>
class HardwareInterface {
 public:
  HardwareInterface() = default;
  HardwareInterface(const HardwareInterface&) = delete;
  HardwareInterface& operator=(const HardwareInterface&) = delete;
  HardwareInterface(HardwareInterface&&) = delete;
  HardwareInterface& operator=(HardwareInterface&&) = delete;
  virtual ~HardwareInterface() = default;

  virtual bool init(const HardwareData& data) = 0;
  virtual bool reset() = 0;

  virtual bool write(const Command& command) = 0;
  virtual bool read(State& state) = 0;

  virtual const std::string& last_error() const = 0;
};

}  // namespace mujoco_simulation
