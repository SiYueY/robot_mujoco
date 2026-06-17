#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <unordered_map>

#include "mujoco_simulation/status.hpp"

namespace mujoco_simulation {

class SensorScheduler {
 public:
  Status register_sensor(std::string_view name, double update_rate, double physics_rate);
  Status unregister_sensor(std::string_view name);

  bool has_sensor(std::string_view name) const;
  bool is_due(std::string_view name, double simulation_time) const;
  Status mark_sampled(std::string_view name, double simulation_time);
  void reset();

  std::uint64_t missed_updates(std::string_view name) const;

 private:
  struct SensorSchedule {
    double update_rate{0.0};
    double period{0.0};
    double next_due_time{0.0};
    std::uint64_t missed_updates{0};
  };

  std::unordered_map<std::string, SensorSchedule> schedules_;
};

}  // namespace mujoco_simulation
