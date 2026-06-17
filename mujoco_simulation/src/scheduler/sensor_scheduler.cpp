#include "mujoco_simulation/scheduler/sensor_scheduler.hpp"

#include <cmath>

namespace mujoco_simulation {
namespace {

constexpr double kScheduleEpsilon = 1.0e-9;

}  // namespace

Status SensorScheduler::register_sensor(std::string_view name, double update_rate,
                                        double physics_rate) {
  const std::string sensor_name(name);
  if (sensor_name.empty()) {
    return Status::invalid_argument("Sensor name must not be empty.");
  }
  if (update_rate <= 0.0) {
    return Status::invalid_argument("Sensor update_rate must be positive: " + sensor_name);
  }
  if (physics_rate <= 0.0) {
    return Status::invalid_argument("Physics rate must be positive for sensor '" + sensor_name +
                                    "'.");
  }
  if (update_rate - physics_rate > kScheduleEpsilon) {
    return Status::invalid_argument("Sensor update_rate exceeds physics rate: " + sensor_name);
  }
  if (schedules_.find(sensor_name) != schedules_.end()) {
    return Status::failed_precondition("Sensor already registered in scheduler: " + sensor_name);
  }

  schedules_.emplace(sensor_name, SensorSchedule{.update_rate = update_rate,
                                                 .period = 1.0 / update_rate,
                                                 .next_due_time = 0.0,
                                                 .missed_updates = 0});
  return Status::Ok();
}

Status SensorScheduler::unregister_sensor(std::string_view name) {
  const auto it = schedules_.find(std::string(name));
  if (it == schedules_.end()) {
    return Status::not_found("Sensor is not registered in scheduler: " + std::string(name));
  }
  schedules_.erase(it);
  return Status::Ok();
}

bool SensorScheduler::has_sensor(std::string_view name) const {
  return schedules_.find(std::string(name)) != schedules_.end();
}

bool SensorScheduler::is_due(std::string_view name, double simulation_time) const {
  const auto it = schedules_.find(std::string(name));
  if (it == schedules_.end()) {
    return false;
  }
  return simulation_time + kScheduleEpsilon >= it->second.next_due_time;
}

Status SensorScheduler::mark_sampled(std::string_view name, double simulation_time) {
  const auto it = schedules_.find(std::string(name));
  if (it == schedules_.end()) {
    return Status::not_found("Sensor is not registered in scheduler: " + std::string(name));
  }

  SensorSchedule& schedule = it->second;
  if (simulation_time + kScheduleEpsilon < schedule.next_due_time) {
    schedule.next_due_time = simulation_time + schedule.period;
    return Status::Ok();
  }

  std::uint64_t due_slots = 0;
  do {
    schedule.next_due_time += schedule.period;
    ++due_slots;
  } while (simulation_time + kScheduleEpsilon >= schedule.next_due_time);
  if (due_slots > 1) {
    schedule.missed_updates += due_slots - 1;
  }
  return Status::Ok();
}

void SensorScheduler::reset() {
  for (auto& [name, schedule] : schedules_) {
    (void)name;
    schedule.next_due_time = 0.0;
    schedule.missed_updates = 0;
  }
}

std::uint64_t SensorScheduler::missed_updates(std::string_view name) const {
  const auto it = schedules_.find(std::string(name));
  return it == schedules_.end() ? 0 : it->second.missed_updates;
}

}  // namespace mujoco_simulation
