#pragma once

#include <chrono>
#include <cstdint>

namespace mujoco_simulation {

struct SchedulerConfig {
  bool realtime_sync{true};
  double realtime_factor{1.0};
  double state_update_rate{1000.0};
  double viewer_update_rate{60.0};
  std::chrono::milliseconds max_schedule_lag{100};
};

struct SchedulerStatistics {
  std::uint64_t physics_steps{0};
  std::uint64_t loop_iterations{0};
  std::uint64_t manual_step_calls{0};
  std::uint64_t reset_requests{0};
  std::uint64_t lag_recoveries{0};
  double last_loop_duration_sec{0.0};
  double last_step_duration_sec{0.0};
  double last_realtime_factor{0.0};
};

}  // namespace mujoco_simulation
