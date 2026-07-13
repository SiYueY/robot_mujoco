#pragma once

#include <mujoco/mujoco.h>

#include <cstdint>
#include <string>

#include "mujoco_simulation/result_code.hpp"

namespace mujoco_simulation {

struct UpdateContext;

class SimulationComponent {
 public:
  virtual ~SimulationComponent();

  bool should_update(double simulation_time);
  void reset_update_schedule();
  std::uint64_t missed_updates() const noexcept;

  virtual ResultCode update(const UpdateContext& context) = 0;
  virtual std::string name() const noexcept = 0;
  virtual ResultCode bind(const mjModel& model) = 0;
  virtual ResultCode reset(const mjModel& model, mjData& data) = 0;

 protected:
  SimulationComponent() = default;

  ResultCode set_update_rate(double update_rate, double physics_rate);
  void set_update_every_step() noexcept;

 private:
  double update_rate_{0.0};
  double period_{0.0};
  double next_due_time_{0.0};
  std::uint64_t missed_updates_{0};
};

}  // namespace mujoco_simulation
