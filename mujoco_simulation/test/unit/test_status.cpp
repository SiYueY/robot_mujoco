#include <gtest/gtest.h>

#include "mujoco_simulation/result_code.hpp"
#include "mujoco_simulation/simulation.hpp"

namespace mujoco_simulation {
namespace {

TEST(ResultCodeTest, EnumValuesRemainDistinct) {
  const ResultCode model_load = ResultCode::ModelLoadFailed;
  EXPECT_EQ(model_load, ResultCode::ModelLoadFailed);

  const ResultCode model_validation = ResultCode::ModelValidationFailed;
  EXPECT_EQ(model_validation, ResultCode::ModelValidationFailed);

  const ResultCode binding = ResultCode::BindingFailed;
  EXPECT_EQ(binding, ResultCode::BindingFailed);

  const ResultCode timeout = ResultCode::Timeout;
  EXPECT_EQ(timeout, ResultCode::Timeout);

  const ResultCode thread_failed = ResultCode::ThreadFailed;
  EXPECT_EQ(thread_failed, ResultCode::ThreadFailed);
}

}  // namespace
}  // namespace mujoco_simulation
