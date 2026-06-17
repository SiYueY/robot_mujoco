#include <gtest/gtest.h>

#include "mujoco_simulation/status.hpp"

namespace mujoco_simulation {
namespace {

TEST(StatusTest, FactoryHelpersProduceExpectedCodesAndMessages) {
  const Status model_load = Status::model_load_failed("load failed");
  EXPECT_EQ(model_load.code(), StatusCode::ModelLoadFailed);
  EXPECT_EQ(model_load.message(), "load failed");

  const Status model_validation = Status::model_validation_failed("validation failed");
  EXPECT_EQ(model_validation.code(), StatusCode::ModelValidationFailed);
  EXPECT_EQ(model_validation.message(), "validation failed");

  const Status binding = Status::binding_failed("binding failed");
  EXPECT_EQ(binding.code(), StatusCode::BindingFailed);
  EXPECT_EQ(binding.message(), "binding failed");

  const Status timeout = Status::timeout("timed out");
  EXPECT_EQ(timeout.code(), StatusCode::Timeout);
  EXPECT_EQ(timeout.message(), "timed out");

  const Status thread_failed = Status::thread_failed("thread failed");
  EXPECT_EQ(thread_failed.code(), StatusCode::ThreadFailed);
  EXPECT_EQ(thread_failed.message(), "thread failed");
}

}  // namespace
}  // namespace mujoco_simulation
