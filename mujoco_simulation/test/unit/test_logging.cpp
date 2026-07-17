#include <gtest/gtest.h>

#include "mujoco_simulation/common/logging.hpp"

namespace mujoco_simulation {
namespace {

TEST(Logging, EasyloggingMacrosAreLinked) {
  LOG_INFO << "easylogging++ integration check";
  SUCCEED();
}

}  // namespace
}  // namespace mujoco_simulation
