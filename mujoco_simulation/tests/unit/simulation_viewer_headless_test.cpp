#include <iostream>

#include <mujoco/mujoco.h>

#include "runtime/context.hpp"
#include "test_support.hpp"
#include "viewer/simulation_viewer.hpp"

namespace {

bool check(bool value, const char *message) {
  if (!value)
    std::cerr << message << '\n';
  return value;
}

} // namespace

int main() {
  mujoco_simulation_test::TemporaryFile model_file(
      "mujoco_simulation_viewer_headless_test.xml");
  if (!check(
          model_file.write("<mujoco model='viewer_headless'><worldbody><geom "
                           "type='plane' size='1 1 .1'/></worldbody></mujoco>"),
          "failed to write MuJoCo model")) {
    return 1;
  }
  char error[1024] = {};
  mjModel *model =
      mj_loadXML(model_file.path().c_str(), nullptr, error, sizeof(error));
  if (!check(model != nullptr, error))
    return 1;
  mjData *data = mj_makeData(model);
  if (!check(data != nullptr, "failed to create MuJoCo data")) {
    mj_deleteModel(model);
    return 1;
  }
  mj_forward(model, data);

  mujoco_simulation::mjContext context(model, data);
  mujoco_simulation::SimulationViewer viewer;
  const bool prepared = viewer.prepare(context);
  viewer.stop();
  context.clear();
  return prepared ? 0 : 1;
}
