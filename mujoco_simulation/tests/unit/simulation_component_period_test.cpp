#include <mujoco/mujoco.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>

#include "component/component.hpp"

namespace {

class TestComponent final : public mujoco_simulation::SimulationComponent {
public:
    explicit TestComponent(double period) : SimulationComponent("test_component", period) {}

    bool init(const mujoco_simulation::mjContext& context) override { return configure(context); }

    bool reset(const mujoco_simulation::mjContext&) override { return reset_schedule(); }

    bool advance(const mujoco_simulation::mjContext&) override { return true; }

    bool update(const mujoco_simulation::mjContext&) override { return true; }
};

bool check(bool value, const char* message) {
    if (!value) {
        std::cerr << message << '\n';
    }
    return value;
}

bool write_model(const std::filesystem::path& path) {
    std::ofstream output(path);
    output << R"(<mujoco model="component_period_test">
  <option timestep="0.001"/>
  <worldbody/>
</mujoco>)";
    return output.good();
}

}  // namespace

int main() {
    const std::filesystem::path model_path =
        std::filesystem::temp_directory_path() / "mujoco_component_period_test.xml";
    if (!check(write_model(model_path), "failed to write MuJoCo test model")) {
        return 1;
    }

    char error[1024] = {};
    mjModel* model = mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error));
    std::filesystem::remove(model_path);
    if (!check(model != nullptr, error)) {
        return 1;
    }
    mjData* data = mj_makeData(model);
    if (!check(data != nullptr, "failed to allocate MuJoCo data")) {
        mj_deleteModel(model);
        return 1;
    }

    mujoco_simulation::mjContext context(model, data);
    TestComponent every_step(0.001);
    TestComponent zero_period(0.0);
    TestComponent every_third_step(0.003);
    TestComponent negative_period(-0.001);
    TestComponent infinite_period(std::numeric_limits<double>::infinity());
    TestComponent too_fast(0.0005);
    TestComponent misaligned(0.0025);

    bool success =
        check(every_step.init(context), "physics-period component should configure successfully") &&
        check(!zero_period.init(context), "zero period should be rejected") &&
        check(every_third_step.init(context), "aligned period should configure successfully") &&
        check(!negative_period.init(context), "negative period was accepted") &&
        check(!infinite_period.init(context), "infinite period was accepted") &&
        check(!too_fast.init(context), "period shorter than physics period was accepted") &&
        check(!misaligned.init(context), "non-integral period multiple was accepted");

    success =
        success &&
        check(every_step.poll_update(0.0), "physics-period component should be due at time zero") &&
        check(
            every_step.poll_update(0.001),
            "physics-period component should be due every physics step") &&
        check(every_third_step.poll_update(0.0), "periodic component should be due at time zero") &&
        check(!every_third_step.poll_update(0.001), "periodic component was due one step early") &&
        check(!every_third_step.poll_update(0.002), "periodic component was due two steps early") &&
        check(
            every_third_step.poll_update(0.003), "periodic component was not due at its period") &&
        check(every_third_step.reset(context), "component schedule reset failed") &&
        check(
            every_third_step.poll_update(0.0),
            "reset did not restore the initial sampling schedule");

    return success ? 0 : 1;
}
