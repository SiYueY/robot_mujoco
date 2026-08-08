#define SIM_LOG_COMPILED_LEVEL 6

#include <mujoco_simulation/log/logging.hpp>

int main() {
    int evaluated = 0;
    SIM_DEBUG << ++evaluated;
    SIM_INFO << ++evaluated;
    SIM_WARN << ++evaluated;
    SIM_ERROR << ++evaluated;
    SIM_FATAL << ++evaluated;
    return evaluated == 0 ? 0 : 1;
}
