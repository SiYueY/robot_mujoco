#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <variant>
#include <vector>

#include "romujoco/component/camera.hpp"
#include "romujoco/component/imu.hpp"
#include "romujoco/component/joint.hpp"
#include "romujoco/component/lidar.hpp"
#include "romujoco/component/mobile_base.hpp"
#include "romujoco/component/mobile_base/mecanum.hpp"
#include "romujoco/component/mobile_base/swerve.hpp"

namespace romujoco {

using ComponentConfig = std::variant<JointInfo, ImuInfo, CameraConfig, LidarInfo, MecanumMobileBaseInfo, SwerveMobileBaseInfo>;
using ComponentConfigList = std::vector<ComponentConfig>;

// Configuration contract for the always-built internal camera renderer.  It
// intentionally describes policy only and does not expose any GL or MuJoCo
// rendering type.
struct CameraRendererConfig {
    int max_scene_geometries{2000};
    bool allow_glfw_backend{true};
    bool allow_egl_backend{true};
    std::size_t completed_ticket_history{8};
};

struct ModelConfig {
    std::string model_path;
    std::string initial_keyframe;
};

struct SchedulerConfig {
    double physics_period{0.001};
    double viewer_period{1.0 / 60.0};
};

struct SimulationConfig {
    ModelConfig model;
    SchedulerConfig scheduler;
    ComponentConfigList components;
    // Disables GUI viewer creation while leaving camera rendering available.
    bool viewer_enabled{true};
    std::chrono::milliseconds viewer_startup_timeout{5000};
    CameraRendererConfig camera_renderer;
};

}  // namespace romujoco
