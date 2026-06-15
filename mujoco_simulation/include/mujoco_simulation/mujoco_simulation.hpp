#pragma once

#include <mujoco/mujoco.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "mujoco_simulation/hardware/camera.hpp"
#include "mujoco_simulation/hardware/data.hpp"
#include "mujoco_simulation/hardware/imu.hpp"
#include "mujoco_simulation/hardware/joint.hpp"
#include "mujoco_simulation/hardware/lidar.hpp"
#include "mujoco_simulation/hardware/mobile_base.hpp"

namespace mujoco_simulation {

class HardwareManager;
class Viewer;

enum class RenderMode {
  Headless,
  Viewer,
};

struct SimulationConfig {
  std::string model_path;
  RenderMode render_mode = RenderMode::Headless;
  double sim_speed_factor = 1.0;
  std::string initial_keyframe;
};

class MuJoCoSimulation {
 public:
  MuJoCoSimulation();
  ~MuJoCoSimulation();

  MuJoCoSimulation(const MuJoCoSimulation &) = delete;
  MuJoCoSimulation &operator=(const MuJoCoSimulation &) = delete;

  bool initialize(const SimulationConfig &config, std::string &error_message);
  void start();
  void stop();

  bool set_paused(bool paused);
  bool paused() const;
  bool reset(const std::string &keyframe, std::string &error_message);
  bool step(uint32_t steps, std::string &error_message);

  bool register_joint(const JointInfo &configuration, std::string &error_message);
  bool configure_joint_command_mode(const std::string &joint_name,
                                    CommandInterfaceType command_mode, std::string &error_message);
  bool write_joint(const JointCommand &command, std::string &error_message);
  bool read_joint(const std::string &joint_name, JointState *state, std::string &error_message);

  bool register_imu(const ImuInfo &configuration, std::string &error_message);
  bool read_imu(const std::string &imu_name, ImuState *state, std::string &error_message);

  bool register_camera(const CameraSpec &configuration, std::string &error_message);
  bool read_camera(const std::string &camera_name, CameraState *state, std::string &error_message);

  bool register_lidar(const LidarInfo &configuration, std::string &error_message);
  bool read_lidar(const std::string &lidar_name, LidarState *state, std::string &error_message);

  bool register_mobile_base(const MobileBaseInfo &configuration, std::string &error_message);
  bool write_mobile_base(const std::string &name, const MobileBaseCommand &command,
                         std::string &error_message);
  bool read_mobile_base(const std::string &name, MobileBaseState *state,
                        std::string &error_message);

  uint64_t step_count() const;
  double simulation_time() const;
  bool has_joint(const std::string &joint_name) const;
  bool has_body(const std::string &body_name) const;
  bool has_site(const std::string &site_name) const;
  bool has_camera(const std::string &camera_name) const;
  int joint_id(const std::string &joint_name) const;
  int body_id(const std::string &body_name) const;
  int site_id(const std::string &site_name) const;
  int camera_id(const std::string &camera_name) const;
  bool camera_fovy(const std::string &camera_name, double *fovy_degrees,
                   std::string &error_message) const;
  const mjModel *model() const;
  bool is_initialized() const;
  bool is_running() const;
  const SimulationConfig &config() const;

  void with_locked_data(const std::function<void(const mjModel &, mjData &)> &callback);
  void with_locked_data(const std::function<void(const mjModel &, const mjData &)> &callback) const;
  bool copy_data_to(mjData *dest) const;

 private:
  void physics_loop();
  bool load_model(const std::string &model_path, std::string &error_message);
  bool start_viewer(std::string &error_message);
  bool inject_viewer_render_resources_locked(std::string &error_message);
  bool viewer_render_resources_ready_locked() const;
  int keyframe_id(const std::string &keyframe) const;

  SimulationConfig config_;
  mjModel *model_ = nullptr;
  mjData *data_ = nullptr;

  std::unique_ptr<HardwareManager> hardware_manager_;
  std::unique_ptr<Viewer> viewer_;
  bool viewer_started_{false};
  bool viewer_render_resources_injected_{false};

  mutable std::mutex mutex_;
  std::thread physics_thread_;
  std::atomic_bool running_{false};
  std::atomic_bool paused_{false};
  std::atomic_uint64_t step_count_{0};
};

RenderMode parse_render_mode(const std::string &value);
const char *to_string(RenderMode mode);

}  // namespace mujoco_simulation
