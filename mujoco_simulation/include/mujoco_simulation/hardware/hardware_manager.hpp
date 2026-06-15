#pragma once

#include <mujoco/mujoco.h>

#include <memory>
#include <string>
#include <unordered_map>

#include "mujoco_simulation/hardware/camera.hpp"
#include "mujoco_simulation/hardware/imu.hpp"
#include "mujoco_simulation/hardware/joint.hpp"
#include "mujoco_simulation/hardware/lidar.hpp"
#include "mujoco_simulation/hardware/mj_context.hpp"
#include "mujoco_simulation/hardware/mobile_base.hpp"

namespace mujoco_simulation {

class HardwareManager {
 public:
  HardwareManager(const mjModel* model, mjData* data, mjvScene* scene = nullptr,
                  mjrContext* render_context = nullptr);

  void set_render_resources(mjvScene* scene, mjrContext* render_context);

  bool register_joint(const JointInfo& data);
  bool unregister_joint(const std::string& name);
  bool has_joint(const std::string& name) const;
  bool configure_joint_command_mode(const std::string& name, CommandInterfaceType command_mode);
  bool write_joint(const std::string& name, const JointCommand& command);
  bool read_joint(const std::string& name, JointState& state);
  std::unordered_map<std::string, JointState> read_joint_states();

  bool register_imu(const ImuInfo& data);
  bool unregister_imu(const std::string& name);
  bool has_imu(const std::string& name) const;
  bool read_imu(const std::string& name, ImuState& state);

  bool register_camera(const CameraSpec& data);
  bool unregister_camera(const std::string& name);
  bool has_camera(const std::string& name) const;
  bool has_registered_camera(const std::string& name) const;
  bool ensure_camera_registered(const std::string& name);
  bool register_pending_cameras();
  bool read_camera(const std::string& name, CameraState& state);

  bool register_lidar(const LidarInfo& data);
  bool unregister_lidar(const std::string& name);
  bool has_lidar(const std::string& name) const;
  bool read_lidar(const std::string& name, LidarState& state);
  std::unordered_map<std::string, LidarState> read_lidar_states();

  bool register_mobile_base(const MobileBaseInfo& data);
  bool unregister_mobile_base(const std::string& name);
  bool write_mobile_base(const std::string& name, const MobileBaseCommand& command);
  bool read_mobile_base(const std::string& name, MobileBaseState& state);
  std::unordered_map<std::string, MobileBaseState> read_mobile_base_states();

  bool reset_all();

  const std::string& last_error() const { return last_error_; }

 private:
  template <typename DeviceMap>
  bool reset_device_map(DeviceMap& devices);

  template <typename DeviceMap, typename State>
  std::unordered_map<std::string, State> read_all_from(DeviceMap& devices);

  bool register_camera_device(const CameraSpec& data);
  bool set_error(const std::string& message);

  MjContext context_{};

  std::unordered_map<std::string, JointInfo> joint_infos_;
  std::unordered_map<std::string, ImuInfo> imu_infos_;
  std::unordered_map<std::string, CameraSpec> camera_specs_;
  std::unordered_map<std::string, CameraSpec> pending_camera_specs_;
  std::unordered_map<std::string, LidarInfo> lidar_infos_;
  std::unordered_map<std::string, std::unique_ptr<Joint>> joints_;
  std::unordered_map<std::string, std::unique_ptr<Imu>> imus_;
  std::unordered_map<std::string, std::unique_ptr<Camera>> cameras_;
  std::unordered_map<std::string, std::unique_ptr<Lidar>> lidars_;
  std::unordered_map<std::string, std::unique_ptr<MobileBase>> mobile_bases_;
  std::string last_error_;
};

}  // namespace mujoco_simulation
