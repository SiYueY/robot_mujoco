#pragma once

#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <thread>
#include <unordered_map>

#include "mujoco_ros2_bridge_msgs/srv/reset_world.hpp"
#include "mujoco_ros2_bridge_msgs/srv/set_realtime_factor.hpp"
#include "mujoco_ros2_bridge_msgs/srv/step_simulation.hpp"
#include "mujoco_simulation/component/camera/camera_sample.hpp"
#include "mujoco_simulation/component/imu/imu_sample.hpp"
#include "mujoco_simulation/component/lidar/lidar_sample.hpp"
#include "mujoco_simulation/status.hpp"
#include "mujoco_simulation_ros/bridge_config.hpp"
#include "rclcpp/context.hpp"
#include "rclcpp/contexts/default_context.hpp"
#include "rclcpp/executor_options.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/time.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace mujoco_simulation_ros {

class ImuPublisher;
class CameraPublisher;
class LidarPublisher;

class SimulationRosBridge {
 public:
  using StatusCallback = std::function<mujoco_simulation::Status()>;
  using StepStatusCallback = std::function<mujoco_simulation::Status(std::uint32_t)>;
  using RealtimeFactorStatusCallback = std::function<mujoco_simulation::Status(double)>;
  using KeyframeResetStatusCallback = std::function<mujoco_simulation::Status(const std::string&)>;

  SimulationRosBridge(
      SimulationRosBridgeConfig config,
      rclcpp::Context::SharedPtr context = rclcpp::contexts::get_global_default_context(),
      StatusCallback reset_callback = {}, StatusCallback start_callback = {},
      StatusCallback stop_callback = {}, StatusCallback pause_callback = {},
      StatusCallback resume_callback = {}, StepStatusCallback step_callback = {},
      RealtimeFactorStatusCallback realtime_factor_callback = {},
      KeyframeResetStatusCallback load_keyframe_callback = {});
  ~SimulationRosBridge();

  SimulationRosBridge(const SimulationRosBridge&) = delete;
  SimulationRosBridge& operator=(const SimulationRosBridge&) = delete;
  SimulationRosBridge(SimulationRosBridge&&) = delete;
  SimulationRosBridge& operator=(SimulationRosBridge&&) = delete;

  void set_time(const rclcpp::Time& sim_time);
  mujoco_simulation::Status publish_imu(std::string_view name,
                                        const mujoco_simulation::ImuSample& sample);
  mujoco_simulation::Status publish_camera(std::string_view name,
                                           const mujoco_simulation::CameraSample& sample);
  mujoco_simulation::Status publish_lidar(std::string_view name,
                                          const mujoco_simulation::LidarSample& sample);

 private:
  SimulationRosBridgeConfig config_;
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Context::SharedPtr context_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread executor_thread_;
  rclcpp::Time sim_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_service_;
  rclcpp::Service<mujoco_ros2_bridge_msgs::srv::StepSimulation>::SharedPtr step_service_;
  rclcpp::Service<mujoco_ros2_bridge_msgs::srv::SetRealtimeFactor>::SharedPtr
      set_realtime_factor_service_;
  rclcpp::Service<mujoco_ros2_bridge_msgs::srv::ResetWorld>::SharedPtr load_keyframe_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  std::unordered_map<std::string, std::unique_ptr<ImuPublisher>> imus_;
  std::unordered_map<std::string, std::unique_ptr<CameraPublisher>> cameras_;
  std::unordered_map<std::string, std::unique_ptr<LidarPublisher>> lidars_;
};

}  // namespace mujoco_simulation_ros
