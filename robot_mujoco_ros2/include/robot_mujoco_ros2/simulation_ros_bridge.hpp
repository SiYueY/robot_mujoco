#pragma once

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "mujoco_simulation/status.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/context.hpp"
#include "rclcpp/contexts/default_context.hpp"
#include "rclcpp/executor_options.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/time.hpp"
#include "robot_mujoco_msgs/srv/reset_world.hpp"
#include "robot_mujoco_msgs/srv/set_realtime_factor.hpp"
#include "robot_mujoco_msgs/srv/step_simulation.hpp"
#include "robot_mujoco_ros2/bridge_config.hpp"
#include "robot_mujoco_ros2/publish_channel.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace robot_mujoco_ros2 {

class ImuPublisher;
class CameraPublisher;
class LidarPublisher;

class SimulationRosBridge {
 public:
  using ServiceGateCallback = std::function<bool()>;
  using StatusCallback = std::function<mujoco_simulation::Status()>;
  using StepStatusCallback = std::function<mujoco_simulation::Status(std::uint32_t)>;
  using RealtimeFactorStatusCallback = std::function<mujoco_simulation::Status(double)>;
  using KeyframeResetStatusCallback = std::function<mujoco_simulation::Status(const std::string&)>;

  SimulationRosBridge(
      SimulationRosBridgeConfig config,
      rclcpp::Context::SharedPtr context = rclcpp::contexts::get_global_default_context(),
      ServiceGateCallback service_gate_callback = {}, StatusCallback reset_callback = {},
      StatusCallback start_callback = {}, StatusCallback stop_callback = {},
      StatusCallback pause_callback = {}, StatusCallback resume_callback = {},
      StepStatusCallback step_callback = {},
      RealtimeFactorStatusCallback realtime_factor_callback = {},
      KeyframeResetStatusCallback load_keyframe_callback = {});
  ~SimulationRosBridge();

  SimulationRosBridge(const SimulationRosBridge&) = delete;
  SimulationRosBridge& operator=(const SimulationRosBridge&) = delete;
  SimulationRosBridge(SimulationRosBridge&&) = delete;
  SimulationRosBridge& operator=(SimulationRosBridge&&) = delete;

  mujoco_simulation::Status start();
  mujoco_simulation::Status stop();
  void update_sim_time(const rclcpp::Time& sim_time);
  mujoco_simulation::Status enqueue_publish_bundle(const PublishBundle& bundle);

 private:
  void publish_clock(const rclcpp::Time& sim_time);
  void run_publish_worker();
  bool is_service_allowed() const;
  template <typename CallbackT>
  mujoco_simulation::Status invoke_service_callback(const CallbackT& callback) const;
  PublishChannelConfig make_publish_channel_config() const;

  SimulationRosBridgeConfig config_;
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Context::SharedPtr context_;
  ServiceGateCallback service_gate_callback_;
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  std::unique_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread executor_thread_;
  std::atomic<std::int64_t> sim_time_ns_{0};
  std::atomic<bool> worker_running_{false};
  std::atomic<bool> shutdown_{false};
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_service_;
  rclcpp::Service<robot_mujoco_msgs::srv::StepSimulation>::SharedPtr step_service_;
  rclcpp::Service<robot_mujoco_msgs::srv::SetRealtimeFactor>::SharedPtr
      set_realtime_factor_service_;
  rclcpp::Service<robot_mujoco_msgs::srv::ResetWorld>::SharedPtr load_keyframe_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  std::vector<std::unique_ptr<ImuPublisher>> imus_;
  std::vector<std::unique_ptr<CameraPublisher>> cameras_;
  std::vector<std::unique_ptr<LidarPublisher>> lidars_;
  std::thread publish_thread_;
  PublishChannel publish_channel_;
};

}  // namespace robot_mujoco_ros2
