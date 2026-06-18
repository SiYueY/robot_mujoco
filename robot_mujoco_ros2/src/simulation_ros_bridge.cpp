#include "robot_mujoco_ros2/simulation_ros_bridge.hpp"

#include <chrono>
#include <memory>
#include <thread>
#include <utility>

#include "rclcpp/create_service.hpp"
#include "robot_mujoco_ros2/message_mapper.hpp"
#include "robot_mujoco_ros2/qos_profiles.hpp"

namespace robot_mujoco_ros2 {
namespace {

using namespace std::chrono_literals;

rclcpp::NodeOptions make_node_options(const rclcpp::Context::SharedPtr& context) {
  rclcpp::NodeOptions options;
  options.context(context)
      .use_global_arguments(false)
      .enable_rosout(false)
      .start_parameter_services(false)
      .start_parameter_event_publisher(false)
      .use_clock_thread(false);
  return options;
}

template <typename ResponseT>
void fill_status_response(const mujoco_simulation::Status& status,
                          const std::string& success_message, ResponseT& response) {
  response.success = status.ok();
  response.message = status.ok() ? success_message : status.message();
}

template <typename ServiceT, typename RequestHandlerT>
typename rclcpp::Service<ServiceT>::SharedPtr create_service(
    rclcpp::Node& node, const std::string& service_name,
    const rclcpp::CallbackGroup::SharedPtr& callback_group, RequestHandlerT&& request_handler) {
  return rclcpp::create_service<ServiceT>(node.get_node_base_interface(),
                                          node.get_node_services_interface(), service_name,
                                          std::forward<RequestHandlerT>(request_handler),
                                          rmw_qos_profile_services_default, callback_group);
}

}  // namespace

class ImuPublisher {
 public:
  ImuPublisher(rclcpp::Node& node, ImuPublisherConfig config)
      : config_(std::move(config)),
        publisher_(node.create_publisher<sensor_msgs::msg::Imu>(config_.topic,
                                                                qos_profiles::sensor_data())) {}

  void publish(const mujoco_simulation::ImuSample& sample, const rclcpp::Time& stamp) {
    publisher_->publish(message_mapper::make_imu_message(config_, sample, stamp));
  }

 private:
  ImuPublisherConfig config_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
};

class CameraPublisher {
 public:
  CameraPublisher(rclcpp::Node& node, CameraPublisherConfig config) : config_(std::move(config)) {
    if (config_.enable_rgb) {
      rgb_ = node.create_publisher<sensor_msgs::msg::Image>(config_.rgb_topic,
                                                            qos_profiles::sensor_data());
      rgb_message_.data.resize(static_cast<std::size_t>(config_.width) * config_.height * 3U);
    }
    if (config_.enable_depth) {
      depth_ = node.create_publisher<sensor_msgs::msg::Image>(config_.depth_topic,
                                                              qos_profiles::sensor_data());
      depth_message_.data.resize(static_cast<std::size_t>(config_.width) * config_.height *
                                 sizeof(float));
    }
    camera_info_ = node.create_publisher<sensor_msgs::msg::CameraInfo>(config_.camera_info_topic,
                                                                       qos_profiles::sensor_data());
  }

  void publish(const CameraFrame& frame) {
    if (rgb_ != nullptr && frame.has_rgb) {
      message_mapper::fill_rgb_image_message(config_, frame, &rgb_message_);
      rgb_->publish(rgb_message_);
    }
    if (depth_ != nullptr && frame.has_depth) {
      message_mapper::fill_depth_image_message(config_, frame, &depth_message_);
      depth_->publish(depth_message_);
    }
    camera_info_->publish(message_mapper::make_camera_info_message(config_, frame));
  }

 private:
  CameraPublisherConfig config_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_;
  sensor_msgs::msg::Image rgb_message_;
  sensor_msgs::msg::Image depth_message_;
};

class LidarPublisher {
 public:
  LidarPublisher(rclcpp::Node& node, LidarPublisherConfig config)
      : config_(std::move(config)),
        publisher_(node.create_publisher<sensor_msgs::msg::LaserScan>(
            config_.topic, qos_profiles::sensor_data())) {}

  void publish(const mujoco_simulation::LidarSample& sample, const rclcpp::Time& stamp) {
    publisher_->publish(message_mapper::make_lidar_message(config_, sample, stamp));
  }

 private:
  LidarPublisherConfig config_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
};

SimulationRosBridge::SimulationRosBridge(
    SimulationRosBridgeConfig config, rclcpp::Context::SharedPtr context,
    ServiceGateCallback service_gate_callback, StatusCallback reset_callback,
    StatusCallback start_callback, StatusCallback stop_callback, StatusCallback pause_callback,
    StatusCallback resume_callback, StepStatusCallback step_callback,
    RealtimeFactorStatusCallback realtime_factor_callback,
    KeyframeResetStatusCallback load_keyframe_callback)
    : config_(std::move(config)),
      node_(std::make_shared<rclcpp::Node>(
          config_.node_name,
          make_node_options(context != nullptr ? context
                                               : rclcpp::contexts::get_global_default_context()))),
      context_(context != nullptr ? std::move(context)
                                  : rclcpp::contexts::get_global_default_context()),
      service_gate_callback_(std::move(service_gate_callback)),
      publish_channel_(make_publish_channel_config()) {
  service_callback_group_ =
      node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  clock_ = node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", qos_profiles::clock());

  start_service_ = create_service<std_srvs::srv::Trigger>(
      *node_, "/start", service_callback_group_,
      [this, callback = std::move(start_callback)](
          const std::shared_ptr<std_srvs::srv::Trigger::Request>,
          std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        fill_status_response(invoke_service_callback(callback), "Simulation started.", *response);
      });
  stop_service_ = create_service<std_srvs::srv::Trigger>(
      *node_, "/stop", service_callback_group_,
      [this, callback = std::move(stop_callback)](
          const std::shared_ptr<std_srvs::srv::Trigger::Request>,
          std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        fill_status_response(invoke_service_callback(callback), "Simulation stopped.", *response);
      });
  pause_service_ = create_service<std_srvs::srv::Trigger>(
      *node_, "/pause", service_callback_group_,
      [this, callback = std::move(pause_callback)](
          const std::shared_ptr<std_srvs::srv::Trigger::Request>,
          std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        fill_status_response(invoke_service_callback(callback), "Simulation paused.", *response);
      });
  resume_service_ = create_service<std_srvs::srv::Trigger>(
      *node_, "/resume", service_callback_group_,
      [this, callback = std::move(resume_callback)](
          const std::shared_ptr<std_srvs::srv::Trigger::Request>,
          std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        fill_status_response(invoke_service_callback(callback), "Simulation resumed.", *response);
      });
  step_service_ = create_service<robot_mujoco_msgs::srv::StepSimulation>(
      *node_, "/step", service_callback_group_,
      [this, callback = std::move(step_callback)](
          const std::shared_ptr<robot_mujoco_msgs::srv::StepSimulation::Request> request,
          std::shared_ptr<robot_mujoco_msgs::srv::StepSimulation::Response> response) {
        if (!is_service_allowed()) {
          fill_status_response(
              mujoco_simulation::Status::invalid_state("Simulation ROS bridge is not active."),
              "Simulation stepped.", *response);
          return;
        }
        if (!callback) {
          fill_status_response(
              mujoco_simulation::Status::invalid_state("Step callback is not configured."),
              "Simulation stepped.", *response);
          return;
        }
        fill_status_response(callback(request->steps), "Simulation stepped.", *response);
      });
  set_realtime_factor_service_ = create_service<robot_mujoco_msgs::srv::SetRealtimeFactor>(
      *node_, "/set_realtime_factor", service_callback_group_,
      [this, callback = std::move(realtime_factor_callback)](
          const std::shared_ptr<robot_mujoco_msgs::srv::SetRealtimeFactor::Request> request,
          std::shared_ptr<robot_mujoco_msgs::srv::SetRealtimeFactor::Response> response) {
        if (!is_service_allowed()) {
          fill_status_response(
              mujoco_simulation::Status::invalid_state("Simulation ROS bridge is not active."),
              "Realtime factor updated.", *response);
          return;
        }
        if (!callback) {
          fill_status_response(mujoco_simulation::Status::invalid_state(
                                   "Realtime factor callback is not configured."),
                               "Realtime factor updated.", *response);
          return;
        }
        fill_status_response(callback(request->realtime_factor), "Realtime factor updated.",
                             *response);
      });
  load_keyframe_service_ = create_service<robot_mujoco_msgs::srv::ResetWorld>(
      *node_, "/load_keyframe", service_callback_group_,
      [this, callback = std::move(load_keyframe_callback)](
          const std::shared_ptr<robot_mujoco_msgs::srv::ResetWorld::Request> request,
          std::shared_ptr<robot_mujoco_msgs::srv::ResetWorld::Response> response) {
        if (!is_service_allowed()) {
          fill_status_response(
              mujoco_simulation::Status::invalid_state("Simulation ROS bridge is not active."),
              "Keyframe reset completed.", *response);
          return;
        }
        if (!callback) {
          fill_status_response(mujoco_simulation::Status::invalid_state(
                                   "Keyframe reset callback is not configured."),
                               "Keyframe reset completed.", *response);
          return;
        }
        fill_status_response(callback(request->keyframe), "Keyframe reset completed.", *response);
      });
  reset_service_ = create_service<std_srvs::srv::Trigger>(
      *node_, "/reset", service_callback_group_,
      [this, callback = std::move(reset_callback)](
          const std::shared_ptr<std_srvs::srv::Trigger::Request>,
          std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        fill_status_response(invoke_service_callback(callback), "Reset requested.", *response);
      });

  imus_.reserve(config_.imus.size());
  for (const auto& imu : config_.imus) {
    imus_.push_back(std::make_unique<ImuPublisher>(*node_, imu));
  }
  cameras_.reserve(config_.cameras.size());
  for (const auto& camera : config_.cameras) {
    cameras_.push_back(std::make_unique<CameraPublisher>(*node_, camera));
  }
  lidars_.reserve(config_.lidars.size());
  for (const auto& lidar : config_.lidars) {
    lidars_.push_back(std::make_unique<LidarPublisher>(*node_, lidar));
  }
}

SimulationRosBridge::~SimulationRosBridge() {
  shutdown_.store(true);
  (void)stop();
}

mujoco_simulation::Status SimulationRosBridge::start() {
  if (shutdown_.load()) {
    return mujoco_simulation::Status::invalid_state("Simulation ROS bridge is shutdown.");
  }
  if (worker_running_.exchange(true)) {
    return mujoco_simulation::Status::Ok();
  }

  rclcpp::ExecutorOptions executor_options;
  executor_options.context = context_;
  executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>(executor_options, 2U);
  executor_->add_node(node_);

  executor_thread_ = std::thread([this]() { executor_->spin(); });
  publish_thread_ = std::thread([this]() { run_publish_worker(); });
  return mujoco_simulation::Status::Ok();
}

mujoco_simulation::Status SimulationRosBridge::stop() {
  if (!worker_running_.exchange(false)) {
    if (executor_ != nullptr && node_ != nullptr) {
      executor_->remove_node(node_);
    }
    executor_.reset();
    return mujoco_simulation::Status::Ok();
  }
  if (executor_ != nullptr) {
    executor_->cancel();
  }
  if (executor_thread_.joinable()) {
    executor_thread_.join();
  }
  if (publish_thread_.joinable()) {
    publish_thread_.join();
  }
  if (executor_ != nullptr && node_ != nullptr) {
    executor_->remove_node(node_);
  }
  executor_.reset();
  return mujoco_simulation::Status::Ok();
}

void SimulationRosBridge::update_sim_time(const rclcpp::Time& sim_time) {
  sim_time_ns_.store(sim_time.nanoseconds());
  publish_channel_.update_clock(sim_time);
}

mujoco_simulation::Status SimulationRosBridge::enqueue_publish_bundle(const PublishBundle& bundle) {
  if (!worker_running_.load()) {
    return mujoco_simulation::Status::invalid_state("Simulation ROS bridge is not active.");
  }
  sim_time_ns_.store(bundle.sim_time.nanoseconds());
  return publish_channel_.publish_bundle(bundle);
}

void SimulationRosBridge::publish_clock(const rclcpp::Time& sim_time) {
  clock_->publish(message_mapper::make_clock_message(sim_time));
}

void SimulationRosBridge::run_publish_worker() {
  while (worker_running_.load() || executor_ != nullptr) {
    bool did_work = false;

    SmallPublishSnapshot snapshot;
    if (publish_channel_.consume_latest_small_snapshot(&snapshot)) {
      publish_clock(snapshot.sim_time);
      for (const auto& imu : snapshot.imus) {
        if (imu.publisher_index < imus_.size()) {
          imus_[imu.publisher_index]->publish(imu.sample, snapshot.sim_time);
        }
      }
      for (const auto& lidar : snapshot.lidars) {
        if (lidar.publisher_index < lidars_.size()) {
          lidars_[lidar.publisher_index]->publish(lidar.sample, snapshot.sim_time);
        }
      }
      did_work = true;
    } else {
      rclcpp::Time sim_time;
      if (publish_channel_.consume_latest_clock(&sim_time)) {
        publish_clock(sim_time);
        did_work = true;
      }
    }

    CameraFrame frame;
    while (publish_channel_.pop_camera_frame(&frame)) {
      if (frame.publisher_index < cameras_.size()) {
        cameras_[frame.publisher_index]->publish(frame);
      }
      did_work = true;
    }

    if (!did_work) {
      if (!worker_running_.load()) {
        break;
      }
      std::this_thread::sleep_for(1ms);
    }
  }
}

bool SimulationRosBridge::is_service_allowed() const {
  return !shutdown_.load() && service_gate_callback_ && service_gate_callback_();
}

PublishChannelConfig SimulationRosBridge::make_publish_channel_config() const {
  PublishChannelConfig channel_config;
  channel_config.imu_count = config_.imus.size();
  channel_config.lidar_count = config_.lidars.size();
  channel_config.lidar_sample_counts.reserve(config_.lidars.size());
  for (const auto& lidar : config_.lidars) {
    channel_config.lidar_sample_counts.push_back(lidar.sample_count);
  }
  channel_config.camera_frames.reserve(config_.cameras.size());
  for (const auto& camera : config_.cameras) {
    channel_config.camera_frames.push_back({.width = camera.width,
                                            .height = camera.height,
                                            .enable_rgb = camera.enable_rgb,
                                            .enable_depth = camera.enable_depth});
  }
  return channel_config;
}

template <typename CallbackT>
mujoco_simulation::Status SimulationRosBridge::invoke_service_callback(
    const CallbackT& callback) const {
  if (!is_service_allowed()) {
    return mujoco_simulation::Status::invalid_state("Simulation ROS bridge is not active.");
  }
  if (!callback) {
    return mujoco_simulation::Status::invalid_state(
        "Requested service callback is not configured.");
  }
  return callback();
}

}  // namespace robot_mujoco_ros2
