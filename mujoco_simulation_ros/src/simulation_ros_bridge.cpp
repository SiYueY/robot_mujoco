#include "mujoco_simulation_ros/simulation_ros_bridge.hpp"

#include <cstring>
#include <memory>
#include <utility>

#include "rclcpp/qos.hpp"

namespace mujoco_simulation_ros {
namespace {

rclcpp::Time select_stamp(const rclcpp::Time& fallback, std::uint64_t timestamp_ns) {
  return timestamp_ns == 0 ? fallback
                           : rclcpp::Time(static_cast<int64_t>(timestamp_ns), RCL_ROS_TIME);
}

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

rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr create_trigger_service(
    rclcpp::Node& node, const std::string& service_name,
    SimulationRosBridge::StatusCallback callback, const std::string& success_message) {
  if (!callback) {
    return nullptr;
  }

  return node.create_service<std_srvs::srv::Trigger>(
      service_name, [callback = std::move(callback), success_message](
                        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        fill_status_response(callback(), success_message, *response);
      });
}

rclcpp::Service<mujoco_ros2_bridge_msgs::srv::StepSimulation>::SharedPtr create_step_service(
    rclcpp::Node& node, const std::string& service_name,
    SimulationRosBridge::StepStatusCallback callback, const std::string& success_message) {
  if (!callback) {
    return nullptr;
  }

  return node.create_service<mujoco_ros2_bridge_msgs::srv::StepSimulation>(
      service_name,
      [callback = std::move(callback), success_message](
          const std::shared_ptr<mujoco_ros2_bridge_msgs::srv::StepSimulation::Request> request,
          std::shared_ptr<mujoco_ros2_bridge_msgs::srv::StepSimulation::Response> response) {
        fill_status_response(callback(request->steps), success_message, *response);
      });
}

rclcpp::Service<mujoco_ros2_bridge_msgs::srv::SetRealtimeFactor>::SharedPtr
create_set_realtime_factor_service(rclcpp::Node& node, const std::string& service_name,
                                   SimulationRosBridge::RealtimeFactorStatusCallback callback,
                                   const std::string& success_message) {
  if (!callback) {
    return nullptr;
  }

  return node.create_service<mujoco_ros2_bridge_msgs::srv::SetRealtimeFactor>(
      service_name,
      [callback = std::move(callback), success_message](
          const std::shared_ptr<mujoco_ros2_bridge_msgs::srv::SetRealtimeFactor::Request> request,
          std::shared_ptr<mujoco_ros2_bridge_msgs::srv::SetRealtimeFactor::Response> response) {
        fill_status_response(callback(request->realtime_factor), success_message, *response);
      });
}

rclcpp::Service<mujoco_ros2_bridge_msgs::srv::ResetWorld>::SharedPtr create_reset_world_service(
    rclcpp::Node& node, const std::string& service_name,
    SimulationRosBridge::KeyframeResetStatusCallback callback, const std::string& success_message) {
  if (!callback) {
    return nullptr;
  }

  return node.create_service<mujoco_ros2_bridge_msgs::srv::ResetWorld>(
      service_name,
      [callback = std::move(callback), success_message](
          const std::shared_ptr<mujoco_ros2_bridge_msgs::srv::ResetWorld::Request> request,
          std::shared_ptr<mujoco_ros2_bridge_msgs::srv::ResetWorld::Response> response) {
        fill_status_response(callback(request->keyframe), success_message, *response);
      });
}

sensor_msgs::msg::CameraInfo camera_info_from_sample(const CameraPublisherConfig& config,
                                                     const mujoco_simulation::CameraSample& sample,
                                                     const rclcpp::Time& stamp) {
  sensor_msgs::msg::CameraInfo info;
  info.header.stamp = stamp;
  info.header.frame_id = config.frame_id;
  info.width = sample.color.has_value()
                   ? sample.color->width
                   : (sample.depth.has_value() ? sample.depth->width : config.width);
  info.height = sample.color.has_value()
                    ? sample.color->height
                    : (sample.depth.has_value() ? sample.depth->height : config.height);
  info.distortion_model = "plumb_bob";
  info.d.assign(5, 0.0);
  info.k = sample.intrinsics.k;
  info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  info.p = sample.intrinsics.p;
  return info;
}

}  // namespace

class ImuPublisher {
 public:
  ImuPublisher(rclcpp::Node& node, ImuPublisherConfig config)
      : config_(std::move(config)),
        publisher_(
            node.create_publisher<sensor_msgs::msg::Imu>(config_.topic, rclcpp::SensorDataQoS())) {}

  void publish(const mujoco_simulation::ImuSample& sample, const rclcpp::Time& stamp) {
    sensor_msgs::msg::Imu message;
    message.header.stamp = select_stamp(stamp, sample.timestamp_ns);
    message.header.frame_id = config_.frame_id;
    message.orientation.x = sample.orientation[0];
    message.orientation.y = sample.orientation[1];
    message.orientation.z = sample.orientation[2];
    message.orientation.w = sample.orientation[3];
    message.orientation_covariance = sample.orientation_covariance;
    message.angular_velocity.x = sample.angular_velocity[0];
    message.angular_velocity.y = sample.angular_velocity[1];
    message.angular_velocity.z = sample.angular_velocity[2];
    message.angular_velocity_covariance = sample.angular_velocity_covariance;
    message.linear_acceleration.x = sample.linear_acceleration[0];
    message.linear_acceleration.y = sample.linear_acceleration[1];
    message.linear_acceleration.z = sample.linear_acceleration[2];
    message.linear_acceleration_covariance = sample.linear_acceleration_covariance;
    publisher_->publish(message);
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
                                                            rclcpp::SensorDataQoS());
    }
    if (config_.enable_depth) {
      depth_ = node.create_publisher<sensor_msgs::msg::Image>(config_.depth_topic,
                                                              rclcpp::SensorDataQoS());
    }
    camera_info_ = node.create_publisher<sensor_msgs::msg::CameraInfo>(config_.camera_info_topic,
                                                                       rclcpp::SensorDataQoS());
  }

  void publish(const mujoco_simulation::CameraSample& sample, const rclcpp::Time& stamp) {
    const rclcpp::Time sample_stamp = select_stamp(stamp, sample.timestamp_ns);

    if (rgb_ != nullptr && sample.color.has_value()) {
      sensor_msgs::msg::Image message;
      message.header.stamp = sample_stamp;
      message.header.frame_id =
          sample.optical_frame_id.empty() ? config_.frame_id : sample.optical_frame_id;
      message.height = sample.color->height;
      message.width = sample.color->width;
      message.encoding = "rgb8";
      message.is_bigendian = false;
      message.step = sample.color->step;
      message.data = sample.color->data;
      rgb_->publish(message);
    }

    if (depth_ != nullptr && sample.depth.has_value()) {
      sensor_msgs::msg::Image message;
      message.header.stamp = sample_stamp;
      message.header.frame_id =
          sample.optical_frame_id.empty() ? config_.frame_id : sample.optical_frame_id;
      message.height = sample.depth->height;
      message.width = sample.depth->width;
      message.encoding = "32FC1";
      message.is_bigendian = false;
      message.step = static_cast<uint32_t>(sample.depth->width * sizeof(float));
      message.data.resize(sample.depth->data.size() * sizeof(float));
      std::memcpy(message.data.data(), sample.depth->data.data(), message.data.size());
      depth_->publish(message);
    }

    camera_info_->publish(camera_info_from_sample(config_, sample, sample_stamp));
  }

 private:
  CameraPublisherConfig config_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_;
};

class LidarPublisher {
 public:
  LidarPublisher(rclcpp::Node& node, LidarPublisherConfig config)
      : config_(std::move(config)),
        publisher_(node.create_publisher<sensor_msgs::msg::LaserScan>(config_.topic,
                                                                      rclcpp::SensorDataQoS())) {}

  void publish(const mujoco_simulation::LidarSample& sample, const rclcpp::Time& stamp) {
    sensor_msgs::msg::LaserScan message;
    message.header.stamp = select_stamp(stamp, sample.timestamp_ns);
    message.header.frame_id = config_.frame_id;
    message.angle_min = static_cast<float>(sample.angle_min);
    message.angle_max = static_cast<float>(sample.angle_max);
    message.angle_increment = static_cast<float>(sample.angle_increment);
    message.scan_time = static_cast<float>(sample.scan_time);
    message.time_increment = static_cast<float>(sample.time_increment);
    message.range_min = static_cast<float>(sample.range_min);
    message.range_max = static_cast<float>(sample.range_max);
    message.ranges.assign(sample.ranges.begin(), sample.ranges.end());
    if (sample.intensities.empty()) {
      message.intensities.assign(message.ranges.size(), 0.0F);
    } else {
      message.intensities.assign(sample.intensities.begin(), sample.intensities.end());
    }
    publisher_->publish(message);
  }

 private:
  LidarPublisherConfig config_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
};

SimulationRosBridge::SimulationRosBridge(
    SimulationRosBridgeConfig config, rclcpp::Context::SharedPtr context,
    StatusCallback reset_callback, StatusCallback start_callback, StatusCallback stop_callback,
    StatusCallback pause_callback, StatusCallback resume_callback, StepStatusCallback step_callback,
    RealtimeFactorStatusCallback realtime_factor_callback,
    KeyframeResetStatusCallback load_keyframe_callback)
    : config_(std::move(config)),
      context_(context != nullptr ? std::move(context)
                                  : rclcpp::contexts::get_global_default_context()) {
  node_ = std::make_shared<rclcpp::Node>(config_.node_name, make_node_options(context_));

  rclcpp::ExecutorOptions executor_options;
  executor_options.context = context_;
  executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>(executor_options);

  clock_ = node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
  start_service_ =
      create_trigger_service(*node_, "/start", std::move(start_callback), "Simulation started.");
  stop_service_ =
      create_trigger_service(*node_, "/stop", std::move(stop_callback), "Simulation stopped.");
  pause_service_ =
      create_trigger_service(*node_, "/pause", std::move(pause_callback), "Simulation paused.");
  resume_service_ =
      create_trigger_service(*node_, "/resume", std::move(resume_callback), "Simulation resumed.");
  step_service_ =
      create_step_service(*node_, "/step", std::move(step_callback), "Simulation stepped.");
  set_realtime_factor_service_ = create_set_realtime_factor_service(
      *node_, "/set_realtime_factor", std::move(realtime_factor_callback),
      "Realtime factor updated.");
  load_keyframe_service_ = create_reset_world_service(
      *node_, "/load_keyframe", std::move(load_keyframe_callback), "Keyframe reset completed.");
  reset_service_ =
      create_trigger_service(*node_, "/reset", std::move(reset_callback), "Reset requested.");

  for (const auto& imu : config_.imus) {
    imus_.emplace(imu.name, std::make_unique<ImuPublisher>(*node_, imu));
  }
  for (const auto& camera : config_.cameras) {
    cameras_.emplace(camera.name, std::make_unique<CameraPublisher>(*node_, camera));
  }
  for (const auto& lidar : config_.lidars) {
    lidars_.emplace(lidar.name, std::make_unique<LidarPublisher>(*node_, lidar));
  }

  executor_->add_node(node_);
  executor_thread_ = std::thread([this]() { executor_->spin(); });
}

SimulationRosBridge::~SimulationRosBridge() {
  if (executor_ != nullptr) {
    executor_->cancel();
  }
  if (executor_thread_.joinable()) {
    executor_thread_.join();
  }
  if (executor_ != nullptr && node_ != nullptr) {
    executor_->remove_node(node_);
  }
  node_.reset();
}

void SimulationRosBridge::set_time(const rclcpp::Time& sim_time) {
  sim_time_ = sim_time;
  if (clock_ != nullptr) {
    rosgraph_msgs::msg::Clock message;
    message.clock = sim_time_;
    clock_->publish(message);
  }
}

mujoco_simulation::Status SimulationRosBridge::publish_imu(
    std::string_view name, const mujoco_simulation::ImuSample& sample) {
  const auto it = imus_.find(std::string(name));
  if (it == imus_.end()) {
    return mujoco_simulation::Status::not_found("SimulationRosBridge IMU publisher not found: " +
                                                std::string(name));
  }
  it->second->publish(sample, sim_time_);
  return mujoco_simulation::Status::Ok();
}

mujoco_simulation::Status SimulationRosBridge::publish_camera(
    std::string_view name, const mujoco_simulation::CameraSample& sample) {
  const auto it = cameras_.find(std::string(name));
  if (it == cameras_.end()) {
    return mujoco_simulation::Status::not_found("SimulationRosBridge camera publisher not found: " +
                                                std::string(name));
  }
  it->second->publish(sample, sim_time_);
  return mujoco_simulation::Status::Ok();
}

mujoco_simulation::Status SimulationRosBridge::publish_lidar(
    std::string_view name, const mujoco_simulation::LidarSample& sample) {
  const auto it = lidars_.find(std::string(name));
  if (it == lidars_.end()) {
    return mujoco_simulation::Status::not_found("SimulationRosBridge lidar publisher not found: " +
                                                std::string(name));
  }
  it->second->publish(sample, sim_time_);
  return mujoco_simulation::Status::Ok();
}

}  // namespace mujoco_simulation_ros
