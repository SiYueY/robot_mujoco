#include <gtest/gtest.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "mujoco_ros2_bridge_msgs/srv/reset_world.hpp"
#include "mujoco_ros2_bridge_msgs/srv/set_realtime_factor.hpp"
#include "mujoco_ros2_bridge_msgs/srv/step_simulation.hpp"
#include "mujoco_simulation_ros/simulation_ros_bridge.hpp"

namespace mujoco_simulation_ros {
namespace {

using namespace std::chrono_literals;

rclcpp::NodeOptions make_node_options() {
  rclcpp::NodeOptions options;
  options.use_global_arguments(false)
      .enable_rosout(false)
      .start_parameter_services(false)
      .start_parameter_event_publisher(false)
      .use_clock_thread(false);
  return options;
}

class SimulationRosBridgeTest : public ::testing::Test {
 protected:
  void SetUp() override {
    ros_home_path_ = std::filesystem::temp_directory_path() /
                     ("simulation_ros_bridge_test_" + std::to_string(::getpid()));
    std::filesystem::create_directories(ros_home_path_ / "log");
    ::setenv("ROS_HOME", ros_home_path_.c_str(), 1);
    ::setenv("ROS_LOG_DIR", (ros_home_path_ / "log").c_str(), 1);
    if (!rclcpp::ok()) {
      int argc = 1;
      const char* argv[] = {"test_simulation_ros_bridge", nullptr};
      rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
    }
  }

  void TearDown() override {
    bridge_.reset();
    if (executor_ != nullptr && subscriber_node_ != nullptr) {
      executor_->remove_node(subscriber_node_);
    }
    executor_.reset();
    subscriber_node_.reset();
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    if (!ros_home_path_.empty()) {
      std::error_code error;
      std::filesystem::remove_all(ros_home_path_, error);
    }
  }

  bool spin_until(const std::function<bool()>& predicate, std::chrono::milliseconds timeout = 2s) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      if (executor_ != nullptr) {
        executor_->spin_some();
      }
      if (predicate()) {
        return true;
      }
      std::this_thread::sleep_for(10ms);
    }
    return predicate();
  }

  void create_subscriber_node(const std::string& node_name) {
    subscriber_node_ = std::make_shared<rclcpp::Node>(node_name, make_node_options());
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(subscriber_node_);
  }

  std::unique_ptr<SimulationRosBridge> bridge_;
  rclcpp::Node::SharedPtr subscriber_node_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::filesystem::path ros_home_path_;
};

TEST_F(SimulationRosBridgeTest, PublishesClockFromSimulationTime) {
  bridge_ = std::make_unique<SimulationRosBridge>(
      SimulationRosBridgeConfig{.node_name = "simulation_ros_bridge_clock_test"});
  create_subscriber_node("simulation_ros_bridge_clock_listener");

  std::mutex mutex;
  bool received = false;
  builtin_interfaces::msg::Time last_clock;
  auto subscription = subscriber_node_->create_subscription<rosgraph_msgs::msg::Clock>(
      "/clock", rclcpp::ClockQoS(),
      [&mutex, &received, &last_clock](const rosgraph_msgs::msg::Clock::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex);
        received = true;
        last_clock = message->clock;
      });

  (void)subscription;
  bridge_->set_time(rclcpp::Time(123456789LL, RCL_ROS_TIME));

  ASSERT_TRUE(spin_until([&mutex, &received]() {
    std::lock_guard<std::mutex> lock(mutex);
    return received;
  }));

  std::lock_guard<std::mutex> lock(mutex);
  EXPECT_EQ(last_clock.sec, 0);
  EXPECT_EQ(last_clock.nanosec, 123456789U);
}

TEST_F(SimulationRosBridgeTest, ResetServiceInvokesCallbackAndReturnsSuccess) {
  std::atomic<int> reset_calls{0};
  bridge_ = std::make_unique<SimulationRosBridge>(
      SimulationRosBridgeConfig{.node_name = "simulation_ros_bridge_reset_test"},
      rclcpp::contexts::get_global_default_context(), [&reset_calls]() {
        ++reset_calls;
        return mujoco_simulation::Status::Ok();
      });

  create_subscriber_node("simulation_ros_bridge_reset_client");

  auto client = subscriber_node_->create_client<std_srvs::srv::Trigger>("/reset");
  ASSERT_TRUE(spin_until([&client]() { return client->wait_for_service(0s); }));

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = client->async_send_request(request);
  ASSERT_TRUE(spin_until([&future]() { return future.wait_for(0s) == std::future_status::ready; }));

  const auto response = future.get();
  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->success);
  EXPECT_EQ(response->message, "Reset requested.");
  EXPECT_EQ(reset_calls.load(), 1);
}

TEST_F(SimulationRosBridgeTest, ResetServiceReturnsStatusFailureMessage) {
  bridge_ = std::make_unique<SimulationRosBridge>(
      SimulationRosBridgeConfig{.node_name = "simulation_ros_bridge_reset_failure_test"},
      rclcpp::contexts::get_global_default_context(),
      []() { return mujoco_simulation::Status::internal("reset failed"); });

  create_subscriber_node("simulation_ros_bridge_reset_failure_client");

  auto client = subscriber_node_->create_client<std_srvs::srv::Trigger>("/reset");
  ASSERT_TRUE(spin_until([&client]() { return client->wait_for_service(0s); }));

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = client->async_send_request(request);
  ASSERT_TRUE(spin_until([&future]() { return future.wait_for(0s) == std::future_status::ready; }));

  const auto response = future.get();
  ASSERT_NE(response, nullptr);
  EXPECT_FALSE(response->success);
  EXPECT_EQ(response->message, "reset failed");
}

TEST_F(SimulationRosBridgeTest, ControlServicesInvokeCallbacksAndReturnSuccess) {
  std::atomic<int> start_calls{0};
  std::atomic<int> stop_calls{0};
  std::atomic<int> pause_calls{0};
  std::atomic<int> resume_calls{0};
  std::atomic<int> step_calls{0};
  std::atomic<int> last_step_count{0};
  std::atomic<double> last_realtime_factor{0.0};
  std::string last_keyframe;
  bridge_ = std::make_unique<SimulationRosBridge>(
      SimulationRosBridgeConfig{.node_name = "simulation_ros_bridge_control_service_test"},
      rclcpp::contexts::get_global_default_context(), SimulationRosBridge::StatusCallback{},
      [&start_calls]() {
        ++start_calls;
        return mujoco_simulation::Status::Ok();
      },
      [&stop_calls]() {
        ++stop_calls;
        return mujoco_simulation::Status::Ok();
      },
      [&pause_calls]() {
        ++pause_calls;
        return mujoco_simulation::Status::Ok();
      },
      [&resume_calls]() {
        ++resume_calls;
        return mujoco_simulation::Status::Ok();
      },
      [&step_calls, &last_step_count](uint32_t steps) {
        ++step_calls;
        last_step_count.store(static_cast<int>(steps));
        return mujoco_simulation::Status::Ok();
      },
      [&last_realtime_factor](double realtime_factor) {
        last_realtime_factor.store(realtime_factor);
        return mujoco_simulation::Status::Ok();
      },
      [&last_keyframe](const std::string& keyframe) {
        last_keyframe = keyframe;
        return mujoco_simulation::Status::Ok();
      });

  create_subscriber_node("simulation_ros_bridge_control_service_client");

  auto start_client = subscriber_node_->create_client<std_srvs::srv::Trigger>("/start");
  auto stop_client = subscriber_node_->create_client<std_srvs::srv::Trigger>("/stop");
  auto pause_client = subscriber_node_->create_client<std_srvs::srv::Trigger>("/pause");
  auto resume_client = subscriber_node_->create_client<std_srvs::srv::Trigger>("/resume");
  auto step_client =
      subscriber_node_->create_client<mujoco_ros2_bridge_msgs::srv::StepSimulation>("/step");
  auto set_realtime_factor_client =
      subscriber_node_->create_client<mujoco_ros2_bridge_msgs::srv::SetRealtimeFactor>(
          "/set_realtime_factor");
  auto load_keyframe_client =
      subscriber_node_->create_client<mujoco_ros2_bridge_msgs::srv::ResetWorld>("/load_keyframe");

  ASSERT_TRUE(spin_until([&start_client]() { return start_client->wait_for_service(0s); }));
  ASSERT_TRUE(spin_until([&stop_client]() { return stop_client->wait_for_service(0s); }));
  ASSERT_TRUE(spin_until([&pause_client]() { return pause_client->wait_for_service(0s); }));
  ASSERT_TRUE(spin_until([&resume_client]() { return resume_client->wait_for_service(0s); }));
  ASSERT_TRUE(spin_until([&step_client]() { return step_client->wait_for_service(0s); }));
  ASSERT_TRUE(spin_until([&set_realtime_factor_client]() {
    return set_realtime_factor_client->wait_for_service(0s);
  }));
  ASSERT_TRUE(
      spin_until([&load_keyframe_client]() { return load_keyframe_client->wait_for_service(0s); }));

  auto trigger_request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto step_request = std::make_shared<mujoco_ros2_bridge_msgs::srv::StepSimulation::Request>();
  step_request->steps = 3;
  auto realtime_factor_request =
      std::make_shared<mujoco_ros2_bridge_msgs::srv::SetRealtimeFactor::Request>();
  realtime_factor_request->realtime_factor = 2.5;
  auto load_keyframe_request =
      std::make_shared<mujoco_ros2_bridge_msgs::srv::ResetWorld::Request>();
  load_keyframe_request->keyframe = "home";

  auto start_future = start_client->async_send_request(trigger_request);
  auto stop_future = stop_client->async_send_request(trigger_request);
  auto pause_future = pause_client->async_send_request(trigger_request);
  auto resume_future = resume_client->async_send_request(trigger_request);
  auto step_future = step_client->async_send_request(step_request);
  auto set_realtime_factor_future =
      set_realtime_factor_client->async_send_request(realtime_factor_request);
  auto load_keyframe_future = load_keyframe_client->async_send_request(load_keyframe_request);

  ASSERT_TRUE(spin_until([&start_future, &stop_future, &pause_future, &resume_future, &step_future,
                          &set_realtime_factor_future, &load_keyframe_future]() {
    return start_future.wait_for(0s) == std::future_status::ready &&
           stop_future.wait_for(0s) == std::future_status::ready &&
           pause_future.wait_for(0s) == std::future_status::ready &&
           resume_future.wait_for(0s) == std::future_status::ready &&
           step_future.wait_for(0s) == std::future_status::ready &&
           set_realtime_factor_future.wait_for(0s) == std::future_status::ready &&
           load_keyframe_future.wait_for(0s) == std::future_status::ready;
  }));

  EXPECT_TRUE(start_future.get()->success);
  EXPECT_TRUE(stop_future.get()->success);
  EXPECT_TRUE(pause_future.get()->success);
  EXPECT_TRUE(resume_future.get()->success);
  EXPECT_TRUE(step_future.get()->success);
  EXPECT_TRUE(set_realtime_factor_future.get()->success);
  EXPECT_TRUE(load_keyframe_future.get()->success);
  EXPECT_EQ(start_calls.load(), 1);
  EXPECT_EQ(stop_calls.load(), 1);
  EXPECT_EQ(pause_calls.load(), 1);
  EXPECT_EQ(resume_calls.load(), 1);
  EXPECT_EQ(step_calls.load(), 1);
  EXPECT_EQ(last_step_count.load(), 3);
  EXPECT_DOUBLE_EQ(last_realtime_factor.load(), 2.5);
  EXPECT_EQ(last_keyframe, "home");
}

TEST_F(SimulationRosBridgeTest, PublishesSensorSamplesUsingSampleTimestamps) {
  SimulationRosBridgeConfig config;
  config.node_name = "simulation_ros_bridge_publish_test";
  config.imus.push_back({.name = "imu", .frame_id = "imu_link", .topic = "/test/imu"});
  config.cameras.push_back({.name = "camera",
                            .frame_id = "camera_link",
                            .rgb_topic = "/test/camera/rgb",
                            .depth_topic = "/test/camera/depth",
                            .camera_info_topic = "/test/camera/info",
                            .width = 2,
                            .height = 1,
                            .enable_rgb = true,
                            .enable_depth = true});
  config.lidars.push_back({.name = "lidar", .frame_id = "lidar_link", .topic = "/test/lidar"});
  bridge_ = std::make_unique<SimulationRosBridge>(std::move(config));
  create_subscriber_node("simulation_ros_bridge_publish_listener");

  std::mutex mutex;
  bool imu_received = false;
  bool rgb_received = false;
  bool depth_received = false;
  bool info_received = false;
  bool lidar_received = false;
  sensor_msgs::msg::Imu imu_message;
  sensor_msgs::msg::Image rgb_message;
  sensor_msgs::msg::Image depth_message;
  sensor_msgs::msg::CameraInfo info_message;
  sensor_msgs::msg::LaserScan lidar_message;

  auto imu_sub = subscriber_node_->create_subscription<sensor_msgs::msg::Imu>(
      "/test/imu", rclcpp::SensorDataQoS(),
      [&mutex, &imu_received, &imu_message](const sensor_msgs::msg::Imu::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex);
        imu_received = true;
        imu_message = *message;
      });
  auto rgb_sub = subscriber_node_->create_subscription<sensor_msgs::msg::Image>(
      "/test/camera/rgb", rclcpp::SensorDataQoS(),
      [&mutex, &rgb_received, &rgb_message](const sensor_msgs::msg::Image::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex);
        rgb_received = true;
        rgb_message = *message;
      });
  auto depth_sub = subscriber_node_->create_subscription<sensor_msgs::msg::Image>(
      "/test/camera/depth", rclcpp::SensorDataQoS(),
      [&mutex, &depth_received, &depth_message](const sensor_msgs::msg::Image::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex);
        depth_received = true;
        depth_message = *message;
      });
  auto info_sub = subscriber_node_->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/test/camera/info", rclcpp::SensorDataQoS(),
      [&mutex, &info_received,
       &info_message](const sensor_msgs::msg::CameraInfo::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex);
        info_received = true;
        info_message = *message;
      });
  auto lidar_sub = subscriber_node_->create_subscription<sensor_msgs::msg::LaserScan>(
      "/test/lidar", rclcpp::SensorDataQoS(),
      [&mutex, &lidar_received,
       &lidar_message](const sensor_msgs::msg::LaserScan::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex);
        lidar_received = true;
        lidar_message = *message;
      });

  (void)imu_sub;
  (void)rgb_sub;
  (void)depth_sub;
  (void)info_sub;
  (void)lidar_sub;

  bridge_->set_time(rclcpp::Time(42LL, RCL_ROS_TIME));

  mujoco_simulation::ImuSample imu_sample;
  imu_sample.timestamp_ns = 100000001ULL;
  imu_sample.orientation = {0.0, 0.0, 0.0, 1.0};
  imu_sample.angular_velocity = {1.0, 2.0, 3.0};
  imu_sample.linear_acceleration = {4.0, 5.0, 6.0};

  mujoco_simulation::CameraSample camera_sample;
  camera_sample.timestamp_ns = 100000001ULL;
  camera_sample.optical_frame_id = "camera_optical";
  camera_sample.intrinsics.k = {100.0, 0.0, 1.0, 0.0, 100.0, 0.5, 0.0, 0.0, 1.0};
  camera_sample.intrinsics.p = {100.0, 0.0, 1.0, 0.0, 0.0, 100.0, 0.5, 0.0, 0.0, 0.0, 1.0, 0.0};
  camera_sample.color = mujoco_simulation::ImageBuffer{
      2, 1, 6, mujoco_simulation::PixelFormat::Rgb8, {1, 2, 3, 4, 5, 6}};
  camera_sample.depth = mujoco_simulation::DepthBuffer{
      2, 1, mujoco_simulation::DepthFormat::Float32Meters, {1.0F, 2.0F}};

  mujoco_simulation::LidarSample lidar_sample;
  lidar_sample.timestamp_ns = 100000001ULL;
  lidar_sample.angle_min = -1.0;
  lidar_sample.angle_max = 1.0;
  lidar_sample.angle_increment = 1.0;
  lidar_sample.scan_time = 0.1;
  lidar_sample.time_increment = 0.05;
  lidar_sample.range_min = 0.2;
  lidar_sample.range_max = 10.0;
  lidar_sample.ranges = {1.0, 2.0, 3.0};

  ASSERT_TRUE(bridge_->publish_imu("imu", imu_sample).ok());
  ASSERT_TRUE(bridge_->publish_camera("camera", camera_sample).ok());
  ASSERT_TRUE(bridge_->publish_lidar("lidar", lidar_sample).ok());

  ASSERT_TRUE(spin_until(
      [&mutex, &imu_received, &rgb_received, &depth_received, &info_received, &lidar_received]() {
        std::lock_guard<std::mutex> lock(mutex);
        return imu_received && rgb_received && depth_received && info_received && lidar_received;
      }));

  std::lock_guard<std::mutex> lock(mutex);
  EXPECT_EQ(imu_message.header.stamp.nanosec, 100000001U);
  EXPECT_EQ(rgb_message.header.stamp.nanosec, 100000001U);
  EXPECT_EQ(depth_message.header.stamp.nanosec, 100000001U);
  EXPECT_EQ(info_message.header.stamp.nanosec, 100000001U);
  EXPECT_EQ(lidar_message.header.stamp.nanosec, 100000001U);
  EXPECT_EQ(rgb_message.header.frame_id, "camera_optical");
  EXPECT_EQ(depth_message.header.frame_id, "camera_optical");
  EXPECT_EQ(info_message.header.frame_id, "camera_link");
  EXPECT_EQ(info_message.k[0], 100.0);
  EXPECT_EQ(info_message.p[0], 100.0);
}

TEST_F(SimulationRosBridgeTest, PublishesSensorSamplesUsingSimulationTimeFallback) {
  SimulationRosBridgeConfig config;
  config.node_name = "simulation_ros_bridge_fallback_test";
  config.imus.push_back({.name = "imu", .frame_id = "imu_link", .topic = "/fallback/imu"});
  bridge_ = std::make_unique<SimulationRosBridge>(std::move(config));
  create_subscriber_node("simulation_ros_bridge_fallback_listener");

  std::mutex mutex;
  bool received = false;
  sensor_msgs::msg::Imu imu_message;
  auto imu_sub = subscriber_node_->create_subscription<sensor_msgs::msg::Imu>(
      "/fallback/imu", rclcpp::SensorDataQoS(),
      [&mutex, &received, &imu_message](const sensor_msgs::msg::Imu::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex);
        received = true;
        imu_message = *message;
      });
  (void)imu_sub;

  bridge_->set_time(rclcpp::Time(987654321LL, RCL_ROS_TIME));
  mujoco_simulation::ImuSample imu_sample;
  imu_sample.timestamp_ns = 0;
  imu_sample.orientation = {0.0, 0.0, 0.0, 1.0};

  ASSERT_TRUE(bridge_->publish_imu("imu", imu_sample).ok());
  ASSERT_TRUE(spin_until([&mutex, &received]() {
    std::lock_guard<std::mutex> lock(mutex);
    return received;
  }));

  std::lock_guard<std::mutex> lock(mutex);
  EXPECT_EQ(imu_message.header.stamp.sec, 0);
  EXPECT_EQ(imu_message.header.stamp.nanosec, 987654321U);
}

TEST_F(SimulationRosBridgeTest, ResetServiceDoesNotPreventClockPublication) {
  std::atomic<bool> reset_started{false};
  std::promise<void> release_reset;
  std::shared_future<void> release_future = release_reset.get_future().share();
  bridge_ = std::make_unique<SimulationRosBridge>(
      SimulationRosBridgeConfig{.node_name = "simulation_ros_bridge_reset_clock_test"},
      rclcpp::contexts::get_global_default_context(), [&reset_started, release_future]() {
        reset_started.store(true);
        release_future.wait();
        return mujoco_simulation::Status::Ok();
      });
  create_subscriber_node("simulation_ros_bridge_reset_clock_listener");

  std::mutex mutex;
  bool clock_received = false;
  builtin_interfaces::msg::Time last_clock;
  auto clock_sub = subscriber_node_->create_subscription<rosgraph_msgs::msg::Clock>(
      "/clock", rclcpp::ClockQoS(),
      [&mutex, &clock_received, &last_clock](const rosgraph_msgs::msg::Clock::SharedPtr message) {
        std::lock_guard<std::mutex> lock(mutex);
        clock_received = true;
        last_clock = message->clock;
      });
  auto client = subscriber_node_->create_client<std_srvs::srv::Trigger>("/reset");
  (void)clock_sub;

  ASSERT_TRUE(spin_until([&client]() { return client->wait_for_service(0s); }));
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = client->async_send_request(request);

  ASSERT_TRUE(spin_until([&reset_started]() { return reset_started.load(); }));

  bridge_->set_time(rclcpp::Time(222222222LL, RCL_ROS_TIME));
  ASSERT_TRUE(spin_until([&mutex, &clock_received]() {
    std::lock_guard<std::mutex> lock(mutex);
    return clock_received;
  }));

  {
    std::lock_guard<std::mutex> lock(mutex);
    EXPECT_EQ(last_clock.sec, 0);
    EXPECT_EQ(last_clock.nanosec, 222222222U);
  }

  release_reset.set_value();
  ASSERT_TRUE(spin_until([&future]() { return future.wait_for(0s) == std::future_status::ready; }));

  const auto response = future.get();
  ASSERT_NE(response, nullptr);
  EXPECT_TRUE(response->success);
}

}  // namespace
}  // namespace mujoco_simulation_ros
